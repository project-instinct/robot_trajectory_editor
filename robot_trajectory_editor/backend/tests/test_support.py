import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('osqp')
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from support import SupportForces, friction_rays


def point_map(point, joints=0):
    x, y, z = point
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.vstack([np.eye(3), cross, np.zeros((joints, 3))])


def test_standing_weight_has_unilateral_support():
    solver = SupportForces([], .5)
    residual, forces = solver.solve(np.array([0., 0., 100., 0., 0., 0.]),
                                    [point_map([0, 0, 0])], [[0, 0, 1]])
    np.testing.assert_allclose(residual, 0, atol=1e-5)
    np.testing.assert_allclose(forces, [[0, 0, 100]], atol=1e-5)


def test_no_contact_preserves_free_flight_equations():
    solver = SupportForces([], .5)
    required = np.array([0., 0., 100., 0., 0., 0.])
    residual, forces = solver.solve(required, [], [])
    np.testing.assert_array_equal(residual, required)
    assert forces.shape == (0, 3)


@pytest.mark.parametrize('required', [[0, 0, -100, 0, 0, 0], [100, 0, 100, 0, 0, 0]])
def test_contact_cannot_pull_or_exceed_friction(required):
    solver = SupportForces([], .5)
    residual, forces = solver.solve(np.array(required, dtype=float),
                                    [point_map([0, 0, 0])], [[0, 0, 1]])
    assert np.linalg.norm(residual) > 1
    assert forces[0, 2] >= 0
    assert np.linalg.norm(forces[0, :2]) <= .5*forces[0, 2] + 1e-10


def test_multiple_contacts_balance_moment(monkeypatch):
    solver = SupportForces([], .5)
    repeated = []
    for moment in [-20., 20.]:
        residual, forces = solver.solve(np.array([0, 0, 100, 0, moment, 0]),
                                        [point_map([-1, 0, 0]), point_map([1, 0, 0])],
                                        [[0, 0, 1], [0, 0, 1]])
        np.testing.assert_allclose(residual, 0, atol=1e-4)
        assert np.all(forces[:, 2] >= 0)
        repeated.append((residual, forces, solver.coefficients.copy()))
    for moment, expected in zip([-20., 20.], repeated):
        actual = solver.solve(np.array([0, 0, 100, 0, moment, 0]),
                              [point_map([-1, 0, 0]), point_map([1, 0, 0])],
                              [[0, 0, 1], [0, 0, 1]])
        # A cold-started QP is independent of the preceding contact allocation.
        for result, baseline in zip((*actual, solver.coefficients), expected):
            np.testing.assert_allclose(result, baseline, atol=1e-10, rtol=0)
    assert len(solver.solvers) == 1
    monkeypatch.setattr('support.osqp.OSQP.solve',
                        lambda self, **kwargs: SimpleNamespace(
                            x=np.zeros(8), info=SimpleNamespace(
                                status_val=7, status='maximum iterations reached')))
    residual, forces = solver.solve(np.array([0, 0, 100, 0, 20, 0]),
                                    [point_map([-1, 0, 0]), point_map([1, 0, 0])],
                                    [[0, 0, 1], [0, 0, 1]])
    np.testing.assert_allclose(residual, repeated[1][0], atol=1e-4)
    np.testing.assert_allclose(forces, repeated[1][1], atol=1e-4)


def test_actuator_limit_restricts_support_distribution():
    solver = SupportForces([10.], .5)
    mapping = point_map([0, 0, 0], joints=1)
    mapping[6, 2] = 1  # Each newton of support needs one unit of joint torque.
    adjusted, _ = solver.solve(np.array([0, 0, 100, 0, 0, 0, 0]), [mapping], [[0, 0, 1]])
    assert abs(adjusted[2]) > 1 or abs(adjusted[6]) > 10 + 1e-4


def test_friction_pyramid_is_inside_cone_for_sloped_normal():
    normal = np.array([1., 2., 3.])
    normal /= np.linalg.norm(normal)
    rays = friction_rays(normal, .7)
    for weights in ([1, 0, 0, 0], [1, 2, 3, 4]):
        force = rays @ weights
        axial = force @ normal
        assert axial >= 0
        assert np.linalg.norm(force-axial*normal) <= .7*axial + 1e-12
