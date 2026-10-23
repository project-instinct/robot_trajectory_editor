"""Analytic dynamics fixtures need no user motion files."""
import copy
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from trajectory_optimize import (CollocationProblem, TrajectoryOptimizeError, hermite, read_trajectory,
                        rotation_kinematics, optimize_trajectory, RobotDynamics, TOLERANCES)


def motion(n=3):
    t = np.arange(n) / 10
    return {"framerate": 10., "joint_names": [], "joint_pos": [[] for _ in t],
            "base_pos_w": [[0., 0., 2. + .2*time - 4.905*time*time] for time in t],
            "base_quat_w": [[1., 0., 0., 0.] for _ in t]}


class FallingBody:
    support = None
    tolerances = TOLERANCES
    lower = upper = velocity = effort = np.array([])

    def forces(self, x, v, a):
        return np.r_[a[:3] + [0, 0, 9.81], a[3:6]]

    def clearances(self, x, derivatives=False, penetration_only=False):
        distances = np.array([x[2] - .1])
        if derivatives:
            gradient = np.zeros((1, len(x)))
            gradient[0, 2] = 1
            return distances, gradient
        return distances

    def validate_surface_intersections(self):
        pass


def test_fixed_configurations_are_not_variables_and_all_other_frames_are_free():
    reference = np.arange(30.).reshape(5, 6)
    problem = CollocationProblem(FallingBody(), reference, [1, 3], .1)
    assert problem.free.tolist() == [0, 2, 4]
    variables = problem.pack(reference, np.zeros_like(reference))
    assert len(variables) == (3 + 5) * 6
    variables += 1
    actual, _ = problem.unpack(variables)
    np.testing.assert_array_equal(actual[[1, 3]], reference[[1, 3]])
    np.testing.assert_array_equal(actual[[0, 2, 4]], reference[[0, 2, 4]] + 1)


def test_no_keyframes_leaves_entire_trajectory_eligible():
    problem = CollocationProblem(FallingBody(), np.zeros((4, 6)), [], .1)
    assert problem.free.tolist() == [0, 1, 2, 3]


def test_hermite_reproduces_ballistic_dynamics_between_frames():
    dt = .1
    q0 = np.array([0., 0., 2., 0., 0., 0.])
    v0 = np.array([0., 0., .2, 0., 0., 0.])
    a = np.array([0., 0., -9.81, 0., 0., 0.])
    q1, v1 = q0 + dt*v0 + .5*dt*dt*a, v0 + dt*a
    for fraction in np.linspace(0, 1, 17):
        q, v, acceleration = hermite(q0, q1, v0, v1, dt, fraction)
        np.testing.assert_allclose(q, q0 + fraction*dt*v0 + .5*(fraction*dt)**2*a, atol=1e-12)
        np.testing.assert_allclose(v, v0 + fraction*dt*a, atol=1e-12)
        np.testing.assert_allclose(acceleration, a, atol=1e-12)


def test_dense_validation_detects_interior_penetration_despite_clear_endpoints():
    reference = np.zeros((2, 6))
    reference[:, 2] = 1
    v = np.zeros_like(reference)
    v[:, 2] = [-10, 10]
    problem = CollocationProblem(FallingBody(), reference, [], 1.)
    with pytest.raises(TrajectoryOptimizeError, match="Validation failed"):
        problem.validate(problem.pack(reference, v))
    assert min(state[0][2] for _, state in problem.samples(problem.pack(reference, v), np.linspace(0, 1, 17))) < 0


def test_invalid_quaternion_and_keyframe_diagnostics():
    data = motion()
    with pytest.raises(TrajectoryOptimizeError, match="indices"):
        read_trajectory(data, [3])
    data["base_quat_w"][1] = [0, 0, 0, 0]
    with pytest.raises(TrajectoryOptimizeError, match="quaternion"):
        read_trajectory(data, [])


def test_rotation_kinematics_matches_finite_difference():
    Rotation = pytest.importorskip("scipy.spatial.transform").Rotation
    r, v, a = np.array([.3, -.7, .2]), np.array([.4, .3, -.2]), np.array([-.1, .2, .3])
    h = 1e-5
    omega, alpha = rotation_kinematics(r, v, a)
    rotation = Rotation.from_rotvec(r).as_matrix()
    plus = Rotation.from_rotvec(r + h*v + .5*h*h*a).as_matrix()
    minus = Rotation.from_rotvec(r - h*v + .5*h*h*a).as_matrix()
    skew = rotation.T @ ((plus-minus)/(2*h))
    np.testing.assert_allclose(omega, [skew[2, 1], skew[0, 2], skew[1, 0]], atol=1e-8)
    wp, _ = rotation_kinematics(r + h*v + .5*h*h*a, v+h*a, a)
    wm, _ = rotation_kinematics(r - h*v + .5*h*h*a, v-h*a, a)
    np.testing.assert_allclose(alpha, (wp-wm)/(2*h), atol=1e-8)


@pytest.fixture
def sphere_urdf(tmp_path):
    pytest.importorskip("pinocchio")
    pytest.importorskip("coal")
    pytest.importorskip("trimesh")
    pytest.importorskip("scipy")
    path = tmp_path / "sphere.urdf"
    path.write_text('''<robot name="sphere"><link name="base">
      <inertial><mass value="1"/><inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.004"/></inertial>
      <collision><geometry><sphere radius="0.1"/></geometry></collision>
    </link></robot>''')
    return str(path)


@pytest.mark.parametrize('fixed', [[0, 3], [0, 1, 2, 3]])
def test_real_model_free_fall_preserves_keyframes_and_metadata(sphere_urdf, fixed):
    data = motion(4)
    before = copy.deepcopy(data)
    result = optimize_trajectory(data, fixed, sphere_urdf)
    assert data == before
    output = result["trajectory"]
    assert output["framerate"] == data["framerate"]
    assert output["joint_names"] == []
    assert len(output["base_pos_w"]) == 4
    for frame in fixed:
        for field in ("base_pos_w", "base_quat_w", "joint_pos"):
            assert output[field][frame] == data[field][frame]
    for name, value in result["residuals"].items():
        assert value <= result["tolerances"][name]


def test_real_model_optimizes_interior_without_anchors(sphere_urdf):
    data = motion(3)
    data["base_pos_w"] = [[0, 0, 2] for _ in range(3)]
    output = optimize_trajectory(data, [], sphere_urdf)["trajectory"]
    assert output["base_pos_w"] != data["base_pos_w"]
    for frame in (0, 2):
        for field in ("joint_pos", "base_pos_w", "base_quat_w"):
            assert output[field][frame] == data[field][frame]


def test_real_model_rejects_fixed_ground_penetration(sphere_urdf):
    data = motion()
    data["base_pos_w"][1][2] = 0
    with pytest.raises(TrajectoryOptimizeError, match="Fixed keyframe 1 penetrates"):
        optimize_trajectory(data, [1], sphere_urdf)


def test_real_model_inverse_dynamics_and_ground(sphere_urdf):
    robot = RobotDynamics(sphere_urdf, [], None, lambda _: None)
    x = np.array([0, 0, 2., .3, -.4, .5])
    np.testing.assert_allclose(robot.clearances(x), [1.9], atol=1e-6)
    v = np.array([.2, .3, -.2, 0, 0, 0])
    a = np.array([0, 0, -9.81, 0, 0, 0])
    np.testing.assert_allclose(robot.forces(x, v, a), np.zeros(6), atol=1e-10)


def test_real_model_loaded_terrain(sphere_urdf):
    terrain = 'v -5 -5 1\nv 5 -5 1\nv 5 5 1\nv -5 5 1\nf 1 2 3\nf 1 3 4\n'
    robot = RobotDynamics(sphere_urdf, [], terrain, lambda _: None)
    np.testing.assert_allclose(robot.clearances(np.array([0, 0, 2., 0, 0, 0])), [.9], atol=1e-6)
    robot.validate_surface_intersections()
    robot.clearances(np.array([0, 0, 1., 0, 0, 0]))
    with pytest.raises(TrajectoryOptimizeError, match='surface intersection'):
        robot.validate_surface_intersections()


def test_missing_inertial_data_has_clear_diagnostic(sphere_urdf, tmp_path):
    path = tmp_path / 'missing.urdf'
    path.write_text('<robot name="missing"><link name="base"/></robot>')
    with pytest.raises(TrajectoryOptimizeError, match='Missing mass or inertia'):
        optimize_trajectory(motion(), [], str(path))


def test_dense_validation_checks_velocity_and_effort_limits():
    class LimitedJoint(FallingBody):
        lower, upper = np.array([-1.]), np.array([1.])
        velocity, effort = np.array([.1]), np.array([.2])

        def forces(self, x, v, a):
            return np.r_[super().forces(x, v, a), a[6]]

    data = motion(2)
    reference = np.c_[data['base_pos_w'], np.zeros((2, 3)), [0., .05]]
    velocity = np.zeros_like(reference)
    velocity[:, 2] = [.2, .2 - .981]
    velocity[:, 6] = .5
    problem = CollocationProblem(LimitedJoint(), reference, [], .1)
    with pytest.raises(TrajectoryOptimizeError, match='velocity residual'):
        problem.validate(problem.pack(reference, velocity))
    velocity[:, 6] = [0., 0.]
    problem.robot.velocity = np.array([10.])
    with pytest.raises(TrajectoryOptimizeError, match='effort residual'):
        problem.validate(problem.pack(reference, velocity))


def test_empty_fixed_sensor_frame_is_not_missing_body_data(sphere_urdf, tmp_path):
    from pathlib import Path
    path = tmp_path / 'with_sensor.urdf'
    path.write_text(Path(sphere_urdf).read_text().replace('</robot>', '''
      <link name="sensor"/>
      <joint name="sensor_frame" type="fixed"><parent link="base"/><child link="sensor"/></joint>
    </robot>'''))
    robot = RobotDynamics(str(path), [], None, lambda _: None)
    assert robot.model.nv == 6


def test_interval_sparsity_covers_observed_constraint_derivatives():
    pytest.importorskip('scipy')
    reference = np.zeros((4, 6))
    reference[:, 2] = [2., 2.1, 2.2, 2.3]
    problem = CollocationProblem(FallingBody(), reference, [1], .1)
    variables = problem.pack(reference, np.full_like(reference, 3.))
    variables[:problem.position_size] += .02
    for function in (problem.equalities, problem.inequalities):
        baseline = function(variables)
        sparsity = problem.constraint_sparsity(len(baseline) // 3).toarray()
        assert sparsity.shape == (len(baseline), len(variables))
        for column in range(len(variables)):
            changed = variables.copy()
            changed[column] += 1e-5
            active = np.abs(function(changed) - baseline) > 1e-10
            assert np.all(sparsity[active, column])
    gradient = problem.objective_gradient(variables)
    for column in range(len(variables)):
        plus, minus = variables.copy(), variables.copy()
        plus[column] += 1e-5
        minus[column] -= 1e-5
        actual = (problem.objective(plus) - problem.objective(minus)) / 2e-5
        assert gradient[column] == pytest.approx(actual, abs=1e-10)


def test_incompatible_fixed_poses_fail_before_solving(sphere_urdf):
    data = motion(4)
    data['base_pos_w'] = [[0, 0, 2] for _ in range(4)]
    with pytest.raises(TrajectoryOptimizeError, match='center-of-mass ballistic residual'):
        optimize_trajectory(data, [0, 1, 3], sphere_urdf)


def test_float32_high_rate_heavy_body_uses_reported_tolerance(sphere_urdf, tmp_path):
    from pathlib import Path
    path = tmp_path / 'heavy.urdf'
    path.write_text(Path(sphere_urdf).read_text().replace('mass value="1"', 'mass value="50"')
                    .replace('0.004', '0.2'))
    data = motion(3)
    data['framerate'] = 90.
    data['base_pos_w'] = np.array([[0, 0, 3.5 - 4.905*(i/90)**2] for i in range(3)], dtype=np.float32).tolist()
    result = optimize_trajectory(data, [], str(path))
    assert result['tolerances']['dynamics'] == pytest.approx(.4905)
    assert result['residuals']['dynamics'] <= result['tolerances']['dynamics']


def test_sparse_solver_preserves_fixed_poses(sphere_urdf, monkeypatch):
    monkeypatch.setattr('trajectory_optimize.DENSE_VARIABLE_LIMIT', 0)
    data = motion(4)
    result = optimize_trajectory(data, [0, 3], sphere_urdf)
    for frame in (0, 3):
        for field in ('base_pos_w', 'base_quat_w', 'joint_pos'):
            assert result['trajectory'][field][frame] == data[field][frame]
    for name, residual in result['residuals'].items():
        assert residual <= result['tolerances'][name]


def test_validation_rejects_unknown_collision_distance():
    class UnknownClearance(FallingBody):
        def clearances(self, x):
            return np.array([np.nan])

    reference = np.zeros((2, 6))
    reference[:, 2] = 2
    problem = CollocationProblem(UnknownClearance(), reference, [], .1)
    with pytest.raises(TrajectoryOptimizeError, match='Non-finite clearance'):
        problem.validate(problem.pack(reference, np.zeros_like(reference)))


@pytest.fixture
def articulated_urdf(sphere_urdf, tmp_path):
    from pathlib import Path
    path = tmp_path / 'articulated.urdf'
    path.write_text(Path(sphere_urdf).read_text().replace('</robot>', '''
      <link name="arm"><inertial><origin xyz="0.2 0 0"/><mass value="0.5"/>
        <inertia ixx="0.01" ixy="0.001" ixz="0" iyy="0.02" iyz="0" izz="0.025"/></inertial>
        <collision><origin xyz="0.4 0 0"/><geometry><sphere radius="0.12"/></geometry></collision></link>
      <joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>
        <origin xyz="0.1 0 0.2" rpy="0.2 0.3 -0.1"/><axis xyz="0 1 0"/>
        <limit lower="-2" upper="2" velocity="5" effort="20"/></joint>
    </robot>'''))
    return str(path)


def test_force_derivatives_match_finite_difference_with_rotating_articulated_base(articulated_urdf):
    robot = RobotDynamics(articulated_urdf, ['hinge'], None, lambda _: None)
    state = [np.array([.1, .2, 2., .3, -.4, .6, .5]),
             np.array([.4, -.3, .2, .6, .3, -.2, -.4]),
             np.array([-.1, .3, -.4, .2, -.5, .6, .7])]
    derivatives = robot.force_derivatives(*state)
    for field in range(3):
        for axis in range(7):
            plus, minus = [x.copy() for x in state], [x.copy() for x in state]
            plus[field][axis] += 1e-6
            minus[field][axis] -= 1e-6
            actual = (robot.forces(*plus) - robot.forces(*minus)) / 2e-6
            np.testing.assert_allclose(derivatives[field][:, axis], actual, atol=2e-7, rtol=2e-6)


def test_effort_scale_expands_urdf_limits(articulated_urdf):
    robot = RobotDynamics(articulated_urdf, ['hinge'], None, lambda _: None, effort_scale=2.)
    assert robot.effort == pytest.approx([40.])


@pytest.mark.parametrize('height', [2., -.1])
def test_collision_derivatives_match_finite_difference(articulated_urdf, height):
    robot = RobotDynamics(articulated_urdf, ['hinge'], None, lambda _: None)
    x = np.array([.1, .2, height, .3, -.4, .6, .5])
    _, derivative = robot.clearances(x, derivatives=True)
    for axis in range(7):
        plus, minus = x.copy(), x.copy()
        plus[axis] += 1e-6
        minus[axis] -= 1e-6
        actual = (robot.clearances(plus)-robot.clearances(minus))/2e-6
        np.testing.assert_allclose(derivative[:, axis], actual, atol=1e-7)


def test_sparse_chain_rule_matches_independent_finite_difference(articulated_urdf):
    from scipy.optimize._numdiff import approx_derivative
    robot = RobotDynamics(articulated_urdf, ['hinge'], None, lambda _: None)
    reference = np.tile([.1, .2, 2., .3, -.4, .6, .5], (3, 1))
    reference[1] += .01
    problem = CollocationProblem(robot, reference, [1], .1)
    variables = problem.pack(reference, np.full_like(reference, .2))
    for equality, function in ((True, problem.equalities), (False, problem.inequalities)):
        actual = problem.constraint_jacobian(variables, equality).toarray()
        expected = approx_derivative(function, variables, method='3-point')
        np.testing.assert_allclose(actual, expected, atol=2e-6, rtol=1e-5)


@pytest.mark.parametrize('height', [2., .3])
def test_watertight_terrain_distance_derivatives_and_containment(articulated_urdf, height):
    import trimesh
    terrain = trimesh.creation.box(extents=[10, 10, 2]).export(file_type='obj')
    robot = RobotDynamics(articulated_urdf, ['hinge'], terrain, lambda _: None)
    x = np.array([.1, .2, height, .3, -.4, .6, .5])
    distances, derivative = robot.clearances(x, derivatives=True)
    assert np.all(distances > 0) if height == 2 else np.all(distances < 0)
    for axis in range(7):
        plus, minus = x.copy(), x.copy()
        plus[axis] += 1e-6
        minus[axis] -= 1e-6
        actual = (robot.clearances(plus)-robot.clearances(minus))/2e-6
        np.testing.assert_allclose(derivative[:, axis], actual, atol=2e-6)


def test_collision_cache_reuses_candidate_and_invalidates_after_edit(articulated_urdf, monkeypatch):
    robot = RobotDynamics(articulated_urdf, ['hinge'], None, lambda _: None)
    reference = np.tile([.1, .2, 2., .3, -.4, .6, .5], (3, 1))
    problem = CollocationProblem(robot, reference, [], .1)
    variables = problem.pack(reference, np.full_like(reference, .2))
    calls = []
    original = robot.clearances

    def counted(x, derivatives=False, **options):
        calls.append(x.copy())
        return original(x, derivatives, **options)

    monkeypatch.setattr(robot, 'clearances', counted)
    problem.inequalities(variables)
    count = len(calls)
    problem.constraint_jacobian(variables, equality=False)
    problem.inequalities(variables.copy())
    assert len(calls) == count
    variables[2] += .01
    problem.inequalities(variables)
    assert len(calls) > count


def test_contact_model_supports_fixed_standing_pose(sphere_urdf):
    data = motion(4)
    data['base_pos_w'] = [[0., 0., .1] for _ in range(4)]
    result = optimize_trajectory(data, [0, 1, 2, 3], sphere_urdf, friction=.7)
    assert result['trajectory'] == data
    assert result['iterations'] == 0
    assert result['friction'] == .7
    assert result['residuals']['dynamics'] < result['tolerances']['dynamics']


def test_contact_forces_are_unavailable_above_terrain(sphere_urdf):
    robot = RobotDynamics(sphere_urdf, [], None, lambda _: None, friction=.7)
    x = np.array([0., 0., .2, 0., 0., 0.])
    required = robot.forces(x, np.zeros(6), np.zeros(6))
    assert required[2] == pytest.approx(9.81)


def test_contact_penalty_gradient_matches_finite_difference(sphere_urdf):
    from trajectory_optimize_contacts import (ContactObjective, _WORKER, evaluate_parallel,
                                              evaluation_pool)
    from scipy.optimize._numdiff import approx_derivative
    from time import perf_counter
    robot = RobotDynamics(sphere_urdf, [], None, lambda _: None, friction=.7)
    reference = np.tile([0., 0., .095, 0., 0., 0.], (3, 1))
    problem = CollocationProblem(robot, reference, [1], .1)
    variables = problem.pack(reference, np.zeros_like(reference))
    objective = ContactObjective(problem, perf_counter()+30)
    _, gradient = objective.evaluate(variables)
    numerical = approx_derivative(lambda x: objective.evaluate(x, gradient=False)[0], variables,
                                  method='3-point', abs_step=1e-6).ravel()
    np.testing.assert_allclose(gradient, numerical, atol=2e-5, rtol=2e-5)
    variables[0] += 2e-4
    variables[problem.position_size] += .005
    serial = []
    for _ in range(3):
        value, derivative = objective.evaluate(variables)
        serial.append((value, derivative.copy(), objective.dynamics_residual.copy(),
                       objective.effort_residual.copy()))
    _WORKER['objective'] = objective
    pool = evaluation_pool(2)
    assert pool is not None
    try:
        parallel = []
        for _ in range(3):
            value, derivative = evaluate_parallel(objective, pool, [[0], [1]], variables)
            parallel.append((value, derivative.copy(), objective.dynamics_residual.copy(),
                             objective.effort_residual.copy()))
    finally:
        pool.close()
        pool.join()
    # Sum order can round differently, but repeated QPs and their residuals
    # should agree much more closely than the finite-difference step above.
    for actual in serial + parallel:
        for observed, baseline in zip(actual, serial[0]):
            np.testing.assert_allclose(observed, baseline, atol=1e-8, rtol=1e-12)


def test_contact_optimization_repairs_ground_penetration(sphere_urdf):
    data = motion(3)
    data['base_pos_w'] = [[0., 0., .1], [0., 0., .095], [0., 0., .1]]
    before = copy.deepcopy(data)
    result = optimize_trajectory(data, [], sphere_urdf, friction=.7)
    assert data == before
    assert min(row[2] for row in result['trajectory']['base_pos_w']) >= .1 - 1e-5
    for name, residual in result['residuals'].items():
        assert residual <= result['tolerances'][name]


def test_stance_alignment_measures_shape_distance_instead_of_bounding_sphere(sphere_urdf, tmp_path):
    from pathlib import Path
    from trajectory_optimize_contacts import stance_offset
    path = tmp_path / 'foot.urdf'
    path.write_text(Path(sphere_urdf).read_text().replace(
        '<sphere radius="0.1"/>', '<box size="0.08 0.04 0.02"/>'))
    robot = RobotDynamics(str(path), [], None, lambda _: None, friction=.7)
    reference = np.tile([0., 0., .055, 0., 0., 0.], (40, 1))
    assert 0 < robot.clearances(reference[0])[0] < .02
    assert robot.clearances(reference[0], exact=True)[0] == pytest.approx(.045)
    problem = CollocationProblem(robot, reference, [], 1/90)
    offset, velocity = stance_offset(problem, reference, lambda: None)
    corrected = reference.copy()
    corrected[:, 2] += offset
    velocities = np.zeros_like(reference)
    velocities[:, 2] = velocity
    # A stationary hovering foot becomes supported without injecting motion or
    # changing the contact margin, and passes the actual dense validator.
    np.testing.assert_allclose(corrected[:, 2], .0105, atol=1e-12)
    np.testing.assert_allclose(velocity, 0., atol=1e-12)
    problem.validate(problem.pack(corrected, velocities))


def test_stance_offset_preserves_fixed_pose_and_continuous_acceleration(sphere_urdf):
    from trajectory_optimize_contacts import stance_offset
    robot = RobotDynamics(sphere_urdf, [], None, lambda _: None, friction=.7)
    reference = np.tile([0., 0., .14, 0., 0., 0.], (101, 1))
    problem = CollocationProblem(robot, reference, [50], .01)
    offset, velocity = stance_offset(problem, reference, lambda: None)
    assert offset[50] == pytest.approx(0., abs=1e-12)
    assert offset[0] < -.03
    # This checks the reconstructed curve, including both sides of each knot
    # and the fixed pose, rather than just the sampled offset values.
    for frame in range(1, problem.n-1):
        left = hermite(offset[frame-1], offset[frame], velocity[frame-1], velocity[frame], problem.dt, 1.)[2]
        right = hermite(offset[frame], offset[frame+1], velocity[frame], velocity[frame+1], problem.dt, 0.)[2]
        assert left == pytest.approx(right, abs=1e-8)


def test_stance_offset_leaves_distant_flight_unsupported(sphere_urdf):
    from trajectory_optimize_contacts import stance_offset
    robot = RobotDynamics(sphere_urdf, [], None, lambda _: None, friction=.7)
    reference = np.tile([0., 0., 2., 0., 0., 0.], (40, 1))
    problem = CollocationProblem(robot, reference, [], 1/90)
    offset, velocity = stance_offset(problem, reference, lambda: None)
    np.testing.assert_array_equal(offset, 0.)
    np.testing.assert_array_equal(velocity, 0.)
    assert not robot.contact_geometry[reference[0].tobytes()][0]


def test_support_envelope_gradient_matches_finite_difference(sphere_urdf, monkeypatch):
    import trajectory_optimize
    # The hover tie-break cost is outside the envelope argument; the search
    # penalizes loaded hovering contacts directly instead.
    monkeypatch.setattr(trajectory_optimize, 'HOVER_FORCE_COST', 0.)
    robot = RobotDynamics(sphere_urdf, [], None, lambda _: None, friction=.7)

    def check(state, schedule=None):
        options = {} if schedule is None else {'schedule': schedule}
        residual = robot.forces(*state, **options)
        assert np.max(np.abs(residual)) > 0
        gradients = robot.support_penalty_gradient(state, residual, **options)
        for field in range(3):
            for axis in range(6):
                plus, minus = [x.copy() for x in state], [x.copy() for x in state]
                plus[field][axis] += 1e-5
                minus[field][axis] -= 1e-5
                rp, rm = robot.forces(*plus, **options), robot.forces(*minus, **options)
                actual = (.5*(rp@rp) - .5*(rm@rm))/2e-5
                assert gradients[field][axis] == pytest.approx(actual, abs=2e-3, rel=2e-3)

    check([np.array([0., 0., .1, .2, -.1, .3]), np.zeros(6), np.array([20., 0., 0., 0., 0., 0.])])
    # A scheduled contact is a material point carried by the link, so its
    # envelope derivative differs from the live nearest-point case.
    x = np.array([0., 0., .12, .2, -.1, .3])
    schedule = robot.infer_contact_schedule(x)
    assert len(schedule) == 1
    check([x, np.zeros(6), np.array([0., 0., 50., 0., 5., 0.])], schedule)


def test_support_envelope_gradient_with_saturated_actuator(articulated_urdf):
    from scipy.optimize import lsq_linear
    from support import friction_rays
    robot = RobotDynamics(articulated_urdf, ['hinge'], None, lambda _: None, friction=.7)
    state = [np.array([0., 0., .095, .2, -.1, .3, .5]), np.zeros(7),
             np.array([0., 0., 1000., 0., 0., 0., 10000.])]

    def residual(values):
        torque = robot.forces(*values)
        return np.r_[torque[:6], torque[6:]-np.clip(torque[6:], -robot.effort, robot.effort)]

    weights = residual(state)
    assert abs(weights[6]) > 1
    assert np.max(robot._support_coefficients) > 1
    gradients = robot.support_penalty_gradient(state, weights)

    def independent_residual(values):
        # An independent bounded least-squares solve avoids differentiating
        # warm-start noise in the production QP's large squared residual.
        robot.clearances(values[0])
        maps, normals, _ = robot.contact_geometry[values[0].tobytes()]
        matrix = np.hstack([mapping @ friction_rays(normal, .7) for mapping, normal in zip(maps, normals)] +
                           [np.eye(7)[:, 6:]])
        lower = np.r_[np.zeros(4*len(maps)), -robot.effort]
        upper = np.r_[np.full(4*len(maps), np.inf), robot.effort]
        required = robot.required_forces(*values)
        result = lsq_linear(matrix, required, bounds=(lower, upper), method='bvls', tol=1e-12)
        assert result.success
        return required-matrix @ result.x

    direction = np.array([.1, -.2, .3, -.4, .5, -.6, .7])
    for field in range(3):
        plus, minus = [x.copy() for x in state], [x.copy() for x in state]
        plus[field] += 1e-5*direction
        minus[field] -= 1e-5*direction
        rp, rm = independent_residual(plus), independent_residual(minus)
        actual = (.5*(rp@rp)-.5*(rm@rm))/2e-5
        assert gradients[field] @ direction == pytest.approx(actual, rel=3e-3, abs=3e-2)


def test_initial_contact_probe_stops_at_first_violation(sphere_urdf, monkeypatch):
    from trajectory_optimize_contacts import ContactObjective
    from time import perf_counter
    robot = RobotDynamics(sphere_urdf, [], None, lambda _: None, friction=.7)
    reference = np.tile([0., 0., .095, 0., 0., 0.], (750, 1))
    problem = CollocationProblem(robot, reference, [], 1/30)
    calls = []
    original = robot.clearances

    def counted(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)

    monkeypatch.setattr(robot, 'clearances', counted)
    objective = ContactObjective(problem, perf_counter()+30)
    objective.evaluate(problem.pack(reference, np.zeros_like(reference)), gradient=False, stop_on_violation=True)
    assert not objective.feasible()
    assert len(calls) == 1


def test_contained_geometry_uses_outward_support_normal(sphere_urdf):
    import trimesh
    terrain = trimesh.creation.box(extents=[10, 10, 2]).export(file_type='obj')
    robot = RobotDynamics(sphere_urdf, [], terrain, lambda _: None, friction=.7)
    x = np.array([0., 0., .3, 0., 0., 0.])
    clearance, derivative = robot.clearances(x, derivatives=True)
    assert clearance[0] < 0
    normal = robot.contact_geometry[x.tobytes()][1][0]
    np.testing.assert_allclose(normal, derivative[0, :3], atol=1e-8)
    assert normal[2] > .99


def test_fixed_hovering_pose_fails_validation(sphere_urdf):
    # The saved trajectory has positions only. Free nodal velocities must not
    # let a static pose 5 cm above the ground pass as dynamically consistent.
    data = motion(4)
    data["base_pos_w"] = [[0., 0., .15]] * 4
    result = optimize_trajectory(data, [0, 1, 2, 3], sphere_urdf, friction=.7)
    assert result["success"] is False
    assert result["residuals"]["dynamics"] == pytest.approx(9.81)


def test_contact_optimization_grounds_hovering_interior(sphere_urdf):
    # Scheduled search contacts reach 5 cm, but validation only accepts 1 mm.
    # The search must close the gap rather than lean on hovering support.
    n = 30
    data = motion(n)
    data["framerate"] = 30.
    data["base_pos_w"] = [[0., 0., .1]] + [[0., 0., .13]] * (n-2) + [[0., 0., .1]]
    result = optimize_trajectory(data, [], sphere_urdf, friction=.7, time_budget=40)
    assert result["success"], result.get("error")
    heights = np.array(result["trajectory"]["base_pos_w"])[:, 2]
    assert np.all(heights < .1 + 1e-3)
