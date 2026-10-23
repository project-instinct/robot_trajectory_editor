"""Analytic fixtures for Trajectory Optimize; no user motion files are needed."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from trajectory_optimize import (CONTACT_MARGIN, Evaluation, Problem, RobotModel, TrajectoryOptimizeError,
                                 frame_equations, initialize, optimize_trajectory, read_trajectory,
                                 solve_normal_equations, validate_frame)


def motion(heights, rate=10., joints=None):
    n = len(heights)
    joints = [[] for _ in range(n)] if joints is None else [list(j) for j in joints]
    return {"framerate": rate, "joint_names": [] if not joints[0] else ["hinge"], "joint_pos": joints,
            "base_pos_w": [[0., 0., float(h)] for h in heights], "base_quat_w": [[1., 0., 0., 0.]] * n}


def ballistic(n, rate=10.):
    t = np.arange(n) / rate
    return 2. + .2*t - 4.905*t*t


@pytest.fixture
def sphere_urdf(tmp_path):
    for module in ("pinocchio", "coal", "trimesh", "scipy", "threadpoolctl"):
        pytest.importorskip(module)
    path = tmp_path / "sphere.urdf"
    path.write_text('''<robot name="sphere"><link name="base">
      <inertial><mass value="1"/><inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.004"/></inertial>
      <collision><geometry><sphere radius="0.1"/></geometry></collision>
    </link></robot>''')
    return str(path)


@pytest.fixture
def articulated_urdf(sphere_urdf, tmp_path):
    from pathlib import Path
    path = tmp_path / "articulated.urdf"
    path.write_text(Path(sphere_urdf).read_text().replace("</robot>", '''
      <link name="arm"><inertial><origin xyz="0.2 0 0"/><mass value="0.5"/>
        <inertia ixx="0.01" ixy="0.001" ixz="0" iyy="0.02" iyz="0" izz="0.025"/></inertial>
        <collision><origin xyz="0.4 0 0"/><geometry><sphere radius="0.12"/></geometry></collision></link>
      <joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>
        <origin xyz="0.1 0 0.2" rpy="0.2 0.3 -0.1"/><axis xyz="0 1 0"/>
        <limit lower="-2" upper="2" velocity="5" effort="20"/></joint>
    </robot>'''))
    return str(path)


def test_invalid_quaternion_and_keyframe_diagnostics():
    data = motion([1., 1., 1.])
    with pytest.raises(TrajectoryOptimizeError, match="Keyframes"):
        read_trajectory(data, [3])
    data["base_quat_w"] = [[2., 0., 0., 0.]] * 3
    with pytest.raises(TrajectoryOptimizeError, match="quaternion"):
        read_trajectory(data, [])


def test_missing_inertial_data_has_clear_diagnostic(tmp_path):
    pytest.importorskip("pinocchio")
    path = tmp_path / "missing.urdf"
    path.write_text('<robot name="missing"><link name="base"/></robot>')
    with pytest.raises(TrajectoryOptimizeError, match="Missing mass or inertia"):
        optimize_trajectory(motion(ballistic(3)), [], str(path))


def test_empty_fixed_sensor_frame_is_not_missing_body_data(sphere_urdf, tmp_path):
    from pathlib import Path
    path = tmp_path / "with_sensor.urdf"
    path.write_text(Path(sphere_urdf).read_text().replace("</robot>", '''
      <link name="sensor"/>
      <joint name="sensor_frame" type="fixed"><parent link="base"/><child link="sensor"/></joint>
    </robot>'''))
    assert RobotModel(str(path), [], None, lambda _: None).model.nv == 6


def test_effort_scale_expands_urdf_limits(articulated_urdf):
    robot = RobotModel(articulated_urdf, ["hinge"], None, lambda _: None, effort_scale=2.)
    assert robot.effort == pytest.approx([40.])


def test_contact_free_motion_recovers_ballistic_flight_and_keeps_fixed_frames(sphere_urdf):
    heights = ballistic(6)
    noisy = heights + np.r_[0., .01, -.01, .02, .01, 0.]
    data = motion(noisy)
    result = optimize_trajectory(data, [], sphere_urdf, friction=None)
    assert result["success"], result.get("error")
    output = result["trajectory"]
    assert output["framerate"] == data["framerate"] and output["joint_names"] == []
    # No keyframes: the first and last frames are the fixed boundary poses.
    assert output["base_pos_w"][0] == data["base_pos_w"][0]
    assert output["base_pos_w"][-1] == data["base_pos_w"][-1]
    np.testing.assert_allclose(np.array(output["base_pos_w"])[:, 2], heights, atol=1e-4)


def test_tagged_keyframes_stay_exact_and_others_are_free(sphere_urdf):
    data = motion(ballistic(7) + np.r_[0., .01, 0., -.02, 0., .01, 0.])
    result = optimize_trajectory(data, [0, 3, 6], sphere_urdf, friction=None)
    output = result["trajectory"]
    for frame in (0, 3, 6):
        assert output["base_pos_w"][frame] == data["base_pos_w"][frame]
    assert output["base_pos_w"][1] != data["base_pos_w"][1]


def test_supported_hovering_interior_is_lowered_onto_the_ground(sphere_urdf):
    result = optimize_trajectory(motion([.1] + [.13]*28 + [.1], rate=30.), [], sphere_urdf, friction=.7)
    assert result["success"], result.get("error")
    heights = np.array(result["trajectory"]["base_pos_w"])[:, 2]
    assert np.all(heights - .1 <= CONTACT_MARGIN)


def test_hovering_first_frame_becomes_a_drop(sphere_urdf):
    result = optimize_trajectory(motion([.109] + [.1005]*40, rate=90.), [], sphere_urdf, friction=.7)
    assert result["success"], result.get("error")
    heights = np.array(result["trajectory"]["base_pos_w"])[:, 2]
    assert heights[0] == pytest.approx(.109)
    assert np.all(np.diff(heights[:4]) < 0)


def test_static_hover_with_every_frame_fixed_fails_with_preview(sphere_urdf):
    result = optimize_trajectory(motion([.15]*4), [0, 1, 2, 3], sphere_urdf, friction=.7)
    assert result["success"] is False
    assert result["residuals"]["dynamics"] == pytest.approx(9.81)
    assert len(result["trajectory"]["base_pos_w"]) == 4


def test_fixed_keyframe_penetration_is_rejected(sphere_urdf):
    with pytest.raises(TrajectoryOptimizeError, match="penetrates terrain"):
        optimize_trajectory(motion([.05, .1, .1]), [], sphere_urdf, friction=.7)


def test_stands_on_a_loaded_terrain_mesh(sphere_urdf):
    terrain = "v -5 -5 1\nv 5 -5 1\nv 5 5 1\nv -5 5 1\nf 1 2 3\nf 1 3 4\n"
    result = optimize_trajectory(motion([1.1005] + [1.12]*18 + [1.1005], rate=30.), [], sphere_urdf,
                                 terrain_obj=terrain, friction=.7)
    assert result["success"], result.get("error")
    heights = np.array(result["trajectory"]["base_pos_w"])[:, 2]
    assert np.all(heights - 1.1 <= CONTACT_MARGIN)


def test_budget_exhausted_still_returns_preview(sphere_urdf):
    heights = ballistic(6) + np.r_[0., .01, -.01, .02, .01, 0.]
    result = optimize_trajectory(motion(heights), [], sphere_urdf, friction=None, time_budget=1e-3)
    assert result["success"] is False
    assert "time budget" in result["error"]
    assert len(result["trajectory"]["base_pos_w"]) == 6


def test_validator_checks_poses_between_frames(articulated_urdf):
    robot = RobotModel(articulated_urdf, ["hinge"], None, lambda _: None, friction=.7)

    def pose(height, angle):
        return robot.configuration([0., 0., height], [1., 0., 0., 0.], [angle])

    # Find a height where both frames clear the ground but the arm dips
    # into it halfway between them.
    heights = np.linspace(.2, 1., 81)
    clear = [h for h in heights
             if min(robot.distances(pose(h, a))[0].min() for a in (.6, 2.)) > 0
             and robot.distances(pose(h, 1.3))[0].min() < -1e-3]
    assert clear
    Q = np.array([pose(clear[0], .6), pose(clear[0], 2.)])
    problem = Problem(robot, Q, [0, 1], .1)
    errors, _ = validate_frame(problem, Q, 0)
    assert errors["clearance"] > 1e-3
    assert min(robot.distances(q)[0].min() for q in Q) > 0


def test_flat_cylinder_support_points_are_its_rim_ends(sphere_urdf, tmp_path):
    from pathlib import Path
    path = tmp_path / "rod.urdf"
    path.write_text(Path(sphere_urdf).read_text().replace(
        '<geometry><sphere radius="0.1"/></geometry>',
        '<origin rpy="0 1.5707963267948966 0"/><geometry><cylinder radius="0.05" length="0.4"/></geometry>'))
    robot = RobotModel(str(path), [], None, lambda _: None, friction=.7)
    q = robot.configuration([0., 0., .0505], [1., 0., 0., 0.], [])
    _, nearest, outward = robot.distances(q)
    points = [robot.support_point(key, nearest, outward) for key in robot.support_keys(0, nearest, outward)]
    np.testing.assert_allclose(sorted(p[1][0] for p in points), [-.2, .2], atol=1e-9)
    np.testing.assert_allclose([p[2] for p in points], [5e-4, 5e-4], atol=1e-9)


@pytest.mark.parametrize("friction", [None, .7])
def test_residual_jacobian_matches_finite_differences(articulated_urdf, friction):
    # Without support the dynamics Jacobian is exact. With support it
    # approximates how the allocated forces follow the motion, so only the
    # other residuals are compared there.
    robot = RobotModel(articulated_urdf, ["hinge"], None, lambda _: None, friction=friction)
    rng = np.random.default_rng(0)
    base = robot.configuration([0., 0., .2], [1., 0., 0., 0.], [.3])
    base[2] -= robot.distances(base)[0].min() - 4e-4
    Q = np.array([robot.pin.integrate(robot.model, base, rng.normal(size=7) * 1e-3) for _ in range(5)])
    problem = Problem(robot, Q.copy(), [0, 4], 1/30)
    initialize(problem)
    for k in (1, 2, 3):
        residual, block, _ = problem.terms(Q, k, True)
        step = rng.normal(size=(3, 7)) * 1e-7
        moved = Q.copy()
        for i, frame in enumerate((k - 1, k, k + 1)):
            if problem.free[frame]:
                moved[frame] = robot.pin.integrate(robot.model, Q[frame], step[i])
            else:
                step[i] = 0.
        changed, _, _ = problem.terms(moved, k, False)
        assert len(changed) == len(residual)
        rows = np.ones(len(residual), dtype=bool)
        if friction is not None:
            # Dynamics rows follow the acceleration rows of an interior frame.
            rows[7:13] = False
        np.testing.assert_allclose((changed - residual)[rows], (block @ step.ravel())[rows],
                                   atol=1e-3 * np.abs(changed - residual).max() + 1e-9)


def test_block_solver_matches_dense_solve(sphere_urdf):
    robot = RobotModel(sphere_urdf, [], None, lambda _: None)
    rng = np.random.default_rng(1)
    n, nv = 7, 6
    problem = Problem(robot, np.tile(robot.configuration([0, 0, 1.], [1, 0, 0, 0], []), (n, 1)), [0, 4], .1)
    terms = [(rng.normal(size=4), rng.normal(size=(4, 3 * nv)), {}) for _ in range(n)]
    # Two worker chunks, as the evaluator would combine them.
    evaluation = Evaluation(problem, [(0., [], frame_equations(problem, np.arange(3), terms[:3])),
                                      (0., [], frame_equations(problem, np.arange(3, n), terms[3:]))])
    evaluation.cost = sum(float(r @ r) for r, _, _ in terms)
    step = solve_normal_equations(evaluation.equations, 1e-2, np.ones(nv))[1:n+1]
    # Dense reference over the free frames only.
    total = np.zeros((n * nv, n * nv))
    gradient = np.zeros(n * nv)
    for k, (residual, block, _) in enumerate(terms):
        full = np.zeros((len(residual), (n + 2) * nv))
        full[:, k*nv:(k+3)*nv] = block
        full = full[:, nv:-nv]
        for frame in np.flatnonzero(~problem.free):
            full[:, frame*nv:(frame+1)*nv] = 0.
        total += full.T @ full
        gradient += full.T @ residual
    free = np.repeat(problem.free, nv)
    damped = total[np.ix_(free, free)]
    damped[np.diag_indices_from(damped)] += 1e-2 * np.maximum(np.diag(damped), 1.) + 1e-12
    expected = np.zeros(n * nv)
    expected[free] = np.linalg.solve(damped, -gradient[free])
    np.testing.assert_allclose(step.ravel(), expected, rtol=1e-8, atol=1e-10)
    # The Gauss-Newton model cost from the normal equations.
    padded = np.zeros((n + 2, nv))
    padded[1:-1] = step
    model = sum(float(np.sum((r + B @ np.where(np.r_[False, problem.free, False][k:k+3, None], padded[k:k+3], 0.)
                                .ravel())**2)) for k, (r, B, _) in enumerate(terms))
    assert evaluation.predicted(step) == pytest.approx(model, rel=1e-10)
