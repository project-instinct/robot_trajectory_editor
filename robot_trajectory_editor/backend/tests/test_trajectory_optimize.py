"""Analytic fixtures for Trajectory Optimize; no user motion files are needed."""
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from trajectory_optimize import (CONTACT_MARGIN, Evaluation, Evaluator, Problem, RobotModel, TrajectoryOptimizeError,
                                 frame_equations, infer_schedule, initialize, optimize_trajectory, read_trajectory,
                                 repair_glitches, smooth_target, solve_normal_equations, update_multipliers,
                                 update_sliding, validate_frame)


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
    # Two frames have no interior frame whose dynamics could be checked.
    with pytest.raises(TrajectoryOptimizeError, match="at least three frames"):
        read_trajectory(motion([1., 1.]), [])
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
    # No keyframes: the endpoints keep their poses except the base height.
    for frame in (0, -1):
        assert output["base_pos_w"][frame][:2] == data["base_pos_w"][frame][:2]
        assert output["base_quat_w"][frame] == data["base_quat_w"][frame]
    # The result is a free-fall parabola.
    z = np.array(output["base_pos_w"])[:, 2]
    np.testing.assert_allclose(np.diff(z, 2) * data["framerate"]**2, -9.81, atol=1e-2)


def test_tagged_keyframes_stay_exact_and_others_are_free(sphere_urdf):
    # Noisy keyframes leave no exact ballistic solution; a short budget suffices.
    data = motion(ballistic(7) + np.r_[0., .01, 0., -.02, 0., .01, 0.])
    result = optimize_trajectory(data, [0, 3, 6], sphere_urdf, friction=None, time_budget=2.)
    output = result["trajectory"]
    for frame in (0, 3, 6):
        assert output["base_pos_w"][frame] == data["base_pos_w"][frame]
    assert output["base_pos_w"][1] != data["base_pos_w"][1]


def test_supported_hovering_interior_is_lowered_onto_the_ground(sphere_urdf):
    result = optimize_trajectory(motion([.1] + [.13]*28 + [.1], rate=30.), [], sphere_urdf, friction=.7)
    assert result["success"], result.get("error")
    heights = np.array(result["trajectory"]["base_pos_w"])[:, 2]
    assert np.all(heights - .1 <= CONTACT_MARGIN)


def test_hovering_tagged_first_frame_becomes_a_drop(sphere_urdf):
    result = optimize_trajectory(motion([.109] + [.1005]*40, rate=90.), [0, 40], sphere_urdf, friction=.7)
    assert result["success"], result.get("error")
    heights = np.array(result["trajectory"]["base_pos_w"])[:, 2]
    assert heights[0] == pytest.approx(.109)
    assert np.all(np.diff(heights[:4]) < 0)


def test_untagged_hovering_or_sunken_endpoints_move_only_vertically(sphere_urdf):
    # The first frame hovers 9 mm and the last sinks 3 mm: both are moved
    # onto the ground instead of forcing a drop and a jump out of the floor.
    quat = (np.array([.98, .1, .1, .1]) / np.linalg.norm([.98, .1, .1, .1])).tolist()
    heights = .1 + np.r_[.009, np.full(28, 5e-4), -.003]
    data = {"framerate": 30., "joint_names": [], "joint_pos": [[]] * 30,
            "base_pos_w": [[.2, -.1, h] for h in heights], "base_quat_w": [quat] * 30}
    result = optimize_trajectory(data, [], sphere_urdf, friction=.7, smoothing_hz=0.)
    assert result["success"], result.get("report")
    output = result["trajectory"]
    for frame in (0, -1):
        assert output["base_quat_w"][frame] == data["base_quat_w"][frame]
        assert output["base_pos_w"][frame][:2] == data["base_pos_w"][frame][:2]
        assert -1e-5 <= output["base_pos_w"][frame][2] - .1 <= CONTACT_MARGIN


def test_static_hover_with_every_frame_fixed_fails_with_preview_and_report(sphere_urdf):
    result = optimize_trajectory(motion([.15]*4), [0, 1, 2, 3], sphere_urdf, friction=.7)
    assert result["success"] is False
    assert result["residuals"]["dynamics"] == pytest.approx(9.81)
    assert len(result["trajectory"]["base_pos_w"]) == 4
    assert "unsupported base force 9.81 N near frame 1" in result["error"]
    report = result["report"].splitlines()
    assert report[0] == "Trajectory Optimize: FAILED (not physically validated)"
    assert report[1] == "Motion: 4 frames at 10 fps, 0 joints; keyframes: frames 0, 1, 2, 3"
    violation = next(line for line in report if line.strip().startswith("dynamics"))
    assert "9.81" in violation and "N" in violation and "frame 1" in violation and "over tolerance in 2 frames" in violation
    assert report[-2].startswith("Within tolerance: effort 0")
    assert report[-1].startswith("Sliding contacts: none")


def test_validated_run_reports_every_check_within_tolerance(sphere_urdf):
    result = optimize_trajectory(motion(ballistic(6)), [], sphere_urdf, friction=None)
    assert result["success"]
    assert result["report"].splitlines()[0] == "Trajectory Optimize: VALIDATED"
    assert "Violations" not in result["report"]


def test_multiplier_update_accumulates_violations_and_grows_stalled_penalties():
    class Stub:
        values = [{"dynamics": np.array([.5, -.2]), "effort": np.array([-1., .3, -np.inf])}]
    weights = {"data": 1., "acceleration": 1., "dynamics": 1., "effort": 1., "position": 1., "velocity": 1.,
               "contact": 1., "clearance": 1., "middle": 1.}
    multipliers = [{}]
    worst = update_multipliers(Stub, multipliers, weights, {})
    np.testing.assert_allclose(multipliers[0]["dynamics"], [.5, -.2])
    np.testing.assert_allclose(multipliers[0]["effort"], [0., .3, 0.])
    assert worst["dynamics"] == pytest.approx(.5) and worst["effort"] == pytest.approx(.3)
    assert weights["dynamics"] == 1.
    # No progress: the factor grows tenfold, and the shift y / rho is kept.
    update_multipliers(Stub, multipliers, weights, worst)
    assert weights["dynamics"] == 10.
    np.testing.assert_allclose(multipliers[0]["dynamics"] / weights["dynamics"], [1., -.4])


def test_fixed_keyframe_penetration_is_rejected(sphere_urdf):
    with pytest.raises(TrajectoryOptimizeError, match="penetrates terrain"):
        optimize_trajectory(motion([.05, .1, .1]), [0, 2], sphere_urdf, friction=.7)


def test_stands_on_a_loaded_terrain_mesh(sphere_urdf):
    terrain = "v -5 -5 1\nv 5 -5 1\nv 5 5 1\nv -5 5 1\nf 1 2 3\nf 1 3 4\n"
    # Hovering 2 cm above the mesh throughout, endpoints included.
    result = optimize_trajectory(motion([1.12]*20, rate=30.), [], sphere_urdf, terrain_obj=terrain, friction=.7)
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
    errors, where, _ = validate_frame(problem, Q, 0)
    assert errors["clearance"] > 1e-3
    assert where["clearance"].endswith(tuple("0123456789"))  # names the geometry and the in-between frame
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
    slipping = 0
    for k in (1, 2, 3):
        residual, block, _, values = problem.terms(Q, k, True)
        slipping += len(values.get("slip", ()))
        step = rng.normal(size=(3, 7)) * 1e-7
        moved = Q.copy()
        for i, frame in enumerate((k - 1, k, k + 1)):
            if problem.free[frame]:
                moved[frame] = robot.pin.integrate(robot.model, Q[frame], step[i])
            else:
                step[i] = 0.
        changed, _, _, _ = problem.terms(moved, k, False)
        assert len(changed) == len(residual)
        rows = np.ones(len(residual), dtype=bool)
        if friction is not None:
            # Dynamics rows follow the acceleration rows of an interior frame,
            # then the planted points' positions: their Jacobian follows the
            # material point, while a sphere's lowest point rolls.
            rows[7:13 + values.get("contact_position", 0)] = False
        np.testing.assert_allclose((changed - residual)[rows], (block @ step.ravel())[rows],
                                   atol=1e-3 * np.abs(changed - residual).max() + 1e-9)
    # With support, the resting spheres' no-slip rows are among those checked.
    assert slipping > 0 if friction is not None else slipping == 0


def test_block_solver_matches_dense_solve(sphere_urdf):
    robot = RobotModel(sphere_urdf, [], None, lambda _: None)
    rng = np.random.default_rng(1)
    n, nv = 7, 6
    problem = Problem(robot, np.tile(robot.configuration([0, 0, 1.], [1, 0, 0, 0], []), (n, 1)), [0, 4], .1)
    terms = [(rng.normal(size=4), rng.normal(size=(4, 3 * nv)), {}, {}) for _ in range(n)]
    # Two worker chunks, as the evaluator would combine them.
    evaluation = Evaluation(problem, [(0., [], [], frame_equations(problem, np.arange(3), terms[:3])),
                                      (0., [], [], frame_equations(problem, np.arange(3, n), terms[3:]))])
    evaluation.cost = sum(float(r @ r) for r, _, _, _ in terms)
    step = solve_normal_equations(evaluation.equations, 1e-2, np.ones(nv))[1:n+1]
    # Dense reference over the free frames only.
    total = np.zeros((n * nv, n * nv))
    gradient = np.zeros(n * nv)
    for k, (residual, block, _, _) in enumerate(terms):
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
                                .ravel())**2)) for k, (r, B, _, _) in enumerate(terms))
    assert evaluation.predicted(step) == pytest.approx(model, rel=1e-10)


def test_glitch_repair_interpolates_joint_jumps_but_keeps_fixed_frames(articulated_urdf):
    robot = RobotModel(articulated_urdf, ["hinge"], None, lambda _: None)
    clean = np.linspace(0., .2, 60)[:, None]  # 0.34 rad/s at 100 fps; the limit is 5 rad/s
    joints = clean.copy()
    joints[12] += 1.3   # a one-frame spike
    joints[40:] += .3   # a step
    joints[50] -= 1.    # a spike on a fixed frame stays
    repaired, count = repair_glitches(robot, joints, [0, 50, 59], 100., 1.)
    assert count > 0
    assert repaired[50, 0] == joints[50, 0]
    np.testing.assert_allclose(repaired[:10], clean[:10], atol=1e-12)
    assert abs(repaired[12, 0] - clean[12, 0]) < .01
    speeds = np.abs(np.diff(repaired[:46, 0])) * 100
    assert np.all(speeds <= 5. + 1e-9)
    unchanged, none = repair_glitches(robot, joints, [], 100., 0.)
    assert none == 0
    np.testing.assert_array_equal(unchanged, joints)


def test_smoothed_target_removes_jitter_and_keeps_fixed_frames_exact(articulated_urdf):
    robot = RobotModel(articulated_urdf, ["hinge"], None, lambda _: None)
    rng = np.random.default_rng(2)
    n, rate = 120, 90.
    t = np.arange(n) / rate
    smooth = .5 * np.sin(2 * np.pi * t)
    noisy = smooth + rng.normal(scale=.02, size=n)
    reference = np.array([robot.configuration([0., 0., 1. + .01 * rng.normal()], [1., 0., 0., 0.], [x])
                          for x in noisy])
    target, _ = smooth_target(robot, reference, [0, 60, n - 1], rate, 6., 0.)
    for frame in (0, 60, n - 1):
        np.testing.assert_array_equal(target[frame], reference[frame])
    joints = target[:, robot.joint_q][:, 0]
    interior = np.r_[10:50, 70:110]
    assert np.std(joints[interior] - smooth[interior]) < .3 * np.std(noisy[interior] - smooth[interior])
    unchanged, _ = smooth_target(robot, reference, [0, n - 1], rate, 0., 0.)
    np.testing.assert_array_equal(unchanged, reference)


@pytest.mark.parametrize("option", ["smoothing_hz", "glitch_speed_scale"])
def test_target_options_are_validated(sphere_urdf, option):
    with pytest.raises(TrajectoryOptimizeError, match=option):
        optimize_trajectory(motion(ballistic(3)), [], sphere_urdf, **{option: -1.})


def test_support_is_inferred_where_the_motion_is_not_free_fall(sphere_urdf):
    robot = RobotModel(sphere_urdf, [], None, lambda _: None, friction=.7)
    rate = 30.
    # Held still 10 cm above the ground: something must support it.
    held = np.array([robot.configuration([0., 0., .2], [1., 0., 0., 0.], [])] * 30)
    schedule = infer_schedule(Problem(robot, held, [0, 29], 1 / rate), held)
    assert all(len(keys) == 1 for keys in schedule[1:-1])
    # Falling freely from the same height: nothing is needed.
    t = np.arange(30) / rate
    falling = np.array([robot.configuration([0., 0., .35 - 4.905 * x * x + .5 * x], [1., 0., 0., 0.], [])
                        for x in t[:8]])
    schedule = infer_schedule(Problem(robot, falling, [0, 7], 1 / rate), falling)
    assert all(len(keys) == 0 for keys in schedule[1:-1])


def test_held_up_hovering_body_is_brought_onto_the_ground(sphere_urdf):
    # The interior hovers 6 cm up, beyond the proximity schedule, while the
    # endpoints rest on the ground. Support inferred from the dynamics
    # brings it down instead of leaving an unsupported hover.
    result = optimize_trajectory(motion([.1005] + [.16]*28 + [.1005], rate=30.), [], sphere_urdf, friction=.7,
                                 smoothing_hz=0.)
    assert result["success"], result.get("report")
    heights = np.array(result["trajectory"]["base_pos_w"])[:, 2]
    assert np.all(heights - .1 <= CONTACT_MARGIN)


def test_tilted_endpoint_is_lowered_without_penetrating(articulated_urdf):
    # An endpoint that may only move vertically, with the body tilted so that
    # only one sphere can touch: it is lowered onto that sphere, not pushed in.
    robot = RobotModel(articulated_urdf, ["hinge"], None, lambda _: None, friction=.7)
    quat = (np.array([.98, .1, .1, .1]) / np.linalg.norm([.98, .1, .1, .1])).tolist()
    q = robot.configuration([0., 0., 1.], quat, [.3])
    lift = 1. - robot.distances(q)[0].min()
    data = {"framerate": 30., "joint_names": ["hinge"], "joint_pos": [[.3]] * 3,
            "base_pos_w": [[0., 0., lift + .02]] * 3, "base_quat_w": [quat] * 3}
    result = optimize_trajectory(data, [], articulated_urdf, friction=.7, smoothing_hz=0., time_budget=10.)
    last = result["trajectory"]
    q = robot.configuration(last["base_pos_w"][-1], last["base_quat_w"][-1], last["joint_pos"][-1])
    assert robot.distances(q)[0].min() >= -1e-5


def gliding(n, speed, rate=30.):
    """A resting sphere translated along x at constant speed without turning."""
    return {"framerate": rate, "joint_names": [], "joint_pos": [[]] * n,
            "base_pos_w": [[speed * k / rate, 0., .1005] for k in range(n)], "base_quat_w": [[1., 0., 0., 0.]] * n}


def test_validator_gives_a_gliding_contact_only_kinetic_friction(sphere_urdf):
    # Gliding at constant speed on a resting contact: kinetic friction acts
    # against the slip and nothing balances it (with a sticking contact the
    # weight alone would be supported exactly).
    result = optimize_trajectory(gliding(10, .3), list(range(10)), sphere_urdf, friction=.7)
    assert not result["success"]
    assert result["residuals"]["dynamics"] > 4.
    assert "Sliding contacts (kinetic friction, checked in dynamics): 8 frames, fastest 0.3 m/s" in result["report"]


def test_resting_contact_that_skates_in_the_target_sticks(sphere_urdf):
    # The target skates 1.5 cm forth and back on a resting sphere. A planted
    # contact may not slide, so the sphere rolls or stays put instead.
    n, rate = 30, 30.
    data = gliding(n, 0., rate)
    for k in range(n):
        data["base_pos_w"][k][0] = .015 * np.sin(np.pi * k / (n - 1))
    result = optimize_trajectory(data, [], sphere_urdf, friction=.7)
    assert result["success"], result.get("report")
    assert "Sliding contacts: none" in result["report"]


def test_static_friction_is_the_limit_that_stops_slipping(sphere_urdf):
    robot = RobotModel(sphere_urdf, [], None, lambda _: None, friction=.5)
    problem = Problem(robot, np.array([robot.configuration([0., 0., .1], [1., 0., 0., 0.], [])] * 3), [], 1 / 30)
    up, still = np.array([[0., 0., 1.]]), np.zeros((1, 2))

    def values(force, balanced, slip=still):
        return [{}, {"support": (np.array([7]), np.array([force]), up, slip, np.array([True]), balanced)}, {}]
    # Inside the cone, or failing dynamics but with friction to spare: sticks.
    assert update_sliding(problem, values([1., 0., 10.], True)) == [] and problem.sliding[1] == {}
    update_sliding(problem, values([3., 0., 10.], False))
    assert problem.sliding[1] == {}
    # All static friction used and the frame still fails: slides against it.
    assert update_sliding(problem, values([5., 0., 10.], True)) == []
    assert update_sliding(problem, values([5., 0., 10.], False)) == [1]
    np.testing.assert_allclose(problem.sliding[1][7], [-1., 0., 0.])
    # It keeps sliding while it moves, and sticks again once almost stopped.
    from support import tangent_basis
    moving = (tangent_basis(up[0]) @ np.array([-.2, 0., 0.]))[None]
    assert update_sliding(problem, values([5., 0., 10.], True, moving)) == []
    np.testing.assert_allclose(problem.sliding[1][7], [-1., 0., 0.], atol=1e-12)
    assert update_sliding(problem, values([5., 0., 10.], True, moving * 1e-2)) == [1]
    assert problem.sliding[1] == {}


def test_overlapping_requests_evaluate_their_own_problems(sphere_urdf, articulated_urdf):
    # Two requests in one server process: creating the second evaluator must
    # not redirect the first one's single-process evaluation.
    sphere = RobotModel(sphere_urdf, [], None, lambda _: None, friction=.7)
    arm = RobotModel(articulated_urdf, ["hinge"], None, lambda _: None, friction=.7)
    first = Problem(sphere, np.array([sphere.configuration([0., 0., .3], [1., 0., 0., 0.], [])] * 4), [], .1)
    first_evaluator = Evaluator(first, 1)
    expected = first_evaluator.evaluate(first.reference, False).cost
    second = Problem(arm, np.array([arm.configuration([0., 0., 1.], [1., 0., 0., 0.], [.2])] * 9), [], .1)
    Evaluator(second, 1)
    assert first_evaluator.evaluate(first.reference, False).cost == expected
    assert len(first_evaluator.validate(first.reference)) == 4


def test_frictionless_contact_never_slides_in_an_undefined_direction(sphere_urdf):
    robot = RobotModel(sphere_urdf, [], None, lambda _: None, friction=0.)
    problem = Problem(robot, np.array([robot.configuration([0., 0., .1], [1., 0., 0., 0.], [])] * 3), [], 1 / 30)
    values = [{}, {"support": (np.array([7]), np.array([[0., 0., 10.]]), np.array([[0., 0., 1.]]),
                               np.zeros((1, 2)), np.array([True]), False)}, {}]
    assert update_sliding(problem, values) == [] and problem.sliding[1] == {}


def test_geometry_wholly_inside_a_solid_terrain_is_penetrating(sphere_urdf):
    # A 4 m solid box with 0.5 m terrain cells: a sphere at its centre is 2 m
    # from the surface but inside the solid, so culling must not hide it.
    import trimesh
    box = trimesh.creation.box(extents=[4., 4., 4.])
    obj = "".join(f"v {x} {y} {z}\n" for x, y, z in box.vertices) + \
        "".join(f"f {a + 1} {b + 1} {c + 1}\n" for a, b, c in box.faces)
    robot = RobotModel(sphere_urdf, [], obj, lambda _: None, friction=.7)
    assert robot.containment and robot.terrain_grid is not None
    inside = robot.configuration([0., 0., 0.], [1., 0., 0., 0.], [])
    for cull in (None, CONTACT_MARGIN):
        assert robot.distances(inside, cull=cull)[0].min() < -1.
    outside = robot.configuration([0., 0., 4.], [1., 0., 0., 0.], [])
    assert robot.distances(outside, cull=CONTACT_MARGIN)[0].min() > 1.


def test_one_sided_joint_limit_is_still_enforced(articulated_urdf, tmp_path):
    path = tmp_path / "one_sided.urdf"
    path.write_text(Path(articulated_urdf).read_text().replace(
        '<limit lower="-2" upper="2" velocity="5" effort="20"/>', '<limit upper="1" velocity="5" effort="20"/>')
        .replace('type="revolute"', 'type="prismatic"'))
    robot = RobotModel(str(path), ["hinge"], None, lambda _: None)
    if np.isfinite(robot.lower[0]):
        pytest.skip("this URDF parser does not leave the missing bound infinite")
    Q = np.array([robot.configuration([0., 0., 2.], [1., 0., 0., 0.], [x]) for x in (1.2, 1.2, 1.2)])
    problem = Problem(robot, Q, [], .1)
    residual, block, stats, values = problem.terms(Q, 1, True, dynamics=False)
    assert stats["position"] == pytest.approx(.2)
    assert np.isfinite(values["position"]).sum() == 1
