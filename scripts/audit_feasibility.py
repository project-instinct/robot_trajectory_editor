"""Audit how far an unmodified trajectory is from the Trajectory Optimize model.

Runs the same independent validator as Trajectory Optimize on the reference
itself, without optimizing, and reports the worst residuals overall and per
window of frames. This separates "the motion asks for more than the model
allows" from "the optimizer failed to find a feasible motion".
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_trajectory_editor/backend'))
from npz_io import parse_npz
from trajectory_optimize import Problem, RobotModel, read_trajectory, validate_frame


def terrain_text(path):
    if path is None:
        return None
    if path.suffix.lower() == '.obj':
        return path.read_text()
    text = trimesh.load(path, force='mesh').export(file_type='obj')
    return text.decode() if isinstance(text, bytes) else text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('--terrain', type=Path)
    parser.add_argument('--frames', type=int, default=0, help='prefix length; 0 means all frames')
    parser.add_argument('--friction', type=float, default=.7)
    parser.add_argument('--window', type=int, default=90, help='frames per reported window')
    args = parser.parse_args()

    payload = parse_npz(args.trajectory.read_bytes())
    if args.frames:
        for field in ('base_pos_w', 'base_quat_w', 'joint_pos'):
            payload[field] = payload[field][:args.frames]
    names, rate, positions, joints, quats, _ = read_trajectory(payload, [])
    robot = RobotModel(str(args.urdf), names, terrain_text(args.terrain), lambda _: None, friction=args.friction)
    Q = np.array([robot.configuration(p, r, j) for p, r, j in zip(positions, quats, joints)])
    problem = Problem(robot, Q, [], 1 / rate)
    results = [validate_frame(problem, Q, k) for k in range(len(Q))]
    names = list(results[0][0])
    worst = {name: max(errors[name] for errors, _, _ in results) for name in names}
    print(json.dumps({'frames': len(Q), 'framerate': rate, 'tolerances': robot.tolerances, 'worst': worst}))
    for start in range(0, len(Q), args.window):
        part = results[start:start + args.window]
        print(json.dumps({'frames': [start, start + len(part) - 1],
                          'worst': {name: max(errors[name] for errors, _, _ in part) for name in names},
                          'dynamics_over_tolerance': sum(int(errors['dynamics'] > robot.tolerances['dynamics'])
                                                         for errors, _, _ in part),
                          'surface_intersection': next((message for _, _, message in part if message), None)}))


if __name__ == '__main__':
    main()
