"""Run Trajectory Optimize on a real NPZ and report progress and the result as JSON lines.

Example:
    python scripts/benchmark_trajectory_optimize.py motion.npz robot.urdf --terrain terrain.stl --frames 750
"""
import argparse
import json
import sys
from pathlib import Path
from time import perf_counter

import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_trajectory_editor/backend'))
import trajectory_optimize
from npz_io import parse_npz


def terrain_text(path):
    if path is None:
        return None
    if path.suffix.lower() == '.obj':
        return path.read_text()
    text = trimesh.load(path, force='mesh').export(file_type='obj')
    return text.decode() if isinstance(text, bytes) else text


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('trajectory', type=Path)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('--terrain', type=Path)
    parser.add_argument('--frames', type=int, default=0, help='prefix length; 0 means all frames')
    parser.add_argument('--keyframes', type=int, nargs='*', default=[])
    parser.add_argument('--friction', type=float, default=.7)
    parser.add_argument('--contact-free', action='store_true')
    parser.add_argument('--time-budget', type=float, default=45.)
    parser.add_argument('--max-effort-scale', type=float, default=1.)
    parser.add_argument('--every', type=int, default=10, help='log every N-th evaluation')
    args = parser.parse_args()

    payload = parse_npz(args.trajectory.read_bytes())
    if args.frames:
        for field in ('base_pos_w', 'base_quat_w', 'joint_pos'):
            payload[field] = payload[field][:args.frames]
    print(json.dumps({'trajectory': str(args.trajectory), 'frames': len(payload['base_pos_w']),
                      'framerate': payload['framerate'], 'joints': len(payload['joint_names'])}), flush=True)
    started = perf_counter()
    original = trajectory_optimize.Evaluator.evaluate
    count = [0]

    def evaluate(self, Q, jacobian, data_weight=1., dynamics=True, acceleration_weight=None):
        result = original(self, Q, jacobian, data_weight, dynamics, acceleration_weight)
        if count[0] % args.every == 0:
            worst = {name: max(s[name] for s in result.stats) for name in result.stats[0]}
            print(json.dumps({'evaluation': count[0], 'seconds': round(perf_counter() - started, 2),
                              'data_weight': data_weight, 'dynamics_stage': dynamics, 'cost': result.cost,
                              'worst': worst}), flush=True)
        count[0] += 1
        return result

    trajectory_optimize.Evaluator.evaluate = evaluate
    result = trajectory_optimize.optimize_trajectory(
        payload, args.keyframes, str(args.urdf), terrain_text(args.terrain),
        friction=None if args.contact_free else args.friction,
        time_budget=args.time_budget, max_effort_scale=args.max_effort_scale)
    print(result['report'], flush=True)
    raise SystemExit(0 if result['success'] else 1)


if __name__ == '__main__':
    main()
