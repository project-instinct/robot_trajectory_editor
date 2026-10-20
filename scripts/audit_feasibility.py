"""Audit whether a retargeted trajectory can satisfy the Trajectory Optimize model.

This runs the same collocation samples and support-force allocation as
Trajectory Optimize, but skips the optimizer. It reports the reference residuals and
the contact/actuation margins, which separates "the motion asks for more than
the model allows" from "the optimizer failed to find a feasible motion".
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_trajectory_editor/backend'))
from npz_io import parse_npz
from trajectory_optimize import GAUSS_POINTS, CollocationProblem, RobotDynamics, hermite, read_trajectory


def terrain_text(path):
    if path is None:
        return None
    if path.suffix.lower() == '.obj':
        return path.read_text()
    mesh = trimesh.load(path, force='mesh')
    text = mesh.export(file_type='obj')
    return text.decode() if isinstance(text, bytes) else text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('--terrain', type=Path)
    parser.add_argument('--frames', type=int, default=0, help='prefix length; 0 means all frames')
    parser.add_argument('--friction', type=float, default=.7)
    parser.add_argument('--samples', choices=('node', 'gauss', 'full'), default='full')
    parser.add_argument('--align', action='store_true',
                        help='apply a smoothed ground-alignment offset before auditing')
    parser.add_argument('--smooth', type=int, default=0,
                        help='Savitzky-Golay window (frames) applied before the audit')
    args = parser.parse_args()

    payload = parse_npz(args.trajectory.read_bytes())
    for field in ('base_pos_w', 'base_quat_w', 'joint_pos'):
        if args.frames:
            payload[field] = payload[field][:args.frames]
    names, rate, positions, joints, quats, fixed = read_trajectory(payload, [])
    robot = RobotDynamics(str(args.urdf), names, terrain_text(args.terrain),
                          lambda _: None, friction=args.friction)
    rotations = robot.Rotation.from_quat(np.asarray(quats)[:, [1, 2, 3, 0]]).as_rotvec()
    for i in range(1, len(rotations)):
        angle = np.linalg.norm(rotations[i])
        if angle > 1e-9:
            axis = rotations[i] / angle
            rotations[i] = min([rotations[i] + k*2*np.pi*axis for k in (-1, 0, 1)],
                               key=lambda r: np.linalg.norm(r - rotations[i-1]))
    reference = np.c_[positions, rotations, joints]
    dt = 1 / rate
    from scipy.signal import savgol_filter
    if args.smooth:
        window = args.smooth + 1 - args.smooth % 2
        window = min(window, len(reference) - (1 - len(reference) % 2))
        if window >= 3:
            reference[:, :3] = savgol_filter(reference[:, :3], window, 2, axis=0)
            reference[:, 3:6] = savgol_filter(reference[:, 3:6], window, 2, axis=0)
            reference[:, 6:] = savgol_filter(reference[:, 6:], window, 2, axis=0)
    if args.align:
        offsets = np.zeros((len(reference), 3))
        for frame in range(len(reference)):
            clearance, jacobian = robot.clearances(reference[frame], derivatives=True)
            index = int(np.argmin(clearance))
            distance, normal = clearance[index], jacobian[index, :3]
            if distance < .05 and normal[2] > .7:
                norm = normal @ normal
                if norm > 1e-8:
                    offsets[frame] = (1e-6 - distance) * normal / norm
        window = min(max(11, args.smooth + 1), len(reference) - (1 - len(reference) % 2))
        if window >= 3:
            offsets = savgol_filter(offsets, window, 2, axis=0)
        reference[:, :3] += offsets
        reference[:, 6:] = np.clip(reference[:, 6:], robot.lower, robot.upper)
    problem = CollocationProblem(robot, reference, [], dt)
    velocities = np.gradient(reference, dt, axis=0, edge_order=2 if len(reference) > 2 else 1)
    raw_velocity_violation = float(np.max(np.maximum(
        np.abs(velocities[:, 6:]) - robot.velocity, 0), initial=0))
    position_violation = float(np.max(np.maximum(
        np.r_[robot.lower - reference[:, 6:], reference[:, 6:] - robot.upper], 0), initial=0))
    velocities[:, 6:] = np.clip(velocities[:, 6:], -robot.velocity, robot.velocity)
    fractions = {'node': (0.,), 'gauss': GAUSS_POINTS,
                 'full': (0., *GAUSS_POINTS, 1.)}[args.samples]
    metrics = {'dynamics': [], 'effort': [], 'clearance': []}
    worst = {name: (0., 0.) for name in metrics}
    contacts = 0
    samples = 0
    for frame in range(len(reference) - 1):
        for fraction in fractions:
            state = hermite(reference[frame], reference[frame+1], velocities[frame],
                            velocities[frame+1], dt, fraction)
            clearance = robot.clearances(state[0])
            torque = robot.forces(*state)
            margins = problem.margins(state, clearance)
            values = {
                'dynamics': float(np.max(np.abs(torque[:6]))),
                'effort': max(0., -float(np.min(margins['effort']))),
                'clearance': max(0., -float(np.min(clearance))),
            }
            contact_filter = robot.contact_geometry.get(state[0].tobytes(), ([], [], []))[0]
            contacts += bool(len(contact_filter))
            samples += 1
            for name, value in values.items():
                metrics[name].append(value)
                if value > worst[name][0]:
                    worst[name] = (value, frame + fraction)

    def summary(values):
        array = np.asarray(values)
        return {'max': float(array.max(initial=0)),
                'p99': float(np.percentile(array, 99)) if len(array) else 0.,
                'mean': float(array.mean()) if len(array) else 0.}

    report = {
        'trajectory': str(args.trajectory),
        'terrain': str(args.terrain) if args.terrain else 'default z=0 ground',
        'frames': len(reference),
        'framerate': rate,
        'joints': len(names),
        'samples': samples,
        'samples_with_contacts': contacts,
        'samples_without_contacts': samples - contacts,
        'position_violation_reference': position_violation,
        'velocity_violation_raw': raw_velocity_violation,
        'residuals': {name: summary(values) for name, values in metrics.items()},
        'worst_frames': {name: worst[name][1] for name in metrics},
        'tolerances': robot.tolerances,
    }
    report['feasible'] = all(report['residuals'][name]['max'] <= robot.tolerances[name]
                             for name in metrics)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
