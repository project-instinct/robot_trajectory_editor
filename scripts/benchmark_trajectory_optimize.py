"""Benchmark Trajectory Optimize on a fixed prefix of a real NPZ (run on the compute host).

Use an external wall-clock limit, e.g. timeout 180s python ... . Progress is
flushed after each stage so a timed-out solve still leaves useful measurements.
"""
import argparse
import json
from pathlib import Path
import resource
import sys
import traceback
from time import perf_counter

import numpy as np
import scipy.optimize
import trimesh
from scipy.optimize._numdiff import approx_derivative

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_trajectory_editor/backend'))
from npz_io import parse_npz
from trajectory_optimize import optimize_trajectory


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
    parser.add_argument('--frames', type=int, default=750)
    parser.add_argument('--mode', choices=('constraints', 'solve'), default='constraints')
    parser.add_argument('--iterations', type=int, default=20)
    parser.add_argument('--friction', type=float, default=.7)
    parser.add_argument('--contact-free', action='store_true')
    parser.add_argument('--time-budget', type=float, default=45.)
    parser.add_argument('--global-worst', action='store_true',
                        help='log the best parent-merged contact residual and its worst sample')
    args = parser.parse_args()
    payload = parse_npz(args.trajectory.read_bytes())
    for field in ('base_pos_w', 'base_quat_w', 'joint_pos'):
        payload[field] = payload[field][:args.frames]
    print(json.dumps({'trajectory': str(args.trajectory), 'frames': len(payload['base_pos_w']),
                      'framerate': payload['framerate'], 'joints': len(payload['joint_names']),
                      'mode': args.mode, 'friction': None if args.contact_free else args.friction}), flush=True)
    original = scipy.optimize.minimize
    started = perf_counter()

    def measure(name, function):
        start = perf_counter()
        value = function()
        print(json.dumps({'stage': name, 'seconds': perf_counter()-start,
                          'shape': list(value.shape), 'nnz': getattr(value, 'nnz', None),
                          'peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024}), flush=True)
        return value

    def minimize(function, initial, **kwargs):
        if not args.contact_free:
            kwargs['options']['maxiter'] = args.iterations
            return original(function, initial, **kwargs)
        problem = function.__self__
        print(json.dumps({'stage': 'setup', 'seconds': perf_counter()-started,
                          'variables': len(initial)}), flush=True)
        if args.mode == 'constraints':
            for name in ('equalities', 'inequalities'):
                f = getattr(problem, name)
                baseline = measure(name, lambda: f(initial))
                jac = kwargs['constraints'][0 if name == 'equalities' else 1].jac
                if callable(jac):
                    measure(name + '_jacobian', lambda: jac(initial))
                else:
                    pattern = problem.constraint_sparsity(len(baseline) // (problem.n-1))
                    measure(name + '_jacobian', lambda: approx_derivative(f, initial, method='2-point',
                                                                         f0=baseline, sparsity=pattern))
            raise SystemExit(0)
        kwargs['options']['maxiter'] = args.iterations
        if kwargs['method'] == 'trust-constr':
            def callback(x, state):
                print(json.dumps({'iteration': state.nit, 'seconds': perf_counter()-started,
                                  'objective': float(state.fun), 'constraint_violation': float(state.constr_violation)}), flush=True)
            kwargs['callback'] = callback
        result = original(function, initial, **kwargs)
        print(json.dumps({'stage': 'solver', 'seconds': perf_counter()-started,
                          'success': bool(result.success), 'iterations': int(result.nit),
                          'message': str(result.message)}), flush=True)
        return result

    scipy.optimize.minimize = minimize
    if not args.contact_free:
        import trajectory_optimize_contacts
        original_contacts = trajectory_optimize_contacts.solve_contacts

        def contact_solve(problem, initial, deadline):
            print(json.dumps({'stage': 'setup', 'seconds': perf_counter()-started,
                              'variables': len(initial)}), flush=True)
            if args.mode == 'constraints':
                objective = trajectory_optimize_contacts.ContactObjective(problem, deadline)
                measure('contact_values', lambda: np.atleast_1d(objective.evaluate(initial, gradient=False)[0]))
                print(json.dumps({'residuals': objective.errors}), flush=True)
                measure('contact_gradient', lambda: objective.evaluate(initial)[1])
                raise SystemExit(0)
            original_evaluate = trajectory_optimize_contacts.ContactObjective.evaluate
            def evaluated(objective, variables, gradient=True, **options):
                start = perf_counter()
                value = original_evaluate(objective, variables, gradient, **options)
                print(json.dumps({'stage': 'contact_evaluation', 'gradient': gradient,
                                  'seconds': perf_counter()-start, 'elapsed': perf_counter()-started,
                                  'objective': float(value[0]), 'residuals': objective.errors}), flush=True)
                return value
            trajectory_optimize_contacts.ContactObjective.evaluate = evaluated
            if args.global_worst:
                original_parallel = trajectory_optimize_contacts.evaluate_parallel
                best_score = float('inf')
                best_dynamics = float('inf')

                def merged(objective, pool, chunks, variables, gradient=True):
                    nonlocal best_score, best_dynamics
                    result = original_parallel(objective, pool, chunks, variables, gradient)
                    score = max(objective.errors[name] / problem.robot.tolerances[name]
                                for name in objective.errors)
                    dynamics = objective.errors['dynamics']
                    improved = []
                    if score < best_score:
                        best_score = score
                        improved.append('score')
                    if dynamics < best_dynamics:
                        best_dynamics = dynamics
                        improved.append('dynamics')
                    if improved:
                        sample, axis = np.unravel_index(
                            np.abs(objective.dynamics_residual).argmax(),
                            objective.dynamics_residual.shape)
                        frame, offset = divmod(sample, len(objective.fractions))
                        fraction = objective.fractions[offset]
                        x, v = problem.unpack(variables)
                        q = trajectory_optimize_contacts.hermite(
                            x[frame], x[frame+1], v[frame], v[frame+1], problem.dt, fraction)[0]
                        problem.robot.clearances(q)
                        contacts = len(problem.robot.contact_geometry[q.tobytes()][0])
                        scheduled = (len(objective.schedules[sample])
                                     if objective.schedules is not None else None)
                        print(json.dumps({'stage': 'global_best', 'elapsed': perf_counter()-started,
                                          'improved': improved, 'score': score,
                                          'residuals': objective.errors,
                                          'worst_dynamics': {'value': float(np.abs(
                                              objective.dynamics_residual[sample, axis])),
                                              'frame': int(frame), 'fraction': float(fraction),
                                              'axis': int(axis), 'contacts': contacts,
                                              'scheduled': scheduled}}), flush=True)
                    return result

                trajectory_optimize_contacts.evaluate_parallel = merged
            return original_contacts(problem, initial, deadline)

        trajectory_optimize_contacts.solve_contacts = contact_solve
    try:
        result = optimize_trajectory(payload, [], str(args.urdf), terrain_text(args.terrain),
                                    friction=None if args.contact_free else args.friction,
                                    time_budget=args.time_budget)
        print(json.dumps({'stage': 'validated', 'seconds': perf_counter()-started,
                          'residuals': result['residuals']}), flush=True)
    except Exception as error:
        traceback.print_exc()
        print(json.dumps({'stage': 'failed', 'seconds': perf_counter()-started,
                          'error': str(error)}), flush=True)
        raise SystemExit(1)


if __name__ == '__main__':
    main()
