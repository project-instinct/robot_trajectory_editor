"""Whole-trajectory contact-aware optimization with local constraint penalties.

The penalty stages avoid the enormous slack-variable factorization of the
interior-point solver. They are only a search method: success still requires
all model constraints to pass the independent float32 trajectory validation.
"""
import multiprocessing
import os
from time import perf_counter
from types import SimpleNamespace

import numpy as np
from scipy.optimize import minimize

from trajectory_optimize import GAUSS_POINTS, TrajectoryOptimizeError, hermite

_WORKER = {}


class _Feasible(Exception):
    def __init__(self, variables):
        self.variables = variables


def _worker_evaluate(args):
    intervals, variables, gradient, penalty, deadline = args
    objective = _WORKER['objective']
    objective.penalty, objective.deadline = penalty, deadline
    value, derivatives = objective.evaluate(variables, gradient=gradient, intervals=intervals)
    return value, derivatives, dict(objective.errors)


def evaluation_pool(workers):
    """Fork a per-request pool so workers inherit the loaded model and terrain."""
    if workers < 2:
        return None
    try:
        return multiprocessing.get_context('fork').Pool(workers)
    except (ValueError, OSError):
        return None


class ContactObjective:
    def __init__(self, problem, deadline):
        self.problem, self.deadline = problem, deadline
        self.penalty = 1e3
        # Without velocity curvature, a penalty step can move the free nodal
        # velocities by orders of magnitude before the line search reacts.
        # Keeping them near the reference finite differences conditions the
        # search; the output positions remain governed by the same penalties.
        self.velocity_weight = 1.
        self.velocity_reference = np.gradient(problem.reference, problem.dt, axis=0,
                                              edge_order=2 if problem.n > 2 else 1)
        self.errors = {}
        self.coefficients = [np.array([hermite(*basis, problem.dt, fraction) for basis in np.eye(4)])
                             for fraction in (0., *GAUSS_POINTS, 1.)]

    def check_time(self):
        if perf_counter() >= self.deadline:
            worst = max(self.errors, key=lambda name: self.errors[name]/self.problem.robot.tolerances[name],
                        default='dynamics')
            raise TrajectoryOptimizeError(f'Trajectory Optimize time budget exhausted; {worst} residual '
                                  f'{self.errors.get(worst, float("inf")):.6g}. Result was not applied.')

    def feasible(self):
        return all(value <= self.problem.robot.tolerances[name] for name, value in self.errors.items())

    def evaluate(self, variables, gradient=True, stop_on_violation=False, intervals=None):
        problem, robot = self.problem, self.problem.robot
        problem.prepare_collision_cache(variables)
        x, v = problem.unpack(variables)
        velocity_error = v - self.velocity_reference
        gx, gv = 2*(x-problem.reference), 2*self.velocity_weight*velocity_error
        loss = float(np.sum((x-problem.reference)**2)
                     + self.velocity_weight*np.sum(velocity_error*velocity_error))
        self.errors = dict.fromkeys(robot.tolerances, 0.)
        force_scale = robot.tolerances['dynamics'] / 1e-3
        frames = range(problem.n - 1) if intervals is None else intervals
        fractions = (0., *GAUSS_POINTS, 1.)
        for frame in frames:
            for fraction in fractions:
                self.check_time()
                state = hermite(x[frame], x[frame+1], v[frame], v[frame+1], problem.dt, fraction)
                q, qd, _ = state
                clearance, clearance_jac = problem.collision(q, penetration_only=True)
                torque = robot.forces(*state)
                if not np.isfinite(torque).all():
                    raise TrajectoryOptimizeError('Non-finite dynamics during Trajectory Optimize.')
                margins = problem.margins(state, clearance)
                self.errors['dynamics'] = max(self.errors['dynamics'], float(np.max(np.abs(torque[:6]))))
                for name, values in margins.items():
                    if np.isnan(values).any() or np.isneginf(values).any():
                        raise TrajectoryOptimizeError(f'Non-finite {name} constraint during Trajectory Optimize.')
                    self.errors[name] = max(self.errors[name], float(np.max(np.maximum(-values, 0), initial=0)))
                # The initial feasibility probe only needs one counterexample.
                # Scanning the rest of an already-invalid motion wastes the solve budget.
                if stop_on_violation and not self.feasible():
                    return loss, None
                dq, dqd, dqdd = np.zeros(problem.d), np.zeros(problem.d), np.zeros(problem.d)
                joint_count = problem.d-6
                for name, target in (('position', dq), ('velocity', dqd)):
                    violations = np.minimum(margins[name], 0.)
                    loss += self.penalty * (violations @ violations)
                    if name == 'position':
                        target[6:] += 2*self.penalty*(violations[:joint_count]-violations[joint_count:])
                    else:
                        target[6:] += 2*self.penalty*(-violations[:joint_count]+violations[joint_count:])
                penetration = np.minimum(clearance, 0.)
                loss += self.penalty * (penetration @ penetration)
                dq += 2*self.penalty*(penetration @ clearance_jac)
                chart = min(float(margins['orientation'][0]), 0.)
                loss += self.penalty * chart*chart
                dq[3:6] -= 2*self.penalty*chart*q[3:6]/(2*np.pi-.01)
                base_error = torque[:6]
                effort_error = np.minimum(margins['effort'], 0.)
                loss += self.penalty*(base_error @ base_error + effort_error @ effort_error)/force_scale**2
                if gradient and (np.max(np.abs(base_error)) > .25*robot.tolerances['dynamics'] or
                                 np.max(np.abs(effort_error), initial=0) > .25*robot.tolerances['effort']):
                    weights = np.r_[base_error,
                                    -effort_error[:joint_count]+effort_error[joint_count:]]
                    weights *= 2*self.penalty/force_scale**2
                    derivatives = robot.support_penalty_gradient(state, weights)
                    dq += derivatives[0]
                    dqd += derivatives[1]
                    dqdd += derivatives[2]
                if gradient:
                    index = (0, 1, 2, 3)[fractions.index(fraction)]
                    contribution = [cq*dq + cv*dqd + ca*dqdd for cq, cv, ca in self.coefficients[index]]
                    gx[frame] += contribution[0]
                    gx[frame+1] += contribution[1]
                    gv[frame] += contribution[2]
                    gv[frame+1] += contribution[3]
        return loss, np.r_[gx[problem.free].ravel(), gv.ravel()]


def solve_contacts(problem, initial, deadline, workers=None):
    objective = ContactObjective(problem, deadline)
    if workers is None:
        workers = int(os.environ.get('TRAJECTORY_OPTIMIZE_WORKERS', 0))
    if workers <= 0:
        workers = 6 if problem.robot.terrain_mesh is not None else max(1, min(16, (os.cpu_count() or 1) - 2))
    # Small regression fixtures gain nothing from worker startup.
    workers = min(workers, max(1, problem.n // 50))
    objective.evaluate(initial, gradient=False, stop_on_violation=True)
    if objective.feasible():
        return SimpleNamespace(x=initial, nit=0, success=True)
    _WORKER['objective'] = objective
    pool = evaluation_pool(workers)
    chunks = np.array_split(np.arange(problem.n - 1), workers) if pool is not None else None

    def evaluate(variables, gradient=True):
        if pool is None:
            return objective.evaluate(variables, gradient=gradient)
        results = pool.map(_worker_evaluate,
                           [(chunk, variables, gradient, objective.penalty, objective.deadline)
                            for chunk in chunks])
        value = sum(result[0] for result in results)
        derivatives = sum(result[1] for result in results) if gradient else None
        objective.errors = {name: max(result[2][name] for result in results)
                            for name in objective.errors}
        return value, derivatives
    x, v = problem.unpack(initial)
    # Joint limits apply to the nodal variables, which are independent of the
    # reference positions. Project the initial guess so the search does not
    # start from avoidable violations; fixed keyframes are never touched.
    x[problem.free, 6:] = np.clip(x[problem.free, 6:], problem.robot.lower, problem.robot.upper)
    v[:, 6:] = np.clip(v[:, 6:], -problem.robot.velocity, problem.robot.velocity)
    # A geometric warm start for near-support/penetrating frames. This changes
    # only free initial guesses, never the reference or fixed configurations.
    for frame in problem.free:
        objective.check_time()
        distances, jac = problem.collision(x[frame], penetration_only=True)
        index = int(np.argmin(distances))
        distance, normal = distances[index], jac[index, :3]
        if distance < 0 or (distance < .05 and normal[2] > .7):
            norm = normal @ normal
            if norm > 1e-8:
                x[frame, :3] += (1e-6-distance)*normal/norm
        x[frame, 6:] = np.clip(x[frame, 6:], problem.robot.lower, problem.robot.upper)
    v = np.gradient(x, problem.dt, axis=0, edge_order=2 if problem.n > 2 else 1)
    v[:, 6:] = np.clip(v[:, 6:], -problem.robot.velocity, problem.robot.velocity)
    variables = problem.pack(x, v)
    scale = np.r_[np.ones(problem.position_size), np.full(problem.n*problem.d, 1/problem.dt)]
    iterations = 0
    best = None

    def score(errors):
        return max(errors[name] / problem.robot.tolerances[name] for name in errors)

    def remember(variables):
        nonlocal best
        candidate = score(objective.errors)
        if best is None or candidate < best[0]:
            best = candidate, variables.copy(), dict(objective.errors)
        return candidate

    remember(variables)
    for penalty in (1e3, 1e5, 1e7):
        objective.penalty = penalty

        def function(scaled):
            candidate = scaled*scale
            value, gradient = evaluate(candidate)
            if remember(candidate) <= 1:
                raise _Feasible(candidate)
            return value, gradient*scale

        try:
            result = minimize(function, variables/scale, jac=True, method='L-BFGS-B',
                              options={'maxiter': int(os.environ.get('TRAJECTORY_OPTIMIZE_MAXITER', 40)),
                                       'maxls': 20, 'ftol': 1e-12, 'gtol': 1e-7})
        except _Feasible as found:
            if pool is not None:
                pool.terminate()
            return SimpleNamespace(x=found.variables, nit=iterations, success=True)
        iterations += result.nit
        variables = result.x*scale
        evaluate(variables, gradient=False)
        if remember(variables) <= 1:
            if pool is not None:
                pool.terminate()
            return SimpleNamespace(x=variables, nit=iterations, success=True)
    if pool is not None:
        pool.terminate()
    worst = max(best[2], key=lambda name: best[2][name]/problem.robot.tolerances[name])
    raise TrajectoryOptimizeError(f'Contact optimization failed: {worst} residual {best[2][worst]:.6g} '
                          f'(tolerance {problem.robot.tolerances[worst]:g}). Fixed keyframes were not relaxed.')
