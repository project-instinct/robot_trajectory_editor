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

from trajectory_optimize import CONTACT_MARGIN, GAUSS_POINTS, TrajectoryOptimizeError, hermite

_WORKER = {}


class _Feasible(Exception):
    def __init__(self, variables):
        self.variables = variables


def _worker_evaluate(args):
    (intervals, variables, gradient, penalty, dynamics_rho, effort_rho, deadline,
     dynamics_multipliers, effort_multipliers) = args
    objective = _WORKER['objective']
    objective.penalty, objective.deadline = penalty, deadline
    objective.dynamics_rho, objective.effort_rho = dynamics_rho, effort_rho
    objective.dynamics_multipliers = dynamics_multipliers
    objective.effort_multipliers = effort_multipliers
    objective.dynamics_residual = np.zeros_like(objective.dynamics_residual)
    objective.effort_residual = np.zeros_like(objective.effort_residual)
    value, derivatives = objective.evaluate(variables, gradient=gradient, intervals=intervals)
    return (value, derivatives, dict(objective.errors),
            objective.dynamics_residual, objective.effort_residual)


def evaluation_pool(workers):
    """Fork a per-request pool so workers inherit the loaded model and terrain."""
    if workers < 2:
        return None
    try:
        return multiprocessing.get_context('fork').Pool(workers)
    except (ValueError, OSError):
        return None


def evaluate_parallel(objective, pool, chunks, variables, gradient=True):
    results = pool.map(_worker_evaluate,
                       [(chunk, variables, gradient, objective.penalty, objective.dynamics_rho,
                         objective.effort_rho, objective.deadline,
                         objective.dynamics_multipliers, objective.effort_multipliers)
                        for chunk in chunks])
    value = sum(result[0] for result in results)
    derivatives = sum(result[1] for result in results) if gradient else None
    problem = objective.problem
    x, v = problem.unpack(variables)
    position_error = x - problem.reference
    velocity_error = v - objective.velocity_reference
    value += float(np.sum(position_error*position_error)
                   + objective.velocity_weight*np.sum(velocity_error*velocity_error))
    if gradient:
        derivatives += np.r_[2*position_error[problem.free].ravel(),
                             (2*objective.velocity_weight*velocity_error).ravel()]
    objective.errors = {name: max(result[2][name] for result in results)
                        for name in objective.errors}
    objective.dynamics_residual = sum(result[3] for result in results)
    objective.effort_residual = sum(result[4] for result in results)
    return value, derivatives


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
        # Optional per-sample fixed contact schedules inferred once from the
        # smoothed, aligned reference. None keeps the old 1 mm live inference.
        self.schedules = None
        self.fractions = (0., *GAUSS_POINTS, 1.)
        self.sample_count = (problem.n - 1) * len(self.fractions)
        # Augmented-Lagrangian multipliers for the six base-force equalities.
        self.dynamics_rho = 1e3
        self.dynamics_multipliers = np.zeros((self.sample_count, 6))
        self.dynamics_residual = np.zeros((self.sample_count, 6))
        self.effort_rho = 1e3
        self.effort_multipliers = np.zeros((self.sample_count, 2*(problem.d-6)))
        self.effort_residual = np.zeros_like(self.effort_multipliers)
        self.coefficients = [np.array([hermite(*basis, problem.dt, fraction) for basis in np.eye(4)])
                             for fraction in self.fractions]

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
        if intervals is None:
            gx, gv = 2*(x-problem.reference), 2*self.velocity_weight*velocity_error
            loss = float(np.sum((x-problem.reference)**2)
                         + self.velocity_weight*np.sum(velocity_error*velocity_error))
        else:
            # The reference term is global, so the caller adds it once after
            # merging interval-local worker results.
            gx, gv = np.zeros_like(x), np.zeros_like(v)
            loss = 0.
        self.errors = dict.fromkeys(robot.tolerances, 0.)
        force_scale = robot.tolerances['dynamics'] / 1e-3
        frames = range(problem.n - 1) if intervals is None else intervals
        fractions = self.fractions
        sample_of = {fraction: index for index, fraction in enumerate(fractions)}
        for frame in frames:
            for fraction in fractions:
                self.check_time()
                state = hermite(x[frame], x[frame+1], v[frame], v[frame+1], problem.dt, fraction)
                q, qd, _ = state
                sample = frame*len(fractions) + sample_of[fraction]
                schedule = self.schedules[sample] if self.schedules is not None else None
                clearance, clearance_jac = problem.collision(q, penetration_only=True, schedule=schedule)
                torque = (robot.forces(*state) if schedule is None
                          else robot.forces(*state, schedule=schedule))
                if not np.isfinite(torque).all():
                    raise TrajectoryOptimizeError('Non-finite dynamics during Trajectory Optimize.')
                margins = problem.margins(state, clearance, schedule)
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
                base_normalized = base_error / force_scale
                multipliers = self.dynamics_multipliers[sample]
                self.dynamics_residual[sample] = base_error
                effort_error = np.minimum(margins['effort'], 0.)
                effort_normalized = effort_error / force_scale
                effort_multipliers = self.effort_multipliers[sample]
                self.effort_residual[sample] = effort_error
                loss += multipliers @ base_normalized \
                        + .5*self.dynamics_rho*(base_normalized @ base_normalized) \
                        + effort_multipliers @ effort_normalized \
                        + .5*self.effort_rho*(effort_normalized @ effort_normalized)
                if gradient and (np.max(np.abs(base_error)) > .25*robot.tolerances['dynamics'] or
                                 np.max(np.abs(effort_error), initial=0) > .25*robot.tolerances['effort'] or
                                 np.any(multipliers) or np.any(effort_multipliers)):
                    base_weight = (multipliers + self.dynamics_rho*base_normalized) / force_scale
                    effort_alpha = effort_multipliers + self.effort_rho*effort_normalized
                    effort_weight = (-effort_alpha[:joint_count]+effort_alpha[joint_count:]) / force_scale
                    derivatives = robot.support_penalty_gradient(
                        state, np.r_[base_weight, effort_weight], schedule=schedule)
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


def stance_offset(problem, positions, check_time):
    """Fit a C2 base correction to nearby support phases, including seated support.

    Bounding-sphere clearance bounds are safe for collision culling but cannot
    measure a ground-alignment offset. Use exact distances here, and a coarse
    spline so switching the nearest shape cannot inject framewise acceleration.
    This only initializes the trajectory; contacts still use the usual 1 mm gap.
    """
    from scipy.interpolate import BSpline
    from scipy.ndimage import median_filter
    from scipy.linalg import null_space

    distances, normals = [], []
    for position in positions:
        check_time()
        distance, normal = problem.robot.clearances(position, return_normals=True, exact=True)
        distances.append(distance)
        normals.append(normal)
    distances, normals = np.asarray(distances), np.asarray(normals)
    window = min(21, problem.n if problem.n % 2 else problem.n - 1)
    # The window bridges short hovering gaps within a support phase. Only
    # nearby, upward-facing surfaces seed the alignment.
    nearby = median_filter(distances, size=(window, 1), mode='nearest') < .05
    eligible = nearby & (normals[:, :, 2] > .7)
    indices = np.argmin(np.where(eligible, distances, np.inf), axis=1)
    targets = np.zeros(problem.n)
    for frame in np.flatnonzero(eligible.any(axis=1)):
        index = indices[frame]
        targets[frame] = (.5*CONTACT_MARGIN - distances[frame, index]) / normals[frame, index, 2]

    times = np.arange(problem.n) * problem.dt
    degree = min(3, problem.n-1)
    # Put knots on trajectory nodes so adding these positions and analytic
    # velocities reproduces the same spline under Hermite reconstruction.
    stride = max(degree, round(.3 / problem.dt))
    knots = np.r_[np.repeat(times[0], degree+1), times[stride:-1:stride],
                   np.repeat(times[-1], degree+1)]
    spline = BSpline(knots, np.eye(len(knots)-degree-1), degree)
    basis = spline(times)
    fixed = np.setdiff1d(np.arange(problem.n), problem.free)
    # Fit in the null space so fixed poses remain exactly unchanged without
    # multiplying the correction by a frame mask that would break continuity.
    free_basis = null_space(basis[fixed]) if len(fixed) else np.eye(basis.shape[1])
    coefficients = free_basis @ np.linalg.lstsq(basis @ free_basis, targets, rcond=None)[0]
    return basis @ coefficients, spline(times, nu=1) @ coefficients


def solve_contacts(problem, initial, deadline, workers=None, solver=None):
    objective = ContactObjective(problem, deadline)
    solver = solver or os.environ.get('TRAJECTORY_OPTIMIZE_SOLVER', 'penalty')
    if workers is None:
        workers = int(os.environ.get('TRAJECTORY_OPTIMIZE_WORKERS', 0))
    if workers <= 0:
        workers = 6 if problem.robot.terrain_mesh is not None else max(1, min(16, (os.cpu_count() or 1) - 2))
    # Small regression fixtures gain nothing from worker startup.
    workers = min(workers, max(1, problem.n // 50))
    objective.evaluate(initial, gradient=False, stop_on_violation=True)
    if objective.feasible():
        return SimpleNamespace(x=initial, nit=0, success=True)
    pool = None
    chunks = None

    def evaluate(variables, gradient=True):
        if pool is None:
            return objective.evaluate(variables, gradient=gradient)
        return evaluate_parallel(objective, pool, chunks, variables, gradient)
    x, v = problem.unpack(initial)
    # Joint limits apply to the nodal variables, which are independent of the
    # reference positions. Project the initial guess so the search does not
    # start from avoidable violations; fixed keyframes are never touched.
    x[problem.free, 6:] = np.clip(x[problem.free, 6:], problem.robot.lower, problem.robot.upper)
    v[:, 6:] = np.clip(v[:, 6:], -problem.robot.velocity, problem.robot.velocity)
    # Smoothing first gives the local search a lower-acceleration starting
    # point without changing the reference or any fixed frame. Ground alignment
    # then runs on that smoothed path, so smoothing cannot undo the contacts.
    from scipy.signal import savgol_filter
    window = int(os.environ.get('TRAJECTORY_OPTIMIZE_SMOOTH', 21))
    window = min(window, problem.n if problem.n % 2 else problem.n - 1)
    if window >= 5:
        smoothed = savgol_filter(x, window, 2, axis=0)
        x[problem.free] = smoothed[problem.free]
    x[problem.free, 6:] = np.clip(x[problem.free, 6:], problem.robot.lower, problem.robot.upper)
    v = np.gradient(x, problem.dt, axis=0, edge_order=2 if problem.n > 2 else 1)
    offsets, offset_velocity = stance_offset(problem, x, objective.check_time)
    x[problem.free, 2] += offsets[problem.free]
    v[:, 2] += offset_velocity
    v[:, 6:] = np.clip(v[:, 6:], -problem.robot.velocity, problem.robot.velocity)
    variables = problem.pack(x, v)
    # Infer the fixed search schedule from the smoothed, aligned reference
    # once, at exactly the collocation fractions the objective samples. This
    # is a search model only: final validation still requires live 1 mm
    # contact inference, non-penetration, and the usual effort/dynamics bounds.
    if os.environ.get('TRAJECTORY_OPTIMIZE_CONTACT_SCHEDULE', '1') != '0':
        problem.robot.contact_geometry.clear()
        problem.robot._contact_request.clear()
        problem.robot._force_cache = None
        problem._collisions.clear()
        schedules = []
        for frame in range(problem.n - 1):
            for fraction in objective.fractions:
                objective.check_time()
                state = hermite(x[frame], x[frame+1], v[frame], v[frame+1], problem.dt, fraction)
                schedules.append(problem.robot.infer_contact_schedule(state[0]))
        objective.schedules = schedules
    # Fork only after the schedule exists: workers inherit the objective, and
    # mutating it in the parent afterwards would not reach them.
    _WORKER['objective'] = objective
    pool = evaluation_pool(workers)
    chunks = np.array_split(np.arange(problem.n - 1), workers) if pool is not None else None
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

    # The early raw-input probe stopped at its first violation. Score the
    # smoothed candidate itself before comparing it with later iterates.
    evaluate(variables, gradient=False)
    remember(variables)
    force_scale = problem.robot.tolerances['dynamics'] / 1e-3

    def run_stage(maximum):
        nonlocal variables, iterations
        def function(scaled):
            candidate = scaled * scale
            value, gradient = evaluate(candidate)
            if remember(candidate) <= 1:
                raise _Feasible(candidate)
            return value, gradient * scale

        try:
            result = minimize(function, variables / scale, jac=True, method='L-BFGS-B',
                              options={'maxiter': maximum, 'maxls': 20,
                                       'ftol': 1e-12, 'gtol': 1e-7})
        except _Feasible as found:
            return found.variables
        iterations += result.nit
        variables = result.x * scale
        evaluate(variables, gradient=False)
        if remember(variables) <= 1:
            return variables
        return None

    if solver == 'augmented':
        rho, previous = 1e3, np.inf
        for _ in range(int(os.environ.get('TRAJECTORY_OPTIMIZE_OUTERS', 8))):
            objective.dynamics_rho = objective.effort_rho = rho
            objective.penalty = rho / 2
            found = run_stage(int(os.environ.get('TRAJECTORY_OPTIMIZE_MAXITER', 10)))
            if found is not None:
                if pool is not None:
                    pool.terminate()
                problem.robot.contact_geometry.clear()
                problem.robot._contact_request.clear()
                problem.robot._force_cache = None
                problem._collisions.clear()
                return SimpleNamespace(x=found, nit=iterations, success=True)
            residual = objective.dynamics_residual / force_scale
            effort_residual = objective.effort_residual / force_scale
            objective.dynamics_multipliers += rho * residual
            objective.effort_multipliers = np.maximum(
                0., objective.effort_multipliers + rho * effort_residual)
            worst = max(float(np.max(np.abs(residual), initial=0)),
                        float(np.max(np.abs(effort_residual), initial=0)))
            if worst > .5 * previous:
                rho = min(rho * 10, 1e7)
            previous = worst
    else:
        objective.dynamics_multipliers[:] = 0.
        objective.effort_multipliers[:] = 0.
        for penalty in (1e3, 1e5, 1e7):
            objective.dynamics_rho = objective.effort_rho = 2 * penalty
            objective.penalty = penalty
            found = run_stage(int(os.environ.get('TRAJECTORY_OPTIMIZE_MAXITER', 40)))
            if found is not None:
                if pool is not None:
                    pool.terminate()
                problem.robot.contact_geometry.clear()
                problem.robot._contact_request.clear()
                problem.robot._force_cache = None
                problem._collisions.clear()
                return SimpleNamespace(x=found, nit=iterations, success=True)
    if pool is not None:
        pool.terminate()
    worst = max(best[2], key=lambda name: best[2][name]/problem.robot.tolerances[name])
    raise TrajectoryOptimizeError(f'Contact optimization failed: {worst} residual {best[2][worst]:.6g} '
                          f'(tolerance {problem.robot.tolerances[worst]:g}). Fixed keyframes were not relaxed.')
