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

from trajectory_optimize import (CONTACT_MARGIN, GAUSS_POINTS, SEARCH_CLEARANCE, TrajectoryOptimizeError,
                                 hermite)

_WORKER = {}


class _Feasible(Exception):
    def __init__(self, variables):
        self.variables = variables


class _Deadline(Exception):
    pass


def _worker_evaluate(args):
    (intervals, variables, gradient, penalty, dynamics_rho, effort_rho, deadline,
     dynamics_multipliers, effort_multipliers, live) = args
    objective = _WORKER['objective']
    objective.penalty, objective.deadline = penalty, deadline
    objective.dynamics_rho, objective.effort_rho = dynamics_rho, effort_rho
    objective.dynamics_multipliers = dynamics_multipliers
    objective.effort_multipliers = effort_multipliers
    # Search uses the fixed schedule; restoration can switch these workers to
    # the live 1 mm inference that the final validator applies.
    objective.schedules = None if live else _WORKER.get('schedules', objective.schedules)
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


def evaluate_parallel(objective, pool, chunks, variables, gradient=True, live=False):
    results = pool.map(_worker_evaluate,
                       [(chunk, variables, gradient, objective.penalty, objective.dynamics_rho,
                         objective.effort_rho, objective.deadline,
                         objective.dynamics_multipliers, objective.effort_multipliers, live)
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
        # Velocities are the finite differences of the positions. Tracking the
        # reference ones, scaled to per-frame displacement, damps framewise
        # zigzags without outweighing the position term.
        self.velocity_weight = problem.dt**2
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
                if schedule is not None:
                    clearance = clearance - SEARCH_CLEARANCE
                torque = (robot.forces(*state) if schedule is None
                          else robot.forces(*state, schedule=schedule))
                if not np.isfinite(torque).all():
                    raise TrajectoryOptimizeError('Non-finite dynamics during Trajectory Optimize.')
                margins = problem.margins(state, clearance, schedule)
                # The validator only accepts support within CONTACT_MARGIN.
                # A scheduled contact that carries load while hovering is a
                # violation, weighted by the share of body weight it carries.
                hovering = []
                if schedule:
                    entry = robot.contact_geometry[q.tobytes()]
                    coefficients = robot._support_coefficients
                    for contact, index in enumerate(entry[2]):
                        load = float(np.sum(coefficients[4*contact:4*contact+4])) / force_scale
                        if load <= 1e-6 or entry[4][contact] <= 0.:
                            continue
                        distance, distance_gradient = robot.geometry_distance(q, index)
                        # Half the margin leaves room for the curve between
                        # the search samples and the denser validation ones.
                        hover = distance - .5*CONTACT_MARGIN
                        if hover > 0.:
                            hovering.append((load, hover, distance_gradient))
                            self.errors['clearance'] = max(self.errors['clearance'], hover)
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
                for load, hover, distance_gradient in hovering:
                    loss += self.penalty * load * hover*hover
                    dq += 2*self.penalty*load*hover*distance_gradient
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


def fixed_frame_taper(problem, width):
    """Per-frame weight that is 0 at fixed frames and rises C1-smoothly to 1.

    Corrections multiplied by it leave fixed poses untouched without the kink
    a hard mask would put between a fixed frame and its corrected neighbour.
    """
    fixed = np.setdiff1d(np.arange(problem.n), problem.free)
    if not len(fixed):
        return np.ones(problem.n)
    distance = np.min(np.abs(np.arange(problem.n)[:, None] - fixed[None, :]), axis=1)
    s = np.clip(distance / max(width, 1), 0., 1.)
    return s*s*(3 - 2*s)


def smooth_free(problem, x, window):
    """Savitzky-Golay smoothing that fades out toward the fixed frames."""
    from scipy.signal import savgol_filter
    smoothed = savgol_filter(x, window, 2, axis=0)
    weight = fixed_frame_taper(problem, window)
    result = x.copy()
    result[problem.free] += weight[problem.free, None] * (smoothed - x)[problem.free]
    return result


def stance_offset(problem, positions, check_time):
    """Fit a smooth vertical base correction to nearby support phases, including seated support.

    Bounding-sphere clearance bounds are safe for collision culling but cannot
    measure a ground-alignment offset. Use exact distances here, and a coarse
    spline so switching the nearest shape cannot inject framewise acceleration.
    Fixed frames keep a zero offset; a hovering fixed pose is joined to the
    supported fit by a ballistic fall or rise. This only initializes the
    trajectory; contacts still use the usual 1 mm gap.
    """
    from scipy.interpolate import BSpline
    from scipy.ndimage import median_filter

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
    stride = max(degree, round(.3 / problem.dt))
    knots = np.r_[np.repeat(times[0], degree+1), times[stride:-1:stride],
                   np.repeat(times[-1], degree+1)]
    spline = BSpline(knots, np.eye(len(knots)-degree-1), degree)
    basis = spline(times)
    offset = basis @ np.linalg.lstsq(basis, targets, rcond=None)[0]
    fixed = np.setdiff1d(np.arange(problem.n), problem.free)
    # A fixed pose cannot move. Where the fit would lift one, fade the lift
    # out smoothly toward it.
    lift = fixed[offset[fixed] > 0]
    if len(lift):
        width = stride
        for frame in lift:
            s = np.clip(np.abs(np.arange(problem.n) - frame) / width, 0., 1.)
            offset -= offset[frame] * (1 - s*s*(3 - 2*s))
    # Where the fit would lower a fixed pose, that pose hovers and the body is
    # airborne there: it falls ballistically from the pose (or rises into it)
    # until it meets the supported fit, as a drop or a hop would.
    gravity = abs(float(problem.robot.model.gravity.linear[2])) or 9.81
    for frame in fixed[offset[fixed] < 0]:
        offset = np.maximum(offset, -.5*gravity*(times - times[frame])**2)
    offset[fixed] = 0.
    return offset


def _spline_correction(problem, targets, seconds=.3):
    """Fit a smooth, nonnegative base correction to per-node lift targets.

    The correction is a C2 cubic B-spline with knots on trajectory nodes, so
    adding its analytic velocities keeps Hermite reconstruction free of the
    framewise acceleration spikes a direct position shift would inject.
    """
    from scipy.interpolate import BSpline
    from scipy.linalg import null_space
    from scipy.optimize import lsq_linear
    times = np.arange(problem.n) * problem.dt
    degree = min(3, problem.n-1)
    stride = max(degree, round(seconds / problem.dt))
    knots = np.r_[np.repeat(times[0], degree+1), times[stride:-1:stride],
                  np.repeat(times[-1], degree+1)]
    spline = BSpline(knots, np.eye(len(knots)-degree-1), degree)
    basis = spline(times)
    fixed = np.setdiff1d(np.arange(problem.n), problem.free)
    if len(fixed):
        # Zero correction at fixed poses without multiplying by a frame mask
        # that would break the spline's continuity.
        free_basis = null_space(basis[fixed])
        coefficients = free_basis @ np.linalg.lstsq(basis @ free_basis, targets, rcond=None)[0]
    else:
        # A nonnegative B-spline basis keeps the whole correction nonnegative,
        # so lifting penetrating samples cannot push another phase underground.
        coefficients = lsq_linear(basis, targets, bounds=(0., np.inf), tol=1e-10).x
    return basis @ coefficients, spline(times, nu=1) @ coefficients


def penetration_lift(problem, x, v, check_time, target=.5*CONTACT_MARGIN):
    """Vertical base offset needed to clear the terrain at dense samples."""
    lift = np.zeros(problem.n)
    for frame in range(problem.n-1):
        check_time()
        for fraction in np.linspace(0., 1., 5):
            state = hermite(x[frame], x[frame+1], v[frame], v[frame+1], problem.dt, fraction)
            distances, normals = problem.robot.clearances(state[0], return_normals=True)
            needed = 0.
            for distance, normal in zip(distances, normals):
                if normal[2] > .2:
                    needed = max(needed, (target-distance)/normal[2])
            if needed > 0.:
                lift[frame] = max(lift[frame], needed)
                lift[frame+1] = max(lift[frame+1], needed)
    return lift


def repair_penetration(problem, variables, deadline):
    """Project a candidate out of the terrain with a low-frequency base lift."""
    x, v = problem.unpack(variables)

    def check_time():
        if perf_counter() >= deadline:
            raise _Deadline

    for _ in range(3):
        lift = penetration_lift(problem, x, v, check_time)
        if not np.any(lift > 0.):
            break
        offset, _ = _spline_correction(problem, lift)
        x[problem.free, 2] += offset[problem.free]
        variables = problem.consistent(x)
        x, v = problem.unpack(variables)
    return variables


def solve_contacts(problem, initial, deadline, workers=None, solver=None):
    objective = ContactObjective(problem, deadline)
    solver = solver or os.environ.get('TRAJECTORY_OPTIMIZE_SOLVER', 'penalty')
    if workers is None:
        workers = int(os.environ.get('TRAJECTORY_OPTIMIZE_WORKERS', 0))
    if workers <= 0:
        workers = 6 if problem.robot.terrain_mesh is not None else max(1, min(16, (os.cpu_count() or 1) - 2))
    # Small regression fixtures gain nothing from worker startup.
    workers = min(workers, max(1, problem.n // 50))
    # Leave room for the restoration and the independent final validation
    # from the start, so a slow setup cannot consume the whole budget.
    reserve = float(os.environ.get('TRAJECTORY_OPTIMIZE_RESTORE_RESERVE', 0) or 0.)
    if reserve <= 0.:
        reserve = min(20., max(8., .5 * (deadline - perf_counter())))
    objective.deadline = deadline - reserve
    pool = None
    chunks = None
    # The best candidate known so far; returned as a failed preview if the
    # budget runs out before the search has scored anything.
    variables = initial

    def setup_timeout():
        if pool is not None:
            pool.terminate()
        problem.robot.contact_geometry.clear()
        problem.robot._contact_request.clear()
        problem.robot._force_cache = None
        problem._collisions.clear()
        return SimpleNamespace(x=variables, nit=0, success=False,
                               message='Trajectory Optimize time budget exhausted before the contact search '
                                       'started; showing the smoothed, ground-aligned initial candidate.')

    try:
        objective.evaluate(initial, gradient=False, stop_on_violation=True)
    except TrajectoryOptimizeError as error:
        if 'time budget exhausted' not in str(error):
            raise
        return setup_timeout()
    if objective.feasible() or problem.position_size == 0:
        # With every frame fixed there is nothing to choose; the final
        # validation reports whether the given trajectory is feasible.
        return SimpleNamespace(x=initial, nit=0, success=True)

    def evaluate(variables, gradient=True, live=False):
        if pool is None:
            if not live:
                return objective.evaluate(variables, gradient=gradient)
            schedules = objective.schedules
            objective.schedules = None
            try:
                return objective.evaluate(variables, gradient=gradient)
            finally:
                objective.schedules = schedules
        return evaluate_parallel(objective, pool, chunks, variables, gradient, live)

    # L-BFGS chooses free positions only; every candidate is expanded with
    # the velocities those positions imply, exactly as the validator does.
    tangent = problem.tangent
    iterations = 0
    best = None

    def score(errors):
        return max(errors[name] / problem.robot.tolerances[name] for name in errors)

    # Tolerances decide feasibility, but they are far too uneven (1e-4 Nm of
    # effort against 0.1% body weight of base force) to choose which failed
    # candidate is least wrong. Rank failures against physical scales instead.
    finite_effort = problem.robot.effort[np.isfinite(problem.robot.effort) & (problem.robot.effort > 0)]
    scales = {'dynamics': problem.robot.tolerances['dynamics'] / 1e-3,
              'effort': float(np.median(finite_effort)) if len(finite_effort) else 1.,
              'clearance': 10*CONTACT_MARGIN, 'position': .1, 'velocity': 1., 'orientation': .1}

    def rank(errors):
        return max(errors[name] / scales[name] for name in errors)

    def remember(variables):
        nonlocal best
        candidate = score(objective.errors)
        if candidate <= 1:
            ranked = -1.
        else:
            ranked = rank(objective.errors)
        if best is None or ranked < best[0]:
            best = ranked, variables.copy(), dict(objective.errors)
        return candidate

    try:
        x, _ = problem.unpack(initial)
        # Project the initial guess into the joint limits so the search does not
        # start from avoidable violations; fixed keyframes are never touched.
        x[problem.free, 6:] = np.clip(x[problem.free, 6:], problem.robot.lower, problem.robot.upper)
        # Smoothing first gives the local search a lower-acceleration starting
        # point without changing the reference or any fixed frame. Ground alignment
        # then runs on that smoothed path, so smoothing cannot undo the contacts.
        window = int(os.environ.get('TRAJECTORY_OPTIMIZE_SMOOTH', 21))
        window = min(window, problem.n if problem.n % 2 else problem.n - 1)
        if window >= 5:
            x = smooth_free(problem, x, window)
        x[problem.free, 6:] = np.clip(x[problem.free, 6:], problem.robot.lower, problem.robot.upper)
        offsets = stance_offset(problem, x, objective.check_time)
        x[problem.free, 2] += offsets[problem.free]
        variables = problem.consistent(x)
        x, v = problem.unpack(variables)
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
        _WORKER['schedules'] = objective.schedules
        pool = evaluation_pool(workers)
        chunks = np.array_split(np.arange(problem.n - 1), workers) if pool is not None else None
        # The early raw-input probe stopped at its first violation. Score the
        # smoothed candidate itself before comparing it with later iterates.
        evaluate(variables, gradient=False)
        remember(variables)
    except TrajectoryOptimizeError as error:
        if 'time budget exhausted' not in str(error):
            raise
        return setup_timeout()
    force_scale = problem.robot.tolerances['dynamics'] / 1e-3

    def run_stage(maximum):
        nonlocal variables, iterations
        def function(positions):
            candidate = problem.expand(positions)
            value, gradient = evaluate(candidate)
            if remember(candidate) <= 1:
                raise _Feasible(candidate)
            return value, tangent.T @ gradient

        try:
            result = minimize(function, variables[:problem.position_size], jac=True, method='L-BFGS-B',
                              options={'maxiter': maximum, 'maxls': 20,
                                       'ftol': 1e-12, 'gtol': 1e-7})
        except _Feasible as found:
            return found.variables
        iterations += result.nit
        variables = problem.expand(result.x)
        evaluate(variables, gradient=False)
        if remember(variables) <= 1:
            return variables
        return None

    def finish(variables):
        if pool is not None:
            pool.terminate()
        problem.robot.contact_geometry.clear()
        problem.robot._contact_request.clear()
        problem.robot._force_cache = None
        problem._collisions.clear()
        return SimpleNamespace(x=variables, nit=iterations, success=True)

    # A timeout during the search is a normal stopping condition; the
    # remembered candidate is still repaired below.
    try:
        if solver == 'augmented':
            rho, previous = 1e3, np.inf
            for _ in range(int(os.environ.get('TRAJECTORY_OPTIMIZE_OUTERS', 8))):
                objective.dynamics_rho = objective.effort_rho = rho
                objective.penalty = rho / 2
                found = run_stage(int(os.environ.get('TRAJECTORY_OPTIMIZE_MAXITER', 10)))
                if found is not None:
                    return finish(found)
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
                    return finish(found)
    except TrajectoryOptimizeError as error:
        if 'time budget exhausted' not in str(error):
            raise

    # Feasibility restoration. The search minimizes a sum of penalties against
    # a fixed schedule and can stop on its budget with a candidate that still
    # penetrates or saturates an actuator. Score restoration candidates with
    # the same live 1 mm contact inference as the final validator, then apply
    # only time-continuous corrections: a smooth base lift fitted to the
    # measured penetration depth and whole-trajectory low-pass projections.
    search, search_errors = best[1].copy(), dict(best[2])
    remaining = deadline - perf_counter()
    validation_reserve = min(8., max(3., .2 * remaining))
    restore_deadline = deadline - validation_reserve
    objective.deadline = restore_deadline
    try:
        # Rank every restoration candidate under the live model, starting
        # with the search result itself.
        best = None
        evaluate(search, gradient=False, live=True)
        remember(search)
        for window in (None, 31, 61, 91):
            if perf_counter() >= restore_deadline:
                break
            if window is None:
                candidate = repair_penetration(problem, best[1], restore_deadline)
            else:
                window = min(window, problem.n if problem.n % 2 else problem.n - 1)
                if window < 5:
                    continue
                x, _ = problem.unpack(best[1])
                x = smooth_free(problem, x, window)
                x[problem.free, 6:] = np.clip(x[problem.free, 6:], problem.robot.lower,
                                             problem.robot.upper)
                candidate = repair_penetration(problem, problem.consistent(x), restore_deadline)
            evaluate(candidate, gradient=False, live=True)
            remember(candidate)
    except _Deadline:
        pass
    except TrajectoryOptimizeError as error:
        if 'time budget exhausted' not in str(error):
            raise
    if best is None:
        best = 0., search, search_errors
    if score(best[2]) <= 1.:
        return finish(best[1])
    if pool is not None:
        pool.terminate()
    worst = max(best[2], key=lambda name: best[2][name]/problem.robot.tolerances[name])
    summary = ', '.join(f'{name} {best[2][name]:.6g}' for name in problem.robot.tolerances)
    return SimpleNamespace(x=best[1], nit=iterations, success=False,
                           message=f'Contact optimization failed: {worst} residual {best[2][worst]:.6g} '
                                   f'(tolerance {problem.robot.tolerances[worst]:g}). Best residuals: {summary}. '
                                   f'Fixed keyframes were not relaxed.')
