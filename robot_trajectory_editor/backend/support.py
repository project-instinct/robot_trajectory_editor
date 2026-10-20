"""Unilateral, friction-limited point support without prescribed contact labels.

A small box-constrained QP finds contact forces and bounded joint torques that
best explain inverse dynamics. Its residual is retained: infeasible support is
never replaced with an arbitrary floating-base wrench.
"""
import numpy as np
from scipy.sparse import csc_matrix, eye
import osqp


def cross(a, b):
    """Cross product without numpy.cross's per-call axis bookkeeping."""
    a, b = np.asarray(a), np.asarray(b)
    if a.ndim == 1 and b.ndim == 1:
        return np.array([a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]])
    a, b = np.atleast_2d(a), np.atleast_2d(b)
    if a.shape[0] == 1:
        a = np.broadcast_to(a, b.shape)
    if b.shape[0] == 1:
        b = np.broadcast_to(b, a.shape)
    return np.column_stack((a[:, 1]*b[:, 2] - a[:, 2]*b[:, 1],
                            a[:, 2]*b[:, 0] - a[:, 0]*b[:, 2],
                            a[:, 0]*b[:, 1] - a[:, 1]*b[:, 0]))


def skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def friction_rays(normal, coefficient):
    """Four generators of a conservative pyramid inside the Coulomb cone."""
    normal = np.asarray(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)
    axis = np.array([1., 0., 0.]) if abs(normal[2]) > .9 else np.array([0., 0., 1.])
    tangent = cross(normal, axis)
    tangent /= np.linalg.norm(tangent)
    second = cross(normal, tangent)
    return np.column_stack([normal + coefficient*tangent, normal - coefficient*tangent,
                            normal + coefficient*second, normal - coefficient*second])


class SupportForces:
    def __init__(self, effort_limits, friction):
        if not np.isfinite(friction) or friction < 0:
            raise ValueError('Terrain friction must be finite and nonnegative.')
        self.effort_limits = np.asarray(effort_limits)
        self.friction = friction
        self.solvers = {}
        self.coefficients = np.empty(0)

    def solve(self, required, point_maps, normals):
        """Return inverse dynamics minus physically admissible contact forces.

        point_maps[k] maps a world force at contact k into generalized forces,
        ordered as the six base forces followed by trajectory joint order.
        Actuator bounds participate in force distribution, so support that
        requires excessive joint torque is not treated as dynamically feasible.
        """
        required = np.asarray(required)
        if not len(point_maps):
            self.coefficients = np.empty(0)
            return required.copy(), np.empty((0, 3))
        rays = [friction_rays(n, self.friction) for n in normals]
        contacts = np.hstack([mapping @ ray for mapping, ray in zip(point_maps, rays)])
        actuators = np.eye(len(required))[:, 6:]
        matrix = np.hstack([contacts, actuators])
        force_count = contacts.shape[1]
        count = matrix.shape[1]
        lower = np.r_[np.zeros(force_count), -self.effort_limits]
        upper = np.r_[np.full(force_count, np.inf), self.effort_limits]
        hessian = matrix.T @ matrix
        hessian.flat[::count+1] += 1e-10
        # Explicit upper-triangular entries preserve the update pattern when a
        # coefficient becomes zero as the contact geometry changes.
        rows, columns = np.triu_indices(count)
        quadratic = csc_matrix((hessian[rows, columns], (rows, columns)), shape=(count, count))
        linear = -matrix.T @ required
        if count not in self.solvers:
            solver = osqp.OSQP()
            # The force residual only needs enough accuracy to drive the outer
            # penalty below the model tolerances; tightening ADMM far beyond
            # that made every dynamics sample several times more expensive.
            solver.setup(P=quadratic, q=linear, A=eye(count, format='csc'), l=lower, u=upper,
                         verbose=False, eps_abs=1e-7, eps_rel=1e-7, max_iter=800,
                         polishing=False, time_limit=.02)
            self.solvers[count] = solver
        else:
            solver = self.solvers[count]
            solver.update(Px=quadratic.data, q=linear, l=lower, u=upper)
        result = solver.solve(raise_error=False)
        if result.x is None or not np.isfinite(result.x).all():
            raise ValueError(f'Support-force solve failed: {result.info.status}')
        # Enforce the cone even when the inner QP stops with finite precision.
        coefficients = np.maximum(result.x[:force_count], 0.)
        self.coefficients = coefficients.copy()
        forces = np.array([ray @ coefficients[4*i:4*i+4] for i, ray in enumerate(rays)])
        return required - contacts @ coefficients, forces
