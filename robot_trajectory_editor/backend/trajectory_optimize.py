"""Fixed-time, floating-base trajectory optimization with optional support forces.

Cubic Hermite configurations are collocated at two Gauss points per interval.
Joint torques are eliminated using inverse dynamics; the six unactuated base
force residuals must vanish after support allocation. Keyframe configurations
never enter the decision vector.
"""
import copy
import io
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


class TrajectoryOptimizeError(ValueError):
    pass


TOLERANCES = {"dynamics": 1e-3, "position": 1e-6, "velocity": 1e-6,
              "effort": 1e-4, "clearance": 1e-5, "orientation": 1e-6}
DENSE_VARIABLE_LIMIT = 500
GAUSS_POINTS = (0.5 - np.sqrt(3) / 6, 0.5 + np.sqrt(3) / 6)
# Contact support is inferred inside this gap; non-penetration itself remains
# much tighter. Retargeted feet routinely sit a fraction of a millimetre above
# the surface after smoothing.
CONTACT_MARGIN = 1e-3
# The fixed search schedule seeds contacts within this proximity, projected
# onto the terrain. It never relaxes the 1 mm live inference used by validation.
CONTACT_SCHEDULE_PROXIMITY = 5e-2
CONTACT_SCHEDULE_NORMAL_Z = .7


def read_trajectory(payload, keyframes):
    names = payload["joint_names"]
    rate = float(payload["framerate"])
    positions = np.asarray(payload["base_pos_w"], dtype=float)
    joints = np.asarray(payload["joint_pos"], dtype=float)
    quats = np.asarray(payload["base_quat_w"], dtype=float)
    n = len(positions)
    if not np.isfinite(rate) or rate <= 0 or n < 2:
        raise TrajectoryOptimizeError("Trajectory Optimize requires at least two frames and a positive finite frame rate.")
    if not isinstance(names, list) or any(not isinstance(v, str) for v in names) or len(set(names)) != len(names):
        raise TrajectoryOptimizeError("Joint names must be unique strings.")
    for name, array, shape in (("base_pos_w", positions, (n, 3)),
                               ("base_quat_w", quats, (n, 4)),
                               ("joint_pos", joints, (n, len(names)))):
        if array.shape != shape or not np.isfinite(array).all():
            raise TrajectoryOptimizeError(f"{name} must contain finite values with shape {shape}.")
    norms = np.linalg.norm(quats, axis=1)
    if np.max(np.abs(norms - 1)) > TOLERANCES["orientation"]:
        raise TrajectoryOptimizeError("Base quaternion norm differs from 1 by more than 1e-6; repair the input first.")
    if not isinstance(keyframes, list) or any(type(k) is not int or k < 0 or k >= n for k in keyframes):
        raise TrajectoryOptimizeError("Keyframes must be integer frame indices within the trajectory.")
    return names, rate, positions, joints, quats, sorted(set(keyframes))


def hermite(q0, q1, v0, v1, dt, s):
    """Configuration, coordinate velocity and acceleration of the same cubic."""
    q = (2*s**3 - 3*s*s + 1)*q0 + (s**3 - 2*s*s + s)*dt*v0 \
        + (-2*s**3 + 3*s*s)*q1 + (s**3 - s*s)*dt*v1
    v = (6*s*s - 6*s)/dt*q0 + (3*s*s - 4*s + 1)*v0 \
        + (-6*s*s + 6*s)/dt*q1 + (3*s*s - 2*s)*v1
    a = (12*s - 6)/dt**2*q0 + (6*s - 4)/dt*v0 \
        + (-12*s + 6)/dt**2*q1 + (6*s - 2)/dt*v1
    return q, v, a


def rotation_kinematics(r, rd, rdd):
    """Body angular velocity/acceleration for exponential rotation coordinates."""
    theta = np.linalg.norm(r)
    cross = np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
    cross_dot = np.array([[0, -rd[2], rd[1]], [rd[2], 0, -rd[0]], [-rd[1], rd[0], 0]])
    dot = r @ rd
    if theta < 1e-4:
        a = 0.5 - theta**2 / 24 + theta**4 / 720
        b = 1/6 - theta**2 / 120 + theta**4 / 5040
        ad = (-1/12 + theta**2 / 180) * dot
        bd = (-1/60 + theta**2 / 1260) * dot
    else:
        a = (1 - np.cos(theta)) / theta**2
        b = (theta - np.sin(theta)) / theta**3
        ad = (theta*np.sin(theta) - 2*(1-np.cos(theta))) / theta**4 * dot
        bd = (theta*(1-np.cos(theta)) - 3*(theta-np.sin(theta))) / theta**5 * dot
    jac = np.eye(3) - a*cross + b*cross@cross
    jac_dot = -ad*cross - a*cross_dot + bd*cross@cross + b*(cross_dot@cross + cross@cross_dot)
    return jac @ rd, jac @ rdd + jac_dot @ rd


class RobotDynamics:
    def __init__(self, urdf_path, names, terrain_obj, resolve_mesh, friction=None):
        try:
            import pinocchio as pin
            import coal
            import trimesh
            from scipy.spatial.transform import Rotation
        except ImportError as error:
            raise TrajectoryOptimizeError("Trajectory Optimize dependencies are missing. Install backend/requirements-trajectory-optimize.txt "
                                  f"in the backend environment ({error}).") from error
        self.pin, self.coal, self.Rotation = pin, coal, Rotation
        root = ET.parse(urdf_path).getroot()
        movable = []
        self.lower, self.upper, self.velocity, self.effort = [], [], [], []
        by_name = {j.attrib["name"]: j for j in root.findall("joint")}
        for joint in by_name.values():
            kind = joint.attrib["type"]
            if kind == "fixed":
                continue
            if kind not in ("revolute", "continuous", "prismatic") or joint.find("mimic") is not None:
                raise TrajectoryOptimizeError(f"Unsupported dynamics joint {joint.attrib['name']}: {kind} or mimic.")
            movable.append(joint.attrib["name"])
            # An unbounded revolute coordinate preserves continuous joint winding.
            if kind == "continuous":
                joint.set("type", "revolute")
                limit = joint.find("limit")
                if limit is None:
                    limit = ET.SubElement(joint, "limit")
                limit.set("lower", "-1e20")
                limit.set("upper", "1e20")
        if set(names) != set(movable):
            raise TrajectoryOptimizeError(f"Trajectory/model joint mismatch: missing {sorted(set(movable)-set(names))}; "
                                  f"unknown {sorted(set(names)-set(movable))}.")
        for name in names:
            limit = by_name[name].find("limit")
            attrs = {} if limit is None else limit.attrib
            for target, key, default in ((self.lower, "lower", -np.inf), (self.upper, "upper", np.inf),
                                         (self.velocity, "velocity", np.inf), (self.effort, "effort", np.inf)):
                target.append(float(attrs.get(key, default)))
            # urdfdom requires these attributes; absent limits remain unbounded
            # in the constraints, rather than silently imposing a guessed value.
            if limit is None:
                limit = ET.SubElement(by_name[name], "limit")
            for key, default in (("lower", "-1e20"), ("upper", "1e20"), ("velocity", "1e20"), ("effort", "1e20")):
                if key not in limit.attrib:
                    limit.set(key, default)
        self.lower, self.upper, self.velocity, self.effort = map(
            np.asarray, (self.lower, self.upper, self.velocity, self.effort))
        if (np.isnan(self.lower).any() or np.isnan(self.upper).any() or
                np.isnan(self.velocity).any() or np.isnan(self.effort).any() or
                np.any(self.lower > self.upper) or np.any(self.velocity < 0) or np.any(self.effort < 0)):
            raise TrajectoryOptimizeError("URDF has invalid joint limits.")
        fixed_links = {joint.find("child").attrib["link"] for joint in by_name.values()
                       if joint.attrib["type"] == "fixed"}
        for link in root.findall("link"):
            # Empty fixed links are coordinate/sensor frames, not physical bodies.
            if (link.attrib["name"] in fixed_links and link.find("inertial") is None
                    and link.find("visual") is None and link.find("collision") is None):
                continue
            if link.find("inertial/mass") is None or link.find("inertial/inertia") is None:
                raise TrajectoryOptimizeError(f"Missing mass or inertia data for link {link.attrib['name']}.")
        for mesh in root.findall(".//mesh"):
            href = mesh.attrib["filename"]
            local = Path(urdf_path).parent / href
            path = str(local.resolve()) if local.is_file() else resolve_mesh(href)
            if not path:
                raise TrajectoryOptimizeError(f"Missing robot mesh: {href}")
            mesh.set("filename", path)
        xml = ET.tostring(root, encoding="unicode")
        try:
            self.model = pin.buildModelFromXML(xml, pin.JointModelFreeFlyer())
            self.geometry = pin.buildGeomFromUrdfString(self.model, xml, pin.GeometryType.COLLISION)
        except Exception as error:
            raise TrajectoryOptimizeError(f"Cannot load dynamics/collision model: {error}") from error
        self.data = self.model.createData()
        self.geometry_data = self.geometry.createData()
        if not self.geometry.geometryObjects:
            raise TrajectoryOptimizeError("The robot has no collision geometry.")
        for inertia in self.model.inertias[1:]:
            eigenvalues = np.linalg.eigvalsh(inertia.inertia)
            if not np.isfinite(inertia.mass) or inertia.mass <= 0 or not np.isfinite(eigenvalues).all() or np.min(eigenvalues) <= 0:
                raise TrajectoryOptimizeError("Robot mass and inertia must be finite and positive for each moving body.")
            if eigenvalues[-1] > eigenvalues[0] + eigenvalues[1] + 1e-10:
                raise TrajectoryOptimizeError("Robot inertia violates the principal-moment triangle inequality.")
        # Float32 positions amplify roundoff when differentiated twice at high
        # frame rates. Validate force/torque residuals at 0.1% of body weight
        # (torque uses a 1 m reference length), with a 1e-3 N/Nm floor.
        weight = sum(inertia.mass for inertia in self.model.inertias) * np.linalg.norm(self.model.gravity.linear)
        self.tolerances = dict(TOLERANCES, dynamics=max(TOLERANCES["dynamics"], 1e-3 * weight))
        self.q_indices = np.array([self.model.joints[self.model.getJointId(n)].idx_q for n in names], dtype=int)
        self.v_indices = np.array([self.model.joints[self.model.getJointId(n)].idx_v for n in names], dtype=int)
        self.terrain_mesh = None
        self.containment = False
        if terrain_obj is None:
            self.terrain = coal.Halfspace(np.array([0., 0., 1.]), 0.)
        else:
            mesh = trimesh.load(io.StringIO(terrain_obj), file_type="obj", force="mesh")
            if len(mesh.faces) == 0 or not np.isfinite(mesh.vertices).all():
                raise TrajectoryOptimizeError("Loaded terrain has no finite triangles.")
            self.terrain_mesh = mesh
            # Winding defines the solid side: a closed mesh with negative
            # volume encloses the free space (a scanned room), so its interior
            # is not solid and must not be treated as penetration.
            self.containment = bool(mesh.is_watertight and float(mesh.volume) > 0)
            self.terrain = coal.BVHModelOBBRSS()
            self.terrain.beginModel(len(mesh.faces), len(mesh.vertices))
            vertices, triangles = coal.StdVec_Vec3s(), coal.StdVec_Triangle()
            for vertex in mesh.vertices:
                vertices.append(vertex)
            for face in mesh.faces:
                triangles.append(coal.Triangle(*(int(index) for index in face)))
            self.terrain.addSubModel(vertices, triangles)
            self.terrain.endModel()
        self.distance_request = coal.DistanceRequest()
        self.distance_request.enable_signed_distance = True
        # Cache each shape's local bounding sphere for broad-phase culling.
        self.geometry_bounds = []
        for geometry in self.geometry.geometryObjects:
            try:
                geometry.geometry.computeLocalAABB()
                center = np.asarray(geometry.geometry.aabb_center, dtype=float)
                radius = float(geometry.geometry.aabb_radius)
                if not np.isfinite(center).all() or not np.isfinite(radius):
                    center, radius = np.zeros(3), np.inf
            except Exception:
                center, radius = np.zeros(3), np.inf
            self.geometry_bounds.append((center, radius))
        self.terrain_grid = None
        if self.terrain_mesh is not None:
            self.terrain_grid = self.terrain_cells(self.terrain_mesh)
            # Build the ray scene once so forked workers share it read-only.
            self.terrain_mesh.contains(np.zeros((1, 3)))
        self.support = None
        if friction is not None:
            from support import SupportForces
            self.support = SupportForces(self.effort, friction)
        self.contact_geometry = {}
        # The presence of an entry alone cannot tell whether a cached mapping
        # used an inferred 1 mm contact or a fixed search schedule.
        self._contact_request = {}
        self._force_cache = None

    @staticmethod
    def terrain_cells(mesh, cell=.5):
        """Coarse terrain AABBs for rigorous broad-phase distance bounds.

        Each triangle contributes its z extent to every XY cell its bounding
        box covers, so the union of the cell AABBs contains the whole terrain.
        """
        low, high = mesh.bounds
        x0, y0 = low[0] - cell, low[1] - cell
        nx = int(np.ceil((high[0] - low[0]) / cell)) + 3
        ny = int(np.ceil((high[1] - low[1]) / cell)) + 3
        zmin = np.full((nx, ny), np.inf)
        zmax = np.full((nx, ny), -np.inf)
        triangles = np.asarray(mesh.vertices, dtype=float)[np.asarray(mesh.faces)]
        i0 = np.clip(np.floor((triangles[:, :, 0].min(1) - x0) / cell).astype(int), 0, nx - 1)
        i1 = np.clip(np.floor((triangles[:, :, 0].max(1) - x0) / cell).astype(int), 0, nx - 1)
        j0 = np.clip(np.floor((triangles[:, :, 1].min(1) - y0) / cell).astype(int), 0, ny - 1)
        j1 = np.clip(np.floor((triangles[:, :, 1].max(1) - y0) / cell).astype(int), 0, ny - 1)
        if (i1 - i0).max() > 8 or (j1 - j0).max() > 8:
            # Very coarse triangles would make the conservative scatter
            # expensive; fall back to exact terrain queries.
            return None
        low_z, high_z = triangles[:, :, 2].min(1), triangles[:, :, 2].max(1)
        for di in range(int((i1 - i0).max()) + 1):
            for dj in range(int((j1 - j0).max()) + 1):
                keep = (i0 + di <= i1) & (j0 + dj <= j1)
                np.minimum.at(zmin, (i0[keep] + di, j0[keep] + dj), low_z[keep])
                np.maximum.at(zmax, (i0[keep] + di, j0[keep] + dj), high_z[keep])
        return x0, y0, cell, zmin, zmax

    def terrain_lower_bound(self, center, radius, margin=1e-3):
        """Rigorous lower bound on a shape's distance to the terrain.

        Returns None when the bound cannot be established from nearby cells.
        """
        if self.terrain_mesh is None:
            return center[2] - radius
        if self.terrain_grid is None:
            return None
        x0, y0, cell, zmin, zmax = self.terrain_grid
        nx, ny = zmin.shape
        half = radius + margin + cell
        i0 = int(np.floor((center[0] - half - x0) / cell))
        i1 = int(np.floor((center[0] + half - x0) / cell))
        j0 = int(np.floor((center[1] - half - y0) / cell))
        j1 = int(np.floor((center[1] + half - y0) / cell))
        if i0 < 0 or j0 < 0 or i1 >= nx or j1 >= ny or i0 > i1 or j0 > j1:
            return None
        best = np.inf
        for i in range(i0, i1 + 1):
            lx, hx = x0 + i*cell, x0 + (i+1)*cell
            dx = max(lx - center[0], 0., center[0] - hx)
            for j in range(j0, j1 + 1):
                if zmax[i, j] < zmin[i, j]:
                    continue
                ly, hy = y0 + j*cell, y0 + (j+1)*cell
                dy = max(ly - center[1], 0., center[1] - hy)
                dz = max(zmin[i, j] - center[2], 0., center[2] - zmax[i, j])
                best = min(best, dx*dx + dy*dy + dz*dz)
        if not np.isfinite(best):
            return None
        return np.sqrt(best) - radius

    def configuration(self, x):
        q = self.pin.neutral(self.model)
        q[:3] = x[:3]
        q[3:7] = self.Rotation.from_rotvec(x[3:6]).as_quat()
        q[self.q_indices] = x[6:]
        return q

    def check_fixed_dynamics(self, reference, fixed, dt):
        if len(fixed) < 3:
            return
        # With no external contact force, the whole-robot COM is ballistic,
        # regardless of internal joint actuation. Incompatible fixed poses
        # cannot be repaired by optimizing intervening frames.
        times = (np.asarray(fixed) - fixed[0]) * dt
        centers = np.array([self.pin.centerOfMass(self.model, self.data, self.configuration(reference[f])).copy()
                            for f in fixed])
        gravity = self.model.gravity.linear
        velocity = (centers[1] - centers[0] - .5*gravity*times[1]**2) / times[1]
        predicted = centers[0] + times[:, None]*velocity + .5*times[:, None]**2*gravity
        errors = np.linalg.norm(centers - predicted, axis=1)
        worst = int(np.argmax(errors))
        if errors[worst] > 1e-5:
            raise TrajectoryOptimizeError(f"Fixed keyframe {fixed[worst]} is incompatible with contact-free dynamics: "
                                  f"center-of-mass ballistic residual {errors[worst]:.6g} m (tolerance 1e-5 m).")

    def kinematics(self, x, xd, xdd):
        q = self.configuration(x)
        rotation = self.Rotation.from_quat(q[3:7]).as_matrix()
        omega, alpha = rotation_kinematics(x[3:6], xd[3:6], xdd[3:6])
        v, a = np.zeros(self.model.nv), np.zeros(self.model.nv)
        v[:3], v[3:6] = rotation.T @ xd[:3], omega
        a[:3], a[3:6] = rotation.T @ xdd[:3] - np.cross(omega, v[:3]), alpha
        v[self.v_indices], a[self.v_indices] = xd[6:], xdd[6:]
        return q, v, a

    @staticmethod
    def contact_request(schedule):
        """Hashable identity of one fixed contact-schedule sample."""
        if schedule is None:
            return None
        return tuple((int(index), *(float(v) for v in point), *(float(v) for v in normal))
                     for index, point, normal in schedule)

    def infer_contact_schedule(self, x, proximity=CONTACT_SCHEDULE_PROXIMITY):
        """Project nearby upward-facing geometry contacts onto the terrain.

        Each entry stores the collision-geometry index, the terrain contact
        point expressed in the parent joint frame, and the world surface
        normal. The local point lets the support mapping follow the link pose
        during the solve, while the schedule itself stays fixed.
        """
        q = self.configuration(x)
        self.pin.updateGeometryPlacements(self.model, self.data, self.geometry, self.geometry_data, q)
        schedule = []
        for index, ((geometry, pose), (local_center, radius)) in enumerate(
                zip(zip(self.geometry.geometryObjects, self.geometry_data.oMg), self.geometry_bounds)):
            center = np.asarray(pose.rotation @ local_center + pose.translation)
            lower = self.terrain_lower_bound(center, radius, margin=proximity)
            if lower is not None and lower > proximity:
                continue
            result = self.coal.DistanceResult()
            distance = self.coal.distance(geometry.geometry,
                                          self.coal.Transform3s(pose.rotation, pose.translation),
                                          self.terrain, self.coal.Transform3s(),
                                          self.distance_request, result)
            nearest = np.asarray(result.getNearestPoint1())
            sign = 1.
            if (self.terrain_mesh is not None and self.containment and distance > 0
                    and self.terrain_mesh.contains(nearest.reshape(1, 3))[0]):
                sign = -1.
            normal = sign * -np.asarray(result.normal)
            if (sign*distance > proximity or not np.isfinite(normal).all()
                    or normal[2] <= CONTACT_SCHEDULE_NORMAL_Z):
                continue
            joint_pose = self.data.oMi[geometry.parentJoint]
            terrain_point = np.asarray(result.getNearestPoint2())
            local_point = joint_pose.rotation.T @ (terrain_point - joint_pose.translation)
            schedule.append((index, local_point, normal))
        return schedule

    def required_forces(self, x, xd, xdd):
        q, v, a = self.kinematics(x, xd, xdd)
        tau = self.pin.rnea(self.model, self.data, q, v, a)
        return np.r_[tau[:6], tau[self.v_indices]]

    def forces(self, x, xd, xdd, schedule=None):
        if self.support is None:
            return self.required_forces(x, xd, xdd)
        request = self.contact_request(schedule)
        key = np.concatenate((x, xd, xdd)).tobytes(), request
        if self._force_cache is not None and self._force_cache[0] == key:
            return self._force_cache[1].copy()
        geometry_key = x.tobytes()
        if (geometry_key not in self.contact_geometry
                or self._contact_request.get(geometry_key) != request):
            self.clearances(x, schedule=schedule)
        mappings, normals = self.contact_geometry[geometry_key][:2]
        adjusted, _ = self.support.solve(self.required_forces(x, xd, xdd), mappings, normals)
        self._support_coefficients = self.support.coefficients.copy()
        self._force_cache = key, adjusted.copy()
        return adjusted

    def coordinate_jacobian(self, x):
        """Map coordinate increments to Pinocchio's local tangent coordinates."""
        transform = np.zeros((self.model.nv, len(x)))
        transform[:3, :3] = self.Rotation.from_rotvec(x[3:6]).as_matrix().T
        for axis in range(3):
            transform[3:6, 3+axis] = rotation_kinematics(x[3:6], np.eye(3)[axis], np.zeros(3))[0]
        transform[self.v_indices, np.arange(6, len(x))] = 1
        return transform

    def force_derivatives(self, x, xd, xdd):
        """RNEA derivatives chained through the floating-base coordinate map.

        Only the three rotation-coordinate derivatives of the small kinematic
        map are differenced; the articulated dynamics derivatives are analytic.
        """
        if self.support is not None:
            if x.tobytes() not in self.contact_geometry:
                self.clearances(x)
            if len(self.contact_geometry[x.tobytes()][0]):
                # Differentiate the constrained support allocation only when a
                # violated dynamics/effort constraint actually needs its gradient.
                state = (x, xd, xdd)
                derivatives = []
                for field in range(3):
                    derivative = np.empty((len(x), len(x)))
                    for axis in range(len(x)):
                        plus, minus = [a.copy() for a in state], [a.copy() for a in state]
                        step = 1e-5 * max(1., abs(state[field][axis]))
                        plus[field][axis] += step
                        minus[field][axis] -= step
                        derivative[:, axis] = (self.forces(*plus)-self.forces(*minus))/(2*step)
                    derivatives.append(derivative)
                return tuple(derivatives)
        return self.required_force_derivatives(x, xd, xdd)

    def required_force_derivatives(self, x, xd, xdd):
        q, v, a = self.kinematics(x, xd, xdd)
        dq, dv, da = self.pin.computeRNEADerivatives(self.model, self.data, q, v, a)
        # Some Pinocchio versions fill only the upper triangle of the mass matrix.
        da = np.triu(da) + np.triu(da, 1).T
        transform = self.coordinate_jacobian(x)
        dx = dq @ transform
        dxd = dv @ transform
        dxdd = da @ transform
        step = 1e-6
        for axis in range(3, 6):
            plus, minus = x.copy(), x.copy()
            plus[axis] += step
            minus[axis] -= step
            _, vp, ap = self.kinematics(plus, xd, xdd)
            _, vm, am = self.kinematics(minus, xd, xdd)
            dx[:, axis] += dv @ ((vp-vm)/(2*step)) + da @ ((ap-am)/(2*step))
        # Acceleration is quadratic in coordinate velocity. A symmetric unit
        # perturbation gives its exact linear derivative without cancellation.
        for axis in range(6):
            plus, minus = xd.copy(), xd.copy()
            plus[axis] += 1
            minus[axis] -= 1
            ap = self.kinematics(x, plus, xdd)[2]
            am = self.kinematics(x, minus, xdd)[2]
            dxd[:, axis] += da @ ((ap-am)/2)
        order = np.r_[np.arange(6), self.v_indices]
        return dx[order], dxd[order], dxdd[order]

    def support_penalty_gradient(self, state, weights, schedule=None):
        """Envelope gradient of the support QP's dynamics/effort residual.

        At its optimum, derivatives of the allocated forces cancel by the KKT
        conditions. Differentiate inverse dynamics and the small active contact
        map, not hundreds of re-solves of the force allocation problem.
        """
        from support import cross, friction_rays, skew
        x, xd, xdd = state
        request = self.contact_request(schedule)
        entry = self.contact_geometry.get(x.tobytes())
        if entry is None or self._contact_request.get(x.tobytes()) != request:
            self.clearances(x, schedule=schedule)
            entry = self.contact_geometry[x.tobytes()]
        coefficients = self._support_coefficients.copy()
        indices = entry[2]
        local_points = entry[3] if len(entry) > 3 else None
        derivatives = self.required_force_derivatives(x, xd, xdd)
        gradient = [weights @ derivative for derivative in derivatives]
        if not len(indices) or not np.any(coefficients):
            return gradient
        pin = self.pin
        q, v, a = self.kinematics(x, xd, xdd)
        pin.computeJointJacobians(self.model, self.data, q)
        transform = self.coordinate_jacobian(x)
        order = np.r_[np.arange(6), self.v_indices]
        joint_weights = np.empty(self.model.nv)
        joint_weights[order] = weights
        external = pin.StdVec_Force([pin.Force.Zero() for _ in range(self.model.njoints)])
        normals = entry[1]
        # The contact wrench depends on six joint-pose coordinates, regardless
        # of the number of robot joints. In the joint's local frame a fixed
        # material contact point and fixed world normal give an exact analytic
        # pose derivative; no distance queries are repeated per coordinate.
        for contact, index in enumerate(indices):
            active_coefficients = coefficients[4*contact:4*contact+4]
            if not np.any(active_coefficients):
                continue
            geometry = self.geometry.geometryObjects[int(index)]
            joint = geometry.parentJoint
            joint_pose = self.data.oMi[joint].copy()
            jac = pin.getJointJacobian(self.model, self.data, joint, pin.LOCAL).copy()
            if local_points is not None:
                # The scheduled contact is a material point carried by the
                # link; its world position and moment arm follow the current
                # pose even when the surface gap changes.
                normal = normals[contact]
                point = joint_pose.rotation @ local_points[contact] + joint_pose.translation
            else:
                geometry_pose = joint_pose * geometry.placement
                result = self.coal.DistanceResult()
                self.coal.distance(geometry.geometry,
                                   self.coal.Transform3s(geometry_pose.rotation, geometry_pose.translation),
                                   self.terrain, self.coal.Transform3s(), self.distance_request, result)
                normal = -np.asarray(result.normal)
                # A body wholly inside a watertight mesh has the opposite
                # signed-distance normal to the unsigned nearest-surface query.
                if normal @ normals[contact] < 0:
                    normal = -normal
                point = result.getNearestPoint1()
            force = friction_rays(normal, self.support.friction) @ active_coefficients
            offset = point - joint_pose.translation
            rotation = joint_pose.rotation
            force_local = rotation.T @ force
            offset_local = rotation.T @ offset
            wrench = np.r_[force_local, cross(offset_local, force_local)]
            external[joint] = external[joint] + pin.Force(wrench)
            # Local pose perturbation (v, omega): the contact is a material
            # point fixed in the link, so both the force and moment simply
            # rotate with the local frame.
            derivative = np.zeros((6, 6))
            derivative[:3, 3:] = skew(force_local)
            if local_points is not None:
                # A scheduled contact is a material point, so its local offset
                # stays constant while the world point rotates with the link.
                derivative[3:, 3:] = (np.outer(force_local, offset_local)
                                      - (offset_local @ force_local) * np.eye(3))
            else:
                # A live nearest point remains on the fixed terrain instead.
                derivative[3:, 3:] = skew(cross(offset_local, force_local))
            gradient[0] -= (jac @ joint_weights) @ derivative @ jac @ transform
        raw_dq = pin.computeRNEADerivatives(self.model, self.data, q, v, a)[0].copy()
        supported_dq = pin.computeRNEADerivatives(self.model, self.data, q, v, a, external)[0]
        gradient[0] += joint_weights @ (supported_dq-raw_dq) @ transform
        return gradient

    def clearances(self, x, derivatives=False, penetration_only=False, return_normals=False, exact=False,
                   schedule=None):
        from support import cross
        q = self.configuration(x)
        self.pin.updateGeometryPlacements(self.model, self.data, self.geometry, self.geometry_data, q)
        if derivatives:
            self.pin.computeJointJacobians(self.model, self.data, q)
            transform = self.coordinate_jacobian(x)
        # Any geometry whose bounding sphere is wholly above the local terrain
        # height cannot touch it. The distance is at least the vertical gap, so
        # this is a safe conservative bound.
        distances, gradients, nearest_points, normals, culled = [], [], [], [], []
        for (geometry, pose), (local_center, radius) in zip(
                zip(self.geometry.geometryObjects, self.geometry_data.oMg), self.geometry_bounds):
            center = np.asarray(pose.rotation @ local_center + pose.translation)
            # Broad-phase skipping is safe for values and for the contact
            # search. Alignment offsets and the contact-free inequality
            # Jacobian need actual distances rather than conservative bounds.
            lower = (self.terrain_lower_bound(center, radius)
                     if not exact and (penetration_only or not derivatives) else None)
            cull = lower is not None and lower > 1e-3
            culled.append(cull)
            if cull:
                distances.append(lower)
                nearest_points.append(np.array([center[0], center[1], 0.]))
                normals.append(np.array([0., 0., 1.]))
                if derivatives:
                    gradients.append(np.zeros(len(x)))
                continue
            result = self.coal.DistanceResult()
            distance = self.coal.distance(geometry.geometry, self.coal.Transform3s(pose.rotation, pose.translation),
                                          self.terrain, self.coal.Transform3s(), self.distance_request, result)
            distances.append(distance)
            nearest_points.append(result.getNearestPoint1().copy())
            normals.append(-np.asarray(result.normal).copy())
            if derivatives and (not penetration_only or distance < 0):
                joint = geometry.parentJoint
                jac = self.pin.getJointJacobian(self.model, self.data, joint, self.pin.LOCAL_WORLD_ALIGNED)
                offset = result.getNearestPoint1() - self.data.oMi[joint].translation
                point_jac = jac[:3] + cross(jac[3:].T, offset).T
                gradients.append(-np.asarray(result.normal) @ point_jac @ transform)
            elif derivatives:
                gradients.append(np.zeros(len(x)))
        distances = np.asarray(distances)
        signs = np.ones(len(distances))
        # Batched containment avoids rebuilding the ray query for every link.
        if self.terrain_mesh is not None and self.containment:
            positive = np.flatnonzero((distances > 0) & ~np.asarray(culled))
            if len(positive):
                inside = self.terrain_mesh.contains(np.asarray(nearest_points)[positive])
                signs[positive[inside]] = -1
        distances *= signs
        if self.support is not None:
            mappings, contact_normals = [], []
            if schedule is None:
                active = np.flatnonzero(distances <= CONTACT_MARGIN)
                if len(active):
                    self.pin.computeJointJacobians(self.model, self.data, q)
                    order = np.r_[np.arange(6), self.v_indices]
                    for index in active:
                        geometry = self.geometry.geometryObjects[int(index)]
                        joint = geometry.parentJoint
                        jac = self.pin.getJointJacobian(self.model, self.data, joint, self.pin.LOCAL_WORLD_ALIGNED)
                        offset = nearest_points[index] - self.data.oMi[joint].translation
                        point_jac = jac[:3] + np.cross(jac[3:].T, offset).T
                        normal = signs[index] * normals[index]
                        if not np.isfinite(normal).all() or np.linalg.norm(normal) < 1e-8:
                            raise TrajectoryOptimizeError(f'Cannot determine contact normal for {geometry.name}.')
                        mappings.append(point_jac.T[order])
                        contact_normals.append(normal)
                entry = mappings, contact_normals, active.copy()
            else:
                # A fixed schedule supplies the support model even when the
                # current link is a few centimetres above the terrain. The
                # mapping still uses the current pose, and zero force remains
                # possible, so lift-off does not need contact removal.
                self.pin.computeJointJacobians(self.model, self.data, q)
                order = np.r_[np.arange(6), self.v_indices]
                active = np.empty(len(schedule), dtype=int)
                local_points = []
                for contact, (index, local_point, normal) in enumerate(schedule):
                    geometry = self.geometry.geometryObjects[int(index)]
                    joint_pose = self.data.oMi[geometry.parentJoint]
                    jac = self.pin.getJointJacobian(self.model, self.data, geometry.parentJoint,
                                                    self.pin.LOCAL_WORLD_ALIGNED)
                    point = joint_pose.rotation @ local_point + joint_pose.translation
                    offset = point - joint_pose.translation
                    point_jac = jac[:3] + np.cross(jac[3:].T, offset).T
                    mappings.append(point_jac.T[order])
                    contact_normals.append(np.asarray(normal))
                    active[contact] = int(index)
                    local_points.append(np.asarray(local_point))
                entry = mappings, contact_normals, active, local_points
            self.contact_geometry[x.tobytes()] = entry
            self._contact_request[x.tobytes()] = self.contact_request(schedule)
        if derivatives:
            return distances, signs[:, None] * np.asarray(gradients)
        if return_normals:
            return distances, signs[:, None] * np.asarray(normals)
        return distances

    def validate_surface_intersections(self):
        # Triangle/triangle distance may be zero for crossing surfaces and
        # therefore does not supply a useful penetration depth. Check collision
        # independently before declaring a loaded-terrain result valid.
        if self.terrain_mesh is None:
            return
        identity = self.coal.Transform3s()
        for (geometry, pose), (local_center, radius) in zip(
                zip(self.geometry.geometryObjects, self.geometry_data.oMg), self.geometry_bounds):
            center = np.asarray(pose.rotation @ local_center + pose.translation)
            lower = self.terrain_lower_bound(center, radius, margin=0.)
            if lower is not None and lower > 0.:
                continue
            result = self.coal.CollisionResult()
            self.coal.collide(geometry.geometry, self.coal.Transform3s(pose.rotation, pose.translation),
                              self.terrain, identity, self.coal.CollisionRequest(), result)
            if result.isCollision():
                raise TrajectoryOptimizeError(f"Terrain surface intersection at robot geometry {geometry.name}.")


class CollocationProblem:
    def __init__(self, robot, reference, keyframes, dt):
        self.robot, self.reference, self.dt = robot, reference.copy(), dt
        self.free = np.array([i for i in range(len(reference)) if i not in keyframes], dtype=int)
        self.n, self.d = reference.shape
        self.position_size = len(self.free) * self.d
        self._collision_variables = None
        self._collisions = {}

    def pack(self, positions, velocities):
        return np.r_[positions[self.free].ravel(), velocities.ravel()]

    def unpack(self, variables):
        positions = self.reference.copy()
        positions[self.free] = variables[:self.position_size].reshape(-1, self.d)
        return positions, variables[self.position_size:].reshape(self.n, self.d)

    def objective(self, variables):
        x, v = self.unpack(variables)
        return np.sum((x - self.reference)**2) + 1e-8 * np.sum(v*v)

    def objective_gradient(self, variables):
        x, v = self.unpack(variables)
        return np.r_[2 * (x - self.reference)[self.free].ravel(), (2e-8 * v).ravel()]

    def constraint_sparsity(self, rows_per_interval):
        from scipy.sparse import coo_matrix
        position_columns = np.full((self.n, self.d), -1, dtype=int)
        position_columns[self.free] = np.arange(self.position_size).reshape(-1, self.d)
        velocity_columns = self.position_size + np.arange(self.n * self.d).reshape(self.n, self.d)
        rows, columns = [], []
        for frame in range(self.n - 1):
            local = np.r_[position_columns[frame:frame+2].ravel(), velocity_columns[frame:frame+2].ravel()]
            local = local[local >= 0]
            rows.append(np.repeat(np.arange(frame*rows_per_interval, (frame+1)*rows_per_interval), len(local)))
            columns.append(np.tile(local, rows_per_interval))
        rows, columns = np.concatenate(rows), np.concatenate(columns)
        return coo_matrix((np.ones(len(rows), dtype=bool), (rows, columns)),
                          shape=((self.n-1)*rows_per_interval, self.position_size+self.n*self.d)).tocsr()

    def samples(self, variables, fractions):
        x, v = self.unpack(variables)
        for i in range(self.n - 1):
            for s in fractions:
                yield i + s, hermite(x[i], x[i+1], v[i], v[i+1], self.dt, s)

    def equalities(self, variables):
        return np.concatenate([self.robot.forces(*state)[:6]
                               for _, state in self.samples(variables, GAUSS_POINTS)])

    def prepare_collision_cache(self, variables):
        # Cache only one candidate. Validation bypasses this cache entirely.
        if self._collision_variables is None or not np.array_equal(variables, self._collision_variables):
            self._collision_variables = variables.copy()
            self._collisions.clear()
            if self.robot.support is not None:
                self.robot.contact_geometry.clear()
                self.robot._contact_request.clear()

    def collision(self, x, penetration_only=False, schedule=None):
        key = (x.tobytes(), penetration_only, RobotDynamics.contact_request(schedule))
        if key not in self._collisions:
            options = dict(derivatives=True, penetration_only=penetration_only)
            if schedule is not None:
                options['schedule'] = schedule
            self._collisions[key] = self.robot.clearances(x, **options)
        return self._collisions[key]

    def margins(self, state, clearance=None, schedule=None):
        x, v, _ = state
        robot = self.robot
        torque = robot.forces(*state) if schedule is None else robot.forces(*state, schedule=schedule)
        torque = torque[6:]
        return {
            "position": np.r_[x[6:] - robot.lower, robot.upper - x[6:]],
            "velocity": np.r_[robot.velocity - v[6:], robot.velocity + v[6:]],
            "effort": np.r_[robot.effort - torque, robot.effort + torque],
            "clearance": robot.clearances(x) if clearance is None else clearance,
            # Keep exponential coordinates away from their 2*pi singularity.
            "orientation": np.array([((2*np.pi - 0.01)**2 - x[3:6] @ x[3:6]) / (2*(2*np.pi - 0.01))]),
        }

    def inequalities(self, variables):
        self.prepare_collision_cache(variables)
        values = [value[~np.isposinf(value)] for _, state in self.samples(variables, (0., *GAUSS_POINTS, 1.))
                  for value in self.margins(state, self.collision(state[0])[0]).values()]
        result = np.concatenate(values)
        if not np.isfinite(result).all():
            raise TrajectoryOptimizeError("Non-finite optimization constraint.")
        return result

    def constraint_jacobian(self, variables, equality):
        """Assemble interval-local state derivatives into the global sparse Jacobian."""
        from scipy.sparse import coo_matrix
        if not equality:
            self.prepare_collision_cache(variables)
        x, v = self.unpack(variables)
        position_columns = np.full((self.n, self.d), -1, dtype=int)
        position_columns[self.free] = np.arange(self.position_size).reshape(-1, self.d)
        velocity_columns = self.position_size + np.arange(self.n*self.d).reshape(self.n, self.d)
        rows, columns, values = [], [], []
        row_offset = 0
        fractions = GAUSS_POINTS if equality else (0., *GAUSS_POINTS, 1.)
        eye = np.eye(self.d)
        for frame in range(self.n-1):
            local_columns = np.r_[position_columns[frame], position_columns[frame+1],
                                   velocity_columns[frame], velocity_columns[frame+1]]
            for fraction in fractions:
                state = hermite(x[frame], x[frame+1], v[frame], v[frame+1], self.dt, fraction)
                fx, fv, fa = self.robot.force_derivatives(*state)
                if equality:
                    dx, dv, da = fx[:6], fv[:6], fa[:6]
                else:
                    clearance = self.collision(state[0])[1]
                    orientation = np.zeros((1, self.d))
                    orientation[0, 3:6] = -state[0][3:6] / (2*np.pi - .01)
                    dx = np.vstack((eye[6:], -eye[6:], np.zeros((2*(self.d-6), self.d)),
                                    -fx[6:], fx[6:], clearance, orientation))
                    dv = np.vstack((np.zeros((2*(self.d-6), self.d)), -eye[6:], eye[6:],
                                    -fv[6:], fv[6:], np.zeros((len(clearance)+1, self.d))))
                    da = np.vstack((np.zeros((4*(self.d-6), self.d)), -fa[6:], fa[6:],
                                    np.zeros((len(clearance)+1, self.d))))
                    limits = np.r_[self.robot.lower, self.robot.upper, self.robot.velocity, self.robot.velocity,
                                   self.robot.effort, self.robot.effort, np.zeros(len(clearance)+1)]
                    keep = np.isfinite(limits)
                    dx, dv, da = dx[keep], dv[keep], da[keep]
                # Hermite is linear in its four endpoint vectors. Its scalar
                # coefficients chain all state derivatives without perturbing
                # unrelated intervals or reevaluating the collision scene.
                coefficients = np.array([hermite(*basis, self.dt, fraction) for basis in np.eye(4)])
                block = np.hstack([cq*dx + cv*dv + ca*da for cq, cv, ca in coefficients])
                ri, ci = np.nonzero(block)
                keep = local_columns[ci] >= 0
                rows.append(ri[keep] + row_offset)
                columns.append(local_columns[ci[keep]])
                values.append(block[ri[keep], ci[keep]])
                row_offset += len(dx)
        return coo_matrix((np.concatenate(values), (np.concatenate(rows), np.concatenate(columns))),
                          shape=(row_offset, len(variables))).tocsr()

    def validate(self, variables, deadline=None):
        tolerances = self.robot.tolerances
        residuals = dict.fromkeys(tolerances, 0.)
        worst = ""
        largest_ratio = 0.
        # Independent dense sampling, including both one-sided endpoint
        # accelerations. This is a numerical check, not a continuous-time proof.
        for frame, state in self.samples(variables, np.linspace(0, 1, 17)):
            if deadline is not None:
                from time import perf_counter
                if perf_counter() >= deadline:
                    raise TrajectoryOptimizeError('Trajectory Optimize time budget exhausted during validation; result was not applied.')
            errors = {"dynamics": np.max(np.abs(self.robot.forces(*state)[:6]))}
            for name, values in self.margins(state).items():
                minimum = float(np.min(values)) if len(values) else np.inf
                if np.isnan(minimum) or minimum == -np.inf:
                    raise TrajectoryOptimizeError(f"Non-finite {name} constraint near frame {frame:.4f}.")
                errors[name] = max(0., -minimum)
            try:
                self.robot.validate_surface_intersections()
            except TrajectoryOptimizeError as error:
                raise TrajectoryOptimizeError(f"Validation failed near frame {frame:.4f}: {error}") from error
            for name, error in errors.items():
                if not np.isfinite(error):
                    raise TrajectoryOptimizeError(f"Non-finite {name} residual near frame {frame:.4f}.")
                residuals[name] = max(residuals[name], float(error))
                ratio = error / tolerances[name]
                if ratio > largest_ratio:
                    largest_ratio = ratio
                    worst = f"{name} residual {error:.6g} near frame {frame:.4f} (tolerance {tolerances[name]:g})"
        if largest_ratio > 1:
            raise TrajectoryOptimizeError(f"Validation failed: {worst}.")
        return residuals


def optimize_trajectory(payload, keyframes, urdf_path, terrain_obj=None, resolve_mesh=lambda _: None,
                       friction=None, time_budget=45.):
    from time import perf_counter
    started = perf_counter()
    names, rate, positions, joints, quats, fixed = read_trajectory(payload, keyframes)
    robot = RobotDynamics(urdf_path, names, terrain_obj, resolve_mesh, friction)
    deadline = started + time_budget if friction is not None else None
    from scipy.optimize import minimize, NonlinearConstraint
    # Use one rotation chart, unwrapping the quaternion representation before
    # choosing equivalent rotation vectors closest to the preceding frame.
    rotations = robot.Rotation.from_quat(quats[:, [1, 2, 3, 0]]).as_rotvec()
    for i in range(1, len(rotations)):
        angle = np.linalg.norm(rotations[i])
        if angle > 1e-9:
            axis = rotations[i] / angle
            options = [rotations[i] + k*2*np.pi*axis for k in (-1, 0, 1)]
            rotations[i] = min(options, key=lambda r: np.linalg.norm(r - rotations[i-1]))
    reference = np.c_[positions, rotations, joints]
    problem = CollocationProblem(robot, reference, fixed, 1/rate)
    for frame in fixed:
        if np.any(joints[frame] < robot.lower - TOLERANCES["position"]) or np.any(joints[frame] > robot.upper + TOLERANCES["position"]):
            raise TrajectoryOptimizeError(f"Fixed keyframe {frame} violates joint position limits.")
        clearance = np.min(robot.clearances(reference[frame]))
        if clearance < -TOLERANCES["clearance"]:
            raise TrajectoryOptimizeError(f"Fixed keyframe {frame} penetrates terrain by {-clearance:.6g} m.")
    if friction is None:
        robot.check_fixed_dynamics(reference, fixed, 1/rate)
    velocities = np.gradient(reference, 1/rate, axis=0, edge_order=2 if len(reference) > 2 else 1)
    initial = problem.pack(reference, velocities)
    if friction is not None:
        from trajectory_optimize_contacts import solve_contacts
        result = solve_contacts(problem, initial, deadline)
    elif len(initial) > DENSE_VARIABLE_LIMIT:
        from scipy.sparse import csr_matrix, diags
        objective_hessian = diags(np.r_[np.full(problem.position_size, 2.),
                                         np.full(problem.n * problem.d, 2e-8)], format="csr")
        # Use the reference objective curvature with linearized constraints.
        # Dense quasi-Newton Hessians/Jacobians would exhaust memory on the
        # documented multi-thousand-frame motions.
        constraint_hessian = csr_matrix((len(initial), len(initial)))
        equalities = NonlinearConstraint(
            problem.equalities, 0, 0,
            jac=lambda x: problem.constraint_jacobian(x, equality=True),
            hess=lambda x, multipliers: constraint_hessian)
        inequalities = NonlinearConstraint(
            problem.inequalities, 0, np.inf,
            jac=lambda x: problem.constraint_jacobian(x, equality=False),
            hess=lambda x, multipliers: constraint_hessian)
        result = minimize(problem.objective, initial, method="trust-constr",
                          jac=problem.objective_gradient, hess=lambda x: objective_hessian,
                          constraints=[equalities, inequalities],
                          options={"maxiter": 300, "gtol": 1e-6, "sparse_jacobian": True,
                                   "initial_barrier_parameter": 1e-6, "initial_barrier_tolerance": 1e-6})
    elif len(problem.equalities(initial)) > len(initial):
        # Many fixed frames can make the collocation equations redundant.
        # SLSQP rejects m > n outright; SVD permits compatible fixed data.
        result = minimize(problem.objective, initial, method="trust-constr",
                          jac=problem.objective_gradient,
                          constraints=[NonlinearConstraint(problem.equalities, 0, 0,
                                           jac=lambda x: problem.constraint_jacobian(x, True).toarray()),
                                       NonlinearConstraint(problem.inequalities, 0, np.inf,
                                           jac=lambda x: problem.constraint_jacobian(x, False).toarray())],
                          options={"maxiter": 300, "gtol": 1e-8,
                                   "factorization_method": "SVDFactorization"})
    else:
        result = minimize(problem.objective, initial, method="SLSQP",
                          jac=problem.objective_gradient,
                          constraints=[{"type": "eq", "fun": problem.equalities,
                                        "jac": lambda x: problem.constraint_jacobian(x, True).toarray()},
                                       {"type": "ineq", "fun": problem.inequalities,
                                        "jac": lambda x: problem.constraint_jacobian(x, False).toarray()}],
                          options={"maxiter": 300, "ftol": 1e-9})
    if not result.success:
        residual = float(np.max(np.abs(problem.equalities(result.x))))
        violation = max(0., -float(np.min(problem.inequalities(result.x))))
        raise TrajectoryOptimizeError(f"Solver failed: {result.message}. Base dynamics residual {residual:.6g}; "
                              f"constraint violation {violation:.6g}. Fixed keyframes were not relaxed.")
    x, velocity = problem.unpack(result.x)
    output = copy.deepcopy(payload)
    output["base_pos_w"] = x[:, :3].astype(np.float32).tolist()
    output["joint_pos"] = x[:, 6:].astype(np.float32).tolist()
    output["base_quat_w"] = robot.Rotation.from_rotvec(x[:, 3:6]).as_quat()[:, [3, 0, 1, 2]].astype(np.float32).tolist()
    for frame in fixed:
        for field in ("joint_pos", "base_pos_w", "base_quat_w"):
            output[field][frame] = copy.deepcopy(payload[field][frame])
    # Validate the actual float32 configuration that the editor/NPZ will keep,
    # rather than only the solver doubles.
    rounded = x.copy()
    rounded[:, :3] = output["base_pos_w"]
    rounded[:, 6:] = output["joint_pos"]
    for i, quat in enumerate(output["base_quat_w"]):
        r = robot.Rotation.from_quat(np.asarray(quat)[[1, 2, 3, 0]]).as_rotvec()
        angle = np.linalg.norm(r)
        if angle > 1e-9:
            r = min([r + k*2*np.pi*r/angle for k in (-1, 0, 1)], key=lambda v: np.linalg.norm(v-x[i, 3:6]))
        rounded[i, 3:6] = r
    residuals = problem.validate(problem.pack(rounded, velocity), deadline)
    return {"trajectory": output, "residuals": residuals, "tolerances": robot.tolerances,
            "validation_samples_per_interval": 17, "iterations": int(result.nit),
            "friction": friction, "seconds": perf_counter() - started}
