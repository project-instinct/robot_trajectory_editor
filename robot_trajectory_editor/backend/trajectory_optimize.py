"""Whole-trajectory Trajectory Optimize by direct transcription on the frame grid.

The decision variables are the configurations of the frames that are not
fixed keyframes. Velocities and accelerations are central differences of
neighbouring configurations on the robot's configuration manifold, so the
saved positions alone determine the motion:

    v_k = (q_{k+1} - q_{k-1}) / 2dt,    a_k = (q_{k+1} - 2 q_k + q_{k-1}) / dt^2

(with "-" the Lie-group difference taken at q_k). At every interior frame the
floating-base inverse dynamics must be explained by bounded joint torques and
unilateral, friction-limited point support at terrain contacts. An
augmented-Lagrangian loop of damped Gauss-Newton (Levenberg-Marquardt) solves
over the whole trajectory drives the dynamics, actuator, contact and limit
violations to zero while staying close to a smoothed target. An independent
validator then checks the float32 output with live contact inference,
including poses between frames.
"""
import copy
import io
import math
import multiprocessing
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from time import perf_counter

import numpy as np


class TrajectoryOptimizeError(ValueError):
    pass


# Acceptance tolerances. Dynamics is scaled to 0.1% of body weight below.
TOLERANCES = {"dynamics": 1e-3, "effort": 1e-4, "position": 1e-6, "velocity": 1e-6, "clearance": 1e-5}
# The validator infers support only from geometry within this terrain gap.
CONTACT_MARGIN = 1e-3
# Initialization aligns support to this gap; the solver keeps scheduled
# support points at most CONTACT_HOLD above the terrain, inside the margin.
CONTACT_TARGET = 5e-4
CONTACT_HOLD = CONTACT_MARGIN - 1e-4
# The solver keeps all other geometry at least this far from the terrain.
CLEARANCE_TARGET = 5e-5
# Solver weighting: one unit of residual is this much violation. Physical
# scales keep the least-squares weights within a few decades; the acceptance
# tolerances above are far too uneven for that. Dynamics is scaled by body
# weight. The solver works slightly inside the limits so that the accepted
# trajectory meets them exactly. Every constraint is a weighted least-squares
# term whose augmented-Lagrangian multiplier removes the remaining violation.
SOLVER_SCALE = {"dynamics": 1e-2, "effort": 1., "velocity": 1., "position": 1e-2, "distance": 1e-3}
SOLVER_MARGIN = {"effort": 1e-3, "velocity": 1e-3, "position": 1e-5}
# How freely the linearization lets allocated support follow a step, per
# squared share of body weight a friction-pyramid ray carries (see terms()).
SUPPORT_DAMPING = .3
# Contact schedule: support points close to an upward-facing surface that
# barely move toward or away from it in the initial trajectory. Sliding along
# it is allowed up to a limit, so a rolling back keeps its support while a
# foot swinging past the ground does not. A link only rests on its lowest
# points (a tilted foot stands on an edge), and only links near the lowest one
# of the frame are candidates (not a shin above a foot).
SCHEDULE_GAP = 2e-2
SCHEDULE_NORMAL_SPEED = .3
SCHEDULE_SLIDE_SPEED = 2.
SCHEDULE_NORMAL_Z = .5
SCHEDULE_POINT_BAND = 1e-3
SCHEDULE_LINK_BAND = 1e-2
# Poses checked between consecutive frames by the validator.
SUBSAMPLES = 8
# Distances from the edited trajectory that count as one unit of data error.
DATA_SCALE = {"base_position": 1e-2, "base_rotation": 2e-2, "joint": 2e-2}
# Acceleration change from the edited trajectory (m/s^2 or rad/s^2) that counts
# as one unit of data error. Central differences cannot see a frame-to-frame
# zigzag in velocity, so without this term the solver may shake a limb at the
# frame rate to manufacture momentum.
ACCELERATION_SCALE = 20.
# Least-squares weights: "data" and "acceleration" scale the two terms that
# stay close to the target; the others are the augmented-Lagrangian penalty
# factors of each constraint kind ("middle" is clearance between frames).
CONSTRAINTS = {"dynamics": False, "effort": True, "position": True, "velocity": True, "contact": True,
               "clearance": True, "middle": True}  # kind -> is an inequality
DEFAULT_WEIGHTS = {"data": 1., "acceleration": 1., **dict.fromkeys(CONSTRAINTS, 1.)}
# Weights of the target terms once the augmented-Lagrangian loop is running.
# The acceleration term stays at full weight: it is what stops frame-rate
# shaking, and the multipliers, not a weak regularization, satisfy the
# constraints.
REGULARIZATION = {"data": 1e-2, "acceleration": 1.}
# Largest augmented-Lagrangian penalty factor. Larger factors make the
# least squares stiff enough that the inner solves stall and limbs shake.
PENALTY_MAX = 100.
# Target preparation defaults (user parameters): low-pass cutoff of the target
# and the joint speed, as a multiple of the URDF limit, that marks a glitch.
DEFAULT_SMOOTHING_HZ = 8.
DEFAULT_GLITCH_SPEED_SCALE = 1.


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
    if np.max(np.abs(np.linalg.norm(quats, axis=1) - 1)) > 1e-6:
        raise TrajectoryOptimizeError("Base quaternion norm differs from 1 by more than 1e-6; repair the input first.")
    if not isinstance(keyframes, list) or any(type(k) is not int or k < 0 or k >= n for k in keyframes):
        raise TrajectoryOptimizeError("Keyframes must be integer frame indices within the trajectory.")
    return names, rate, positions, joints, quats, sorted(set(keyframes))


def skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


class RobotModel:
    """Pinocchio dynamics, collision geometry and terrain for one request."""

    def __init__(self, urdf_path, names, terrain_obj, resolve_mesh, friction=None, effort_scale=1.0):
        try:
            import pinocchio as pin
            import coal
            import trimesh
        except ImportError as error:
            raise TrajectoryOptimizeError("Trajectory Optimize dependencies are missing. Install "
                                          "backend/requirements-trajectory-optimize.txt in the backend "
                                          f"environment ({error}).") from error
        self.pin, self.coal = pin, coal
        root = ET.parse(urdf_path).getroot()
        movable = []
        lower, upper, velocity, effort = [], [], [], []
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
            for target, key, default in ((lower, "lower", -np.inf), (upper, "upper", np.inf),
                                         (velocity, "velocity", np.inf), (effort, "effort", np.inf)):
                target.append(float(attrs.get(key, default)))
            # urdfdom requires these attributes; absent limits stay unbounded
            # rather than silently imposing a guessed value.
            if limit is None:
                limit = ET.SubElement(by_name[name], "limit")
            for key in ("lower", "upper", "velocity", "effort"):
                if key not in limit.attrib:
                    limit.set(key, "1e20" if key != "lower" else "-1e20")
        lower, upper, velocity, effort = map(np.asarray, (lower, upper, velocity, effort))
        if (np.isnan(lower).any() or np.isnan(upper).any() or np.isnan(velocity).any() or np.isnan(effort).any()
                or np.any(lower > upper) or np.any(velocity < 0) or np.any(effort < 0)):
            raise TrajectoryOptimizeError("URDF has invalid joint limits.")
        if type(effort_scale) not in (int, float) or not math.isfinite(effort_scale) or effort_scale < 1.0:
            raise TrajectoryOptimizeError("max_effort_scale must be a finite number at least 1.")
        effort = effort * effort_scale
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
            if (not np.isfinite(inertia.mass) or inertia.mass <= 0 or not np.isfinite(eigenvalues).all()
                    or np.min(eigenvalues) <= 0):
                raise TrajectoryOptimizeError("Robot mass and inertia must be finite and positive for each moving body.")
            if eigenvalues[-1] > eigenvalues[0] + eigenvalues[1] + 1e-10:
                raise TrajectoryOptimizeError("Robot inertia violates the principal-moment triangle inequality.")
        self.weight = sum(i.mass for i in self.model.inertias) * np.linalg.norm(self.model.gravity.linear)
        self.tolerances = dict(TOLERANCES, dynamics=max(TOLERANCES["dynamics"], 1e-3 * self.weight))
        # Trajectory joint order <-> Pinocchio coordinates. Joint limits are
        # kept in Pinocchio's velocity order, the order of rnea's output.
        self.names = list(names)
        self.q_indices = np.array([self.model.joints[self.model.getJointId(n)].idx_q for n in names], dtype=int)
        self.v_indices = np.array([self.model.joints[self.model.getJointId(n)].idx_v for n in names], dtype=int)
        order = self.v_indices - 6
        self.joint_q = np.empty(len(names), dtype=int)
        self.joint_q[order] = self.q_indices
        self.lower, self.upper = np.empty(len(names)), np.empty(len(names))
        self.velocity, self.effort = np.empty(len(names)), np.empty(len(names))
        self.joint_names_v = [None] * len(names)
        for name, slot in zip(names, order):
            self.joint_names_v[slot] = name
        self.lower[order], self.upper[order] = lower, upper
        self.velocity[order], self.effort[order] = velocity, effort

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
        # Point-to-terrain queries use a vanishing sphere.
        self.probe = coal.Sphere(1e-9)
        # Each shape's local bounding sphere, for broad-phase culling.
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
        """Rigorous lower bound on a shape's distance to the terrain, or None."""
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

    def configuration(self, position, quat_wxyz, joints):
        q = self.pin.neutral(self.model)
        q[:3] = position
        quat = np.asarray(quat_wxyz, dtype=float)[[1, 2, 3, 0]]
        q[3:7] = quat / np.linalg.norm(quat)
        q[self.q_indices] = joints
        return q

    def pose(self, q):
        """Base position, wxyz quaternion and joints in trajectory order."""
        return q[:3], q[[6, 3, 4, 5]], q[self.q_indices]

    def distances(self, q, cull=None):
        """Signed terrain distance, robot-side nearest point and outward support
        normal of every collision geometry at configuration q.

        Geometry whose rigorous lower bound exceeds `cull` is not queried: its
        bound is returned with NaN point and normal.
        """
        self.pin.updateGeometryPlacements(self.model, self.data, self.geometry, self.geometry_data, q)
        count = len(self.geometry.geometryObjects)
        distance = np.empty(count)
        points = np.full((count, 3), np.nan)
        normals = np.full((count, 3), np.nan)
        identity = self.coal.Transform3s()
        for index, (geometry, pose) in enumerate(zip(self.geometry.geometryObjects, self.geometry_data.oMg)):
            if cull is not None:
                local_center, radius = self.geometry_bounds[index]
                lower = self.terrain_lower_bound(pose.rotation @ local_center + pose.translation, radius)
                if lower is not None and lower > cull:
                    distance[index] = lower
                    continue
            result = self.coal.DistanceResult()
            distance[index] = self.coal.distance(geometry.geometry, self.coal.Transform3s(pose.rotation, pose.translation),
                                                 self.terrain, identity, self.distance_request, result)
            points[index] = result.getNearestPoint1()
            normals[index] = -np.asarray(result.normal)
        if self.containment:
            # A body wholly inside a watertight terrain has a positive unsigned
            # distance; flip it to a penetration with an outward normal.
            positive = np.flatnonzero((distance > 0) & np.isfinite(points[:, 0]))
            if len(positive):
                inside = positive[self.terrain_mesh.contains(points[positive])]
                distance[inside] *= -1
                normals[inside] *= -1
        return distance, points, normals

    def support_points(self, index, normal, nearest):
        """Candidate support points of one geometry facing a terrain normal.

        A flat-lying cylinder or box rests along a line or a face, where its
        single nearest point jumps between equally near ends. The lowest
        points of a cylinder's end rims (a capsule's end spheres, a box's
        corners) span that support and move continuously with the pose.
        Requires the geometry placements of the current configuration.
        """
        coal = self.coal
        shape = self.geometry.geometryObjects[int(index)].geometry
        pose = self.geometry_data.oMg[int(index)]
        rotation, center = pose.rotation, pose.translation
        if isinstance(shape, coal.Sphere):
            return (center - shape.radius * normal)[None]
        if isinstance(shape, (coal.Cylinder, coal.Capsule)):
            axis = rotation[:, 2]
            ends = np.array([center + shape.halfLength * axis, center - shape.halfLength * axis])
            if isinstance(shape, coal.Capsule):
                return ends - shape.radius * normal
            radial = normal - (normal @ axis) * axis
            length = np.linalg.norm(radial)
            return ends - shape.radius * radial / length if length > 1e-9 else ends
        if isinstance(shape, coal.Box):
            signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=float)
            return center + (signs * shape.halfSide) @ rotation.T
        return np.asarray(nearest, dtype=float)[None]

    def point_distance(self, point):
        """Signed terrain distance and outward normal at a point."""
        if self.terrain_mesh is None:
            return float(point[2]), np.array([0., 0., 1.])
        result = self.coal.DistanceResult()
        distance = self.coal.distance(self.probe, self.coal.Transform3s(np.eye(3), point), self.terrain,
                                      self.coal.Transform3s(), self.distance_request, result) - self.probe.radius
        normal = -np.asarray(result.normal)
        if self.containment and distance > 0 and self.terrain_mesh.contains(point[None])[0]:
            return -distance, -normal
        return distance, normal

    def support_point(self, key, nearest, outward):
        """Position, terrain distance and normal of one support point.

        `key` is geometry * 16 + candidate. Requires `distances(q)` at the
        current configuration, whose nearest points and normals are passed.
        """
        index, candidate = divmod(int(key), 16)
        point = self.support_points(index, outward[index], nearest[index])[candidate]
        gap, normal = self.point_distance(point)
        return index, point, gap, normal

    def support_keys(self, index, nearest, outward):
        return [int(index) * 16 + c for c in range(len(self.support_points(index, outward[index], nearest[index])))]

    def point_jacobian(self, index, point):
        """World-aligned Jacobian of a point carried by a geometry's parent joint.

        Requires computeJointJacobians at the current configuration.
        """
        joint = self.geometry.geometryObjects[int(index)].parentJoint
        jac = self.pin.getJointJacobian(self.model, self.data, joint, self.pin.LOCAL_WORLD_ALIGNED)
        return jac[:3] - skew(point - self.data.oMi[joint].translation) @ jac[3:]

    def surface_intersection(self, q):
        """Name of a geometry crossing a mesh terrain surface, or None.

        Triangle distances can be zero for crossing surfaces without a useful
        penetration depth, so mesh terrains get an explicit collision check.
        """
        if self.terrain_mesh is None:
            return None
        self.pin.updateGeometryPlacements(self.model, self.data, self.geometry, self.geometry_data, q)
        identity = self.coal.Transform3s()
        for (geometry, pose), (local_center, radius) in zip(
                zip(self.geometry.geometryObjects, self.geometry_data.oMg), self.geometry_bounds):
            lower = self.terrain_lower_bound(pose.rotation @ local_center + pose.translation, radius, margin=0.)
            if lower is not None and lower > 0.:
                continue
            result = self.coal.CollisionResult()
            self.coal.collide(geometry.geometry, self.coal.Transform3s(pose.rotation, pose.translation),
                              self.terrain, identity, self.coal.CollisionRequest(), result)
            if result.isCollision():
                return geometry.name
        return None


class Problem:
    """Residuals of one trajectory on the frame grid, frame by frame.

    Every residual owned by frame k depends on frames k-1, k and k+1 only, so
    its Jacobian is one dense block and the Gauss-Newton system is banded.
    """

    def __init__(self, robot, reference, fixed, dt):
        self.robot, self.reference, self.dt = robot, reference, dt
        self.n, self.nv = len(reference), robot.model.nv
        self.free = np.ones(self.n, dtype=bool)
        self.free[fixed] = False
        self.data_scale = np.r_[np.full(3, DATA_SCALE["base_position"]), np.full(3, DATA_SCALE["base_rotation"]),
                                np.full(self.nv - 6, DATA_SCALE["joint"])]
        # Support-point keys that may carry support at each frame.
        self.schedule = [np.empty(0, dtype=int) for _ in range(self.n)]
        # Second differences of the edited trajectory, in each frame's tangent.
        self.reference_curvature = np.zeros((self.n, self.nv))
        for k in range(1, self.n - 1):
            plus, minus, _, _ = self.kinematics(reference, k)
            self.reference_curvature[k] = plus + minus

    def kinematics(self, Q, k):
        pin, model = self.robot.pin, self.robot.model
        plus = pin.difference(model, Q[k], Q[k+1])
        minus = pin.difference(model, Q[k], Q[k-1])
        return plus, minus, (plus - minus) / (2*self.dt), (plus + minus) / self.dt**2

    def contact_torque(self, q, contacts):
        """Generalized force of world forces at points fixed to their joints."""
        pin, robot = self.robot.pin, self.robot
        pin.computeJointJacobians(robot.model, robot.data, q)
        torque = np.zeros(self.nv)
        for joint, local_points, forces in contacts:
            jac = pin.getJointJacobian(robot.model, robot.data, joint, pin.LOCAL_WORLD_ALIGNED)
            arms = local_points @ robot.data.oMi[joint].rotation.T
            moment = np.sum(arms[:, [1, 2, 0]] * forces[:, [2, 0, 1]] - arms[:, [2, 0, 1]] * forces[:, [1, 2, 0]],
                            axis=0)
            torque += jac[:3].T @ forces.sum(axis=0) + jac[3:].T @ moment
        return torque

    def terms(self, Q, k, jacobian, weights=None, multipliers=None, dynamics=True):
        """Residual vector of frame k, its Jacobian on frames (k-1, k, k+1),
        the largest violation of each constraint kind in physical units, and
        the scaled constraint values for the multiplier update.

        Every constraint is a least-squares term in units of SOLVER_SCALE. With
        augmented-Lagrangian `multipliers` (per kind, for this frame) and
        penalty factors in `weights`, an equality c enters as
        sqrt(rho) * (c + y / rho), an inequality g <= 0 as
        sqrt(rho) * max(0, g + y / rho).
        """
        robot, pin, model, data = self.robot, self.robot.pin, self.robot.model, self.robot.data
        nv, dt = self.nv, self.dt
        weights = dict(DEFAULT_WEIGHTS, **(weights or {}))
        multipliers = multipliers or {}
        q = Q[k]
        residuals, blocks = [], []
        stats = {"dynamics": 0., "effort": 0., "clearance": 0., "contact": 0., "position": 0., "velocity": 0.}
        values = {}

        def add(residual, previous=None, current=None, following=None):
            residuals.append(residual)
            if jacobian:
                block = np.zeros((len(residual), 3*nv))
                for column, part in enumerate((previous, current, following)):
                    if part is not None:
                        block[:, column*nv:(column+1)*nv] = part
                blocks.append(block)

        def constrain(kind, value, inequality):
            """Shifted residual of one constraint kind; returns the rows that
            enter the least squares and their factor sqrt(rho)."""
            values[kind] = value
            rho = weights[kind]
            shift = multipliers.get(kind)
            shifted = value + (shift / rho if shift is not None else 0.)
            rows = np.flatnonzero(shifted > 0) if inequality else np.arange(len(value))
            return rows, np.sqrt(rho) * shifted[rows], np.sqrt(rho)

        interior = 0 < k < self.n - 1
        if interior and self.free[k-1:k+2].any():
            # Stay close to the target's accelerations.
            plus, minus, v, a = self.kinematics(Q, k)
            scale = ACCELERATION_SCALE * dt**2 / np.sqrt(weights["acceleration"])
            parts = [None, None, None]
            if jacobian:
                plus_q = pin.dDifference(model, q, Q[k+1], pin.ArgumentPosition.ARG0)
                plus_next = pin.dDifference(model, q, Q[k+1], pin.ArgumentPosition.ARG1)
                minus_q = pin.dDifference(model, q, Q[k-1], pin.ArgumentPosition.ARG0)
                minus_previous = pin.dDifference(model, q, Q[k-1], pin.ArgumentPosition.ARG1)
                parts = [minus_previous / scale, (plus_q + minus_q) / scale, plus_next / scale]
            add((plus + minus - self.reference_curvature[k]) / scale, *parts)
        if dynamics and interior and self.free[k-1:k+2].any():
            keys = self.schedule[k] if robot.support is not None else ()
            maps, normals, points, contacts = [], [], [], []
            if len(keys):
                _, nearest, outward = robot.distances(q)
                pin.computeJointJacobians(model, data, q)
                for key in keys:
                    index, point, _, normal = robot.support_point(key, nearest, outward)
                    maps.append(robot.point_jacobian(index, point).T)
                    normals.append(normal)
                    points.append(point)
                    contacts.append(index)
            required = pin.rnea(model, data, q, v, a).copy()
            if robot.support is not None:
                remainder, _ = robot.support.solve(required, maps, normals)
                coefficients = robot.support.coefficients
            else:
                remainder, coefficients = required, np.empty(0)
            torque = remainder[6:]
            stats["dynamics"] = float(np.max(np.abs(remainder[:6])))
            stats["effort"] = float(max(np.max(np.abs(torque) - robot.effort, initial=0.), 0.))
            force_scale = SOLVER_SCALE["dynamics"] * robot.weight
            _, base, base_factor = constrain("dynamics", remainder[:6] / force_scale, False)
            # The allocation used the true actuator bounds; the solver asks
            # for a little headroom below them.
            saturated, effort, effort_factor = constrain(
                "effort", (np.abs(torque) - (robot.effort - SOLVER_MARGIN["effort"])) / SOLVER_SCALE["effort"], True)
            rows = np.r_[np.arange(6), 6 + saturated]
            factor = np.r_[np.full(6, base_factor / force_scale),
                           np.sign(torque[saturated]) * effort_factor / SOLVER_SCALE["effort"]]
            parts = [None, None, None]
            if jacobian:
                dtau_dq, dtau_dv, mass = (np.array(m) for m in pin.computeRNEADerivatives(model, data, q, v, a))
                mass = np.triu(mass) + np.triu(mass, 1).T
                previous = -dtau_dv @ minus_previous / (2*dt) + mass @ minus_previous / dt**2
                current = dtau_dq + dtau_dv @ (plus_q - minus_q) / (2*dt) + mass @ (plus_q + minus_q) / dt**2
                following = dtau_dv @ plus_next / (2*dt) + mass @ plus_next / dt**2
                active = coefficients > 1e-12
                if active.any():
                    current = current - self.contact_force_derivative(q, contacts, points, normals, coefficients)
                # Gauss-Newton over the configurations and the active ray
                # weights, with the weight changes eliminated. They are
                # damped relative to the weights themselves, which stands in
                # for their bounds: a heavily loaded ray can follow the motion
                # (as at a landing), a lightly loaded one cannot reverse.
                projection = np.eye(len(rows))
                if active.any():
                    span = robot.support.contacts[rows][:, active]
                    gram = span.T @ span
                    share = coefficients[active] / robot.weight
                    gram[np.diag_indices_from(gram)] += SUPPORT_DAMPING / share**2
                    projection -= span @ np.linalg.solve(gram, span.T)
                parts = [factor[:, None] * (projection @ part[rows]) for part in (previous, current, following)]
            add(np.r_[base, effort], *parts)

        if self.free[k]:
            self.geometry_terms(q, k, add, constrain, stats, jacobian)
            # Joint position limits, about the middle of each range.
            x = q[robot.joint_q]
            middle, half = (robot.upper + robot.lower) / 2, (robot.upper - robot.lower) / 2
            offset = np.where(np.isfinite(half), x - middle, 0.)
            stats["position"] = float(max(np.max(np.abs(offset) - half, initial=0.), 0.))
            hit, residual, factor = constrain(
                "position", np.where(np.isfinite(half), (np.abs(offset) - (half - SOLVER_MARGIN["position"]))
                                     / SOLVER_SCALE["position"], -np.inf), True)
            if len(hit):
                part = np.zeros((len(hit), nv))
                part[np.arange(len(hit)), 6 + hit] = np.sign(offset[hit]) * factor / SOLVER_SCALE["position"]
                add(residual, current=part)
            # Stay close to the target.
            scale = self.data_scale / np.sqrt(weights["data"])
            part = None
            if jacobian:
                part = pin.dDifference(model, self.reference[k], q, pin.ArgumentPosition.ARG1) / scale[:, None]
            add(pin.difference(model, self.reference[k], q) / scale, current=part)

        if k < self.n - 1 and self.free[k:k+2].any():
            # Joint velocity limits on the interval to the next frame.
            rate = (Q[k+1][robot.joint_q] - q[robot.joint_q]) / dt
            stats["velocity"] = float(max(np.max(np.abs(rate) - robot.velocity, initial=0.), 0.))
            hit, residual, factor = constrain(
                "velocity", (np.abs(rate) - (robot.velocity - SOLVER_MARGIN["velocity"])) / SOLVER_SCALE["velocity"],
                True)
            if len(hit):
                part = np.zeros((len(hit), nv))
                part[np.arange(len(hit)), 6 + hit] = np.sign(rate[hit]) * factor / (dt * SOLVER_SCALE["velocity"])
                add(residual, current=-part, following=part)
            # Non-penetration halfway to the next frame.
            middle = pin.interpolate(model, q, Q[k+1], .5)
            distance, nearest, outward = robot.distances(middle, cull=CONTACT_MARGIN)
            stats["clearance"] = max(stats["clearance"], float(max(-distance.min(), 0.)))
            hit, residual, factor = constrain("middle", (CLEARANCE_TARGET - distance) / SOLVER_SCALE["distance"], True)
            if np.isnan(nearest[hit, 0]).any():
                # A multiplier keeps a culled geometry active: query it exactly.
                distance, nearest, outward = robot.distances(middle)
                hit, residual, factor = constrain("middle", (CLEARANCE_TARGET - distance) / SOLVER_SCALE["distance"],
                                                  True)
            if len(hit):
                part = None
                if jacobian:
                    pin.computeJointJacobians(model, data, middle)
                    part = -np.array([outward[i] @ robot.point_jacobian(i, nearest[i]) for i in hit]) \
                        * (.5 * factor / SOLVER_SCALE["distance"])
                add(residual, current=part if self.free[k] else None, following=part if self.free[k+1] else None)

        residual = np.concatenate(residuals) if residuals else np.empty(0)
        block = np.vstack(blocks) if jacobian and blocks else np.empty((0, 3*nv))
        return residual, block, stats, values

    def geometry_terms(self, q, k, add, constrain, stats, jacobian):
        """Scheduled support points held inside the contact margin; all other
        geometry kept out of the terrain."""
        robot, pin = self.robot, self.robot.pin
        keys = self.schedule[k] if robot.support is not None else np.empty(0, dtype=int)
        holding = np.unique(keys // 16)
        distance, nearest, outward = robot.distances(q, cull=CONTACT_MARGIN)
        if np.isnan(nearest[holding, 0]).any():
            # A scheduled contact far from the terrain still needs its exact
            # distance and gradient to be pulled back.
            distance, nearest, outward = robot.distances(q)
        stats["clearance"] = max(stats["clearance"], float(max(-distance.min(), 0.)))
        scale = SOLVER_SCALE["distance"]
        if jacobian:
            pin.computeJointJacobians(robot.model, robot.data, q)
        if len(keys):
            support = [robot.support_point(key, nearest, outward) for key in keys]
            gaps = np.array([gap for _, _, gap, _ in support])
            stats["contact"] = float(max(gaps.max() - CONTACT_HOLD, 0.))
            # Only an upper bound: a curved sole or a tilted foot cannot put
            # every scheduled point at one height; clearance is the lower bound.
            hit, residual, factor = constrain("contact", (gaps - CONTACT_HOLD) / scale, True)
            if len(hit):
                part = None
                if jacobian:
                    part = np.array([support[i][3] @ robot.point_jacobian(support[i][0], support[i][1])
                                     for i in hit]) * (factor / scale)
                add(residual, current=part)
        # Scheduled geometry also stays out: its other points may tilt into
        # the terrain while the scheduled ones sit at the contact target.
        clearance = (CLEARANCE_TARGET - distance) / scale
        hit, residual, factor = constrain("clearance", clearance, True)
        if np.isnan(nearest[hit, 0]).any():
            # A multiplier keeps a culled geometry active: query it exactly.
            distance, nearest, outward = robot.distances(q)
            if jacobian:
                pin.computeJointJacobians(robot.model, robot.data, q)
            clearance = (CLEARANCE_TARGET - distance) / scale
            hit, residual, factor = constrain("clearance", clearance, True)
        if len(hit):
            part = None
            if jacobian:
                part = -np.array([outward[i] @ robot.point_jacobian(i, nearest[i]) for i in hit]) * (factor / scale)
            add(residual, current=part)

    def contact_force_derivative(self, q, contacts, points, normals, coefficients):
        """Derivative of the support's generalized force with the allocated
        world forces held fixed and the contact points carried by their links."""
        from support import friction_rays
        pin, robot = self.robot.pin, self.robot
        groups = {}
        for i, (index, point, normal) in enumerate(zip(contacts, points, normals)):
            force = friction_rays(normal, robot.support.friction) @ coefficients[4*i:4*i+4]
            if not np.any(force):
                continue
            joint = robot.geometry.geometryObjects[int(index)].parentJoint
            pose = robot.data.oMi[joint]
            # Point relative to the joint origin, expressed in the joint frame.
            groups.setdefault(joint, ([], []))
            groups[joint][0].append(pose.rotation.T @ (point - pose.translation))
            groups[joint][1].append(force)
        grouped = [(joint, np.array(p), np.array(f)) for joint, (p, f) in groups.items()]
        base = self.contact_torque(q, grouped)
        step = 1e-6
        derivative = np.empty((self.nv, self.nv))
        for axis in range(self.nv):
            moved = pin.integrate(robot.model, q, step * np.eye(self.nv)[axis])
            derivative[:, axis] = (self.contact_torque(moved, grouped) - base) / step
        return derivative


# Forked evaluation workers inherit the problem; only configurations travel.
_WORKER = {}


def frame_equations(problem, frames, terms):
    """Normal-equation contributions of consecutive frames' residual blocks.

    Frame k's block couples frames k-1..k+1, so the matrix has a diagonal
    block per frame and couplings to the next two frames. Frames are indexed
    with one padding frame in front; fixed frames get no coupling. Returns
    the padded index of the first slice row and the slices.
    """
    nv = problem.nv
    movable = np.r_[False, problem.free, False]
    count = len(frames) + 2
    diagonal = np.zeros((count, nv, nv))
    next1, next2 = np.zeros((count, nv, nv)), np.zeros((count, nv, nv))
    gradient = np.zeros((count, nv))
    for i, (k, (residual, block, _, _)) in enumerate(zip(frames, terms)):
        if not len(residual):
            continue
        block = block.reshape(len(residual), 3, nv) * movable[k:k+3, None]
        normal = np.einsum("rin,rjm->injm", block, block)
        for j in range(3):
            diagonal[i+j] += normal[j, :, j]
            gradient[i+j] += residual @ block[:, j]
        next1[i] += normal[0, :, 1]
        next1[i+1] += normal[1, :, 2]
        next2[i] += normal[0, :, 2]
    return frames[0], diagonal, next1, next2, gradient


def _evaluate_frames(args):
    Q, frames, jacobian, weights, multipliers, dynamics = args
    problem = _WORKER["problem"]
    terms = [problem.terms(Q, k, jacobian, weights, None if multipliers is None else multipliers[i], dynamics)
             for i, k in enumerate(frames)]
    cost = sum(float(r @ r) for r, _, _, _ in terms)
    return (cost, [stats for _, _, stats, _ in terms], [values for _, _, _, values in terms],
            frame_equations(problem, frames, terms) if jacobian else None)


def _validate_frames(args):
    Q, frames = args
    return [validate_frame(_WORKER["problem"], Q, k) for k in frames]


class Evaluation:
    """Cost, per-frame violations and (optionally) normal equations of one trajectory."""

    def __init__(self, problem, parts):
        self.cost = sum(cost for cost, _, _, _ in parts)
        self.stats = [stats for _, frame_stats, _, _ in parts for stats in frame_stats]
        self.values = [values for _, _, frame_values, _ in parts for values in frame_values]
        self.equations = None
        if parts[0][3] is not None:
            n, nv = problem.n, problem.nv
            frames = n + 2 + (n % 2)  # even, for pairing in the solve
            diagonal = np.zeros((frames, nv, nv))
            next1, next2 = np.zeros((frames, nv, nv)), np.zeros((frames, nv, nv))
            gradient = np.zeros((frames, nv))
            for _, _, _, (start, d, a, b, g) in parts:
                end = start + len(d)
                diagonal[start:end] += d
                next1[start:end] += a
                next2[start:end] += b
                gradient[start:end] += g
            movable = np.zeros(frames, dtype=bool)
            movable[1:n+1] = problem.free
            self.equations = diagonal, next1, next2, gradient, movable

    def predicted(self, step):
        """Cost of the Gauss-Newton model at a step (frames x nv)."""
        diagonal, next1, next2, gradient, _ = self.equations
        delta = np.zeros_like(gradient)
        delta[1:len(step)+1] = step
        quadratic = (np.einsum("fi,fij,fj->", delta, diagonal, delta)
                     + 2 * np.einsum("fi,fij,fj->", delta[:-1], next1[:-1], delta[1:])
                     + 2 * np.einsum("fi,fij,fj->", delta[:-2], next2[:-2], delta[2:]))
        return self.cost + 2 * float(np.sum(gradient * delta)) + float(quadratic)


class Evaluator:
    """Frame-parallel evaluation through a forked worker pool."""

    def __init__(self, problem, workers):
        _WORKER["problem"] = problem
        self.problem = problem
        self.pool = None
        if workers > 1:
            try:
                self.pool = multiprocessing.get_context("fork").Pool(workers)
            except (ValueError, OSError):
                self.pool = None
        count = workers if self.pool is not None else 1
        self.chunks = [chunk for chunk in np.array_split(np.arange(problem.n), count) if len(chunk)]

    def map(self, function, arguments):
        if self.pool is None:
            return [function(args) for args in arguments]
        return self.pool.map(function, arguments)

    def evaluate(self, Q, jacobian, weights=None, multipliers=None, dynamics=True):
        """Evaluate every frame. `multipliers` is a per-frame list of
        per-kind arrays, or None."""
        return Evaluation(self.problem, self.map(_evaluate_frames, [
            (Q, chunk, jacobian, weights, None if multipliers is None else [multipliers[k] for k in chunk], dynamics)
            for chunk in self.chunks]))

    def validate(self, Q):
        return [item for part in self.map(_validate_frames, [(Q, chunk) for chunk in self.chunks]) for item in part]

    def close(self):
        if self.pool is not None:
            self.pool.terminate()


def solve_normal_equations(equations, damping, floor):
    """Levenberg-Marquardt step by block-tridiagonal Cholesky on frame pairs.

    Damping scales with each variable's curvature, but never below `floor`
    (per coordinate): directions the residuals barely see, such as a free
    arm swing, would otherwise take huge linearized steps.
    """
    from scipy.linalg import solve_triangular
    diagonal, next1, next2, gradient, movable = equations
    frames, nv = gradient.shape
    damped = diagonal.copy()
    index = np.arange(nv)
    damped[:, index, index] += damping * np.maximum(diagonal[:, index, index], floor) + 1e-12
    damped[~movable] = np.eye(nv)
    rhs = np.where(movable[:, None], -gradient, 0.)
    pairs = frames // 2
    size = 2 * nv
    D = np.zeros((pairs, size, size))
    U = np.zeros((pairs, size, size))
    D[:, :nv, :nv], D[:, nv:, nv:] = damped[0::2], damped[1::2]
    D[:, :nv, nv:] = next1[0::2]
    D[:, nv:, :nv] = next1[0::2].transpose(0, 2, 1)
    U[:, :nv, :nv] = next2[0::2]
    U[:, nv:, :nv] = next1[1::2]
    U[:, nv:, nv:] = next2[1::2]
    b = rhs.reshape(pairs, size)
    L = np.empty_like(D)
    E = np.zeros_like(D)
    y = np.empty_like(b)
    for i in range(pairs):
        block = D[i]
        if i:
            C = solve_triangular(L[i-1], U[i-1], lower=True)
            E[i] = C.T
            block = block - C.T @ C
        L[i] = np.linalg.cholesky(block)
        y[i] = solve_triangular(L[i], b[i] - (E[i] @ y[i-1] if i else 0.), lower=True)
    x = np.empty_like(b)
    for i in range(pairs - 1, -1, -1):
        x[i] = solve_triangular(L[i], y[i] - (E[i+1].T @ x[i+1] if i < pairs - 1 else 0.), lower=True, trans="T")
    return x.reshape(frames, nv)


def step_configurations(problem, Q, step):
    pin, model = problem.robot.pin, problem.robot.model
    moved = Q.copy()
    for k in np.flatnonzero(problem.free):
        moved[k] = pin.integrate(model, Q[k], step[k])
    return moved


def feasible_in_model(stats, tolerances):
    worst = {name: max(s[name] for s in stats) for name in stats[0]}
    return (worst["dynamics"] <= .5 * tolerances["dynamics"] and worst["effort"] <= .5 * tolerances["effort"]
            and worst["clearance"] <= .5 * tolerances["clearance"] and worst["position"] <= 0.
            and worst["velocity"] <= 0. and worst["contact"] <= 0.)


def levenberg_marquardt(problem, Q, evaluate, stop, accept=None, patience=3):
    """Minimize one weighted residual over the whole trajectory.

    Damping follows Nielsen's gain-ratio rule. Returns the final
    configurations, the number of accepted steps and whether `accept`
    (if given) ended the solve.
    """
    current = evaluate(Q)
    damping, growth, iterations, stalls = 1e-3, 2., 0, 0
    changed = True
    while perf_counter() < stop:
        if changed and accept is not None and accept(Q, current):
            return Q, iterations, True
        changed = False
        try:
            step = solve_normal_equations(current.equations, damping, problem.data_scale**-2)[1:problem.n+1]
        except np.linalg.LinAlgError:
            damping *= 10
            continue
        trial_Q = step_configurations(problem, Q, step)
        trial = evaluate(trial_Q)
        predicted = current.cost - current.predicted(step)
        gain = (current.cost - trial.cost) / predicted if predicted > 0 else -1.
        if np.isfinite(trial.cost) and trial.cost < current.cost:
            stalls = stalls + 1 if trial.cost > current.cost * (1 - 1e-4) else 0
            Q, current = trial_Q, trial
            damping = max(damping * max(1/3, 1 - (2*min(gain, 1.) - 1)**3), 1e-9)
            growth = 2.
            iterations += 1
            changed = True
            if stalls >= patience:
                break
        else:
            damping *= growth
            growth *= 2
            if damping > 1e8:
                break
    return Q, iterations, False


def update_multipliers(evaluation, multipliers, weights, previous):
    """One augmented-Lagrangian update after an inner solve.

    y <- y + rho c for equalities and y <- max(0, y + rho g) for
    inequalities. A kind whose worst violation has not fallen to a quarter
    of the previous one gets a ten times larger penalty factor, up to
    PENALTY_MAX. Returns the worst violation of each kind (scaled units).
    """
    worst = {}
    for kind, inequality in CONSTRAINTS.items():
        violation = 0.
        for k, values in enumerate(evaluation.values):
            value = values.get(kind)
            if value is None:
                continue
            rho = weights[kind]
            y = multipliers[k].get(kind)
            if y is None or len(y) != len(value):
                y = np.zeros(len(value))
            if inequality:
                finite = np.where(np.isfinite(value), value, -1e30)
                multipliers[k][kind] = np.maximum(0., y + rho * finite)
                violation = max(violation, float(np.max(finite, initial=0.)))
            else:
                multipliers[k][kind] = y + rho * value
                violation = max(violation, float(np.max(np.abs(value), initial=0.)))
        worst[kind] = violation
        if violation > .25 * previous.get(kind, np.inf) and weights[kind] < PENALTY_MAX:
            # Keep the shift y / rho unchanged when the factor grows.
            weights[kind] *= 10
            for frame in multipliers:
                if kind in frame:
                    frame[kind] = frame[kind] * 10
    return worst


def optimize(problem, Q, evaluator, stop, validate):
    """Whole-trajectory solve until validated or out of time.

    A kinematic pass first puts the scheduled contacts on the terrain and
    clears penetration and limits without dynamics, which is cheap and well
    conditioned. Then an augmented-Lagrangian loop: Levenberg-Marquardt on the
    shifted least squares, a multiplier update, and again. The multipliers
    drive the constraints to zero while the terms that stay close to the
    target (and stop frame-rate shaking) keep fixed weights. Each inner solve
    gets at most a third of the remaining time, so the multipliers are
    updated well before the budget runs out.
    """
    def slice_end():
        now = perf_counter()
        return min(stop, now + max(.5, (stop - now) / 3))

    weights = dict(DEFAULT_WEIGHTS)
    Q, iterations, _ = levenberg_marquardt(
        problem, Q, lambda X: evaluator.evaluate(X, True, weights, None, False), slice_end(), patience=2)
    validated = {}

    def accept(X, evaluation):
        if not feasible_in_model(evaluation.stats, problem.robot.tolerances):
            return False
        validated["result"] = validate(X)
        return validated["result"][1] is None

    multipliers = [{} for _ in range(problem.n)]
    previous = {}
    # One inner solve close to the target, then the regularization settles.
    for outer in range(10**6):
        if outer == 1:
            weights.update(data=REGULARIZATION["data"], acceleration=REGULARIZATION["acceleration"])
        Q, steps, done = levenberg_marquardt(
            problem, Q, lambda X: evaluator.evaluate(X, True, weights, multipliers), slice_end(), accept)
        iterations += steps
        if done:
            return Q, iterations, None, validated["result"]
        if perf_counter() >= stop:
            return Q, iterations, "time budget exhausted", None
        previous = update_multipliers(evaluator.evaluate(Q, False, weights, multipliers), multipliers, weights,
                                      previous)


def validate_frame(problem, Q, k):
    """Independent check of frame k and of the interval to frame k+1.

    Returns the largest violation of each kind, what it is (joint, geometry
    or base axis), and a terrain-crossing message or None.
    """
    robot, pin = problem.robot, problem.robot.pin
    q = Q[k]
    errors = dict.fromkeys(TOLERANCES, 0.)
    where = dict.fromkeys(TOLERANCES, "")
    message = None
    x = q[robot.joint_q]
    beyond = np.maximum(x - robot.upper, robot.lower - x)
    if len(beyond) and beyond.max() > 0:
        errors["position"], where["position"] = float(beyond.max()), robot.joint_names_v[int(beyond.argmax())]
    samples = [0.] if k == problem.n - 1 else np.arange(SUBSAMPLES) / SUBSAMPLES
    for fraction in samples:
        pose = q if fraction == 0. else pin.interpolate(robot.model, q, Q[k+1], fraction)
        distance, _, _ = robot.distances(pose, cull=CONTACT_MARGIN)
        if -distance.min() > errors["clearance"]:
            errors["clearance"] = float(-distance.min())
            where["clearance"] = robot.geometry.geometryObjects[int(distance.argmin())].name + \
                ("" if fraction == 0. else f" at frame {k + fraction:.3f}")
        crossing = robot.surface_intersection(pose)
        if crossing is not None and message is None:
            message = f"terrain surface intersection at robot geometry {crossing} near frame {k + fraction:.3f}"
    if k < problem.n - 1:
        rate = np.abs(Q[k+1][robot.joint_q] - x) / problem.dt - robot.velocity
        if len(rate) and rate.max() > 0:
            errors["velocity"], where["velocity"] = float(rate.max()), robot.joint_names_v[int(rate.argmax())]
    if 0 < k < problem.n - 1:
        _, _, v, a = problem.kinematics(Q, k)
        required = pin.rnea(robot.model, robot.data, q, v, a).copy()
        remainder = required
        if robot.support is not None:
            # Live contacts: every nearest point and support point of the
            # robot surface within the contact margin.
            distance, nearest, outward = robot.distances(q, cull=CONTACT_MARGIN)
            pin.computeJointJacobians(robot.model, robot.data, q)
            maps, normals = [], []
            for index in np.flatnonzero(distance <= CONTACT_MARGIN):
                maps.append(robot.point_jacobian(index, nearest[index]).T)
                normals.append(outward[index])
                for key in robot.support_keys(index, nearest, outward):
                    _, point, gap, normal = robot.support_point(key, nearest, outward)
                    if gap <= CONTACT_MARGIN:
                        maps.append(robot.point_jacobian(index, point).T)
                        normals.append(normal)
            remainder, _ = robot.support.solve(required, maps, normals)
        axis = int(np.argmax(np.abs(remainder[:6])))
        errors["dynamics"] = float(np.abs(remainder[axis]))
        where["dynamics"] = "unsupported base force" if axis < 3 else "unsupported base moment"
        over = np.abs(remainder[6:]) - robot.effort
        if len(over) and over.max() > 0:
            errors["effort"], where["effort"] = float(over.max()), robot.joint_names_v[int(over.argmax())]
    return errors, where, message


# Units and the order of checks in the failure report. Checks are ranked by
# their worst value on a physical scale (the solver's), not by the ratio to
# their very uneven acceptance tolerances.
REPORT_UNITS = {"dynamics": "N", "effort": "Nm", "position": "rad", "velocity": "rad/s", "clearance": "m"}


def report_unit(name, where):
    return "Nm" if name == "dynamics" and where.endswith("moment") else REPORT_UNITS[name]


def validate(problem, Q, evaluator):
    """Validate a trajectory.

    Returns the largest residual of each kind, a headline for the worst
    violation (None when everything passes) and per-check findings: worst
    value, its frame and location, and how many frames exceed the tolerance.
    """
    robot = problem.robot
    tolerances = robot.tolerances
    results = evaluator.validate(Q)
    residuals = dict.fromkeys(TOLERANCES, 0.)
    findings = {}
    for name in TOLERANCES:
        values = np.array([errors[name] for errors, _, _ in results])
        frame = int(values.argmax())
        residuals[name] = float(values[frame])
        findings[name] = {"value": float(values[frame]), "frame": frame, "where": results[frame][1][name],
                          "frames_over": int(np.sum(values > tolerances[name])), "tolerance": tolerances[name]}
    scale = {"dynamics": SOLVER_SCALE["dynamics"] * robot.weight, "effort": SOLVER_SCALE["effort"],
             "position": SOLVER_SCALE["position"], "velocity": SOLVER_SCALE["velocity"],
             "clearance": SOLVER_SCALE["distance"]}
    failing = sorted((name for name in TOLERANCES if findings[name]["frames_over"]),
                     key=lambda name: -findings[name]["value"] / scale[name])
    crossing = next((message for _, _, message in results if message), None)
    worst = None
    if failing:
        f = findings[failing[0]]
        worst = (f"{f['where'] or failing[0]} {f['value']:.4g} {report_unit(failing[0], f['where'])} near frame {f['frame']} "
                 f"(tolerance {f['tolerance']:g})")
    if crossing is not None:
        worst = f"{crossing}; {worst}" if worst else crossing
    return residuals, worst, {"order": failing, "checks": findings, "crossing": crossing}


def format_report(result, summary, context):
    """Plain-text report of one run, meant to be copied verbatim."""
    lines = [f"Trajectory Optimize: {'VALIDATED' if result['success'] else 'FAILED (not physically validated)'}"]
    for key in ("motion", "robot", "terrain", "parameters"):
        lines.append(f"{key.capitalize()}: {context[key]}")
    stopped = context.get("stopped") or "validated"
    lines.append(f"Solver: {result['seconds']:.1f} s, {result['iterations']} iterations, stopped: {stopped}")
    checks = summary["checks"]
    if summary["crossing"]:
        lines.append(f"Surface crossing: {summary['crossing']}")
    if summary["order"]:
        lines.append("Violations (largest first):")
        for name in summary["order"]:
            c = checks[name]
            where = f" ({c['where']})" if c["where"] else ""
            lines.append(f"  {name:<9} {c['value']:<10.4g} {report_unit(name, c['where']):<5} at frame {c['frame']}{where}; "
                         f"tolerance {c['tolerance']:g}; over tolerance in {c['frames_over']} frames")
    passed = [name for name in TOLERANCES if not checks[name]["frames_over"]]
    if passed:
        lines.append("Within tolerance: " + ", ".join(f"{name} {checks[name]['value']:.3g}" for name in passed))
    return "\n".join(lines)


def initialize(problem):
    """Start from the edited trajectory, aligned with nearby support.

    Free frames are shifted vertically by a smooth spline so that support
    geometry rests just inside the contact margin. A fixed pose that hovers
    above that fit is joined to it by a ballistic fall or rise (a drop or a
    hop), because the body is airborne there. The contact schedule is then
    read from the aligned trajectory.
    """
    from scipy.interpolate import BSpline
    from scipy.ndimage import median_filter
    robot, pin, model = problem.robot, problem.robot.pin, problem.robot.model
    n, dt = problem.n, problem.dt
    Q = problem.reference.copy()
    for k in np.flatnonzero(problem.free):
        Q[k][robot.joint_q] = np.clip(Q[k][robot.joint_q], robot.lower, robot.upper)
    if robot.support is None:
        return Q
    measured = [robot.distances(q) for q in Q]
    distances = np.array([d for d, _, _ in measured])
    normals = np.array([o for _, _, o in measured])
    window = min(21, n if n % 2 else n - 1)
    # The window bridges short hovering gaps within a support phase.
    nearby = median_filter(distances, size=(window, 1), mode="nearest") < .05
    eligible = nearby & (normals[:, :, 2] > .7)
    targets = np.zeros(n)
    for k in np.flatnonzero(eligible.any(axis=1)):
        index = np.argmin(np.where(eligible[k], distances[k], np.inf))
        targets[k] = (CONTACT_TARGET - distances[k, index]) / normals[k, index, 2]
    times = np.arange(n) * dt
    degree = min(3, n - 1)
    stride = max(degree, round(.3 / dt))
    knots = np.r_[np.repeat(times[0], degree + 1), times[stride:-1:stride], np.repeat(times[-1], degree + 1)]
    basis = BSpline(knots, np.eye(len(knots) - degree - 1), degree)(times)
    # Fixed frames cannot move, so only free frames shape the fit.
    fit = basis @ np.linalg.lstsq(basis[problem.free], targets[problem.free], rcond=None)[0]
    offset = fit.copy()
    fixed = np.flatnonzero(~problem.free)
    # Height of each fixed pose's support above the contact target.
    height = np.array([np.min(np.where(eligible[k], distances[k], np.inf)) for k in fixed]) - CONTACT_TARGET
    # A supported fixed pose cannot move: near it, blend the neighbours'
    # support height into its own.
    for k in fixed[~(height > CONTACT_MARGIN - CONTACT_TARGET)]:
        s = np.clip(np.abs(np.arange(n) - k) / stride, 0., 1.)
        offset -= targets[k] * (1 - s*s*(3 - 2*s))
    # A hovering fixed pose is airborne: its neighbours fall from (or rise
    # into) its height ballistically until they meet the supported fit, as a
    # drop or a hop would.
    gravity = abs(float(model.gravity.linear[2])) or 9.81
    lift = np.zeros(n)
    for k, h in zip(fixed, height):
        if np.isfinite(h) and h > CONTACT_MARGIN - CONTACT_TARGET:
            lift = np.maximum(lift, h - .5 * gravity * (times - times[k])**2)
    airborne = lift > 0
    offset += lift
    for k in np.flatnonzero(problem.free):
        Q[k][2] += offset[k]
    # Contact schedule from the aligned trajectory: support points near an
    # upward-facing surface that are nearly at rest or already penetrating.
    # Fixed frames cannot be moved onto the terrain; they may only use support
    # that is already inside the contact margin.
    joints = [g.parentJoint for g in robot.geometry.geometryObjects]
    for k in np.flatnonzero(~airborne):
        distance, nearest, outward = robot.distances(Q[k])
        before, after = max(k - 1, 0), min(k + 1, n - 1)
        velocity = pin.difference(model, Q[before], Q[after]) / ((after - before) * dt)
        pin.computeJointJacobians(model, robot.data, Q[k])
        if not problem.free[k]:
            problem.schedule[k] = np.array([key for index in np.flatnonzero(distance <= CONTACT_MARGIN)
                                            for key in robot.support_keys(index, nearest, outward)
                                            if robot.support_point(key, nearest, outward)[2] <= CONTACT_MARGIN],
                                           dtype=int)
            continue
        candidates = []
        for index in np.flatnonzero(distance < SCHEDULE_GAP):
            for key in robot.support_keys(index, nearest, outward):
                _, point, gap, normal = robot.support_point(key, nearest, outward)
                speed = robot.point_jacobian(index, point) @ velocity
                moving = (abs(speed @ normal) >= SCHEDULE_NORMAL_SPEED
                          or np.linalg.norm(speed - (speed @ normal) * normal) >= SCHEDULE_SLIDE_SPEED)
                if gap < SCHEDULE_GAP and normal[2] > SCHEDULE_NORMAL_Z and (gap < 0 or not moving):
                    candidates.append((key, joints[index], gap))
        lowest = {}
        for _, joint, gap in candidates:
            lowest[joint] = min(lowest.get(joint, np.inf), gap)
        floor = min(lowest.values(), default=np.inf)
        problem.schedule[k] = np.array([key for key, joint, gap in candidates
                                        if lowest[joint] <= floor + SCHEDULE_LINK_BAND
                                        and gap <= lowest[joint] + SCHEDULE_POINT_BAND], dtype=int)
    return Q


def repair_glitches(robot, joints, fixed, rate, speed_scale, longest=.25):
    """Replace joint samples around frame-to-frame jumps by interpolation.

    A jump is an interval whose joint speed exceeds `speed_scale` times the
    URDF velocity limit. Its free frames are re-interpolated from the nearest
    good frames, and a span that still contains a jump grows by a frame on
    each side, up to `longest` seconds. Fixed frames are never changed.
    Returns the repaired joints (frames x joints) and how many samples changed.
    """
    joints = joints.copy()
    if speed_scale <= 0:
        return joints, 0
    n = len(joints)
    limit = speed_scale * robot.velocity
    is_fixed = np.zeros(n, dtype=bool)
    is_fixed[fixed] = True
    frames = np.arange(n)
    longest = max(3, round(longest * rate))
    repaired = 0
    for joint in np.flatnonzero(np.isfinite(limit)):
        original = joints[:, joint].copy()
        bad = np.zeros(n, dtype=bool)
        values = original
        for _ in range(longest):
            jumps = np.flatnonzero(np.abs(np.diff(values)) * rate > limit[joint])
            if not len(jumps):
                break
            grown = bad.copy()
            grown[jumps] = grown[jumps + 1] = True
            for i in jumps:
                if bad[i] or bad[i + 1]:
                    # Still too fast inside a repaired span: widen it.
                    start = i if bad[i] else i + 1
                    while start > 0 and bad[start - 1]:
                        start -= 1
                    end = start
                    while end < n - 1 and bad[end + 1]:
                        end += 1
                    if end - start + 1 < longest:
                        grown[max(start - 1, 0)] = grown[min(end + 1, n - 1)] = True
            grown &= ~is_fixed
            if np.array_equal(grown, bad) or grown.all():
                break
            bad = grown
            values = original.copy()
            values[bad] = np.interp(frames[bad], frames[~bad], original[~bad])
        joints[:, joint] = values
        repaired += int(bad.sum())
    return joints, repaired


def smooth_target(robot, reference, fixed, rate, smoothing_hz, glitch_speed_scale):
    """The trajectory the solver stays close to.

    Joint glitches are repaired first, then base position, base orientation
    and joints are low-pass filtered without phase lag. Fixed frames keep
    their exact poses: the filter's offset at each fixed frame is blended
    back out over its neighbours, without a kink. Returns the target
    configurations and the number of repaired joint samples.
    """
    from scipy.signal import butter, filtfilt
    pin, model = robot.pin, robot.model
    n = len(reference)
    target = reference.copy()
    repaired = 0
    if glitch_speed_scale > 0:
        joints, repaired = repair_glitches(robot, target[:, robot.joint_q], fixed, rate, glitch_speed_scale)
        target[:, robot.joint_q] = joints
    if 0 < smoothing_hz < rate / 2 and n > 6:
        b, a = butter(2, smoothing_hz / (rate / 2))
        pad = min(3 * max(len(a), len(b)), n - 1)
        quats = target[:, 3:7].copy()
        for k in range(1, n):
            if quats[k] @ quats[k-1] < 0:
                quats[k] *= -1
        target[:, :3] = filtfilt(b, a, target[:, :3], axis=0, padlen=pad)
        quats = filtfilt(b, a, quats, axis=0, padlen=pad)
        target[:, 3:7] = quats / np.linalg.norm(quats, axis=1, keepdims=True)
        target[:, robot.joint_q] = filtfilt(b, a, target[:, robot.joint_q], axis=0, padlen=pad)
    fixed = sorted(fixed)
    if len(fixed) and not np.array_equal(target[fixed], reference[fixed]):
        # Offset needed at each fixed frame, faded out toward the
        # neighbouring fixed frames (or over about one filter period).
        width = max(1, round(rate / smoothing_hz)) if smoothing_hz > 0 else 1
        offsets = {f: pin.difference(model, target[f], reference[f]) for f in fixed}
        correction = np.zeros((n, model.nv))
        bounds = [-np.inf, *fixed, np.inf]
        for i, f in enumerate(fixed):
            reach_before = min(width, f - bounds[i]) if np.isfinite(bounds[i]) else width
            reach_after = min(width, bounds[i + 2] - f) if np.isfinite(bounds[i + 2]) else width
            for k in range(max(0, f - reach_before), min(n, f + reach_after + 1)):
                reach = reach_before if k < f else reach_after
                s = 1 - abs(k - f) / max(reach, 1)
                correction[k] += offsets[f] * s*s*(3 - 2*s)
        for k in range(n):
            if np.any(correction[k]):
                target[k] = pin.integrate(model, target[k], correction[k])
        target[fixed] = reference[fixed]
    return target, repaired


def _build_output(payload, robot, Q, fixed):
    output = copy.deepcopy(payload)
    positions, quats, joints = zip(*(robot.pose(q) for q in Q))
    output["base_pos_w"] = np.asarray(positions).astype(np.float32).tolist()
    output["base_quat_w"] = np.asarray(quats).astype(np.float32).tolist()
    output["joint_pos"] = np.asarray(joints).astype(np.float32).tolist()
    for frame in fixed:
        for field in ("joint_pos", "base_pos_w", "base_quat_w"):
            output[field][frame] = copy.deepcopy(payload[field][frame])
    # Validate the float32 trajectory that the editor will keep.
    saved = np.array([robot.configuration(p, r, j) for p, r, j in
                      zip(output["base_pos_w"], output["base_quat_w"], output["joint_pos"])])
    return output, saved


def optimize_trajectory(payload, keyframes, urdf_path, terrain_obj=None, resolve_mesh=lambda _: None,
                        friction=None, time_budget=45., max_effort_scale=1.0,
                        smoothing_hz=DEFAULT_SMOOTHING_HZ, glitch_speed_scale=DEFAULT_GLITCH_SPEED_SCALE):
    """Optimize the whole trajectory.

    `smoothing_hz` is the low-pass cutoff of the target the solver stays
    close to (0 keeps every frame's detail). `glitch_speed_scale` repairs
    joint jumps faster than that multiple of the URDF velocity limit in the
    target (0 disables). Fixed frames are never changed.
    """
    started = perf_counter()
    if type(max_effort_scale) not in (int, float) or not math.isfinite(max_effort_scale) or max_effort_scale < 1.0:
        raise TrajectoryOptimizeError("max_effort_scale must be a finite number at least 1.")
    for name, value in (("smoothing_hz", smoothing_hz), ("glitch_speed_scale", glitch_speed_scale)):
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise TrajectoryOptimizeError(f"{name} must be a finite number at least 0.")
    names, rate, positions, joints, quats, fixed = read_trajectory(payload, keyframes)
    # With no tagged keyframes the trajectory endpoints are the boundary
    # conditions; only the interior frames are optimized.
    if not fixed:
        fixed = [0, len(positions) - 1]
    robot = RobotModel(urdf_path, names, terrain_obj, resolve_mesh, friction, effort_scale=max_effort_scale)
    reference = np.array([robot.configuration(p, r, j) for p, r, j in zip(positions, quats, joints)])
    for frame in fixed:
        x = reference[frame][robot.joint_q]
        if np.any(x < robot.lower - TOLERANCES["position"]) or np.any(x > robot.upper + TOLERANCES["position"]):
            raise TrajectoryOptimizeError(f"Fixed keyframe {frame} violates joint position limits.")
        clearance = float(np.min(robot.distances(reference[frame])[0]))
        if clearance < -TOLERANCES["clearance"]:
            raise TrajectoryOptimizeError(f"Fixed keyframe {frame} penetrates terrain by {-clearance:.6g} m.")
    target, repaired = smooth_target(robot, reference, fixed, rate, smoothing_hz, glitch_speed_scale)
    problem = Problem(robot, target, fixed, 1 / rate)
    workers = int(os.environ.get("TRAJECTORY_OPTIMIZE_WORKERS", 0)) or max(1, min(12, (os.cpu_count() or 1) - 2))
    workers = min(workers, max(1, problem.n // 60))
    deadline = started + time_budget
    from threadpoolctl import threadpool_limits
    # The work is many small dense blocks per frame, across forked workers.
    # Multithreaded BLAS on blocks this small only contends for the cores.
    blas = threadpool_limits(limits=1)
    Q = initialize(problem)
    evaluator = Evaluator(problem, workers)
    try:
        def check(candidate):
            _, saved = _build_output(payload, robot, candidate, fixed)
            return validate(problem, saved, evaluator)

        # Keep time for the final validation, a few residual passes long.
        probe = perf_counter()
        evaluator.evaluate(Q, False)
        reserve = max(1., 3 * (perf_counter() - probe))
        iterations, reason, checked = 0, None, None
        if problem.free.any():
            Q, iterations, reason, checked = optimize(problem, Q, evaluator, deadline - reserve, check)
        output, saved = _build_output(payload, robot, Q, fixed)
        residuals, worst, summary = checked if checked is not None else validate(problem, saved, evaluator)
    finally:
        evaluator.close()
        blas.restore_original_limits()
    result = {"trajectory": output, "residuals": residuals, "tolerances": robot.tolerances,
              "validation_samples_per_interval": SUBSAMPLES, "iterations": iterations,
              "friction": friction, "max_effort_scale": max_effort_scale, "smoothing_hz": smoothing_hz,
              "glitch_speed_scale": glitch_speed_scale, "repaired_joint_samples": repaired,
              "seconds": perf_counter() - started, "success": worst is None}
    if worst is not None:
        stopped = f" Optimizer stopped: {reason}." if reason else ""
        result["error"] = f"Validation failed: {worst}.{stopped}"
    keys = (f"frames {', '.join(map(str, fixed))}" if keyframes
            else f"none (frames 0 and {len(positions) - 1} fixed)")
    mass = robot.weight / np.linalg.norm(robot.model.gravity.linear)
    terrain = ("flat ground z=0" if robot.terrain_mesh is None else
               f"mesh with {len(robot.terrain_mesh.faces)} triangles, bounds "
               f"{np.round(robot.terrain_mesh.bounds, 3).tolist()}")
    context = {
        "motion": f"{len(positions)} frames at {rate:g} fps, {len(names)} joints; keyframes: {keys}",
        "robot": f"{Path(urdf_path).name}, mass {mass:.2f} kg",
        "terrain": terrain,
        "parameters": (f"friction {friction}, max_effort_scale {max_effort_scale:g}, smoothing {smoothing_hz:g} Hz, "
                       f"glitch threshold {glitch_speed_scale:g}x ({repaired} joint samples repaired), "
                       f"budget {time_budget:g} s"),
        "stopped": reason,
    }
    result["report"] = format_report(result, summary, context)
    return result
