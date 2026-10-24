# Robot Trajectory Editor

Web-based editor for inspecting, repairing, and authoring robot joint and floating-base trajectories directly against a URDF model in 3D.

Teaser: ![teaser](media/teaser.gif)

## Key Features

- **Trajectory I/O and playback**: load and save `.npz` motions, scrub or play them on a URDF robot, and inspect or numerically edit every joint and base channel.
- **Visual timeline editing**: draw channel values by dragging keyframes, select time segments, and apply fill, smoothing, joint-space interpolation, nudging, and multi-level undo.
- **Trajectory base transform**: shift and rotate the entire base position/orientation sequence with live sliders (`x`/`y`/`z`/`roll`/`pitch`/`yaw`) — updates the 3D view and timeline in real time, confirmed with Apply and revertable with a single `Ctrl+Z`.
- **Direct 3D manipulation**: drag robot links with IK, translate or rotate the base, and pin multiple links to world-space poses while editing the motion.
- **Cartesian trajectory interpolation**: interpolate selected link targets linearly or cubically with per-link position-only or full-pose constraints; floating-base IK solves each frame transactionally.
- **Terrain workflow**: load `.obj` or `.stl` terrain, crop, down-sample, move, rotate, and export the edited mesh as `.obj`.
- **Keyboard-first navigation**: step frames, mark timeline segments, cycle Edit Targets within their channel groups, nudge values, play, save, and undo without leaving the viewport.

## Quick Start

```bash
# Launch both backend + frontend (prompts to install missing Python/npm dependencies)
./launch.sh [--robot-urdf /path/to/robot.urdf] [--port 5000] [--nudge-step 0.01] [--frame-step 1]
```

Then open http://localhost:5173.

### Manual launch

#### 1. Start Backend

```bash
cd robot_trajectory_editor/backend
pip install -r requirements.txt
python app.py --robot-urdf /path/to/robot.urdf --port 5000
```

#### 2. Start Frontend

```bash
cd robot_trajectory_editor/frontend
npm install
npm run dev
```

Open http://localhost:5173 in a browser.

## Testing

```bash
# Backend tests
cd robot_trajectory_editor/backend && pytest tests/ -v

# Frontend tests
cd robot_trajectory_editor/frontend && npm test
```

## Architecture

- **Backend**: Flask + numpy for `.npz` trajectory I/O and URDF asset serving
- **Frontend**: React + TypeScript + Vite, Three.js for 3D visualization, zustand for state
- **IK**: `closed-chain-ik`, wrapped by `frontend/src/three/IkSolver.ts`; direct manipulation keeps the trajectory-driven base locked, while Cartesian interpolation solves the floating base and joints together
- **Terrain**: `.obj` and `.stl` load client-side; Save Terrain exports `.obj`

## Viewport Interaction

- Left-drag a body link: IK-drag that link (joints solve, base stays)
- Left-drag the base (root) link: translate the base (grab offset preserved)
- Shift+drag the base: yaw the base around world Z through its origin
- Double-click a link: pin it at its current world pose (highlighted); dragging then moves the base while IK keeps the link fixed. Double-click again or Esc to unpin

## Timeline Interaction

- Click / drag: scrub playhead; mouse wheel: step frames
- Alt + mouse wheel: zoom the timeline around the pointer; two-finger pinch: zoom and pan the visible frame range
- The visible frame/value ranges are labeled; drag the global navigator's left/right or top/bottom edges to adjust them
- Shift+drag: select segment (Fill/Smooth apply to it; Esc clears)
- `I` / `O`: set the segment start/end to the current playhead frame; when used alone, the other endpoint defaults to the timeline end/start; `U`: clear it
- Double-click: add keyframe anchor at that frame
- Drag a keyframe with a mouse or pen like a drawing pen: horizontal = time, vertical = selected channel value; the playhead and 3D scene follow
- Right-click keyframe: remove it

## Cartesian Interpolation

Select a timeline segment, click **Cartesian Interpolate**, choose one or more links, select a position-only or position-and-orientation constraint for each, and choose a linear or cubic method. Link targets are interpolated in world space and solved back to the floating-base pose and joint positions with IK. For jumps, full torso pose plus position-only ankle targets avoids overconstraining straight legs. The edit is transactional: if IK fails on any frame, the trajectory remains unchanged.

## Keyboard Shortcuts

- `Up` / `Down`: nudge the selected Edit Target's value at the current frame by `--nudge-step`
- `Left` / `Right`: step the current frame backward/forward by `--frame-step` (works with or without an Edit Target selected)
- `,` / `.`: select the previous/next Edit Target, wrapping within the current Joint, Base Position, or Base Orientation block
- `Space`: toggle trajectory playback (play/pause)
- `I` / `O`: set the selected segment's start/end to the current playhead frame (a lone `I` selects to the timeline end; a lone `O` selects from the timeline start; selecting a start after the existing end resets the end to the timeline end)
- `U`: clear the selected timeline segment
- `Esc`: unpin the pinned link, or clear the segment selection

## CLI Options

```
python backend/app.py --robot-urdf <path> [--port 5000] [--host 127.0.0.1] [--nudge-step 0.01] [--frame-step 1]
```

- `--robot-urdf`: Local path to a URDF file
- `--port`: Flask server port (default 5000)
- `--host`: Flask server host (default 127.0.0.1)
- `--nudge-step`: Value step applied by the Up/Down arrow keys on the selected edit target (default 0.01)
- `--frame-step`: Frame step applied by the Left/Right arrow keys (default 1)

## `.npz` Format

| Key | Shape | dtype | Meaning |
|---|---|---|---|
| `framerate` | `[]` (0-d) | float32 | trajectory framerate |
| `joint_names` | `(N,)` | str | ordered joint names |
| `joint_pos` | `(F, N)` | float32 | joint position sequence |
| `base_pos_w` | `(F, 3)` | float32 | base position, world xyz |
| `base_quat_w` | `(F, 4)` | float32 | base orientation, world wxyz |

## Trajectory Optimize

**Trajectory Optimize** optimizes the entire edited motion, regardless of the timeline
selection. Every keyframe anchor fixes all joints and the full base pose exactly.
Without anchors, the start and end frames are fixed boundary conditions and only the
interior frames are eligible. Clicking the button opens the parameter dialog
(`max_effort_scale`, terrain friction, target smoothing, glitch speed threshold). A validated result updates the editor as one
undo step. A solver/validation failure returns the best candidate as a preview:
**Keep** applies it as one undo step, **Dump** restores the original. Results are
discarded if the trajectory or scene changes during the request.

Install the optional dynamics dependencies in the backend environment:

```bash
pip install -r robot_trajectory_editor/backend/requirements-trajectory-optimize.txt
```

### Model

The decision variables are the configurations of the non-fixed frames, on the
original grid `dt = 1 / framerate`. Velocities and accelerations are central
differences of neighbouring configurations on the robot's configuration manifold
(Pinocchio `difference`), so the saved positions alone determine the motion. At
every interior frame the floating-base inverse dynamics (Pinocchio RNEA) must be
explained by joint torques within the URDF effort limits (times
**`max_effort_scale`**, API field `max_effort_scale`, default `1`, at least `1`) and by
unilateral point support at the terrain. Each support point uses a four-ray
pyramid inside the Coulomb cone with **terrain friction μ** (API field `friction`,
default `0.7`). Joint position and velocity limits, valid base orientation and
non-penetration of the loaded terrain (or the `z=0` ground) are enforced. This is a
point-support model without a sticking/sliding law. A boundary frame has no
dynamics constraint of its own, because its velocity is free.

The solver stays close to a *target* prepared from the edited motion, so
retargeting noise is not reproduced:

- **Glitch speed threshold** (API field `glitch_speed_scale`, default `1`, `0`
  disables): joint jumps faster than this multiple of the URDF velocity limit are
  re-interpolated from the nearest good frames, over at most 0.25 s.
- **Target smoothing** (API field `smoothing_hz`, default `8` Hz, `0` disables): a
  zero-phase second-order Butterworth low-pass of base position, base orientation
  and joints.

Fixed frames keep their exact poses; the filter's offset at each fixed frame is
blended back out over about one filter period. The response reports
`repaired_joint_samples`.

No contact labels are required. Support points are the lowest points of each
collision primitive (a cylinder's end rims, a capsule's end spheres, a box's
corners, a sphere's lowest point), so a flat-lying foot rests on its whole sole.
Contacts are inferred from the motion:

- **Initialization.** Free frames are shifted vertically by a smooth spline (knots
  about 0.3 s apart) so the support geometry near an upward-facing surface rests
  0.5 mm above it. Near a supported fixed frame the shift blends into that frame's
  own height. A fixed frame that hovers more than 1 mm is airborne: its neighbours
  fall from (or rise into) its height ballistically, as a drop or a hop.
- **Contact schedule.** Each frame schedules the support points that are within
  2 cm of an upward-facing surface and barely move toward or away from it (below
  0.3 m/s; sliding along it below 2 m/s is allowed, so a rolling back keeps its
  support while a swinging foot does not), or are already penetrating. Only each
  link's lowest points (within 1 mm) and links near the frame's lowest support
  (within 1 cm) qualify. The solver keeps scheduled points at most 0.9 mm above
  the terrain, inside the 1 mm contact margin, and all geometry at least 0.05 mm
  above it.
- **Support from the dynamics.** A free frame without such points whose centre of
  mass is clearly not in free fall (it needs an external force above 20% of body
  weight, from its acceleration smoothed over 0.1 s) schedules its lowest geometry
  within 25 cm, on links within 5 cm of the lowest one. The solver then has to
  bring that geometry to the terrain, for example hands catching a dive whose
  retargeted arms stop short of the floor.
- **Refresh.** The schedule is read again from the current trajectory after every
  multiplier update, so contact timing can move with the motion; a point stays
  scheduled while it is within reach, so the schedule does not flip on small changes.
- **Steadiness.** Contacts and breaks in a contact shorter than 0.05 s are removed,
  so feet are not held and released on alternate frames.

### Solver

Every constraint (base force, actuator effort, contact height, non-penetration at
frames and halfway between them, joint position and velocity limits) is a
least-squares term in physical units (1% of body weight, 1 Nm, 1 mm, 0.01 rad,
1 rad/s per unit), with an augmented-Lagrangian multiplier. Damped Gauss-Newton
(Levenberg-Marquardt) solves over the whole trajectory alternate with multiplier
updates, which drive the violations to zero without huge weights. A constraint kind
that stops improving gets a ten times larger penalty factor, up to 100. Every residual of frame *k* depends on frames *k-1..k+1* only, so the normal
equations are block-banded and are solved exactly by a block-tridiagonal
Cholesky factorization on frame pairs. Residuals and their Jacobians are evaluated
in parallel by forked worker processes (`TRAJECTORY_OPTIMIZE_WORKERS`, default up
to 12); BLAS threading is limited to one thread meanwhile, since the dense blocks
are small. The stages are:

1. A kinematic pass without dynamics puts the scheduled points on the terrain and
   clears penetration and limits.
2. The augmented-Lagrangian loop on the full problem, with two regularization
   terms: one stays close to the target poses (1 cm or 0.02 rad per unit, weight
   `1` in the first solve and `1e-2` after it) and one stays close to the target's
   accelerations (5 m/s² or rad/s² per unit, always at full weight). The latter
   stops the solver from shaking a limb to satisfy each frame's contacts: a frame-rate
   zigzag that central differences cannot see, or a slow 2-10 Hz wobble. Each inner solve gets at most a third of
   the remaining time, so the multipliers are updated before the budget runs out.

The support forces are re-allocated at every evaluation by a bounded least-squares
problem with the joint torques (solved exactly by an active set over saturated
actuators, with OSQP and BVLS as fallbacks). In the linearization the allocated
ray weights may follow the motion, damped relative to their own size, so loaded
contacts can absorb a landing while lightly loaded ones cannot reverse.

The solve stops at the first iterate that passes validation, or at the 45-second
budget (a validation reserve is kept). A timeout is a failure: its candidate is
only a preview and is never applied without **Keep**.

### Validation

The float32 trajectory that the editor keeps is checked independently with live
contacts: every nearest point and support point within 1 mm of the terrain may
carry support. The checks are base dynamics and actuator effort at every interior
frame, joint position and velocity limits, and non-penetration at every frame and
at 8 poses per interval (Pinocchio `interpolate`), plus an explicit surface-crossing
check on loaded meshes. Tolerances: base dynamics 0.1% of robot weight in N (Nm
for moments), with a `1e-3` floor; effort `1e-4` Nm; position `1e-6` rad;
velocity `1e-6` rad/s; penetration `1e-5` m; quaternion norm error `1e-6` on input.

Every run returns a plain-text `report` (shown in the dialog with **Copy report**):
the motion, robot, terrain and parameters, the solver time and stop reason, and
each violated check with its worst value, frame, location (joint, geometry, or
unsupported base force/moment), tolerance and the number of frames over it. Checks
are ordered by their size on the solver's physical scales, not by the ratio to
their tolerances, which differ by orders of magnitude (`1e-4` Nm of effort against
0.3 N of base force), so the headline names the physically largest problem.

The model must provide inertial data for physical bodies and collision geometry,
with a one-to-one mapping between trajectory joints and movable URDF joints.
Revolute, continuous and prismatic joints are supported; mimic and other joint
types report an unsupported-model diagnostic. Empty fixed sensor/coordinate frames
may omit inertial data. Visual meshes are not substituted for collision geometry.
The currently edited terrain is sent in world coordinates. A watertight mesh with
positive winding volume also excludes its interior; a negative-volume mesh
encloses free space (for example a scanned room), so its interior is not solid.
Fixed keyframes that violate joint limits or penetrate the terrain are rejected
before solving.

### Measurements

`scripts/benchmark_trajectory_optimize.py` runs Trajectory Optimize on a real NPZ and
logs the solver progress as JSON lines; `scripts/audit_feasibility.py` runs the
validator on the unmodified motion. Both take `--terrain` (STL or OBJ) and
`--frames`:

```bash
python scripts/benchmark_trajectory_optimize.py motion.npz robot.urdf --terrain terrain.stl --frames 750
python scripts/audit_feasibility.py motion.npz robot.urdf --terrain terrain.stl
```

Measured on the development machine (14 cores), μ = 0.7, 45 s budget:

Nokov `diveroll4` (`20251106_diveroll4_simpleLab_noWall`, flat ground, endpoints
fixed, 751 frames, defaults): the reference jitters by 0.83° RMS (high-pass, max
41°, a 1.3 rad one-frame wrist jump at frame 444); 62 joint samples are repaired.
In the reference the hands stay 12-17 cm above the floor while the body is
clearly held up (frames 342-368, centre-of-mass acceleration about -2 m/s²), so
without inferred support the body would have to fall freely and hit the floor at
about 2 m/s. With support from the dynamics the hands and then the back carry it.
With 45 s, 236 of 262 frames before the dive (0-261), 37 of 192 in the dive roll
(262-453) and 224 of 267 standing frames (454-720) pass every check; the solver adds
0.27° RMS of 2-10 Hz wobble to the standing part (with the weaker 20 rad/s²
acceleration term: 96, 9 and 134 frames, 0.44°). The final frame hovers 17.7 mm,
so the fixed endpoint forces a 2 cm hop in the last frames. The take-off and the
dive itself still exceed the waist, hip and knee limits, and the G1 collision model
has no hands, forearms, thighs or shins: hand support is carried by the wrist
cylinders.

Earlier measurements (before target smoothing):

| Motion | Frames | Result |
|---|---:|---|
| Kengo `walk_0052`, flat ground, all frames free | 200 | Validated in 16 s (dynamics 0.024 N, tolerance 0.41 N) |
| Kengo `walk_0052`, flat ground, endpoints fixed | 200 | Fails: the fixed first frame hovers 35 mm, so the robot must drop 35 mm and land; effort 29 Nm over the limit at the landing |
| Kengo `walk_0052`, flat ground, all frames free | 750 | Not converged in 45 s (dynamics 257 N); 240 s reaches 4 N in the solver model |
| Kengo `walk_0052`, flat ground, endpoints fixed | 750 | Fails: dynamics 77 N, effort 33 Nm |
| Nokov `diveroll4`, flat ground, endpoints fixed | 751 | Fails: the reference has a 1.3 rad one-frame wrist jump and no contact during parts of the roll |
| Nokov `highClimbRight1`, 50 cm boxes, endpoints fixed | 1199 | Fails; mesh-terrain evaluations are slower (26 iterations in 45 s) |

The unmodified references are far from the model: the audit of Kengo
`walk_0052` (200 frames) finds 1581 N of base residual, because the retargeted feet
hover or stand on tilted edges. Fixed hovering endpoints are a recurring obstacle:
the solver must turn them into a drop or a hop.
