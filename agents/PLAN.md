# Robot Trajectory Editor — Implementation Plan

> Source requirements: `agents/TASK.md`. Decisions confirmed with user on 2026-07-30.

## 1. Confirmed Decisions

| Decision | Choice |
|---|---|
| Frontend | React + TypeScript + Vite, Three.js for 3D, zustand for state |
| Backend | Flask + numpy (`.npz` IO), CLI entry |
| File UX | Browser upload (file picker) / download (Save-As dialog) |
| IK | `closed-chain-ik@0.0.3` (urdf-loader compatible), behind an `IkSolver` interface with custom CCD fallback |
| Timeline | Dense per-frame storage (1:1 with `.npz`) + keyframe anchors preserved by Fill/Smooth |

## 2. Test Assets

- **Local (this Mac):** `/Users/leo/NutstoreFiles/2Projects/isaacSim/instinctlab/source/instinctlab/instinctlab/assets/resources/unitree_g1/urdf/g1_29dof_torsobase_popsicle.urdf`
  - Verified: root link = `pelvis` (floating-base joint commented out); meshes referenced by **relative** paths `../meshes/*.STL` (uppercase ext) → backend asset route must resolve relative to the URDF's directory, case-sensitively.
- **Remote (`ssh ziwen-galaxea-desktop-rustdesk`):** `/home/ziwen/Projects/project-instinct/galaxea_robot_assets/galaxea_robot_assets/kengo_with_fist/urdf/kengo_with_fist.urdf`
  - Secondary validation target; copy asset dir locally when needed.
- Sample `.npz` trajectory: generate synthetic fixture matching the spec if user doesn't provide one.

## 3. Repository Layout

```
robot-trajectory-editor/
├── agents/{TASK.md, PLAN.md}
├── backend/
│   ├── app.py                 # Flask entry + CLI: --robot-urdf, --port (5000), --host
│   ├── npz_io.py              # numpy <-> JSON per spec below
│   ├── tests/test_npz_io.py   # pytest roundtrip fixtures
│   └── requirements.txt       # flask, numpy, pytest
└── frontend/
    ├── package.json / vite.config.ts (proxy /api→Flask) / tsconfig.json / index.html
    └── src/
        ├── main.tsx, App.tsx           # layout: left panels | viewport | right panels; bottom timeline
        ├── state/
        │   ├── Trajectory.ts           # ★ dedicated robot-state class (task requirement)
        │   └── store.ts                # zustand: currentFrame, selection, playback, pinned link…
        ├── three/
        │   ├── Viewport.ts             # scene, camera, OrbitControls, z=0 ground plane + grid
        │   ├── RobotModel.ts           # urdf-loader wrapper; pose application from frame
        │   ├── IkSolver.ts             # interface + closed-chain-ik impl (+ CCD fallback)
        │   ├── DragController.ts       # raycast drag, pin/unpin logic
        │   └── Terrain.ts              # OBJ wrapper (OBJLoader/OBJExporter)
        ├── components/
        │   ├── LeftPanel.tsx           # Load/Save Trajectory, Load/Save/Edit Terrain, robot select
        │   ├── TargetPanel.tsx         # edit-target selection: joint / base pos / base quat
        │   ├── StatePanel.tsx          # right: joint sliders + base pos/quat numeric inputs
        │   ├── OpsPanel.tsx            # right: Fill, Smooth, Play, Pause
        │   ├── Timeline.tsx            # bottom: ruler, curve, keyframes, segment select
        │   ├── TerrainEditor.tsx       # popup: crop / downsample / move / rotate
        │   └── PinnedBanner.tsx        # "link X pinned" alert overlay
        └── api/client.ts
```

## 4. `.npz` Format (per TASK.md)

| Key | Shape | dtype | Meaning |
|---|---|---|---|
| `framerate` | `[]` (0-d) | float32 | trajectory framerate |
| `joint_names` | `(joint_count,)` | str | ordered joint names |
| `joint_pos` | `(frame_count, joint_count)` | float32 | joint position sequence |
| `base_pos_w` | `(frame_count, 3)` | float32 | base position, world, xyz |
| `base_quat_w` | `(frame_count, 4)` | float32 | base orientation, world, **wxyz** |

Backend must handle the 0-d `framerate` (`float(arr)`) and always write float32.

## 5. Backend (Flask + numpy)

CLI: `python backend/app.py --robot-urdf <path> [--port 5000] [--host 127.0.0.1] [--nudge-step 0.01] [--frame-step 1]`

| Endpoint | Purpose |
|---|---|
| `GET /api/config` | launch-time editor config (`nudge_step` from `--nudge-step`, default 0.01; `frame_step` from `--frame-step`, default 1) |
| `GET /api/robot/info` | whether CLI provided a URDF (drives robot-selector visibility) |
| `GET /api/robot/urdf` + `GET /api/robot/assets/<path>` | serve URDF + meshes; resolves relative & `package://` mesh paths against the URDF dir (verified pattern `../meshes/*.STL`) |
| `POST /api/robot/upload` | no CLI URDF: upload folder (`.urdf` + meshes, `webkitdirectory`) → temp dir, served as above |
| `POST /api/trajectory/parse` | `.npz` upload → JSON payload |
| `POST /api/trajectory/serialize` | JSON payload → numpy → `.npz` download (browser Save-As) |

Terrain mesh files (`.obj` and `.stl`) are client-side only (Three.js `OBJLoader`/`STLLoader`; export always as `.obj`).

## 6. `Trajectory` Class (frontend, dedicated — task requirement)

Single source of truth; dense storage in `Float32Array`s (1:1 with `.npz`); editor-only `keyframes: Set<number>` (anchors, not persisted).

- Fields: `framerate`, `jointNames`, `jointPos`, `basePoseW`, `baseQuatW`, `keyframes`
- IO: `fromJSON` / `toJSON` (↔ backend endpoints)
- Frame ops: `getFrame(i)`, `setJointValue(frame, jointIdx, v)`, `setBasePose(frame, pos, quat)`
- Keyframes: `insertKeyframe / removeKeyframe / moveKeyframe`
- Range ops: `fillChannelRange(start, end, channel, value)`; `smoothRange(start, end, channel, sigma?)` — Gaussian low-pass filter over the segment samples (removes high-frequency jitter, preserves motion shape; segment endpoints and keyframe anchors kept, smoothing strength tapered at the edges for continuity with the untouched frames); quaternions: hemisphere-continuity fix → per-component filter → renormalize
- Channel type: `{kind:'joint', index} | {kind:'basePos'} | {kind:'baseQuat'}`
- Unit tests (vitest): roundtrip, fill/smooth, quaternion continuity

## 7. Main 3D Viewport

- Three.js + OrbitControls; grid + flat z=0 ground plane when no terrain loaded.
- Robot via `urdf-loader@0.13.1`; URDF root link (`pelvis` for G1) driven by `base_pos_w`/`base_quat_w` through a root group; joints set per current frame; updates on scrub/edit/playback.
- **Drag interactions** (both modes use IK via `closed-chain-ik`, isolated behind `IkSolver` interface):
  - *Default mode:* drag any body link → link follows cursor (goal in camera-facing plane at grab depth); IK solves joints on the base→link chain. Dragging the base link moves the base pose. (Per TASK.md L62: default mode drags change joint positions.)
  - *Pinned mode (double-click a link):* link world pose frozen; mesh highlighted (emissive); `PinnedBanner` alert shown; dragging moves the **base** while IK re-solves the chain to keep the link fixed; double-click again / Esc unpins.
- All drag results write into `Trajectory` at the current frame and refresh StatePanel + Timeline.

## 8. UI Panels

- **Left (files):** Load Trajectory (picker→`/api/trajectory/parse`), Save Trajectory (serialize→Save-As), Load/Save Terrain (`.obj`), Edit Terrain (popup), robot selector (only when no CLI URDF → folder upload).
- **Left (target):** select edit target — joint by name, base position, or base orientation.
- **Right (state):** per-joint sliders + numeric inputs (URDF limits), base pos xyz, base quat wxyz; edits write to current frame.
- **Right (ops):** Fill (selected channel's current-frame value → segment or whole of that channel only), Smooth (Gaussian low-pass on selected channel → segment or whole), Play/Pause (playback at `framerate`).
- **Keyboard:** `Up`/`Down` nudge the selected edit target at the current frame by `nudgeStep`; `Left`/`Right` step the current frame by `frameStep` (clamped, no target needed); `Space` toggles play/pause. Steps come from backend `--nudge-step`/`--frame-step` via `/api/config`; arrows/space are ignored while a form control is focused.

## 9. Timeline (bottom)

- Frame ruler + draggable playhead; curve view of selected channel (joint: 1 curve; base pos: xyz; base quat: wxyz; component toggles).
- Keyframes as diamond markers: click empty → add at frame w/ current value; drag horizontal → retime; drag vertical → change value; right-click/Del → remove; anchors preserved by Smooth/Fill.
- Segment selection: Shift+drag on ruler → highlighted range consumed by Fill/Smooth; Esc clears.

## 10. Terrain Editor (popup)

Own Three.js viewport: **move/rotate** via TransformControls gizmos; **crop** via X/Y/Z min-max bounding-box sliders with live preview; **downsample** via ratio slider (`SimplifyModifier`). Closing commits the mesh to the main viewport; Save Terrain exports `.obj`.

## 11. Build Order

1. Backend scaffold: CLI, npz parse/serialize, pytest roundtrip (synthetic fixture per §4)
2. Frontend scaffold: Vite/React/TS/zustand/three, layout shell
3. `Trajectory` class + vitest suite
4. Viewport: ground plane, URDF loading via backend assets (validate early with local G1), pose-from-frame
5. Left/right panels: load/save trajectory, StatePanel, TargetPanel
6. Timeline: scrub, curve, keyframes, segment select, Play/Pause
7. Fill / Smooth ops
8. Viewport dragging + IK: default mode, then pinned mode + banner/highlight
9. Terrain load/save + Terrain Editor popup
10. Polish: error/loading states, README (launch instructions), validate with kengo URDF

## 12. Testing

- `backend/tests/test_npz_io.py`: synthetic arrays per §4 → serialize → parse → assert equality, dtypes, 0-d framerate.
- `frontend`: vitest for `Trajectory` (fill/smooth/quat, keyframe ops, JSON roundtrip).
- Manual E2E: launch with `--robot-urdf <local G1 path>`, synthetic + user-provided `.npz`.

## 13. Risks & Mitigations

- `closed-chain-ik@0.0.3` maturity → `IkSolver` interface; custom CCD fallback.
- URDF mesh path quirks (relative `../meshes`, `package://`, case-sensitive `.STL`) → backend asset route; validate in step 4.
- Large trajectories → typed arrays, version-counter rerenders, decimated curve rendering.
