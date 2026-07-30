# Robot Trajectory Editor

Web-based editor for robot trajectories. Create, edit, and visualize robot joint/pose sequences in 3D.

## Quick Start

```bash
# Launch both backend + frontend
./launch.sh [--robot-urdf /path/to/robot.urdf] [--port 5000]
```

Then open http://localhost:5173.

### Manual launch

```bash
cd robot_trajectory_editor/backend
pip install -r requirements.txt
python app.py --robot-urdf /path/to/robot.urdf --port 5000
```

### 2. Start Frontend

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
- **IK**: Custom CCD solver behind an IkSolver interface in `frontend/src/three/IkSolver.ts`

## CLI Options

```
python backend/app.py --robot-urdf <path> [--port 5000] [--host 127.0.0.1]
```

- `--robot-urdf`: Local path to a URDF file
- `--port`: Flask server port (default 5000)
- `--host`: Flask server host (default 127.0.0.1)

## `.npz` Format

| Key | Shape | dtype | Meaning |
|---|---|---|---|
| `framerate` | `[]` (0-d) | float32 | trajectory framerate |
| `joint_names` | `(N,)` | str | ordered joint names |
| `joint_pos` | `(F, N)` | float32 | joint position sequence |
| `base_pose_w` | `(F, 3)` | float32 | base position, world xyz |
| `base_quat_w` | `(F, 4)` | float32 | base orientation, world wxyz |
