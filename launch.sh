#!/usr/bin/env bash
set -e

URDF_ARG=""
PORT=5000

while [[ $# -gt 0 ]]; do
    case "$1" in
        --robot-urdf) URDF_ARG="--robot-urdf $2"; shift 2 ;;
        --port) PORT="$2"; shift 2 ;;
        *) echo "Unknown: $1"; exit 1 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BACKEND_DIR="$SCRIPT_DIR/robot_trajectory_editor/backend"
FRONTEND_DIR="$SCRIPT_DIR/robot_trajectory_editor/frontend"

cleanup() {
    echo ""
    echo "Shutting down..."
    kill $BACKEND_PID 2>/dev/null || true
    kill $FRONTEND_PID 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

echo "=== Starting Backend (Flask :$PORT) ==="
source ~/miniconda3/etc/profile.d/conda.sh
conda activate omniretargeting
cd "$BACKEND_DIR"
python app.py $URDF_ARG --port "$PORT" &
BACKEND_PID=$!

echo "=== Starting Frontend (Vite :5173) ==="
cd "$FRONTEND_DIR"
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"
npm run dev &
FRONTEND_PID=$!

echo ""
echo "Frontend: http://localhost:5173"
echo "Backend:  http://localhost:$PORT"
echo "Press Ctrl+C to stop both."
wait
