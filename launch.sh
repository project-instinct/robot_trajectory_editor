#!/usr/bin/env bash
set -e

URDF_ARG=""
NUDGE_STEP_ARG=""
FRAME_STEP_ARG=""
PORT=5000

while [[ $# -gt 0 ]]; do
    case "$1" in
        --robot-urdf) URDF_ARG="--robot-urdf $2"; shift 2 ;;
        --nudge-step) NUDGE_STEP_ARG="--nudge-step $2"; shift 2 ;;
        --frame-step) FRAME_STEP_ARG="--frame-step $2"; shift 2 ;;
        --port) PORT="$2"; shift 2 ;;
        *) echo "Unknown: $1"; exit 1 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BACKEND_DIR="$SCRIPT_DIR/robot_trajectory_editor/backend"
FRONTEND_DIR="$SCRIPT_DIR/robot_trajectory_editor/frontend"

# Locate a Python interpreter (python3 preferred, python as fallback).
PYTHON="$(command -v python3 || command -v python || true)"
if [ -z "$PYTHON" ]; then
    echo "Error: Python 3 is not installed or not on PATH."
    echo "Please install Python 3.9+ (https://www.python.org/) and re-run ./launch.sh"
    exit 1
fi

# Check backend dependencies; prompt to install if missing.
if ! "$PYTHON" -c "import flask, numpy" 2>/dev/null; then
    echo "Warning: backend dependencies (flask, numpy) are missing from this Python environment."
    echo "  Install them with:  pip install -r \"$BACKEND_DIR/requirements.txt\""
    read -r -p "Run this install command now? [y/N] " ans
    if [[ "$ans" == "y" || "$ans" == "Y" ]]; then
        pip install -r "$BACKEND_DIR/requirements.txt"
    else
        echo "Aborting: backend dependencies required."
        exit 1
    fi
fi

# Check frontend dependencies; prompt to install if missing.
if [ ! -d "$FRONTEND_DIR/node_modules" ]; then
    echo "Warning: frontend dependencies (node_modules) not found."
    echo "  Install them with:  npm install"
    read -r -p "Run this install command now? [y/N] " ans
    if [[ "$ans" == "y" || "$ans" == "Y" ]]; then
        (cd "$FRONTEND_DIR" && npm install)
    else
        echo "Aborting: frontend dependencies required."
        exit 1
    fi
fi

cleanup() {
    echo ""
    echo "Shutting down..."
    kill $BACKEND_PID 2>/dev/null || true
    kill $FRONTEND_PID 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

echo "=== Starting Backend (Flask :$PORT) ==="
cd "$BACKEND_DIR"
"$PYTHON" app.py $URDF_ARG $NUDGE_STEP_ARG $FRAME_STEP_ARG --port "$PORT" &
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
