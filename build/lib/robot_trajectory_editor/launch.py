"""Entry point for `python -m robot_trajectory_editor.launch` and `robot-trajectory-editor`."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
BACKEND_DIR = os.path.join(PACKAGE_DIR, "backend")
FRONTEND_DIR = os.path.join(PACKAGE_DIR, "frontend")
FRONTEND_DIST_DIR = os.path.join(FRONTEND_DIR, "dist")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m robot_trajectory_editor.launch",
        description="Launch the Robot Trajectory Editor web application.",
    )
    parser.add_argument(
        "--robot-urdf",
        "--robot_urdf_path",
        dest="robot_urdf_path",
        default=None,
        help="Path to robot URDF file (local path or package://<python_pkg>/<relpath>).",
    )
    parser.add_argument("--port", type=int, default=5000, help="Server port (default: 5000).")
    parser.add_argument("--host", default="127.0.0.1", help="Server host (default: 127.0.0.1).")
    parser.add_argument(
        "--nudge-step",
        type=float,
        default=0.01,
        help="Value step applied by Up/Down arrow keys on the selected edit target (default: 0.01).",
    )
    parser.add_argument(
        "--frame-step",
        type=int,
        default=1,
        help="Frame step applied by Left/Right arrow keys (default: 1).",
    )
    parser.add_argument(
        "--file-explorer-side",
        choices=("client", "server"),
        default="client",
        help="Where file dialogs browse/save files: 'client' (browser) or 'server' (in-page).",
    )
    parser.add_argument(
        "--dev",
        action="store_true",
        help="Run Vite dev server (npm run dev on :5173) alongside Flask instead of serving frontend/dist/.",
    )
    parser.add_argument("--debug", action="store_true", help="Enable Flask debug mode.")
    return parser


def _launch_dev(args: argparse.Namespace) -> None:
    if not os.path.isdir(os.path.join(FRONTEND_DIR, "node_modules")):
        print(
            f"Error: frontend dependencies not found in {FRONTEND_DIR}/node_modules.\n"
            f"Run: (cd \"{FRONTEND_DIR}\" && npm install)",
            file=sys.stderr,
        )
        sys.exit(1)

    backend_cmd = [
        sys.executable,
        os.path.join(BACKEND_DIR, "app.py"),
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--nudge-step",
        str(args.nudge_step),
        "--frame-step",
        str(args.frame_step),
        "--file-explorer-side",
        args.file_explorer_side,
    ]
    if args.robot_urdf_path:
        backend_cmd.extend(["--robot-urdf", args.robot_urdf_path])
    if args.debug:
        backend_cmd.append("--debug")

    procs: list[subprocess.Popen] = []

    def _shutdown(_signum=None, _frame=None) -> None:
        for proc in procs:
            if proc.poll() is None:
                proc.terminate()
        for proc in procs:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print(f"=== Starting Backend (Flask http://{args.host}:{args.port}) ===")
    backend_proc = subprocess.Popen(backend_cmd, cwd=BACKEND_DIR)
    procs.append(backend_proc)

    print("=== Starting Frontend (Vite http://localhost:5173) ===")
    frontend_proc = subprocess.Popen(["npm", "run", "dev"], cwd=FRONTEND_DIR)
    procs.append(frontend_proc)

    print("\nFrontend: http://localhost:5173")
    print(f"Backend:  http://{args.host}:{args.port}")
    print("Press Ctrl+C to stop both.")

    try:
        while True:
            for proc in procs:
                if proc.poll() is not None:
                    _shutdown()
            time.sleep(0.2)
    except KeyboardInterrupt:
        _shutdown()


def _launch_single_server(args: argparse.Namespace) -> None:
    index_html = os.path.join(FRONTEND_DIST_DIR, "index.html")
    if not os.path.isfile(index_html):
        print(
            f"Error: built frontend not found at {index_html}.\n"
            f"Build it with: (cd \"{FRONTEND_DIR}\" && npm install && npm run build)\n"
            "Or run with --dev to start the Vite dev server.",
            file=sys.stderr,
        )
        sys.exit(1)

    if BACKEND_DIR not in sys.path:
        sys.path.insert(0, BACKEND_DIR)

    import app as backend_app

    try:
        backend_app.configure_app(
            robot_urdf_path=args.robot_urdf_path,
            nudge_step=args.nudge_step,
            frame_step=args.frame_step,
            frontend_dist_dir=FRONTEND_DIST_DIR,
            file_explorer_side=args.file_explorer_side,
        )
    except (ValueError, FileNotFoundError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)

    display_host = "localhost" if args.host in ("127.0.0.1", "0.0.0.0") else args.host
    print(f"Robot Trajectory Editor: http://{display_host}:{args.port}")
    backend_app.app.run(host=args.host, port=args.port, debug=args.debug)


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    if args.dev:
        _launch_dev(args)
    else:
        _launch_single_server(args)


if __name__ == "__main__":
    main()
