import argparse
import glob
import importlib.util
import io
import math
import os
import tempfile
import shutil
import sys
import threading

from flask import Flask, request, jsonify, send_file, send_from_directory, abort
from npz_io import parse_npz, serialize_npz
from trajectory_optimize import TrajectoryOptimizeError, optimize_trajectory

app = Flask(__name__)

ROBOT_URDF_PATH: str | None = None
UPLOADED_ROBOT_DIR: str | None = None
CURRENT_URDF_REL: str | None = None
FRONTEND_DIST_DIR: str | None = None
FILE_EXPLORER_SIDE: str = "client"
NUDGE_STEP: float = 0.01
# One Trajectory Optimize at a time: each run forks a worker pool and sets the
# process-wide BLAS thread limit, which overlapping runs in the threaded
# server would fight over (and forking beside a running solve can deadlock).
OPTIMIZE_LOCK = threading.Lock()
FRAME_STEP: int = 1


def get_urdf_dir() -> str | None:
    if ROBOT_URDF_PATH and os.path.isfile(ROBOT_URDF_PATH):
        return os.path.dirname(os.path.abspath(ROBOT_URDF_PATH))
    if UPLOADED_ROBOT_DIR:
        return UPLOADED_ROBOT_DIR
    return None


def _allowed_roots() -> list[str]:
    """Directories asset requests are allowed to resolve into."""
    if ROBOT_URDF_PATH and os.path.isfile(ROBOT_URDF_PATH):
        urdf_dir = os.path.dirname(os.path.abspath(ROBOT_URDF_PATH))
        # urdf_dir and its parent (robot package root, e.g. for ../meshes/x.STL)
        return [urdf_dir, os.path.dirname(urdf_dir)]
    if UPLOADED_ROBOT_DIR:
        return [os.path.abspath(UPLOADED_ROBOT_DIR)]
    return []


def _is_within(path: str, roots: list[str]) -> bool:
    ap = os.path.abspath(path)
    for root in roots:
        try:
            if os.path.commonpath([ap, root]) == root:
                return True
        except ValueError:
            continue
    return False


def _safe_upload_relative_path(filename: str) -> str:
    """Return a normalized relative upload path or raise for path traversal."""
    # Browsers normally submit POSIX paths, but normalize Windows separators too.
    normalized = os.path.normpath(filename.replace("\\", "/"))
    if normalized in ("", ".") or os.path.isabs(normalized):
        raise ValueError("Uploaded filenames must be relative paths")
    if normalized == ".." or normalized.startswith(f"..{os.sep}"):
        raise ValueError("Uploaded filenames may not escape the upload directory")
    return normalized


def resolve_package_uri(uri: str) -> str | None:
    """Resolve package://<python_pkg>/<relpath> to an existing local file, or None."""
    if not uri.startswith("package://"):
        return None
    rest = uri[len("package://"):]
    pkg_name, sep, rel_path = rest.partition("/")
    if not pkg_name or not sep or not rel_path:
        return None
    try:
        spec = importlib.util.find_spec(pkg_name)
    except (ImportError, ValueError):
        return None
    if spec is None:
        return None
    if spec.submodule_search_locations:
        roots = [os.path.abspath(p) for p in spec.submodule_search_locations]
    elif spec.origin:
        roots = [os.path.dirname(os.path.abspath(spec.origin))]
    else:
        return None
    for root in roots:
        candidate = os.path.normpath(os.path.join(root, rel_path))
        if _is_within(candidate, [root]) and os.path.isfile(candidate):
            return candidate
    return None


def resolve_urdf_path(urdf_path: str) -> str:
    """Resolve a local file path or package://<python_pkg>/<relpath> URDF URI."""
    if not urdf_path.startswith("package://"):
        return os.path.abspath(urdf_path)
    rest = urdf_path[len("package://"):]
    pkg_name, sep, rel_path = rest.partition("/")
    if not pkg_name or not sep or not rel_path:
        raise ValueError(f"Invalid package:// URDF path: {urdf_path}")
    resolved = resolve_package_uri(urdf_path)
    if resolved is None:
        raise FileNotFoundError(
            f"Could not resolve '{urdf_path}': check that Python package '{pkg_name}' is installed "
            f"and contains '{rel_path}'."
        )
    return resolved


def resolve_mesh_path(mesh_href: str) -> str | None:
    if mesh_href.startswith("package://"):
        pkg_resolved = resolve_package_uri(mesh_href)
        if pkg_resolved is not None:
            return pkg_resolved

    urdf_dir = get_urdf_dir()
    if not urdf_dir:
        return None

    if mesh_href.startswith("package://"):
        parts = mesh_href[len("package://"):].split("/", 1)
        if len(parts) == 2:
            mesh_href = parts[1]
        else:
            mesh_href = parts[0]

    roots = _allowed_roots()
    parent_dir = os.path.dirname(urdf_dir)
    urdf_file_dir = None
    if CURRENT_URDF_REL:
        urdf_file_dir = os.path.dirname(os.path.join(urdf_dir, CURRENT_URDF_REL))

    candidates = [
        os.path.normpath(os.path.join(urdf_dir, mesh_href)),
        os.path.normpath(os.path.join(parent_dir, mesh_href)),
    ]
    if urdf_file_dir:
        candidates.append(os.path.normpath(os.path.join(urdf_file_dir, mesh_href)))

    for c in candidates:
        if _is_within(c, roots) and os.path.isfile(c):
            return c

    filename = os.path.basename(mesh_href)
    found = glob.glob(os.path.join(urdf_dir, "**", filename), recursive=True)
    if found:
        return found[0]

    return None


@app.route("/api/config")
def editor_config():
    return jsonify({
        "nudge_step": NUDGE_STEP,
        "frame_step": FRAME_STEP,
        "file_explorer_side": FILE_EXPLORER_SIDE,
    })


@app.route("/api/robot/info")
def robot_info():
    return jsonify({
        "has_urdf": (ROBOT_URDF_PATH is not None and os.path.isfile(ROBOT_URDF_PATH)) or UPLOADED_ROBOT_DIR is not None,
        "urdf_path": ROBOT_URDF_PATH,
    })


def _find_urdfs_in_dir(directory: str) -> list[str]:
    candidates = sorted(glob.glob(os.path.join(directory, "**", "*.urdf"), recursive=True) +
                        glob.glob(os.path.join(directory, "**", "*.URDF"), recursive=True))
    return candidates


def _current_urdf_dir() -> str | None:
    if ROBOT_URDF_PATH and os.path.isfile(ROBOT_URDF_PATH):
        return os.path.dirname(os.path.abspath(ROBOT_URDF_PATH))
    return UPLOADED_ROBOT_DIR


@app.route("/api/robot/urdfs")
def list_urdfs():
    urdf_dir = _current_urdf_dir()
    if not urdf_dir:
        return jsonify({"urdfs": []})

    candidates = _find_urdfs_in_dir(urdf_dir)
    result = []
    for c in candidates:
        rel = os.path.relpath(c, urdf_dir)
        result.append({"path": rel, "name": os.path.basename(c)})

    return jsonify({"urdfs": result, "root_dir": urdf_dir})


@app.route("/api/robot/urdf")
def serve_urdf():
    global CURRENT_URDF_REL
    urdf_rel = request.args.get("path")

    if urdf_rel:
        urdf_dir = _current_urdf_dir()
        if not urdf_dir:
            return abort(404, "No robot directory set")
        urdf_path = os.path.normpath(os.path.join(urdf_dir, urdf_rel))
        if not _is_within(urdf_path, [os.path.abspath(urdf_dir)]) or not os.path.isfile(urdf_path):
            return abort(404, f"URDF file not found: {urdf_rel}")
        CURRENT_URDF_REL = urdf_rel
    else:
        if ROBOT_URDF_PATH and os.path.isfile(ROBOT_URDF_PATH):
            urdf_path = ROBOT_URDF_PATH
            CURRENT_URDF_REL = None
        elif UPLOADED_ROBOT_DIR:
            candidates = _find_urdfs_in_dir(UPLOADED_ROBOT_DIR)
            if not candidates:
                return abort(404, "No URDF files found in uploaded directory")
            urdf_path = candidates[0]
            CURRENT_URDF_REL = os.path.relpath(urdf_path, UPLOADED_ROBOT_DIR)
        else:
            return abort(404, "No robot URDF provided")

    if urdf_path.startswith("package://"):
        return jsonify({"error": "package:// URDF not supported for serving; use local path"}), 400

    return send_file(urdf_path, mimetype="application/xml")


@app.route("/api/robot/assets/<path:asset_path>")
def serve_asset(asset_path: str):
    resolved = resolve_mesh_path(asset_path)
    if resolved is None:
        return abort(404, "No robot directory set")

    if not os.path.isfile(resolved):
        return abort(404, f"Asset not found: {asset_path}")

    return send_file(resolved)


@app.route("/api/robot/<path:rest>")
def serve_robot_catchall(rest: str):
    if rest.startswith("assets/") or rest in ("urdf", "urdfs", "info", "upload"):
        return abort(404)
    resolved = resolve_mesh_path(rest)
    if resolved and os.path.isfile(resolved):
        return send_file(resolved)
    return abort(404, f"Asset not found: {rest}")


@app.route("/api/robot/upload", methods=["POST"])
def upload_robot():
    global UPLOADED_ROBOT_DIR

    uploaded_files = request.files.getlist("files")
    if not uploaded_files:
        return jsonify({"error": "No files uploaded"}), 400

    temp_dir = tempfile.mkdtemp(prefix="robot_urdf_")
    try:
        saved_count = 0
        for f in uploaded_files:
            if not f.filename:
                continue
            rel_path = _safe_upload_relative_path(f.filename)
            dest = os.path.join(temp_dir, rel_path)
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            f.save(dest)
            saved_count += 1
        if saved_count == 0:
            raise ValueError("No named files uploaded")
    except ValueError as e:
        shutil.rmtree(temp_dir, ignore_errors=True)
        return jsonify({"error": str(e)}), 400

    if UPLOADED_ROBOT_DIR and os.path.isdir(UPLOADED_ROBOT_DIR):
        shutil.rmtree(UPLOADED_ROBOT_DIR, ignore_errors=True)
    UPLOADED_ROBOT_DIR = temp_dir

    return jsonify({"status": "ok", "dir": temp_dir})


@app.route("/api/trajectory/parse", methods=["POST"])
def parse_trajectory_endpoint():
    if "file" not in request.files:
        return jsonify({"error": "No file uploaded"}), 400

    file = request.files["file"]
    try:
        data = parse_npz(file.read())
        return jsonify(data)
    except Exception as e:
        return jsonify({"error": str(e)}), 400


@app.route("/api/trajectory/optimize", methods=["POST"])
def optimize_trajectory_endpoint():
    payload = request.get_json()
    if not isinstance(payload, dict):
        return jsonify({"error": "Trajectory Optimize requires a JSON object."}), 400
    root = _current_urdf_dir()
    relative = payload.get("urdf_path")
    if relative is not None:
        if not isinstance(relative, str) or not root:
            return jsonify({"error": "No robot directory is loaded."}), 400
        path = os.path.abspath(os.path.join(root, relative))
        if not _is_within(path, [os.path.abspath(root)]) or not os.path.isfile(path):
            return jsonify({"error": "Selected robot URDF is unavailable."}), 400
    elif ROBOT_URDF_PATH and os.path.isfile(ROBOT_URDF_PATH):
        path = ROBOT_URDF_PATH
    else:
        return jsonify({"error": "Select a robot URDF before Trajectory Optimize."}), 400
    try:
        terrain = payload.get("terrain_obj")
        if terrain is not None and not isinstance(terrain, str):
            raise TrajectoryOptimizeError("Terrain must be OBJ text or null for the default ground.")
        friction = payload.get("friction", 0.7)
        if type(friction) not in (int, float) or not math.isfinite(friction) or friction < 0:
            raise TrajectoryOptimizeError("Terrain friction must be a finite nonnegative number.")
        max_effort_scale = payload.get("max_effort_scale", 1.0)
        if type(max_effort_scale) not in (int, float) or not math.isfinite(max_effort_scale) or max_effort_scale < 1.0:
            raise TrajectoryOptimizeError("max_effort_scale must be a finite number at least 1.")
        options = {key: payload[key] for key in ("smoothing_hz", "glitch_speed_scale") if key in payload}
        with OPTIMIZE_LOCK:
            result = optimize_trajectory(payload["trajectory"], payload.get("keyframes", []),
                                        path, terrain, resolve_mesh_path, friction=friction,
                                        max_effort_scale=max_effort_scale, **options)
        return jsonify(result), (200 if result.get("success", True) else 422)
    except (TrajectoryOptimizeError, KeyError, TypeError, ValueError, OSError) as error:
        return jsonify({"error": str(error)}), 422
    except Exception as error:
        app.logger.exception("Trajectory Optimize failed")
        return jsonify({"error": f"Trajectory Optimize failed: {error}"}), 500


@app.route("/api/trajectory/serialize", methods=["POST"])
def serialize_trajectory_endpoint():
    payload = request.get_json()
    if not payload:
        return jsonify({"error": "No payload"}), 400

    try:
        npz_bytes = serialize_npz(payload)
        return send_file(
            io.BytesIO(npz_bytes),
            mimetype="application/octet-stream",
            as_attachment=True,
            download_name="trajectory.npz",
        )
    except Exception as e:
        return jsonify({"error": str(e)}), 400


def _require_server_explorer():
    if FILE_EXPLORER_SIDE != "server":
        return jsonify({"error": "Server file explorer is disabled. Launch with --file-explorer-side server."}), 403
    return None


@app.route("/api/fs/list")
def fs_list():
    denied = _require_server_explorer()
    if denied is not None:
        return denied

    raw_path = request.args.get("path", "").strip()
    target_dir = os.path.abspath(os.path.expanduser(raw_path)) if raw_path else os.getcwd()
    if not os.path.isdir(target_dir):
        return jsonify({"error": f"Directory not found: {target_dir}"}), 400

    ext_param = request.args.get("ext", "").strip()
    exts = tuple(e.strip().lower() for e in ext_param.split(",") if e.strip()) if ext_param else ()

    entries = []
    try:
        with os.scandir(target_dir) as it:
            for entry in it:
                try:
                    is_dir = entry.is_dir(follow_symlinks=True)
                    if not is_dir and exts and not entry.name.lower().endswith(exts):
                        continue
                    item: dict[str, object] = {
                        "name": entry.name,
                        "path": os.path.join(target_dir, entry.name),
                        "is_dir": is_dir,
                    }
                    if not is_dir:
                        item["size"] = entry.stat(follow_symlinks=True).st_size
                    entries.append(item)
                except OSError:
                    continue
    except OSError as exc:
        return jsonify({"error": str(exc)}), 400

    entries.sort(key=lambda item: (not bool(item["is_dir"]), str(item["name"]).lower()))
    parent = os.path.dirname(target_dir)
    return jsonify({
        "path": target_dir,
        "parent": parent if parent != target_dir else None,
        "entries": entries,
    })


@app.route("/api/fs/trajectory/load", methods=["POST"])
def fs_load_trajectories():
    denied = _require_server_explorer()
    if denied is not None:
        return denied

    payload = request.get_json()
    if not isinstance(payload, dict):
        return jsonify({"error": "JSON object required"}), 400

    paths: list[str] = []
    directory = payload.get("directory")
    if isinstance(directory, str) and directory.strip():
        root = os.path.abspath(os.path.expanduser(directory.strip()))
        if not os.path.isdir(root):
            return jsonify({"error": f"Directory not found: {root}"}), 400
        for dirpath, _dirnames, filenames in os.walk(root):
            for fname in sorted(filenames):
                if fname.lower().endswith(".npz"):
                    paths.append(os.path.join(dirpath, fname))
        paths.sort()
    elif isinstance(payload.get("paths"), list):
        for p in payload["paths"]:
            if isinstance(p, str) and p.strip():
                paths.append(os.path.abspath(os.path.expanduser(p.strip())))
    else:
        return jsonify({"error": "Provide 'paths' or 'directory'"}), 400

    trajectories = []
    errors = []
    base_root = os.path.abspath(os.path.expanduser(directory.strip())) if isinstance(directory, str) and directory.strip() else None
    for p in paths:
        if not os.path.isfile(p):
            errors.append(f"{p}: File not found")
            continue
        try:
            with open(p, "rb") as f:
                data = parse_npz(f.read())
            display_name = os.path.relpath(p, base_root) if base_root else os.path.basename(p)
            trajectories.append({
                "path": p,
                "name": display_name,
                "data": data,
            })
        except Exception as exc:
            errors.append(f"{os.path.basename(p)}: {exc}")

    return jsonify({"trajectories": trajectories, "errors": errors})


@app.route("/api/fs/trajectory/save", methods=["POST"])
def fs_save_trajectory():
    denied = _require_server_explorer()
    if denied is not None:
        return denied

    payload = request.get_json()
    if not isinstance(payload, dict) or not isinstance(payload.get("path"), str) or not payload.get("trajectory"):
        return jsonify({"error": "Provide 'path' and 'trajectory'"}), 400

    target = os.path.abspath(os.path.expanduser(payload["path"].strip()))
    if not target.lower().endswith(".npz"):
        target += ".npz"
    try:
        npz_bytes = serialize_npz(payload["trajectory"])
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, "wb") as f:
            f.write(npz_bytes)
        return jsonify({"status": "ok", "path": target})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 400


@app.route("/api/fs/terrain")
def fs_load_terrain():
    denied = _require_server_explorer()
    if denied is not None:
        return denied

    raw_path = request.args.get("path", "").strip()
    if not raw_path:
        return jsonify({"error": "Missing 'path' query parameter"}), 400
    target = os.path.abspath(os.path.expanduser(raw_path))
    if not os.path.isfile(target):
        return jsonify({"error": f"Terrain file not found: {target}"}), 404
    return send_file(target)


@app.route("/api/fs/terrain/save", methods=["POST"])
def fs_save_terrain():
    denied = _require_server_explorer()
    if denied is not None:
        return denied

    payload = request.get_json()
    if not isinstance(payload, dict) or not isinstance(payload.get("path"), str) or not isinstance(payload.get("content"), str):
        return jsonify({"error": "Provide 'path' and 'content'"}), 400

    target = os.path.abspath(os.path.expanduser(payload["path"].strip()))
    if not target.lower().endswith(".obj"):
        target += ".obj"
    try:
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, "w", encoding="utf-8") as f:
            f.write(payload["content"])
        return jsonify({"status": "ok", "path": target})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 400


@app.route("/api/fs/robot/select", methods=["POST"])
def fs_select_robot():
    global ROBOT_URDF_PATH, UPLOADED_ROBOT_DIR, CURRENT_URDF_REL
    denied = _require_server_explorer()
    if denied is not None:
        return denied

    payload = request.get_json()
    if not isinstance(payload, dict) or not isinstance(payload.get("path"), str) or not payload["path"].strip():
        return jsonify({"error": "Provide 'path' to a URDF file"}), 400

    target = os.path.abspath(os.path.expanduser(payload["path"].strip()))
    if not os.path.isfile(target):
        return jsonify({"error": f"URDF file not found: {target}"}), 404
    ROBOT_URDF_PATH = target
    UPLOADED_ROBOT_DIR = None
    CURRENT_URDF_REL = None
    return jsonify({"status": "ok", "urdf_path": target})


@app.route("/api/<path:rest>")
def serve_api_catchall(rest: str):
    if rest.startswith("robot/") or rest.startswith("trajectory/") or rest.startswith("fs/"):
        return abort(404)
    resolved = resolve_mesh_path(rest)
    if resolved and os.path.isfile(resolved):
        return send_file(resolved)
    return abort(404, f"Not found: {rest}")


@app.route("/")
def serve_frontend_index():
    if FRONTEND_DIST_DIR and os.path.isfile(os.path.join(FRONTEND_DIST_DIR, "index.html")):
        return send_from_directory(FRONTEND_DIST_DIR, "index.html")
    return abort(404, "Frontend build not configured")


@app.route("/<path:filename>")
def serve_frontend_asset(filename: str):
    if filename == "api" or filename.startswith("api/"):
        return abort(404)
    if FRONTEND_DIST_DIR and os.path.isdir(FRONTEND_DIST_DIR):
        return send_from_directory(FRONTEND_DIST_DIR, filename)
    return abort(404)


def configure_app(
    robot_urdf_path: str | None = None,
    nudge_step: float = 0.01,
    frame_step: int = 1,
    frontend_dist_dir: str | None = None,
    file_explorer_side: str = "client",
) -> None:
    global ROBOT_URDF_PATH, NUDGE_STEP, FRAME_STEP, FRONTEND_DIST_DIR, FILE_EXPLORER_SIDE
    if file_explorer_side not in ("client", "server"):
        raise ValueError(f"Invalid file_explorer_side: {file_explorer_side!r} (expected 'client' or 'server')")
    ROBOT_URDF_PATH = resolve_urdf_path(robot_urdf_path) if robot_urdf_path else None
    NUDGE_STEP = nudge_step
    FRAME_STEP = frame_step
    FRONTEND_DIST_DIR = os.path.abspath(frontend_dist_dir) if frontend_dist_dir else None
    FILE_EXPLORER_SIDE = file_explorer_side


def main():
    parser = argparse.ArgumentParser(description="Robot Trajectory Editor Backend")
    parser.add_argument(
        "--robot-urdf", "--robot_urdf_path",
        dest="robot_urdf_path",
        default=None,
        help="Path to robot URDF file (local path or package://<python_pkg>/<relpath>)",
    )
    parser.add_argument("--port", type=int, default=5000, help="Flask server port")
    parser.add_argument("--host", default="127.0.0.1", help="Flask server host")
    parser.add_argument("--nudge-step", type=float, default=0.01,
                        help="Value step applied by the Up/Down arrow keys on the selected edit target")
    parser.add_argument("--frame-step", type=int, default=1,
                        help="Frame step applied by the Left/Right arrow keys")
    parser.add_argument("--file-explorer-side", choices=("client", "server"), default="client",
                        help="Where file dialogs browse/save files: 'client' (browser) or 'server' (in-page)")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    args = parser.parse_args()

    try:
        configure_app(
            robot_urdf_path=args.robot_urdf_path,
            nudge_step=args.nudge_step,
            frame_step=args.frame_step,
            file_explorer_side=args.file_explorer_side,
        )
    except (ValueError, FileNotFoundError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)

    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == "__main__":
    main()
