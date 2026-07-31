import argparse
import glob
import io
import os
import tempfile
import shutil
import sys

from flask import Flask, request, jsonify, send_file, abort
from npz_io import parse_npz, serialize_npz

app = Flask(__name__)

ROBOT_URDF_PATH: str | None = None
UPLOADED_ROBOT_DIR: str | None = None
CURRENT_URDF_REL: str | None = None


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


def resolve_mesh_path(mesh_href: str) -> str | None:
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
        if not os.path.isfile(urdf_path):
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
    for f in uploaded_files:
        rel_path = f.filename
        if not rel_path:
            continue
        dest = os.path.join(temp_dir, rel_path)
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        f.save(dest)

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


@app.route("/api/<path:rest>")
def serve_api_catchall(rest: str):
    if rest.startswith("robot/") or rest.startswith("trajectory/"):
        return abort(404)
    resolved = resolve_mesh_path(rest)
    if resolved and os.path.isfile(resolved):
        return send_file(resolved)
    return abort(404, f"Not found: {rest}")


def main():
    global ROBOT_URDF_PATH
    parser = argparse.ArgumentParser(description="Robot Trajectory Editor Backend")
    parser.add_argument("--robot-urdf", dest="robot_urdf_path", default=None, help="Path to robot URDF file")
    parser.add_argument("--port", type=int, default=5000, help="Flask server port")
    parser.add_argument("--host", default="127.0.0.1", help="Flask server host")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    args = parser.parse_args()

    ROBOT_URDF_PATH = args.robot_urdf_path

    if ROBOT_URDF_PATH and ROBOT_URDF_PATH.startswith("package://"):
        print(f"Warning: package:// URDF ({ROBOT_URDF_PATH}) - backend cannot resolve this. Use a local path.", file=sys.stderr)

    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == "__main__":
    main()
