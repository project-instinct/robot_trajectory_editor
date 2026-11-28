import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import _safe_upload_relative_path, app


def test_config_returns_default_steps():
    client = app.test_client()
    resp = client.get("/api/config")
    assert resp.status_code == 200
    data = resp.get_json()
    assert data["nudge_step"] == 0.01
    assert data["frame_step"] == 1


def test_urdf_path_cannot_escape_robot_directory(tmp_path):
    import app as app_module

    urdf_dir = tmp_path / "robot"
    urdf_dir.mkdir()
    (tmp_path / "outside.urdf").write_text("outside")
    app_module.ROBOT_URDF_PATH = str(urdf_dir / "inside.urdf")
    (urdf_dir / "inside.urdf").write_text("inside")

    client = app.test_client()
    assert client.get("/api/robot/urdf?path=../outside.urdf").status_code == 404
    assert client.get("/api/robot/urdf?path=inside.urdf").status_code == 200


def test_upload_rejects_path_traversal():
    assert _safe_upload_relative_path("meshes/foot.stl") == os.path.join("meshes", "foot.stl")
    for filename in ("../outside", "/tmp/outside", "..\\outside"):
        try:
            _safe_upload_relative_path(filename)
        except ValueError:
            pass
        else:
            raise AssertionError(f"unsafe filename accepted: {filename}")


def test_trajectory_optimize_requires_loaded_robot(monkeypatch):
    import app as app_module
    monkeypatch.setattr(app_module, 'ROBOT_URDF_PATH', None)
    monkeypatch.setattr(app_module, 'UPLOADED_ROBOT_DIR', None)
    response = app.test_client().post('/api/trajectory/optimize', json={'trajectory': {}})
    assert response.status_code == 400
    assert 'URDF' in response.get_json()['error']


def test_trajectory_optimize_rejects_invalid_friction(tmp_path, monkeypatch):
    import app as app_module
    path = tmp_path / 'robot.urdf'
    path.write_text('model')
    monkeypatch.setattr(app_module, 'ROBOT_URDF_PATH', str(path))
    for value in [-.1, None, True, '0.7']:
        response = app.test_client().post('/api/trajectory/optimize', json={'trajectory': {}, 'friction': value})
        assert response.status_code == 422
        assert 'friction' in response.get_json()['error']


def test_trajectory_optimize_uses_requested_model_and_propagates_failure(tmp_path, monkeypatch):
    import app as app_module
    from trajectory_optimize import TrajectoryOptimizeError
    first, selected = tmp_path / 'first.urdf', tmp_path / 'selected.urdf'
    first.write_text('first')
    selected.write_text('selected')
    monkeypatch.setattr(app_module, 'ROBOT_URDF_PATH', str(first))
    def fail(trajectory, keyframes, path, terrain, resolver, friction, max_effort_scale=1.0, **options):
        assert friction == .7
        assert max_effort_scale == 1.0
        assert options == {'smoothing_hz': 4, 'glitch_speed_scale': 0}
        assert path == str(selected)
        assert keyframes == [0, 2]
        assert terrain == 'edited mesh'
        raise TrajectoryOptimizeError('Fixed keyframe 2 penetrates terrain by 0.01 m.')
    monkeypatch.setattr(app_module, 'optimize_trajectory', fail)
    response = app.test_client().post('/api/trajectory/optimize', json={
        'trajectory': {}, 'keyframes': [0, 2], 'urdf_path': 'selected.urdf', 'terrain_obj': 'edited mesh',
        'smoothing_hz': 4, 'glitch_speed_scale': 0,
    })
    assert response.status_code == 422
    assert '0.01 m' in response.get_json()['error']
    assert 'trajectory' not in response.get_json()
    response = app.test_client().post('/api/trajectory/optimize', json={'urdf_path': '../outside.urdf'})
    assert response.status_code == 400


def test_trajectory_optimize_runs_one_request_at_a_time(tmp_path, monkeypatch):
    import threading
    import time
    import app as app_module
    path = tmp_path / 'robot.urdf'
    path.write_text('model')
    monkeypatch.setattr(app_module, 'ROBOT_URDF_PATH', str(path))
    running, overlap = [0], [False]

    def solve(*args, **kwargs):
        running[0] += 1
        overlap[0] |= running[0] > 1
        time.sleep(.05)
        running[0] -= 1
        return {'success': True}
    monkeypatch.setattr(app_module, 'optimize_trajectory', solve)

    def post():
        assert app.test_client().post('/api/trajectory/optimize', json={'trajectory': {}}).status_code == 200
    threads = [threading.Thread(target=post) for _ in range(3)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert not overlap[0]


def test_resolve_package_uri_and_urdf_path(tmp_path, monkeypatch):
    import importlib
    import app as app_module

    pkg_root = tmp_path / "fake_robot_pkg"
    (pkg_root / "urdf").mkdir(parents=True)
    (pkg_root / "meshes").mkdir(parents=True)
    (pkg_root / "__init__.py").write_text("")
    urdf_file = pkg_root / "urdf" / "robot.urdf"
    urdf_file.write_text("<robot name='fake'/>")
    mesh_file = pkg_root / "meshes" / "link.stl"
    mesh_file.write_text("solid")
    (tmp_path / "outside.urdf").write_text("outside")

    monkeypatch.syspath_prepend(str(tmp_path))
    importlib.invalidate_caches()

    assert app_module.resolve_urdf_path("package://fake_robot_pkg/urdf/robot.urdf") == str(urdf_file)
    assert app_module.resolve_mesh_path("package://fake_robot_pkg/meshes/link.stl") == str(mesh_file)

    for bad in (
        "package://fake_robot_pkg/../outside.urdf",
        "package://nonexistent_pkg_xyz/urdf/robot.urdf",
        "package://fake_robot_pkg/urdf/missing.urdf",
    ):
        try:
            app_module.resolve_urdf_path(bad)
        except FileNotFoundError:
            pass
        else:
            raise AssertionError(f"Expected FileNotFoundError for {bad}")

    try:
        app_module.resolve_urdf_path("package://onlypkg")
    except ValueError:
        pass
    else:
        raise AssertionError("Expected ValueError for malformed package:// URI")


def test_frontend_static_serving(tmp_path, monkeypatch):
    import app as app_module

    dist_dir = tmp_path / "dist"
    (dist_dir / "assets").mkdir(parents=True)
    (dist_dir / "index.html").write_text("<html><body>Editor</body></html>")
    (dist_dir / "assets" / "main.js").write_text("console.log('ok');")

    monkeypatch.setattr(app_module, "FRONTEND_DIST_DIR", str(dist_dir))
    client = app.test_client()

    resp = client.get("/")
    assert resp.status_code == 200
    assert b"Editor" in resp.data

    asset = client.get("/assets/main.js")
    assert asset.status_code == 200
    assert b"console.log" in asset.data

    missing_api = client.get("/api/robot/nonexistent")
    assert missing_api.status_code == 404


def test_launch_cli_parses_both_urdf_flag_aliases():
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)
    from robot_trajectory_editor.launch import build_parser

    parser = build_parser()
    a = parser.parse_args(["--robot-urdf", "/tmp/a.urdf", "--port", "5050", "--dev"])
    assert a.robot_urdf_path == "/tmp/a.urdf"
    assert a.port == 5050
    assert a.dev is True

    b = parser.parse_args(["--robot_urdf_path", "package://robot_description/urdf/robot.urdf"])
    assert b.robot_urdf_path == "package://robot_description/urdf/robot.urdf"
    assert b.dev is False
