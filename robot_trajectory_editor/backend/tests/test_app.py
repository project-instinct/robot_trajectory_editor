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
    def fail(trajectory, keyframes, path, terrain, resolver, friction):
        assert friction == .7
        assert path == str(selected)
        assert keyframes == [0, 2]
        assert terrain == 'edited mesh'
        raise TrajectoryOptimizeError('Fixed keyframe 2 penetrates terrain by 0.01 m.')
    monkeypatch.setattr(app_module, 'optimize_trajectory', fail)
    response = app.test_client().post('/api/trajectory/optimize', json={
        'trajectory': {}, 'keyframes': [0, 2], 'urdf_path': 'selected.urdf', 'terrain_obj': 'edited mesh',
    })
    assert response.status_code == 422
    assert '0.01 m' in response.get_json()['error']
    assert 'trajectory' not in response.get_json()
    response = app.test_client().post('/api/trajectory/optimize', json={'urdf_path': '../outside.urdf'})
    assert response.status_code == 400
