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
