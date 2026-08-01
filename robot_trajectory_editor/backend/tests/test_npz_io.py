import json
import os
import sys
import tempfile

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from npz_io import parse_npz, serialize_npz


def make_synthetic_payload(frame_count=10, joint_count=7):
    return {
        "framerate": 50.0,
        "joint_names": [f"joint_{i}" for i in range(joint_count)],
        "joint_pos": np.random.randn(frame_count, joint_count).astype(np.float32).tolist(),
        "base_pos_w": np.random.randn(frame_count, 3).astype(np.float32).tolist(),
        "base_quat_w": np.random.randn(frame_count, 4).astype(np.float32).tolist(),
    }


def test_roundtrip():
    payload = make_synthetic_payload()
    npz_bytes = serialize_npz(payload)
    result = parse_npz(npz_bytes)

    assert result["framerate"] == payload["framerate"]
    assert result["joint_names"] == payload["joint_names"]
    np.testing.assert_allclose(result["joint_pos"], payload["joint_pos"])
    np.testing.assert_allclose(result["base_pos_w"], payload["base_pos_w"])
    np.testing.assert_allclose(result["base_quat_w"], payload["base_quat_w"])


def test_framerate_is_float():
    payload = make_synthetic_payload(frame_count=1, joint_count=1)
    payload["framerate"] = 30.0
    npz_bytes = serialize_npz(payload)
    result = parse_npz(npz_bytes)
    assert isinstance(result["framerate"], float)
    assert result["framerate"] == 30.0


def test_dtypes():
    payload = make_synthetic_payload(frame_count=3, joint_count=2)
    npz_bytes = serialize_npz(payload)

    buf = tempfile.NamedTemporaryFile(suffix=".npz", delete=False)
    try:
        buf.write(npz_bytes)
        buf.close()
        data = np.load(buf.name, allow_pickle=True)
        assert data["framerate"].dtype == np.float32
        assert data["joint_pos"].dtype == np.float32
        assert data["base_pos_w"].dtype == np.float32
        assert data["base_quat_w"].dtype == np.float32
    finally:
        os.unlink(buf.name)


def test_empty_trajectory():
    payload = {
        "framerate": 1.0,
        "joint_names": [],
        "joint_pos": [],
        "base_pos_w": [],
        "base_quat_w": [],
    }
    npz_bytes = serialize_npz(payload)
    result = parse_npz(npz_bytes)
    assert result["framerate"] == 1.0
    assert result["joint_names"] == []
    assert result["joint_pos"] == []
    assert result["base_pos_w"] == []
    assert result["base_quat_w"] == []


def test_parse_spec_compliant_file():
    """A .npz written exactly per TASK.md spec keys must parse."""
    import io
    buf = io.BytesIO()
    np.savez(
        buf,
        framerate=np.array(30.0, dtype=np.float32),
        joint_names=np.array(["j0", "j1"]),
        joint_pos=np.zeros((4, 2), dtype=np.float32),
        base_pos_w=np.zeros((4, 3), dtype=np.float32),
        base_quat_w=np.tile(np.array([1, 0, 0, 0], dtype=np.float32), (4, 1)),
    )
    result = parse_npz(buf.getvalue())
    assert result["framerate"] == 30.0
    assert len(result["joint_pos"]) == 4
    assert len(result["base_pos_w"][0]) == 3
    assert len(result["base_quat_w"][0]) == 4


def test_parse_missing_key_reports_spec_keys():
    import io
    buf = io.BytesIO()
    np.savez(buf, framerate=np.array(30.0, dtype=np.float32))
    with pytest.raises(ValueError, match="base_pos_w"):
        parse_npz(buf.getvalue())


def test_parse_rejects_pickle_object_arrays():
    import io

    buf = io.BytesIO()
    np.savez(
        buf,
        framerate=np.array(30.0, dtype=np.float32),
        joint_names=np.array(["j0"], dtype=object),
        joint_pos=np.zeros((1, 1), dtype=np.float32),
        base_pos_w=np.zeros((1, 3), dtype=np.float32),
        base_quat_w=np.array([[1, 0, 0, 0]], dtype=np.float32),
    )
    with pytest.raises(ValueError, match="allow_pickle"):
        parse_npz(buf.getvalue())
