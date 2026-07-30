import io
import json
import os
from typing import Any

import numpy as np


def parse_npz(data: bytes) -> dict[str, Any]:
    buf = io.BytesIO(data)
    try:
        npz = np.load(buf, allow_pickle=True)
    except Exception as e:
        raise ValueError(f"Not a valid .npz file: {e}")

    required = ["framerate", "joint_names", "joint_pos", "base_pos_w", "base_quat_w"]
    missing = [k for k in required if k not in npz.files]
    if missing:
        raise ValueError(f"Missing keys in .npz: {missing}. Found keys: {list(npz.files)}")

    with npz:
        framerate_arr = npz["framerate"]
        try:
            framerate = float(framerate_arr.item())
        except (ValueError, AttributeError):
            framerate = float(framerate_arr[0])

        joint_names = [str(n) for n in npz["joint_names"]]
        joint_pos = npz["joint_pos"].tolist()
        base_pos_w = npz["base_pos_w"].tolist()
        base_quat_w = npz["base_quat_w"].tolist()

    return {
        "framerate": framerate,
        "joint_names": joint_names,
        "joint_pos": joint_pos,
        "base_pos_w": base_pos_w,
        "base_quat_w": base_quat_w,
    }


def serialize_npz(payload: dict[str, Any]) -> bytes:
    framerate = np.array(payload["framerate"], dtype=np.float32).reshape(())
    joint_names = np.array(payload["joint_names"], dtype=str)
    joint_pos = np.array(payload["joint_pos"], dtype=np.float32)
    base_pos_w = np.array(payload["base_pos_w"], dtype=np.float32)
    base_quat_w = np.array(payload["base_quat_w"], dtype=np.float32)

    buf = io.BytesIO()
    np.savez(
        buf,
        framerate=framerate,
        joint_names=joint_names,
        joint_pos=joint_pos,
        base_pos_w=base_pos_w,
        base_quat_w=base_quat_w,
    )
    return buf.getvalue()
