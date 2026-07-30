import io
import json
import os
from typing import Any

import numpy as np


def parse_npz(data: bytes) -> dict[str, Any]:
    buf = io.BytesIO(data)
    with np.load(buf, allow_pickle=True) as npz:
        framerate_arr = npz["framerate"]
        framerate = float(framerate_arr.item()) if framerate_arr.ndim == 0 else float(framerate_arr[0])

        joint_names = npz["joint_names"].tolist()
        joint_pos = npz["joint_pos"].tolist()
        base_pose_w = npz["base_pose_w"].tolist()
        base_quat_w = npz["base_quat_w"].tolist()

    return {
        "framerate": framerate,
        "joint_names": joint_names,
        "joint_pos": joint_pos,
        "base_pose_w": base_pose_w,
        "base_quat_w": base_quat_w,
    }


def serialize_npz(payload: dict[str, Any]) -> bytes:
    framerate = np.array(payload["framerate"], dtype=np.float32).reshape(())
    joint_names = np.array(payload["joint_names"], dtype=str)
    joint_pos = np.array(payload["joint_pos"], dtype=np.float32)
    base_pose_w = np.array(payload["base_pose_w"], dtype=np.float32)
    base_quat_w = np.array(payload["base_quat_w"], dtype=np.float32)

    buf = io.BytesIO()
    np.savez(
        buf,
        framerate=framerate,
        joint_names=joint_names,
        joint_pos=joint_pos,
        base_pose_w=base_pose_w,
        base_quat_w=base_quat_w,
    )
    return buf.getvalue()
