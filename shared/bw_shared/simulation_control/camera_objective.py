import numpy as np

from bw_shared.geometry.transform3d import Transform3D


def make_camera_objective(camera_pose: Transform3D) -> dict:
    rpy_deg = np.degrees(camera_pose.rpy)

    return {
        "type": "idle",
        "init": {
            "type": "flu",
            "x": camera_pose.position.x,
            "y": camera_pose.position.y,
            "z": camera_pose.position.z,
            "roll": rpy_deg[0],
            "pitch": rpy_deg[1],
            "yaw": rpy_deg[2],
        },
        "sequence": [],
    }
