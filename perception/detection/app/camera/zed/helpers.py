from typing import Any

import numpy as np
from sensor_msgs.msg import CameraInfo


def zed_to_ros_camera_info(camera_information: Any) -> CameraInfo:
    intrinsics = camera_information.camera_configuration.calibration_parameters.left_cam
    resolution = intrinsics.image_size

    distortion = intrinsics.disto
    intrinsics = np.array(
        [
            [intrinsics.fx, 0, intrinsics.cx],
            [0, intrinsics.fy, intrinsics.cy],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )
    projection = np.append(intrinsics, np.zeros((3, 1)), axis=1)

    return CameraInfo(
        height=resolution.height,
        width=resolution.width,
        distortion_model="plumb_bob",
        D=distortion,
        K=intrinsics.flatten().tolist(),
        R=np.eye(3).flatten().tolist(),
        P=projection.flatten().tolist(),
    )
