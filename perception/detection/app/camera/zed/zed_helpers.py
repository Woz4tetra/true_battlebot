from typing import Any

import numpy as np
import pyzed.sl as sl
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import CameraInfo, Imu
from std_msgs.msg import Header


def zed_to_ros_camera_info(camera_information: Any) -> CameraInfo:
    zed_intrinsics = camera_information.camera_configuration.calibration_parameters.left_cam
    resolution = zed_intrinsics.image_size

    distortion = np.array(zed_intrinsics.disto).flatten().tolist()
    intrinsics = np.array(
        [
            [zed_intrinsics.fx, 0, zed_intrinsics.cx],
            [0, zed_intrinsics.fy, zed_intrinsics.cy],
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


def zed_to_ros_imu(header: Header, imu_data: sl.IMUData) -> Imu:
    quaternion = imu_data.get_pose().get_orientation().get()
    linear_acceleration = imu_data.get_linear_acceleration()
    angular_velocity = np.deg2rad(np.array(imu_data.get_angular_velocity()))
    return Imu(
        header=header,
        orientation=Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]),
        linear_acceleration=Vector3(x=linear_acceleration[0], y=linear_acceleration[1], z=linear_acceleration[2]),
        angular_velocity=Vector3(x=angular_velocity[0], y=angular_velocity[1], z=angular_velocity[2]),
    )


def zed_status_to_str(status: sl.ERROR_CODE) -> str:
    return f"<{status.name} ({status.value})>: {status}"
