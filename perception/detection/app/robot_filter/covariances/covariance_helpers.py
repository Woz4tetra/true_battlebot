import numpy as np
from bw_shared.geometry.transform3d import Transform3D
from geometry_msgs.msg import Pose


def get_pose_distance(pose: Pose) -> float:
    return float(
        np.linalg.norm(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ]
        )
    )


def pose_distance_covariance_scale(distance: float) -> float:
    if distance < 0.25:
        return 10.0
    return float(0.25 * distance**2.0)


def pose_angle_covariance_scale(pose: Pose) -> float:
    angles = Transform3D.from_pose_msg(pose).rpy
    angle_diff = abs(angles.pitch) + abs(angles.yaw)  # use pitch and yaw angles
    if angle_diff < np.pi / 4:  # if angle is less than 45 degrees
        return 1.0
    else:
        return 1.0 / angle_diff  # scale covariance inversely proportional to angle
