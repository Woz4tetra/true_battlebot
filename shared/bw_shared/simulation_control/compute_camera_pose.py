import numpy as np
from geometry_msgs.msg import Vector3

from bw_shared.geometry.projection_math.look_rotation import look_rotation
from bw_shared.geometry.transform3d import Transform3D


def compute_camera_pose(distance: float, azimuth_angle: float, elevation_angle: float) -> Transform3D:
    position_array = np.array(
        [
            distance * np.cos(elevation_angle) * np.cos(azimuth_angle),
            -1 * distance * np.cos(elevation_angle) * np.sin(azimuth_angle),
            distance * np.sin(elevation_angle),
        ]
    )

    rotation = look_rotation(-1 * position_array)

    return Transform3D.from_position_and_quaternion(Vector3(*position_array), rotation)
