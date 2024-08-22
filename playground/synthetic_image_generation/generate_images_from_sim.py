import random

import numpy as np
import rospy
from bw_shared.geometry.projection_math.look_rotation import look_rotation
from bw_shared.geometry.transform3d import Transform3D
from geometry_msgs.msg import Pose, Quaternion, Vector3


def compute_camera_pose(distance: float, azimuth_angle: float, elevation_angle: float) -> Transform3D:
    position_array = np.array(
        [
            distance * np.cos(elevation_angle) * np.cos(azimuth_angle),
            -distance * np.cos(elevation_angle) * np.sin(azimuth_angle),
            distance * np.sin(elevation_angle),
        ]
    )

    rotation = look_rotation(-1 * position_array)

    return Transform3D.from_position_and_quaternion(Vector3(*position_array), rotation)


def get_random_camera_pose() -> Transform3D:
    angle_inset = 0.2
    distance_range = (2.2, 2.5)
    azimuth_angle_range = (-np.pi / 4 + angle_inset, np.pi / 4 - angle_inset)
    elevation_angle_range = (0.4, 0.5)
    flip_azimuth = random.uniform(0.0, 1.0) > 0.5

    distance = random.uniform(*distance_range)
    azimuth_angle = random.uniform(*azimuth_angle_range)
    elevation_angle = random.uniform(*elevation_angle_range)

    if flip_azimuth:
        azimuth_angle = azimuth_angle + np.pi

    pose = compute_camera_pose(distance, azimuth_angle, elevation_angle)
    rotation = pose.quaternion_np
    randomized_rotation = np.random.normal(rotation, 0.01)
    return Transform3D.from_position_and_quaternion(pose.position, Quaternion(*randomized_rotation))


def generate_spawn_grid() -> np.ndarray:
    x = np.linspace(-1, 1, 10)
    y = np.linspace(-1, 1, 10)
    grid = np.meshgrid(x, y)
    return np.array(grid).reshape(2, -1).T


def generate_target_object(spawn_grid: np.ndarray, target_name: str) -> dict:
    angle = random.uniform(0, 360)
    x, y = spawn_grid[random.randint(0, spawn_grid.shape[0] - 1)]
    return {
        "type": "target",
        "init": {"type": "relative", "x": x, "y": y, "yaw": angle},
        "sequence": [{"target_name": target_name}],
    }


def main() -> None:
    rospy.init_node("generate_images_from_sim")

    camera_set_pose_pub = rospy.Publisher("/camera_0/set_pose", Pose, queue_size=1)

    while not rospy.is_shutdown():
        pose = get_random_camera_pose()
        camera_set_pose_pub.publish(pose.to_pose_msg())
        rospy.sleep(0.6)


if __name__ == "__main__":
    main()
