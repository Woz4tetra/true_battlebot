import random

import numpy as np
from bw_interfaces.msg import SimulationConfig
from geometry_msgs.msg import Quaternion

from bw_shared.enums.cage_model import CageModel
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.simulation_control.camera_objective import make_camera_objective
from bw_shared.simulation_control.compute_camera_pose import compute_camera_pose
from bw_shared.simulation_control.make_objective import make_objective


def get_random_camera_pose(cage_model: CageModel) -> Transform3D:
    angle_inset = 0.2
    if cage_model == CageModel.DRIVE_TEST_BOX:
        distance_range = (1.0, 1.5)
        elevation_angle_range = (0.2, 0.9)
    else:
        distance_range = (2.2, 2.5)
        elevation_angle_range = (0.4, 0.5)
    azimuth_angle_range = (-np.pi / 4 + angle_inset, np.pi / 4 - angle_inset)
    flip_azimuth = random.uniform(0.0, 1.0) > 0.5

    distance = random.uniform(*distance_range)
    azimuth_angle = random.uniform(*azimuth_angle_range)
    elevation_angle = random.uniform(*elevation_angle_range)

    if flip_azimuth:
        azimuth_angle = azimuth_angle + np.pi

    pose = compute_camera_pose(distance, azimuth_angle, elevation_angle)
    rotation = pose.quaternion_np
    randomized_rotation = np.random.normal(rotation, 0.01)
    camera_pose = Transform3D.from_position_and_quaternion(pose.position, Quaternion(*randomized_rotation))
    return camera_pose


def get_random_camera_objective(objective_name: str, cage_model: CageModel) -> SimulationConfig:
    return make_objective(objective_name, make_camera_objective(get_random_camera_pose(cage_model)))
