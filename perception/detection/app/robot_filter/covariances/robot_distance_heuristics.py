from typing import List

import numpy as np
from app.robot_filter.covariances.covariance_helpers import (
    get_pose_distance,
    pose_distance_covariance_scale,
)
from app.robot_filter.covariances.covariance_heuristics import CovarianceHeuristics
from bw_interfaces.msg import EstimatedObject


class RobotDistanceHeuristics(CovarianceHeuristics[EstimatedObject]):
    def __init__(self, base_covariance_scalar: float, yaw_covariance: float) -> None:
        self.base_covariance = np.diag([base_covariance_scalar] * 6)
        self.yaw_covariance = yaw_covariance

    def compute_covariance(self, measurement: EstimatedObject) -> List[float]:
        distance = get_pose_distance(measurement.pose.pose)
        covariance = np.copy(self.base_covariance)
        covariance *= pose_distance_covariance_scale(distance)
        covariance[5, 5] = self.yaw_covariance
        return list(covariance.flatten().tolist())
