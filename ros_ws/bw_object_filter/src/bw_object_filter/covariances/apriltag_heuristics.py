from typing import List

import numpy as np
from bw_interfaces.msg import EstimatedObject

from bw_object_filter.covariances.covariance_helpers import (
    get_pose_distance,
    pose_angle_covariance_scale,
    pose_distance_covariance_scale,
)
from bw_object_filter.covariances.covariance_heuristics import CovarianceHeuristics


class ApriltagHeuristics(CovarianceHeuristics[EstimatedObject]):
    def __init__(self, base_covariance_scalar: float) -> None:
        self.base_covariance = np.diag([base_covariance_scalar] * 6)

    def compute_covariance(self, measurement: EstimatedObject) -> List[float]:
        distance = get_pose_distance(measurement.pose.pose)
        covariance = np.copy(self.base_covariance)
        covariance *= pose_distance_covariance_scale(distance)
        covariance *= pose_angle_covariance_scale(measurement.pose.pose)
        return covariance.flatten().tolist()
