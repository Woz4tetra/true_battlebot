from typing import List

import numpy as np
from bw_interfaces.msg import EstimatedObject

from bw_object_filter.covariances.covariance_helpers import (
    get_pose_distance,
    pose_distance_covariance_scale,
)
from bw_object_filter.covariances.covariance_heuristics import CovarianceHeuristics


class RobotHeuristics(CovarianceHeuristics[EstimatedObject]):
    def __init__(self, base_covariance_scalar: float) -> None:
        self.base_covariance = np.diag([base_covariance_scalar] * 6)

    def compute_covariance(self, measurement: EstimatedObject) -> List[float]:
        distance = get_pose_distance(measurement.pose.pose)
        covariance = np.copy(self.base_covariance)
        covariance *= pose_distance_covariance_scale(distance)
        covariance[5, 5] = 1000.0  # ignore the orientation
        return covariance.flatten().tolist()
