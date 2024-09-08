import math
from typing import List

import numpy as np
from geometry_msgs.msg import Twist

from bw_object_filter.covariances.covariance_heuristics import CovarianceHeuristics


class CmdVelHeuristics(CovarianceHeuristics[Twist]):
    def __init__(self, base_covariance_scalar: float) -> None:
        self.base_covariance = np.diag([base_covariance_scalar] * 6)

    def compute_covariance(self, twist: Twist) -> List[float]:
        magnitude = np.linalg.norm([twist.linear.x, twist.linear.y, twist.angular.z])
        covariance = np.copy(self.base_covariance)
        covariance *= math.exp(magnitude * 0.1)
        return covariance.flatten().tolist()
