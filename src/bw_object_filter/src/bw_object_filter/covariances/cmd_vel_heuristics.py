import math
from typing import List

import numpy as np
from geometry_msgs.msg import TwistWithCovariance

from bw_object_filter.covariances.covariance_heuristics import CovarianceHeuristics


class CmdVelHeuristics(CovarianceHeuristics[TwistWithCovariance]):
    def __init__(self, base_covariance_scalar: float) -> None:
        self.base_covariance = np.diag([base_covariance_scalar] * 6)

    def compute_covariance(self, measurement: TwistWithCovariance) -> List[float]:
        twist = measurement.twist
        magnitude = np.linalg.norm([twist.linear.x, twist.linear.y, twist.angular.z])
        covariance = self.base_covariance
        covariance *= math.exp(magnitude)
        return covariance.flatten().tolist()
