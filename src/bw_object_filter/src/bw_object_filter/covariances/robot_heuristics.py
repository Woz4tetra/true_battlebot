from typing import List

import numpy as np
from bw_interfaces.msg import EstimatedRobotArray

from bw_object_filter.covariances.covariance_heuristics import CovarianceHeuristics


class RobotHeuristics(CovarianceHeuristics[EstimatedRobotArray]):
    def compute_covariance(self, measurement: EstimatedRobotArray) -> List[float]:
        pass
