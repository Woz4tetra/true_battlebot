from typing import List

import numpy as np
from geometry_msgs.msg import TwistWithCovariance

from bw_object_filter.covariances.covariance_heuristics import CovarianceHeuristics


class CmdVelHeuristics(CovarianceHeuristics[TwistWithCovariance]):
    def compute_covariance(self, measurement: TwistWithCovariance) -> List[float]:
        pass
