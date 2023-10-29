from typing import List

import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray

from bw_object_filter.covariances.covariance_heuristics import CovarianceHeuristics


class ApriltagHeuristics(CovarianceHeuristics[AprilTagDetectionArray]):
    def compute_covariance(self, measurement: AprilTagDetectionArray) -> List[float]:
        pass
