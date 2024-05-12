from typing import List

import numpy as np
from bw_interfaces.msg import EstimatedObject

from bw_object_filter.covariances.covariance_heuristics import CovarianceHeuristics


class RobotStaticHeuristics(CovarianceHeuristics[EstimatedObject]):
    def __init__(self, linear_covariance_scalar: float, angular_covariance: float) -> None:
        self.base_covariance = (
            np.diag(
                [
                    linear_covariance_scalar,
                    linear_covariance_scalar,
                    angular_covariance,
                    linear_covariance_scalar,
                    linear_covariance_scalar,
                    angular_covariance,
                ]
            )
            .flatten()
            .tolist()
        )

    def compute_covariance(self, measurement: EstimatedObject) -> List[float]:
        return self.base_covariance
