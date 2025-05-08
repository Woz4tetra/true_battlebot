from typing import List

import numpy as np
from app.robot_filter.covariances.covariance_heuristics import CovarianceHeuristics
from bw_interfaces.msg import EstimatedObject


class RobotStaticHeuristics(CovarianceHeuristics[EstimatedObject]):
    def __init__(self, linear_covariance_scalar: float, angular_covariance: float) -> None:
        self.base_covariance = list(
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
