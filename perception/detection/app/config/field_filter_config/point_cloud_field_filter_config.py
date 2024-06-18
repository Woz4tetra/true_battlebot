from dataclasses import dataclass, field
from typing import Literal

import numpy as np


class BasePlaneSolverConfig:
    pass


@dataclass
class LeastSquaresPlaneSolverConfig(BasePlaneSolverConfig):
    type: Literal["LeastSquaresPlaneSolver"] = "LeastSquaresPlaneSolver"
    inlier_threshold: float = 0.1


@dataclass
class RansacPlaneSolverConfig(BasePlaneSolverConfig):
    type: Literal["RansacPlaneSolver"] = "RansacPlaneSolver"

    # see sklearn.linear_model.RANSACRegressor for details
    min_samples: int | None = None
    residual_threshold: float | None = 0.01
    max_trials: int = 100
    max_skips: int | None = None
    stop_n_inliers: int | None = None
    stop_score: float = np.inf
    stop_probability: float = 0.99
    loss: str = "absolute_error"
    random_seed: int | None = None


@dataclass
class PointCloudFieldFilterConfig:
    type: Literal["PointCloudFieldFilter"] = "PointCloudFieldFilter"
    solver: BasePlaneSolverConfig = field(default_factory=RansacPlaneSolverConfig)
