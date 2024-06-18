from dataclasses import dataclass
from typing import Literal

import numpy as np


@dataclass
class RansacFieldFilterConfig:
    type: Literal["RansacFieldFilter"] = "RansacFieldFilter"

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
