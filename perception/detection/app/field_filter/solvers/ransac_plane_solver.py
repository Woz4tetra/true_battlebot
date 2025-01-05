import numpy as np
from app.config.field_filter.point_cloud_field_filter_config import RansacPlaneSolverConfig
from app.field_filter.solvers.base_plane_solver import BasePlaneSolver
from sklearn.linear_model import RANSACRegressor


class RansacPlaneSolver(BasePlaneSolver):
    def __init__(self, config: RansacPlaneSolverConfig) -> None:
        self.config = config
        self.ransac = RANSACRegressor(
            min_samples=config.min_samples,
            residual_threshold=config.residual_threshold,
            max_trials=config.max_trials,
            max_skips=config.max_skips if config.max_skips else np.inf,  # type: ignore
            stop_n_inliers=config.stop_n_inliers if config.stop_n_inliers else np.inf,  # type: ignore
            stop_score=config.stop_score,
            stop_probability=config.stop_probability,
            loss=config.loss,
            random_state=config.random_seed,
        )

    def solve(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        data_x = points[:, 0:2]
        target_y = points[:, 2]
        self.ransac.fit(data_x, target_y)
        a, b = self.ransac.estimator_.coef_
        normal = np.array([a, b, -1])
        magnitude = np.linalg.norm(normal)
        if magnitude < 1e-6:
            return np.array([0.0, 0.0, -1.0]), self.ransac.inlier_mask_
        normal = normal / magnitude
        return normal, self.ransac.inlier_mask_
