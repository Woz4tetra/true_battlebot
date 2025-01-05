import numpy as np
import pytest
from app.config.field_filter.point_cloud_field_filter_config import (
    LeastSquaresPlaneSolverConfig,
    PointCloudFieldFilterConfig,
    RansacPlaneSolverConfig,
)
from app.field_filter.point_cloud_field_filter import PointCloudFieldFilter
from app.field_filter.solvers.least_squares_solver import LeastSquaresSolver
from app.field_filter.solvers.ransac_plane_solver import RansacPlaneSolver


@pytest.fixture
def ransac_plane_solver() -> RansacPlaneSolver:
    return RansacPlaneSolver(
        RansacPlaneSolverConfig(
            min_samples=None,
            residual_threshold=0.01,
            max_trials=100,
            max_skips=None,
            stop_n_inliers=None,
            stop_score=np.inf,
            stop_probability=0.99,
            loss="absolute_error",
            random_seed=4176,
        )
    )


@pytest.fixture
def least_squares_plane_solver() -> LeastSquaresSolver:
    return LeastSquaresSolver(
        LeastSquaresPlaneSolverConfig(
            inlier_threshold=0.1,
        )
    )


@pytest.fixture
def field_filter(ransac_plane_solver: RansacPlaneSolver) -> PointCloudFieldFilter:
    return PointCloudFieldFilter(
        plane_solver=ransac_plane_solver, filter_config=PointCloudFieldFilterConfig(solver=ransac_plane_solver.config)
    )
