from app.config.field_filter_config.point_cloud_field_filter_config import RansacPlaneSolverConfig
import pytest

from app.config.field_filter_config.point_cloud_field_filter_config import RansacPlaneSolverConfig
from app.field_filter.point_cloud_field_filter import PointCloudFieldFilter
from app.field_filter.solvers.ransac_plane_solver import RansacPlaneSolver

@pytest.fixture
def plane_solver() -> RansacPlaneSolver:
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
def field_filter(plane_solver: RansacPlaneSolver) -> PointCloudFieldFilter:
    return PointCloudFieldFilter(
        plane_solver=plane_solver
    )
