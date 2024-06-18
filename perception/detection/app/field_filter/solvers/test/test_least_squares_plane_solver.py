import numpy as np
import pytest
from app.config.field_filter_config.point_cloud_field_filter_config import LeastSquaresPlaneSolverConfig
from app.field_filter.solvers.least_squares_solver import LeastSquaresSolver


@pytest.fixture
def plane_solver() -> LeastSquaresSolver:
    return LeastSquaresSolver(
        LeastSquaresPlaneSolverConfig(
            inlier_threshold=0.1,
        )
    )


def test_solve(plane_solver: LeastSquaresSolver) -> None:
    plane_coeffs = np.array([0.1, 0.2, 0.3])
    expected_normal = np.array([plane_coeffs[0], plane_coeffs[1], -1])
    expected_normal = expected_normal / np.linalg.norm(expected_normal)

    x_data = np.linspace(0, 1, 10)
    y_data = np.linspace(0, 1, 10)
    xx, yy = np.meshgrid(x_data, y_data)
    zz = plane_coeffs[0] * xx + plane_coeffs[1] * yy + plane_coeffs[2]
    points = np.c_[xx.ravel(), yy.ravel(), zz.ravel()]

    normal_vector, inlier_mask = plane_solver.solve(points)
    assert np.sum(inlier_mask) == len(x_data) * len(y_data), inlier_mask
    assert np.allclose(normal_vector, expected_normal)
