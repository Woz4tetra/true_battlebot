import numpy as np
import pytest
from app.config.field_filter_config.point_cloud_field_filter_config import RansacPlaneSolverConfig
from app.field_filter.solvers.ransac_plane_solver import RansacPlaneSolver
from bw_shared.geometry.projection_math.points_transform import points_transform_by
from bw_shared.geometry.projection_math.rotation_matrix_from_vectors import rotation_matrix_from_vectors


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


def test_solve(plane_solver: RansacPlaneSolver) -> None:
    plane_normal, inlier_mask = plane_solver.solve(
        np.array(
            [
                [0, 0, 0],
                [0, 1, 0],
                [1, 1, 0],
                [1, 0, 0],
            ]
        )
    )

    assert np.sum(inlier_mask) == 4, inlier_mask
    assert np.allclose(plane_normal, [0.0, 0.0, -1.0]), plane_normal

    plane_coeffs = np.array([1.0, 2.0, 3.0])
    expected_normal = plane_coeffs / np.linalg.norm(plane_coeffs)

    x_data = np.linspace(0, 1, 100)
    y_data = np.linspace(0, 1, 100)
    xx, yy = np.meshgrid(x_data, y_data)
    zz = plane_coeffs[0] * xx + plane_coeffs[1] * yy + plane_coeffs[2]
    points = np.c_[xx.ravel(), yy.ravel(), zz.ravel()]

    plane_normal, inlier_mask = plane_solver.solve(points)
    assert np.sum(inlier_mask) == len(x_data) * len(y_data), inlier_mask
    assert np.allclose(plane_normal, expected_normal)

    up_vec = np.array([0.0, 0.0, 1.0])
    plane_tfmat = np.eye(4)
    plane_tfmat[:3, :3] = rotation_matrix_from_vectors(plane_normal, up_vec)
    plane_tfmat[:3, 3] = np.mean(points, axis=0)
    flattened_points = points_transform_by(points, np.linalg.inv(plane_tfmat))
    assert np.allclose(flattened_points[:, 2], 0.0, atol=1e-6)
