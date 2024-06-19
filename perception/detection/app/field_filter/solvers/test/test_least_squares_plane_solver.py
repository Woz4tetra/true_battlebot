import numpy as np
from app.field_filter.solvers.least_squares_solver import LeastSquaresSolver


def test_solve(least_squares_plane_solver: LeastSquaresSolver) -> None:
    plane_coeffs = np.array([0.1, 0.2, 0.3])
    expected_normal = np.array([plane_coeffs[0], plane_coeffs[1], -1])
    expected_normal = expected_normal / np.linalg.norm(expected_normal)

    x_data = np.linspace(0, 1, 10)
    y_data = np.linspace(0, 1, 10)
    xx, yy = np.meshgrid(x_data, y_data)
    zz = plane_coeffs[0] * xx + plane_coeffs[1] * yy + plane_coeffs[2]
    points = np.c_[xx.ravel(), yy.ravel(), zz.ravel()]

    normal_vector, inlier_mask = least_squares_plane_solver.solve(points)
    assert np.sum(inlier_mask) == len(x_data) * len(y_data), inlier_mask
    assert np.allclose(normal_vector, expected_normal)
