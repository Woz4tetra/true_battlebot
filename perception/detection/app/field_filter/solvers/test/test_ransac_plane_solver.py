import numpy as np
from app.field_filter.solvers.ransac_plane_solver import RansacPlaneSolver
from bw_shared.geometry.projection_math.points_transform import points_forward_by
from bw_shared.geometry.projection_math.rotation_matrix_from_vectors import rotation_matrix_from_vectors


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
    expected_normal = np.array([plane_coeffs[0], plane_coeffs[1], -1])
    expected_normal = expected_normal / np.linalg.norm(expected_normal)


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
    flattened_points = points_forward_by(points, np.linalg.inv(plane_tfmat))
    z_points = flattened_points[:, 2]
    print(np.min(z_points), np.max(z_points))
    assert np.allclose(z_points, np.mean(z_points, axis=0), atol=1e-6)
