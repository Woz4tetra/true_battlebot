import numpy as np
import scipy.linalg
from app.config.field_filter_config.point_cloud_field_filter_config import LeastSquaresPlaneSolverConfig
from app.field_filter.solvers.base_plane_solver import BasePlaneSolver
from bw_shared.geometry.projection_math.points_transform import points_forward_by
from bw_shared.geometry.projection_math.rotation_matrix_from_vectors import rotation_matrix_from_vectors


class LeastSquaresSolver(BasePlaneSolver):
    def __init__(self, config: LeastSquaresPlaneSolverConfig) -> None:
        self.config = config

    def compute_inliers(self, points: np.ndarray, plane_normal: np.ndarray, threshold: float) -> np.ndarray:
        up_vec = np.array([0.0, 0.0, 1.0])
        plane_tfmat = np.eye(4)
        plane_tfmat[:3, :3] = rotation_matrix_from_vectors(plane_normal, up_vec)
        plane_tfmat[:3, 3] = np.mean(points, axis=0)
        flattened_points = points_forward_by(points, np.linalg.inv(plane_tfmat))
        z_points = flattened_points[:, 2]
        z_points -= np.mean(z_points)
        return np.abs(z_points) < threshold

    def solve(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        data_x = np.c_[points[:, 0], points[:, 1], np.ones(points.shape[0])]
        target_y = points[:, 2]
        (a, b, c), _, _, _ = scipy.linalg.lstsq(data_x, target_y)
        normal = np.array([a, b, -1])
        magnitude = np.linalg.norm(normal)
        if magnitude < 1e-6:
            return np.array([0.0, 0.0, -1.0]), np.zeros(points.shape[0], dtype=bool)
        normal = normal / magnitude
        inlier_mask = self.compute_inliers(points, normal, self.config.inlier_threshold)
        return normal, inlier_mask
