import logging

import numpy as np
from app.config.field_filter_config.point_cloud_field_filter_config import PointCloudFieldFilterConfig
from app.field_filter.field_filter_interface import FieldFilterInterface
from app.field_filter.helpers import get_field
from app.field_filter.solvers.base_plane_solver import BasePlaneSolver
from bw_interfaces.msg import EstimatedObject, SegmentationInstanceArray
from bw_shared.geometry.projection_math.find_minimum_rectangle import (
    find_minimum_rectangle,
    get_rectangle_angle,
    get_rectangle_extents,
)
from bw_shared.geometry.projection_math.points_transform import points_forward_by
from bw_shared.geometry.projection_math.rotation_matrix_from_vectors import rotation_matrix_from_vectors
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from geometry_msgs.msg import PoseWithCovariance, Vector3
from perception_tools.inference.common import msg_to_mask
from perception_tools.messages.point_cloud import PointCloud


class PointCloudFieldFilter(FieldFilterInterface):
    def __init__(self, plane_solver: BasePlaneSolver, filter_config: PointCloudFieldFilterConfig) -> None:
        self.logger = logging.getLogger("perception")
        self.field_filter_config = filter_config
        self.plane_solver = plane_solver

    def compute_plane_coeffs(self, filtered_points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return self.plane_solver.solve(filtered_points)

    def compute_plane_vectors(
        self, inlier_point_cloud: PointCloud, plane_coeffs: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        plane_normal = plane_coeffs
        plane_center = np.mean(inlier_point_cloud.points, axis=0)
        return plane_normal, plane_center

    def compute_plane_transform(self, plane_normal: np.ndarray, plane_center: np.ndarray) -> Transform3D:
        towards_camera_vec = np.array([0.0, 0.0, 1.0])
        plane_tfmat = np.eye(4)
        plane_tfmat[:3, :3] = rotation_matrix_from_vectors(plane_normal, towards_camera_vec)
        plane_tfmat[:3, 3] = plane_center
        return Transform3D(plane_tfmat)

    def compute_field(
        self, segmentations: SegmentationInstanceArray, point_cloud: PointCloud
    ) -> tuple[EstimatedObject, PointCloud | None]:
        try:
            field = get_field(segmentations)
        except ValueError as e:
            self.logger.error(f"Failed to get field segmentation: {e}")
            return EstimatedObject(), point_cloud

        self.logger.debug("Computing mask")
        mask = msg_to_mask(field.contours, segmentations.width, segmentations.height)

        if len(point_cloud.points) == 0:
            self.logger.error("Point cloud is empty. Skipping filtering.")
            return EstimatedObject(), point_cloud

        cloud_size = point_cloud.points.shape[0] * point_cloud.points.shape[1]
        if mask.size != cloud_size:
            self.logger.error(
                f"Mask size {mask.size} does not match point cloud size {cloud_size}. Skipping filtering."
            )
            return EstimatedObject(), point_cloud

        # assumes the point cloud is same shape as the contour source image
        self.logger.debug("Applying mask")
        filtered_points = point_cloud.filtered_points(mask)
        self.logger.debug(f"Filtered points shape: {filtered_points.shape}")
        plane_normal, plane_inliers = self.compute_plane_coeffs(filtered_points)
        plane_center = np.mean(filtered_points, axis=0)
        inlier_points = filtered_points[plane_inliers]
        self.logger.debug(
            f"Plane normal: {plane_normal}. "
            f"Plane center: {plane_center}. "
            f"Number of inliers: {np.sum(plane_inliers)}."
        )
        self.logger.debug(f"Inlier points shape: {inlier_points.shape}")

        # find an orientation for the plane that makes it face the camera
        plane_transform = self.compute_plane_transform(plane_normal, plane_center)
        self.logger.debug(f"Plane transform: {plane_transform}")

        # The following steps find an abitrary rotation of the plane to the camera.
        # Rotation about the normal is computed using the edge of the rectangle closest to the camera.

        # rotate points so they face the camera. This transform also normalizes with respect to the plane center
        flattened_points = points_forward_by(inlier_points, plane_transform.inverse().tfmat)
        flattened_points2d = flattened_points[:, :2]  # remove z component
        min_rect = find_minimum_rectangle(flattened_points2d)  # find minimum rectangle around 2D points
        extents = get_rectangle_extents(min_rect)  # get the 2D bounds of the rectangle
        # get the angle of the rectangle with respect to and nearest to the x-axis
        angle = get_rectangle_angle(min_rect)
        centroid = np.mean(min_rect, axis=0)  # compute 2D centroid
        # transform the flattened 2D pose back to 3D
        field_centered_plane = Transform3D.from_position_and_rpy(
            Vector3(centroid[0], centroid[1], 0), RPY((0, 0, angle))
        ).forward_by(plane_transform)
        self.logger.debug(f"Field centered plane transform: {field_centered_plane}")
        self.logger.debug(f"Minimum rectangle: {min_rect}. Rectangle extents: {extents}. Rectangle angle: {angle}.")

        estimated_field = EstimatedObject(
            header=segmentations.header,
            child_frame_id="map_relative",
            pose=PoseWithCovariance(pose=field_centered_plane.to_pose_msg()),
            size=Vector3(x=extents.x, y=extents.y, z=0.0),
            label=field.label,
        )
        inlier_point_cloud = PointCloud(header=point_cloud.header, points=inlier_points)
        self.logger.debug(f"Estimated field. Inlier point cloud shape: {inlier_point_cloud.points.shape}.")
        return estimated_field, inlier_point_cloud