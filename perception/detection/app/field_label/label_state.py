import json
from enum import Enum

import numpy as np
import open3d
import open3d as o3d
from bw_interfaces.msg import EstimatedObject
from bw_shared.enums.label import Label
from bw_shared.geometry.projection_math.plane_from_3_points import plane_from_3_points
from bw_shared.geometry.projection_math.points_transform import points_transform_by
from bw_shared.geometry.projection_math.project_segmentation import project_segmentation
from bw_shared.geometry.projection_math.rotation_matrix_from_vectors import transform_matrix_from_vectors
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.xy import XY
from geometry_msgs.msg import Vector3
from image_geometry import PinholeCameraModel
from perception_tools.messages.point_cloud import PointCloud


class HighlightedPointType(Enum):
    PLANE = 0
    EXTENT = 1


class ClickState(Enum):
    UP = 0
    MOVE = 1
    DOWN = 2


def nearest_point_in_cloud(cloud: o3d.geometry.PointCloud, point: np.ndarray) -> np.ndarray:
    points = np.asarray(cloud.points)
    distances = np.linalg.norm(points - point, axis=1)
    return points[np.argmin(distances)]


def compute_plane(plane_points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    point1, point2, point3 = plane_points
    plane_tri_points = np.array([point1, point2, point3])
    plane_normal = -1 * plane_from_3_points(point1, point2, point3)
    plane_center = np.mean(plane_tri_points, axis=0)
    return plane_center, plane_normal


def compute_field_estimate(
    plane_center: np.ndarray, plane_normal: np.ndarray, extent_points: np.ndarray
) -> tuple[Transform3D, XY]:
    plane_transform = transform_matrix_from_vectors(plane_center, plane_normal)
    flattened_points = points_transform_by(extent_points, plane_transform.inverse().tfmat)
    centeroid = np.mean(flattened_points, axis=0)

    flat_point1 = flattened_points[0]
    flat_point2 = flattened_points[1]
    angle_1_2 = np.arctan2(flat_point2[1] - flat_point1[1], flat_point2[0] - flat_point1[0])
    angle_1_2 += np.pi
    flat_transform = Transform3D.from_position_and_rpy(Vector3(centeroid[0], centeroid[1], 0.0), RPY((0, 0, angle_1_2)))
    field_centered_plane = flat_transform.forward_by(plane_transform)

    length_1_2 = float(np.linalg.norm(flattened_points[1] - flattened_points[0]))
    length_2_3 = float(np.linalg.norm(flattened_points[2] - flattened_points[1]))
    extents = XY(length_2_3, length_1_2)

    return field_centered_plane, extents


class LabelState:
    def __init__(self) -> None:
        self.unhighlighted_radius = 3
        self.highlighted_radius = 10
        self.highlighted_index = -1
        self.highlighted_point_type = HighlightedPointType.PLANE
        self.image_plane_points = np.zeros((0, 2), dtype=np.int32)
        self.image_extent_points = np.zeros((0, 2), dtype=np.int32)
        self.cloud_plane_points = np.zeros((0, 3), dtype=np.float32)
        self.cloud_extent_points = np.zeros((0, 3), dtype=np.float32)
        self.did_click = False
        self.click_state = ClickState.UP
        self.camera_model = PinholeCameraModel()
        self.plane_point_markers: list[open3d.geometry.TriangleMesh] = []
        self.extent_point_markers: list[open3d.geometry.TriangleMesh] = []
        self.plane_marker = open3d.geometry.TriangleMesh.create_box(1, 1, 0.001)
        self.plane_marker.translate([-0.5, -0.5, 0.0])
        self.prev_plane_tf = np.eye(4)
        self.origin_vis = open3d.geometry.TriangleMesh.create_coordinate_frame()
        self.field_estimate = EstimatedObject()

    def save_label_state(self, save_path: str) -> None:
        state = {
            "image_plane_points": self.image_plane_points.tolist(),
            "image_extent_points": self.image_extent_points.tolist(),
        }
        with open(save_path, "w") as file:
            json.dump(state, file)

    def load_label_state(self, load_path: str) -> None:
        with open(load_path, "r") as file:
            state = json.load(file)
            self.image_plane_points = np.array(state["image_plane_points"], dtype=np.int32)
            self.image_extent_points = np.array(state["image_extent_points"], dtype=np.int32)

    def create_markers(self) -> None:
        for point in self.cloud_plane_points:
            marker = open3d.geometry.TriangleMesh.create_sphere(radius=0.03)
            marker.compute_vertex_normals()
            marker.paint_uniform_color([0.0, 1.0, 0.0])
            marker.translate(point, relative=False)
            self.plane_point_markers.append(marker)
        for point in self.cloud_extent_points:
            marker = open3d.geometry.TriangleMesh.create_sphere(radius=0.03)
            marker.compute_vertex_normals()
            marker.paint_uniform_color([0.0, 1.0, 0.5])
            marker.translate(point, relative=False)
            self.extent_point_markers.append(marker)
        self.plane_marker.compute_vertex_normals()
        self.plane_marker.paint_uniform_color([0.8, 0.0, 0.0])

    def update_highlighted_index(self, mouse_point: tuple[int, int]) -> None:
        mouse_array = np.array(mouse_point)
        for index, point in enumerate(self.image_plane_points):
            if np.linalg.norm(point - mouse_array) < self.highlighted_radius:
                self.highlighted_index = index
                self.highlighted_point_type = HighlightedPointType.PLANE
                return
        for index, point in enumerate(self.image_extent_points):
            if np.linalg.norm(point - mouse_array) < self.highlighted_radius:
                self.highlighted_index = index
                self.highlighted_point_type = HighlightedPointType.EXTENT
                return
        self.highlighted_index = -1

    def update_cloud_points(self, cloud: PointCloud) -> None:
        min_point = np.min(cloud.points, axis=0)
        max_point = np.max(cloud.points, axis=0)
        max_distance = np.linalg.norm(max_point - min_point) * 2
        self.cloud_plane_points = np.zeros((len(self.image_plane_points), 3), dtype=np.float32)
        self.cloud_extent_points = np.zeros((len(self.image_extent_points), 3), dtype=np.float32)

        for index, uv_point in enumerate(self.image_plane_points):
            ray = np.array(self.camera_model.projectPixelTo3dRay(uv_point))
            if np.any(np.isnan(ray)):
                raise RuntimeError("Failed to project pixel to 3D ray")
            nearest_distance = None
            nearest_cloud_point = None
            for distance in np.linspace(0.01, max_distance, 100):
                point = ray * distance
                nearest_point_to_ray = nearest_point_in_cloud(cloud, point)
                distance = np.linalg.norm(nearest_point_to_ray - point)
                if nearest_distance is None or distance < nearest_distance:
                    nearest_distance = distance
                    nearest_cloud_point = nearest_point_to_ray
            self.cloud_plane_points[index] = nearest_cloud_point

        plane_center, plane_normal = compute_plane(self.cloud_plane_points)
        extent_rays = []
        for index, uv_point in enumerate(self.image_extent_points):
            ray = np.array(self.camera_model.projectPixelTo3dRay(uv_point))
            if np.any(np.isnan(ray)):
                raise RuntimeError("Failed to project pixel to 3D ray")
            extent_rays.append(ray)
        self.cloud_extent_points = project_segmentation(np.array(extent_rays), plane_center, plane_normal)

        plane_transform, plane_extents = compute_field_estimate(plane_center, plane_normal, self.cloud_extent_points)
        self.field_estimate = EstimatedObject()
        self.field_estimate.header = cloud.header.to_msg()
        self.field_estimate.child_frame_id = "map_relative"
        self.field_estimate.pose.pose = plane_transform.to_pose_msg()
        self.field_estimate.size = plane_extents.to_msg()
        self.field_estimate.label = Label.FIELD.value

        for index, marker in enumerate(self.plane_point_markers):
            marker.translate(self.cloud_plane_points[index], relative=False)
        for index, marker in enumerate(self.extent_point_markers):
            marker.translate(self.cloud_extent_points[index], relative=False)

        h_tf = plane_transform.tfmat
        scaling_matrix = np.array(
            [
                [plane_extents[1], 0, 0, 0],
                [0, plane_extents[0], 0, 0],
                [0, 0, 1.0, 0],
                [0, 0, 0, 1.0],
            ]
        )
        h_tf = np.dot(h_tf, scaling_matrix)
        vis_tf = np.dot(h_tf, np.linalg.inv(self.prev_plane_tf))
        self.prev_plane_tf = h_tf
        self.plane_marker.transform(vis_tf)
        self.origin_vis.transform(vis_tf)
