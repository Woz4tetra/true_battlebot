import numpy as np
import open3d
import open3d as o3d
from bw_shared.geometry.projection_math.plane_from_3_points import plane_from_3_points
from bw_shared.geometry.projection_math.points_transform import points_transform_by
from bw_shared.geometry.projection_math.rotation_matrix_from_vectors import transform_matrix_from_vectors
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.xy import XY
from geometry_msgs.msg import Vector3
from image_geometry import PinholeCameraModel


def nearest_point_in_cloud(cloud: o3d.geometry.PointCloud, point: np.ndarray) -> np.ndarray:
    points = np.asarray(cloud.points)
    distances = np.linalg.norm(points - point, axis=1)
    return points[np.argmin(distances)]


def compute_field_estimate(point1: np.ndarray, point2: np.ndarray, point3: np.ndarray) -> tuple[Transform3D, XY]:
    plane_points = np.array([point1, point2, point3])
    plane_normal = plane_from_3_points(point1, point2, point3)
    plane_center = np.mean(plane_points, axis=0)
    plane_transform = transform_matrix_from_vectors(plane_center, plane_normal)
    flattened_points = points_transform_by(plane_points, plane_transform.inverse().tfmat)
    centeroid = np.mean(flattened_points, axis=0)

    length_1_2 = float(np.linalg.norm(flattened_points[1] - flattened_points[0]))
    length_2_3 = float(np.linalg.norm(flattened_points[2] - flattened_points[1]))
    angle_2_3 = np.arctan2(point3[1] - point2[1], point3[0] - point2[0])
    flat_transform = Transform3D.from_position_and_rpy(Vector3(centeroid[0], centeroid[1], 0.0), RPY((0, 0, angle_2_3)))
    field_centered_plane = flat_transform.forward_by(plane_transform)

    extents = XY(length_1_2, length_2_3)

    return field_centered_plane, extents


class LabelState:
    def __init__(self) -> None:
        self.unhighlighted_radius = 3
        self.highlighted_radius = 10
        self.highlighted_index = -1
        self.image_points = np.zeros((0, 2), dtype=np.int32)
        self.cloud_points = np.zeros((0, 3), dtype=np.float32)
        self.is_clicked = False
        self.camera_model = PinholeCameraModel()
        self.point_markers = []
        self.plane_marker = open3d.geometry.TriangleMesh.create_box(1, 1, 0.01)
        self.field_estimate = (Transform3D.identity(), XY(0, 0))

    def create_markers(self) -> None:
        for point in self.cloud_points:
            marker = open3d.geometry.TriangleMesh.create_sphere(radius=0.05)
            marker.compute_vertex_normals()
            marker.paint_uniform_color([1.0, 0.0, 0.0])
            marker.translate(point, relative=False)
            self.point_markers.append(marker)
        self.plane_marker.compute_vertex_normals()
        self.plane_marker.paint_uniform_color([0.8, 0.0, 0.0])

    def update_highlighted_index(self, mouse_point: tuple[int, int]) -> None:
        mouse_array = np.array(mouse_point)
        for index, point in enumerate(self.image_points):
            if np.linalg.norm(point - mouse_array) < self.highlighted_radius:
                self.highlighted_index = index
                return
        self.highlighted_index = -1

    def update_cloud_points(self, cloud: o3d.geometry.PointCloud) -> None:
        min_point = np.min(cloud.points, axis=0)
        max_point = np.max(cloud.points, axis=0)
        max_distance = np.linalg.norm(max_point - min_point) * 2
        self.cloud_points = np.zeros((len(self.image_points), 3), dtype=np.float32)
        for index, uv_point in enumerate(self.image_points):
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
            self.cloud_points[index] = nearest_cloud_point
        for index, marker in enumerate(self.point_markers):
            marker.translate(self.cloud_points[index], relative=False)

        self.field_estimate = compute_field_estimate(self.cloud_points[0], self.cloud_points[1], self.cloud_points[2])
        self.plane_marker.transform(self.field_estimate[0].tfmat, relative=False)
