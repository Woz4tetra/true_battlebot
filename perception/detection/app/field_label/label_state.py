import numpy as np
import open3d
import open3d as o3d
from app.field_label.field_label_app import nearest_point_in_cloud
from image_geometry import PinholeCameraModel


class LabelState:
    def __init__(self) -> None:
        self.ratio = 1.0
        self.unhighlighted_radius = 6
        self.highlighted_radius = 10
        self.highlighted_index = -1
        self.image_points = np.zeros((0, 2), dtype=np.int32)
        self.cloud_points = np.zeros((0, 3), dtype=np.float32)
        self.is_clicked = False
        self.interpolate_indices = [1, 3, 5, 7]
        self.camera_model = PinholeCameraModel()
        self.markers = []

    def create_markers(self) -> None:
        for point in self.cloud_points:
            marker = open3d.geometry.TriangleMesh.create_sphere(radius=0.05)
            marker.compute_vertex_normals()
            marker.paint_uniform_color([1.0, 0.0, 0.0])
            marker.translate(point, relative=False)
            self.markers.append(marker)

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
        print(f"Max distance: {max_distance}")
        self.cloud_points = np.zeros((len(self.image_points), 3), dtype=np.float32)
        for index, scaled_uv_point in enumerate(self.image_points):
            uv_point = scaled_uv_point / self.ratio
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
        for index, marker in enumerate(self.markers):
            marker.translate(self.cloud_points[index], relative=False)
