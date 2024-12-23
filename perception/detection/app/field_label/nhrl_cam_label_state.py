import json

import numpy as np
import open3d
from app.field_label.click_state import ClickState
from bw_shared.geometry.transform3d import Transform3D


class NhrlCamLabelState:
    def __init__(self) -> None:
        self.unhighlighted_radius = 3
        self.highlighted_radius = 10
        self.highlighted_index = -1
        self.image_points = np.zeros((0, 2), dtype=np.int32)
        self.did_click = False
        self.click_state = ClickState.UP
        self.plane_point_markers: list[open3d.geometry.TriangleMesh] = []
        self.camera_coordinate_frame: open3d.geometry.TriangleMesh = open3d.geometry.TriangleMesh()
        self.prev_camera_tf = np.eye(4)

    def save_label_state(self, save_path: str) -> None:
        state = {
            "image_points": self.image_points.tolist(),
        }
        with open(save_path, "w") as file:
            json.dump(state, file)

    def load_label_state(self, load_path: str) -> None:
        with open(load_path, "r") as file:
            state = json.load(file)
            self.image_points = np.array(state["image_points"], dtype=np.int32)

    def create_markers(self) -> None:
        for _ in self.image_points:
            marker = open3d.geometry.TriangleMesh.create_sphere(radius=0.03)
            marker.compute_vertex_normals()
            marker.paint_uniform_color([0.7, 0.0, 0.7])
            self.plane_point_markers.append(marker)
        self.camera_coordinate_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])

    def update_highlighted_index(self, mouse_point: tuple[int, int]) -> None:
        mouse_array = np.array(mouse_point)
        for index, point in enumerate(self.image_points):
            if np.linalg.norm(point - mouse_array) < self.highlighted_radius:
                self.highlighted_index = index
                return
        self.highlighted_index = -1

    def set_plane_points(self, points: np.ndarray) -> None:
        if len(points) != len(self.plane_point_markers):
            raise ValueError("Number of points must match")

        for marker, point in zip(self.plane_point_markers, points):
            marker.translate(point, relative=False)

    def set_camera_coordinate_frame(self, transform: Transform3D) -> open3d.geometry.TriangleMesh:
        vis_tf = np.dot(transform.tfmat, np.linalg.inv(self.prev_camera_tf))
        self.prev_camera_tf = transform.tfmat
        self.camera_coordinate_frame.transform(vis_tf)
