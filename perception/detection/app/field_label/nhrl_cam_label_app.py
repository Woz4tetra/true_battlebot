import os
from typing import Any

import cv2
import numpy as np
from app.field_label.click_state import ClickState
from app.field_label.command_line_args import CommandLineArgs
from app.field_label.field_label_app import FieldLabelApp
from app.field_label.nhrl_cam_label_config import NhrlCamLabelConfig
from app.field_label.nhrl_cam_label_state import NhrlCamLabelState
from bw_shared.geometry.projection_math.points_transform import points_transform_by
from bw_shared.geometry.projection_math.project_segmentation import project_segmentation
from bw_shared.geometry.transform3d import Transform3D
from numba import njit
from open3d.visualization import Visualizer  # type: ignore
from perception_tools.geometry.project_pixel_to_3d_ray import project_pixel_to_3d_ray
from perception_tools.geometry.transform_to_plane import transform_to_plane
from perception_tools.geometry.xyzquat_to_matrix import xyzquat_to_matrix
from perception_tools.messages.camera_data import CameraData


# @njit
def cost_function(
    known_camera_points: np.ndarray,
    known_nhrl_pixels: np.ndarray,
    tf_knowncamera_from_map: np.ndarray,
    nhrl_cx: float,
    nhrl_cy: float,
    nhrl_fx: float,
    nhrl_fy: float,
    map_nhrl_x: float,
    map_nhrl_y: float,
    map_nhrl_z: float,
    map_nhrl_qx: float,
    map_nhrl_qy: float,
    map_nhrl_qz: float,
    map_nhrl_qw: float,
) -> float:
    tf_map_from_nhrl = xyzquat_to_matrix(
        map_nhrl_x, map_nhrl_y, map_nhrl_z, map_nhrl_qx, map_nhrl_qy, map_nhrl_qz, map_nhrl_qw
    )
    tf_camera_from_nhrl = np.dot(tf_knowncamera_from_map, tf_map_from_nhrl)
    predicted_plane_center, predicted_plane_normal = transform_to_plane(tf_map_from_nhrl)
    rays = np.empty((0, 3))
    for u, v in known_nhrl_pixels:
        ray = project_pixel_to_3d_ray(u, v, nhrl_cx, nhrl_cy, nhrl_fx, nhrl_fy)
        rays = np.append(rays, ray)
    predicted_points_nhrl = project_segmentation(rays, predicted_plane_center, predicted_plane_normal)
    predicted_points_camera = points_transform_by(predicted_points_nhrl, tf_camera_from_nhrl)
    return float(np.sum(np.linalg.norm(known_camera_points - predicted_points_camera)))


class NhrlCamLabelApp(FieldLabelApp):
    def __init__(
        self,
        config: NhrlCamLabelConfig,
        args: CommandLineArgs,
        camera_data: CameraData,
        tf_pointcloud_from_camera: Transform3D,
        video_frame: np.ndarray,
    ) -> None:
        super().__init__(config, args, camera_data, tf_pointcloud_from_camera)
        self.video_label_state = NhrlCamLabelState()
        self.config = config
        self.frame_height = self.config.video_frame_height
        self.video_frame = video_frame
        self.frame_width = int(self.frame_height * self.video_frame.shape[1] / self.video_frame.shape[0])
        self.rescaled_frame = cv2.resize(
            self.video_frame, (self.frame_width, self.frame_height), interpolation=cv2.INTER_LINEAR
        )
        self.nhrl_window_name = "NHRL camera"

    def on_nhrl_mouse(self, event: int, x: int, y: int, flags: int, param: Any) -> None:
        clipped_x = np.clip(x, 0, self.frame_width - 1)
        clipped_y = np.clip(y, 0, self.frame_height - 1)
        clipped = clipped_x, clipped_y
        if event == cv2.EVENT_MOUSEMOVE:
            if self.video_label_state.did_click:
                self.video_label_state.click_state = ClickState.MOVE
                self.video_label_state.image_points[self.video_label_state.highlighted_index] = np.array(clipped)
            else:
                self.video_label_state.update_highlighted_index(clipped)
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.video_label_state.click_state = ClickState.DOWN
            if self.video_label_state.highlighted_index != -1:
                self.video_label_state.did_click = True
        elif event == cv2.EVENT_LBUTTONUP:
            self.video_label_state.click_state = ClickState.UP
            if self.video_label_state.did_click:
                self.video_label_state.did_click = False
                self.update_pending = True
                self.video_label_state.save_label_state(self.config.video_state_path)

    def initialize_default_video_points(self) -> None:
        border = 0.3
        anchor_points_list = [
            [-border, border],
            [-border, -border],
            [border, -border],
        ]
        for _ in range(self.config.num_extra_points):
            anchor_points_list.append([np.random.random() * border, np.random.random() * border])

        anchor_points = np.array(anchor_points_list)
        anchor_points[:] += 1.0
        anchor_points[:] *= 0.5
        image_points = anchor_points * np.array([self.frame_width, self.frame_height])
        self.video_label_state.image_points = image_points.astype(np.int32)

    def initialize_app(self) -> tuple[np.ndarray, Visualizer]:
        cv2.namedWindow(self.nhrl_window_name)
        cv2.moveWindow(self.nhrl_window_name, 1600, 0)
        cv2.setMouseCallback(self.nhrl_window_name, self.on_nhrl_mouse)

        if os.path.isfile(self.config.video_state_path):
            self.video_label_state.load_label_state(self.config.video_state_path)
        else:
            self.initialize_default_video_points()

        return super().initialize_app()

    def update_video_visualization(self, show_image: np.ndarray) -> None:
        cv2.polylines(show_image, [self.video_label_state.image_points], isClosed=True, color=(0, 255, 0), thickness=1)
        for index, point in enumerate(self.video_label_state.image_points):
            if index == self.video_label_state.highlighted_index:
                color = (0, 0, 255)
                if self.video_label_state.click_state == ClickState.MOVE:
                    radius = 0
                else:
                    radius = self.video_label_state.highlighted_radius
            else:
                radius = self.video_label_state.unhighlighted_radius
                if index <= 2:
                    color = (0, 255, 0)
                else:
                    color = (200, 200, 200)
            if radius > 0:
                cv2.circle(show_image, tuple(point), radius, color, -1)

    def compute_camera_geometry(self) -> None:
        print("camera_model", self.label_state.camera_model)
        print("image_points", self.video_label_state.image_points)
        print("image_plane_points", self.label_state.image_plane_points)
        print("cloud_plane_points", self.label_state.cloud_plane_points)
        print("field_estimate", self.label_state.field_estimate)

        print(self.label_state.camera_model.cx())
        print(self.label_state.camera_model.cy())
        print(self.label_state.camera_model.fx())
        print(self.label_state.camera_model.fy())

        all_image_points = self.video_label_state.image_points

        # ret, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        #     all_object_points,
        #     all_image_points,
        #     image_size,
        #     None,  # type: ignore
        #     None,  # type: ignore
        # )

    def compute_result_before_exit(self) -> None:
        self.compute_camera_geometry()
        super().compute_result_before_exit()

    def tick_labeling(self, rectified_image: np.ndarray, vis: Visualizer) -> bool:
        is_running = super().tick_labeling(rectified_image, vis)
        show_frame = np.copy(self.rescaled_frame)
        self.update_video_visualization(show_frame)
        cv2.imshow(self.nhrl_window_name, show_frame)
        return is_running
