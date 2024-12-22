import os
from typing import Any

import cv2
import numpy as np
from app.field_label.click_state import ClickState
from app.field_label.command_line_args import CommandLineArgs
from app.field_label.field_label_app import FieldLabelApp
from app.field_label.nhrl_cam_label_config import NhrlCamLabelConfig
from app.field_label.nhrl_cam_label_state import NhrlCamLabelState
from bw_shared.geometry.camera.camera_info_loader import read_calibration
from bw_shared.geometry.projection_math.points_transform import points_transform_by
from bw_shared.geometry.projection_math.project_segmentation import project_segmentation
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from geometry_msgs.msg import Vector3
from image_geometry import PinholeCameraModel
from open3d.visualization import Visualizer  # type: ignore
from perception_tools.geometry.transform_to_plane import transform_to_plane
from perception_tools.messages.camera_data import CameraData


def cost_function(
    known_camera_points: np.ndarray,
    known_nhrl_rays: np.ndarray,
    tf_knowncamera_from_map: np.ndarray,
    tf_map_from_nhrl: np.ndarray,
) -> float:
    tf_camera_from_nhrl = np.dot(tf_knowncamera_from_map, tf_map_from_nhrl)
    predicted_plane_center, predicted_plane_normal = transform_to_plane(tf_map_from_nhrl)
    predicted_points_nhrl = project_segmentation(known_nhrl_rays, predicted_plane_center, predicted_plane_normal)
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
        self.nhrl_camera_model = PinholeCameraModel()
        self.nhrl_camera_model.fromCameraInfo(read_calibration(config.nhrl_camera_calibration_path))
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
        rectified_image, vis = super().initialize_app()
        cv2.namedWindow(self.nhrl_window_name)
        cv2.moveWindow(self.nhrl_window_name, 1600, 0)
        cv2.setMouseCallback(self.nhrl_window_name, self.on_nhrl_mouse)

        if os.path.isfile(self.config.video_state_path):
            self.video_label_state.load_label_state(self.config.video_state_path)
        else:
            self.initialize_default_video_points()
        self.video_label_state.create_markers()

        for marker in self.video_label_state.plane_point_markers:
            vis.add_geometry(marker)

        view_control = vis.get_view_control()
        view_control.set_front([0, 0, -1])
        view_control.set_up([0, -1, 0])

        return rectified_image, vis

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

    def update_3d_visualization(self, vis: Visualizer) -> bool:
        should_exit = super().update_3d_visualization(vis)
        for marker in self.video_label_state.plane_point_markers:
            vis.update_geometry(marker)
        vis.update_geometry(self.video_label_state.camera_coordinate_frame)
        return should_exit

    def compute_camera_geometry(self) -> None:
        nhrl_camera_pixels = self.video_label_state.image_points

        tf_map_from_camera0 = Transform3D.from_pose_msg(self.label_state.field_estimate.pose.pose)
        tf_camera0_from_map = tf_map_from_camera0.inverse()
        camera_0_plane_center, camera_0_plane_normal = transform_to_plane(tf_map_from_camera0.tfmat)
        camera_0_plane_points = project_segmentation(
            self.label_state.cloud_plane_points, camera_0_plane_center, camera_0_plane_normal
        )

        tf_map_from_nhrl_guess = Transform3D.from_position_and_rpy(Vector3(1.0, 0.0, 1.0), RPY((0.0, 0.0, 90.0)))
        tf_map_from_nhrl = tf_map_from_nhrl_guess

        known_nhrl_rays = np.zeros((0, 3))
        for u, v in nhrl_camera_pixels:
            ray = self.nhrl_camera_model.projectPixelTo3dRay((u, v))
            known_nhrl_rays = np.append(known_nhrl_rays, [ray], axis=0)
        nhrl_plane_center, nhrl_plane_normal = transform_to_plane(tf_map_from_nhrl.tfmat)
        plane_points_nhrl = project_segmentation(known_nhrl_rays, nhrl_plane_center, nhrl_plane_normal)
        tf_camera0_from_nhrl = tf_camera0_from_map.forward_by(tf_map_from_nhrl)
        plane_points_nhrl_in_camera0 = points_transform_by(plane_points_nhrl, tf_camera0_from_nhrl.tfmat)

        print("\nGuess cost:")
        print(cost_function(camera_0_plane_points, known_nhrl_rays, tf_camera0_from_map.tfmat, tf_map_from_nhrl.tfmat))

        print("\nTF map from NHRL:")
        print(tf_map_from_nhrl)

        self.video_label_state.set_plane_points(plane_points_nhrl_in_camera0)
        self.video_label_state.set_camera_coordinate_frame(tf_camera0_from_nhrl)

    def compute_result_before_exit(self) -> None:
        self.compute_camera_geometry()
        super().compute_result_before_exit()

    def tick_labeling(self, rectified_image: np.ndarray, vis: Visualizer) -> bool:
        is_running = super().tick_labeling(rectified_image, vis)
        show_frame = np.copy(self.rescaled_frame)
        self.update_video_visualization(show_frame)
        cv2.imshow(self.nhrl_window_name, show_frame)
        return is_running

    def key_callback(self, vis: Visualizer, key: str) -> None:
        if key == "j":
            self.compute_camera_geometry()
