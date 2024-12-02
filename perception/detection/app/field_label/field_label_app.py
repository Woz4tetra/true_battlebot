import os
from typing import Any

import cv2
import numpy as np
import open3d
from app.field_label.cloud_to_open3d import cloud_to_open3d
from app.field_label.command_line_args import CommandLineArgs
from app.field_label.field_label_config import FieldLabelConfig
from app.field_label.label_state import ClickState, HighlightedPointType, LabelState
from app.field_label.transform_estimated_object import transform_estimated_object
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.camera.image_rectifier import ImageRectifier
from bw_shared.geometry.transform3d import Transform3D
from open3d.visualization import Visualizer  # type: ignore
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.point_cloud import PointCloud
from perception_tools.messages.transform_point_cloud import transform_point_cloud


class FieldLabelApp:
    def __init__(self, config: FieldLabelConfig, args: CommandLineArgs) -> None:
        self.args = args
        self.config = config
        self.labels = LabelState()
        self.cloud = PointCloud()
        self.image_size = (0, 0)

    def on_mouse(self, event: int, x: int, y: int, flags: int, param: Any):
        clipped_x = np.clip(x, 0, self.image_size[0] - 1)
        clipped_y = np.clip(y, 0, self.image_size[1] - 1)
        clipped = clipped_x, clipped_y
        if event == cv2.EVENT_MOUSEMOVE:
            if self.labels.did_click:
                self.labels.click_state = ClickState.MOVE
                point_array = (
                    self.labels.image_plane_points
                    if self.labels.highlighted_point_type == HighlightedPointType.PLANE
                    else self.labels.image_extent_points
                )
                point_array[self.labels.highlighted_index] = np.array(clipped)
            else:
                self.labels.update_highlighted_index(clipped)
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.labels.click_state = ClickState.DOWN
            if self.labels.highlighted_index != -1:
                self.labels.did_click = True
        elif event == cv2.EVENT_LBUTTONUP:
            self.labels.click_state = ClickState.UP
            if self.labels.did_click:
                self.labels.did_click = False
                self.labels.update_cloud_points(self.cloud)
                self.labels.save_label_state(self.config.label_state_path)

    def initialize_default_image_points(self, new_image_width: int, new_image_height: int) -> None:
        border = 0.3
        anchor_points = np.array(
            [
                [-border, border],
                [-border, -border],
                [border, -border],
            ]
        )
        anchor_points[:] += 1.0
        anchor_points[:] *= 0.5
        image_plane_points = anchor_points * np.array([new_image_width, new_image_height])
        self.labels.image_plane_points = image_plane_points.astype(np.int32)

        border = 0.4
        anchor_points = np.array(
            [
                [border, border],
                [-border, border],
                [-border, -border],
                [border, -border],
            ]
        )
        anchor_points[:] += 1.0
        anchor_points[:] *= 0.5
        image_extent_points = anchor_points * np.array([new_image_width, new_image_height])
        self.labels.image_extent_points = image_extent_points.astype(np.int32)

    def initialize_app(
        self, camera_data: CameraData, tf_pointcloud_from_camera: Transform3D
    ) -> tuple[np.ndarray, Visualizer]:
        pointcloud_in_camera = transform_point_cloud(
            camera_data.point_cloud, tf_pointcloud_from_camera, camera_data.color_image.header
        )

        point_cloud = cloud_to_open3d(pointcloud_in_camera, max_value=self.config.max_cloud_distance)
        o3d_downsample_cloud = point_cloud.voxel_down_sample(voxel_size=0.02)

        min_bound = np.array([-5, -5, -5])
        max_bound = np.array([5, 5, 5])
        o3d_downsample_cloud = o3d_downsample_cloud.crop(
            open3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
        )
        self.cloud = PointCloud(pointcloud_in_camera.header, o3d_downsample_cloud.points, o3d_downsample_cloud.colors)

        cv_window_name = "Image"
        window_height = 800
        ratio = window_height / camera_data.color_image.data.shape[1]
        new_image_width = int(camera_data.color_image.data.shape[1] * ratio)
        new_image_height = int(camera_data.color_image.data.shape[0] * ratio)
        rectifier = ImageRectifier(
            camera_data.camera_info, new_size=(new_image_width, new_image_height), padding=self.config.image_padding
        )

        if os.path.isfile(self.config.label_state_path):
            self.labels.load_label_state(self.config.label_state_path)
        else:
            self.initialize_default_image_points(rectifier.width, rectifier.height)
        self.image_size = (rectifier.width, rectifier.height)

        self.labels.camera_model.fromCameraInfo(rectifier.get_rectified_info())
        rectified_image = rectifier.rectify(camera_data.color_image.data)

        self.labels.update_cloud_points(self.cloud)
        self.labels.create_markers()

        vis = Visualizer()
        vis.create_window(width=window_height, height=window_height)
        vis.add_geometry(o3d_downsample_cloud)
        for marker in self.labels.plane_point_markers:
            vis.add_geometry(marker)
        for marker in self.labels.extent_point_markers:
            vis.add_geometry(marker)
        vis.add_geometry(self.labels.plane_marker)
        vis.add_geometry(self.labels.origin_vis)

        view_control = vis.get_view_control()
        view_control.set_front([0, 0, -1])
        view_control.set_up([0, -1, 0])

        cv2.namedWindow(cv_window_name)
        cv2.moveWindow(cv_window_name, window_height, 0)
        cv2.setMouseCallback(cv_window_name, self.on_mouse)

        return rectified_image, vis

    def update_3d_visualization(self, vis: Visualizer) -> bool:
        if not vis.poll_events():
            return False
        vis.update_renderer()
        for marker in self.labels.plane_point_markers:
            vis.update_geometry(marker)
        for marker in self.labels.extent_point_markers:
            vis.update_geometry(marker)
        vis.update_geometry(self.labels.plane_marker)
        vis.update_geometry(self.labels.origin_vis)
        return True

    def update_image_visualization(self, show_image: np.ndarray) -> None:
        cv2.polylines(show_image, [self.labels.image_plane_points], isClosed=True, color=(0, 255, 0), thickness=1)
        for index, point in enumerate(self.labels.image_plane_points):
            if (
                index == self.labels.highlighted_index
                and self.labels.highlighted_point_type == HighlightedPointType.PLANE
            ):
                color = (0, 0, 255)
                if self.labels.click_state == ClickState.MOVE:
                    radius = 0
                else:
                    radius = self.labels.highlighted_radius
            else:
                radius = self.labels.unhighlighted_radius
                color = (0, 255, 0)
            if radius > 0:
                cv2.circle(show_image, tuple(point), radius, color, -1)

        cv2.polylines(show_image, [self.labels.image_extent_points], isClosed=True, color=(0, 255, 128), thickness=1)
        for index, point in enumerate(self.labels.image_extent_points):
            if (
                index == self.labels.highlighted_index
                and self.labels.highlighted_point_type == HighlightedPointType.EXTENT
            ):
                color = (0, 0, 255)
                if self.labels.click_state == ClickState.MOVE:
                    radius = 0
                else:
                    radius = self.labels.highlighted_radius
            elif index == 0:
                color = (255, 0, 128)
                radius = self.labels.unhighlighted_radius
            else:
                color = (0, 255, 128)
                radius = self.labels.unhighlighted_radius
            if radius > 0:
                cv2.circle(show_image, tuple(point), radius, color, -1)

    def label_camera_data(
        self, camera_data: CameraData, tf_pointcloud_from_camera: Transform3D
    ) -> EstimatedObject | None:
        rectified_image, vis = self.initialize_app(camera_data, tf_pointcloud_from_camera)

        while True:
            if not self.update_3d_visualization(vis):
                break
            show_image = np.copy(rectified_image)
            self.update_image_visualization(show_image)
            cv2.imshow("Image", show_image)
            keycode = cv2.waitKeyEx(1)
            key = chr(keycode & 0xFF)
            if key == "q":
                break
            elif key == "\n" or key == "\r":
                print("Computed field estimate")
                field_estimate_in_pointcloud = transform_estimated_object(
                    self.labels.field_estimate, tf_pointcloud_from_camera, camera_data.point_cloud.header
                )
                print(field_estimate_in_pointcloud)
                return field_estimate_in_pointcloud
        return None
