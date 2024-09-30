from typing import Any, cast

import cv2
import numpy as np
import open3d as o3d
import open3d.visualization
from app.field_label.command_line_args import BagCommandLineArgs, CommandLineArgs, TopicCommandLineArgs
from app.field_label.label_state import LabelState
from bw_shared.geometry.camera.image_rectifier import ImageRectifier
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import PointCloud
from rosbag.bag import Bag


def cloud_to_open3d(cloud: PointCloud, max_value: float = 10.0) -> o3d.geometry.PointCloud:
    delete_points = np.isnan(cloud.points) | ((cloud.points > max_value) | (cloud.points < -max_value))
    points = cloud.points[~np.any(delete_points, axis=1)]

    cloud_o3d = o3d.geometry.PointCloud()
    cloud_o3d.points = o3d.utility.Vector3dVector(points)
    colors = cloud.colors[~np.any(delete_points, axis=1)]
    cloud_o3d.colors = o3d.utility.Vector3dVector(colors)

    return cloud_o3d


def load_from_bag(bag_file: str, cloud_topic: str, image_topic: str, info_topic: str) -> CameraData:
    point_cloud = None
    color_image = None
    camera_info = None
    with Bag(bag_file, "r") as bag:
        for topic, msg, timestamp in bag.read_messages(topics=[cloud_topic, image_topic, info_topic]):  # type: ignore
            if topic == cloud_topic:
                point_cloud = PointCloud.from_msg(msg)
            elif topic == image_topic:
                color_image = Image.from_msg(msg)
            elif topic == info_topic:
                camera_info = msg

            if point_cloud is not None and color_image is not None and camera_info is not None:
                break
    assert point_cloud is not None and color_image is not None and camera_info is not None
    return CameraData(color_image=color_image, point_cloud=point_cloud, camera_info=camera_info)


class FieldLabelApp:
    def __init__(self, args: CommandLineArgs) -> None:
        self.args = args
        self.labels = LabelState()
        self.cloud = PointCloud()

    def run_bag(self, args: BagCommandLineArgs) -> None:
        camera_data = load_from_bag(args.bag_file, args.cloud_topic, args.image_topic, args.info_topic)
        self.label_camera_data(camera_data)

    def on_mouse(self, event: int, x: int, y: int, flags: int, param: Any):
        if event == cv2.EVENT_MOUSEMOVE:
            if self.labels.is_clicked:
                self.labels.image_points[self.labels.highlighted_index] = np.array([x, y])
            else:
                self.labels.update_highlighted_index((x, y))
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.labels.is_clicked = True
        elif event == cv2.EVENT_LBUTTONUP:
            self.labels.is_clicked = False
            self.labels.update_cloud_points(self.cloud)

    def label_camera_data(self, camera_data: CameraData) -> None:
        point_cloud = cloud_to_open3d(camera_data.point_cloud)
        self.cloud = point_cloud.voxel_down_sample(voxel_size=0.02)

        cv_window_name = "Image"
        window_height = 800
        ratio = window_height / camera_data.color_image.data.shape[1]
        new_image_width = int(camera_data.color_image.data.shape[1] * ratio)
        new_image_height = int(camera_data.color_image.data.shape[0] * ratio)
        rectifier = ImageRectifier(camera_data.camera_info, new_size=(new_image_width, new_image_height))

        anchor_points = np.array(
            [
                [0.8, 0.8],
                [-0.8, 0.8],
                [-0.8, -0.8],
                [0.8, -0.8],
            ]
        )
        anchor_points[:] += 1.0
        anchor_points[:] *= 0.5
        image_points = anchor_points * np.array([new_image_width, new_image_height])
        self.labels.image_points = image_points.astype(np.int32)
        self.labels.camera_model.fromCameraInfo(rectifier.get_rectified_info())
        rectified_image = rectifier.rectify(camera_data.color_image.data)

        self.labels.update_cloud_points(self.cloud)
        self.labels.create_markers()

        vis = open3d.visualization.Visualizer()  # type: ignore
        vis.create_window(width=window_height, height=window_height)
        vis.add_geometry(self.cloud)
        for marker in self.labels.markers:
            vis.add_geometry(marker)

        cv2.namedWindow(cv_window_name)
        cv2.moveWindow(cv_window_name, window_height, 0)
        cv2.setMouseCallback(cv_window_name, self.on_mouse)

        while True:
            if not vis.poll_events():
                break
            vis.update_renderer()
            for marker in self.labels.markers:
                vis.update_geometry(marker)

            show_image = np.copy(rectified_image)
            cv2.polylines(show_image, [self.labels.image_points], isClosed=True, color=(0, 255, 0), thickness=1)
            for index, point in enumerate(self.labels.image_points):
                if index == self.labels.highlighted_index:
                    radius = self.labels.highlighted_radius
                    color = (0, 0, 255)
                else:
                    radius = self.labels.unhighlighted_radius
                    color = (0, 255, 0)
                cv2.circle(show_image, tuple(point), radius, color, -1)

            cv2.imshow("Image", show_image)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break

    def run_topic(self, args: TopicCommandLineArgs) -> None:
        pass

    def run(self) -> None:
        match self.args.command:
            case "bag":
                self.run_bag(cast(BagCommandLineArgs, self.args))
            case "topic":
                self.run_topic(cast(TopicCommandLineArgs, self.args))
            case _:
                raise RuntimeError(f"Unknown command: {self.args.command}")
