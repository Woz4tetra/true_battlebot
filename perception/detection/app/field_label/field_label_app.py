import os
import sys
from typing import Any, cast

import cv2
import numpy as np
import open3d as o3d
import open3d.visualization
import rospy
from app.field_label.command_line_args import BagCommandLineArgs, CommandLineArgs, TopicCommandLineArgs
from app.field_label.field_label_config import FieldLabelConfig
from app.field_label.label_state import ClickState, HighlightedPointType, LabelState
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.camera.image_rectifier import ImageRectifier
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import PointCloud
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from rosbag.bag import Bag
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import PointCloud2 as RosPointCloud
from std_msgs.msg import Empty


def cloud_to_open3d(cloud: PointCloud, max_value: float) -> o3d.geometry.PointCloud:
    delete_points = np.isnan(cloud.points) | ((cloud.points > max_value) | (cloud.points < -max_value))
    keep_rows = ~np.any(delete_points, axis=1)
    points = cloud.points[keep_rows]

    cloud_o3d = o3d.geometry.PointCloud()
    cloud_o3d.points = o3d.utility.Vector3dVector(points)
    if cloud.colors.size > 0:
        colors = cloud.colors[keep_rows]
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


def load_from_topics(
    cloud_subscriber: RosPollSubscriber[RosPointCloud],
    image_subscriber: RosPollSubscriber[RosImage],
    info_subscriber: RosPollSubscriber[CameraInfo],
) -> CameraData:
    point_cloud = None
    color_image = None
    camera_info = None
    while point_cloud is None or color_image is None or camera_info is None:
        if point_cloud is None and (cloud_msg := cloud_subscriber.receive()):
            point_cloud = PointCloud.from_msg(cloud_msg)
            print("Received point cloud")
        if color_image is None and (image_msg := image_subscriber.receive()):
            color_image = Image.from_msg(image_msg)
            print("Received color image")
        if camera_info is None and (info_msg := info_subscriber.receive()):
            camera_info = info_msg
            print("Received camera info")
        rospy.sleep(0.1)
        if rospy.is_shutdown():
            sys.exit(0)
    return CameraData(color_image=color_image, point_cloud=point_cloud, camera_info=camera_info)


class FieldLabelApp:
    def __init__(self, config: FieldLabelConfig, args: CommandLineArgs) -> None:
        self.args = args
        self.config = config
        self.labels = LabelState()
        self.cloud = PointCloud()
        self.image_size = (0, 0)

    def run_bag(self, args: BagCommandLineArgs) -> None:
        camera_data = load_from_bag(
            args.bag_file, self.config.cloud_topic, self.config.image_topic, self.config.info_topic
        )
        self.label_camera_data(camera_data)

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

    def run_topic(self, args: TopicCommandLineArgs) -> None:
        cloud_subscriber = RosPollSubscriber(self.config.cloud_topic, RosPointCloud)
        image_subscriber = RosPollSubscriber(self.config.image_topic, RosImage)
        info_subscriber = RosPollSubscriber(self.config.info_topic, CameraInfo)
        request_publisher = RosPublisher(self.config.field_request_topic, Empty)

        rospy.sleep(2.0)  # Wait for subscribers to connect

        print("Requesting camera data")
        request_publisher.publish(Empty())

        camera_data = load_from_topics(cloud_subscriber, image_subscriber, info_subscriber)
        self.label_camera_data(camera_data)

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

    def label_camera_data(self, camera_data: CameraData) -> None:
        response_publisher = RosPublisher(self.config.field_response_topic, EstimatedObject)

        point_cloud = cloud_to_open3d(camera_data.point_cloud, max_value=self.config.max_cloud_distance)
        o3d_downsample_cloud = point_cloud.voxel_down_sample(voxel_size=0.02)

        min_bound = np.array([-5, -5, -5])
        max_bound = np.array([5, 5, 5])
        o3d_downsample_cloud = o3d_downsample_cloud.crop(
            open3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
        )
        self.cloud = PointCloud(
            camera_data.point_cloud.header, o3d_downsample_cloud.points, o3d_downsample_cloud.colors
        )

        cv_window_name = "Image"
        window_height = 800
        ratio = window_height / camera_data.color_image.data.shape[1]
        new_image_width = int(camera_data.color_image.data.shape[1] * ratio)
        new_image_height = int(camera_data.color_image.data.shape[0] * ratio)
        self.image_size = (new_image_width, new_image_height)
        rectifier = ImageRectifier(camera_data.camera_info, new_size=self.image_size)

        if os.path.isfile(self.config.label_state_path):
            self.labels.load_label_state(self.config.label_state_path)
        else:
            self.initialize_default_image_points(new_image_width, new_image_height)

        self.labels.camera_model.fromCameraInfo(rectifier.get_rectified_info())
        rectified_image = rectifier.rectify(camera_data.color_image.data)

        self.labels.update_cloud_points(self.cloud)
        self.labels.create_markers()

        vis = open3d.visualization.Visualizer()  # type: ignore
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

        while True:
            if not vis.poll_events():
                break
            vis.update_renderer()
            for marker in self.labels.plane_point_markers:
                vis.update_geometry(marker)
            for marker in self.labels.extent_point_markers:
                vis.update_geometry(marker)
            vis.update_geometry(self.labels.plane_marker)
            vis.update_geometry(self.labels.origin_vis)

            show_image = np.copy(rectified_image)
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

            cv2.polylines(
                show_image, [self.labels.image_extent_points], isClosed=True, color=(0, 255, 128), thickness=1
            )
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
                else:
                    radius = self.labels.unhighlighted_radius
                    color = (0, 255, 128)
                if radius > 0:
                    cv2.circle(show_image, tuple(point), radius, color, -1)

            cv2.imshow("Image", show_image)
            keycode = cv2.waitKeyEx(1)
            key = chr(keycode & 0xFF)
            if key == "q":
                break
            elif key == "\n" or key == "\r":
                print("Computed field estimate")
                print(self.labels.field_estimate)
                response_publisher.publish(self.labels.field_estimate)

    def run(self) -> None:
        rospy.init_node("field_label_app")
        match self.args.command:
            case "bag":
                self.run_bag(cast(BagCommandLineArgs, self.args))
            case "topic":
                self.run_topic(cast(TopicCommandLineArgs, self.args))
            case _:
                raise RuntimeError(f"Unknown command: {self.args.command}")
