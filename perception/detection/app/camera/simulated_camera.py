import logging
import time

import numpy as np
from app.camera.camera_interface import CameraInterface, CameraMode
from app.config.camera_config.simulated_camera_config import SimulatedCameraConfig
from app.config.camera_topic_config import CameraTopicConfig
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import PointCloud
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Empty


class SimulatedCamera(CameraInterface):
    def __init__(
        self,
        config: SimulatedCameraConfig,
        camera_topic_config: CameraTopicConfig,
        color_image_sub: RosPollSubscriber[RosImage],
        depth_image_sub: RosPollSubscriber[RosImage],
        camera_info_sub: RosPollSubscriber[CameraInfo],
        depth_request_pub: RosPublisher[Empty],
    ) -> None:
        self.config = config
        self.camera_topic_config = camera_topic_config
        self.color_image_sub = color_image_sub
        self.depth_image_sub = depth_image_sub
        self.camera_info_sub = camera_info_sub
        self.depth_request_pub = depth_request_pub
        self.camera_data = CameraData()
        self.mode = CameraMode.ROBOT_FINDER
        self.mode_change_time = 0.0
        self.depth_receive_time = 0.0
        self.logger = logging.getLogger("perception")

    def open(self, mode: CameraMode) -> bool:
        if self.mode != mode:
            self.mode_change_time = time.monotonic()
        self.mode = mode
        return True

    def check_frame_id(self, frame_id: str) -> None:
        if frame_id != self.camera_topic_config.frame_id:
            self.logger.warning(f"Invalid frame_id: {frame_id}")

    def poll(self) -> CameraData | None:
        now = time.time()
        if self.mode == CameraMode.FIELD_FINDER:
            if self.mode_change_time > self.depth_receive_time:
                self.logger.debug("Requested depth image")
                self.depth_request_pub.publish(Empty())
        if color := self.color_image_sub.receive():
            self._update_color(now, color)
        if depth := self.depth_image_sub.receive():
            self._update_depth(now, depth)
        if camera_info := self.camera_info_sub.receive():
            self._update_camera_info(now, camera_info)
        return self.camera_data

    def _update_camera_info(self, now: float, camera_info: CameraInfo) -> None:
        info_time = camera_info.header.stamp.to_sec()
        self.check_frame_id(camera_info.header.frame_id)
        self.logger.debug(f"Received info. Delay: {now - info_time}")
        self.camera_data.camera_info = camera_info

    def _update_color(self, now: float, color: RosImage) -> None:
        color_image_time = color.header.stamp.to_sec()
        self.logger.debug(f"Received color image. Delay: {now - color_image_time}")
        self.camera_data.color_image = Image.from_msg(color)
        self.check_frame_id(color.header.frame_id)

    def _update_depth(self, now: float, depth: RosImage) -> None:
        depth_image_time = depth.header.stamp.to_sec()
        self.check_frame_id(depth.header.frame_id)
        self.logger.debug(f"Received depth image. Delay: {now - depth_image_time}")
        if (
            len(self.camera_data.color_image.header.frame_id) > 0
            and len(self.camera_data.camera_info.header.frame_id) > 0
        ):
            depth_image = Image.from_msg(depth, "passthrough")
            depth_image.data = depth_image.data.astype(np.uint16)
            self.camera_data.point_cloud = PointCloud.from_rgbd(
                self.camera_data.color_image,
                depth_image,
                self.camera_data.camera_info,
            )
            self.depth_receive_time = time.monotonic()

    def close(self) -> None:
        pass
