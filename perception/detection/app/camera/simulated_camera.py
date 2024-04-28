import logging
import time

from perception_tools.messages.camera.camera_data import CameraData
from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.compressed_image import CompressedImage
from perception_tools.messages.camera.image import Image
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber

from app.camera.camera_interface import CameraInterface
from app.config.camera_config.simulated_camera_config import SimulatedCameraConfig
from app.config.camera_topic_config import CameraTopicConfig


class SimulatedCamera(CameraInterface):
    def __init__(
        self,
        config: SimulatedCameraConfig,
        camera_topic_config: CameraTopicConfig,
        color_image_sub: RosPollSubscriber[CompressedImage],
        depth_image_sub: RosPollSubscriber[CompressedImage],
        camera_info_sub: RosPollSubscriber[CameraInfo],
    ) -> None:
        self.config = config
        self.camera_topic_config = camera_topic_config
        self.color_image_sub = color_image_sub
        self.depth_image_sub = depth_image_sub
        self.camera_info_sub = camera_info_sub
        self.camera_data = CameraData()
        self.logger = logging.getLogger("perception")

    def check_frame_id(self, frame_id: str) -> None:
        if frame_id != self.camera_topic_config.frame_id:
            self.logger.warning(f"Invalid frame_id: {frame_id}")

    def poll(self) -> CameraData | None:
        now = time.time()
        if color := self.color_image_sub.receive():
            color_image_time = color.header.stamp
            self.check_frame_id(color.header.frame_id)
            self.logger.debug(f"Received color image. Delay: {now - color_image_time}")
            self.camera_data.color_image = Image.from_compressed(color)
        if depth := self.depth_image_sub.receive():
            depth_image_time = depth.header.stamp
            self.check_frame_id(depth.header.frame_id)
            self.logger.debug(f"Received depth image. Delay: {now - depth_image_time}")
            self.camera_data.depth_image = Image.from_compressed(depth)
        if camera_info := self.camera_info_sub.receive():
            info_time = camera_info.header.stamp
            self.check_frame_id(camera_info.header.frame_id)
            self.logger.debug(f"Received info. Delay: {now - info_time}")
            self.camera_data.camera_info = camera_info
            return self.camera_data
        return None
