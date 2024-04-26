import logging
import time

from perception_tools.messages.camera.camera_data import CameraData
from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.compressed_image import CompressedImage
from perception_tools.messages.camera.image import Image
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber

from app.camera.camera_interface import CameraInterface
from app.config.camera_config.simulated_camera_config import SimulatedCameraConfig


class SimulatedCamera(CameraInterface):
    def __init__(
        self,
        config: SimulatedCameraConfig,
        color_image_sub: RosPollSubscriber[CompressedImage],
        depth_image_sub: RosPollSubscriber[CompressedImage],
        camera_info_sub: RosPollSubscriber[CameraInfo],
    ) -> None:
        self.config = config
        self.color_image_sub = color_image_sub
        self.depth_image_sub = depth_image_sub
        self.camera_info_sub = camera_info_sub
        self.camera_data = CameraData()
        self.logger = logging.getLogger("perception")

    def poll(self) -> CameraData | None:
        now = time.time()
        if color := self.color_image_sub.receive():
            color_image_time = color.header.stamp
            self.logger.debug(f"Received color image. Delay: {now - color_image_time}")
            self.camera_data.color_image = Image.from_compressed(color)
        if depth := self.depth_image_sub.receive():
            depth_image_time = depth.header.stamp
            self.logger.debug(f"Received depth image. Delay: {now - depth_image_time}")
            self.camera_data.depth_image = Image.from_compressed(depth)
        if camera_info := self.camera_info_sub.receive():
            info_time = camera_info.header.stamp
            self.logger.debug(f"Received info. Delay: {now - info_time}")
            self.camera_data.camera_info = camera_info
            return self.camera_data
        return None
