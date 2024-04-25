import logging

from perception_tools.messages.camera.camera_data import CameraData
from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.image import Image
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber

from app.camera.camera_interface import CameraInterface
from app.config.camera_config.simulated_camera_config import SimulatedCameraConfig


class SimulatedCamera(CameraInterface):
    def __init__(
        self,
        config: SimulatedCameraConfig,
        color_image_sub: RosPollSubscriber[Image],
        depth_image_sub: RosPollSubscriber[Image],
        camera_info_sub: RosPollSubscriber[CameraInfo],
    ) -> None:
        self.config = config
        self.color_image_sub = color_image_sub
        self.depth_image_sub = depth_image_sub
        self.camera_info_sub = camera_info_sub
        self.camera_data = CameraData()
        self.logger = logging.getLogger("perception")

    def poll(self) -> CameraData | None:
        if color := self.color_image_sub.receive():
            self.camera_data.color_image = color
        if depth := self.depth_image_sub.receive():
            self.camera_data.depth_image = depth
        if camera_info := self.camera_info_sub.receive():
            self.camera_data.camera_info = camera_info
            return self.camera_data
        return None
