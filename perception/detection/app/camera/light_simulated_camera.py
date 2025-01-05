import logging

import numpy as np
from app.camera.camera_interface import CameraInterface, CameraMode
from app.config.camera.light_simulated_camera_config import LightSimulatedCameraConfig
from app.config.camera_topic_config import CameraTopicConfig
from bw_shared.messages.header import Header
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import PointCloud
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from sensor_msgs.msg import CameraInfo


class LightSimulatedCamera(CameraInterface):
    def __init__(
        self,
        config: LightSimulatedCameraConfig,
        camera_topic_config: CameraTopicConfig,
        camera_info_sub: RosPollSubscriber[CameraInfo],
    ) -> None:
        self.config = config
        self.camera_topic_config = camera_topic_config
        self.camera_info_sub = camera_info_sub
        header = Header.auto()
        self.camera_data = CameraData(
            color_image=Image(header, np.zeros((480, 640, 3), dtype=np.uint8)),
            point_cloud=PointCloud(header),
            camera_info=CameraInfo(header=header.to_msg()),
        )
        self.logger = logging.getLogger("perception")

    def open(self) -> bool:
        return True

    def switch_mode(self, mode: CameraMode) -> bool:
        return True

    def poll(self) -> CameraData | None:
        if camera_info := self.camera_info_sub.receive():
            self.camera_data.camera_info = camera_info
            self.camera_data.color_image = Image(
                Header.from_msg(camera_info.header),
                np.zeros((camera_info.height, camera_info.width, 3), dtype=np.uint8),
            )
            self.camera_data.point_cloud.header = Header.from_msg(camera_info.header)
        return self.camera_data

    def close(self) -> None:
        pass
