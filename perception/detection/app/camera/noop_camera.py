import numpy as np
from app.camera.camera_interface import CameraInterface, CameraMode
from app.config.camera.noop_camera_config import NoopCameraConfig
from bw_shared.messages.header import Header
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import PointCloud
from sensor_msgs.msg import CameraInfo


class NoopCamera(CameraInterface):
    def __init__(self, config: NoopCameraConfig) -> None:
        pass

    def open(self) -> bool:
        return True

    def switch_mode(self, mode: CameraMode) -> bool:
        return True

    def poll(self) -> CameraData | None:
        header = Header.auto()
        return CameraData(
            color_image=Image(header, np.zeros((480, 640, 3), dtype=np.uint8)),
            point_cloud=PointCloud(header),
            camera_info=CameraInfo(header=header.to_msg()),
        )

    def close(self) -> None:
        pass
