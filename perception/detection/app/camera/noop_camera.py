from bw_shared.messages.header import Header
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import PointCloud
from sensor_msgs.msg import CameraInfo

from app.camera.camera_interface import CameraInterface, CameraMode
from app.config.camera_config.noop_camera_config import NoopCameraConfig


class NoopCamera(CameraInterface):
    def __init__(self, config: NoopCameraConfig) -> None:
        pass

    def open(self, mode: CameraMode) -> bool:
        return True

    def poll(self) -> CameraData | None:
        header = Header.auto()
        return CameraData(
            color_image=Image(header),
            point_cloud=PointCloud(header),
            camera_info=CameraInfo(header),
        )

    def close(self) -> None:
        pass
