from perception_tools.messages.camera.camera_data import CameraData
from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.image import Image
from perception_tools.messages.camera.point_cloud import PointCloud
from perception_tools.messages.std_msgs.header import Header

from app.camera.camera_interface import CameraInterface
from app.config.camera_config.noop_camera_config import NoopCameraConfig


class NoopCamera(CameraInterface):
    def __init__(self, config: NoopCameraConfig) -> None:
        pass

    def open(self) -> bool:
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
