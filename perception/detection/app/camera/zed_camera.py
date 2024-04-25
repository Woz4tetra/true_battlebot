from perception_tools.messages.camera.camera_data import CameraData

from app.camera.camera_interface import CameraInterface
from app.config.camera_config.zed_camera_config import ZedCameraConfig


class ZedCamera(CameraInterface):
    def __init__(self, config: ZedCameraConfig) -> None:
        pass

    def poll(self) -> CameraData | None:
        return None
