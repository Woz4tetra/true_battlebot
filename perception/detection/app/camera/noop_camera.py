from perception_tools.messages.camera.camera_data import CameraData

from app.camera.camera_interface import CameraInterface
from app.config.camera_config.noop_config import NoopCameraConfig


class NoopCamera(CameraInterface):
    def __init__(self, config: NoopCameraConfig) -> None:
        pass

    def poll(self) -> CameraData | None:
        return None
