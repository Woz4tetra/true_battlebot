from camera.camera_interface import CameraInterface
from config.camera_config.noop_config import NoopCameraConfig


class NoopCamera(CameraInterface):
    def __init__(self, config: NoopCameraConfig) -> None:
        pass
