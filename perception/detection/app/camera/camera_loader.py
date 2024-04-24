from typing import Union

from app.config.camera_config.camera_types import CameraConfig
from app.config.camera_config.noop_config import NoopCameraConfig
from app.config.camera_config.zed_config import ZedConfig

from .noop_camera import NoopCamera
from .zed_camera import ZedCamera

CameraImplementation = Union[ZedCamera, NoopCamera]


def load_camera(config: CameraConfig) -> CameraImplementation:
    return {ZedConfig: ZedCamera, NoopCameraConfig: NoopCamera}[type(config)](config)
