from typing import Union

from .noop_camera_config import NoopCameraConfig
from .simulated_camera_config import SimulatedCameraConfig
from .zed_camera_config import ZedCameraConfig

CameraConfig = Union[NoopCameraConfig, ZedCameraConfig, SimulatedCameraConfig]
