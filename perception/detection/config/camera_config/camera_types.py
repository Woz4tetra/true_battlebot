from typing import Union

from .noop_config import NoopCameraConfig
from .zed_config import ZedConfig

CameraConfig = Union[NoopCameraConfig, ZedConfig]
