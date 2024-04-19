from typing import Union

from .noop_config import NoopConfig
from .zed_config import ZedConfig

CameraConfig = Union[NoopConfig, ZedConfig]
