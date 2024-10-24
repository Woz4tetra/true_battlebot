from typing import Union

from .light_simulated_camera_config import LightSimulatedCameraConfig
from .noop_camera_config import NoopCameraConfig
from .simulated_camera_config import SimulatedCameraConfig
from .svo_playback_camera_config import SvoPlaybackCameraConfig
from .zed_camera_config import ZedCameraConfig

CameraConfig = Union[
    NoopCameraConfig,
    ZedCameraConfig,
    SimulatedCameraConfig,
    SvoPlaybackCameraConfig,
    LightSimulatedCameraConfig,
]
