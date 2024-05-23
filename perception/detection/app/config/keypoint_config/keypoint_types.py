from typing import Union

from .noop_keypoint_config import NoopKeypointConfig
from .simulated_keypoint_config import SimulatedKeypointConfig
from .yolo_keypoint_config import YoloKeypointConfig

KeypointConfig = Union[NoopKeypointConfig, YoloKeypointConfig, SimulatedKeypointConfig]
