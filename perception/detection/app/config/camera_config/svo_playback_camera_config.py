from dataclasses import dataclass
from typing import Literal


@dataclass
class SvoPlaybackCameraConfig:
    type: Literal["SvoPlaybackCamera"] = "SvoPlaybackCamera"
    path: str = "/media/storage/svo/playback.svo"
    field_grab_index: int = 0
    start_index: int = 0
