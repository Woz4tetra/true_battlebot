from dataclasses import dataclass
from typing import Literal


@dataclass
class SvoPlaybackCameraConfig:
    type: Literal["SvoPlaybackCamera"] = "SvoPlaybackCamera"
    filename: str = "playback.svo"
    svo_directory: str = "/data/svo"
    field_grab_time: float = 0.0
    start_time: float = 0.0
    time_sync_threshold: float = 0.1  # seconds
