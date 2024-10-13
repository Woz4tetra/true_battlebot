from dataclasses import dataclass
from typing import Literal

from bw_shared.script_tools.directories import BAGS_DIR, SVO_DIR


@dataclass
class SvoPlaybackCameraConfig:
    type: Literal["SvoPlaybackCamera"] = "SvoPlaybackCamera"
    svo_name: str = "playback"
    bag_name: str = ""
    svo_directory: str = str(SVO_DIR)
    bag_directory: str = str(BAGS_DIR)
    field_grab_time: float = 0.0
    start_time: float = 0.0
    time_sync_warning: float = 0.01  # seconds
    progress_log_interval: float = 1.0  # seconds
