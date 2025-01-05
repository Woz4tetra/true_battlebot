from dataclasses import dataclass


@dataclass
class VideoFilterConfig:
    min_motion_area: int = 10000
