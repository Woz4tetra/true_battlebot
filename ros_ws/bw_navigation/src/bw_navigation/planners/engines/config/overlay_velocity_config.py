import math
from dataclasses import dataclass


@dataclass
class OverlayVelocityConfig:
    friendly_mirror_proximity: float = 1.0  # meters
    friendly_mirror_magnify: float = 1.5
    angle_threshold: float = 40.0  # degrees

    def __post_init__(self) -> None:
        self.angle_threshold_rad = math.radians(self.angle_threshold)
