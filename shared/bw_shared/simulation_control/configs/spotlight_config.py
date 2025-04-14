from dataclasses import dataclass


@dataclass
class SpotlightConfig:
    enabled: bool = True
    range: float = 3.0
    spot_angle: float = 116.6
    intensity: float = 6.0
    shadow_strength: float = 0.744
