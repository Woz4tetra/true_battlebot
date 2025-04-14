from dataclasses import dataclass


@dataclass
class EvenLightingConfig:
    enabled: bool = False
    intensity: float = 1.0
    size: float = 10.0
