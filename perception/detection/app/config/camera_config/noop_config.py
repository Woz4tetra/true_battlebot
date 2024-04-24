from dataclasses import dataclass


@dataclass
class NoopCameraConfig:
    type: str = "NoopCamera"
