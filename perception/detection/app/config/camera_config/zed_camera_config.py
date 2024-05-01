from dataclasses import dataclass
from typing import Literal


@dataclass
class ZedCameraConfig:
    type: Literal["ZedCamera"] = "ZedCamera"
    serial_number: int = -1
