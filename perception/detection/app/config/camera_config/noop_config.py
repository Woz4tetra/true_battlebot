from dataclasses import dataclass
from typing import Literal


@dataclass
class NoopCameraConfig:
    type: Literal["NoopCamera"] = "NoopCamera"
