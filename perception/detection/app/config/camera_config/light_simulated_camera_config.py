from dataclasses import dataclass
from typing import Literal


@dataclass
class LightSimulatedCameraConfig:
    type: Literal["LightSimulatedCamera"] = "LightSimulatedCamera"
