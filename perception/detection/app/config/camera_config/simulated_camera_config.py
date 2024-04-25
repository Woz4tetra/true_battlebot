from dataclasses import dataclass
from typing import Literal


@dataclass
class SimulatedCameraConfig:
    type: Literal["SimulatedCamera"] = "SimulatedCamera"
    namespace: str = "camera_0"
