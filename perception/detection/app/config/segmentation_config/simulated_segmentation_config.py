from dataclasses import dataclass
from typing import Literal


@dataclass
class SimulatedSegmentationConfig:
    type: Literal["SimulatedSegmentation"] = "SimulatedSegmentation"
    separate_friendlies: bool = True
    debug: bool = False
    namespace: str = "camera_0"
