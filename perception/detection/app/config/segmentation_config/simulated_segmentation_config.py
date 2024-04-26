from dataclasses import dataclass
from typing import Literal


@dataclass
class SimulatedSegmentationConfig:
    type: Literal["SimulatedSegmentation"] = "SimulatedSegmentation"
    separate_friendlies: bool = True
    debug: bool = True
    namespace: str = "camera_0"
    compression_error_tolerance: int = 15
