from dataclasses import dataclass, field
from typing import Literal


@dataclass
class SimulatedSegmentationConfig:
    type: Literal["SimulatedSegmentation"] = "SimulatedSegmentation"
    debug: bool = True
    compression_error_tolerance: int = 15
    simulated_to_real_labels: dict[str, str] = field(default_factory=dict)
