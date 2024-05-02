from dataclasses import dataclass, field
from typing import Literal


@dataclass
class SimulatedSegmentationConfig:
    type: Literal["SimulatedSegmentation"] = "SimulatedSegmentation"
    debug: bool = True
    compression_error_tolerance: int = -1
    simulated_to_real_labels: dict[str, str] = field(default_factory=dict)
    apply_noise: bool = False
    random_sample_interval: float = 0.0  # seconds. 0.0 == noise generators always apply
