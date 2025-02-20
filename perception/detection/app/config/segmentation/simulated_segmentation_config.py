from dataclasses import dataclass, field
from typing import Literal

from bw_shared.configs.model_to_system_labels_map import ModelToSystemLabelsMap


@dataclass
class SimulatedSegmentationConfig:
    type: Literal["SimulatedSegmentation"] = "SimulatedSegmentation"
    debug: bool = True
    compression_error_tolerance: int = -1
    model_to_system_labels: ModelToSystemLabelsMap = field(default_factory=ModelToSystemLabelsMap)
    apply_noise: bool = False
    random_sample_interval: float = 0.0  # seconds. 0.0 == noise generators always apply
