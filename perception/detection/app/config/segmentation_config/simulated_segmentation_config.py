from dataclasses import dataclass, field
from typing import Literal

from app.config.keypoint_config.model_to_system_labels_config import ModelToSystemLabelsConfig


@dataclass
class SimulatedSegmentationConfig:
    type: Literal["SimulatedSegmentation"] = "SimulatedSegmentation"
    debug: bool = True
    compression_error_tolerance: int = -1
    model_to_system_labels: ModelToSystemLabelsConfig = field(default_factory=ModelToSystemLabelsConfig)
    apply_noise: bool = False
    random_sample_interval: float = 0.0  # seconds. 0.0 == noise generators always apply
