from dataclasses import dataclass, field
from typing import Literal

from app.config.keypoint_config.model_to_system_labels_config import ModelToSystemLabelsConfig


@dataclass
class SemanticSegmentationConfig:
    type: Literal["SemanticSegmentation"] = "SemanticSegmentation"
    model_path: str = "model.torchscript"
    metadata_path: str = "model.json"
    image_delay_threshold: float = 0.2
    debug: bool = False
    model_to_system_labels: ModelToSystemLabelsConfig = field(default_factory=ModelToSystemLabelsConfig)
