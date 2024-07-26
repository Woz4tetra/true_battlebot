from dataclasses import dataclass, field
from typing import Literal

from app.config.keypoint_config.model_to_system_labels_config import ModelToSystemLabelsConfig


@dataclass
class YoloKeypointConfig:
    type: Literal["YoloKeypoint"] = "YoloKeypoint"
    model_path: str = "model.pt"
    metadata_path: str = "model_metadata.json"
    threshold: float = 0.85
    debug: bool = False
    model_to_system_labels: ModelToSystemLabelsConfig = field(default_factory=ModelToSystemLabelsConfig)
