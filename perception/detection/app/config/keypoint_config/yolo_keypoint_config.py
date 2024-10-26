from dataclasses import dataclass, field
from typing import Literal

from bw_shared.configs.model_to_system_labels_map import ModelToSystemLabelsMap


@dataclass
class YoloKeypointConfig:
    type: Literal["YoloKeypoint"] = "YoloKeypoint"
    model_path: str = "model.pt"
    metadata_path: str = ""
    threshold: float = 0.85
    iou_threshold: float = 0.5
    debug_timing: bool = False
    debug_image: bool = False
    model_to_system_labels: ModelToSystemLabelsMap = field(default_factory=ModelToSystemLabelsMap)
