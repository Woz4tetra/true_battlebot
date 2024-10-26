from dataclasses import dataclass, field
from typing import Literal

from bw_shared.configs.model_to_system_labels_map import ModelToSystemLabelsMap


@dataclass
class SemanticSegmentationConfig:
    type: Literal["SemanticSegmentation"] = "SemanticSegmentation"
    model_path: str = "model.torchscript"
    metadata_path: str = ""
    image_delay_threshold: float = 0.2
    debug: bool = False
    model_to_system_labels: ModelToSystemLabelsMap = field(default_factory=ModelToSystemLabelsMap)
