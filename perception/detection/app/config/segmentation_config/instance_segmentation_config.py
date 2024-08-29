from dataclasses import dataclass, field
from typing import Literal

from bw_shared.configs.model_to_system_labels_map import ModelToSystemLabelsMap


@dataclass
class InstanceSegmentationConfig:
    type: Literal["InstanceSegmentation"] = "InstanceSegmentation"
    model_path: str = "model.torchscript"
    metadata_path: str = "model.json"
    threshold: float = 0.8
    nms_threshold: float = 0.4
    mask_conversion_threshold: float = 0.5
    decimate: float = 1.0
    image_delay_threshold: float = 0.2
    debug: bool = False
    model_to_system_labels: ModelToSystemLabelsMap = field(default_factory=ModelToSystemLabelsMap)
