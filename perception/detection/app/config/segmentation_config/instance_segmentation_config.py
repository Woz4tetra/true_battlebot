from dataclasses import dataclass
from typing import Literal


@dataclass
class InstanceSegmentationConfig:
    type: Literal["InstanceSegmentation"] = "InstanceSegmentation"
    model_path: str = "model.torchscript"
    metadata_path: str = "model_metadata.json"
    threshold: float = 0.8
    nms_threshold: float = 0.4
    mask_conversion_threshold: float = 0.5
    decimate: float = 1.0
    image_delay_threshold: float = 0.2
    debug: bool = False
