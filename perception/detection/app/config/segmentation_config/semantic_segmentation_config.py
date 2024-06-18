from dataclasses import dataclass
from typing import Literal


@dataclass
class SemanticSegmentationConfig:
    type: Literal["SemanticSegmentation"] = "SemanticSegmentation"
    model_path: str = "model.torchscript"
    metadata_path: str = "model.json"
    image_delay_threshold: float = 0.2
    debug: bool = False
