from dataclasses import dataclass


@dataclass
class InstanceSegmentationConfig:
    type: str = "InstanceSegmentation"
    model_path: str = "model.torchscript"
    metadata_path: str = "model_metadata.json"
    threshold: float = 0.8
    nms_threshold: float = 0.4
    mask_conversion_threshold: float = 0.5
    decimate: float = 1.0
    image_delay_threshold: float = 0.2
    debug: bool = False
