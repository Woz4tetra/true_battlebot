from dataclasses import dataclass


@dataclass
class InstanceSegmentationConfig:
    type: str = "InstanceSegmentation"
    model_path = "model.torchscript"
    metadata_path = "model_metadata.json"
    threshold = 0.8
    nms_threshold = 0.4
    mask_conversion_threshold = 0.5
    decimate = 1.0
    image_delay_threshold = 0.2
    debug = False
