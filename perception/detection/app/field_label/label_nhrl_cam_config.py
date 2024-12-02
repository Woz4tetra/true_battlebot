from dataclasses import dataclass

from app.field_label.field_label_config import FieldLabelConfig


@dataclass
class LabelNhrlCamConfig(FieldLabelConfig):
    image_topic: str = "/camera_1/image_rect"
    info_topic: str = "/camera_1/camera_info"
