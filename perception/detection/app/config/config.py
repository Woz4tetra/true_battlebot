from __future__ import annotations

from dataclasses import asdict, dataclass, field

import dacite

from app.config.camera_config.camera_types import CameraConfig
from app.config.camera_config.noop_camera_config import NoopCameraConfig
from app.config.camera_topic_config import CameraTopicConfig
from app.config.field_filter_config.field_filter_types import FieldFilterConfig
from app.config.field_filter_config.field_request_config import FieldRequestConfig
from app.config.field_filter_config.ransac_field_filter_config import RansacFieldFilterConfig
from app.config.rosbridge_config import RosBridgeConfig
from app.config.segmentation_config.segmentation_types import InstanceSegmentationConfig, SegmentationConfig


@dataclass
class Config:
    poll_rate: float = 1000.0
    camera_topic: CameraTopicConfig = field(default_factory=lambda: CameraTopicConfig("/camera_0", "camera_0"))
    camera: CameraConfig = field(default_factory=NoopCameraConfig)
    rosbridge: RosBridgeConfig = field(default_factory=RosBridgeConfig)
    field_segmentation: SegmentationConfig = field(default_factory=InstanceSegmentationConfig)
    robot_segmentation: SegmentationConfig = field(default_factory=InstanceSegmentationConfig)
    field_filter: FieldFilterConfig = field(default_factory=RansacFieldFilterConfig)
    field_request: FieldRequestConfig = field(default_factory=FieldRequestConfig)

    @classmethod
    def from_dict(cls, data: dict) -> Config:
        return dacite.from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)