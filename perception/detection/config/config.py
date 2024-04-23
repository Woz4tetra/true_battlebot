from __future__ import annotations

from dataclasses import asdict, dataclass, field

from config.camera_config.camera_types import CameraConfig
from config.camera_config.noop_config import NoopCameraConfig
from config.field_filter_config import FieldFilterConfig
from config.rosbridge_config import RosBridgeConfig
from config.segmentation_config.segmentation_types import InstanceSegmentationConfig, SegmentationConfig
from dacite import from_dict


@dataclass
class Config:
    poll_rate: float = 1000.0
    camera: CameraConfig = field(default_factory=NoopCameraConfig)
    rosbridge: RosBridgeConfig = field(default_factory=RosBridgeConfig)
    field_segmentation: SegmentationConfig = field(default_factory=InstanceSegmentationConfig)
    field_filter: FieldFilterConfig = field(default_factory=FieldFilterConfig)

    @classmethod
    def from_dict(cls, data: dict) -> Config:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)
