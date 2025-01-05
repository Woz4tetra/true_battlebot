from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field as dataclass_field
from typing import TypeVar

from app.config.metrics_tool.camera_pose_config import CameraPoseConfig
from app.config.metrics_tool.field_config import FieldConfig
from app.config.metrics_tool.tracker.sam2_tracker_config import Sam2TrackerConfig
from app.config.metrics_tool.tracker.tracker_types import TrackerConfig
from app.config.metrics_tool.video_filter_config import VideoFilterConfig
from bw_shared.messages.dataclass_utils import from_dict, to_dict

T = TypeVar("T")


@dataclass
class MetricsToolConfig:
    cache_dir: str = "/data/cache/metrics_tool"
    camera: CameraPoseConfig = dataclass_field(default_factory=CameraPoseConfig)
    video_filter: VideoFilterConfig = dataclass_field(default_factory=VideoFilterConfig)
    field: FieldConfig = dataclass_field(default_factory=FieldConfig)
    tracker: TrackerConfig = dataclass_field(default_factory=Sam2TrackerConfig)

    def to_dict(self):
        return to_dict(self)

    @classmethod
    def from_dict(cls: type[T], data: dict) -> T:
        return from_dict(cls, data)
