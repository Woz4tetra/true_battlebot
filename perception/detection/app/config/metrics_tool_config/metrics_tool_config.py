from __future__ import annotations

from dataclasses import dataclass, field
from typing import TypeVar

from app.config.metrics_tool_config.camera_pose_config import CameraPoseConfig
from app.config.metrics_tool_config.field_config import FieldConfig
from app.config.metrics_tool_config.video_filter_config import VideoFilterConfig
from bw_shared.messages.dataclass_utils import from_dict, to_dict

T = TypeVar("T")


@dataclass
class MetricsToolConfig:
    cache_dir: str = "/data/cache/metrics_tool"
    camera: CameraPoseConfig = field(default_factory=CameraPoseConfig)
    video_filter: VideoFilterConfig = field(default_factory=VideoFilterConfig)
    field: FieldConfig = field(default_factory=FieldConfig)

    def to_dict(self):
        return to_dict(self)

    @classmethod
    def from_dict(cls: type[T], data: dict) -> T:
        return from_dict(cls, data)
