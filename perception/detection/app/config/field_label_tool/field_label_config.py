from __future__ import annotations

from dataclasses import dataclass
from typing import TypeVar

from bw_shared.messages.dataclass_utils import from_dict, to_dict

T = TypeVar("T")


@dataclass
class FieldLabelConfig:
    cloud_topic: str = "/camera_0/point_cloud/cloud_registered"
    image_topic: str = "/camera_0/rgb/image_raw"
    info_topic: str = "/camera_0/rgb/camera_info"
    field_request_topic: str = "/field_request"
    max_cloud_distance: float = 1000.0
    label_state_path: str = "/data/cache/label_state.json"
    image_padding: int = 0
    num_extra_points: int = 0
    start_time: float = 0.0

    def to_dict(self):
        return to_dict(self)

    @classmethod
    def from_dict(cls: type[T], data: dict) -> T:
        return from_dict(cls, data)
