from __future__ import annotations

from dataclasses import asdict, dataclass

from config.camera_config.camera_types import CameraConfig
from dacite import from_dict


@dataclass
class Config:
    camera: CameraConfig

    @classmethod
    def from_dict(cls, data: dict) -> Config:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)
