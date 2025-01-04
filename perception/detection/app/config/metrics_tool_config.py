from __future__ import annotations

from dataclasses import dataclass, field
from typing import TypeVar

from bw_shared.messages.dataclass_utils import from_dict, to_dict

T = TypeVar("T")


@dataclass
class CameraPositionConfig:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class CameraRotationConfig:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class CameraPoseConfig:
    position: CameraPositionConfig = field(default_factory=CameraPositionConfig)
    rotation: CameraRotationConfig = field(default_factory=CameraRotationConfig)
    intrinsics: str = ""


@dataclass
class MetricsToolConfig:
    camera: CameraPoseConfig = field(default_factory=CameraPoseConfig)

    def to_dict(self):
        return to_dict(self)

    @classmethod
    def from_dict(cls: type[T], data: dict) -> T:
        return from_dict(cls, data)
