from dataclasses import dataclass, field

from bw_shared.geometry.rpy import RPY


@dataclass
class CameraRotationConfig:
    roll: float = 0.0  # degrees
    pitch: float = 0.0  # degrees
    yaw: float = 0.0  # degrees

    def to_rpy(self) -> RPY:
        return RPY.from_degrees((self.roll, self.pitch, self.yaw))


@dataclass
class CameraPositionConfig:
    x: float = 0.0  # meters
    y: float = 0.0  # meters
    z: float = 0.0  # meters

    def to_tuple(self) -> tuple[float, float, float]:
        return self.x, self.y, self.z


@dataclass
class CameraPoseConfig:
    position: CameraPositionConfig = field(default_factory=CameraPositionConfig)
    rotation: CameraRotationConfig = field(default_factory=CameraRotationConfig)
    intrinsics: str = ""
