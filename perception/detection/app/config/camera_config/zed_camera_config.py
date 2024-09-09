from dataclasses import dataclass, field
from typing import Literal

from app.camera.zed.video_settings import Zed2iVideoSettings


@dataclass
class ZedCameraConfig:
    type: Literal["ZedCamera"] = "ZedCamera"
    serial_number: int = -1
    video_settings: Zed2iVideoSettings = field(default_factory=Zed2iVideoSettings)
