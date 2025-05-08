from dataclasses import dataclass, field
from typing import Literal

from app.camera.zed.zed_video_settings import Zed2iVideoSettings
from perception_tools.camera.zed_resolutions import Zed2iResolutions


@dataclass
class ZedCameraConfig:
    type: Literal["ZedCamera"] = "ZedCamera"
    serial_number: int = -1
    video_settings: Zed2iVideoSettings = field(default_factory=Zed2iVideoSettings)
    svo_directory: str = "/data/svo"
    resolution: Zed2iResolutions = Zed2iResolutions.MODE_HD1080
    fps: int = 30
    enable_positional_tracking: bool = True
