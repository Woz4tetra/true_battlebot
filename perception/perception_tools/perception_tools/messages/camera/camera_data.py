from dataclasses import dataclass, field

from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.image import Image


@dataclass
class CameraData:
    color_image: Image = field(default_factory=Image)
    depth_image: Image = field(default_factory=Image)
    camera_info: CameraInfo = field(default_factory=CameraInfo)
