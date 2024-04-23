from dataclasses import dataclass

from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.image import Image


@dataclass
class CameraData:
    color_image: Image
    depth_image: Image
    camera_info: CameraInfo
