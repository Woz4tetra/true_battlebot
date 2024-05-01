from dataclasses import dataclass, field

from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.image import Image
from perception_tools.messages.camera.point_cloud import PointCloud


@dataclass
class CameraData:
    color_image: Image = field(default_factory=Image)
    point_cloud: PointCloud = field(default_factory=PointCloud)
    camera_info: CameraInfo = field(default_factory=CameraInfo)
