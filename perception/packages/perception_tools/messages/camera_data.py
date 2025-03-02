from dataclasses import dataclass, field

from sensor_msgs.msg import CameraInfo, Imu

from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import PointCloud


@dataclass
class CameraData:
    color_image: Image = field(default_factory=Image)
    point_cloud: PointCloud = field(default_factory=PointCloud)
    camera_info: CameraInfo = field(default_factory=CameraInfo)
    imu: Imu = field(default_factory=Imu)

    def set_header(self, header):
        self.color_image.header = header
        self.point_cloud.header = header
        self.camera_info.header = header
        self.imu.header = header
