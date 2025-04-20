from dataclasses import dataclass, field

from bw_shared.enums.frame_id import FrameId
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.transform3d_stamped import Transform3DStamped
from sensor_msgs.msg import CameraInfo, Imu

from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import PointCloud


@dataclass
class CameraData:
    color_image: Image = field(default_factory=Image)
    point_cloud: PointCloud = field(default_factory=PointCloud)
    camera_info: CameraInfo = field(default_factory=CameraInfo)
    imu: Imu = field(default_factory=Imu)
    tf_camera_from_world: Transform3D | None = None
    world_frame: FrameId = FrameId.WORLD_CAMERA_0

    def set_header(self, header):
        self.color_image.header = header
        self.point_cloud.header = header
        self.camera_info.header = header
        self.imu.header = header

    @property
    def tfstamped_camera_from_world(self) -> Transform3DStamped | None:
        if self.tf_camera_from_world is None:
            return None
        return Transform3DStamped(
            header=self.color_image.header,
            child_frame_id=self.world_frame,
            transform=self.tf_camera_from_world,
        )
