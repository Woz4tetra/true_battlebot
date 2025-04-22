import logging

from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo


class CameraModelLoader:
    def __init__(self) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.model: PinholeCameraModel | None = None

    def update_model(self, camera_info: CameraInfo) -> bool:
        self.camera_info.header.stamp = camera_info.header.stamp
        self.camera_info.header.seq = camera_info.header.seq
        if self.camera_info.header != camera_info.header:
            self.model = PinholeCameraModel()
            self.model.fromCameraInfo(camera_info)
            self.logger.info(f"Camera model loaded: {camera_info}")
            self.camera_info = camera_info
            return True
        return False

    def get_model(self) -> PinholeCameraModel | None:
        return self.model
