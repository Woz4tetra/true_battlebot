import logging

from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo


class CameraModelLoader:
    def __init__(self) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.camera_info: CameraInfo | None = None
        self.model: PinholeCameraModel | None = None

    def update_model(self, camera_info: CameraInfo) -> None:
        if self.camera_info is not None and self.camera_info.header.frame_id == camera_info.header.frame_id:
            return
        self.model = PinholeCameraModel()
        self.model.fromCameraInfo(camera_info)
        self.logger.info(f"Camera model loaded: {camera_info}")
        self.camera_info = camera_info

    def get_model(self) -> PinholeCameraModel | None:
        return self.model
