import logging

import pyzed.sl as sl
from perception_tools.messages.camera.camera_data import CameraData
from perception_tools.messages.camera.camera_info import CameraInfo

from app.camera.camera_interface import CameraInterface
from app.config.camera_config.zed_camera_config import ZedCameraConfig


class ZedCamera(CameraInterface):
    def __init__(self, config: ZedCameraConfig) -> None:
        self.logger = logging.getLogger("perception")
        self.camera = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
        self.init_params.coordinate_units = sl.UNIT.METER

        self.runtime_parameters = sl.RuntimeParameters()

        camera_info = CameraInfo()
        self.camera_data = CameraData(camera_info=camera_info)

    def open(self) -> bool:
        status = None
        while True:
            status = self.camera.open(self.init_params)
            if status == sl.ERROR_CODE.SUCCESS:
                self.logger.info("ZED Camera opened successfully")
                break
            self.logger.error("ZED Camera failed to open. Retrying...")
        return True

    def poll(self) -> CameraData | None:
        status = self.camera.grab(self.runtime_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            self.logger.error(f"ZED Camera failed to grab frame: {status.name} ({status.value}): {str(status)}")
            return None
        image = sl.Mat()
        point_cloud = sl.Mat()
        self.camera.retrieve_image(image, sl.VIEW.LEFT)
        self.camera.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

    def close(self) -> None:
        self.camera.close()
