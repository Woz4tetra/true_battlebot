import logging

import pyzed.sl as sl
from perception_tools.messages.camera.camera_data import CameraData
from perception_tools.messages.camera.camera_info import CameraInfo

from app.camera.camera_interface import CameraInterface
from app.config.camera_config.zed_camera_config import ZedCameraConfig


class ZedCamera(CameraInterface):
    def __init__(self, config: ZedCameraConfig) -> None:
        self.logger = logging.getLogger("perception")
        self.config = config
        self.camera = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.camera_resolution = sl.RESOLUTION.HD1080
        if self.config.serial_number != -1:
            self.init_params.set_from_serial_number(self.config.serial_number, sl.BUS_TYPE.USB)
            self.logger.info(f"ZED Camera serial number: {self.config.serial_number}")
        else:
            self.logger.info("Auto detecting ZED Camera serial number")

        self.runtime_parameters = sl.RuntimeParameters()

        self.camera_data = CameraData()

        self.color_image = sl.Mat()
        self.point_cloud = sl.Mat()

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
        self.camera.retrieve_image(self.color_image, sl.VIEW.LEFT)
        self.camera.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

        self.camera_data.color_image = self.color_image.get_data()
        self.camera_data.point_cloud = self.point_cloud.get_data()[..., :6]

    def close(self) -> None:
        self.camera.close()
