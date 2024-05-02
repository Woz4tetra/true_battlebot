import logging

import numpy as np
import pyzed.sl as sl
from bw_shared.messages.header import Header
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import PointCloud
from sensor_msgs.msg import CameraInfo

from app.camera.camera_interface import CameraInterface
from app.config.camera_config.zed_camera_config import ZedCameraConfig
from app.config.camera_topic_config import CameraTopicConfig


class ZedCamera(CameraInterface):
    def __init__(self, config: ZedCameraConfig, camera_topic_config: CameraTopicConfig) -> None:
        self.logger = logging.getLogger("perception")
        self.config = config
        self.camera_topic_config = camera_topic_config
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

        self.header = Header(0.0, self.camera_topic_config.frame_id, 0)
        self.camera_info = CameraInfo()

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

        self.camera_info = self.load_camera_info()
        return True

    def load_camera_info(self) -> CameraInfo:
        camera_information = self.camera.get_camera_information()
        intrinsics = camera_information.camera_configuration.calibration_parameters.left_cam
        resolution = intrinsics.image_size

        raw_intrinsics = [intrinsics.fx, 0, intrinsics.cx, 0, intrinsics.fy, intrinsics.cy, 0, 0, 1]

        return CameraInfo(
            height=resolution.height,
            width=resolution.width,
            distortion_model="plumb_bob",
            D=intrinsics.disto,
            K=raw_intrinsics,
        )

    def next_header(self) -> Header:
        image_time = self.camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
        self.header.stamp = image_time * 1e-9
        self.header.seq += 1
        return self.header

    def poll(self) -> CameraData | None:
        status = self.camera.grab(self.runtime_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            self.logger.error(f"ZED Camera failed to grab frame: {status.name} ({status.value}): {str(status)}")
            return None
        self.camera.retrieve_image(self.color_image, sl.VIEW.LEFT)
        self.camera.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

        color_image_data = self.color_image.get_data()

        raw_cloud_data = self.point_cloud.get_data()
        points = raw_cloud_data[..., 0:3]
        colors = raw_cloud_data[..., 3].view(np.uint32)

        header = self.next_header()
        self.camera_info.header = header.to_msg()
        camera_data = CameraData(
            color_image=Image(header, color_image_data),
            point_cloud=PointCloud(header, points, colors),
            camera_info=self.camera_info,
        )

        return camera_data

    def close(self) -> None:
        self.camera.close()
