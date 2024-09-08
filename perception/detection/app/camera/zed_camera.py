import logging

import numpy as np
import pyzed.sl as sl
import rospy
from app.camera.camera_interface import CameraInterface, CameraMode
from app.camera.zed.helpers import (
    set_field_finder_settings,
    set_robot_finder_settings,
    zed_to_ros_camera_info,
)
from app.config.camera_config.zed_camera_config import ZedCameraConfig
from app.config.camera_topic_config import CameraTopicConfig
from bw_shared.messages.header import Header
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import CloudFieldName, PointCloud
from perception_tools.rosbridge.ros_publisher import RosPublisher
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage


class ZedCamera(CameraInterface):
    def __init__(
        self,
        config: ZedCameraConfig,
        camera_topic_config: CameraTopicConfig,
        color_image_pub: RosPublisher[RosImage],
        camera_info_pub: RosPublisher[CameraInfo],
    ) -> None:
        self.logger = logging.getLogger("perception")

        self.config = config
        self.camera_topic_config = camera_topic_config
        self.color_image_pub = color_image_pub
        self.camera_info_pub = camera_info_pub

        self.camera = sl.Camera()
        self.init_params = sl.InitParameters()
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

        self.is_open = False
        self.mode = CameraMode.ROBOT_FINDER
        self.mode_callbacks = {
            CameraMode.ROBOT_FINDER: self._set_robot_finder_settings,
            CameraMode.FIELD_FINDER: self._set_field_finder_settings,
        }

    def _set_robot_finder_settings(self):
        self.logger.info("Setting ZED Camera to Robot Finder mode")
        set_robot_finder_settings(self.init_params)

    def _set_field_finder_settings(self):
        self.logger.info("Setting ZED Camera to Field Finder mode")
        set_field_finder_settings(self.init_params)

    def open(self, mode: CameraMode) -> bool:
        status = None
        if self.is_open:
            if mode == self.mode:
                return True
            else:
                self.close()
        self.mode = mode
        self.mode_callbacks[mode]()
        while not rospy.is_shutdown():
            status = self.camera.open(self.init_params)
            if status == sl.ERROR_CODE.SUCCESS:
                self.logger.info("ZED Camera opened successfully")
                break
            self.logger.error("ZED Camera failed to open. Retrying...")
            self.camera.close()

        self.logger.info("ZED Camera opened successfully")

        # settings = self.camera.get_camera_settings()
        # self.logger.info(f"Camera settings: {settings}")

        self.camera_info = self.load_camera_info()
        self.is_open = True
        return True

    def load_camera_info(self) -> CameraInfo:
        camera_information = self.camera.get_camera_information()
        return zed_to_ros_camera_info(camera_information)

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
        color_image_data = self.color_image.get_data()[..., 0:3]

        if self.mode == CameraMode.FIELD_FINDER:
            self.logger.info("Retrieving point cloud")
            self.camera.retrieve_measure(self.point_cloud, sl.MEASURE.XYZBGRA)
            raw_cloud_data = self.point_cloud.get_data()
            points = raw_cloud_data[..., 0:3]
            colors = raw_cloud_data[..., 3].view(np.uint32)
        else:
            points = np.array([])
            colors = np.array([])

        header = self.next_header()
        self.camera_info.header = header.to_msg()
        image = Image(header, color_image_data)
        point_cloud = PointCloud(header, points, colors, color_encoding=CloudFieldName.BGRA)
        camera_data = CameraData(
            color_image=image,
            point_cloud=point_cloud,
            camera_info=self.camera_info,
        )

        self.color_image_pub.publish(image.to_msg())
        self.camera_info_pub.publish(self.camera_info)

        return camera_data

    def close(self) -> None:
        self.camera.close()
        self.is_open = False
