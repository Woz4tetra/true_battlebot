import logging

import numpy as np
import pyzed.sl as sl
from bw_shared.messages.header import Header
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import CloudFieldName, PointCloud
from perception_tools.rosbridge.ros_publisher import RosPublisher
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage

from app.camera.camera_interface import CameraInterface, CameraMode
from app.camera.zed_helpers import set_field_finder_settings, set_robot_finder_settings, zed_to_ros_camera_info
from app.config.camera_config.svo_playback_camera_config import SvoPlaybackCameraConfig
from app.config.camera_topic_config import CameraTopicConfig


class SvoPlaybackCamera(CameraInterface):
    def __init__(
        self,
        config: SvoPlaybackCameraConfig,
        camera_topic_config: CameraTopicConfig,
        color_image_pub: RosPublisher[RosImage],
        camera_info_pub: RosPublisher[CameraInfo],
    ) -> None:
        self.logger = logging.getLogger("perception")

        self.config = config
        self.camera_topic_config = camera_topic_config
        self.color_image_pub = color_image_pub
        self.camera_info_pub = camera_info_pub

        input_type = sl.InputType()
        input_type.set_from_svo_file(self.config.path)  # Set init parameter to run from the .svo

        self.camera = sl.Camera()
        self.init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=False)

        self.runtime_parameters = sl.RuntimeParameters()

        self.mode = CameraMode.ROBOT_FINDER

        self.color_image = sl.Mat()
        self.point_cloud = sl.Mat()
        self.header = Header(0.0, self.camera_topic_config.frame_id, 0)

        self.camera_info = zed_to_ros_camera_info(self.camera.get_camera_information())

        self.field_data = self.load_field_frame()

    def open(self, mode: CameraMode) -> bool:
        self.mode = mode
        return True

    def poll(self) -> CameraData | None:
        status = self.camera.grab(self.runtime_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            self.logger.error(f"ZED Camera failed to grab frame: {status.name} ({status.value}): {str(status)}")
            return None
        if self.mode == CameraMode.FIELD_FINDER:
            self.logger.info("Using cached field frame data")
            header = self.next_header()
            self.field_data.set_header(header)
            return self.field_data

        self.camera.retrieve_image(self.color_image, sl.VIEW.LEFT)
        color_image_data = self.color_image.get_data()[..., 0:3]

        header = self.next_header()
        self.camera_info.header = header.to_msg()
        image = Image(header, color_image_data)
        point_cloud = PointCloud(header, np.array([]), np.array([]), color_encoding=CloudFieldName.BGRA)

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

    def _set_robot_finder_settings(self):
        self.logger.info("Setting ZED Camera to Robot Finder mode")
        set_robot_finder_settings(self.init_params)

    def _set_field_finder_settings(self):
        self.logger.info("Setting ZED Camera to Field Finder mode")
        set_field_finder_settings(self.init_params)

    def next_header(self) -> Header:
        image_time = self.camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
        self.header.stamp = image_time * 1e-9
        self.header.seq += 1
        return self.header

    def load_field_frame(self) -> CameraData:
        self._set_field_finder_settings()
        status = self.camera.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise Exception(f"Error opening camera: {status}")

        self.camera.set_svo_position(self.config.field_grab_index)
        status = self.camera.grab(self.runtime_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            raise Exception(
                f"Error grabbing field frame: {status}. Attempted to get index: {self.config.field_grab_index}"
            )

        self.camera.retrieve_image(self.color_image, sl.VIEW.LEFT)
        color_image_data = self.color_image.get_data()[..., 0:3]

        self.camera.retrieve_measure(self.point_cloud, sl.MEASURE.XYZBGRA)
        raw_cloud_data = self.point_cloud.get_data()
        points = raw_cloud_data[..., 0:3]
        colors = raw_cloud_data[..., 3].view(np.uint32)

        header = self.next_header()
        image = Image(header, color_image_data)
        point_cloud = PointCloud(header, points, colors, color_encoding=CloudFieldName.BGRA)
        camera_data = CameraData(color_image=image, point_cloud=point_cloud, camera_info=self.camera_info)

        self.camera.close()

        self._set_robot_finder_settings()
        status = self.camera.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise Exception(f"Error opening camera: {status}")
        self.camera.set_svo_position(self.config.start_index)

        return camera_data