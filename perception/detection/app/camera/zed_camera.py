import logging
from pathlib import Path

import numpy as np
import pyzed.sl as sl
import rospy
from app.camera.camera_interface import CameraInterface, CameraMode
from app.camera.zed.helpers import zed_status_to_str, zed_to_ros_camera_info, zed_to_ros_imu
from app.camera.zed.video_settings import Zed2iVideoSettings, ZedParameterError
from app.config.camera.zed_camera_config import ZedCameraConfig
from app.config.camera_topic_config import CameraTopicConfig
from bw_interfaces.msg import ControlRecording
from bw_shared.messages.header import Header
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import CloudFieldName, PointCloud
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu
from sensor_msgs.msg import Image as RosImage


class ZedCamera(CameraInterface):
    def __init__(
        self,
        config: ZedCameraConfig,
        camera_topic_config: CameraTopicConfig,
        color_image_pub: RosPublisher[RosImage] | None,
        camera_info_pub: RosPublisher[CameraInfo] | None,
        compressed_image_pub: RosPublisher[CompressedImage] | None,
        imu_pub: RosPublisher[Imu],
        record_svo_sub: RosPollSubscriber[ControlRecording],
    ) -> None:
        self.logger = logging.getLogger("perception")

        self.config = config
        self.camera_topic_config = camera_topic_config
        self.color_image_pub = color_image_pub
        self.camera_info_pub = camera_info_pub
        self.compressed_image_pub = compressed_image_pub
        self.imu_pub = imu_pub
        self.record_svo_sub = record_svo_sub

        self.camera = sl.Camera()

        self.init_params = sl.InitParameters()
        self.init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
        self.init_params.camera_resolution = self.config.resolution.to_zed()
        self.init_params.camera_fps = self.config.fps
        self.init_params.coordinate_units = sl.UNIT.METER

        if self.config.serial_number != -1:
            self.init_params.set_from_serial_number(self.config.serial_number, sl.BUS_TYPE.USB)
            self.logger.info(f"ZED Camera serial number: {self.config.serial_number}")
        else:
            self.logger.info("Auto detecting ZED Camera serial number")

        self.runtime_parameters = sl.RuntimeParameters()

        self.recording_parameters = sl.RecordingParameters()
        self.recording_parameters.compression_mode = sl.SVO_COMPRESSION_MODE.H264

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

    def _set_robot_finder_settings(self) -> bool:
        self.logger.info("Setting ZED Camera to Robot Finder mode")
        self.runtime_parameters.enable_depth = False
        return True

    def _set_field_finder_settings(self) -> bool:
        self.logger.info("Setting ZED Camera to Field Finder mode")
        self.runtime_parameters.enable_depth = True
        return True

    def open(self) -> bool:
        status = None
        if self.is_open:
            return True
        while not rospy.is_shutdown():
            status = self.camera.open(self.init_params)
            if status == sl.ERROR_CODE.SUCCESS:
                self.logger.info("ZED Camera opened successfully")
                break
            self.logger.error(f"ZED Camera failed to open: {zed_status_to_str(status)}")
            self.logger.error("Retrying...")
            self.camera.close()

        try:
            self.config.video_settings.apply_to_camera(self.camera)
        except ZedParameterError:
            self.logger.error("Failed to apply camera settings")
            self.close()
            return False

        self.logger.info(f"ZED camera settings: {str(Zed2iVideoSettings.from_camera(self.camera))}")

        self.logger.info("ZED Camera opened successfully")

        self.camera_info = self.load_camera_info()
        self.is_open = True
        return True

    def switch_mode(self, mode: CameraMode) -> bool:
        if mode == self.mode:
            return True
        self.mode = mode
        return self.mode_callbacks[mode]()

    def load_camera_info(self) -> CameraInfo:
        camera_information = self.camera.get_camera_information()
        return zed_to_ros_camera_info(camera_information)

    def next_header(self) -> Header:
        image_time = self.camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
        self.header.stamp = image_time * 1e-9
        self.header.seq += 1
        return self.header

    def _poll_recording(self) -> None:
        if not (control_msg := self.record_svo_sub.receive()):
            return
        command = control_msg.command
        match command:
            case ControlRecording.START:
                self.logger.info("Starting ZED Camera recording")
                file_path = Path(control_msg.name)
                file_path = file_path.with_suffix(".svo2")
                file_path = self.config.svo_directory / file_path
                self.logger.info(f"Recording to {file_path}")
                self.recording_parameters.video_filename = str(file_path)
                self.camera.enable_recording(self.recording_parameters)
            case ControlRecording.STOP:
                self.logger.info("Stopping ZED Camera recording")
                self.camera.disable_recording()
            case _:
                self.logger.error(f"Invalid ZED Camera recording command: {command}")

    def poll(self) -> CameraData | None:
        self._poll_recording()
        status = self.camera.grab(self.runtime_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            self.logger.error(f"ZED Camera failed to grab frame: {zed_status_to_str(status)}")
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

        sensors_data = sl.SensorsData()
        imu_status = self.camera.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)
        if imu_status == sl.ERROR_CODE.SUCCESS:
            imu_data = sensors_data.get_imu_data()
            self.imu_pub.publish(zed_to_ros_imu(self.camera_info.header, imu_data))
        else:
            self.logger.warning(f"Failed to get IMU data: {zed_status_to_str(imu_status)}")
        if self.color_image_pub:
            self.color_image_pub.publish(image.to_msg())
        if self.camera_info_pub:
            self.camera_info_pub.publish(self.camera_info)
        if self.compressed_image_pub:
            self.compressed_image_pub.publish(image.to_compressed_msg())

        return camera_data

    def close(self) -> None:
        self.logger.info("Closing ZED Camera")
        self.camera.close()
        self.is_open = False
