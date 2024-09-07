import logging
import time

import numpy as np
import pyzed.sl as sl
from app.camera.camera_interface import CameraInterface, CameraMode
from app.camera.zed_helpers import set_field_finder_settings, set_robot_finder_settings, zed_to_ros_camera_info
from app.config.camera_config.svo_playback_camera_config import SvoPlaybackCameraConfig
from app.config.camera_topic_config import CameraTopicConfig
from bw_shared.messages.header import Header
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import CloudFieldName, PointCloud
from perception_tools.rosbridge.ros_publisher import RosPublisher
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage


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
        self.camera_fps = 0.0
        self.real_time_start = 0.0
        self.camera_start_time = 0.0

        self.color_image = sl.Mat()
        self.point_cloud = sl.Mat()
        self.header = Header(0.0, self.camera_topic_config.frame_id, 0)

        self.load_field_frame()
        self.logger.debug("SvoPlaybackCamera initialized")

    def open(self, mode: CameraMode) -> bool:
        if self.mode != mode:
            self.logger.debug(f"Opening SvoPlaybackCamera in mode: {mode}")
        self.mode = mode
        return True

    def poll(self) -> CameraData | None:
        real_time = self.get_relative_real_time()
        camera_time = self.get_relative_camera_time()
        self.logger.debug(f"Capture time: {camera_time}. Real time: {real_time}")
        if not (abs(real_time - camera_time) < self.config.time_sync_threshold):
            if real_time > camera_time:
                self.logger.warning(f"Capture time is behind real time: {camera_time} < {real_time}")
                self.set_svo_time(real_time)
            else:
                self.logger.warning(f"Real time is behind capture time: {real_time} < {camera_time}")
                return None

        t0 = time.perf_counter()
        status = self.camera.grab(self.runtime_parameters)
        t1 = time.perf_counter()
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
        now = time.time()
        self.logger.debug(f"Frame time: {header.stamp}. Delay: {now - header.stamp}. Image grab time: {t1 - t0}")
        self.field_data.camera_info.header = header.to_msg()
        image = Image(header, color_image_data)
        point_cloud = PointCloud(header, np.array([]), np.array([]), color_encoding=CloudFieldName.BGRA)

        camera_data = CameraData(
            color_image=image,
            point_cloud=point_cloud,
            camera_info=self.field_data.camera_info,
        )

        self.color_image_pub.publish(image.to_msg())
        self.camera_info_pub.publish(self.field_data.camera_info)

        return camera_data

    def close(self) -> None:
        self.camera.close()

    def _set_robot_finder_settings(self):
        self.logger.info("Setting ZED Camera to Robot Finder mode")
        set_robot_finder_settings(self.init_params)

    def _set_field_finder_settings(self):
        self.logger.info("Setting ZED Camera to Field Finder mode")
        set_field_finder_settings(self.init_params)

    def get_svo_time(self) -> float:
        image_time = self.camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
        return image_time * 1e-9

    def next_header(self) -> Header:
        self.header.stamp = self.get_relative_camera_time()
        self.header.seq += 1
        return self.header

    def open_and_grab(self) -> None:
        status = self.camera.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise Exception(f"Error opening camera: {status}")

        status = self.camera.grab(self.runtime_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            raise Exception(f"Error grabbing initial frame: {status}")

    def get_number_of_frames(self) -> int:
        return self.camera.get_svo_number_of_frames()

    def grab_frame_at(self, frame_num: int) -> None:
        self.set_svo_position(frame_num)
        status = self.camera.grab(self.runtime_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            raise Exception(f"Error grabbing last frame: {status}")

    def cache_field_frame(self, zed_info: sl.CameraInformation) -> None:
        status = self.camera.grab(self.runtime_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            raise Exception(f"Error grabbing field frame: {status}. Attempted to get: {self.config.field_grab_time}")
        self.logger.debug("Grabbed field frame")

        self.camera.retrieve_image(self.color_image, sl.VIEW.LEFT)
        color_image_data = self.color_image.get_data()[..., 0:3]

        self.camera.retrieve_measure(self.point_cloud, sl.MEASURE.XYZBGRA)
        raw_cloud_data = self.point_cloud.get_data()
        points = raw_cloud_data[..., 0:3]
        colors = raw_cloud_data[..., 3].view(np.uint32)

        header = self.next_header()
        now = time.time()
        self.logger.debug(f"Field frame time: {header.stamp}. Delay: {now - header.stamp}")
        image = Image(header, color_image_data)
        point_cloud = PointCloud(header, points, colors, color_encoding=CloudFieldName.BGRA)
        camera_info = zed_to_ros_camera_info(zed_info)
        self.field_data = CameraData(color_image=image, point_cloud=point_cloud, camera_info=camera_info)

    def load_field_frame(self) -> None:
        self.logger.debug("Loading field frame data")
        self._set_field_finder_settings()
        self.open_and_grab()

        self.camera_start_time = self.get_svo_time()
        num_frames = self.get_number_of_frames()
        self.grab_frame_at(num_frames - 1)
        camera_stop_time = self.get_svo_time()
        self.camera_fps = num_frames / (camera_stop_time - self.camera_start_time)
        if self.camera_fps < 0.0:
            raise Exception(f"Invalid camera FPS: {self.camera_fps}")
        if self.camera_fps < 1.0:
            self.logger.warning(f"Low camera FPS: {self.camera_fps}")

        self.logger.info(f"Camera FPS: {self.camera_fps}")
        self.logger.info(f"SVO has {num_frames} frames")

        zed_info = self.camera.get_camera_information()

        self.set_svo_time(self.config.field_grab_time)
        self.cache_field_frame(zed_info)
        self.logger.debug("Closing camera")
        self.camera.close()

        self._set_robot_finder_settings()
        self.logger.debug("Reopening camera in Robot Finder mode")
        self.open_and_grab()

        self.set_svo_time(self.config.start_time)

        self.real_time_start = time.perf_counter() - self.config.start_time

    def set_svo_time(self, time_since_start: float) -> None:
        self.logger.debug(f"Jumping to time: {time_since_start}")
        position = int(time_since_start * self.camera_fps)
        self.set_svo_position(position)

    def set_svo_position(self, position: int) -> None:
        self.logger.debug(f"Setting SVO position to: {position}")
        self.camera.set_svo_position(position)

    def get_relative_real_time(self) -> float:
        return time.perf_counter() - self.real_time_start

    def get_relative_camera_time(self) -> float:
        return self.get_svo_time() - self.camera_start_time
