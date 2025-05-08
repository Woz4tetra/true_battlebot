import logging
import time

import numpy as np
import pyzed.sl as sl
from app.camera.zed.zed_helpers import zed_status_to_str, zed_to_ros_camera_info, zed_to_ros_imu
from bw_shared.messages.header import Header
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import CloudFieldName, PointCloud
from sensor_msgs.msg import CameraInfo, Imu


class SvoCamera:
    def __init__(self, svo_path: str, frame_id: str) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)

        input_type = sl.InputType()
        # Set init parameter to run from the .svo
        input_type.set_from_svo_file(svo_path)

        self.camera = sl.Camera()
        self.init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=False)
        self.init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
        self.init_params.coordinate_units = sl.UNIT.METER

        self.runtime_parameters = sl.RuntimeParameters()

        self.recording_parameters = sl.RecordingParameters()
        self.recording_parameters.compression_mode = sl.SVO_COMPRESSION_MODE.H264

        self.camera_fps = 0.0
        self.camera_start_time = 0.0
        self.is_open = False
        self.num_frames = 0

        self.color_image = sl.Mat()
        self.point_cloud = sl.Mat()
        self.camera_info = CameraInfo()
        self.frame_id = frame_id
        self.header = Header(0.0, self.frame_id, 0)

    def get_rgb_data(self) -> tuple[CameraData | None, sl.ERROR_CODE]:
        t0 = time.perf_counter()
        status = self.camera.grab(self.runtime_parameters)
        t1 = time.perf_counter()
        self.logger.debug(f"Image grab time: {t1 - t0}")
        if status != sl.ERROR_CODE.SUCCESS:
            return None, status
        self.camera.retrieve_image(self.color_image, sl.VIEW.LEFT)
        color_image_data = self.color_image.get_data()[..., 0:3]
        header = self.next_header()
        image = Image(header, color_image_data)
        point_cloud = PointCloud(header, np.array([]), np.array([]), color_encoding=CloudFieldName.BGRA)
        self.camera_info.header = header.to_msg()

        return CameraData(color_image=image, point_cloud=point_cloud, camera_info=self.camera_info), status

    def get_camera_data(self) -> tuple[CameraData | None, sl.ERROR_CODE]:
        t0 = time.perf_counter()
        status = self.camera.grab(self.runtime_parameters)
        t1 = time.perf_counter()
        self.logger.debug(f"Image grab time: {t1 - t0}")
        if status != sl.ERROR_CODE.SUCCESS:
            return None, status
        self.camera.retrieve_image(self.color_image, sl.VIEW.LEFT)
        color_image_data = self.color_image.get_data()[..., 0:3]
        header = self.next_header()
        image = Image(header, color_image_data)

        self.camera.retrieve_measure(self.point_cloud, sl.MEASURE.XYZBGRA)
        raw_cloud_data = self.point_cloud.get_data()
        points = raw_cloud_data[..., 0:3]
        colors = raw_cloud_data[..., 3].view(np.uint32)
        point_cloud = PointCloud(header, points, colors, color_encoding=CloudFieldName.BGRA)

        self.camera_info.header = header.to_msg()

        return CameraData(color_image=image, point_cloud=point_cloud, camera_info=self.camera_info), status

    def get_imu_data(self) -> tuple[Imu | None, sl.ERROR_CODE]:
        sensors_data = sl.SensorsData()
        imu_status = self.camera.get_sensors_data(sensors_data, sl.TIME_REFERENCE.IMAGE)
        if imu_status == sl.ERROR_CODE.SUCCESS:
            imu_zed_data = sensors_data.get_imu_data()
            return zed_to_ros_imu(self.camera_info.header, imu_zed_data), imu_status
        return None, imu_status

    def close(self) -> None:
        self.camera.close()
        self.is_open = False

    def set_enable_depth(self, enable_depth: bool) -> None:
        self.runtime_parameters.enable_depth = enable_depth

    def get_svo_time(self) -> float:
        image_time = self.camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
        return float(image_time) * 1e-9

    def next_header(self) -> Header:
        self.header = Header(self.get_svo_time(), self.frame_id, self.header.seq + 1)
        return self.header

    def open(self) -> sl.ERROR_CODE:
        status = self.camera.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            return status
        self.is_open = True
        self._load_timing_info()
        return status

    def get_number_of_frames(self) -> int:
        return int(self.camera.get_svo_number_of_frames())

    def _grab(self) -> None:
        status = self.camera.grab(self.runtime_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            raise Exception(f"Error grabbing initial frame: {zed_status_to_str(status)}")

    def _grab_frame_at(self, frame_num: int) -> None:
        self.set_svo_position(frame_num)
        status = self.camera.grab(self.runtime_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            raise Exception(f"Error grabbing last frame: {zed_status_to_str(status)}")

    def set_svo_time(self, time_since_start: float) -> None:
        self.logger.debug(f"Jumping to time: {time_since_start}")
        position = int(time_since_start * self.camera_fps)
        self.set_svo_position(position)

    def set_svo_position(self, position: int) -> None:
        self.logger.debug(f"Setting SVO position to: {position}")
        self.camera.set_svo_position(position)

    def get_relative_time(self) -> float:
        return self.get_svo_time() - self.camera_start_time

    def start_recording(self, svo_path: str) -> None:
        self.recording_parameters.video_filename = svo_path
        self.camera.enable_recording(self.recording_parameters)

    def stop_recording(self) -> None:
        self.camera.disable_recording()

    def _load_timing_info(self) -> None:
        self._grab()
        self.camera_start_time = self.get_svo_time()
        self.num_frames = self.get_number_of_frames()
        self._grab_frame_at(self.num_frames - 2)
        camera_stop_time = self.get_svo_time()
        self.camera_fps = self.num_frames / (camera_stop_time - self.camera_start_time)
        if self.camera_fps < 0.0:
            raise Exception(f"Invalid camera FPS: {self.camera_fps}")
        if self.camera_fps < 1.0:
            self.logger.warning(f"Low camera FPS: {self.camera_fps}")

        self.logger.debug(f"Camera FPS: {self.camera_fps}")
        self.logger.debug(f"SVO has {self.num_frames} frames")

        zed_info = self.camera.get_camera_information()
        self.camera_info = zed_to_ros_camera_info(zed_info)
