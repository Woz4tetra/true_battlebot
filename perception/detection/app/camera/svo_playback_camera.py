import logging
import time
from pathlib import Path
from threading import Lock, Thread

import numpy as np
import pyzed.sl as sl
import rospy
from app.camera.camera_interface import CameraInterface, CameraMode
from app.camera.zed.helpers import zed_status_to_str, zed_to_ros_camera_info, zed_to_ros_imu
from app.config.camera_config.svo_playback_camera_config import SvoPlaybackCameraConfig
from app.config.camera_topic_config import CameraTopicConfig
from bw_interfaces.msg import ControlRecording
from bw_shared.messages.header import Header
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import CloudFieldName, PointCloud
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from rosbag.bag import Bag
from sensor_msgs.msg import CameraInfo, Imu
from sensor_msgs.msg import Image as RosImage


class SvoPlaybackCamera(CameraInterface):
    def __init__(
        self,
        config: SvoPlaybackCameraConfig,
        camera_topic_config: CameraTopicConfig,
        color_image_pub: RosPublisher[RosImage],
        camera_info_pub: RosPublisher[CameraInfo],
        imu_pub: RosPublisher[Imu],
        record_svo_sub: RosPollSubscriber[ControlRecording],
        bag_publishers: dict[str, RosPublisher],
    ) -> None:
        self.logger = logging.getLogger("perception")

        self.config = config
        self.camera_topic_config = camera_topic_config
        self.color_image_pub = color_image_pub
        self.camera_info_pub = camera_info_pub
        self.imu_pub = imu_pub
        self.record_svo_sub = record_svo_sub
        self.bag_publishers = bag_publishers

        self.svo_directory = Path(self.config.svo_directory)
        self.bag_directory = Path(self.config.bag_directory)

        input_type = sl.InputType()
        # Set init parameter to run from the .svo
        svo_name = self.config.svo_name if self.config.svo_name.endswith("svo2") else self.config.svo_name + ".svo2"
        svo_path = self.svo_directory / svo_name
        input_type.set_from_svo_file(str(svo_path))

        self.camera = sl.Camera()
        self.init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=False)
        self.init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
        self.init_params.coordinate_units = sl.UNIT.METER

        self.runtime_parameters = sl.RuntimeParameters()

        self.recording_parameters = sl.RecordingParameters()
        self.recording_parameters.compression_mode = sl.SVO_COMPRESSION_MODE.H264

        self.mode = CameraMode.ROBOT_FINDER
        self.camera_fps = 0.0
        self.real_time_start = 0.0
        self.camera_start_time = 0.0
        self.is_open = False
        self.is_polling = False
        self.did_finish = False
        self.last_log_time = 0.0
        self.num_frames = 0

        self.color_image = sl.Mat()
        self.point_cloud = sl.Mat()
        self.header = Header(0.0, self.camera_topic_config.frame_id, 0)

        bag_name = self.config.bag_name if self.config.bag_name else svo_path.stem + ".bag"
        bag_name = bag_name if bag_name.endswith("bag") else bag_name + ".bag"
        bag_path = self.bag_directory / bag_name
        if bag_path.exists():
            self.bag = Bag(str(), "r")
            self.bag_time = self.bag.get_start_time()
            self.bag_iters = self.bag.read_messages(
                start_time=rospy.Time.from_sec(self.bag_time + self.config.start_time),
                topics=list(self.bag_publishers.keys()),
            )
        else:
            self.logger.warning(f"Bag file not found: {bag_path}. Skipping bag playback.")
            self.bag = None
            self.bag_time = 0.0
            self.bag_iters = iter([])

        self.camera_data: CameraData | None = None
        self.imu_data: Imu | None = None
        self.data_lock = Lock()

        self.poll_thread = Thread(target=self.poll_task, daemon=True)

        self.load_field_frame()
        self.logger.debug("SvoPlaybackCamera initialized")

    def switch_mode(self, mode: CameraMode) -> bool:
        if self.mode != mode:
            self.logger.debug(f"Opening SvoPlaybackCamera in mode: {mode}")
        self.mode = mode
        return True

    def poll_task(self) -> None:
        try:
            while True:
                if self.did_finish:
                    return

                real_time = self.get_relative_real_time()
                camera_time = self.get_relative_camera_time()
                self.logger.debug(f"Capture time: {camera_time}. Real time: {real_time}")
                if not (abs(real_time - camera_time) < self.config.time_sync_warning):
                    if real_time > camera_time:
                        self.logger.warning(f"Capture time is behind real time: {real_time - camera_time:0.6f} s")
                    else:
                        time.sleep(camera_time - real_time)
                        continue

                t0 = time.perf_counter()
                status = self.camera.grab(self.runtime_parameters)
                t1 = time.perf_counter()
                if status == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                    self.logger.info("End of SVO file reached")
                    self.did_finish = True
                    break
                if status != sl.ERROR_CODE.SUCCESS:
                    self.logger.error(f"ZED Camera failed to grab frame: {zed_status_to_str(status)}")
                    break

                self.camera.retrieve_image(self.color_image, sl.VIEW.LEFT)
                color_image_data = self.color_image.get_data()[..., 0:3]

                header = self.next_header()
                now = time.time()
                self.logger.debug(
                    f"Frame {header.seq} / {self.num_frames} "
                    f"time: {header.stamp}. "
                    f"Delay: {now - header.stamp}. "
                    f"Image grab time: {t1 - t0}"
                )
                if now - self.last_log_time > self.config.progress_log_interval:
                    self.last_log_time = now
                    self.logger.info(f"Frame {header.seq} / {self.num_frames} time: {header.stamp}")

                self.field_data.camera_info.header = header.to_msg()
                image = Image(header, color_image_data)
                point_cloud = PointCloud(header, np.array([]), np.array([]), color_encoding=CloudFieldName.BGRA)

                with self.data_lock:
                    self.camera_data = CameraData(
                        color_image=image,
                        point_cloud=point_cloud,
                        camera_info=self.field_data.camera_info,
                    )

                sensors_data = sl.SensorsData()
                imu_status = self.camera.get_sensors_data(sensors_data, sl.TIME_REFERENCE.IMAGE)
                if imu_status == sl.ERROR_CODE.SUCCESS:
                    imu_zed_data = sensors_data.get_imu_data()
                    with self.data_lock:
                        self.imu_data = zed_to_ros_imu(self.field_data.camera_info.header, imu_zed_data)
                else:
                    self.logger.warning(f"Failed to get IMU data: {zed_status_to_str(imu_status)}")
        except BaseException as e:
            self.logger.error(f"Error in poll task: {e}", exc_info=True)
            raise
        finally:
            self.logger.info("Poll task finished")

    def poll(self) -> CameraData | None:
        if self.did_finish:
            return None

        if self.mode == CameraMode.FIELD_FINDER:
            self.logger.info("Using cached field frame data")
            header = self.next_header()
            self.field_data.set_header(header)
            return self.field_data

        camera_data: CameraData | None = None
        self._poll_recording()
        with self.data_lock:
            if self.imu_data:
                self.imu_pub.publish(self.imu_data)
                self.imu_data = None
            if self.camera_data:
                camera_data = self.camera_data
                self.camera_data = None
                self.color_image_pub.publish(camera_data.color_image.to_msg())
                self.camera_info_pub.publish(self.field_data.camera_info)

        if self.bag and camera_data:
            while self.bag_time < camera_data.color_image.header.stamp:
                topic, msg, timestamp = next(self.bag_iters)  # type: ignore
                self.bag_publishers[topic].publish(msg)
                self.logger.debug(f"Published message from bag: {topic}")
                self.bag_time = timestamp.to_sec()

        return camera_data

    def close(self) -> None:
        self.camera.close()
        if self.bag:
            self.bag.close()

    def _set_robot_finder_settings(self) -> bool:
        self.logger.info("Setting ZED Camera to Robot Finder mode")
        self.runtime_parameters.enable_depth = False
        return True

    def _set_field_finder_settings(self) -> bool:
        self.logger.info("Setting ZED Camera to Field Finder mode")
        self.runtime_parameters.enable_depth = True
        return True

    def get_svo_time(self) -> float:
        image_time = self.camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
        return image_time * 1e-9

    def next_header(self) -> Header:
        self.header.stamp = self.get_svo_time()
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

    def _open_camera(self) -> bool:
        status = self.camera.open(self.init_params)
        success = status == sl.ERROR_CODE.SUCCESS
        if not success:
            self.logger.error(f"Failed to open camera: {zed_status_to_str(status)}")
        self.is_open = success
        return success

    def open(self) -> bool:
        status = True
        if not self.is_open:
            status = self._open_camera()
        if not self.is_polling:
            self.poll_thread.start()
            self.is_polling = True
        return status

    def grab(self) -> None:
        status = self.camera.grab(self.runtime_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            raise Exception(f"Error grabbing initial frame: {zed_status_to_str(status)}")

    def get_number_of_frames(self) -> int:
        return self.camera.get_svo_number_of_frames()

    def grab_frame_at(self, frame_num: int) -> None:
        self.set_svo_position(frame_num)
        status = self.camera.grab(self.runtime_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            raise Exception(f"Error grabbing last frame: {zed_status_to_str(status)}")

    def cache_field_frame(self, zed_info: sl.CameraInformation) -> None:
        status = self.camera.grab(self.runtime_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            raise Exception(
                f"Error grabbing field frame: {zed_status_to_str(status)}. "
                f"Attempted to get: {self.config.field_grab_time}"
            )
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
        self._open_camera()
        self._set_field_finder_settings()
        self.grab()

        self.camera_start_time = self.get_svo_time()
        self.num_frames = self.get_number_of_frames()
        self.grab_frame_at(self.num_frames - 2)
        camera_stop_time = self.get_svo_time()
        self.camera_fps = self.num_frames / (camera_stop_time - self.camera_start_time)
        if self.camera_fps < 0.0:
            raise Exception(f"Invalid camera FPS: {self.camera_fps}")
        if self.camera_fps < 1.0:
            self.logger.warning(f"Low camera FPS: {self.camera_fps}")

        self.logger.info(f"Camera FPS: {self.camera_fps}")
        self.logger.info(f"SVO has {self.num_frames} frames")

        zed_info = self.camera.get_camera_information()

        self.set_svo_time(self.config.field_grab_time)
        self.cache_field_frame(zed_info)

        self._set_robot_finder_settings()
        self.logger.debug("Setting camera to Robot Finder mode")
        self.grab()

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
