import logging
import time
from pathlib import Path
from threading import Lock, Thread

import pyzed.sl as sl
import rospy
from app.camera.camera_interface import CameraInterface, CameraMode
from app.camera.zed.helpers import zed_status_to_str
from app.camera.zed.svo_camera import SvoCamera
from app.config.camera.svo_playback_camera_config import SvoPlaybackCameraConfig
from app.config.camera_topic_config import CameraTopicConfig
from bw_interfaces.msg import ControlRecording
from perception_tools.messages.camera_data import CameraData
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from rosbag.bag import Bag
from rosgraph_msgs.msg import Clock
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
        clock_pub: RosPublisher[Clock],
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)

        self.config = config
        self.camera_topic_config = camera_topic_config
        self.svo_directory = Path(self.config.svo_directory)
        self.bag_directory = Path(self.config.bag_directory)

        svo_name = self.config.svo_name if self.config.svo_name.endswith("svo2") else self.config.svo_name + ".svo2"
        svo_path = self.svo_directory / svo_name
        self.camera = SvoCamera(str(svo_path), camera_topic_config.frame_id)

        self.color_image_pub = color_image_pub
        self.camera_info_pub = camera_info_pub
        self.imu_pub = imu_pub
        self.record_svo_sub = record_svo_sub
        self.bag_publishers = bag_publishers
        self.clock_pub = clock_pub

        self.mode = CameraMode.ROBOT_FINDER
        self.real_time_start = 0.0
        self.is_polling = False
        self.did_finish = False
        self.last_log_time = 0.0

        bag_name = self.config.bag_name if self.config.bag_name else svo_path.stem + ".bag"
        bag_name = bag_name if bag_name.endswith("bag") else bag_name + ".bag"
        bag_path = self.bag_directory / bag_name
        if bag_path.exists():
            self.bag = Bag(str(bag_path), "r")
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
        self.clock_thread = Thread(target=self.clock_task, daemon=True)

        self.load_field_frame()
        self.logger.debug("SvoPlaybackCamera initialized")

    def switch_mode(self, mode: CameraMode) -> bool:
        if self.mode != mode:
            self.logger.debug(f"Opening SvoPlaybackCamera in mode: {mode}")
        self.mode = mode
        return True

    def poll_task(self) -> None:
        try:
            while self.task_tick():
                pass
        except BaseException as e:
            self.logger.error(f"Error in poll task: {e}", exc_info=True)
            raise
        finally:
            self.logger.info("Poll task finished")

    def task_tick(self) -> bool:
        if self.did_finish:
            return False

        real_time = self.get_relative_real_time()
        camera_time = self.camera.get_relative_time()
        self.logger.debug(f"Capture time: {camera_time}. Real time: {real_time}")
        if not (abs(real_time - camera_time) < self.config.time_sync_warning):
            if real_time > camera_time:
                self.logger.warning(f"Capture time is behind real time: {real_time - camera_time:0.6f} s")
            else:
                time.sleep(camera_time - real_time)
                return True

        t0 = time.perf_counter()
        camera_data, status = self.camera.get_rgb_data()
        t1 = time.perf_counter()
        if status == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            self.logger.info("End of SVO file reached")
            self.did_finish = True
            return False
        if status != sl.ERROR_CODE.SUCCESS:
            self.logger.error(f"ZED Camera failed to grab frame: {zed_status_to_str(status)}")
            return True
        if camera_data is None:
            raise Exception("Camera data is not set with a successful status")

        header = camera_data.camera_info.header
        now = time.time()
        self.logger.debug(
            f"Frame {header.seq} / {self.camera.num_frames} "
            f"time: {header.stamp}. "
            f"Delay: {now - header.stamp.to_sec()}. "
            f"Image grab time: {t1 - t0}"
        )
        if now - self.last_log_time > self.config.progress_log_interval:
            self.last_log_time = now
            self.logger.info(f"Frame {header.seq} / {self.camera.num_frames} time: {header.stamp}")

        with self.data_lock:
            self.camera_data = camera_data

        imu_data, imu_status = self.camera.get_imu_data()
        if imu_status != sl.ERROR_CODE.SUCCESS:
            self.logger.warning(f"Failed to get IMU data: {zed_status_to_str(imu_status)}")
        if imu_data is None:
            raise Exception("IMU data is not set with a successful status")

        with self.data_lock:
            self.imu_data = imu_data

        return True

    def clock_task(self) -> None:
        bag_start_time = self.bag.get_start_time() if self.bag else time.time()
        start_time = time.perf_counter()
        publish_rate = 0.01
        while True:
            if self.did_finish:
                return
            now = time.perf_counter()
            real_time = now - start_time
            bag_time = bag_start_time + real_time
            clock = Clock(clock=rospy.Time.from_sec(bag_time))
            self.clock_pub.publish(clock)
            sleep_time = publish_rate - (now - start_time) % publish_rate
            time.sleep(sleep_time)

    def poll(self) -> CameraData | None:
        if self.did_finish:
            return None

        if self.mode == CameraMode.FIELD_FINDER:
            self.logger.info("Using cached field frame data")
            header = self.camera.next_header()
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
        self.camera.set_enable_depth(False)
        return True

    def _set_field_finder_settings(self) -> bool:
        self.logger.info("Setting ZED Camera to Field Finder mode")
        self.camera.set_enable_depth(True)
        return True

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
                self.camera.start_recording(str(file_path))
            case ControlRecording.STOP:
                self.logger.info("Stopping ZED Camera recording")
                self.camera.stop_recording()
            case _:
                self.logger.error(f"Invalid ZED Camera recording command: {command}")

    def _open_camera(self) -> bool:
        status = self.camera.open()
        success = status == sl.ERROR_CODE.SUCCESS
        if not success:
            self.logger.error(f"Failed to open camera: {zed_status_to_str(status)}")
        return success

    def open(self) -> bool:
        status = True
        if not self.camera.is_open:
            status = self._open_camera()
        if not self.is_polling:
            self.poll_thread.start()
            self.clock_thread.start()
            self.is_polling = True
        return status

    def cache_field_frame(self) -> None:
        camera_data, status = self.camera.get_camera_data()
        if status != sl.ERROR_CODE.SUCCESS or camera_data is None:
            raise Exception(
                f"Error grabbing field frame: {zed_status_to_str(status)}. "
                f"Attempted to get: {self.config.field_grab_time}"
            )
        self.logger.debug("Grabbed field frame")

        now = time.time()
        self.field_data = camera_data
        timestamp = self.field_data.camera_info.header.stamp.to_sec()
        self.logger.debug(f"Field frame time: {timestamp}. Delay: {now - timestamp}")

    def load_field_frame(self) -> None:
        self.logger.debug("Loading field frame data")
        self._open_camera()
        self._set_field_finder_settings()

        self.camera.set_svo_time(self.config.field_grab_time)
        self.cache_field_frame()

        self._set_robot_finder_settings()
        self.logger.debug("Setting camera to Robot Finder mode")

        self.camera.set_svo_time(self.config.start_time)
        self.real_time_start = time.perf_counter() - self.config.start_time

    def get_relative_real_time(self) -> float:
        return time.perf_counter() - self.real_time_start
