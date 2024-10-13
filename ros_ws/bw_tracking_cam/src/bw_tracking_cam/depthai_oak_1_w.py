#!/usr/bin/env python
from __future__ import annotations

import time
from typing import Any, Optional

import depthai as dai
import numpy as np
import rospy
from bw_tools.get_param import get_param
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

from bw_tracking_cam.oak_1_resolution_mode import Oak1ResolutionMode
from bw_tracking_cam.oak_time_helpers import get_frame_time, ros_time_from_nsec
from bw_tracking_cam.tracking_camera_node import ImageSupplier, TrackingCameraNode, config_from_ros_param


class OakImageSupplier(ImageSupplier):
    def __init__(self, camera_name: str, fps: float, exposure_time: int, iso: int, resolution_mode: Oak1ResolutionMode):
        self.camera_name = camera_name
        self.fps = fps
        self.resolution_mode = resolution_mode
        self.exposure_time = exposure_time
        self.iso = iso

        self.camera_info = CameraInfo()
        self.ros_base_time = rospy.Time(0)
        self.steady_base_time = time.perf_counter_ns()
        self.frame_num = 0

    def update_base_time(self, steady_base_time_ns: int) -> None:
        current_ros_time = rospy.Time.now()
        current_steady_time_ns = time.perf_counter_ns()

        # Calculate expected offset in nanoseconds
        expected_offset = current_steady_time_ns - steady_base_time_ns

        # Update ros_base_time
        new_base_time_ns = current_ros_time.to_nsec() - expected_offset
        self.ros_base_time = ros_time_from_nsec(new_base_time_ns)

    def init_pipeline(self, cam_key: dai.CameraBoardSocket) -> dai.Pipeline:
        # Create pipeline
        pipeline = dai.Pipeline()

        # Define source and output
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        xout_video = pipeline.create(dai.node.XLinkOut)

        xout_video.setStreamName("video")

        # Properties
        cam_rgb.setBoardSocket(cam_key)
        cam_rgb.setResolution(self.resolution_mode.to_dai())
        cam_rgb.setFps(self.fps)

        xout_video.input.setBlocking(False)
        xout_video.input.setQueueSize(1)

        # Linking
        cam_rgb.video.link(xout_video.input)

        control_in = pipeline.create(dai.node.XLinkIn)
        control_in.setStreamName("control")
        control_in.out.link(cam_rgb.inputControl)

        return pipeline

    def make_camera_info(self, calib_data: dai.CalibrationHandler, cam_key: dai.CameraBoardSocket) -> CameraInfo:
        instrinsics, info_width, info_height = calib_data.getDefaultIntrinsics(cam_key)

        camera_info = CameraInfo()
        camera_info.header.frame_id = self.camera_name
        camera_info.width = info_width
        camera_info.height = info_height
        instrinsics_array = np.array(instrinsics)
        projection = np.zeros((3, 4))
        camera_info.K = instrinsics_array.reshape(9).tolist()
        camera_info.D = np.array(calib_data.getDistortionCoefficients(cam_key)).tolist()
        projection[:3, :3] = instrinsics_array[:3, :3]
        camera_info.P = projection.reshape(12).tolist()

        return camera_info

    def __enter__(self) -> OakImageSupplier:
        cam_key = dai.CameraBoardSocket.CAM_A

        pipeline = self.init_pipeline(cam_key)

        # Connect to device and start pipeline
        self.device = dai.Device(pipeline)
        # Camera control
        control_queue = self.device.getInputQueue("control")
        ctrl = dai.CameraControl()
        ctrl.setManualExposure(self.exposure_time, self.iso)
        control_queue.send(ctrl)

        self.video = self.device.getOutputQueue(name="video", maxSize=1, blocking=False)

        calib_data = self.device.readCalibration()
        self.camera_info = self.make_camera_info(calib_data, cam_key)

        self.update_base_time(self.steady_base_time)

        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        del self.video
        del self.device

    def get_image(self) -> tuple[Header, Optional[np.ndarray]]:
        video_in = self.video.get()
        frame: np.ndarray = video_in.getCvFrame()  # type: ignore

        frame_time_nsec = int(video_in.getTimestamp().total_seconds() * 1e9)  # type: ignore

        timestamp = get_frame_time(self.ros_base_time, self.steady_base_time, frame_time_nsec)
        header = Header(stamp=timestamp, frame_id=self.camera_name, seq=self.frame_num)

        self.frame_num += 1
        return header, frame

    def get_info(self) -> CameraInfo:
        return self.camera_info


class DepthAiOak1W:
    def __init__(self):
        self.fps = get_param("~fps", 60.0)
        self.exposure_time = get_param("~exposure_time", 8000)  # microseconds
        self.iso = get_param("~iso", 800)
        self.resolution_mode = Oak1ResolutionMode("mode_" + get_param("~resolution_mode", "1080_p"))
        self.config = config_from_ros_param()

    def run(self):
        with OakImageSupplier(
            self.config.camera_name, self.fps, self.exposure_time, self.iso, self.resolution_mode
        ) as video:
            node = TrackingCameraNode(self.config, video)

            while not rospy.is_shutdown():
                node.tick()


if __name__ == "__main__":
    rospy.init_node("depthai_oak_1_w", log_level=rospy.DEBUG)
    DepthAiOak1W().run()
