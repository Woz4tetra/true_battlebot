#!/usr/bin/env python
import time

import cv2
import depthai as dai
import numpy as np
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from bw_interfaces.msg import EstimatedObjectArray
from bw_tools.configs.rosparam_client import get_shared_config
from bw_tools.get_param import get_param
from bw_tools.tag_detection.apriltag_detector import ApriltagDetector, ApriltagDetectorConfig
from bw_tools.tag_detection.bundle_detector import BundleDetectorInterface, RansacBundleDetector
from bw_tools.tag_detection.draw_helpers import draw_bundle
from bw_tools.tag_detection.image_rectifier import ImageRectifier
from bw_tools.tag_detection.tag_family import TagFamily
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from bw_tracking_cam.bundle_result_helpers import bundle_result_to_apriltag_ros, bundle_result_to_object
from bw_tracking_cam.oak_1_resolution_mode import Oak1ResolutionMode
from bw_tracking_cam.oak_time_helpers import get_frame_time, ros_time_from_nsec


class DepthAiOak1W:
    def __init__(self):
        shared_config = get_shared_config()
        self.bundle_configs = {robot_config.name: robot_config.tags for robot_config in shared_config.robots.robots}
        rospy.loginfo(f"Bundle Configs: {self.bundle_configs}")

        self.camera_name = get_param("~camera_name", "camera")
        self.width = get_param("~width", 1920)
        self.height = get_param("~height", 1080)
        self.fps = get_param("~fps", 60.0)
        self.exposure_time = get_param("~exposure_time", 8000)  # microseconds
        self.iso = get_param("~iso", 800)
        self.resolution_mode = Oak1ResolutionMode("mode_" + get_param("~resolution_mode", "1080_p"))
        self.debug_image = get_param("~debug_image", False)
        self.camera_matrix_alpha = get_param("~camera_matrix_alpha", 0.0)

        detector_config = ApriltagDetectorConfig(tag_family=TagFamily.TAG36H11)
        self.tag_detector = ApriltagDetector(detector_config)

        self.ros_base_time = rospy.Time(0)
        self.steady_base_time = time.perf_counter_ns()
        self.frame_num = 0

        self.bridge = CvBridge()

        self.camera_info_pub = rospy.Publisher(f"{self.camera_name}/camera_info", CameraInfo, queue_size=1)
        self.camera_image_pub = rospy.Publisher(f"{self.camera_name}/image_rect", Image, queue_size=1)
        self.debug_image_pub = rospy.Publisher(f"{self.camera_name}/debug_image", Image, queue_size=1)
        self.robot_tag_pub = rospy.Publisher(f"{self.camera_name}/robot_tags", EstimatedObjectArray, queue_size=1)
        self.april_tag_pub = rospy.Publisher(f"{self.camera_name}/tag_detections", AprilTagDetectionArray, queue_size=1)

    def update_base_time(self, steady_base_time_ns: int) -> None:
        current_ros_time = rospy.Time.now()
        current_steady_time_ns = time.perf_counter_ns()

        # Calculate expected offset in nanoseconds
        expected_offset = current_steady_time_ns - steady_base_time_ns

        # Update ros_base_time
        new_base_time_ns = current_ros_time.to_nsec() - expected_offset
        self.ros_base_time = ros_time_from_nsec(new_base_time_ns)

    def make_ransac_params(self) -> cv2.UsacParams:
        microsac_params = cv2.UsacParams()
        microsac_params.threshold = 5.0
        microsac_params.confidence = 0.99999
        microsac_params.score = cv2.SCORE_METHOD_MSAC
        microsac_params.maxIterations = 10_000
        microsac_params.loIterations = 100
        microsac_params.loMethod = cv2.LOCAL_OPTIM_GC

        microsac_params.final_polisher = cv2.LSQ_POLISHER
        microsac_params.final_polisher_iterations = 10_000

        return microsac_params

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

    def make_rectifier(self, camera_info: CameraInfo) -> ImageRectifier:
        return ImageRectifier(camera_info, self.width, self.height, alpha=self.camera_matrix_alpha)

    def make_bundle_detectors(self, rectified_info: CameraInfo) -> dict[str, BundleDetectorInterface]:
        bundle_detectors = {}
        params = self.make_ransac_params()
        for name, bundle_config in self.bundle_configs.items():
            detector = RansacBundleDetector(bundle_config, params, rectified_info)
            bundle_detectors[name] = detector
        return bundle_detectors

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

    def tick(
        self,
        video: dai.DataOutputQueue,
        rectifier: ImageRectifier,
        tag_detector: ApriltagDetector,
        bundle_detectors: dict[str, BundleDetectorInterface],
        rectified_info: CameraInfo,
    ) -> None:
        video_in = video.get()
        frame: np.ndarray = video_in.getCvFrame()  # type: ignore

        frame_time_nsec = int(video_in.getTimestamp().total_seconds() * 1e9)  # type: ignore

        timestamp = get_frame_time(self.ros_base_time, self.steady_base_time, frame_time_nsec)
        header = Header(stamp=timestamp, frame_id=self.camera_name, seq=self.frame_num)
        rectified_info.header = header

        rectified = rectifier.rectify(frame)
        gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
        results = {}
        draw_image = rectified if self.debug_image else None
        for name, bundle_detector in bundle_detectors.items():
            detections = tag_detector.detect(gray)
            result = bundle_detector.detect(detections)
            draw_image = draw_bundle(rectified_info, draw_image, result) if (draw_image is not None) else None
            results[name] = result

        obj_msg = bundle_result_to_object(header, results)
        self.robot_tag_pub.publish(obj_msg)

        apriltag_msg = bundle_result_to_apriltag_ros(header, results.values())
        self.april_tag_pub.publish(apriltag_msg)

        self.camera_info_pub.publish(rectified_info)
        try:
            self.camera_image_pub.publish(self.bridge.cv2_to_imgmsg(rectified, "bgr8", header=header))
        except CvBridgeError as e:
            rospy.logerr(e)
        if draw_image is not None:
            try:
                self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(draw_image, "bgr8", header=header))
            except CvBridgeError as e:
                rospy.logerr(e)

        self.frame_num += 1

    def run(self):
        cam_key = dai.CameraBoardSocket.CAM_A

        pipeline = self.init_pipeline(cam_key)

        # Connect to device and start pipeline
        with dai.Device(pipeline) as device:
            # Camera control
            control_queue = device.getInputQueue("control")
            ctrl = dai.CameraControl()
            ctrl.setManualExposure(self.exposure_time, self.iso)
            control_queue.send(ctrl)

            video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

            calib_data = device.readCalibration()
            camera_info = self.make_camera_info(calib_data, cam_key)

            if self.width is None:
                self.width = camera_info.width
            if self.height is None:
                self.height = camera_info.height

            rectifier = self.make_rectifier(camera_info)

            rectified_info = rectifier.get_rectified_info()
            bundle_detectors = self.make_bundle_detectors(rectified_info)

            self.update_base_time(self.steady_base_time)

            while not rospy.is_shutdown():
                self.tick(video, rectifier, self.tag_detector, bundle_detectors, rectified_info)


if __name__ == "__main__":
    rospy.init_node("depthai_oak_1_w", log_level=rospy.DEBUG)
    DepthAiOak1W().run()
