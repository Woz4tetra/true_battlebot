#!/usr/bin/env python
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from bw_interfaces.msg import EstimatedObjectArray
from bw_shared.geometry.camera.image_rectifier import ImageRectifier
from bw_tools.configs.rosparam_client import get_shared_config
from bw_tools.get_param import get_param
from bw_tools.tag_detection.apriltag_detector import ApriltagDetector, ApriltagDetectorConfig
from bw_tools.tag_detection.bundle_detector import BundleDetectorInterface, RansacBundleDetector
from bw_tools.tag_detection.draw_helpers import draw_bundle
from bw_tools.tag_detection.tag_family import TagFamily
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Header

from bw_tracking_cam.bundle_result_helpers import bundle_result_to_apriltag_ros, bundle_result_to_object


class ImageSupplier(ABC):
    @abstractmethod
    def get_image(self) -> tuple[Header, Optional[np.ndarray]]: ...

    @abstractmethod
    def get_info(self) -> CameraInfo: ...


@dataclass
class TrackingCameraConfig:
    camera_name: str
    width: Optional[int]
    height: Optional[int]
    compressed_fps: float
    debug_image: bool
    camera_matrix_alpha: float
    detect_params_path: str
    refine_params_path: str
    publish_camera: bool


def config_from_ros_param() -> TrackingCameraConfig:
    return TrackingCameraConfig(
        camera_name=get_param("~camera_name", "camera"),
        width=get_param("~width", 1920),
        height=get_param("~height", 1080),
        compressed_fps=get_param("~compressed_fps", 10.0),
        debug_image=get_param("~debug_image", False),
        camera_matrix_alpha=get_param("~camera_matrix_alpha", 0.0),
        detect_params_path=get_param("~detect_params_path", ""),
        refine_params_path=get_param("~refine_params_path", ""),
        publish_camera=True,
    )


class TrackingCameraNode:
    def __init__(self, config: TrackingCameraConfig, image_supplier: ImageSupplier) -> None:
        shared_config = get_shared_config()
        self.bundle_configs = {robot_config.name: robot_config.tags for robot_config in shared_config.robots.robots}
        rospy.loginfo(f"Bundle Configs: {self.bundle_configs}")

        self.camera_info = image_supplier.get_info()
        self.config = config

        self.camera_name = self.config.camera_name
        self.width = self.camera_info.width if self.config.width is None else self.config.width
        self.height = self.camera_info.height if self.config.height is None else self.config.height
        self.compressed_fps = self.config.compressed_fps
        self.debug_image = self.config.debug_image
        self.camera_matrix_alpha = self.config.camera_matrix_alpha
        self.detect_params_path = self.config.detect_params_path
        self.refine_params_path = self.config.refine_params_path

        detector_config = ApriltagDetectorConfig(tag_family=TagFamily.TAG36H11)
        self.tag_detector = ApriltagDetector(detector_config)

        self.prev_compressed_pub_time = rospy.Time(0)
        self.compressed_pub_interval = rospy.Duration.from_sec(1.0 / self.compressed_fps)

        self.bridge = CvBridge()
        self.image_supplier = image_supplier

        self.rectifier = self.make_rectifier(self.camera_info)

        self.rectified_info = self.rectifier.get_rectified_info()
        self.bundle_detectors = self.make_bundle_detectors(self.rectified_info)

        if self.config.publish_camera:
            self.camera_info_pub = rospy.Publisher(f"{self.camera_name}/camera_info", CameraInfo, queue_size=1)
            self.camera_image_pub = rospy.Publisher(f"{self.camera_name}/image_rect", Image, queue_size=1)
            self.compressed_image_pub = rospy.Publisher(
                f"{self.camera_name}/image_rect/compressed", CompressedImage, queue_size=1
            )
        else:
            self.camera_info_pub = None
            self.camera_image_pub = None
            self.compressed_image_pub = None
        self.debug_image_pub = rospy.Publisher(f"{self.camera_name}/debug_image", Image, queue_size=1)
        self.robot_tag_pub = rospy.Publisher(f"{self.camera_name}/robot_tags", EstimatedObjectArray, queue_size=1)
        self.april_tag_pub = rospy.Publisher(f"{self.camera_name}/tag_detections", AprilTagDetectionArray, queue_size=1)

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

    def make_rectifier(self, camera_info: CameraInfo) -> ImageRectifier:
        return ImageRectifier(camera_info, (self.width, self.height), alpha=self.camera_matrix_alpha)

    def make_bundle_detectors(self, rectified_info: CameraInfo) -> dict[str, BundleDetectorInterface]:
        bundle_detectors = {}
        params = self.make_ransac_params()
        for name, bundle_config in self.bundle_configs.items():
            detector = RansacBundleDetector(bundle_config, params, rectified_info)
            bundle_detectors[name] = detector
        return bundle_detectors

    def tick(self) -> None:
        header, frame = self.image_supplier.get_image()
        if frame is None:
            return

        self.rectified_info.header = header
        timestamp = header.stamp

        rectified = self.rectifier.rectify(frame)
        gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
        results = {}
        draw_image = rectified if self.debug_image else None
        for name, bundle_detector in self.bundle_detectors.items():
            detections = self.tag_detector.detect(gray)
            result = bundle_detector.detect(detections)
            draw_image = draw_bundle(self.rectified_info, draw_image, result) if (draw_image is not None) else None
            results[name] = result

        obj_msg = bundle_result_to_object(header, results)
        self.robot_tag_pub.publish(obj_msg)

        apriltag_msg = bundle_result_to_apriltag_ros(header, results.values())
        self.april_tag_pub.publish(apriltag_msg)

        if self.camera_info_pub:
            self.camera_info_pub.publish(self.rectified_info)
        try:
            image_msg = self.bridge.cv2_to_imgmsg(rectified, "bgr8")
            image_msg.header = header
            if self.camera_image_pub:
                self.camera_image_pub.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr(e)

        if timestamp - self.prev_compressed_pub_time > self.compressed_pub_interval:
            self.prev_compressed_pub_time = timestamp
            try:
                compressed_msg = self.bridge.cv2_to_compressed_imgmsg(rectified, "png")
                compressed_msg.header = header
                if self.compressed_image_pub:
                    self.compressed_image_pub.publish(compressed_msg)
            except CvBridgeError as e:
                rospy.logerr(e)

        if draw_image is not None:
            try:
                draw_msg = self.bridge.cv2_to_imgmsg(draw_image, "bgr8")
                draw_msg.header = header
                self.debug_image_pub.publish(draw_msg)
            except CvBridgeError as e:
                rospy.logerr(e)
