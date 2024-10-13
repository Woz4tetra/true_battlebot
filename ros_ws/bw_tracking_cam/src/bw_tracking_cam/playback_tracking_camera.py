#!/usr/bin/env python
from __future__ import annotations

from typing import Optional

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from bw_tracking_cam.tracking_camera_node import ImageSupplier, TrackingCameraNode, config_from_ros_param


class PlaybackImageSupplier(ImageSupplier):
    def __init__(self, camera_name: str):
        self.camera_name = camera_name
        self.bridge = CvBridge()

        self.camera_info = CameraInfo()
        self.last_image: Optional[np.ndarray] = None
        self.last_header = Header()
        self.frame_num = 0

        self.image_subscriber = rospy.Subscriber(f"/{self.camera_name}/image_rect", Image, self.image_callback)
        self.info_subscriber = rospy.Subscriber(f"/{self.camera_name}/camera_info", CameraInfo, self.info_callback)

    def image_callback(self, image: Image) -> None:
        self.last_header = image.header
        self.last_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

    def info_callback(self, info: CameraInfo) -> None:
        self.camera_info = info

    def get_image(self) -> tuple[Header, Optional[np.ndarray]]:
        return self.last_header, self.last_image

    def get_info(self) -> CameraInfo:
        return self.camera_info

    def wait_for_info(self) -> None:
        while not self.camera_info.header.frame_id:
            rospy.sleep(0.1)
        rospy.loginfo("Received camera info")
        self.info_subscriber.unregister()


def run() -> None:
    config = config_from_ros_param()
    image_supplier = PlaybackImageSupplier(config.camera_name)
    image_supplier.wait_for_info()
    config.publish_camera = False
    node = TrackingCameraNode(config, image_supplier)

    while not rospy.is_shutdown():
        node.tick()


if __name__ == "__main__":
    rospy.init_node("playback_tracking_camera", log_level=rospy.DEBUG)
    run()
