#!/usr/bin/env python
from typing import Dict, Tuple

import cv2
import numpy as np
import rospy
from bw_interfaces.msg import Contour, SegmentationInstance, SegmentationInstanceArray, UVKeypoint
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class SegmentationRelay:
    def __init__(self) -> None:
        rospy.init_node("segmentation_relay")

        self.simulated_to_real_labels: Dict[str, str] = {
            "Mini bot": "robot",
            "Main bot": "robot",
            "Enemy bot": "robot",
            "Field": "field",
        }
        self.real_model_labels = ("field", "robot")

        self.simulated_segmentations: Dict[int, str] = {}
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("image", Image, self.image_callback)
        self.segmentation_pub = rospy.Publisher("segmentation", SegmentationInstanceArray, queue_size=10)
        self.debug_image_pub = rospy.Publisher("segmentation_relay_debug", Image, queue_size=1)
        self.simulated_segmentation_sub = rospy.Subscriber(
            "simulated_segmentation", SegmentationInstanceArray, self.simulated_segmentation_callback, queue_size=10
        )

    def image_callback(self, msg: Image) -> None:
        if len(self.simulated_segmentations) == 0:
            rospy.logwarn("No simulated segmentation received yet")
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        debug_image_enabled = self.debug_image_pub.get_num_connections() > 0

        segmentation_array = SegmentationInstanceArray()

        if debug_image_enabled:
            debug_image = np.zeros_like(image)
        else:
            debug_image = None
        object_counts = {label: 0 for label in self.real_model_labels}
        for color, label in self.simulated_segmentations.items():
            color_rgb = self.color_i32_to_rgb(color)
            mask = np.all(image == color_rgb, axis=2).astype(np.uint8)
            mask = self.bridge_gaps(mask, 3)
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if hierarchy is None:  # empty mask
                continue
            has_holes = (hierarchy.reshape(-1, 4)[:, 3] >= 0).sum() > 0  # type: ignore
            object_index = object_counts[label]
            segmentation = SegmentationInstance(
                contours=[self.to_contours_msg(contour) for contour in contours],
                score=1.0,
                label=label,
                class_index=self.real_model_labels.index(label),
                object_index=object_index,
                has_holes=has_holes,
            )
            object_counts[label] += 1
            segmentation_array.instances.append(segmentation)  # type: ignore

            if debug_image is not None:
                debug_image = cv2.drawContours(debug_image, contours, -1, color=color_rgb, thickness=1)
        if debug_image is not None:
            self.publish_debug_image(debug_image)

        segmentation_array.header = msg.header
        segmentation_array.height = msg.height
        segmentation_array.width = msg.width

        self.segmentation_pub.publish(segmentation_array)

    def color_i32_to_rgb(self, color: int) -> Tuple[int, int, int]:
        return color & 0xFF, (color >> 8) & 0xFF, (color >> 16) & 0xFF

    def to_contours_msg(self, contours: np.ndarray) -> Contour:
        contour_msg = Contour()
        for x, y in contours[:, 0]:
            contour_msg.points.append(UVKeypoint(x, y))  # type: ignore
        return contour_msg

    def bridge_gaps(self, image: np.ndarray, distance: int) -> np.ndarray:
        image = cv2.dilate(image, np.ones((distance, distance), np.uint8), iterations=1)
        return cv2.erode(image, np.ones((distance, distance), np.uint8), iterations=1)

    def publish_debug_image(self, image: np.ndarray) -> None:
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

    def simulated_segmentation_callback(self, msg: SegmentationInstanceArray) -> None:
        rospy.loginfo_once("Received simulated segmentation")
        for instant in msg.instances:  # type: ignore
            if instant.label not in self.simulated_to_real_labels:
                continue
            color = instant.class_index
            label = self.simulated_to_real_labels[instant.label]
            self.simulated_segmentations[color] = label

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    node = SegmentationRelay()
    node.run()