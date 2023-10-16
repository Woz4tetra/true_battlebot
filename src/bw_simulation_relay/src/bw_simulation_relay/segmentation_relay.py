#!/usr/bin/env python3
from typing import Dict

import rospy
from bw_interfaces.msg import Labels
from sensor_msgs.msg import CameraInfo, Image


class SegmentationRelay:
    def __init__(self) -> None:
        rospy.init_node("segmentation_relay")

        self.image_sub = rospy.Subscriber("image", Image, self.image_callback)
        self.info_sub = rospy.Subscriber("camera_info", CameraInfo, self.info_callback)
        self.labels_sub = rospy.Subscriber("labels", Labels, self.labels_callback)
        self.label_map: Dict[str, int] = {}

    def image_callback(self, msg: Image) -> None:
        pass

    def info_callback(self, msg: CameraInfo) -> None:
        pass

    def labels_callback(self, msg: Labels) -> None:
        for name, obj_id in zip(msg.labels, msg.ids):
            if name in self.label_map and self.label_map[name] != obj_id:
                rospy.logwarn(f"Label {name} has changed ID from {self.label_map[name]} to {obj_id}")
            self.label_map[name] = obj_id

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    node = SegmentationRelay()
    node.run()
