#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image


class SegmentationRelay:
    def __init__(self) -> None:
        rospy.init_node("segmentation_relay")

        self.image_sub = rospy.Subscriber("image", Image, self.image_callback)

    def image_callback(self, msg: Image) -> None:
        pass

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    node = SegmentationRelay()
    node.run()
