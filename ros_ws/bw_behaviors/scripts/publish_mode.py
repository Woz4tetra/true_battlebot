#!/usr/bin/env python3
import argparse

import rospy
from bw_behaviors.modes import Mode
from bw_interfaces.msg import BehaviorMode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", default="", help="Name of mode")
    parser.add_argument("--cli", action="store_true", help="Enter CLI mode")
    args = parser.parse_args()

    rospy.init_node("set_mode_script")
    mode_pub = rospy.Publisher("/bw/behavior_mode", BehaviorMode, queue_size=1, latch=True)

    def publish_mode(mode_str: str):
        mode = Mode(mode_str)
        mode_pub.publish(BehaviorMode(mode=mode.value))

    if len(args.mode) > 0:
        publish_mode(args.mode)

    if args.cli:
        while not rospy.is_shutdown():
            mode_str = input("> ")
            publish_mode(mode_str)
    else:
        rospy.sleep(0.5)


if __name__ == "__main__":
    main()
