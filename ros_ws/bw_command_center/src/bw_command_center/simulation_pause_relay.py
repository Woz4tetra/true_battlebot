#!/usr/bin/env python
from dataclasses import dataclass

import rospy
from bw_interfaces.msg import BehaviorMode as BehaviorModeMsg
from bw_tools.messages.behavior_mode import BehaviorMode
from std_msgs.msg import Bool


@dataclass
class AppData:
    set_pause_pub: rospy.Publisher


def behavior_callback(app: AppData, msg: BehaviorModeMsg) -> None:
    mode = BehaviorMode.from_msg(msg)
    if mode == BehaviorMode.FIGHT:
        app.set_pause_pub.publish(Bool(data=False))


def main() -> None:
    rospy.init_node("simulation_pause_relay", log_level=rospy.DEBUG)
    set_pause_pub = rospy.Publisher("/simulation/set_pause", Bool)
    app = AppData(set_pause_pub)
    rospy.Subscriber("/behavior_mode", BehaviorModeMsg, callback=lambda msg: behavior_callback(app, msg))
    rospy.spin()


if __name__ == "__main__":
    main()
