import rospy
from bw_interfaces.msg import BehaviorMode as RosBehaviorMode
from bw_tools.structs.behavior_mode import BehaviorMode


class ModeManager:
    def __init__(self) -> None:
        self.mode_sub = rospy.Subscriber("behavior_mode", RosBehaviorMode, self.mode_callback, queue_size=10)
        self.mode = BehaviorMode.IDLE
        rospy.logdebug(f"Mode manager initialized in mode {self.mode}")

    def set_mode(self, mode: BehaviorMode) -> None:
        if mode != self.mode:
            rospy.loginfo(f"Setting mode to {mode}")
        self.mode = mode

    def mode_callback(self, msg: RosBehaviorMode) -> None:
        rospy.loginfo(f"Got mode switch message {msg}")
        self.set_mode(BehaviorMode(msg.mode))
