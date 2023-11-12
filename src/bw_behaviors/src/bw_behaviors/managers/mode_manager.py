import rospy
from bw_interfaces.msg import BehaviorMode

from bw_behaviors.structs.modes import Mode


class ModeManager:
    def __init__(self) -> None:
        self.mode_sub = rospy.Subscriber("behavior_mode", BehaviorMode, self.mode_callback, queue_size=10)
        self.mode = Mode.IDLE

    def set_mode(self, mode: Mode) -> None:
        if mode != self.mode:
            rospy.loginfo(f"Setting mode to {mode}")
        self.mode = mode

    def mode_callback(self, msg: BehaviorMode) -> None:
        rospy.loginfo(f"Got mode switch message {msg}")
        self.set_mode(Mode(msg.mode))
