import copy

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from bw_tools.typing import get_param
from mbf_msgs.msg import RecoveryAction, RecoveryGoal
from py_trees.common import Status


class RecoveryManager:
    def __init__(self) -> None:
        self.action = actionlib.SimpleActionClient("/move_base_flex/recovery", RecoveryAction)
        rospy.loginfo("Waiting for MBF recovery action")
        self.action.wait_for_server()
        behaviors = get_param("/move_base_flex/recovery_behaviors", [])
        self.behavior_names = [behavior["name"] for behavior in behaviors]
        self._reset_behaviors()
        self.recovery_sent = False

    def _reset_behaviors(self) -> None:
        self.available_behaviors = copy.deepcopy(self.behavior_names)

    def _get_next_behavior(self) -> str:
        return self.available_behaviors.pop(0)

    def send_recovery(self) -> None:
        goal = RecoveryGoal()
        try:
            goal.behavior = self._get_next_behavior()
        except IndexError:
            self._reset_behaviors()
            rospy.loginfo("No more recovery behaviors available")
            self.recovery_sent = False
            return
        rospy.loginfo(f"Sending {goal.behavior} MBF recovery action")
        self.action.send_goal(goal)
        self.recovery_sent = True

    def get_status(self) -> Status:
        if not self.recovery_sent:
            return Status.FAILURE
        if self.action.get_result() is None:
            return Status.RUNNING
        state = self.action.get_state()
        if state == GoalStatus.SUCCEEDED:
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def cancel(self) -> None:
        rospy.loginfo("Canceling MBF recovery action")
        self.action.cancel_all_goals()
