from typing import Optional

import actionlib
import rospy
from bw_tools.structs.pose2d_stamped import Pose2DStamped
from mbf_msgs.msg import GetPathAction, GetPathFeedback, GetPathGoal, GetPathResult
from nav_msgs.msg import Path
from py_trees.common import Status


class GetPathManager:
    def __init__(self, goal_xy_tolerance: float) -> None:
        self.goal_xy_tolerance = goal_xy_tolerance
        self.action = actionlib.SimpleActionClient("/move_base_flex/get_path", GetPathAction)
        rospy.loginfo("Waiting for MBF get path action")
        self.action.wait_for_server()
        self.status = Status.RUNNING
        self.goal: Optional[GetPathGoal] = None
        self.path = Path()
        rospy.loginfo("MBF get path action connected")

    def set_goal(self, goal_pose: Pose2DStamped, concurrency_slot: int = 0) -> None:
        rospy.loginfo(f"Setting goal to {goal_pose}")
        goal = GetPathGoal()
        goal.use_start_pose = False
        goal.tolerance = self.goal_xy_tolerance
        goal.target_pose = goal_pose.to_msg()
        goal.concurrency_slot = concurrency_slot
        self.goal = goal

    def get_path(self) -> Optional[Path]:
        if self.status == Status.SUCCESS:
            return self.path
        else:
            return None

    def send_goal(self) -> bool:
        if self.goal is None:
            rospy.logwarn("No goal set")
            return False
        self.status = Status.RUNNING
        self.action.send_goal(self.goal, done_cb=self.action_done, feedback_cb=self.feedback_cb)
        self.goal = None
        self.status = Status.RUNNING
        return True

    def get_status(self) -> Status:
        return self.status

    def action_done(self, goal_status, result: GetPathResult) -> None:
        self.status = Status.SUCCESS if result.outcome == GetPathResult.SUCCESS else Status.FAILURE
        if self.status == Status.FAILURE:
            rospy.logerr(f"MBF get path action failed: {result.message}")
        else:
            self.path = result.path

    def feedback_cb(self, feedback: GetPathFeedback) -> None:
        pass

    def cancel(self) -> None:
        rospy.loginfo("Canceling MBF get path action")
        self.action.cancel_all_goals()
