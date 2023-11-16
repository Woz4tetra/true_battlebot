import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from mbf_msgs.msg import ExePathAction, ExePathGoal, ExePathResult
from nav_msgs.msg import Path
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class ExePath(Behaviour):
    def __init__(self, container: Container, concurrency_slot: int = 0, path_ready_on_start: bool = True) -> None:
        super().__init__(self.__class__.__name__)
        self.concurrency_slot = concurrency_slot
        self.get_path_manager = container.get_path_manager
        self.path_ready_on_start = path_ready_on_start
        self.goal_sent = False
        self.action = actionlib.SimpleActionClient("/move_base_flex/exe_path", ExePathAction)
        rospy.loginfo("Waiting for MBF exe path action")
        self.action.wait_for_server()
        rospy.loginfo("MBF exe path action connected")

    def initialise(self) -> None:
        self.goal_sent = False

    def send_goal(self, path: Path) -> None:
        goal = ExePathGoal()
        goal.path = path
        goal.concurrency_slot = self.concurrency_slot
        self.goal_sent = True
        self.action.send_goal(goal)

    def update(self) -> Status:
        if not self.goal_sent:
            path = self.get_path_manager.get_path()
            if path is None:
                if self.path_ready_on_start:
                    rospy.logwarn("No path set")
                    return Status.FAILURE
                else:
                    return Status.RUNNING
            self.send_goal(path)
        return self.get_status()

    def get_status(self) -> Status:
        if self.action.get_result() is None:
            return Status.RUNNING
        state = self.action.get_state()
        if state == GoalStatus.SUCCEEDED:
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def terminate(self, new_status: Status) -> None:
        if self.get_status() == Status.RUNNING:
            self.cancel()

    def action_done(self, goal_status, result: ExePathResult) -> None:
        self.status = Status.SUCCESS if result.outcome == ExePathResult.SUCCESS else Status.FAILURE
        if self.status == Status.FAILURE:
            rospy.logerr(f"MBF exe path action failed: {result.message}")

    def cancel(self) -> None:
        rospy.loginfo("Canceling MBF exe path action")
        self.action.cancel_all_goals()
