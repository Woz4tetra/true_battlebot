import rospy
from bw_tools.structs.pose2d_stamped import Pose2DStamped
from geometry_msgs.msg import PoseStamped
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class SetGoalToSimple(Behaviour):
    def __init__(self, container: Container, concurrency_slot: int = 0) -> None:
        super().__init__(self.__class__.__name__)
        self.concurrency_slot = concurrency_slot
        self.get_path_manager = container.get_path_manager
        self.behavior_start = rospy.Time.now()
        self.goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_callback)

    def initialise(self) -> None:
        self.behavior_start = rospy.Time.now()
        rospy.loginfo(f"Waiting for goal on {self.goal_sub.resolved_name}")

    def update(self) -> Status:
        goal = self.get_path_manager.goal
        if goal is not None and goal.target_pose.header.stamp > self.behavior_start:
            return Status.SUCCESS
        else:
            return Status.RUNNING

    def goal_callback(self, msg: PoseStamped) -> None:
        goal = Pose2DStamped.from_msg(msg)
        self.get_path_manager.set_goal(goal, self.concurrency_slot)
