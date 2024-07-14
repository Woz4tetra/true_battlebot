import rospy
from geometry_msgs.msg import PoseStamped
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class SendClickedGoal(Behaviour):
    def __init__(self, container: Container) -> None:
        super().__init__(self.__class__.__name__)
        self.go_to_goal_manager = container.go_to_goal_manager
        self.start_time = rospy.Time.now()
        self.goal = PoseStamped()
        self.goal_sent = False
        self.goal_sub = rospy.Subscriber("clicked_goal", PoseStamped, self.goal_callback)

    def initialise(self) -> None:
        self.start_time = rospy.Time.now()
        self.goal_sent = False
        rospy.loginfo(f"Waiting for goal on {self.goal_sub.resolved_name}")

    def update(self) -> Status:
        if not self.goal_sent and self.goal.header.stamp > self.start_time:
            rospy.loginfo("Received new goal")
            self.goal_sent = True
            self.go_to_goal_manager.send_pose_goal(self.goal)

        return self.go_to_goal_manager.get_status()

    def goal_callback(self, msg: PoseStamped) -> None:
        self.goal = msg
