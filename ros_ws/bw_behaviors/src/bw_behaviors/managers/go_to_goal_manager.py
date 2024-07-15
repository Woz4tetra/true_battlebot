import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from bw_interfaces.msg import GoToGoalAction, GoToGoalFeedback, GoToGoalGoal
from bw_tools.messages.goal_strategy import GoalStrategy
from bw_tools.messages.goal_type import GoalType
from bw_tools.messages.target_type import TargetType
from geometry_msgs.msg import PoseStamped
from py_trees.common import Status


class GoToGoalManager:
    def __init__(self) -> None:
        self.go_to_goal_client = SimpleActionClient("go_to_goal", GoToGoalAction)
        rospy.loginfo("Waiting for go to goal action server")
        self.go_to_goal_client.wait_for_server()
        rospy.loginfo("Go to goal action server is ready")
        self.status = Status.RUNNING
        self.strategy = GoalStrategy.CRASH_OPPONENT
        self.feedback = GoToGoalFeedback()

    def set_strategy(self, strategy: GoalStrategy) -> None:
        self.strategy = strategy

    def send_pose_goal(self, pose: PoseStamped) -> None:
        rospy.loginfo("Sending go to goal action")
        goal = GoToGoalGoal()
        goal.goal = pose
        goal.strategy = self.strategy.value
        goal.goal_type = GoalType.FIXED_POSE.value
        self.go_to_goal_client.send_goal(goal, feedback_cb=self.feedback_callback)

    def send_target_goal(self, target_type: TargetType) -> None:
        rospy.loginfo("Sending go to goal action")
        goal = GoToGoalGoal()
        goal.target_type = target_type.value
        goal.strategy = self.strategy.value
        goal.goal_type = GoalType.TRACKED_TARGET.value
        self.go_to_goal_client.send_goal(goal, feedback_cb=self.feedback_callback)

    def feedback_callback(self, feedback: GoToGoalFeedback) -> None:
        self.feedback = feedback

    def get_feedback(self) -> GoToGoalFeedback:
        return self.feedback

    def get_status(self) -> Status:
        if self.go_to_goal_client.get_result() is None:
            return Status.RUNNING
        state = self.go_to_goal_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def cancel(self) -> None:
        rospy.loginfo("Cancelling go to goal action")
        self.go_to_goal_client.cancel_all_goals()
