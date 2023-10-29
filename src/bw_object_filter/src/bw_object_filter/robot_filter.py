import rospy
from bw_interfaces.msg import EstimatedRobotArray


class RobotFilter:
    def __init__(self) -> None:
        self.robots_sub = rospy.Subscriber("estimation/robots", EstimatedRobotArray, self.robot_estimation_callback)

    def robot_estimation_callback(self, msg: EstimatedRobotArray) -> None:
        pass
