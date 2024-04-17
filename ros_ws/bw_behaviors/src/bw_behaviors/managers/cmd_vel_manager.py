import rospy
from geometry_msgs.msg import Twist


class CmdVelManager:
    def __init__(self) -> None:
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def publish(self, cmd_vel: Twist) -> None:
        self.cmd_vel_pub.publish(cmd_vel)

    def stop(self) -> None:
        self.publish(Twist())
