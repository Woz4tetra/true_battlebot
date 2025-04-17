import genpy
import rospy
from app.robot_filter.covariances.cmd_vel_heuristics import CmdVelHeuristics
from geometry_msgs.msg import Twist, TwistStamped, TwistWithCovariance


class CmdVelTracker:
    def __init__(self, cmd_vel_heuristics: CmdVelHeuristics, command_timeout: genpy.Duration) -> None:
        self.last_command: TwistStamped = TwistStamped()
        self.cmd_vel_heuristics = cmd_vel_heuristics
        self.command_timeout = command_timeout

    def set_velocity(self, command: Twist) -> None:
        self.last_command = TwistStamped(header=rospy.Header(stamp=rospy.Time.now()), twist=command)

    def is_timed_out(self) -> bool:
        return rospy.Time.now() - self.last_command.header.stamp > self.command_timeout

    def get_velocity(self) -> TwistWithCovariance:
        if self.is_timed_out():
            twist = Twist()
        else:
            twist = self.last_command.twist
        return TwistWithCovariance(
            twist=twist,
            covariance=self.cmd_vel_heuristics.compute_covariance(twist),
        )
