import rospy
from geometry_msgs.msg import Twist, TwistStamped, TwistWithCovariance

from bw_object_filter.covariances.cmd_vel_heuristics import CmdVelHeuristics


class CmdVelTracker:
    def __init__(self, cmd_vel_heuristics: CmdVelHeuristics, command_timeout: rospy.Duration) -> None:
        self.last_command = TwistStamped()
        self.cmd_vel_heuristics = cmd_vel_heuristics
        self.command_timeout = command_timeout

    def set_command(self, command: Twist) -> None:
        command_stamped = TwistStamped(header=rospy.Header(stamp=rospy.Time.now()), twist=command)
        self.last_command = command_stamped

    def get_command(self) -> TwistWithCovariance:
        if rospy.Time.now() - self.last_command.header.stamp > self.command_timeout:
            twist = Twist()
        else:
            twist = self.last_command.twist
        return TwistWithCovariance(
            twist=twist,
            covariance=self.cmd_vel_heuristics.compute_covariance(twist),  # type: ignore
        )
