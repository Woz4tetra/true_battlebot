import genpy
import rospy
from geometry_msgs.msg import TwistStamped, TwistWithCovariance

from bw_object_filter.covariances.cmd_vel_heuristics import CmdVelHeuristics


class CmdVelTracker:
    def __init__(
        self,
        cmd_vel_heuristics: CmdVelHeuristics,
        history_length: genpy.Duration,
        command_timeout: genpy.Duration,
    ) -> None:
        self.commands: list[TwistStamped] = []
        self.cmd_vel_heuristics = cmd_vel_heuristics
        self.history_length = history_length
        self.command_timeout = command_timeout

    def add_velocity(self, command: TwistStamped) -> None:
        now = rospy.Time.now()
        self.commands.append(command)
        while self.commands and now - self.commands[0].header.stamp > self.history_length:
            self.commands.pop(0)

    def get_nearest_time(self, lookup_time: genpy.Time) -> int:
        nearest_time = 0
        for index, cmd in enumerate(self.commands):
            if cmd.header.stamp > lookup_time:
                break
            nearest_time = index
        return nearest_time

    def get_nearest_velocity(self, lookup_time: genpy.Time) -> TwistStamped:
        if not self.commands:
            return TwistStamped()
        nearest_time = self.get_nearest_time(lookup_time)
        return self.commands[nearest_time]

    def get_velocity(self, lookup_time: genpy.Time) -> TwistWithCovariance:
        if rospy.Time.now() - self.commands[-1].header.stamp > self.command_timeout:
            twist = TwistStamped()
        else:
            twist = self.get_nearest_velocity(lookup_time)
        return TwistWithCovariance(
            twist=twist.twist,
            covariance=self.cmd_vel_heuristics.compute_covariance(twist),  # type: ignore
        )

    def get_velocities_after(self, lookup_time: genpy.Time) -> list[TwistStamped]:
        velocities = []
        for cmd in self.commands:
            if cmd.header.stamp > lookup_time:
                velocities.append(cmd)
        return velocities

    def get_velocities(self) -> list[TwistStamped]:
        velocities = self.get_velocities_after(rospy.Time.now() - self.history_length)
        return velocities
