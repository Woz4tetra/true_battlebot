#!/usr/bin/env python
import rospy
from bw_interfaces.msg import EstimatedObject, EstimatedObjectArray
from bw_tools.configs.robot_config import RobotConfig, RobotFleetConfig, RobotTeam
from bw_tools.dataclass_deserialize import dataclass_deserialize
from bw_tools.typing import get_param
from geometry_msgs.msg import Quaternion, Twist
from tf_conversions import transformations


class TeleopNode:
    def __init__(self) -> None:
        robot_config = get_param("~robots", None)
        if robot_config is None:
            raise ValueError("Must specify robot_config in the parameter server")
        self.robots = dataclass_deserialize(RobotFleetConfig, robot_config)

        self.filter_state_array_sub = rospy.Subscriber(
            "filtered_states", EstimatedObjectArray, self.filtered_states_callback, queue_size=50
        )
        self.states = {robot.name: True for robot in self.robots.robots if robot.team == RobotTeam.OUR_TEAM}
        self.cmd_vel_subs = [
            rospy.Subscriber(
                f"{robot.name}/cmd_vel/absolute", Twist, self.cmd_vel_callback, callback_args=robot, queue_size=10
            )
            for robot in self.robots.robots
            if robot.team == RobotTeam.OUR_TEAM
        ]
        self.cmd_vel_pubs = {
            robot.name: rospy.Publisher(f"{robot.name}/cmd_vel/relative", Twist, queue_size=10)
            for robot in self.robots.robots
            if robot.team == RobotTeam.OUR_TEAM
        }

    def cmd_vel_callback(self, msg: Twist, robot: RobotConfig) -> None:
        is_right_side_up = self.states[robot.name]
        if not is_right_side_up:
            msg.linear.x *= -1
        self.cmd_vel_pubs[robot.name].publish(msg)

    def filtered_states_callback(self, msg: EstimatedObjectArray) -> None:
        for robot in msg.robots:  # type: ignore
            robot: EstimatedObject
            if robot.label in self.states:
                self.states[robot.label] = self.is_orientation_right_side_up(robot.state.pose.pose.orientation)

    def is_orientation_right_side_up(self, quaternion: Quaternion) -> bool:
        angles = transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return abs(angles[0]) < 1.5

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("bw_teleop")
    TeleopNode().run()