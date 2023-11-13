#!/usr/bin/env python
import rospy
from bw_tools.configs.robot_config import RobotFleetConfig, RobotTeam
from bw_tools.dataclass_deserialize import dataclass_deserialize
from bw_tools.typing import get_param
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point, Polygon, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class TargetSelector:
    def __init__(self) -> None:
        robot_config = get_param("~robots", None)
        if robot_config is None:
            raise ValueError("Must specify robot_config in the parameter server")
        self.robot_radius = get_param("~robot_radius", 0.2)
        self.controlled_bot_name = get_param("~controlled_bot_name", "mini_bot")

        all_robots = dataclass_deserialize(RobotFleetConfig, robot_config)
        self.non_controlled_robots = [robot for robot in all_robots.robots if robot.name != self.controlled_bot_name]
        self.non_controlled_robot_names = [robot.name for robot in self.non_controlled_robots]
        self.their_robot_names = [robot.name for robot in all_robots.robots if robot.team == RobotTeam.THEIR_TEAM]

        self.ignore_opponent_obstacles = False

        self.obstacles = ObstacleArrayMsg()
        self.obstacles.obstacles = [ObstacleMsg() for _ in range(len(self.non_controlled_robots))]

        self.obstacle_pub = rospy.Publisher("obstacles", ObstacleArrayMsg, queue_size=10)
        self.recommended_goal_pub = rospy.Publisher("recommended_goal", PoseStamped, queue_size=1)

        self.ignore_opponents_sub = rospy.Subscriber("ignore_opponents", Bool, self.ignore_opponents_callback)
        self.filter_state_subs = {
            robot.name: rospy.Subscriber(
                f"{robot.name}/filter_state",
                Odometry,
                lambda msg, name=robot.name: self.filter_state_callback(msg, name),
                queue_size=10,
            )
            for robot in self.non_controlled_robots
        }

    def filter_state_callback(self, msg: Odometry, robot_name: str) -> None:
        if self.ignore_opponent_obstacles and robot_name in self.their_robot_names:
            return
        object_id = self.non_controlled_robot_names.index(robot_name)
        position = Point(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y, z=0.0)
        self.obstacles.header = msg.header
        self.obstacles.obstacles[object_id] = ObstacleMsg(
            header=msg.header,
            id=object_id,
            radius=self.robot_radius,
            polygon=Polygon(points=[position]),
            orientation=msg.pose.pose.orientation,
            velocities=msg.twist,
        )
        self.obstacle_pub.publish(self.obstacles)

    def ignore_opponents_callback(self, msg: Bool) -> None:
        self.ignore_opponent_obstacles = msg.data
        rospy.loginfo("Ignoring opponents" if self.ignore_opponent_obstacles else "Not ignoring opponents")

    def run(self) -> None:
        # rospy.spin()
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = 0.5
        goal.pose.orientation.w = 1.0
        while True:
            self.recommended_goal_pub.publish(goal)
            goal.pose.position.x *= -1
            rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node("target_selector")
    TargetSelector().run()
