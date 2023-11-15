#!/usr/bin/env python
from threading import Lock
from typing import Optional, Tuple

import rospy
from bw_interfaces.msg import EstimatedObject, EstimatedObjectArray
from bw_tools.configs.robot_config import RobotFleetConfig, RobotTeam
from bw_tools.dataclass_deserialize import dataclass_deserialize
from bw_tools.structs.pose2d import Pose2D
from bw_tools.typing import get_param
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Polygon, PoseStamped


class TargetSelector:
    def __init__(self) -> None:
        robot_config = get_param("~robots", None)
        if robot_config is None:
            raise ValueError("Must specify robot_config in the parameter server")
        self.robot_radius = get_param("~robot_radius", 0.2)
        self.guidance_bot_name = get_param("~guidance_bot_name", "main_bot")
        self.controlled_bot_name = get_param("~controlled_bot_name", "mini_bot")

        all_robots = dataclass_deserialize(RobotFleetConfig, robot_config)
        self.non_controlled_robots = [robot for robot in all_robots.robots if robot.name != self.controlled_bot_name]
        self.non_controlled_robot_names = [robot.name for robot in self.non_controlled_robots]
        self.their_robot_names = [robot.name for robot in all_robots.robots if robot.team == RobotTeam.THEIR_TEAM]

        self.ignore_opponent_obstacles = False

        self.obstacles = ObstacleArrayMsg()
        self.obstacles.obstacles = [ObstacleMsg() for _ in range(len(self.non_controlled_robots))]

        self.field = EstimatedObject()

        self.obstacle_pub = rospy.Publisher("obstacles", ObstacleArrayMsg, queue_size=10)
        self.recommended_goal_pub = rospy.Publisher("recommended_goal", PoseStamped, queue_size=1)

        self.field_sub = rospy.Subscriber("filter/field", EstimatedObject, self.field_callback)
        self.filter_states_sub = rospy.Subscriber(
            "filtered_states", EstimatedObjectArray, self.filter_state_callback, queue_size=10
        )
        self.filter_lock = Lock()
        self.obstacle_lock = Lock()
        self.filter_states = EstimatedObjectArray()

    def field_callback(self, msg: EstimatedObject) -> None:
        self.field = msg

    def filter_state_callback(self, msg: EstimatedObjectArray) -> None:
        with self.filter_lock:
            self.filter_states = msg
        for robot in msg.robots:  # type: ignore
            robot: EstimatedObject
            robot_name = robot.label
            if self.ignore_opponent_obstacles and robot_name in self.their_robot_names:
                continue
            if robot_name not in self.non_controlled_robot_names:
                continue
            with self.obstacle_lock:
                object_id = self.non_controlled_robot_names.index(robot_name)
                position = robot.state.pose.pose.position
                self.obstacles.header = robot.header
                self.obstacles.obstacles[object_id] = ObstacleMsg(  # type: ignore
                    header=robot.header,
                    id=object_id,
                    radius=max(robot.size.x, robot.size.y, robot.size.z) / 2.0,
                    polygon=Polygon(points=[position]),
                    orientation=robot.state.pose.pose.orientation,
                    velocities=robot.state.twist,
                )
        self.obstacle_pub.publish(self.obstacles)

    def get_robot_state(self, name: str) -> Optional[EstimatedObject]:
        with self.filter_lock:
            for robot in self.filter_states.robots:  # type: ignore
                robot: EstimatedObject
                if robot.label == name:
                    return robot
            return None

    def get_nearest_opponent_state(self, target: EstimatedObject) -> Optional[EstimatedObject]:
        opponent_poses = []
        opponent_states = []
        with self.filter_lock:
            for robot in self.filter_states.robots:  # type: ignore
                robot: EstimatedObject
                if robot.label in self.their_robot_names:
                    opponent_poses.append(Pose2D.from_msg(robot.state.pose.pose))
                    opponent_states.append(robot)
        if len(opponent_poses) == 0:
            return None
        target_pose = Pose2D.from_msg(target.state.pose.pose)
        distances = [target_pose.magnitude(pose) for pose in opponent_poses]
        closest_index = distances.index(min(distances))
        return opponent_states[closest_index]

    def compute_goal(
        self, controlled: EstimatedObject, guidance: EstimatedObject, opponent: EstimatedObject, field: EstimatedObject
    ) -> Tuple[PoseStamped, bool]:
        """
        If the controlled robot is not in line with the guidance bot and opponent, select goal that's behind the
        opponent pointing towards the guidance bot.
        Otherwise, if the controlled bot is in position, select goal that's just in front the guidance bot
        (push the opponent to the guidance bot).
        """
        return PoseStamped(), False

    def goal_update(self) -> None:
        with self.obstacle_lock:
            self.ignore_opponent_obstacles = False
            if len(self.field.header.frame_id) == 0:
                return

            guidance_pose = self.get_robot_state(self.guidance_bot_name)
            if guidance_pose is None:
                return

            controlled_pose = self.get_robot_state(self.controlled_bot_name)
            if controlled_pose is None:
                return

            opponent_pose = self.get_nearest_opponent_state(guidance_pose)
            if opponent_pose is None:
                return

            goal, self.ignore_opponent_obstacles = self.compute_goal(
                controlled_pose, guidance_pose, opponent_pose, self.field
            )
            self.recommended_goal_pub.publish(goal)

    def run(self) -> None:
        rate = rospy.Rate(10)
        while True:
            rate.sleep()
            self.goal_update()


if __name__ == "__main__":
    rospy.init_node("target_selector")
    TargetSelector().run()
