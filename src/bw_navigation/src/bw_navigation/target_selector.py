#!/usr/bin/env python
from threading import Lock
from typing import Optional, Tuple

import rospy
from bw_interfaces.msg import EstimatedField
from bw_tools.configs.robot_config import RobotFleetConfig, RobotTeam
from bw_tools.dataclass_deserialize import dataclass_deserialize
from bw_tools.structs.pose2d import Pose2D
from bw_tools.typing import get_param
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point, Polygon, PoseStamped
from nav_msgs.msg import Odometry


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

        self.field = EstimatedField()

        self.obstacle_pub = rospy.Publisher("obstacles", ObstacleArrayMsg, queue_size=10)
        self.recommended_goal_pub = rospy.Publisher("recommended_goal", PoseStamped, queue_size=1)

        self.field_sub = rospy.Subscriber("filter/field", EstimatedField, self.field_callback)
        self.filter_state_subs = {
            robot.name: rospy.Subscriber(
                f"{robot.name}/filter_state",
                Odometry,
                lambda msg, name=robot.name: self.filter_state_callback(msg, name),
                queue_size=10,
            )
            for robot in self.non_controlled_robots
        }
        self.filter_lock = Lock()
        self.obstacle_lock = Lock()
        self.filter_states = {robot.name: Odometry() for robot in all_robots.robots}

    def field_callback(self, msg: EstimatedField) -> None:
        self.field = msg

    def filter_state_callback(self, msg: Odometry, robot_name: str) -> None:
        with self.filter_lock:
            self.filter_states[robot_name] = msg
        with self.obstacle_lock:
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

    def get_guidance_pose(self) -> Optional[Pose2D]:
        with self.filter_lock:
            state = self.filter_states[self.guidance_bot_name]
            if len(state.header.frame_id) == 0:
                return None
            else:
                return Pose2D.from_msg(state.pose.pose)

    def get_controlled_pose(self) -> Optional[Pose2D]:
        with self.filter_lock:
            state = self.filter_states[self.controlled_bot_name]
            if len(state.header.frame_id) == 0:
                return None
            else:
                return Pose2D.from_msg(state.pose.pose)

    def get_nearest_opponent_pose(self, target_pose: Pose2D) -> Optional[Pose2D]:
        opponent_poses = []
        with self.filter_lock:
            for name in self.their_robot_names:
                state = self.filter_states[name]
                if len(state.header.frame_id) != 0:
                    opponent_poses.append(Pose2D.from_msg(state.pose.pose))
        if len(opponent_poses) == 0:
            return None
        distances = [target_pose.magnitude(pose) for pose in opponent_poses]
        closest_index = distances.index(min(distances))
        return opponent_poses[closest_index]

    def compute_goal(
        self, controlled_pose: Pose2D, guidance_pose: Pose2D, opponent_pose: Pose2D, field: EstimatedField
    ) -> Tuple[PoseStamped, bool]:
        """
        If the controlled robot is not in line with the guidance bot and opponent, select goal that's behind the
        opponent pointing towards the guidance bot.
        Otherwise, if the controlled bot is in position, select goal that's just in front the guidance bot
        (push the opponent to the guidance bot).
        """

    def goal_update(self) -> None:
        with self.obstacle_lock:
            self.ignore_opponent_obstacles = False
            if len(self.field.header.frame_id) == 0:
                return

            guidance_pose = self.get_guidance_pose()
            if guidance_pose is None:
                return

            controlled_pose = self.get_controlled_pose()
            if controlled_pose is None:
                return

            opponent_pose = self.get_nearest_opponent_pose(guidance_pose)
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
