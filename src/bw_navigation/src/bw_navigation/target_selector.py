#!/usr/bin/env python
from threading import Lock
from typing import Optional, Tuple, cast

import numpy as np
import rospy
from bw_interfaces.msg import EstimatedObject, EstimatedObjectArray
from bw_tools.configs.robot_config import RobotFleetConfig, RobotTeam
from bw_tools.structs.pose2d import Pose2D
from bw_tools.typing import get_param
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32, Polygon, PoseStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from bw_navigation.selector_algorithms import CrashSelector, PushFromBehindSelector, SacrificialSelector
from bw_navigation.selector_algorithms.base_selector import BaseSelector
from bw_navigation.selector_algorithms.match_state import MatchState

CLOUD_FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
]


class TargetSelector:
    def __init__(self) -> None:
        robot_config = get_param("/robots", None)
        if robot_config is None:
            raise ValueError("Must specify robot_config in the parameter server")
        self.robot_radius = get_param("~robot_radius", 0.2)
        self.guidance_bot_name = get_param("~guidance_bot_name", "main_bot")
        self.controlled_bot_name = get_param("~controlled_bot_name", "mini_bot")
        self.algorithm_name = get_param("~algorithm", "sacrificial_selector")

        self.selection_algorithm: BaseSelector = {
            "sacrificial_selector": SacrificialSelector,
            "push_from_behind_selector": PushFromBehindSelector,
            "crash_selector": CrashSelector,
        }[self.algorithm_name]()

        all_robots = RobotFleetConfig.from_dict(robot_config)
        self.non_controlled_robots = [robot for robot in all_robots.robots if robot.name != self.controlled_bot_name]
        self.non_controlled_robot_names = [robot.name for robot in self.non_controlled_robots]
        self.their_robot_names = [robot.name for robot in all_robots.robots if robot.team == RobotTeam.THEIR_TEAM]

        self.ignore_opponent_obstacles = False

        self.field = EstimatedObject()

        self.obstacle_pub = rospy.Publisher("obstacles", ObstacleArrayMsg, queue_size=10)
        self.obstacle_cloud_pub = rospy.Publisher("obstacle_cloud", PointCloud2, queue_size=1)
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
        cloud_obstacles = np.array([], dtype=np.float32)
        obstacles = ObstacleArrayMsg()
        for robot in msg.robots:
            robot_name = robot.label
            if self.ignore_opponent_obstacles and robot_name in self.their_robot_names:
                continue
            if robot_name not in self.non_controlled_robot_names:
                continue
            with self.obstacle_lock:
                object_id = self.non_controlled_robot_names.index(robot_name)
                position = cast(Point32, robot.state.pose.pose.position)
                obstacles.header = robot.state.header
                diameter = max(robot.size.x, robot.size.y, robot.size.z)
                obstacles.obstacles.append(
                    ObstacleMsg(
                        header=robot.state.header,
                        id=object_id,
                        radius=diameter,  # TEB actually uses this as diameter
                        polygon=Polygon(points=[position]),
                        orientation=robot.state.pose.pose.orientation,
                        velocities=robot.state.twist,
                    )
                )
                object_pose = Pose2D.from_msg(robot.state.pose.pose)
                single_cloud_obstacle = self.get_obstacle_point(object_pose, diameter / 2, 8)
                if len(cloud_obstacles) == 0:
                    cloud_obstacles = single_cloud_obstacle
                else:
                    cloud_obstacles = np.append(cloud_obstacles, single_cloud_obstacle, axis=0)
        if len(obstacles.obstacles):
            cloud_msg = self.get_obstacle_cloud(obstacles.header, cloud_obstacles)
            self.obstacle_cloud_pub.publish(cloud_msg)
        self.obstacle_pub.publish(obstacles)

    def get_obstacle_point(self, center: Pose2D, radius: float, num_points: int) -> np.ndarray:
        angles = np.linspace(0, 2 * np.pi, num_points + 1)[1:]
        p0 = Pose2D(x=radius, y=0.0, theta=0.0)
        points = [p0]
        for angle in angles:
            offset = p0.transform_by(Pose2D(0.0, 0.0, angle))
            offset.theta = 0.0
            points.append(offset)
        points = [center.transform_by(point) for point in points]
        point_array = np.array([point.to_array()[0:2] for point in points], dtype=np.float32)
        return point_array

    def get_obstacle_cloud(self, header: Header, points: np.ndarray) -> PointCloud2:
        points_3d = np.concatenate((points, np.zeros((points.shape[0], 1))), axis=1)
        return point_cloud2.create_cloud(header, CLOUD_FIELDS, points_3d.tolist())

    def get_robot_state(self, name: str) -> Optional[EstimatedObject]:
        with self.filter_lock:
            for robot in self.filter_states.robots:
                if robot.label == name:
                    return robot
            return None

    def get_nearest_opponent_state(self, target: EstimatedObject) -> Optional[EstimatedObject]:
        opponent_poses = []
        opponent_states = []
        with self.filter_lock:
            for robot in self.filter_states.robots:
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
        state = MatchState(controlled.state.header.frame_id, controlled, guidance, opponent, field)
        result = self.selection_algorithm.get_target(state)
        return result.goal, result.ignore_opponent_obstacles

    def goal_update(self) -> None:
        with self.obstacle_lock:
            self.ignore_opponent_obstacles = False
            if len(self.field.state.header.frame_id) == 0:
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
