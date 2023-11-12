#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from bw_interfaces.msg import EstimatedField
from bw_tools.configs.robot_config import RobotFleetConfig
from bw_tools.dataclass_deserialize import dataclass_deserialize
from bw_tools.structs.header import Header
from bw_tools.structs.occupancy_grid import OccupancyGrid
from bw_tools.typing import get_param
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point, Polygon
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap, GetMapResponse


class MapConverter:
    def __init__(self) -> None:
        robot_config = get_param("~robots", None)
        if robot_config is None:
            raise ValueError("Must specify robot_config in the parameter server")
        self.resolution = get_param("~resolution", 0.05)
        self.robot_radius = get_param("~robot_radius", 0.2)
        self.controlled_bot_name = get_param("~controlled_bot_name", "mini_bot")

        all_robots = dataclass_deserialize(RobotFleetConfig, robot_config)
        self.non_controlled_robots = [robot for robot in all_robots.robots if robot.name != self.controlled_bot_name]
        self.robot_names = [robot.name for robot in self.non_controlled_robots]
        self.filter_state_subs = {
            robot.name: rospy.Subscriber(
                f"{robot.name}/filter_state",
                Odometry,
                lambda msg, name=robot.name: self.filter_state_callback(msg, name),
                queue_size=10,
            )
            for robot in self.non_controlled_robots
        }

        self.map_msg = OccupancyGridMsg()
        self.obstacles = ObstacleArrayMsg()
        self.obstacles.obstacles = [ObstacleMsg() for _ in range(len(self.non_controlled_robots))]

        self.map_pub = rospy.Publisher("map", OccupancyGridMsg, queue_size=1, latch=True)
        self.obstacle_pub = rospy.Publisher("/move_base/TebLocalPlannerROS/obstacles", ObstacleArrayMsg, queue_size=10)

        self.map_server = rospy.Service("/static_map", GetMap, self.get_map_service)

        self.field_sub = rospy.Subscriber("filter/field", EstimatedField, self.field_callback)

    def field_callback(self, msg: EstimatedField) -> None:
        rospy.loginfo("Received field. Publishing map.")
        resolution = self.resolution
        width = msg.size.x
        height = msg.size.y

        px_width = int(round(width / resolution))
        px_height = int(round(height / resolution))

        image = np.full((px_height, px_width), 0, dtype=np.int8)

        image = cv2.rectangle(image, (0, 0), (px_width - 1, px_height - 1), (99,), 1)

        map = OccupancyGrid.from_image(Header.auto("map"), image, resolution)
        self.map_msg = map.to_msg()

        self.map_pub.publish(self.map_msg)

    def filter_state_callback(self, msg: Odometry, robot_name: str) -> None:
        object_id = self.robot_names.index(robot_name)
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

    def get_map_service(self, req: GetMap) -> GetMapResponse:
        while len(self.map_msg.header.frame_id) == 0:
            rospy.sleep(0.1)
        rospy.loginfo("Sending map in response to request")
        return GetMapResponse(map=self.map_msg)

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("map_converter")
    MapConverter().run()
