#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from bw_interfaces.msg import EstimatedObject
from bw_tools.structs.header import Header
from bw_tools.structs.occupancy_grid import OccupancyGrid
from bw_tools.typing import get_param
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from nav_msgs.srv import GetMap, GetMapResponse


class MapConverter:
    def __init__(self) -> None:
        self.resolution = get_param("~resolution", 0.05)

        self.map_msg = OccupancyGridMsg()

        self.map_pub = rospy.Publisher("map", OccupancyGridMsg, queue_size=1, latch=True)
        self.map_server = rospy.Service("/static_map", GetMap, self.get_map_service)

        self.field_sub = rospy.Subscriber("filter/field", EstimatedObject, self.field_callback)

    def field_callback(self, msg: EstimatedObject) -> None:
        rospy.loginfo("Received field. Publishing map.")
        resolution = self.resolution
        width = msg.size.x
        height = msg.size.y

        px_width = int(round(width / resolution))
        px_height = int(round(height / resolution))

        image = np.full((px_height, px_width), 0, dtype=np.int8)

        image = cv2.rectangle(image, (0, 0), (px_width - 1, px_height - 1), (100,), 1).astype(np.int8)

        map = OccupancyGrid.from_image(Header.auto("map"), image, resolution)
        self.map_msg = map.to_msg()

        self.map_pub.publish(self.map_msg)

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
