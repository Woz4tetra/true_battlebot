import math
from typing import Optional

import rospy
from bw_interfaces.msg import CageCorner as RosCageCorner
from bw_interfaces.msg import EstimatedField
from bw_tools.structs.cage_corner import CageCorner
from bw_tools.structs.header import Header
from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.pose2d_stamped import Pose2DStamped


class CornerManager:
    def __init__(self, corner_offset: float) -> None:
        self.corner_offset = corner_offset

        self.field = EstimatedField()
        self.cage_corner: Optional[CageCorner] = None

        self.field_sub = rospy.Subscriber("filter/field", EstimatedField, self.field_callback, queue_size=1)
        self.corner_side_sub = rospy.Subscriber("cage_corner", RosCageCorner, self.corner_side_callback, queue_size=1)

    def corner_side_callback(self, corner: RosCageCorner) -> None:
        self.cage_corner = CageCorner.from_msg(corner)

    def field_callback(self, field: EstimatedField) -> None:
        self.field = field

    def get_goal(self) -> Optional[Pose2DStamped]:
        if len(self.field.header.frame_id) == 0:
            rospy.logwarn("No field received yet")
            return None
        if self.cage_corner is None:
            rospy.logwarn("No cage corner received yet")
            return None

        x = self.field.size.x - self.corner_offset
        y = self.field.size.y - self.corner_offset
        if self.cage_corner == CageCorner.DOOR_SIDE:
            goal = Pose2D(-x, y, 0.0)
        else:
            goal = Pose2D(x, -y, 0.0)
        goal.theta = goal.heading() + math.pi

        return Pose2DStamped(Header.auto("map"), goal)
