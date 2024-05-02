import math
from typing import Optional

import rospy
from bw_interfaces.msg import EstimatedObject
from bw_shared.messages.header import Header
from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.pose2d_stamped import Pose2DStamped


class CornerManager:
    def __init__(self, corner_offset: float) -> None:
        self.corner_offset = corner_offset

        self.field = EstimatedObject()

        self.field_sub = rospy.Subscriber("filter/field", EstimatedObject, self.field_callback, queue_size=1)

        rospy.logdebug("Corner manager initialized")

    def field_callback(self, field: EstimatedObject) -> None:
        self.field = field

    def get_goal(self) -> Optional[Pose2DStamped]:
        if len(self.field.header.frame_id) == 0:
            rospy.logwarn("No field received yet")
            return None

        x = self.field.size.x / 2.0 - self.corner_offset
        y = self.field.size.y / 2.0 - self.corner_offset
        goal = Pose2D(-x, y, 0.0)
        goal.theta = goal.heading() + math.pi

        return Pose2DStamped(Header.auto("map"), goal)
