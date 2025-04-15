#!/usr/bin/env python

import rospy
from bw_interfaces.msg import Heartbeat
from bw_tools.get_param import get_param
from std_msgs.msg import Empty


class FieldPerceptionMiddleLayer:
    def __init__(self) -> None:
        self.perception_heartbeat = Heartbeat()
        self.field_request_pending = False
        auto_initialize = get_param("~auto_initialize", False)

        self.request_pub = rospy.Publisher("/field_request", Empty, queue_size=1)
        self.perception_heartbeat_sub = rospy.Subscriber(
            "/perception/heartbeat", Heartbeat, self.perception_heartbeat_callback, queue_size=1
        )

        if auto_initialize:
            self.request_field()

    def perception_heartbeat_callback(self, heartbeat: Heartbeat) -> None:
        self.perception_heartbeat = heartbeat

    def request_field(self) -> None:
        rospy.loginfo("Requesting field")
        self.field_request_pending = True

    def check_pending_field_request(self) -> None:
        if self.field_request_pending:
            delay = rospy.Time.now() - self.perception_heartbeat.header.stamp
            if delay > rospy.Duration(1):
                rospy.logwarn(f"Field request pending. Perception node is not responding. Delay: {delay.to_sec()}")
            else:
                rospy.loginfo("Field request pending. Sending request.")
                self.request_pub.publish(Empty())
                self.field_request_pending = False

    def run(self) -> None:
        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            self.check_pending_field_request()


if __name__ == "__main__":
    rospy.init_node("field_perception_middle_layer", log_level=rospy.DEBUG)
    FieldPerceptionMiddleLayer().run()
