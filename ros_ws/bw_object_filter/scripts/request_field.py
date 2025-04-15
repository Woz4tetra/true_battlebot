#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty

rospy.init_node("request_field")

request_pub = rospy.Publisher("plane_request", Empty, queue_size=1)
request_pub.publish(Empty())

print("Requesting field")
while not rospy.is_shutdown():
    input("Press enter to request field again")
    request_pub.publish(Empty())
