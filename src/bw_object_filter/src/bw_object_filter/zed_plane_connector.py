#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from zed_interfaces.msg import PlaneStamped


def plane_callback(plane: PlaneStamped, pose_pub: rospy.Publisher) -> None:
    pose = PoseStamped()
    pose.header = plane.header
    pose.pose.position.x = plane.pose.translation.x
    pose.pose.position.y = plane.pose.translation.y
    pose.pose.position.z = plane.pose.translation.z
    pose.pose.orientation.x = plane.pose.rotation.x
    pose.pose.orientation.y = plane.pose.rotation.y
    pose.pose.orientation.z = plane.pose.rotation.z
    pose.pose.orientation.w = plane.pose.rotation.w
    pose_pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("zed_plane_connector")
    pose_pub = rospy.Publisher("pose", PoseStamped, queue_size=10)
    rospy.Subscriber("plane", PlaneStamped, plane_callback, callback_args=pose_pub)
    rospy.Subscriber("plane_marker", Marker, lambda msg: None)  # fixes a bug in zed_wrapper
    rospy.spin()
