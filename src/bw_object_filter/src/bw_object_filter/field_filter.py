#!/usr/bin/env python
import rospy
import tf2_ros
from bw_interfaces.msg import EstimatedField
from bw_tools.structs.transform3d import Transform3D
from bw_tools.typing import get_param, seconds_to_duration
from geometry_msgs.msg import Point, PointStamped, Pose, Vector3
from zed_interfaces.msg import PlaneStamped


class FieldFilter:
    def __init__(self) -> None:
        self.request_interval = seconds_to_duration(get_param("request_interval", 1.0))

        self.map_frame = get_param("map_frame", "map")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.recommended_point_sub = rospy.Subscriber(
            "estimation/recommended_field_point", PointStamped, self.recommended_point_callback
        )
        self.plane_request_pub = rospy.Publisher("plane_request", PointStamped, queue_size=1)
        self.plane_response_sub = rospy.Subscriber("plane_response", PlaneStamped, self.plane_response_callback)
        self.estimated_field_pub = rospy.Publisher("filter/field", EstimatedField, queue_size=1)

        self.prev_request_time = rospy.Time(0)

    def recommended_point_callback(self, point: PointStamped) -> None:
        if rospy.Time.now() - self.prev_request_time > self.request_interval:
            self.publish_plane_request(point)
            self.prev_request_time = rospy.Time.now()

    def publish_plane_request(self, request_point: PointStamped) -> None:
        self.plane_request_pub.publish(request_point)

    def plane_response_callback(self, plane: PlaneStamped) -> None:
        estimated_field = EstimatedField(
            header=plane.header,
            center=Pose(
                position=Point(plane.pose.translation.x, plane.pose.translation.y, plane.pose.translation.z),
                orientation=plane.pose.rotation,
            ),
            size=Vector3(x=plane.extents[0], y=plane.extents[1]),
        )
        self.estimated_field_pub.publish(estimated_field)
        self.publish_field_tf(estimated_field)

    def publish_field_tf(self, estimated_field: EstimatedField) -> None:
        transform = Transform3D.from_position_and_quaternion(
            Vector3(
                x=estimated_field.center.position.x,
                y=estimated_field.center.position.y,
                z=estimated_field.center.position.z,
            ),
            estimated_field.center.orientation,
        )
        transform = transform.inverse()

        field_tf = tf2_ros.TransformStamped()
        field_tf.header.stamp = estimated_field.header.stamp
        field_tf.header.frame_id = self.map_frame
        field_tf.child_frame_id = estimated_field.header.frame_id
        field_tf.transform = transform.to_msg()
        self.tf_broadcaster.sendTransform(field_tf)

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("field_filter")
    field_filter = FieldFilter()
    field_filter.run()
