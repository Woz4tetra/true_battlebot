#!/usr/bin/env python
import math
from typing import Optional

import numpy as np
import rospy
import tf2_ros
from bw_interfaces.msg import CageCorner as RosCageCorner
from bw_interfaces.msg import EstimatedField
from bw_tools.structs.cage_corner import CageCorner
from bw_tools.structs.rpy import RPY
from bw_tools.structs.transform3d import Transform3D
from bw_tools.typing import get_param
from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty
from zed_interfaces.msg import PlaneStamped


class FieldFilter:
    def __init__(self) -> None:
        self.angle_delta_threshold = math.radians(get_param("angle_delta_threshold_degrees", 3.0))
        self.map_frame = get_param("map_frame", "map")

        self.has_manual_query = True
        self.current_imu = Imu()
        self.prev_imu = Imu()
        self.current_imu.orientation.w = 1.0
        self.prev_imu.orientation.w = 1.0
        self.cage_corner: Optional[CageCorner] = None
        self.field_rotations = {
            CageCorner.DOOR_SIDE: Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, math.pi / 2))),
            CageCorner.FAR_SIDE: Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, -math.pi / 2))),
        }
        self.estimated_field = EstimatedField()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.recommended_point_sub = rospy.Subscriber(
            "estimation/recommended_field_point", PointStamped, self.recommended_point_callback
        )
        self.camera_tilt_sub = rospy.Subscriber("imu", Imu, self.imu_callback, queue_size=1)
        self.manual_request_sub = rospy.Subscriber(
            "manual_plane_request", Empty, self.manual_request_callback, queue_size=1
        )
        self.plane_request_pub = rospy.Publisher("plane_request", PointStamped, queue_size=1)
        self.plane_response_sub = rospy.Subscriber("plane_response", PlaneStamped, self.plane_response_callback)
        self.estimated_field_pub = rospy.Publisher("filter/field", EstimatedField, queue_size=1, latch=True)
        self.corner_side_sub = rospy.Subscriber(
            "set_cage_corner", RosCageCorner, self.corner_side_callback, queue_size=1
        )
        self.corner_pub = rospy.Publisher("cage_corner", RosCageCorner, queue_size=1, latch=True)

        self.prev_request_time = rospy.Time(0)

    def recommended_point_callback(self, point: PointStamped) -> None:
        if self.did_camera_tilt():
            self.publish_plane_request(point)
            self.prev_request_time = rospy.Time.now()

    def manual_request_callback(self, _: Empty) -> None:
        self.has_manual_query = True

    def publish_plane_request(self, request_point: PointStamped) -> None:
        self.plane_request_pub.publish(request_point)

    def plane_response_callback(self, plane: PlaneStamped) -> None:
        self.rotate_to_cage_aligned(plane)
        self.estimated_field = EstimatedField(
            header=plane.header,
            center=Pose(
                position=Point(plane.pose.translation.x, plane.pose.translation.y, plane.pose.translation.z),
                orientation=plane.pose.rotation,
            ),
            size=Vector3(x=plane.extents[0], y=plane.extents[1]),
        )
        self.estimated_field_pub.publish(self.estimated_field)

    def rotate_to_cage_aligned(self, plane: PlaneStamped) -> None:
        if self.cage_corner is None:
            rospy.logwarn("No cage corner received. Assuming our team's corner is on the door side.")
            cage_corner = CageCorner.DOOR_SIDE
        else:
            cage_corner = self.cage_corner
        plane_rotation = Transform3D.from_position_and_quaternion(Vector3(), plane.pose.rotation)
        plane_rotation = plane_rotation.transform_by(self.field_rotations[cage_corner])
        plane.pose.rotation = plane_rotation.quaternion

    def corner_side_callback(self, corner: RosCageCorner) -> None:
        self.cage_corner = CageCorner.from_msg(corner)
        self.corner_pub.publish(corner)

    def imu_callback(self, imu: Imu) -> None:
        self.current_imu = imu

    def did_camera_tilt(self) -> bool:
        if self.has_manual_query:
            rospy.loginfo("Manual plane request received")
            self.has_manual_query = False
            return True
        delta_rpy = self.compute_delta_rpy(self.current_imu.orientation, self.prev_imu.orientation)
        self.prev_imu = self.current_imu
        return np.any(np.abs(delta_rpy.to_array()) > self.angle_delta_threshold)

    def compute_delta_rpy(self, new_quat: Quaternion, old_quat: Quaternion) -> RPY:
        new_transform = Transform3D.from_position_and_quaternion(Vector3(0, 0, 0), new_quat)
        old_transform = Transform3D.from_position_and_quaternion(Vector3(0, 0, 0), old_quat)
        delta_transform = new_transform.transform_by(old_transform.inverse())
        return delta_transform.rpy

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
        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            if len(self.estimated_field.header.frame_id) == 0:
                continue
            self.estimated_field.header.stamp = rospy.Time.now()
            self.publish_field_tf(self.estimated_field)


if __name__ == "__main__":
    rospy.init_node("field_filter")
    field_filter = FieldFilter()
    field_filter.run()
