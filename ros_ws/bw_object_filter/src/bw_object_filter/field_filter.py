#!/usr/bin/env python
import math
from typing import Optional, Tuple

import rospy
import tf2_ros
from bw_interfaces.msg import CageCorner as RosCageCorner
from bw_interfaces.msg import EstimatedObject
from bw_shared.enums.field_type import FieldType
from bw_shared.enums.label import Label
from bw_shared.environment import get_map
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.xyz import XYZ
from bw_tools.configs.rosparam_client import get_shared_config
from bw_tools.get_param import get_param
from bw_tools.structs.cage_corner import CageCorner
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA, Empty, Header
from visualization_msgs.msg import Marker, MarkerArray


class FieldFilter:
    def __init__(self) -> None:
        shared_config = get_shared_config()
        map_name = get_map()
        map_config = shared_config.get_map(FieldType(map_name))

        self.map_frame = get_param("~map_frame", "map")
        self.relative_map_frame = get_param("~relative_map_frame", "map_relative")
        auto_initialize = get_param("~auto_initialize", False)
        self.expected_size = XYZ.from_size(map_config.size)
        field_dims_buffer = get_param("~field_dims_buffer", 0.25)
        buffer_extents = XYZ(field_dims_buffer, field_dims_buffer, field_dims_buffer)
        self.extents_range = (self.expected_size - buffer_extents, self.expected_size + buffer_extents)
        rospy.loginfo(f"Map name: {map_name}")
        rospy.loginfo(f"Extents range: {self.extents_range}")

        self.cage_corner: Optional[CageCorner] = None
        self.field_rotations = {
            CageCorner.DOOR_SIDE: Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, 0))),
            CageCorner.FAR_SIDE: Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, math.pi))),
        }
        self.estimated_field = EstimatedObject()
        self.field_rotate_tf = self.field_rotations[CageCorner.DOOR_SIDE]

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.request_pub = rospy.Publisher("/perception/field/request", Empty, queue_size=1)
        self.response_sub = rospy.Subscriber("/perception/field/response", EstimatedObject, self.response_callback)
        self.estimated_field_pub = rospy.Publisher("filter/field", EstimatedObject, queue_size=1, latch=True)
        self.estimated_field_marker_pub = rospy.Publisher("filter/field/marker", MarkerArray, queue_size=1, latch=True)
        self.corner_pub = rospy.Publisher("cage_corner", RosCageCorner, queue_size=1, latch=True)

        self.manual_request_sub = rospy.Subscriber(
            "manual_plane_request", Empty, self.manual_request_callback, queue_size=1
        )
        self.corner_side_sub = rospy.Subscriber(
            "set_cage_corner", RosCageCorner, self.corner_side_callback, queue_size=1
        )

        if auto_initialize:
            self.request_field()

    def manual_request_callback(self, _: Empty) -> None:
        rospy.loginfo("Manual plane request received")
        self.request_field()

    def request_field(self) -> None:
        rospy.loginfo("Requesting field")
        self.request_pub.publish(Empty())

    def response_callback(self, field: EstimatedObject) -> None:
        rospy.loginfo("Received field response")
        transform = Transform3D.from_pose_msg(field.pose.pose).inverse()

        extents = XYZ.from_msg(field.size)
        passes = self.extents_range[0] < extents < self.extents_range[1]

        if not passes:
            rospy.logwarn(f"Field size does not match expected size: Got {extents}, expected {self.expected_size}")
            return

        self.estimated_field = EstimatedObject()
        self.estimated_field.size = field.size
        self.estimated_field.header = Header(frame_id=field.child_frame_id, stamp=field.header.stamp)
        self.estimated_field.child_frame_id = field.header.frame_id
        self.estimated_field.pose.pose = transform.to_pose_msg()
        self.estimated_field.label = Label.FIELD.value
        self.estimated_field_pub.publish(self.estimated_field)
        self.publish_field_markers(self.estimated_field)

    def get_cage_aligned_transform(self) -> Transform3D:
        if self.cage_corner is None:
            rospy.logwarn("No cage corner received. Assuming our team's corner is on the door side.")
            cage_corner = CageCorner.DOOR_SIDE
        else:
            cage_corner = self.cage_corner
        return self.field_rotations[cage_corner]

    def corner_side_callback(self, corner: RosCageCorner) -> None:
        rospy.loginfo("Cage corner set. Resetting filters.")
        self.estimated_field_pub.publish(self.estimated_field)
        self.cage_corner = CageCorner.from_msg(corner)
        self.corner_pub.publish(corner)

    def publish_field_tf(self, estimated_field: EstimatedObject) -> tf2_ros.TransformStamped:
        field_pose = estimated_field.pose.pose
        transform = Transform3D.from_pose_msg(field_pose)

        field_tf = tf2_ros.TransformStamped()
        field_tf.header.stamp = estimated_field.header.stamp
        field_tf.header.frame_id = estimated_field.header.frame_id
        field_tf.child_frame_id = estimated_field.child_frame_id
        field_tf.transform = transform.to_msg()
        return field_tf

    def publish_aligned_tf(self, estimated_field: EstimatedObject) -> tf2_ros.TransformStamped:
        transform = self.get_cage_aligned_transform()
        aligned_tf = tf2_ros.TransformStamped()
        aligned_tf.header.stamp = rospy.Time.now()
        aligned_tf.header.frame_id = self.map_frame
        aligned_tf.child_frame_id = estimated_field.header.frame_id
        aligned_tf.transform = transform.to_msg()
        return aligned_tf

    def publish_transforms(self) -> None:
        field_tf = self.publish_field_tf(self.estimated_field)
        aligned_tf = self.publish_aligned_tf(self.estimated_field)
        self.tf_broadcaster.sendTransform([field_tf, aligned_tf])

    def publish_field_markers(self, estimated_field: EstimatedObject) -> None:
        markers = [
            self.estimated_object_to_plane_marker(estimated_field, ColorRGBA(0, 1, 0.5, 0.25), 0),
            self.estimated_object_to_text_marker(estimated_field, "door side", ColorRGBA(1, 1, 1, 1), 1, (0, 1)),
            self.estimated_object_to_text_marker(estimated_field, "far side", ColorRGBA(1, 1, 1, 1), 2, (0, -1)),
        ]
        self.estimated_field_marker_pub.publish(MarkerArray(markers=markers))

    def estimated_object_to_plane_marker(
        self, estimated_object: EstimatedObject, color: ColorRGBA, id: int = 0
    ) -> Marker:
        marker = Marker()
        marker.header = Header(frame_id=self.map_frame, stamp=rospy.Time.now())
        marker.type = Marker.CUBE
        marker.ns = estimated_object.label
        marker.id = id
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.pose.orientation.w = 1.0
        marker.scale.x = estimated_object.size.x if estimated_object.size.x > 0 else 0.01
        marker.scale.y = estimated_object.size.y if estimated_object.size.y > 0 else 0.01
        marker.scale.z = estimated_object.size.z if estimated_object.size.z > 0 else 0.01
        marker.color = color
        return marker

    def estimated_object_to_text_marker(
        self,
        estimated_object: EstimatedObject,
        text: str,
        color: ColorRGBA,
        id: int = 0,
        relative: Tuple[float, float] = (1.0, 0.0),
        text_size: float = 0.1,
    ) -> Marker:
        marker = Marker()
        marker.header = Header(frame_id=self.map_frame, stamp=rospy.Time.now())
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = text
        marker.ns = estimated_object.label
        marker.id = id
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.pose.position.x = relative[0] * estimated_object.size.x * 0.5
        marker.pose.position.y = relative[1] * estimated_object.size.y * 0.5
        marker.scale.z = text_size
        marker.color = color
        return marker

    def run(self) -> None:
        while not rospy.is_shutdown():
            rospy.sleep(0.5)
            if len(self.estimated_field.header.frame_id) == 0:
                continue
            self.estimated_field.header.stamp = rospy.Time.now()
            self.publish_transforms()


if __name__ == "__main__":
    rospy.init_node("field_filter", log_level=rospy.DEBUG)
    field_filter = FieldFilter()
    field_filter.run()
