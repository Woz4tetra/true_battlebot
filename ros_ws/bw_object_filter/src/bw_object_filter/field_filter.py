#!/usr/bin/env python
import copy
import math
from typing import Optional, Tuple

import rospy
import tf2_ros
from bw_interfaces.msg import CageCorner as RosCageCorner
from bw_interfaces.msg import EstimatedObject, Heartbeat
from bw_shared.enums.field_type import FieldType
from bw_shared.enums.label import Label
from bw_shared.environment import get_map
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.xyz import XYZ
from bw_tools.configs.rosparam_client import get_shared_config
from bw_tools.get_param import get_param
from bw_tools.messages.cage_corner import CageCorner
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA, Empty, Header
from visualization_msgs.msg import Marker, MarkerArray


class FieldFilter:
    def __init__(self) -> None:
        shared_config = get_shared_config()
        map_name = get_map()
        self.field_type = FieldType(map_name)
        map_config = shared_config.get_map(self.field_type)

        self.map_frame = get_param("~map_frame", "map")
        self.relative_map_frame = get_param("~relative_map_frame", "map_relative")
        auto_initialize = get_param("~auto_initialize", False)
        self.check_field_size = self.field_type != FieldType.FLOOR
        self.expected_size = XYZ.from_size(map_config.size)
        field_dims_buffer = get_param("~field_dims_buffer", 0.35)
        buffer_extents = XYZ(field_dims_buffer, field_dims_buffer, field_dims_buffer)
        self.extents_range = (self.expected_size - buffer_extents, self.expected_size + buffer_extents)
        rospy.loginfo(f"Map name: {map_name}")
        rospy.loginfo(f"Extents range: {self.extents_range}")
        if not self.check_field_size:
            rospy.logwarn("Field size check is disabled.")

        self.cage_corner: Optional[CageCorner] = None
        self.field_rotations = {
            CageCorner.BLUE_SIDE: Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, -math.pi / 2))),
            CageCorner.RED_SIDE: Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, math.pi / 2))),
        }
        self.estimated_field = EstimatedObject()
        self.field_rotate_tf = self.field_rotations[CageCorner.BLUE_SIDE]
        self.perception_heartbeat = Heartbeat()
        self.field_request_pending = False

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.request_pub = rospy.Publisher("/perception/field/request", Empty, queue_size=1)
        self.estimated_field_pub = rospy.Publisher("filter/field", EstimatedObject, queue_size=1, latch=True)
        self.estimated_field_marker_pub = rospy.Publisher("filter/field/marker", MarkerArray, queue_size=1, latch=True)
        self.corner_pub = rospy.Publisher("cage_corner", RosCageCorner, queue_size=1, latch=True)

        self.manual_request_sub = rospy.Subscriber(
            "manual_plane_request", Empty, self.manual_request_callback, queue_size=1
        )
        self.corner_side_sub = rospy.Subscriber(
            "set_cage_corner", RosCageCorner, self.corner_side_callback, queue_size=1
        )
        self.response_sub = rospy.Subscriber("/perception/field/response", EstimatedObject, self.response_callback)
        self.perception_heartbeat_sub = rospy.Subscriber(
            "/perception/heartbeat", Heartbeat, self.perception_header, queue_size=1
        )

        if auto_initialize:
            self.request_field()

    def manual_request_callback(self, _: Empty) -> None:
        rospy.loginfo("Manual plane request received")
        self.request_field()

    def perception_header(self, heartbeat: Heartbeat) -> None:
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

    def response_callback(self, field: EstimatedObject) -> None:
        rospy.loginfo("Received field response")
        tf_camera_from_relativemap = Transform3D.from_pose_msg(field.pose.pose).inverse()

        extents = XYZ.from_msg(field.size)
        passes = (not self.check_field_size) or self.extents_range[0] < extents < self.extents_range[1]

        if not passes:
            rospy.logwarn(f"Field size does not match expected size: Got {extents}, expected {self.expected_size}")
            return

        self.estimated_field = EstimatedObject()
        self.estimated_field.size = field.size
        self.estimated_field.header = Header(frame_id=field.child_frame_id, stamp=field.header.stamp)
        self.estimated_field.child_frame_id = field.header.frame_id
        self.estimated_field.pose.pose = tf_camera_from_relativemap.to_pose_msg()
        self.estimated_field.label = Label.FIELD.value

        tf_map_from_relativemap = self.get_cage_aligned_transform()
        aligned_field = copy.deepcopy(self.estimated_field)
        aligned_field.pose.pose = tf_map_from_relativemap.forward_by(tf_camera_from_relativemap).to_pose_msg()
        aligned_field.header.frame_id = self.map_frame

        rospy.loginfo("Publishing field")
        self.estimated_field_pub.publish(aligned_field)
        self.publish_field_markers(aligned_field)

    def get_cage_aligned_transform(self) -> Transform3D:
        if self.cage_corner is None:
            rospy.loginfo("No cage corner received. Assuming our team's corner is on the blue side.")
            cage_corner = CageCorner.BLUE_SIDE
        else:
            cage_corner = self.cage_corner
        return self.field_rotations[cage_corner]

    def corner_side_callback(self, corner: RosCageCorner) -> None:
        rospy.loginfo("Cage corner set.")
        self.cage_corner = CageCorner.from_msg(corner)
        self.corner_pub.publish(corner)

    def get_field_tf(self, estimated_field: EstimatedObject) -> tf2_ros.TransformStamped:
        field_pose = estimated_field.pose.pose
        transform = Transform3D.from_pose_msg(field_pose)

        field_tf = tf2_ros.TransformStamped()
        field_tf.header.stamp = estimated_field.header.stamp
        field_tf.header.frame_id = estimated_field.header.frame_id
        field_tf.child_frame_id = estimated_field.child_frame_id
        field_tf.transform = transform.to_msg()
        return field_tf

    def get_aligned_tf(self, estimated_field: EstimatedObject) -> tf2_ros.TransformStamped:
        transform = self.get_cage_aligned_transform()
        aligned_tf = tf2_ros.TransformStamped()
        aligned_tf.header.stamp = rospy.Time.now()
        aligned_tf.header.frame_id = self.map_frame
        aligned_tf.child_frame_id = estimated_field.header.frame_id
        aligned_tf.transform = transform.to_msg()
        return aligned_tf

    def publish_transforms(self) -> None:
        field_tf = self.get_field_tf(self.estimated_field)
        aligned_tf = self.get_aligned_tf(self.estimated_field)
        self.tf_broadcaster.sendTransform([field_tf, aligned_tf])

    def publish_field_markers(self, estimated_field: EstimatedObject) -> None:
        markers = [
            self.estimated_object_to_plane_marker(estimated_field, ColorRGBA(0, 1, 0.5, 0.25), 0),
            self.estimated_object_to_text_marker(estimated_field, "blue", ColorRGBA(0, 0, 1, 1), 1, (-1, 1)),
            self.estimated_object_to_text_marker(estimated_field, "red", ColorRGBA(1, 0, 0, 1), 2, (1, -1)),
            self.estimated_object_to_text_marker(estimated_field, "door", ColorRGBA(1, 1, 1, 1), 3, (0, 1)),
            self.estimated_object_to_text_marker(estimated_field, "audience", ColorRGBA(1, 1, 1, 1), 4, (0, -1)),
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
            self.check_pending_field_request()
            if len(self.estimated_field.header.frame_id) == 0:
                continue
            self.estimated_field.header.stamp = rospy.Time.now()
            self.publish_transforms()


if __name__ == "__main__":
    rospy.init_node("field_filter", log_level=rospy.DEBUG)
    field_filter = FieldFilter()
    field_filter.run()
