import copy
import logging
import math

import bw_interfaces.msg as bw_interfaces
import tf2_ros
from app.config.field_filter.global_field_manager_config import GlobalFieldManagerConfig
from bw_interfaces.msg import EstimatedObject
from bw_shared.configs.maps_config import MapConfig
from bw_shared.enums.cage_corner import CageCorner
from bw_shared.enums.field_type import FieldType
from bw_shared.enums.label import Label
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.xyz import XYZ
from bw_shared.messages.header import Header
from geometry_msgs.msg import Vector3
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header as RosHeader
from visualization_msgs.msg import Marker, MarkerArray


def make_extents_range(expected_size: XYZ, field_dims_buffer: float) -> tuple[XYZ, XYZ]:
    buffer_extents = XYZ(field_dims_buffer, field_dims_buffer, field_dims_buffer)
    return expected_size - buffer_extents, expected_size + buffer_extents


class GlobalFieldManager:
    def __init__(
        self,
        config: GlobalFieldManagerConfig,
        map_config: MapConfig,
        set_corner_side_sub: RosPollSubscriber[bw_interfaces.CageCorner],
        estimated_field_pub: RosPublisher[EstimatedObject],
        estimated_field_marker_pub: RosPublisher[MarkerArray],
        tf_broadcaster: tf2_ros.StaticTransformBroadcaster,
        latched_corner_pub: RosPublisher[bw_interfaces.CageCorner],
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.config = config
        self.map_config = map_config
        self.field_type = map_config.type

        # If we're on the floor, we don't care about field size
        self.should_check_field_size = self.field_type != FieldType.FLOOR
        if not self.should_check_field_size:
            self.logger.warning("Field size check is disabled.")

        self.expected_size = XYZ.from_size(map_config.size)
        self.extents_range = make_extents_range(self.expected_size, self.config.field_dims_buffer)

        self.map_frame = self.config.map_frame
        self.cage_corner: CageCorner | None = None
        self.field_rotations = {
            CageCorner.BLUE_SIDE: Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, -math.pi / 2))),
            CageCorner.RED_SIDE: Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, math.pi / 2))),
        }
        self.set_corner_side_sub = set_corner_side_sub
        self.latched_corner_pub = latched_corner_pub
        self.estimated_field_pub = estimated_field_pub
        self.estimated_field_marker_pub = estimated_field_marker_pub
        self.tf_broadcaster = tf_broadcaster

    def process_field(self, field: EstimatedObject) -> EstimatedObject | None:
        tf_camera_from_relativemap = Transform3D.from_pose_msg(field.pose.pose).inverse()

        extents = XYZ.from_msg(field.size)
        passes = (not self.should_check_field_size) or self.extents_range[0] < extents < self.extents_range[1]

        if not passes:
            self.logger.warning(
                f"Field size does not match expected size: Got {extents}, expected {self.expected_size}"
            )
            return None

        relative_field = EstimatedObject()
        relative_field.size = field.size
        relative_field.header = RosHeader(frame_id=field.child_frame_id, stamp=field.header.stamp)
        relative_field.child_frame_id = field.header.frame_id
        relative_field.pose.pose = tf_camera_from_relativemap.to_pose_msg()
        relative_field.label = Label.FIELD.value

        tf_map_from_relativemap = self.get_cage_aligned_transform()
        aligned_field = copy.deepcopy(relative_field)
        aligned_field.pose.pose = tf_map_from_relativemap.transform_by(tf_camera_from_relativemap).to_pose_msg()
        aligned_field.header.frame_id = self.map_frame

        self.logger.info("Publishing field")
        self.estimated_field_pub.publish(aligned_field)
        self.publish_field_markers(aligned_field)
        self.publish_transforms(relative_field)
        return aligned_field

    def get_cage_aligned_transform(self) -> Transform3D:
        if msg := self.set_corner_side_sub.receive():
            self.cage_corner = CageCorner.from_msg(msg)
            self.latched_corner_pub.publish(msg)
        if self.cage_corner is None:
            self.logger.info("No cage corner received. Assuming our team's corner is on the blue side.")
            cage_corner = CageCorner.BLUE_SIDE
        else:
            cage_corner = self.cage_corner
        return self.field_rotations[cage_corner]

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
        marker.header = Header.auto(frame_id=self.map_frame).to_msg()
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
        relative: tuple[float, float] = (1.0, 0.0),
        text_size: float = 0.1,
    ) -> Marker:
        marker = Marker()
        marker.header = Header.auto(frame_id=self.map_frame).to_msg()
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
        aligned_tf.header.stamp = estimated_field.header.stamp
        aligned_tf.header.frame_id = self.map_frame
        aligned_tf.child_frame_id = estimated_field.header.frame_id
        aligned_tf.transform = transform.to_msg()
        return aligned_tf

    def publish_transforms(self, relative_field: EstimatedObject) -> None:
        tfs = [self.get_field_tf(relative_field), self.get_aligned_tf(relative_field)]
        self.tf_broadcaster.sendTransform(tfs)
