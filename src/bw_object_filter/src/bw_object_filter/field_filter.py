#!/usr/bin/env python
import copy
import math
from typing import Optional, Tuple

import numpy as np
import rospy
import tf2_ros
from bw_interfaces.msg import CageCorner as RosCageCorner
from bw_interfaces.msg import EstimatedObject, SegmentationInstanceArray
from bw_tools.structs.cage_corner import CageCorner
from bw_tools.structs.rpy import RPY
from bw_tools.structs.transform3d import Transform3D
from bw_tools.transforms import lookup_pose_in_frame
from bw_tools.typing import get_param
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion, Vector3
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Imu
from std_msgs.msg import ColorRGBA, Empty, Header
from visualization_msgs.msg import Marker, MarkerArray
from zed_interfaces.msg import PlaneStamped

from bw_object_filter.field_math.find_minimum_rectangle import (
    find_minimum_rectangle,
    get_rectangle_angle,
    get_rectangle_extents,
)
from bw_object_filter.field_math.get_field_segmentation import get_field_segmentation
from bw_object_filter.field_math.points_transform import points_transform
from bw_object_filter.field_math.project_segmentation import project_segmentation, raycast_segmentation


class FieldFilter:
    def __init__(self) -> None:
        self.angle_delta_threshold = math.radians(get_param("angle_delta_threshold_degrees", 3.0))
        self.base_frame = get_param("~base_frame", "camera")
        self.map_frame = get_param("~map_frame", "map")
        self.relative_map_frame = get_param("~relative_map_frame", "map_relative")
        auto_initialize = get_param("~auto_initialize", False)

        self.has_manual_query_been_received = auto_initialize
        self.has_manual_query = auto_initialize
        self.current_imu = Imu()
        self.prev_imu = Imu()
        self.current_imu.orientation.w = 1.0
        self.prev_imu.orientation.w = 1.0
        self.cage_corner: Optional[CageCorner] = None
        self.field_rotations = {
            CageCorner.DOOR_SIDE: Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, -math.pi / 2))),
            CageCorner.FAR_SIDE: Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, math.pi / 2))),
        }
        self.estimated_field = EstimatedObject()

        self.tf_buffer = tf2_ros.Buffer()  # type: ignore
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)  # type: ignore
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()  # type: ignore

        self.field_rotate_tf = Transform3D.from_position_and_rpy(Vector3(), RPY((0, -np.pi / 2, 0)))

        self.prev_request_time = rospy.Time(0)
        self.segmentation = SegmentationInstanceArray()
        self.camera_model = PinholeCameraModel()

        self.plane_request_pub = rospy.Publisher("plane_request", PointStamped, queue_size=1)
        self.plane_response_sub = rospy.Subscriber("plane_response", PlaneStamped, self.plane_response_callback)
        self.estimated_field_pub = rospy.Publisher("filter/field", EstimatedObject, queue_size=1, latch=True)
        self.estimated_field_marker_pub = rospy.Publisher("filter/field/marker", MarkerArray, queue_size=1, latch=True)
        self.debug_marker_pub = rospy.Publisher("filter/field/debug", MarkerArray, queue_size=1, latch=True)
        self.corner_pub = rospy.Publisher("cage_corner", RosCageCorner, queue_size=1, latch=True)
        self.reset_filters_pub = rospy.Publisher("reset_filters", Empty, queue_size=1)

        self.recommended_point_sub = rospy.Subscriber(
            "estimation/recommended_field_point", PointStamped, self.recommended_point_callback
        )
        self.camera_tilt_sub = rospy.Subscriber("imu", Imu, self.imu_callback, queue_size=1)
        self.manual_request_sub = rospy.Subscriber(
            "manual_plane_request", Empty, self.manual_request_callback, queue_size=1
        )
        self.corner_side_sub = rospy.Subscriber(
            "set_cage_corner", RosCageCorner, self.corner_side_callback, queue_size=1
        )
        self.segmentation_sub = rospy.Subscriber(
            "segmentation", SegmentationInstanceArray, self.segmentation_callback, queue_size=1
        )
        self.camera_info_sub = rospy.Subscriber("camera_info", CameraInfo, self.camera_info_callback, queue_size=1)

    def segmentation_callback(self, segmentation: SegmentationInstanceArray) -> None:
        self.segmentation = segmentation

    def camera_info_callback(self, camera_info: CameraInfo) -> None:
        rospy.loginfo("Received camera info")
        self.camera_model.fromCameraInfo(camera_info)
        self.camera_info_sub.unregister()

    def recommended_point_callback(self, point: PointStamped) -> None:
        if not self.has_manual_query_been_received:
            return
        if self.has_manual_query:
            rospy.loginfo("Manual plane request received")
            self.has_manual_query = False
            should_request = True
        else:
            should_request = self.did_camera_tilt()
            if should_request:
                rospy.loginfo("Camera tilt detected.")
        if should_request:
            self.reset_camera_tilt_detection()
            self.publish_plane_request(point)
            self.prev_request_time = rospy.Time.now()

    def manual_request_callback(self, _: Empty) -> None:
        self.has_manual_query = True
        self.has_manual_query_been_received = True

    def publish_plane_request(self, request_point: PointStamped) -> None:
        rospy.loginfo("Requesting plane")
        self.plane_request_pub.publish(request_point)

    def plane_response_callback(self, plane: PlaneStamped) -> None:
        field_segmentation = get_field_segmentation(self.segmentation)
        if field_segmentation is None:
            rospy.logwarn("No field segmentation received. Cannot estimate field.")
            return

        plane_pose = PoseStamped(header=plane.header)
        plane_pose.pose.position = plane.pose.translation
        plane_pose.pose.orientation = plane.pose.rotation
        lens_plane_pose = lookup_pose_in_frame(
            self.tf_buffer, plane_pose, self.segmentation.header.frame_id, timeout=rospy.Duration(30.0)
        )
        if lens_plane_pose is None:
            return
        lens_plane_pose.pose.orientation = self.rotate_field_orientation(
            lens_plane_pose.pose.orientation, self.field_rotate_tf
        )
        plane_transform = Transform3D.from_position_and_quaternion(
            lens_plane_pose.pose.position, lens_plane_pose.pose.orientation
        )

        plane_center = plane_transform.position_array
        plane_normal = plane_transform.rotation_matrix @ np.array((0, 0, 1))

        rays = raycast_segmentation(self.camera_model, field_segmentation)
        projected_points = project_segmentation(rays, plane_center, plane_normal)

        flattened_points = points_transform(projected_points, plane_transform.inverse().tfmat)
        flattened_points2d = flattened_points[:, :2]
        min_rect = find_minimum_rectangle(flattened_points2d)
        extents = get_rectangle_extents(min_rect)
        angle = get_rectangle_angle(min_rect)
        rospy.loginfo(f"Field angle: {angle}. Extents: {extents}")
        centroid = np.mean(min_rect, axis=0)
        field_centered_plane = Transform3D.from_position_and_rpy(
            Vector3(centroid[0], centroid[1], 0), RPY((0, 0, angle))
        ).forward_by(plane_transform)

        lens_field_pose = PoseStamped(header=self.segmentation.header, pose=field_centered_plane.to_pose_msg())
        base_field_pose = lookup_pose_in_frame(
            self.tf_buffer, lens_field_pose, self.base_frame, timeout=rospy.Duration(30.0)
        )
        if base_field_pose is None:
            return

        self.estimated_field = EstimatedObject(
            header=base_field_pose.header,
            size=Vector3(x=extents[0], y=extents[1]),
        )
        self.estimated_field.state.pose.pose = base_field_pose.pose
        self.estimated_field_pub.publish(self.estimated_field)
        self.publish_field_markers(self.estimated_field)
        self.publish_debug_markers(flattened_points, min_rect, centroid)
        self.reset_filters()

    def reset_filters(self) -> None:
        self.reset_filters_pub.publish(Empty())

    def rotate_field_orientation(
        self,
        field_orientation: Quaternion,
        rotate_tf: Transform3D,
    ) -> Quaternion:
        field_tf = Transform3D.from_position_and_quaternion(Vector3(), field_orientation)
        rotated_field = field_tf.transform_by(rotate_tf)
        return rotated_field.quaternion

    def get_cage_aligned_transform(self) -> Transform3D:
        if self.cage_corner is None:
            rospy.logwarn("No cage corner received. Assuming our team's corner is on the door side.")
            cage_corner = CageCorner.DOOR_SIDE
        else:
            cage_corner = self.cage_corner
        return self.field_rotations[cage_corner]

    def corner_side_callback(self, corner: RosCageCorner) -> None:
        rospy.loginfo("Cage corner set. Resetting filters.")
        self.reset_filters()
        self.cage_corner = CageCorner.from_msg(corner)
        self.corner_pub.publish(corner)

    def imu_callback(self, imu: Imu) -> None:
        self.current_imu = imu

    def did_camera_tilt(self) -> bool:
        delta_rpy = self.compute_delta_rpy(self.current_imu.orientation, self.prev_imu.orientation)
        return bool(np.any(np.abs(delta_rpy.to_array()) > self.angle_delta_threshold))

    def reset_camera_tilt_detection(self) -> None:
        self.prev_imu = self.current_imu

    def compute_delta_rpy(self, new_quat: Quaternion, old_quat: Quaternion) -> RPY:
        new_transform = Transform3D.from_position_and_quaternion(Vector3(0, 0, 0), new_quat)
        old_transform = Transform3D.from_position_and_quaternion(Vector3(0, 0, 0), old_quat)
        delta_transform = new_transform.transform_by(old_transform.inverse())
        return delta_transform.rpy

    def publish_field_tf(self, estimated_field: EstimatedObject) -> tf2_ros.TransformStamped:
        field_pose = estimated_field.state.pose.pose
        transform = Transform3D.from_pose_msg(field_pose).inverse()

        field_tf = tf2_ros.TransformStamped()  # type: ignore
        field_tf.header.stamp = estimated_field.header.stamp
        field_tf.header.frame_id = self.relative_map_frame
        field_tf.child_frame_id = estimated_field.header.frame_id
        field_tf.transform = transform.to_msg()
        return field_tf

    def publish_aligned_tf(self) -> tf2_ros.TransformStamped:
        transform = self.get_cage_aligned_transform()
        aligned_tf = tf2_ros.TransformStamped()  # type: ignore
        aligned_tf.header.stamp = rospy.Time.now()
        aligned_tf.header.frame_id = self.map_frame
        aligned_tf.child_frame_id = self.relative_map_frame
        aligned_tf.transform = transform.to_msg()
        return aligned_tf

    def publish_transforms(self) -> None:
        field_tf = self.publish_field_tf(self.estimated_field)
        aligned_tf = self.publish_aligned_tf()
        self.tf_broadcaster.sendTransform([field_tf, aligned_tf])

    def publish_field_markers(self, estimated_field: EstimatedObject) -> None:
        markers = [
            self.estimated_object_to_plane_marker(estimated_field, ColorRGBA(0, 1, 0.5, 0.25), 0),
            self.estimated_object_to_text_marker(estimated_field, "door side", ColorRGBA(1, 1, 1, 1), 1, (0, 1)),
            self.estimated_object_to_text_marker(estimated_field, "far side", ColorRGBA(1, 1, 1, 1), 2, (0, -1)),
        ]
        self.estimated_field_marker_pub.publish(MarkerArray(markers=markers))

    def publish_debug_markers(self, flattened_points: np.ndarray, min_rect: np.ndarray, centroid: np.ndarray) -> None:
        header = Header(frame_id=self.relative_map_frame, stamp=rospy.Time.now())
        self.debug_marker_pub.publish(
            MarkerArray(
                markers=[
                    self.polygon_to_marker(header, flattened_points, ColorRGBA(0, 0.5, 1, 1), 1),
                    self.polygon_to_marker(header, min_rect, ColorRGBA(1, 0, 0, 1), 2),
                    self.point_to_marker(header, (centroid[0], centroid[1], 0.0), ColorRGBA(1, 0, 0, 1), 3, 0.1),
                ]
            )
        )

    def polygon_to_marker(
        self,
        header: Header,
        points: np.ndarray,
        color: ColorRGBA,
        id: int = 0,
        line_width: float = 0.05,
        closed: bool = True,
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.type = Marker.LINE_STRIP
        marker.ns = "polygon"
        marker.id = id
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.scale.x = line_width
        marker.color = color
        if points.shape[1] == 3:
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
        else:
            marker.points = [Point(x=x, y=y, z=0) for x, y in points]
        if closed:
            marker.points.append(marker.points[0])
        return marker

    def point_to_marker(
        self,
        header: Header,
        point: Tuple[float, float, float],
        color: ColorRGBA,
        id: int = 0,
        size: float = 1.0,
        type: int = Marker.SPHERE,
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.type = type
        marker.ns = "polygon"
        marker.id = id
        marker.action = Marker.ADD
        marker.pose.position = Point(x=point[0], y=point[1], z=point[2])
        marker.pose.orientation.w = 1.0
        marker.frame_locked = False
        marker.scale = Vector3(x=size, y=size, z=size)
        marker.color = color
        return marker

    def estimated_object_to_plane_marker(
        self, estimated_object: EstimatedObject, color: ColorRGBA, id: int = 0
    ) -> Marker:
        marker = Marker()
        marker.header = estimated_object.header
        marker.type = Marker.CUBE
        marker.ns = estimated_object.label
        marker.id = id
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.pose = estimated_object.state.pose.pose
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
    rospy.init_node("field_filter")
    field_filter = FieldFilter()
    field_filter.run()
