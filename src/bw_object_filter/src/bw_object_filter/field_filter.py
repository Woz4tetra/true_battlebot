#!/usr/bin/env python
import math
from typing import Optional

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
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Vector3
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Imu
from std_msgs.msg import ColorRGBA, Empty
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
        camera_is_situated_on_start = get_param("~camera_is_situated_on_start", False)

        self.has_manual_query_been_received = camera_is_situated_on_start
        self.has_manual_query = camera_is_situated_on_start
        self.current_imu = Imu()
        self.prev_imu = Imu()
        self.current_imu.orientation.w = 1.0
        self.prev_imu.orientation.w = 1.0
        self.cage_corner: Optional[CageCorner] = None
        self.field_rotations = {
            CageCorner.DOOR_SIDE: Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, math.pi / 2))),
            CageCorner.FAR_SIDE: Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, -math.pi / 2))),
        }
        self.estimated_field = EstimatedObject()

        self.tf_buffer = tf2_ros.Buffer()  # type: ignore
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)  # type: ignore
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()  # type: ignore

        self.field_rotate_tf = Transform3D.from_position_and_rpy(Vector3(), RPY((0, -np.pi / 2, 0)))

        self.marker_color = ColorRGBA(0, 1, 0.5, 0.75)
        self.prev_request_time = rospy.Time(0)
        self.segmentation = SegmentationInstanceArray()
        self.camera_model = PinholeCameraModel()

        self.plane_request_pub = rospy.Publisher("plane_request", PointStamped, queue_size=1)
        self.plane_response_sub = rospy.Subscriber("plane_response", PlaneStamped, self.plane_response_callback)
        self.estimated_field_pub = rospy.Publisher("filter/field", EstimatedObject, queue_size=1, latch=True)
        self.estimated_field_marker_pub = rospy.Publisher("filter/field/marker", MarkerArray, queue_size=1, latch=True)
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
        # angle = get_rectangle_angle(min_rect) % np.pi
        centroid = np.mean(min_rect, axis=0)
        field_centered_plane = Transform3D.from_position_and_rpy(
            Vector3(centroid[0], centroid[1], 0), RPY((0, 0, 0))
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
        transform = Transform3D.from_position_and_quaternion(
            Vector3(
                x=field_pose.position.x,
                y=field_pose.position.y,
                z=field_pose.position.z,
            ),
            field_pose.orientation,
        )
        transform = transform.inverse()

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
        self.estimated_field_marker_pub.publish(
            MarkerArray(markers=[self.estimated_object_to_marker(estimated_field, self.marker_color, 0, Marker.CUBE)])
        )

    def estimated_object_to_marker(
        self, estimated_object: EstimatedObject, color: ColorRGBA, id: int = 0, type: int = Marker.ARROW
    ) -> Marker:
        marker = Marker()
        marker.header = estimated_object.header
        marker.type = type
        marker.ns = estimated_object.label
        marker.id = id
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose = estimated_object.state.pose.pose
        marker.scale.x = estimated_object.size.x if estimated_object.size.x > 0 else 0.01
        marker.scale.y = estimated_object.size.y if estimated_object.size.y > 0 else 0.01
        marker.scale.z = estimated_object.size.z if estimated_object.size.z > 0 else 0.01
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
