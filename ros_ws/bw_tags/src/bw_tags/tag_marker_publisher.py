#!/usr/bin/env python
import copy
from typing import List, Optional, Tuple

import rospy
import tf2_geometry_msgs
import tf2_ros
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from bw_tools.get_param import get_param
from bw_tools.structs.transform3d import Transform3D
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException  # type: ignore
from visualization_msgs.msg import Marker, MarkerArray


class TagMarkerPublisher:
    def __init__(self):
        self.name = "tag_marker_publisher"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)
        self.marker_publish_rate = get_param("~marker_publish_rate", 0.0)
        self.debug = get_param("~debug", False)
        self.base_frame = get_param("~base_frame", "")
        self.stale_detection_seconds = rospy.Duration.from_sec(get_param("~stale_detection_seconds", 1.0))

        self.tag_msg = AprilTagDetectionArray()
        self.rotate_quat = (0.5, -0.5, -0.5, -0.5)

        self.marker_colors = {None: (1.0, 1.0, 1.0, 1.0)}

        self.tag_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10)
        self.marker_pub = rospy.Publisher("tag_markers", MarkerArray, queue_size=10)
        if self.debug:
            self.rotated_debug_pub = rospy.Publisher("rotated_debug", PoseArray, queue_size=10)
        else:
            self.rotated_debug_pub = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("%s init complete" % self.name)

    def tag_callback(self, msg: AprilTagDetectionArray):
        base_detections = AprilTagDetectionArray()
        assert msg.detections is not None and base_detections.detections is not None
        for detection in msg.detections:
            pose: Pose = detection.pose.pose.pose
            detection.pose.pose.pose.orientation = self.rotate_tag_orientation(pose.orientation, self.rotate_quat)
            new_detection = self.transform_tag_to_base(detection)
            if new_detection is None:
                continue
            base_detections.header = new_detection.pose.header
            base_detections.detections.append(new_detection)

        if len(base_detections.detections) != 0:
            self.tag_msg = base_detections
        if self.debug:
            self.publish_debug_rotation(self.tag_msg)
        if self.marker_publish_rate <= 0.0:
            self.publish_marker(self.tag_msg)

    def publish_debug_rotation(self, msg: AprilTagDetectionArray):
        if self.rotated_debug_pub is None:
            return
        pose_array = PoseArray()
        assert pose_array.poses is not None
        pose_array.header = msg.header
        assert msg.detections is not None
        for detection in msg.detections:
            pose: Pose = detection.pose.pose.pose
            pose_array.poses.append(pose)
        self.rotated_debug_pub.publish(pose_array)

    def transform_tag_to_base(self, detection: AprilTagDetection) -> Optional[AprilTagDetection]:
        source_frame = detection.pose.header.frame_id
        if len(self.base_frame) == 0 or self.base_frame == source_frame:
            return detection
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                rospy.Time(0),
                self.stale_detection_seconds,
            )
        except (
            LookupException,
            ConnectivityException,
            ExtrapolationException,
        ) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (self.base_frame, source_frame, e))
            return None
        tag_pose_stamped = PoseStamped()
        tag_pose_stamped.header = detection.pose.header
        tag_pose_stamped.pose = detection.pose.pose.pose

        base_pose_stamped = tf2_geometry_msgs.do_transform_pose(tag_pose_stamped, transform)

        new_detection = copy.deepcopy(detection)
        new_detection.pose.pose.pose = base_pose_stamped.pose
        new_detection.pose.header = base_pose_stamped.header

        return new_detection

    def rotate_tag_orientation(
        self,
        tag_orientation: Quaternion,
        rotate_quat: Tuple[float, float, float, float],
    ) -> Quaternion:
        rotate_tf = Transform3D.from_position_and_quaternion(Vector3(), Quaternion(*rotate_quat))
        tag_tf = Transform3D.from_position_and_quaternion(Vector3(), tag_orientation)
        rotated_tag = tag_tf.transform_by(rotate_tf)
        return rotated_tag.quaternion

    def publish_marker(self, msg: AprilTagDetectionArray):
        markers = MarkerArray()
        assert msg.detections is not None
        for detection in msg.detections:
            pose: Pose = detection.pose.pose.pose
            header = detection.pose.header
            tag_id: List[int] = detection.id
            size = 0.0
            for tag_size in detection.size:
                size += tag_size
            name = "-".join([str(sub_id) for sub_id in tag_id])
            pose_stamped = PoseStamped()
            pose_stamped.header = header
            pose_stamped.pose = pose

            if name in self.marker_colors:
                marker_color = self.marker_colors[name]
            else:
                marker_color = self.marker_colors[None]
            x_position_marker = self.make_marker(name, pose_stamped, marker_color, size)
            y_position_marker = self.make_marker(name, pose_stamped, marker_color, size)
            z_position_marker = self.make_marker(name, pose_stamped, marker_color, size)
            text_marker = self.make_marker(name, pose_stamped, marker_color, size)
            square_marker = self.make_marker(name, pose_stamped, marker_color, size)

            self.prep_position_marker("x", x_position_marker, None, size, (1.0, 0.0, 0.0))
            self.prep_position_marker(
                "y",
                y_position_marker,
                (0.0000, 0.0000, 0.7071, 0.7071),
                size,
                (0.0, 1.0, 0.0),
            )
            self.prep_position_marker(
                "z",
                z_position_marker,
                (0.0000, -0.7071, 0.0000, 0.7071),
                size,
                (0.0, 0.0, 1.0),
            )
            self.prep_text_marker(text_marker, name)
            self.prep_square_marker(square_marker, size)

            markers.markers.extend(
                [
                    x_position_marker,
                    y_position_marker,
                    z_position_marker,
                    text_marker,
                    square_marker,
                ]
            )
        self.marker_pub.publish(markers)

    def prep_position_marker(self, name, position_marker, rotate_quat, size, color: Tuple[float, float, float]):
        position_marker.type = Marker.ARROW
        position_marker.ns = "pos" + name + position_marker.ns
        position_marker.color.r = color[0]
        position_marker.color.g = color[1]
        position_marker.color.b = color[2]
        position_marker.color.a = 0.75
        position_marker.scale.x = size / 4.0
        position_marker.scale.y = size / 2.5
        position_marker.scale.z = size / 2.0
        if rotate_quat is not None:
            position_marker.pose.orientation = self.rotate_tag_orientation(
                position_marker.pose.orientation, rotate_quat
            )

        p1 = Point()
        p2 = Point()

        p2.x = size

        position_marker.points.append(p1)
        position_marker.points.append(p2)

    def prep_text_marker(self, text_marker, name):
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.ns = "text" + text_marker.ns
        text_marker.text = name
        text_marker.scale.x = 0.0
        text_marker.scale.y = 0.0
        text_marker.color = ColorRGBA(
            r=1.0,
            g=1.0,
            b=1.0,
            a=1.0,
        )

    def prep_square_marker(self, square_marker, tag_size):
        square_marker.type = Marker.CUBE
        square_marker.ns = "cube" + square_marker.ns
        square_marker.scale.x = 0.001
        square_marker.scale.y = tag_size
        square_marker.scale.z = tag_size

    def make_marker(self, name, pose, color, size):
        # name: str, marker name
        # pose: PoseStamped
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(pose.pose)
        marker.header = pose.header
        if self.marker_publish_rate > 0.0:
            marker.lifetime = rospy.Duration.from_sec(2.0 / self.marker_publish_rate)
        else:
            marker.lifetime = rospy.Duration.from_sec(0.0)
        marker.ns = name
        marker.id = 0  # all waypoint names should be unique
        marker.frame_locked = False

        scale_vector = Vector3()
        scale_vector.x = size
        scale_vector.y = size
        scale_vector.z = size
        marker.scale = scale_vector
        marker.color = ColorRGBA(
            r=color[0],
            g=color[1],
            b=color[2],
            a=color[3],
        )

        return marker

    def run(self):
        if self.marker_publish_rate > 0.0:
            rate = rospy.Rate(self.marker_publish_rate)
            while not rospy.is_shutdown():
                self.publish_marker(self.tag_msg)
                rate.sleep()
        else:
            rospy.spin()


if __name__ == "__main__":
    node = TagMarkerPublisher()
    node.run()
