import copy
import logging

import numpy as np
import rospy
from app.config.keypoint_config.keypoint_to_object_converter_config import KeypointToObjectConverterConfig
from bw_interfaces.msg import EstimatedObject, EstimatedObjectArray, KeypointInstance, KeypointInstanceArray
from bw_shared.configs.label_config import LabelsConfig
from bw_shared.enums.keypoint_name import KeypointName
from bw_shared.enums.label import Label
from bw_shared.epsilon import EPSILON
from bw_shared.geometry.array_conversions import xyz_to_array
from bw_shared.geometry.plane import Plane
from bw_shared.geometry.projection_math.rotation_matrix_from_vectors import transform_matrix_from_vectors
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.transform3d_stamped import Transform3DStamped
from geometry_msgs.msg import Point, Vector3
from image_geometry import PinholeCameraModel
from perception_tools.rosbridge.ros_publisher import RosPublisher
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header as RosHeader
from visualization_msgs.msg import Marker, MarkerArray


class KeypointToObjectConverter:
    def __init__(
        self,
        config: KeypointToObjectConverterConfig,
        labels: LabelsConfig,
        robot_pub: RosPublisher[EstimatedObjectArray],
        robot_marker_pub: RosPublisher[MarkerArray],
    ) -> None:
        self.config = config
        self.labels = labels
        self.robot_pub = robot_pub
        self.robot_marker_pub = robot_marker_pub
        self.logger = logging.getLogger(self.__class__.__name__)

        self.included_labels = self.config.included_labels
        if not self.included_labels:
            self.logger.info("No labels specified, including all labels.")

        self.camera_model: PinholeCameraModel | None = None
        self.camera_info = CameraInfo()

        self.point_marker_scale = (0.05, 0.05, 0.05)
        self.front_marker_color = (1.0, 0.0, 0.0, 0.75)
        self.back_marker_color = (0.0, 0.0, 1.0, 0.75)
        self.line_marker_scale = (0.01, 1.0, 1.0)
        self.line_marker_color = (0.5, 0.5, 0.5, 0.75)
        self.pose_marker_color = (0.0, 1.0, 0.0, 0.75)
        self.pose_marker_scale = (0.5, 0.05, 0.05)
        self.text_marker_color = (1.0, 1.0, 1.0, 0.75)
        self.text_marker_scale = (1.0, 1.0, 0.1)

    def convert_to_objects(
        self, camera_info: CameraInfo, keypoints: KeypointInstanceArray, tf_camera_from_map: Transform3DStamped
    ) -> EstimatedObjectArray | None:
        self._set_camera_info(camera_info)

        robot_array = EstimatedObjectArray()
        robot_markers = MarkerArray()
        robot_markers.markers = []
        field_plane_in_camera = Plane.from_transform(tf_camera_from_map.transform)

        for instance in keypoints.instances:
            if not self._is_label_included(instance.label):
                continue

            points = self._project_keypoints_to_field(instance, plane=field_plane_in_camera)
            back_index = self._get_index(instance.names, KeypointName.FRONT)
            front_index = self._get_index(instance.names, KeypointName.BACK)
            if front_index == -1 or back_index == -1:
                self.logger.debug(f"Keypoint not found in instance {instance.label}")
                continue
            front_point = xyz_to_array(points[front_index])
            back_point = xyz_to_array(points[back_index])
            transform = self._get_pose_from_points(front_point, back_point)
            if transform is None:
                continue

            length = float(np.linalg.norm(front_point - back_point))

            robot_msg = EstimatedObject()
            robot_msg.label = instance.label
            robot_msg.header = keypoints.header
            robot_msg.pose.pose = transform.to_pose_msg()
            robot_msg.size.x = length
            robot_msg.size.y = length
            robot_msg.size.z = self._get_label_height(Label(instance.label))
            robot_msg.score = instance.score
            robot_array.robots.append(robot_msg)

            robot_markers.markers.extend(self._make_markers(instance.object_index, robot_msg, front_point, back_point))

        self.robot_pub.publish(robot_array)
        self.robot_marker_pub.publish(robot_markers)

        return robot_array

    def _set_camera_info(self, camera_info: CameraInfo) -> None:
        if camera_info.header.frame_id == self.camera_info.header.frame_id:
            return
        self.logger.info("Camera model loaded")
        self.camera_info = camera_info
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info)

    def _is_label_included(self, label: str) -> bool:
        if not self.included_labels:
            return True
        for included_label in self.included_labels:
            if label == included_label:
                return True
        return False

    def _project_keypoints_to_field(self, instance: KeypointInstance, plane: Plane) -> list[Vector3]:
        points: list[Vector3] = []

        for keypoint in instance.keypoints:
            pixel = np.array([keypoint.x, keypoint.y], dtype=np.float64)
            center = self._project_to_field(pixel, plane)
            if center is None:
                self.logger.warning(f"Failed to project keypoint {keypoint} to field")
                return []
            points.append(center)
        return points

    def _get_index(self, names: list[str], name: str) -> int:
        try:
            return names.index(name)
        except ValueError:
            return -1

    def _get_pose_from_points(self, front_point: np.ndarray, back_point: np.ndarray) -> Transform3D | None:
        origin_vec = np.array([-1.0, 0.0, 0.0])

        # Calculate the direction vector from front_point to back_point

        offset_vector = front_point - back_point
        magnitude = np.linalg.norm(offset_vector)
        if magnitude <= EPSILON:
            return None
        direction = offset_vector / magnitude

        # Calculate the rotation from the plane normal to the direction vector
        center = (front_point + back_point) / 2.0
        transform = transform_matrix_from_vectors(center, direction, origin_vec)
        return transform

    def _get_label_height(self, label: Label) -> float:
        return self.labels.labels_map[label].height

    def _make_markers(
        self,
        object_index: int,
        robot_msg: EstimatedObject,
        front_point: np.ndarray,
        back_point: np.ndarray,
    ) -> list[Marker]:
        front_point_msg = self._array_to_point(front_point)
        back_point_msg = self._array_to_point(back_point)
        front_marker = self._make_keypoint_marker(
            robot_msg.header,
            robot_msg.label + "_front",
            object_index,
            self.point_marker_scale,
            self.front_marker_color,
        )
        front_marker.pose.position = front_point_msg
        front_marker.type = Marker.SPHERE

        back_marker = self._make_keypoint_marker(
            robot_msg.header,
            robot_msg.label + "_back",
            object_index,
            self.point_marker_scale,
            self.back_marker_color,
        )
        back_marker.pose.position = back_point_msg
        back_marker.type = Marker.SPHERE

        line_marker = self._make_keypoint_marker(
            robot_msg.header,
            robot_msg.label + "_line",
            object_index,
            self.line_marker_scale,
            self.line_marker_color,
        )
        line_marker.points = [front_point_msg, back_point_msg]
        line_marker.type = Marker.LINE_LIST

        pose_marker = self._make_keypoint_marker(
            robot_msg.header,
            robot_msg.label + "_pose",
            object_index,
            self.pose_marker_scale,
            self.pose_marker_color,
        )
        pose_marker.pose = robot_msg.pose.pose
        pose_marker.type = Marker.ARROW

        text_marker = self._make_keypoint_marker(
            robot_msg.header,
            robot_msg.label + "_text",
            object_index,
            self.text_marker_scale,
            self.text_marker_color,
        )
        text_marker.pose = copy.deepcopy(robot_msg.pose.pose)
        text_marker.pose.position.y -= 0.1
        text_marker.pose.position.z -= 0.1
        text_marker.text = f"{robot_msg.label}_{object_index}"
        text_marker.type = Marker.TEXT_VIEW_FACING

        return [front_marker, back_marker, line_marker, pose_marker, text_marker]

    def _array_to_point(self, array: np.ndarray) -> Point:
        return Point(x=array[0], y=array[1], z=array[2])

    def _make_keypoint_marker(
        self,
        header: RosHeader,
        namespace: str,
        object_index: int,
        scale: tuple[float, float, float],
        color: tuple[float, float, float, float],
    ) -> Marker:
        return Marker(
            header=header,
            ns=namespace,
            id=object_index,
            frame_locked=False,
            lifetime=rospy.Duration.from_sec(1.0),
            action=Marker.ADD,
            scale=Vector3(x=scale[0], y=scale[1], z=scale[2]),
            color=ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3]),
        )

    def _project_to_field(self, pixel: np.ndarray, plane: Plane) -> Vector3 | None:
        if self.camera_model is None:
            self.logger.warning("Camera model not set")
            return None
        root_vector = Vector3(*self.camera_model.projectPixelTo3dRay(pixel))
        return plane.ray_intersection(root_vector)
