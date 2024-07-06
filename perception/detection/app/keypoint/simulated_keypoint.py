import logging

import cv2
import numpy as np
from app.config.keypoint_config.simulated_keypoint_config import SimulatedKeypointConfig
from app.keypoint.ground_truth_manager import GroundTruthManager
from app.keypoint.keypoint_interface import KeypointInterface
from bw_interfaces.msg import EstimatedObject, KeypointInstance, KeypointInstanceArray, UVKeypoint
from bw_shared.enums.keypoint_name import RobotKeypointsNames
from bw_shared.enums.label import Label
from bw_shared.geometry.transform3d import Transform3D
from image_geometry import PinholeCameraModel
from perception_tools.messages.image import Image
from sensor_msgs.msg import CameraInfo


class SimulatedKeypoint(KeypointInterface):
    def __init__(self, config: SimulatedKeypointConfig, ground_truth_manager: GroundTruthManager) -> None:
        self.ground_truth_manager = ground_truth_manager
        self.config = config
        self.logger = logging.getLogger("perception")
        self.debug = self.config.debug
        self.model: PinholeCameraModel | None = None
        self.keypoint_names = [name.value for name in RobotKeypointsNames]

        self.model_to_system_labels = {
            model_label: Label(real_label) for model_label, real_label in self.config.model_to_system_labels.items()
        }
        if len(self.model_to_system_labels) == 0:
            self.logger.warning("No simulated to real label mapping provided")
        self.logger.info(f"Simulated to real label mapping: {self.model_to_system_labels}")
        self.real_model_labels = tuple(Label)

    def process_image(self, camera_info: CameraInfo, rgb_image: Image) -> tuple[KeypointInstanceArray, Image | None]:
        robots = self.ground_truth_manager.get_robots()
        if self.model is None:
            self.model = PinholeCameraModel()
            self.model.fromCameraInfo(camera_info)
            self.logger.info("Camera model loaded")
        if robots is None or self.model is None:
            return KeypointInstanceArray(), None

        array = KeypointInstanceArray(header=camera_info.header, height=camera_info.height, width=camera_info.width)
        debug_image = Image.from_other(rgb_image) if self.debug else None
        object_counts = {label: 0 for label in self.real_model_labels}
        for robot in robots:
            instance = self.odometry_to_keypoint(self.model, debug_image, robot, object_counts)
            if instance is None:
                self.logger.warning("Failed to convert odometry to keypoint instance")
                continue
            array.instances.append(instance)
        return array, debug_image

    def odometry_to_keypoint(
        self,
        model: PinholeCameraModel,
        debug_image: Image | None,
        robot: EstimatedObject,
        object_counts: dict[Label, int],
    ) -> KeypointInstance | None:
        """
        Convert robot estimated object to keypoint instance.
        Using the size field, project forward and backward in robot space.
        Compute these two points in camera world space.
        Project these two points to pixel space.
        Fill the keypoint instance with the pixel coordinates.
        """
        label = self.model_to_system_labels[robot.child_frame_id]
        radius = max(robot.size.x, robot.size.y) / 2
        tf_camera_from_robot = Transform3D.from_pose_msg(robot.pose.pose)
        pos_robotcenter_to_robotfront = np.array([0, radius, 0, 1])
        pos_robotcenter_to_robotback = np.array([0, -radius, 0, 1])
        forward_pixel = self.robot_point_to_camera_pixel(tf_camera_from_robot, pos_robotcenter_to_robotfront, model)
        backward_pixel = self.robot_point_to_camera_pixel(tf_camera_from_robot, pos_robotcenter_to_robotback, model)
        object_index = object_counts[label]
        object_counts[label] += 1
        if forward_pixel is None or backward_pixel is None:
            return None

        if debug_image:
            cv2.line(
                debug_image.data,
                (int(forward_pixel.x), int(forward_pixel.y)),
                (int(backward_pixel.x), int(backward_pixel.y)),
                (128, 128, 128),
                2,
            )
            cv2.circle(debug_image.data, (int(forward_pixel.x), int(forward_pixel.y)), 5, (0, 0, 255), -1)
            cv2.circle(debug_image.data, (int(backward_pixel.x), int(backward_pixel.y)), 5, (255, 0, 0), -1)
            cv2.putText(
                debug_image.data,
                f"{label.value}_{object_index}",
                (int(forward_pixel.x), int(forward_pixel.y)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

        return KeypointInstance(
            keypoints=[forward_pixel, backward_pixel],
            names=self.keypoint_names,
            score=1.0,
            label=label,
            class_index=self.real_model_labels.index(label),
            object_index=object_index,
        )

    def robot_point_to_camera_pixel(
        self, tf_camera_from_robot: Transform3D, pos_robotcenter_to_robotpoint: np.ndarray, model: PinholeCameraModel
    ) -> UVKeypoint | None:
        pos_camera_to_robotpoint = np.dot(tf_camera_from_robot.tfmat, pos_robotcenter_to_robotpoint)
        pixel_robot_point = model.project3dToPixel(pos_camera_to_robotpoint[:3])
        if np.any(np.isnan(pixel_robot_point)):
            return None
        return UVKeypoint(x=pixel_robot_point[0], y=pixel_robot_point[1])
