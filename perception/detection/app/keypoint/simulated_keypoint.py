import logging

import cv2
import numpy as np
from app.config.keypoint_config.simulated_keypoint_config import SimulatedKeypointConfig
from app.keypoint.ground_truth_manager import GroundTruthManager
from app.keypoint.keypoint_interface import KeypointInterface
from bw_interfaces.msg import KeypointInstance, KeypointInstanceArray, UVKeypoint
from bw_shared.enums.keypoint_name import RobotKeypointsNames
from bw_shared.enums.label import Label
from bw_shared.geometry.transform3d import Transform3D
from image_geometry import PinholeCameraModel
from nav_msgs.msg import Odometry
from perception_tools.messages.image import Image
from sensor_msgs.msg import CameraInfo


class SimulatedKeypoint(KeypointInterface):
    def __init__(self, config: SimulatedKeypointConfig, ground_truth_manager: GroundTruthManager) -> None:
        self.ground_truth_manager = ground_truth_manager
        self.config = config
        self.logger = logging.getLogger("perception")
        self.debug = self.config.debug
        self.model: PinholeCameraModel | None = None
        self.camera_info = CameraInfo()
        self.keypoint_names = [name.value for name in RobotKeypointsNames]

        self.simulated_to_real_labels = {
            sim_label: Label(real_label) for sim_label, real_label in self.config.simulated_to_real_labels.items()
        }
        if len(self.simulated_to_real_labels) == 0:
            self.logger.warning("No simulated to real label mapping provided")
        self.logger.info(f"Simulated to real label mapping: {self.simulated_to_real_labels}")
        self.real_model_labels = tuple(Label)

    def process_image(self, rgb_image: Image) -> tuple[KeypointInstanceArray, Image | None]:
        robots = self.ground_truth_manager.get_robots()
        if self.model is None and (camera_info := self.ground_truth_manager.get_camera_info()):
            self.model = PinholeCameraModel()
            self.model.fromCameraInfo(camera_info)
            self.camera_info = camera_info
        if robots is None or self.model is None:
            return KeypointInstanceArray(), None

        array = KeypointInstanceArray(
            header=self.camera_info.header, height=self.camera_info.height, width=self.camera_info.width
        )
        debug_image = Image.from_other(rgb_image) if self.debug else None
        object_counts = {label: 0 for label in self.real_model_labels}
        for odom in robots:
            instance = self.odometry_to_keypoint(self.model, debug_image, odom, object_counts)
            array.instances.append(instance)
        return array, debug_image

    def odometry_to_keypoint(
        self, model: PinholeCameraModel, debug_image: Image | None, odometry: Odometry, object_counts: dict[Label, int]
    ) -> KeypointInstance:
        """
        Convert odometry to keypoint instance.
        Using the assumed radius, project forward and backward in robot space.
        Compute these two points in camera world space.
        Project these two points to pixel space.
        Fill the keypoint instance with the pixel coordinates.
        """
        label = self.simulated_to_real_labels[odometry.child_frame_id]
        radius = self.config.radii.get(odometry.child_frame_id, self.config.default_radius)
        transform = Transform3D.from_pose_msg(odometry.pose.pose)
        forward_robot_point = np.array([radius, 0, 0, 1])
        backward_robot_point = np.array([-radius, 0, 0, 1])
        forward_pixel = self.robot_point_to_camera_pixel(transform, forward_robot_point, model)
        backward_pixel = self.robot_point_to_camera_pixel(transform, backward_robot_point, model)
        object_index = object_counts[label]
        object_counts[label] += 1

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
        self, camera_to_robot: Transform3D, robot_point: np.ndarray, model: PinholeCameraModel
    ) -> UVKeypoint:
        camera_point = np.dot(camera_to_robot.tfmat, robot_point)
        pixel = model.project3dToPixel(camera_point[:3])
        return UVKeypoint(x=pixel[0], y=pixel[1])
