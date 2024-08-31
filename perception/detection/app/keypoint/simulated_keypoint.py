import logging

import cv2
from app.config.keypoint_config.simulated_keypoint_config import SimulatedKeypointConfig
from app.keypoint.ground_truth_manager import GroundTruthManager
from app.keypoint.keypoint_interface import KeypointInterface
from bw_interfaces.msg import EstimatedObject, KeypointInstance, KeypointInstanceArray, LabelMap
from bw_shared.enums.keypoint_name import KeypointName
from bw_shared.enums.label import Label, ModelLabel
from bw_shared.geometry.projection_math.project_object_to_uv import ProjectionError, project_object_to_front_back_uv
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
        self.camera_info = CameraInfo()
        self.keypoint_names = [KeypointName.FRONT, KeypointName.BACK]
        self.model_labels = tuple(ModelLabel)
        self.system_labels = tuple(Label)

        self.model_to_system_labels = self.config.model_to_system_labels.labels
        if len(self.model_to_system_labels) == 0:
            self.logger.warning("No simulated to real label mapping provided")
        self.class_indices = self.config.model_to_system_labels.get_class_indices(self.model_labels)
        self.logger.info(f"Simulated to real label mapping: {self.model_to_system_labels}")

    def process_image(
        self, camera_info: CameraInfo, rgb_image: Image
    ) -> tuple[KeypointInstanceArray | None, Image | None]:
        robots = self.ground_truth_manager.get_robots()
        self.camera_info.header.stamp = camera_info.header.stamp
        self.camera_info.header.seq = camera_info.header.seq
        if self.camera_info != camera_info:
            self.model = PinholeCameraModel()
            self.model.fromCameraInfo(camera_info)
            self.logger.info(f"Camera model loaded: {camera_info}")
            self.camera_info = camera_info
        if self.model is None:
            self.logger.debug("Camera model not loaded")
            return None, None
        if robots is None:
            self.logger.debug("No robots found")
            return None, None

        array = KeypointInstanceArray(header=camera_info.header, height=camera_info.height, width=camera_info.width)
        debug_image = Image.from_other(rgb_image) if self.debug else None
        object_counts = {label: 0 for label in self.system_labels}
        for robot in robots:
            instance = self.odometry_to_keypoint(self.model, debug_image, robot, object_counts)
            if instance is None:
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
        label = self.model_to_system_labels[ModelLabel(robot.label)]
        object_index = object_counts[label]
        object_counts[label] += 1
        try:
            forward_pixel, backward_pixel = project_object_to_front_back_uv(robot, model)
        except ProjectionError as e:
            self.logger.warning(f"Projection error: {e}")
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
            names=self.keypoint_names,  # type: ignore
            score=1.0,
            label=label,
            class_index=self.class_indices[label],
            object_index=object_index,
        )

    def get_model_to_system_labels(self) -> LabelMap:
        return self.config.model_to_system_labels.to_msg()
