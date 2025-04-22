import logging

import cv2
from app.config.keypoint_config.simulated_keypoint_config import SimulatedKeypointConfig
from app.keypoint.ground_truth_manager import GroundTruthManager
from app.keypoint.keypoint_interface import KeypointInterface
from bw_interfaces.msg import EstimatedObject, KeypointInstance, KeypointInstanceArray, LabelMap
from bw_shared.enums.keypoint_name import KeypointName
from bw_shared.enums.label import Label, ModelLabel
from bw_shared.geometry.projection_math.project_object_to_uv import ProjectionError, project_object_to_front_back_uv
from bw_shared.messages.field import Field
from image_geometry import PinholeCameraModel
from perception_tools.camera.camera_model_loader import CameraModelLoader
from perception_tools.messages.image import Image
from sensor_msgs.msg import CameraInfo


class SimulatedKeypoint(KeypointInterface):
    def __init__(self, config: SimulatedKeypointConfig, ground_truth_manager: GroundTruthManager) -> None:
        self.ground_truth_manager = ground_truth_manager
        self.config = config
        self.logger = logging.getLogger(self.__class__.__name__)
        self.debug = self.config.debug
        self.model_loader = CameraModelLoader()
        self.keypoint_names = [KeypointName.FRONT, KeypointName.BACK]
        self.model_labels = tuple(ModelLabel)
        self.system_labels = tuple(Label)
        self.prev_stamps: set[float] = set()

        self.model_to_system_labels = self.config.model_to_system_labels.labels
        if len(self.model_to_system_labels) == 0:
            self.logger.warning("No simulated to real label mapping provided")
        self.class_indices = self.config.model_to_system_labels.get_class_indices(self.model_labels)
        self.logger.info(f"Simulated to real label mapping: {self.model_to_system_labels}")

    def process_image(
        self, camera_info: CameraInfo, rgb_image: Image, field: Field
    ) -> tuple[KeypointInstanceArray | None, Image | None]:
        robots = self.ground_truth_manager.get_robots()
        stamps = set([robot.header.stamp.to_sec() for robot in robots])
        if stamps == self.prev_stamps:
            return None, None
        self.prev_stamps = stamps
        self.model_loader.update_model(camera_info)
        if not (model := self.model_loader.get_model()):
            self.logger.debug("Camera model not loaded")
            return None, None
        if robots is None:
            self.logger.debug("No robots found")
            return None, None

        array = KeypointInstanceArray(header=camera_info.header, height=camera_info.height, width=camera_info.width)
        debug_image = Image.from_other(rgb_image) if self.debug else None
        object_counts = {label: 0 for label in self.system_labels}
        for robot in robots:
            instance = self.odometry_to_keypoint(model, debug_image, robot, object_counts)
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
        if label == Label.SKIP:
            return None
        object_index = object_counts[label]
        object_counts[label] += 1
        try:
            front_pixel, back_pixel = project_object_to_front_back_uv(robot, model)
        except ProjectionError as e:
            self.logger.warning(f"Projection error: {e}")
            return None

        if debug_image:
            cv2.line(
                debug_image.data,
                (int(front_pixel.x), int(front_pixel.y)),
                (int(back_pixel.x), int(back_pixel.y)),
                (128, 128, 128),
                2,
            )
            cv2.circle(debug_image.data, (int(front_pixel.x), int(front_pixel.y)), 5, (0, 0, 255), -1)
            cv2.circle(debug_image.data, (int(back_pixel.x), int(back_pixel.y)), 5, (255, 0, 0), -1)
            cv2.putText(
                debug_image.data,
                f"{label.value}_{object_index}",
                (int(front_pixel.x), int(front_pixel.y)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

        return KeypointInstance(
            keypoints=[front_pixel, back_pixel],
            names=self.keypoint_names,  # type: ignore
            score=1.0,
            label=label,
            class_index=self.class_indices[label],
            object_index=object_index,
        )

    def get_model_to_system_labels(self) -> LabelMap:
        return self.config.model_to_system_labels.to_msg()
