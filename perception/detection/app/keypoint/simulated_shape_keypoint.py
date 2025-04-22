import logging

import cv2
import numpy as np
from app.config.keypoint_config.simulated_shape_keypoint_config import SimulatedShapeKeypointConfig
from app.keypoint.keypoint_interface import KeypointInterface
from app.keypoint.pattern_finder.pattern_finder import PatternFinder
from app.keypoint.warp_field_perspective import get_warp_perspective_field
from app.segmentation.simulated_segmentation_manager import SimulatedSegmentationManager
from bw_interfaces.msg import KeypointInstance, KeypointInstanceArray, LabelMap, UVKeypoint
from bw_shared.enums.keypoint_name import KeypointName
from bw_shared.enums.label import NON_FIELD_GROUP, OUR_ROBOT_GROUP, Label, ModelLabel
from bw_shared.geometry.camera.image_rectifier import ImageRectifier
from bw_shared.messages.field import Field
from perception_tools.camera.camera_model_loader import CameraModelLoader
from perception_tools.messages.image import Image
from sensor_msgs.msg import CameraInfo


class SimulatedShapeKeypoint(KeypointInterface):
    def __init__(
        self,
        config: SimulatedShapeKeypointConfig,
        simulated_segmentation_manager: SimulatedSegmentationManager,
        pattern_finder: PatternFinder,
    ) -> None:
        self.simulated_segmentation_manager = simulated_segmentation_manager
        self.config = config
        self.logger = logging.getLogger(self.__class__.__name__)
        self.debug = self.config.debug_image
        self.model_loader = CameraModelLoader()
        self.keypoint_names = [KeypointName.FRONT, KeypointName.BACK]
        self.model_labels = tuple(NON_FIELD_GROUP)
        self.our_team_labels = tuple(OUR_ROBOT_GROUP)
        self.system_labels = tuple(Label)
        self.rectifier: ImageRectifier | None = None
        self.pattern_finder = pattern_finder
        self.camera_info = CameraInfo()

        self.model_to_system_labels = self.config.model_to_system_labels.labels
        if len(self.model_to_system_labels) == 0:
            self.logger.warning("No simulated to real label mapping provided")
        self.class_indices = self.config.model_to_system_labels.get_class_indices(self.model_labels)
        self.logger.info(f"Simulated to real label mapping: {self.model_to_system_labels}")

    def process_image(
        self, camera_info: CameraInfo, rgb_image: Image, field: Field
    ) -> tuple[KeypointInstanceArray | None, Image | None]:
        if self.camera_info.header.frame_id != camera_info.header.frame_id:
            self.camera_info = camera_info
            self.rectifier = ImageRectifier(camera_info)
            self.model_loader.update_model(self.rectifier.get_rectified_info())
        if not (camera_model := self.model_loader.get_model()):
            self.logger.debug("Camera model not loaded")
            return None, None
        assert self.rectifier is not None
        if not (layer_image := self.simulated_segmentation_manager.get_layer_image()):
            self.logger.debug("No layer image found")
            return None, None
        rectified_layer_image = self.rectifier.rectify(layer_image.data)
        contour_map, exceptions = self.simulated_segmentation_manager.convert_layer_image_to_contour_map(
            rectified_layer_image, selected_labels=self.model_labels
        )
        if exceptions:
            for color, exception in exceptions.items():
                self.logger.warning(f"Exception for color {color}: {exception}")
        if contour_map is None:
            return None, None
        rectified_rgb_image = self.rectifier.rectify(rgb_image.data)

        tf_warp, new_dims = get_warp_perspective_field(rectified_rgb_image.shape[1], field, camera_model)
        if tf_warp is None or new_dims is None:
            self.logger.warning("No perspective transform found for field")
            return None, None

        warped_image = cv2.warpPerspective(
            rectified_rgb_image, tf_warp, new_dims, flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
        )
        debug_image = Image(rgb_image.header, np.copy(warped_image)) if self.debug else None

        keypoint_labels, keypoints_array = self.compute_warped_keypoint_arrays(
            warped_image, tf_warp, contour_map, debug_image
        )
        if debug_image is not None:
            debug_image.data = self.rectifier.unrectify(debug_image.data)
        array = KeypointInstanceArray(
            header=camera_info.header,
            height=camera_info.height,
            width=camera_info.width,
            instances=self.keypoint_instances_from_keypoint_arrays(keypoint_labels, keypoints_array),
        )
        return array, debug_image

    def compute_warped_keypoint_arrays(
        self,
        warped_image: np.ndarray,
        tf_warp: np.ndarray,
        contour_map: dict[ModelLabel, list[np.ndarray]],
        debug_image: Image | None,
    ) -> tuple[list[ModelLabel], np.ndarray]:
        keypoint_labels = []
        warped_keypoints = []
        for model_label, contours in contour_map.items():
            if model_label not in self.our_team_labels:
                continue  # TODO: Handle other labels
            for contour in contours:
                label = self.model_to_system_labels[model_label]
                channel = self.config.label_to_color_channel.get(label, 0)
                transformed_contour = cv2.perspectiveTransform(contour.astype(np.float32), tf_warp)
                warped_channel = warped_image[:, :, channel]
                warped_front, warped_back = self.pattern_finder.find(warped_channel, transformed_contour, debug_image)
                if warped_front is None or warped_back is None:
                    self.logger.warning(f"Failed to find keypoints for label {model_label}")
                    continue
                warped_keypoints.append((warped_front.x, warped_front.y))
                warped_keypoints.append((warped_back.x, warped_back.y))
                keypoint_labels.append(model_label)
                keypoint_labels.append(model_label)
        tf_warp_inv = np.linalg.inv(tf_warp)

        if debug_image is not None:
            debug_image.data = cv2.warpPerspective(
                debug_image.data, tf_warp_inv, (debug_image.width, debug_image.height), flags=cv2.INTER_LINEAR
            )

        if len(warped_keypoints) == 0:
            self.logger.warning("No keypoints found")
            return [], np.array([])
        warped_keypoints_array = np.array(warped_keypoints, dtype=np.float32).reshape(-1, 1, 2)
        keypoints_array = cv2.perspectiveTransform(warped_keypoints_array, tf_warp_inv)
        return keypoint_labels, keypoints_array.reshape(-1, 2)

    def keypoint_instances_from_keypoint_arrays(
        self, keypoint_labels: list[ModelLabel], keypoints_array: np.ndarray
    ) -> list[KeypointInstance]:
        keypoint_instances = []
        object_counts = {label: 0 for label in self.system_labels}
        for index in range(0, len(keypoint_labels), 2):
            front_pixel = UVKeypoint(x=keypoints_array[index][0], y=keypoints_array[index][1])
            back_pixel = UVKeypoint(x=keypoints_array[index + 1][0], y=keypoints_array[index + 1][1])
            model_label = keypoint_labels[index]
            label = self.model_to_system_labels[model_label]
            if label == Label.SKIP:
                continue
            object_index = object_counts[label]
            object_counts[label] += 1
            keypoint_instances.append(
                KeypointInstance(
                    keypoints=[front_pixel, back_pixel],
                    names=self.keypoint_names,  # type: ignore
                    score=1.0,
                    label=label.value,
                    class_index=self.class_indices[label],
                    object_index=object_index,
                )
            )
        return keypoint_instances

    def get_model_to_system_labels(self) -> LabelMap:
        return self.config.model_to_system_labels.to_msg()
