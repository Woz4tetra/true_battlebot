import logging
from dataclasses import dataclass
from threading import Lock

import rospy
from bw_interfaces.msg import EstimatedObjectArray, SegmentationInstanceArray
from bw_shared.enums.label import ModelLabel
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as ImageMsg

from perception_tools.inference.simulated_mask_to_contours import make_simulated_segmentation_color_map
from perception_tools.messages.image import Image


@dataclass
class RobotDataShapshot:
    model: PinholeCameraModel
    image: Image
    layer: Image
    robots: EstimatedObjectArray
    color_to_model_label_map: dict[int, ModelLabel]


class SimulationRobotTopicSync:
    def __init__(self, filter_labels: tuple[ModelLabel, ...]) -> None:
        self.logger = logging.getLogger("perception")
        self.model: PinholeCameraModel | None = None
        self.image: Image | None = None
        self.layer: Image | None = None
        self.robots: EstimatedObjectArray | None = None
        self.color_to_model_label_map: dict[int, ModelLabel] = {}
        self.image_timestamp = 0.0
        self.lock = Lock()

        self.image_layer_max_delay = 0.001
        self.image_ground_max_delay = 0.015
        self.cache_size = 5
        self.layer_cache: dict[float, Image] = {}
        self.ground_truth_cache: dict[float, EstimatedObjectArray] = {}

        camera_ns = "/camera_0/"
        self.filter_labels = filter_labels

        rospy.Subscriber(
            camera_ns + "ground_truth/robots",
            EstimatedObjectArray,
            self.ground_truth_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            camera_ns + "rgb/camera_info",
            CameraInfo,
            self.camera_info_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            camera_ns + "rgb/image_raw",
            ImageMsg,
            self.image_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            camera_ns + "layer/image_raw",
            ImageMsg,
            self.layer_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            camera_ns + "simulated_segmentation",
            SegmentationInstanceArray,
            self.simulated_segmentation_label_callback,
            queue_size=1,
        )

    def get_snapshot(self) -> RobotDataShapshot | None:
        with self.lock:
            if self.image is None:
                self.logger.warning("Missing image")
                return None
            image = Image.from_other(self.image)
            self.image = None
            if self.layer is None:
                self.logger.warning("Missing layer")
                return None
            if self.robots is None:
                self.logger.warning("Missing robots")
                return None
            if self.model is None:
                self.logger.warning("Missing camera model")
                return None
            if not self.color_to_model_label_map:
                self.logger.warning("Missing color map data.")
                return None
            if self.robots.robots[0].header.stamp.to_sec() - self.image_timestamp > self.image_layer_max_delay:
                self.logger.warning("Robot and image timestamps are too far apart.")
                return None
            layer = Image.from_other(self.layer)
            self.layer = None
            robots = self.robots
            self.robots = None
            model = self.model

            return RobotDataShapshot(model, image, layer, robots, self.color_to_model_label_map)

    def image_callback(self, msg: ImageMsg) -> None:
        with self.lock:
            if self.image is not None:
                return
            self.image = Image.from_msg(msg)
            self.image_timestamp = msg.header.stamp.to_sec()

    def layer_callback(self, msg: ImageMsg) -> None:
        with self.lock:
            layer = Image.from_msg(msg)
            self.layer_cache[msg.header.stamp.to_sec()] = layer
            selected_key = min(self.layer_cache.keys(), key=lambda k: abs(k - self.image_timestamp))
            selected_msg = self.layer_cache[selected_key]
            if abs(selected_key - self.image_timestamp) <= self.image_layer_max_delay:
                self.layer = selected_msg
            while len(self.layer_cache) > self.cache_size:
                del self.layer_cache[min(self.layer_cache.keys())]

    def camera_info_callback(self, msg: CameraInfo) -> None:
        with self.lock:
            if self.model is not None:
                return
            self.logger.info("Received camera info.")
            self.model = PinholeCameraModel()
            self.model.fromCameraInfo(msg)

    def ground_truth_callback(self, msg: EstimatedObjectArray) -> None:
        with self.lock:
            self.ground_truth_cache[msg.robots[-1].header.stamp.to_sec()] = msg
            selected_key = min(self.ground_truth_cache.keys(), key=lambda k: abs(k - self.image_timestamp))
            selected_msg = self.ground_truth_cache[selected_key]
            if abs(selected_key - self.image_timestamp) <= self.image_ground_max_delay:
                self.robots = selected_msg
            else:
                self.logger.info(
                    f"Ground truth and image timestamps are too far apart. {selected_key - self.image_timestamp}"
                )
                self.robots = None
            while len(self.ground_truth_cache) > self.cache_size:
                del self.ground_truth_cache[min(self.ground_truth_cache.keys())]

    def simulated_segmentation_label_callback(self, msg: SegmentationInstanceArray) -> None:
        with self.lock:
            color_to_model_label_map, skipped_labels = make_simulated_segmentation_color_map(msg, self.filter_labels)
            if color_to_model_label_map != self.color_to_model_label_map:
                self.color_to_model_label_map = color_to_model_label_map
                labels = [label.value for label in self.color_to_model_label_map.values()]
                self.logger.info(f"Received labels: {str(labels)[1:-1]}. Skipped: {skipped_labels}")
