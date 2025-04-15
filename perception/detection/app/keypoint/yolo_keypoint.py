import logging
import time

import numpy as np
from app.config.keypoint_config.yolo_keypoint_config import YoloKeypointConfig
from app.keypoint.keypoint_interface import KeypointInterface
from app.profiling.context_timer import ContextTimer
from bw_interfaces.msg import KeypointInstance, KeypointInstanceArray, LabelMap, UVKeypoint
from bw_shared.enums.label import Label, ModelLabel
from perception_tools.data_directory import get_data_directory
from perception_tools.inference.common import load_metadata
from perception_tools.messages.image import Image
from sensor_msgs.msg import CameraInfo
from ultralytics import YOLO


class YoloKeypoint(KeypointInterface):
    def __init__(self, config: YoloKeypointConfig) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.config = config
        data_dir = get_data_directory()
        model_path = data_dir / "models" / self.config.model_path
        if self.config.metadata_path:
            metadata_path = data_dir / "models" / self.config.metadata_path
        else:
            metadata_path = data_dir / "models" / (model_path.stem + ".json")
        self.model_metadata = load_metadata(metadata_path)

        self.model_to_system_labels = self.config.model_to_system_labels.labels
        self.class_indices = self.config.model_to_system_labels.get_class_indices(self.model_metadata.labels)

        self.logger.info(f"Loading model from {model_path}")
        self.model = YOLO(str(model_path))
        self.logger.info("Model loaded")
        self.keypoint_names = self.model_metadata.keypoint_map

        self.warmup()

    def warmup(self) -> None:
        self.logger.info("Warming up model")
        t0 = time.perf_counter()
        for _ in range(3):
            image = np.random.randint(0, 256, size=(720, 1280, 3)).astype(np.uint8)
            self.model(image)
        t1 = time.perf_counter()
        self.logger.info(f"Model warmed up in {t1 - t0} seconds")

    def process_image(self, camera_info: CameraInfo, msg: Image) -> tuple[KeypointInstanceArray | None, Image | None]:
        with ContextTimer("YoloKeypoint.model"):
            result = self.model(
                msg.data,
                verbose=self.config.debug_timing,
                conf=self.config.threshold,
                nms=True,
                iou=self.config.iou_threshold,
            )[0]

        with ContextTimer("YoloKeypoint.copy"):
            boxes = result.boxes.cpu()
            confidences = boxes.conf.float().numpy()
            ids = boxes.cls.int().numpy()  # get the class ids
            keypoints = result.keypoints.cpu().xy.int().numpy()  # get the keypoints
            labels = [ModelLabel(result.names[index]) for index in ids]

        if self.config.debug_image:
            with ContextTimer("YoloKeypoint.debug"):
                img_array = result.plot(kpt_line=True, kpt_radius=6)  # plot a BGR array of predictions
                debug_image = Image(header=msg.header, data=img_array)
        else:
            debug_image = None

        with ContextTimer("YoloKeypoint.message"):
            keypoint_instances = []
            object_counts = {label: 0 for label in Label}
            for keypoint, model_label, confidence in zip(keypoints, labels, confidences):
                if model_label == ModelLabel.BACKGROUND:
                    continue
                keypoint_names = self.keypoint_names[model_label]
                if len(keypoint) != len(keypoint_names):
                    raise ValueError(f"Expected {len(keypoint_names)} keypoints, but got {len(keypoint)}")
                label = self.model_to_system_labels[model_label]
                if label == Label.SKIP:
                    continue
                system_label_class_idx = self.class_indices[label]
                named_keypoints = {
                    "front": UVKeypoint(x=keypoint[0][0], y=keypoint[0][1]),
                    "back": UVKeypoint(x=keypoint[1][0], y=keypoint[1][1]),
                }
                keypoint_instances.append(
                    KeypointInstance(
                        keypoints=[named_keypoints[keypoint_name] for keypoint_name in keypoint_names],
                        names=[name.value for name in keypoint_names],  # type: ignore
                        label=label.value,
                        class_index=system_label_class_idx,
                        object_index=object_counts[label],
                        score=confidence,
                    )
                )
                object_counts[label] += 1

            keypoint_msg = KeypointInstanceArray(
                header=msg.header.to_msg(),
                height=msg.data.shape[0],
                width=msg.data.shape[1],
                instances=keypoint_instances,
            )

        return keypoint_msg, debug_image

    def get_model_to_system_labels(self) -> LabelMap:
        return self.config.model_to_system_labels.to_msg()
