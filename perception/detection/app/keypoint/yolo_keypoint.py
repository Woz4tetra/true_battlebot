import logging
import time

import numpy as np
from app.config.keypoint_config.yolo_keypoint_config import YoloKeypointConfig
from app.keypoint.keypoint_interface import KeypointInterface
from bw_interfaces.msg import KeypointInstance, KeypointInstanceArray, UVKeypoint
from bw_shared.enums.keypoint_name import RobotKeypointsNames
from bw_shared.enums.label import Label
from perception_tools.data_directory import get_data_directory
from perception_tools.messages.image import Image

logging.setLoggerClass(logging.Logger)  # fix rospy breaking logs
from ultralytics import YOLO


class YoloKeypoint(KeypointInterface):
    def __init__(self, config: YoloKeypointConfig) -> None:
        self.logger = logging.getLogger("perception")
        self.config = config
        data_dir = get_data_directory()
        model_path = data_dir / "models" / config.model_name

        self.model_to_system_labels = {
            model_label: Label(real_label) for model_label, real_label in self.config.model_to_system_labels.items()
        }

        self.logger.info(f"Loading model from {model_path}")
        self.model = YOLO(str(model_path))
        self.logger.info("Model loaded")
        self.keypoint_names = [name.value for name in RobotKeypointsNames]

        self.warmup()

    def warmup(self) -> None:
        self.logger.info("Warming up model")
        t0 = time.perf_counter()
        for _ in range(3):
            image = np.random.random_integers(0, 255, size=(720, 1280, 3)).astype(np.uint8)
            self.model(image)
        t1 = time.perf_counter()
        self.logger.info(f"Model warmed up in {t1 - t0} seconds")

    def process_image(self, msg: Image) -> tuple[KeypointInstanceArray, Image | None]:
        result = self.model(msg.data, verbose=self.config.debug)[0]

        ids = result.boxes.cpu().cls.int().numpy()  # get the class ids
        keypoints = result.keypoints.cpu().xy.int().numpy()  # get the keypoints
        labels = [self.model_to_system_labels.get(result.names[index], Label.BACKGROUND) for index in ids]

        if self.config.debug:
            img_array = result.plot(kpt_line=True, kpt_radius=6)  # plot a BGR array of predictions
            debug_image = Image(header=msg.header, data=img_array)
        else:
            debug_image = None

        keypoint_instances = []
        object_counts = {label: 0 for label in Label}
        for keypoint, label, class_idx in zip(keypoints, labels, ids):
            if label == Label.BACKGROUND:
                continue
            if len(keypoint) != len(RobotKeypointsNames):
                raise ValueError(f"Expected {len(RobotKeypointsNames)} keypoints, but got {len(keypoint)}")
            kp_front = UVKeypoint(x=keypoint[0][0], y=keypoint[0][1])
            kp_back = UVKeypoint(x=keypoint[1][0], y=keypoint[1][1])
            keypoint_instances.append(
                KeypointInstance(
                    keypoints=[kp_front, kp_back],
                    names=self.keypoint_names,
                    label=label,
                    class_index=class_idx,
                    object_index=object_counts[label],
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
