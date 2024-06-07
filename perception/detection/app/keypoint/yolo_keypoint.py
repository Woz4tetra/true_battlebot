import logging

from app.config.keypoint_config.yolo_keypoint_config import YoloKeypointConfig
from app.keypoint.keypoint_interface import KeypointInterface
from bw_interfaces.msg import KeypointInstance, KeypointInstanceArray, UVKeypoint
from bw_shared.enums.keypoint_name import RobotKeypointsNames
from bw_shared.enums.label import Label
from perception_tools.messages.image import Image
from ultralytics import YOLO


class YoloKeypoint(KeypointInterface):
    def __init__(self, config: YoloKeypointConfig) -> None:
        self.config = config
        self.model = YOLO(config.model_path)
        self.keypoint_names = [name.value for name in RobotKeypointsNames]
        self.logger = logging.getLogger("perception")

    def process_image(self, msg: Image) -> tuple[KeypointInstanceArray, Image | None]:
        result = self.model(msg.data)[0]

        ids = result.boxes.cpu().cls.int().numpy()  # get the class ids
        keypoints = result.keypoints.cpu().xy.int().numpy()  # get the keypoints
        labels = [Label(result.names[index]) for index in ids]

        if self.config.debug:
            img_array = result.plot(kpt_line=True, kpt_radius=6)  # plot a BGR array of predictions
            debug_image = Image(header=msg.header, data=img_array)
        else:
            debug_image = None

        keypoint_instances = []
        object_counts = {label: 0 for label in Label}
        for keypoint, label, class_idx in zip(keypoints, labels, ids):
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
