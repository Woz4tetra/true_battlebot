import pytest
from app.config.keypoint_config.yolo_keypoint_config import YoloKeypointConfig
from app.keypoint.yolo_keypoint import YoloKeypoint
from bw_shared.configs.model_to_system_labels_map import ModelToSystemLabelsMap
from bw_shared.enums.label import Label, ModelLabel

MODEL_NAME = "yolo11n-pose_2024-10-09"


@pytest.fixture(scope="module")
def yolo_keypoint_config() -> YoloKeypointConfig:
    return YoloKeypointConfig(
        model_path=MODEL_NAME + ".pt",
        metadata_path=MODEL_NAME + ".json",
        debug_timing=True,
        model_to_system_labels=ModelToSystemLabelsMap(
            {
                ModelLabel.MR_STABS_MK1: Label.CONTROLLED_ROBOT,
                ModelLabel.MR_STABS_MK2: Label.CONTROLLED_ROBOT,
                ModelLabel.MRS_BUFF_MK1: Label.FRIENDLY_ROBOT,
                ModelLabel.MRS_BUFF_MK2: Label.FRIENDLY_ROBOT,
                ModelLabel.ROBOT: Label.ROBOT,
                ModelLabel.REFEREE: Label.REFEREE,
            }
        ),
    )


@pytest.fixture(scope="module")
def yolo_keypoint(yolo_keypoint_config: YoloKeypointConfig) -> YoloKeypoint:
    return YoloKeypoint(yolo_keypoint_config)
