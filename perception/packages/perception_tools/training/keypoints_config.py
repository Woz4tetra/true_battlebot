from __future__ import annotations

import toml
from bw_shared.enums.keypoint_name import KeypointName
from bw_shared.enums.label import ModelLabel


def load_keypoints_config(config_path: str) -> dict[ModelLabel, list[KeypointName]]:
    with open(config_path, "r") as f:
        config = toml.load(f)
    return {
        ModelLabel(model_label): [KeypointName(name) for name in key_names]
        for model_label, key_names in config["keypoint_names"].items()
    }
