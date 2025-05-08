from perception_tools.training.keypoints_config import KeypointsConfig

from auto_label.backend.manual_label_backend import ManualLabelBackend


class AiInterpolator:
    def __init__(self, manual_label_backend: ManualLabelBackend, keypoints_config: KeypointsConfig) -> None:
        self.manual_label_backend = manual_label_backend
        self.keypoints_config = keypoints_config

    def interpolate_all_keyframes(self) -> None:
        pass

    def interpolate_from_current_frame(self) -> None:
        pass
