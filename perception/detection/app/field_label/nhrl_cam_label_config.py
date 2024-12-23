from __future__ import annotations

from dataclasses import dataclass

from app.field_label.field_label_config import FieldLabelConfig


@dataclass
class NhrlCamLabelConfig(FieldLabelConfig):
    video_frame_height: int = 1000
    num_extra_points: int = 20
    svo_start_time: float = 0.0
    video_state_path: str = "/data/nhrl_cam_label_state.json"
    nhrl_camera_calibration_path: str = "/data/videos/iphone_15_pro_wide.toml"
    camera_0_location: str = "blue"
