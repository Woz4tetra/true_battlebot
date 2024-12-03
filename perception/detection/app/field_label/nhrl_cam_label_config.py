from __future__ import annotations

from dataclasses import dataclass

from app.field_label.field_label_config import FieldLabelConfig


@dataclass
class NhrlCamLabelConfig(FieldLabelConfig):
    video_frame_height: int = 1000
    num_extra_points: int = 20
    video_state_path: str = "/data/nhrl_cam_label_state.json"
