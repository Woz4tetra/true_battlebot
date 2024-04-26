from dataclasses import dataclass


@dataclass
class CameraTopicConfig:
    namespace: str
    left_lens_frame_id: str
    frame_id: str
