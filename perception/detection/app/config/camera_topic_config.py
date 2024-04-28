from dataclasses import dataclass


@dataclass
class CameraTopicConfig:
    namespace: str
    frame_id: str
