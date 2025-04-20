from dataclasses import dataclass

from bw_shared.enums.frame_id import FrameId


@dataclass
class CameraTopicConfig:
    namespace: str = "/camera_0"
    frame_id: FrameId = FrameId.CAMERA_0
    world_frame_id: FrameId = FrameId.WORLD_CAMERA_0
