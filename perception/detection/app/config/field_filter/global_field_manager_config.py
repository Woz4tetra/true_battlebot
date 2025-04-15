from dataclasses import dataclass
from typing import Literal

from bw_shared.enums.frame_id import FrameId


@dataclass
class GlobalFieldManagerConfig:
    type: Literal["GlobalFieldManager"] = "GlobalFieldManager"
    map_frame: FrameId = FrameId.MAP
    field_dims_buffer: float = 0.35
