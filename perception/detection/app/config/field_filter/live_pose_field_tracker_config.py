from dataclasses import dataclass
from typing import Literal


@dataclass
class LivePoseFieldTrackerConfig:
    type: Literal["LivePoseFieldTracker"] = "LivePoseFieldTracker"
