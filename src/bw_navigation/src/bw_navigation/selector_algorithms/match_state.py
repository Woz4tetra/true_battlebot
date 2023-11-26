from dataclasses import dataclass

from bw_interfaces.msg import EstimatedObject
from geometry_msgs.msg import PoseStamped


@dataclass
class MatchState:
    frame_id: str
    controlled_bot: EstimatedObject
    guidance_bot: EstimatedObject
    opponent_bot: EstimatedObject
    field: EstimatedObject


@dataclass
class SelectionResult:
    goal: PoseStamped
    ignore_opponent_obstacles: bool
