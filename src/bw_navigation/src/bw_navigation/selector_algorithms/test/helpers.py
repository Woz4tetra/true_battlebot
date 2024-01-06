from bw_interfaces.msg import EstimatedObject
from bw_navigation.selector_algorithms.match_state import MatchState
from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.xy import XY
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry


def pose2d_to_odom(pose: Pose2D) -> Odometry:
    msg = Odometry()
    msg.pose.pose = pose.to_msg()
    return msg


def make_match_state(control: Pose2D, guidance: Pose2D, opponent: Pose2D, size: XY) -> MatchState:
    return MatchState(
        frame_id="map",
        controlled_bot=EstimatedObject(state=pose2d_to_odom(control)),
        guidance_bot=EstimatedObject(state=pose2d_to_odom(guidance)),
        opponent_bot=EstimatedObject(state=pose2d_to_odom(opponent)),
        field=EstimatedObject(size=Vector3(x=size.x, y=size.y, z=0.0)),
    )
