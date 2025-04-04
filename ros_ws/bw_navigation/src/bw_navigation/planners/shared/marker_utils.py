from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker


def make_pose_marker(
    pose: Pose,
    ns: str = "",
    marker_id: int = 0,
    color: tuple[float, float, float] = (1.0, 0.2, 0.1),
    scale: tuple[float, float, float] = (0.1, 0.02, 0.02),
    alpha: float = 1.0,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = ns
    marker.id = marker_id
    marker.pose = pose
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]
    marker.color.a = alpha
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    return marker


def make_text_marker(
    text: str,
    pose: Pose,
    ns: str = "",
    marker_id: int = 0,
    color: tuple[float, float, float] = (1.0, 1.0, 1.0),
    scale: float = 0.1,
    alpha: float = 1.0,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = ns
    marker.id = marker_id
    marker.pose = pose
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.scale.z = scale
    marker.color.a = alpha
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.text = text
    return marker
