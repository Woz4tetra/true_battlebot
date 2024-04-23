from dataclasses import dataclass

from perception_tools.messages.geometry.transform import Pose
from perception_tools.messages.geometry.xyz import XYZ
from perception_tools.messages.std_msgs.header import Header


@dataclass
class FieldResult:
    header: Header
    pose: Pose
    size: XYZ
