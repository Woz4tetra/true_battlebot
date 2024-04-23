from bw_interfaces.msg import SystemSummary
from bw_shared.environment import get_image_version, get_map, get_robot


def get_system_info() -> SystemSummary:
    return SystemSummary(
        robot=get_robot(),
        map=get_map(),
        version=get_image_version(),
    )
