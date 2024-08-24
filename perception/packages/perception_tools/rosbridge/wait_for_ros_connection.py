import os
import re
import time

from perception_tools.rosbridge.check_connection import check_connection


def wait_for_ros_connection(ros_master_uri: str = "", connection_timeout: float = 5.0) -> str:
    if not ros_master_uri:
        ros_master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    match = re.search(r"http://(.*):(.*)", ros_master_uri)
    if not match:
        raise ValueError(f"Invalid ROS_MASTER_URI: {ros_master_uri}")
    host_ip = match.group(1)
    host_port = int(match.group(2))

    is_connected = False
    start_time = time.monotonic()
    while time.monotonic() - start_time < connection_timeout:
        if check_connection(host_ip, host_port):
            is_connected = True
            break
        time.sleep(0.1)
    if not is_connected:
        raise RuntimeError(f"Failed to connect to ROS master. {ros_master_uri}")
    return ros_master_uri
