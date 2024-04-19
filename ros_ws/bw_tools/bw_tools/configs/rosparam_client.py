import rospy
from bw_shared.configs.shared_config import SharedConfig


def get_shared_config() -> SharedConfig:
    key = "/shared_config"
    while not rospy.has_param(key):
        rospy.loginfo_throttle(3, f"Waiting for {key} to be set")
        rospy.sleep(0.1)
    shared_config = rospy.get_param(key)
    return SharedConfig.from_dict(shared_config)  # type: ignore
