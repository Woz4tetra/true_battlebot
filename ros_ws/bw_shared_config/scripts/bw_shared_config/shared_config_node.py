#!/usr/bin/python
import rospy
from bw_shared.configs.shared_config import SharedConfig
from bw_shared.environment import get_map, get_robot


def main() -> None:
    rospy.init_node("shared_config_node")
    config = SharedConfig.from_files()
    rospy.set_param("shared_config", config.to_dict())
    rospy.set_param("map", get_map())
    rospy.set_param("robot", get_robot())
    rospy.spin()


if __name__ == "__main__":
    main()
