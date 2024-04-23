#!/usr/bin/python
import rospy
from bw_shared.configs.shared_config import SharedConfig


def main() -> None:
    rospy.init_node("shared_config_node")
    config = SharedConfig.from_files()
    rospy.set_param("shared_config", config.to_dict())
    rospy.spin()


if __name__ == "__main__":
    main()
