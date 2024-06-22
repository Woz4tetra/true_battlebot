#!/usr/bin/python
import copy

import rospy
from bw_interfaces.msg import ConfiguredOpponents, RobotFleetConfigMsg
from bw_shared.configs.robot_fleet_config import RobotFleetConfig
from bw_shared.configs.shared_config import SharedConfig


class DynamicOpponentConfigNode:
    def __init__(self) -> None:
        config = SharedConfig.from_files()
        self.opponent_templates = {template.name: template for template in config.opponent_templates.robots}
        self.configured_opponents_sub = rospy.Subscriber(
            "configured_opponents", ConfiguredOpponents, self.opponent_keys_callback
        )
        self.opponent_templates_pub = rospy.Publisher(
            "opponent_templates", ConfiguredOpponents, queue_size=1, latch=True
        )
        self.opponent_fleet_pub = rospy.Publisher("opponent_fleet", RobotFleetConfigMsg, queue_size=1, latch=True)

    def opponent_keys_callback(self, msg: ConfiguredOpponents) -> None:
        rospy.loginfo(f"Received configured opponents: {msg.names}")
        opponent_fleet = RobotFleetConfig()
        name_counts = {}
        for key in msg.names:
            opponent_instance = copy.deepcopy(self.opponent_templates[key])
            base_name = opponent_instance.name
            if base_name not in name_counts:
                name_counts[base_name] = 0
            opponent_instance.name = f"{base_name}_{name_counts[base_name]}"
            name_counts[base_name] += 1
            opponent_fleet.robots.append(opponent_instance)
        self.opponent_fleet_pub.publish(opponent_fleet.to_msg())

    def run(self) -> None:
        while not rospy.is_shutdown():
            self.opponent_templates_pub.publish(
                ConfiguredOpponents(
                    names=list(
                        self.opponent_templates.keys(),
                    )
                )
            )
            rospy.sleep(1.0)


def main() -> None:
    rospy.init_node("dynamic_opponent_config_node")
    DynamicOpponentConfigNode().run()


if __name__ == "__main__":
    main()
