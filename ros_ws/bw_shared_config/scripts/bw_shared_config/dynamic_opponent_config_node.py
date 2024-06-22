#!/usr/bin/python
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
        self.opponent_fleet_pub = rospy.Publisher("opponent_fleet", RobotFleetConfigMsg, queue_size=1, latch=True)

    def opponent_keys_callback(self, msg: ConfiguredOpponents) -> None:
        rospy.loginfo(f"Received configured opponents: {msg.names}")
        opponent_fleet = RobotFleetConfig()
        for key in msg.names:
            opponent_fleet.robots.append(self.opponent_templates[key])
        self.opponent_fleet_pub.publish(opponent_fleet.to_msg())

    def run(self) -> None:
        rospy.spin()


def main() -> None:
    rospy.init_node("dynamic_opponent_config_node")
    DynamicOpponentConfigNode().run()


if __name__ == "__main__":
    main()
