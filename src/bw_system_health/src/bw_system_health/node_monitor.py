#!/usr/bin/env python
import rosnode
import rospy
from bw_interfaces.msg import HealthSummary
from std_msgs.msg import Header


class NodeMonitor:
    def __init__(self) -> None:
        self.health_pub = rospy.Publisher("health", HealthSummary, queue_size=1, latch=True)
        self.nodes: dict[str, bool] = {}

    def run(self) -> None:
        while not rospy.is_shutdown():
            nodes = rosnode.get_node_names()
            summary = HealthSummary(header=Header(stamp=rospy.Time.now()))
            for node_name in self.nodes.keys():
                self.nodes[node_name] = False
            for node_name in nodes:
                self.nodes[node_name] = True
            for node_name, is_alive in self.nodes.items():
                node_list = summary.active_nodes if is_alive else summary.dead_nodes
                node_list.append(node_name)
            self.health_pub.publish(summary)
            rospy.sleep(1)


if __name__ == "__main__":
    rospy.init_node("node_monitor", log_level=rospy.DEBUG)
    NodeMonitor().run()
