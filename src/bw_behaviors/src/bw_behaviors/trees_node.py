#!/usr/bin/env python3
import py_trees
import rospy
from bw_tools.typing import get_param
from py_trees import display
from py_trees.trees import BehaviourTree
from py_trees.visitors import DisplaySnapshotVisitor

from bw_behaviors.container import Container
from bw_behaviors.subtrees import make_mode_tree


class SparseDisplaySnapshotVisitor(DisplaySnapshotVisitor):
    def __init__(self) -> None:
        super().__init__()
        self.status_str = ""

    def finalise(self) -> None:
        status_str = ""
        if self.root is not None:
            status_str = "\n" + display.unicode_tree(
                root=self.root,
                show_only_visited=self.display_only_visited_behaviours,
                show_status=False,
                visited=self.visited,
                previously_visited=self.previously_visited,
            )
        if self.display_blackboard:
            status_str = display.unicode_blackboard(key_filter=self.visited_blackboard_keys)
        if self.display_activity_stream:
            status_str = display.unicode_blackboard_activity_stream()
        if status_str != self.status_str:
            self.status_str = status_str
            print(status_str)


class TreesNode:
    def __init__(self) -> None:
        self.container = Container()
        self.tree = self.make_tree()
        self.tick_rate = get_param("~tick_rate", 10.0)

        py_trees.logging.level = py_trees.logging.Level.INFO
        snapshot_visitor = SparseDisplaySnapshotVisitor()

        self.tree.visitors.append(snapshot_visitor)

        rospy.on_shutdown(lambda: self.tree.interrupt())

        rospy.loginfo("Behaviors node initialized")

    def make_tree(self) -> BehaviourTree:
        return BehaviourTree(make_mode_tree(self.container))

    def run(self):
        self.tree.setup(timeout=15)
        rate = rospy.Rate(self.tick_rate)
        while not rospy.is_shutdown():
            self.tree.tick()
            rate.sleep()
        self.tree.shutdown()


if __name__ == "__main__":
    rospy.init_node("bw_behaviors", log_level=rospy.DEBUG)
    node = TreesNode()
    node.run()
