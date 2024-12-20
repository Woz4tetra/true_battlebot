import time
from unittest import mock

import pytest
import rospy

with mock.patch("rospy.Subscriber"), mock.patch("actionlib.simple_action_client.SimpleActionClient"):
    from bw_behaviors.config.container_config import ContainerConfig
    from bw_behaviors.container import Container
    from bw_behaviors.managers.corner_manager import CornerManager
    from bw_behaviors.managers.go_to_goal_manager import GoToGoalManager
    from bw_behaviors.managers.mode_manager import ModeManager
    from bw_behaviors.trees_node import TreesNode
    from bw_tools.messages.behavior_mode import BehaviorMode


class MockContainer(Container):
    def __init__(self) -> None:
        self.config = ContainerConfig()

        self.mode_manager = ModeManager()
        self.mode_manager.mode_sub.side_effect = lambda *args, **kwargs: None  # type: ignore

        self.corner_manager = CornerManager(self.config.corner_offset)
        self.corner_manager.field_sub.side_effect = lambda *args, **kwargs: None  # type: ignore

        self.go_to_goal_manager = GoToGoalManager(self.config.go_to_goal)


@pytest.fixture
def container() -> Container:
    return MockContainer()


@pytest.fixture
def node(container: Container) -> TreesNode:
    with (
        mock.patch("rospy.rostime.get_rostime", side_effect=lambda: rospy.Time.from_sec(time.time())),
        mock.patch("rospy.Publisher"),
    ):
        return TreesNode(container)


def test_tick(node: TreesNode):
    with mock.patch("rospy.rostime.get_rostime", side_effect=lambda: rospy.Time.from_sec(time.time())):
        tree = node.tree
        container = node.container
        tree.setup(timeout=15)

        tree.tick()
        tip = tree.tip()
        assert tip is not None and tip.name == "idle"

        container.mode_manager.mode = BehaviorMode.CORNER
        tree.tick()
        tip = tree.tip()
        assert tip is not None and tip.name == "SendCornerGoal"

        tree.shutdown()
