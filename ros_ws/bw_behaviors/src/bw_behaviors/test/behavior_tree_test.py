import pytest
from bw_behaviors.container import ContainerConfig
from bw_behaviors.trees_node import TreesNode
from bw_tools.messages.behavior_mode import BehaviorMode


@pytest.fixture
def node() -> TreesNode:
    return TreesNode(ContainerConfig())


def test_tick(node: TreesNode):
    tree = node.tree
    container = node.container
    tree.setup(timeout=15)

    tree.tick()
    tip = tree.tip()
    assert tip is not None and tip.name == "idle"

    container.mode_manager.mode = BehaviorMode.CORNER
    tree.tick()
    tip = tree.tip()
    assert tip is not None and tip.name == "corner"

    tree.shutdown()
