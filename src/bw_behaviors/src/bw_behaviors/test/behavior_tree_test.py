import pytest
from bw_behaviors.trees_node import TreesNode
from bw_tools.structs.behavior_mode import BehaviorMode
from py_trees.trees import BehaviourTree


@pytest.fixture
def node() -> TreesNode:
    return TreesNode()


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
