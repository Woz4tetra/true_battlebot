from py_trees.behaviour import Behaviour
from py_trees.behaviours import Running

from bw_behaviors.container import Container


def make_idle_behavior(container: Container) -> Behaviour:
    return Running("idle")
