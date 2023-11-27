from py_trees.behaviour import Behaviour
from py_trees.behaviours import Running
from py_trees.composites import Sequence

from bw_behaviors.behaviors.stop_driving import StopDriving
from bw_behaviors.container import Container


def make_idle_behavior(container: Container) -> Behaviour:
    return Sequence(
        "idle_sequence",
        memory=True,
        children=[StopDriving(container), Running("idle")],
    )
