from bw_tools.messages.behavior_mode import BehaviorMode
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from bw_behaviors.behaviors.send_corner_goal import SendCornerGoal
from bw_behaviors.behaviors.set_mode import SetMode
from bw_behaviors.container import Container


def make_stay_in_corner_behavior(container: Container) -> Behaviour:
    return Sequence(
        "stay_in_corner_sequence",
        memory=True,
        children=[
            SendCornerGoal(container),
            SetMode(container, BehaviorMode.IDLE),
        ],
    )
