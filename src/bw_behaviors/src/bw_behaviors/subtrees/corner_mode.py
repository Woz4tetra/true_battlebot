from bw_tools.structs.behavior_mode import BehaviorMode
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from bw_behaviors.behaviors.set_goal_to_corner import SetGoalToCorner
from bw_behaviors.behaviors.set_mode import SetMode
from bw_behaviors.behaviors.supplied_goal_behavior import make_supplied_goal_behavior
from bw_behaviors.container import Container


def make_stay_in_corner_behavior(container: Container) -> Behaviour:
    return Sequence(
        "stay_in_corner_sequence",
        memory=True,
        children=[
            make_supplied_goal_behavior("stay_in_corner", SetGoalToCorner(container), container),
            SetMode(container, BehaviorMode.IDLE),
        ],
    )
