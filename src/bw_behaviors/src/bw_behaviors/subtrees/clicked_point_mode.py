from bw_tools.structs.behavior_mode import BehaviorMode
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from bw_behaviors.behaviors.set_goal_to_simple import SetGoalToSimple
from bw_behaviors.behaviors.set_mode import SetMode
from bw_behaviors.behaviors.supplied_goal_behavior import make_supplied_goal_behavior
from bw_behaviors.container import Container


def make_clicked_point_behavior(container: Container) -> Behaviour:
    return Sequence(
        "clicked_point_main_sequence",
        memory=True,
        children=[
            make_supplied_goal_behavior("clicked_point", SetGoalToSimple(container), container),
            SetMode(container, BehaviorMode.IDLE),
        ],
    )
