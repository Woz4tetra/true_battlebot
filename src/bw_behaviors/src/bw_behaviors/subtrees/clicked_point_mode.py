from bw_tools.structs.behavior_mode import BehaviorMode
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from bw_behaviors.behaviors.exe_path import ExePath
from bw_behaviors.behaviors.get_path import GetPath
from bw_behaviors.behaviors.set_goal_to_simple import SetGoalToSimple
from bw_behaviors.behaviors.set_mode import SetMode
from bw_behaviors.container import Container


def make_clicked_point_behavior(container: Container) -> Behaviour:
    return Sequence(
        "clicked_point_sequence",
        memory=True,
        children=[
            SetGoalToSimple(container),
            GetPath(container),
            ExePath(container),
            SetMode(container, BehaviorMode.IDLE),
        ],
    )
