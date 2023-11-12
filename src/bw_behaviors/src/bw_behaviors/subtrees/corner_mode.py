from py_trees.behaviour import Behaviour
from py_trees.behaviours import Running
from py_trees.composites import Sequence

from bw_behaviors.behaviors.exe_path import ExePath
from bw_behaviors.behaviors.get_path import GetPath
from bw_behaviors.behaviors.set_goal_to_corner import SetGoalToCorner
from bw_behaviors.container import Container


def make_stay_in_corner_behavior(container: Container) -> Behaviour:
    return Sequence(
        "stay_in_corner_sequence",
        memory=True,
        children=[
            SetGoalToCorner(container),
            GetPath(container),
            ExePath(container),
            Running("stay_in_corner_running"),
        ],
    )
