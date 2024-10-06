from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from bw_behaviors.behaviors.send_clicked_goal import SendClickedGoal
from bw_behaviors.container import Container


def make_clicked_point_behavior(container: Container) -> Behaviour:
    return Sequence(
        "clicked_point_main_sequence",
        memory=False,
        children=[
            SendClickedGoal(container),
        ],
    )
