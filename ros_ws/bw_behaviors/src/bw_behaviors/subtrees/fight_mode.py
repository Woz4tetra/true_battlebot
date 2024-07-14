from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from bw_behaviors.behaviors.send_target_goal import SendTargetGoal
from bw_behaviors.container import Container


def make_fight_behavior(container: Container) -> Behaviour:
    return Sequence(
        "fight_sequence",
        memory=True,
        children=[SendTargetGoal(container, "opponent_3lb_1")],
    )
