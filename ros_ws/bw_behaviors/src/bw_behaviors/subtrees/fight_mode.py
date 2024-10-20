from bw_tools.messages.target_type import TargetType
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from bw_behaviors.behaviors.send_target_goal import SendTargetGoal
from bw_behaviors.container import Container


def make_fight_behavior(container: Container) -> Behaviour:
    target_goal = SendTargetGoal(container, TargetType.NEAREST_OPPONENT, continuously_select_goal=True)
    # target_goal = SendTargetGoal(container, TargetType.LARGEST_OPPONENT, continuously_select_goal=False)
    return Sequence(
        "fight_sequence",
        memory=False,
        children=[target_goal],
    )
