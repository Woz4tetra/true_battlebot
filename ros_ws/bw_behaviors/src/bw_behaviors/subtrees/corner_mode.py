from bw_tools.messages.behavior_mode import BehaviorMode
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from bw_behaviors.behaviors.send_corner_goal import SendCornerGoal
from bw_behaviors.behaviors.set_mode import SetMode
from bw_behaviors.container import Container


def make_stay_in_corner_behavior(container: Container) -> Behaviour:
    config = container.config.corner_mode
    return Sequence(
        "stay_in_corner_sequence",
        memory=False,
        children=[
            SendCornerGoal(container, engine_config=config.engine_config, xy_tolerance=config.xy_tolerance),
            SetMode(container, BehaviorMode.IDLE),
        ],
    )
