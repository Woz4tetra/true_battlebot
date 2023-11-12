from typing import Callable, Dict, List

from bw_tools.structs.behavior_mode import BehaviorMode
from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence

from bw_behaviors.container import Container
from bw_behaviors.subtrees.clicked_point_mode import make_clicked_point_behavior
from bw_behaviors.subtrees.corner_mode import make_stay_in_corner_behavior
from bw_behaviors.subtrees.fight_mode import make_fight_behavior
from bw_behaviors.subtrees.idle_mode import make_idle_behavior
from bw_behaviors.subtrees.is_mode import IsMode


def make_mode_tree(container: Container) -> Behaviour:
    subtrees: Dict[BehaviorMode, Callable[[Container], Behaviour]] = {
        BehaviorMode.IDLE: make_idle_behavior,
        BehaviorMode.CORNER: make_stay_in_corner_behavior,
        BehaviorMode.FIGHT: make_fight_behavior,
        BehaviorMode.CLICKED_POINT: make_clicked_point_behavior,
    }

    sequences: List[Behaviour] = []
    for mode, make_fn in subtrees.items():
        sequence = Sequence(
            f"{mode.value}_sequence",
            memory=False,
            children=[IsMode(mode, container), make_fn(container)],
        )
        sequences.append(sequence)

    return Selector(
        "mode_selector",
        memory=False,
        children=sequences,
    )
