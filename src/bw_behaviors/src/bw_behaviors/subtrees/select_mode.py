from typing import Callable, Dict, List

from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence

from bw_behaviors.container import Container
from bw_behaviors.structs.modes import Mode
from bw_behaviors.subtrees.corner_mode import make_stay_in_corner_behavior
from bw_behaviors.subtrees.fight_mode import make_fight_behavior
from bw_behaviors.subtrees.idle_mode import make_idle_behavior
from bw_behaviors.subtrees.is_mode import IsMode


def make_mode_tree(container: Container) -> Behaviour:
    subtrees: Dict[Mode, Callable[[Container], Behaviour]] = {
        Mode.IDLE: make_idle_behavior,
        Mode.CORNER: make_stay_in_corner_behavior,
        Mode.FIGHT: make_fight_behavior,
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
