from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from bw_behaviors.container import Container


def make_fight_behavior(container: Container) -> Behaviour:
    return Sequence(
        "fight_sequence",
        memory=True,
        children=[
            # TODO: send goal
        ],
    )
