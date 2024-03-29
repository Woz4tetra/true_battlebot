import rospy
from py_trees.behaviour import Behaviour
from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Selector, Sequence
from py_trees.decorators import SuccessIsFailure

from bw_behaviors.behaviors.exe_path import ExePath
from bw_behaviors.behaviors.get_path import GetPath
from bw_behaviors.behaviors.recovery_behavior import RecoveryBehavior
from bw_behaviors.behaviors.set_goal_to_optimal_attack import SetGoalToOptimalAttack
from bw_behaviors.behaviors.wait_for_target_to_move import WaitForTargetToMove
from bw_behaviors.container import Container


def make_fight_behavior(container: Container) -> Behaviour:
    set_goal = SetGoalToOptimalAttack(container, concurrency_slot=0)
    get_path = GetPath(container)
    wait_for_target = WaitForTargetToMove(
        container, min_next_time_to_move=rospy.Duration.from_sec(1.0), moved_threshold=0.25
    )
    exe_selector = Selector(
        "fight_exe_selector",
        memory=True,
        children=[
            ExePath(container, concurrency_slot=1, path_ready_on_start=False),
            SuccessIsFailure("recovery_success_is_failure", RecoveryBehavior(container)),
        ],
    )
    return Parallel(
        "fight_parallel",
        policy=ParallelPolicy.SuccessOnOne(),
        children=[
            Sequence(
                "fight_sequence",
                memory=True,
                children=[set_goal, get_path, wait_for_target],
            ),
            exe_selector,
        ],
    )
