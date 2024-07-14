from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class GoalStrategy(EnumAutoLowerStr):
    CRASH_OPPONENT = auto()
    CRASH_OPPONENT_AVOID_FRIENDLY = auto()
