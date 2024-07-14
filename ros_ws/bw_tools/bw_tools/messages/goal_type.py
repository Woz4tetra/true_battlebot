from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class GoalType(EnumAutoLowerStr):
    FIXED_POSE = auto()
    TRACKED_TARGET = auto()
