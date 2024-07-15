from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class TargetType(EnumAutoLowerStr):
    NEAREST_OPPONENT = auto()
    LARGEST_OPPONENT = auto()
