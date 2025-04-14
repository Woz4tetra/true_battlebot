from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class ScenarioInitType(EnumAutoLowerStr):
    ABSOLUTE = auto()
    RELATIVE = auto()
    WORLD = auto()
    FLU = auto()
