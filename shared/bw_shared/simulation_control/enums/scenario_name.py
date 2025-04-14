from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class ScenarioName(str, EnumAutoLowerStr):
    SIMPLE_SCENARIO = auto()
    IMAGE_SYNTHESIS = auto()
