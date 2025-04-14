from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class ObjectiveName(str, EnumAutoLowerStr):
    RANDOMIZED_CAMERA = auto()
    MINI_BOT_RANDOMIZED_SEQUENCE = auto()
    RANDOMIZED_START_TARGET_OPPONENT = auto()
    MAIN_BOT_RANDOMIZED_SEQUENCE = auto()
    RANDOMIZED_START_TARGET_MAIN = auto()
    RANDOMIZED_IDLE = auto()
    MAIN_BOT_SIMPLE_SEQUENCE = auto()
