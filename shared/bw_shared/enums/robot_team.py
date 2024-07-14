# When adding fields to this file, make sure to update the bw_shared_config C++ module as well
from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class RobotTeam(str, EnumAutoLowerStr):
    OUR_TEAM = auto()
    THEIR_TEAM = auto()
    REFEREE = auto()
