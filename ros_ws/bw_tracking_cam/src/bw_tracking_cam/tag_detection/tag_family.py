from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class TagFamily(str, EnumAutoLowerStr):
    TAG16H5 = auto()
    TAG25H9 = auto()
    TAG36H10 = auto()
    TAG36H11 = auto()
