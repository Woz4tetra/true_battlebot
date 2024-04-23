from dataclasses import asdict, dataclass


@dataclass
class UVKeypoint:
    x: int
    y: int

    def to_dict(self):
        return asdict(self)
