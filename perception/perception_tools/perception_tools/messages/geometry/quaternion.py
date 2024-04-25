from dataclasses import dataclass


@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    def to_raw(self):
        return {"x": self.x, "y": self.y, "z": self.z, "w": self.w}

    @classmethod
    def from_raw(cls, msg):
        return cls(msg["x"], msg["y"], msg["z"], msg["w"])
