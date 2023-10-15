from typing import Tuple
from scipy.spatial.transform import Rotation


def from_quat(quat: Tuple[float, float, float, float]) -> Rotation:
    return Rotation.from_quat(quat)  # type: ignore
