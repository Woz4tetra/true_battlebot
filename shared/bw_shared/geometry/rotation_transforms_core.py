import math
from typing import Dict, Tuple, Union

import numpy as np

Quaternion = Tuple[float, float, float, float]  # qx, qy, qz, qw
EulerAngles = Tuple[float, float, float]

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

AXIS_KEY = Union[str, Tuple[int, int, int, int]]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE: Dict[str, Tuple[int, int, int, int]] = {
    "sxyz": (0, 0, 0, 0),
    "sxyx": (0, 0, 1, 0),
    "sxzy": (0, 1, 0, 0),
    "sxzx": (0, 1, 1, 0),
    "syzx": (1, 0, 0, 0),
    "syzy": (1, 0, 1, 0),
    "syxz": (1, 1, 0, 0),
    "syxy": (1, 1, 1, 0),
    "szxy": (2, 0, 0, 0),
    "szxz": (2, 0, 1, 0),
    "szyx": (2, 1, 0, 0),
    "szyz": (2, 1, 1, 0),
    "rzyx": (0, 0, 0, 1),
    "rxyx": (0, 0, 1, 1),
    "ryzx": (0, 1, 0, 1),
    "rxzx": (0, 1, 1, 1),
    "rxzy": (1, 0, 0, 1),
    "ryzy": (1, 0, 1, 1),
    "rzxy": (1, 1, 0, 1),
    "ryxy": (1, 1, 1, 1),
    "ryxz": (2, 0, 0, 1),
    "rzxz": (2, 0, 1, 1),
    "rxyz": (2, 1, 0, 1),
    "rzyz": (2, 1, 1, 1),
}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())


def euler_matrix_core(roll: float, pitch: float, yaw: float, axes: AXIS_KEY = "sxyz") -> np.ndarray:
    """
    Ripped from tf_conversions.transformations for portability.
    Return homogeneous rotation matrix from Euler angles and axis sequence.

    euler_angles: Euler's roll, pitch and yaw angles in that order.
    axes: One of 24 axis sequences as string or encoded tuple

    >>> R = euler_matrix((1, 2, 3), 'syxz')
    >>> np.allclose(np.sum(R[0]), -1.34786452)
    True
    >>> R = euler_matrix((1, 2, 3), (0, 1, 0, 1))
    >>> np.allclose(np.sum(R[0]), -0.383436184)
    True
    >>> ai, aj, ak = (4.0*math.pi) * (np.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
            R = euler_matrix((ai, aj, ak), axes)
    >>> for axes in _TUPLE2AXES.keys():
            R = euler_matrix((ai, aj, ak), axes)

    """
    if isinstance(axes, str):
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
    else:
        _ = _TUPLE2AXES[axes]  # ensure key is valid. Throw exception otherwise.
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    if frame:
        roll, yaw = yaw, roll
    if parity:
        roll, pitch, yaw = -roll, -pitch, -yaw

    si, sj, sk = math.sin(roll), math.sin(pitch), math.sin(yaw)
    ci, cj, ck = math.cos(roll), math.cos(pitch), math.cos(yaw)
    cc, cs = ci * ck, ci * sk
    sc, ss = si * ck, si * sk

    mat = np.identity(4)
    if repetition:
        mat[i, i] = cj
        mat[i, j] = sj * si
        mat[i, k] = sj * ci
        mat[j, i] = sj * sk
        mat[j, j] = -cj * ss + cc
        mat[j, k] = -cj * cs - sc
        mat[k, i] = -sj * ck
        mat[k, j] = cj * sc + cs
        mat[k, k] = cj * cc - ss
    else:
        mat[i, i] = cj * ck
        mat[i, j] = sj * sc - cs
        mat[i, k] = sj * cc + ss
        mat[j, i] = cj * sk
        mat[j, j] = sj * ss + cc
        mat[j, k] = sj * cs - sc
        mat[k, i] = -sj
        mat[k, j] = cj * si
        mat[k, k] = cj * ci
    return mat


def euler_from_matrix_core(matrix: np.ndarray, axes: AXIS_KEY = "sxyz") -> EulerAngles:
    """
    Ripped from tf_conversions.transformations for portability.
    Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> np.allclose(R0, R1)
    True
    >>> angles = (4.0*math.pi) * (np.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not np.allclose(R0, R1): print axes, "failed"

    """
    if isinstance(axes, str):
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
    else:
        _ = _TUPLE2AXES[axes]  # ensure key is valid. Throw exception otherwise.
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    mat = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(mat[i, j] * mat[i, j] + mat[i, k] * mat[i, k])
        if sy > _EPS:
            ax = math.atan2(mat[i, j], mat[i, k])
            ay = math.atan2(sy, mat[i, i])
            az = math.atan2(mat[j, i], -mat[k, i])
        else:
            ax = math.atan2(-mat[j, k], mat[j, j])
            ay = math.atan2(sy, mat[i, i])
            az = 0.0
    else:
        cy = math.sqrt(mat[i, i] * mat[i, i] + mat[j, i] * mat[j, i])
        if cy > _EPS:
            ax = math.atan2(mat[k, j], mat[k, k])
            ay = math.atan2(-mat[k, i], cy)
            az = math.atan2(mat[j, i], mat[i, i])
        else:
            ax = math.atan2(-mat[j, k], mat[j, j])
            ay = math.atan2(-mat[k, i], cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az


def quaternion_matrix_core(quaternion: Quaternion) -> np.ndarray:
    """
    Ripped from tf_conversions.transformations for portability.
    Return homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix(Quaternion(x=0.06146124, y=0, z=0, w=0.99810947))
    >>> np.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """
    q = np.array(quaternion, dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array(
        (
            (1.0 - q[1, 1] - q[2, 2], q[0, 1] - q[2, 3], q[0, 2] + q[1, 3], 0.0),
            (q[0, 1] + q[2, 3], 1.0 - q[0, 0] - q[2, 2], q[1, 2] - q[0, 3], 0.0),
            (q[0, 2] - q[1, 3], q[1, 2] + q[0, 3], 1.0 - q[0, 0] - q[1, 1], 0.0),
            (0.0, 0.0, 0.0, 1.0),
        ),
        dtype=np.float64,
    )


def quaternion_from_matrix_core(matrix: np.ndarray) -> Quaternion:
    """
    Ripped from tf_conversions.transformations for portability.
    Return quaternion from rotation matrix.

    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R)
    >>> np.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
    True

    """
    q = np.empty((4,), dtype=np.float64)
    mat = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    t = np.trace(mat)
    if t > mat[3, 3]:
        q[3] = t
        q[2] = mat[1, 0] - mat[0, 1]
        q[1] = mat[0, 2] - mat[2, 0]
        q[0] = mat[2, 1] - mat[1, 2]
    else:
        i, j, k = 0, 1, 2
        if mat[1, 1] > mat[0, 0]:
            i, j, k = 1, 2, 0
        if mat[2, 2] > mat[i, i]:
            i, j, k = 2, 0, 1
        t = mat[i, i] - (mat[j, j] + mat[k, k]) + mat[3, 3]
        q[i] = t
        q[j] = mat[i, j] + mat[j, i]
        q[k] = mat[k, i] + mat[i, k]
        q[3] = mat[k, j] - mat[j, k]
    q *= 0.5 / math.sqrt(t * mat[3, 3])
    return tuple(q)


def euler_from_quaternion_core(quaternion: Quaternion, axes: AXIS_KEY = "sxyz") -> EulerAngles:
    """
    Ripped from tf_conversions.transformations for portability.
    Return Euler angles from quaternion for specified axis sequence.

    >>> angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(angles, [0.123, 0, 0])
    True
    """
    return euler_from_matrix_core(quaternion_matrix_core(quaternion), axes)


def quaternion_from_euler_core(roll: float, pitch: float, yaw: float, axes: AXIS_KEY = "sxyz") -> Quaternion:
    """
    Ripped from tf_conversions.transformations for portability.
    Return quaternion from Euler angles and axis sequence.

    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple

    >>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
    >>> numpy.allclose(q, [0.310622, -0.718287, 0.444435, 0.435953])
    True
    """
    if isinstance(axes, str):
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    else:
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    if frame:
        roll, yaw = yaw, roll
    if parity:
        pitch = -pitch

    roll = roll / 2.0
    pitch = pitch / 2.0
    yaw = yaw / 2.0
    ci = math.cos(roll)
    si = math.sin(roll)
    cj = math.cos(pitch)
    sj = math.sin(pitch)
    ck = math.cos(yaw)
    sk = math.sin(yaw)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    quaternion = np.empty((4,), dtype=np.float64)
    if repetition:
        quaternion[i] = cj * (cs + sc)
        quaternion[j] = sj * (cc + ss)
        quaternion[k] = sj * (cs - sc)
        quaternion[3] = cj * (cc - ss)
    else:
        quaternion[i] = cj * sc - sj * cs
        quaternion[j] = cj * ss + sj * cc
        quaternion[k] = cj * cs - sj * sc
        quaternion[3] = cj * cc + sj * ss
    if parity:
        quaternion[j] *= -1

    return tuple(quaternion)
