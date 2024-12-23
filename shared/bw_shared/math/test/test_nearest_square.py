import pytest
from bw_shared.math.nearest_square import nearest_square


@pytest.mark.parametrize(
    ("value",),
    [
        (-10,),
        (-0.5,),
        (float("inf"),),
        (float("nan"),),
        (float("-inf"),),
        ("a",),
    ],
)
def test_bad_values(value: float) -> None:
    with pytest.raises((ValueError, TypeError, OverflowError)):
        nearest_square(value)


@pytest.mark.parametrize(
    ("value", "expected_lower", "expected_upper"),
    [
        (0, 0, 0),
        (1, 1, 1),
        (2, 1, 4),
        (3, 1, 4),
        (4, 4, 4),
        (5, 4, 9),
        (6, 4, 9),
        (7, 4, 9),
        (8, 4, 9),
        (9, 9, 9),
        (10, 9, 16),
        (11, 9, 16),
        (12, 9, 16),
        (13, 9, 16),
        (14, 9, 16),
        (15, 9, 16),
        (16, 16, 16),
        (17, 16, 25),
        (18, 16, 25),
        (19, 16, 25),
        (20, 16, 25),
        (21, 16, 25),
        (22, 16, 25),
        (23, 16, 25),
        (24, 16, 25),
        (25, 25, 25),
        (26, 25, 36),
        (27, 25, 36),
        (28, 25, 36),
        (29, 25, 36),
        (30, 25, 36),
        (31, 25, 36),
        (32, 25, 36),
        (33, 25, 36),
        (34, 25, 36),
        (35, 25, 36),
        (36, 36, 36),
        (37, 36, 49),
        (38, 36, 49),
        (39, 36, 49),
        (40, 36, 49),
        (41, 36, 49),
        (42, 36, 49),
        (43, 36, 49),
        (44, 36, 49),
        (45, 36, 49),
        (46, 36, 49),
        (47, 36, 49),
        (48, 36, 49),
        (49, 49, 49),
        (50, 49, 64),
        (51, 49, 64),
    ],
)
def test_nearest_square(value: float, expected_lower: int, expected_upper: int) -> None:
    assert nearest_square(value, False) == expected_lower
    assert nearest_square(value, True) == expected_upper
