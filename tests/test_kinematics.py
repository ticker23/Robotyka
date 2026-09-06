import math

import pytest

from anti_drone.kinematics import forward_kinematics, is_reachable


@pytest.mark.parametrize(
    ("q1", "q2", "l1", "l2", "expected"),
    [
        (0.0, 0.0, 1.0, 1.0, (2.0, 0.0)),
        (math.pi / 2, 0.0, 1.0, 1.0, (0.0, 2.0)),
        (
            math.pi / 4,
            -math.pi / 2,
            2.0,
            1.0,
            ((3 * math.sqrt(2)) / 2, math.sqrt(2) / 2),
        ),
    ],
)
def test_forward_kinematics(
    q1: float, q2: float, l1: float, l2: float, expected: tuple[float, float]
) -> None:
    x, y = forward_kinematics(q1, q2, l1, l2)

    assert x == pytest.approx(expected[0])
    assert y == pytest.approx(expected[1])


@pytest.mark.parametrize(
    ("l1", "l2"),
    [
        (0.0, 1.0),
        (1.0, 0.0),
        (-1.0, 1.0),
        (1.0, -1.0),
    ],
)
def test_forward_kinematics_invalid_link_lengths(l1: float, l2: float) -> None:
    with pytest.raises(ValueError, match="link lengths must be greater than zero"):
        forward_kinematics(0.0, 0.0, l1, l2)


@pytest.mark.parametrize(
    ("x", "y", "l1", "l2", "expected"),
    [
        (1.0, 1.0, 1.0, 1.0, True),
        (2.0, 0.0, 1.0, 1.0, True),
        (0.0, 0.0, 2.0, 2.0, True),
        (3.0, 0.0, 1.0, 1.0, False),
        (0.25, 0.0, 2.0, 1.0, False),
    ],
)
def test_is_reachable(x: float, y: float, l1: float, l2: float, expected: bool) -> None:
    assert is_reachable(x, y, l1, l2) is expected


@pytest.mark.parametrize(
    ("l1", "l2"),
    [
        (0.0, 1.0),
        (1.0, 0.0),
        (-1.0, 1.0),
        (1.0, -1.0),
    ],
)
def test_is_reachable_invalid_link_lengths(l1: float, l2: float) -> None:
    with pytest.raises(ValueError, match="link lengths must be greater than zero"):
        is_reachable(1.0, 0.0, l1, l2)
