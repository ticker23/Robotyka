import math

import pytest

from anti_drone.trajectory import CircularTrajectory, LinearTrajectory


@pytest.mark.parametrize(
    ("trajectory", "time", "expected"),
    [
        (
            LinearTrajectory(start_position=(1.0, 2.0), velocity=(0.0, 0.0)),
            0.0,
            (1.0, 2.0),
        ),
        (
            LinearTrajectory(start_position=(1.0, 2.0), velocity=(3.0, -4.0)),
            2.0,
            (7.0, -6.0),
        ),
        (
            LinearTrajectory(start_position=(-1.0, 1.5), velocity=(-2.0, 0.5)),
            3.0,
            (-7.0, 3.0),
        ),
    ],
)
def test_linear_trajectory_positions(
    trajectory: LinearTrajectory, time: float, expected: tuple[float, float]
) -> None:
    assert trajectory.get_position(time) == pytest.approx(expected)


@pytest.mark.parametrize(
    "trajectory",
    [
        LinearTrajectory(start_position=(0.0, 0.0), velocity=(1.0, 1.0)),
        LinearTrajectory(start_position=(1.0, -1.0), velocity=(0.0, 0.0)),
    ],
)
def test_linear_trajectory_negative_time(trajectory: LinearTrajectory) -> None:
    with pytest.raises(ValueError, match="simulation time must not be negative"):
        trajectory.get_position(-0.1)


def test_linear_trajectory_is_deterministic() -> None:
    trajectory = LinearTrajectory(start_position=(2.0, -3.0), velocity=(0.5, 1.25))

    first = trajectory.get_position(4.0)
    second = trajectory.get_position(4.0)

    assert first == pytest.approx(second)


@pytest.mark.parametrize(
    ("trajectory", "time", "expected"),
    [
        (
            CircularTrajectory(center=(0.0, 0.0), radius=2.0, angular_velocity=0.0),
            0.0,
            (2.0, 0.0),
        ),
        (
            CircularTrajectory(center=(0.0, 0.0), radius=2.0, angular_velocity=1.0),
            math.pi / 2,
            (0.0, 2.0),
        ),
        (
            CircularTrajectory(center=(0.0, 0.0), radius=2.0, angular_velocity=1.0),
            math.pi,
            (-2.0, 0.0),
        ),
        (
            CircularTrajectory(center=(1.0, -1.0), radius=3.0, angular_velocity=1.0),
            math.pi / 2,
            (1.0, 2.0),
        ),
        (
            CircularTrajectory(
                center=(1.0, 2.0),
                radius=2.0,
                angular_velocity=1.0,
                phase=math.pi / 2,
            ),
            0.0,
            (1.0, 4.0),
        ),
        (
            CircularTrajectory(center=(0.0, 0.0), radius=2.0, angular_velocity=-1.0),
            math.pi / 2,
            (0.0, -2.0),
        ),
        (
            CircularTrajectory(
                center=(0.0, 0.0), radius=2.0, angular_velocity=0.0, phase=math.pi / 3
            ),
            4.0,
            (1.0, math.sqrt(3.0)),
        ),
    ],
)
def test_circular_trajectory_positions(
    trajectory: CircularTrajectory, time: float, expected: tuple[float, float]
) -> None:
    assert trajectory.get_position(time) == pytest.approx(expected)


def test_circular_trajectory_invalid_radius() -> None:
    with pytest.raises(ValueError, match="radius must be greater than zero"):
        CircularTrajectory(center=(0.0, 0.0), radius=0.0, angular_velocity=1.0)


@pytest.mark.parametrize(
    "trajectory",
    [
        CircularTrajectory(center=(0.0, 0.0), radius=1.0, angular_velocity=0.0),
        CircularTrajectory(center=(1.0, 1.0), radius=2.0, angular_velocity=-1.0, phase=0.5),
    ],
)
def test_circular_trajectory_negative_time(trajectory: CircularTrajectory) -> None:
    with pytest.raises(ValueError, match="simulation time must not be negative"):
        trajectory.get_position(-1.0)


def test_circular_trajectory_is_deterministic() -> None:
    trajectory = CircularTrajectory(
        center=(1.0, 2.0), radius=3.0, angular_velocity=0.75, phase=0.25
    )

    first = trajectory.get_position(1.5)
    second = trajectory.get_position(1.5)

    assert first == pytest.approx(second)
