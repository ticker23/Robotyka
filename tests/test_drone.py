import math

import pytest

from anti_drone.drone import Drone
from anti_drone.trajectory import CircularTrajectory, LinearTrajectory


class ConstantTrajectory:
    def __init__(self, position: tuple[float, float]) -> None:
        self.position = position
        self.calls: list[float] = []

    def get_position(self, time: float) -> tuple[float, float]:
        self.calls.append(time)
        return self.position


class AbsoluteTimeTrajectory:
    def __init__(self) -> None:
        self.calls: list[float] = []

    def get_position(self, time: float) -> tuple[float, float]:
        self.calls.append(time)
        if time < 0:
            raise ValueError("invalid time")
        return time, -time


def test_drone_initializes_with_valid_trajectory() -> None:
    trajectory = ConstantTrajectory((2.0, -1.0))

    drone = Drone(trajectory)

    assert drone.trajectory is trajectory
    assert drone.position == pytest.approx((2.0, -1.0))


def test_drone_initial_position_comes_from_trajectory_at_zero() -> None:
    trajectory = AbsoluteTimeTrajectory()

    drone = Drone(trajectory)

    assert trajectory.calls == [0.0]
    assert drone.position == pytest.approx((0.0, 0.0))


@pytest.mark.parametrize(
    ("time", "expected"),
    [
        (1.0, (1.0, -1.0)),
        (2.5, (2.5, -2.5)),
        (10.0, (10.0, -10.0)),
    ],
)
def test_drone_updates_position_for_absolute_simulation_times(
    time: float, expected: tuple[float, float]
) -> None:
    drone = Drone(AbsoluteTimeTrajectory())

    drone.update(time)

    assert drone.position == pytest.approx(expected)


def test_drone_follows_linear_trajectory() -> None:
    trajectory = LinearTrajectory(start_position=(1.0, -2.0), velocity=(0.5, 3.0))
    drone = Drone(trajectory)

    drone.update(4.0)

    assert drone.position == pytest.approx((3.0, 10.0))


def test_drone_follows_circular_trajectory() -> None:
    trajectory = CircularTrajectory(center=(1.0, 2.0), radius=3.0, angular_velocity=1.0)
    drone = Drone(trajectory)

    drone.update(math.pi / 2)

    assert drone.position == pytest.approx((1.0, 5.0))


def test_multiple_updates_replace_previous_position() -> None:
    drone = Drone(AbsoluteTimeTrajectory())

    drone.update(1.0)
    assert drone.position == pytest.approx((1.0, -1.0))

    drone.update(3.0)
    assert drone.position == pytest.approx((3.0, -3.0))


def test_repeated_updates_with_same_time_are_deterministic() -> None:
    drone = Drone(AbsoluteTimeTrajectory())

    drone.update(2.0)
    first_position = drone.position
    drone.update(2.0)
    second_position = drone.position

    assert first_position == pytest.approx((2.0, -2.0))
    assert second_position == pytest.approx(first_position)


def test_invalid_time_error_propagates_from_trajectory() -> None:
    drone = Drone(AbsoluteTimeTrajectory())

    with pytest.raises(ValueError, match="invalid time"):
        drone.update(-1.0)


def test_drone_uses_trajectory_contract_not_specific_implementation() -> None:
    trajectory = AbsoluteTimeTrajectory()
    drone = Drone(trajectory)

    drone.update(5.0)

    assert trajectory.calls == [0.0, 5.0]
    assert drone.position == pytest.approx((5.0, -5.0))
