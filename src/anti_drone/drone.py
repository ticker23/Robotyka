"""Simulated drone target state."""

from __future__ import annotations

from anti_drone.trajectory import Trajectory


class Drone:
    """A simulated drone whose true position is defined by a trajectory."""

    def __init__(self, trajectory: Trajectory) -> None:
        self.trajectory = trajectory
        self._position = trajectory.get_position(0.0)

    @property
    def position(self) -> tuple[float, float]:
        """Return the current true position of the drone."""

        return self._position

    def update(self, time: float) -> None:
        """Update the current position from the assigned trajectory."""

        self._position = self.trajectory.get_position(time)
