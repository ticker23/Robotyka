"""Deterministic trajectory definitions for the simulated drone."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin
from typing import Protocol


class Trajectory(Protocol):
    """Structural interface for time-based trajectories."""

    def get_position(self, time: float) -> tuple[float, float]:
        """Return the position for the supplied simulation time."""


def _validate_time(time: float) -> None:
    """Reject negative simulation time values."""

    if time < 0:
        raise ValueError("simulation time must not be negative")


@dataclass(frozen=True, slots=True)
class LinearTrajectory:
    """A deterministic linear trajectory with constant velocity."""

    start_position: tuple[float, float]
    velocity: tuple[float, float]

    def get_position(self, time: float) -> tuple[float, float]:
        """Return the position at the specified simulation time."""

        _validate_time(time)

        x0, y0 = self.start_position
        vx, vy = self.velocity
        return x0 + vx * time, y0 + vy * time


@dataclass(frozen=True, slots=True)
class CircularTrajectory:
    """A deterministic circular trajectory with constant angular velocity."""

    center: tuple[float, float]
    radius: float
    angular_velocity: float
    phase: float = 0.0

    def __post_init__(self) -> None:
        if self.radius <= 0:
            raise ValueError("radius must be greater than zero")

    def get_position(self, time: float) -> tuple[float, float]:
        """Return the position at the specified simulation time."""

        _validate_time(time)

        cx, cy = self.center
        angle = self.angular_velocity * time + self.phase
        return cx + self.radius * cos(angle), cy + self.radius * sin(angle)
