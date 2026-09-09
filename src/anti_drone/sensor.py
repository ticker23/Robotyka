"""Position sensor model for the two-dimensional simulation."""

from __future__ import annotations

from math import hypot
from random import gauss


class Sensor:
    """A fixed-position range sensor with optional Gaussian measurement noise."""

    def __init__(
        self,
        position: tuple[float, float],
        detection_range: float,
        noise_std: float = 0.0,
    ) -> None:
        if detection_range <= 0:
            raise ValueError("detection_range must be greater than zero")
        if noise_std < 0:
            raise ValueError("noise_std must be greater than or equal to zero")

        self.position = position
        self.detection_range = detection_range
        self.noise_std = noise_std

    def is_detected(self, target_position: tuple[float, float]) -> bool:
        """Return whether a target position lies within the detection range."""

        sensor_x, sensor_y = self.position
        target_x, target_y = target_position
        distance = hypot(target_x - sensor_x, target_y - sensor_y)
        return distance <= self.detection_range

    def measure(self, target_position: tuple[float, float]) -> tuple[float, float] | None:
        """Return the measured target position, or None when it is not detected."""

        if not self.is_detected(target_position):
            return None

        if self.noise_std == 0.0:
            return target_position

        target_x, target_y = target_position
        return (
            target_x + gauss(0.0, self.noise_std),
            target_y + gauss(0.0, self.noise_std),
        )
