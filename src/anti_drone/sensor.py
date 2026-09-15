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


class TargetTracker:
    def __init__(self) -> None:
        self.current_position: tuple[float, float] | None = None
        self.previous_position: tuple[float, float] | None = None
        self.velocity: tuple[float, float] | None = None

    def update(self, measured_position: tuple[float, float], dt: float) -> None:
        if dt <= 0:
            raise ValueError("dt must be greater than zero")

        self.previous_position = self.current_position
        self.current_position = measured_position

        if self.previous_position is None:
            return

        dx = self.current_position[0] - self.previous_position[0]
        dy = self.current_position[1] - self.previous_position[1]

        vx = dx / dt
        vy = dy / dt
        self.velocity = (vx, vy)

    def prediction(self, prediction_time: float) -> tuple[float, float]:
        if self.velocity is None or self.current_position is None:
            return None

        if prediction_time < 0:
            raise ValueError("prediction_time must not be negative")

        pred_x = self.current_position[0] + self.velocity[0] * prediction_time
        pred_y = self.current_position[1] + self.velocity[1] * prediction_time
        return pred_x, pred_y
