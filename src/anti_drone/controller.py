"""Scalar PID controller for feedback control."""

from __future__ import annotations


class PIDController:
    """A minimal discrete-time PID controller."""

    def __init__(self, kp: float, ki: float, kd: float) -> None:
        if kp < 0:
            raise ValueError("kp must be greater than or equal to zero")
        if ki < 0:
            raise ValueError("ki must be greater than or equal to zero")
        if kd < 0:
            raise ValueError("kd must be greater than or equal to zero")

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral = 0.0
        self._previous_error: float | None = None

    def update(self, target: float, current: float, dt: float) -> float:
        """Return the PID output for the supplied target, current value, and timestep."""

        if dt <= 0:
            raise ValueError("dt must be greater than zero")

        error = target - current
        proportional = self.kp * error

        self._integral += error * dt
        integral_term = self.ki * self._integral

        if self._previous_error is None:
            derivative = 0.0
        else:
            derivative = (error - self._previous_error) / dt

        derivative_term = self.kd * derivative
        self._previous_error = error

        return proportional + integral_term + derivative_term

    def reset(self) -> None:
        """Reset dynamic controller state while preserving configured gains."""

        self._integral = 0.0
        self._previous_error = None
