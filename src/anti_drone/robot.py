"""State model for the planar 2-DOF robot."""

from __future__ import annotations


class Robot:
    """A minimal two-joint robot state model."""

    def __init__(
        self,
        link_1: float,
        link_2: float,
        q1: float = 0.0,
        q2: float = 0.0,
    ) -> None:
        if link_1 <= 0:
            raise ValueError("link_1 must be greater than zero")
        if link_2 <= 0:
            raise ValueError("link_2 must be greater than zero")

        self.link_1 = link_1
        self.link_2 = link_2
        self.q1 = q1
        self.q2 = q2

    def update(self, q1_velocity: float, q2_velocity: float, dt: float) -> None:
        """Update joint angles from velocity commands over a positive timestep."""

        if dt <= 0:
            raise ValueError("dt must be greater than zero")

        self.q1 += q1_velocity * dt
        self.q2 += q2_velocity * dt
