"""Planar kinematics for the 2-DOF anti-drone manipulator."""

from __future__ import annotations

from math import acos, atan2, cos, hypot, sin
from typing import Literal

_WORKSPACE_TOLERANCE = 1e-12


def _validate_link_lengths(l1: float, l2: float) -> None:
    """Validate manipulator link lengths.

    The initial specification requires both links to be strictly positive.
    """

    if l1 <= 0 or l2 <= 0:
        raise ValueError("link lengths must be greater than zero")


def forward_kinematics(q1: float, q2: float, l1: float, l2: float) -> tuple[float, float]:
    """Return the end-effector position for a planar 2-link manipulator.

    Parameters are expressed in radians and the Cartesian workspace follows
    the standard convention with the base at the origin.
    """

    _validate_link_lengths(l1, l2)

    x = l1 * cos(q1) + l2 * cos(q1 + q2)
    y = l1 * sin(q1) + l2 * sin(q1 + q2)
    return x, y


def is_reachable(x: float, y: float, l1: float, l2: float) -> bool:
    """Return whether a Cartesian target lies inside the manipulator workspace."""

    _validate_link_lengths(l1, l2)

    distance = hypot(x, y)
    inner_radius = abs(l1 - l2)
    outer_radius = l1 + l2

    return (
        (inner_radius - _WORKSPACE_TOLERANCE) <= distance <= (outer_radius + _WORKSPACE_TOLERANCE)
    )


def inverse_kinematics(
    x: float,
    y: float,
    l1: float,
    l2: float,
    elbow: Literal["up", "down"] = "up",
) -> tuple[float, float]:
    """Return one valid joint configuration for a reachable Cartesian target."""

    _validate_link_lengths(l1, l2)

    distance = hypot(x, y)
    inner_radius = abs(l1 - l2)
    outer_radius = l1 + l2

    if (
        distance < inner_radius - _WORKSPACE_TOLERANCE
        or distance > outer_radius + _WORKSPACE_TOLERANCE
    ):
        raise ValueError("target is outside the reachable workspace")

    if elbow not in {"up", "down"}:
        raise ValueError("elbow must be 'up' or 'down'")

    d = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    d = max(-1.0, min(1.0, d))

    q2 = acos(d)
    if elbow == "down":
        q2 = -q2

    q1 = atan2(y, x) - atan2(l2 * sin(q2), l1 + l2 * cos(q2))
    return q1, q2
