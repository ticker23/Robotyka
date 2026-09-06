"""Planar kinematics for the 2-DOF anti-drone manipulator."""

from __future__ import annotations

from math import cos, hypot, sin


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
    tolerance = 1e-12

    return (inner_radius - tolerance) <= distance <= (outer_radius + tolerance)
