## Implementation Phases

The kinematics module will be implemented incrementally.

### Phase 1

Implement:

- `forward_kinematics()`
- `is_reachable()`

### Phase 2

Implement:

- `inverse_kinematics()`

The full module specification remains the target architecture,
while individual implementation tasks may cover only part of the module.
## Status

Planned

## Purpose

This document specifies the initial kinematics module for the Anti-Drone
System simulation.

The module provides mathematical functions for a planar robotic
manipulator with two revolute joints (2-DOF).

The module is responsible only for kinematic calculations.

It must not contain:

- simulation logic,
- PID control,
- sensor logic,
- visualization,
- trajectory generation.

---

# 1. Robot Model

The robot consists of two rigid links connected by two revolute joints.

```text
                  End Effector
                       P(x, y)
                          ●
                         /
                        /
                     L2/
                      /
                     ● Joint 2
                    /
                   /
                L1/
                 /
                ● Base
               (0,0)
````

Parameters:

* `L1` — length of the first link,
* `L2` — length of the second link,
* `q1` — angle of the first joint relative to the positive X-axis,
* `q2` — relative angle of the second joint relative to the first link.

The orientation of the second link is:

```text
q1 + q2
```

---

# 2. Coordinate System

The robot base is located at:

```text
(0, 0)
```

Coordinate convention:

```text
        Y
        ↑
        │
        │
        │
        └────────────→ X
```

Rules:

* positive X points right,
* positive Y points up,
* positive angles rotate counter-clockwise,
* all angles are represented internally in radians.

---

# 3. Module Location

The implementation must be placed in:

```text
src/anti_drone/kinematics.py
```

Tests must be placed in:

```text
tests/test_kinematics.py
```

---

# 4. Required Functions

The initial module must provide:

```python
forward_kinematics()
inverse_kinematics()
is_reachable()
```

---

# 5. Forward Kinematics

## Purpose

Calculate the Cartesian position of the end effector from joint angles.

Input:

```text
q1
q2
L1
L2
```

Output:

```text
x
y
```

Mathematical model:

```text
x = L1 * cos(q1) + L2 * cos(q1 + q2)

y = L1 * sin(q1) + L2 * sin(q1 + q2)
```

Proposed API:

```python
def forward_kinematics(
    q1: float,
    q2: float,
    l1: float,
    l2: float,
) -> tuple[float, float]:
```

The function must:

* use radians,
* return the end-effector position `(x, y)`,
* not modify input values,
* contain no simulation logic.

---

# 6. Target Reachability

## Purpose

Determine whether a Cartesian target can be reached by the manipulator.

The target distance from the base is:

```text
r = sqrt(x² + y²)
```

A target is reachable when:

```text
|L1 - L2| ≤ r ≤ L1 + L2
```

Proposed API:

```python
def is_reachable(
    x: float,
    y: float,
    l1: float,
    l2: float,
) -> bool:
```

The function must return:

```text
True
```

when the target is inside the reachable workspace.

Otherwise:

```text
False
```

---

# 7. Inverse Kinematics

## Purpose

Calculate joint angles required to reach a Cartesian target position.

Input:

```text
x
y
L1
L2
```

Output:

```text
q1
q2
```

The implementation must support two valid manipulator configurations:

```text
elbow-up
elbow-down
```

Proposed API:

```python
from typing import Literal


def inverse_kinematics(
    x: float,
    y: float,
    l1: float,
    l2: float,
    elbow: Literal["up", "down"] = "up",
) -> tuple[float, float]:
```

---

## Mathematical Model

Define:

```text
D = (x² + y² - L1² - L2²) / (2 * L1 * L2)
```

The second joint angle is:

```text
q2 = ±acos(D)
```

The sign depends on the selected elbow configuration.

The first joint angle is:

```text
q1 = atan2(y, x)
     - atan2(
         L2 * sin(q2),
         L1 + L2 * cos(q2)
       )
```

The implementation must account for floating-point precision.

Before calling `acos`, the calculated value of `D` should be constrained
to the valid range:

```text
[-1, 1]
```

---

# 8. Unreachable Targets

If a target is outside the robot workspace,
`inverse_kinematics()` must raise:

```python
ValueError
```

The kinematics module is responsible only for detecting the mathematical
impossibility of the requested position.

The higher-level simulation layer will decide how the system responds to
an unreachable target.

---

# 9. Input Validation

The initial implementation should validate:

* link lengths must be greater than zero,
* `elbow` must be either `"up"` or `"down"`.

Invalid input should raise an appropriate exception.

Do not add unnecessary validation unrelated to the module responsibility.

---

# 10. Testing Requirements

Tests must be implemented in:

```text
tests/test_kinematics.py
```

The test suite should cover at minimum:

## Forward Kinematics

* zero angles,
* known configurations,
* vertical configuration.

Example:

```text
L1 = 1
L2 = 1

q1 = 0
q2 = 0

Expected:

x = 2
y = 0
```

---

## Reachability

Test:

* reachable target,
* target outside maximum reach,
* target inside minimum reach,
* boundary positions.

---

## Inverse Kinematics

Test:

* known reachable target,
* elbow-up configuration,
* elbow-down configuration,
* unreachable target.

---

## FK → IK → FK Roundtrip

For a reachable target:

```text
Target Position
       ↓
Inverse Kinematics
       ↓
Joint Angles
       ↓
Forward Kinematics
       ↓
Reconstructed Position
```

The reconstructed position must match the original target within
floating-point tolerance.

---

# 11. Non-Goals

The following are explicitly outside the scope of this task:

* joint velocity calculation,
* robot dynamics,
* joint limits,
* collision detection,
* obstacle avoidance,
* trajectory planning,
* PID control,
* visualization,
* simulation loop,
* 3D kinematics.

Do not implement these features.

---

# 12. Acceptance Criteria

The task is complete when:

* `src/anti_drone/kinematics.py` exists,
* all three required functions are implemented,
* the implementation follows the mathematical model in this document,
* unreachable targets are handled correctly,
* invalid input is handled appropriately,
* tests cover the specified behavior,
* all tests pass,
* no unrelated modules are modified,
* no unnecessary dependencies are added.


