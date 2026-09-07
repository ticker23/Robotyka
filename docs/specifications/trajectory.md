# Trajectory Module Specification

## 1. Purpose

The trajectory module defines deterministic movement paths for the
simulated drone.

A trajectory maps simulation time to a two-dimensional Cartesian
position.

The module is independent from:

- drone state,
- sensors,
- robot control,
- kinematics,
- simulation orchestration.

Its sole responsibility is generating positions based on time and
trajectory parameters.

---

## 2. Coordinate System

All positions use the global two-dimensional Cartesian coordinate system
defined in `docs/architecture.md`.

The coordinate format is:

```text
(x, y)
```

where:

- `x` represents the horizontal coordinate,
- `y` represents the vertical coordinate.

Positions are represented as:

```python
tuple[float, float]
```

Time is represented in seconds.

---

## 3. Module Interface

All trajectory implementations must expose a common interface.

Conceptually:

```text
time
 │
 ▼
Trajectory
 │
 ▼
position (x, y)
```

The common contract is:

```python
get_position(time: float) -> tuple[float, float]
```

A trajectory must return the position corresponding to the supplied
simulation time.

Trajectory implementations must not modify external state.

For identical parameters and identical time input, the returned position
must always be identical.

---

## 4. Trajectory Protocol

The module should define a structural interface using `typing.Protocol`.

Conceptually:

```python
class Trajectory(Protocol):
    def get_position(
        self,
        time: float,
    ) -> tuple[float, float]: ...
```

Concrete trajectory classes are not required to inherit explicitly from
the protocol.

Any object implementing the required interface is considered compatible.

---

# 5. Linear Trajectory

## 5.1 Purpose

The linear trajectory represents constant velocity movement in the
two-dimensional workspace.

The position is calculated from an initial position and a constant
velocity vector.

---

## 5.2 Parameters

The trajectory requires:

```text
start_position
velocity
```

Where:

```text
start_position = (x0, y0)

velocity = (vx, vy)
```

---

## 5.3 Mathematical Model

The position at time `t` is defined as:

```text
x(t) = x0 + vx × t

y(t) = y0 + vy × t
```

Where:

- `x0`, `y0` are the initial coordinates,
- `vx`, `vy` are velocity components,
- `t` is simulation time.

---

## 5.4 Conceptual API

```python
LinearTrajectory(
    start_position: tuple[float, float],
    velocity: tuple[float, float],
)
```

Position retrieval:

```python
trajectory.get_position(time)
```

---

## 5.5 Behaviour

At:

```text
t = 0
```

the returned position must equal:

```text
start_position
```

Positive and negative velocity components are valid.

A velocity of:

```text
(0, 0)
```

is valid and represents a stationary target.

---

## 5.6 Validation

Simulation time must not be negative.

A negative time value must raise:

```text
ValueError
```

---

# 6. Circular Trajectory

## 6.1 Purpose

The circular trajectory represents movement around a fixed center point.

The target moves along a circle with constant angular velocity.

---

## 6.2 Parameters

The trajectory requires:

```text
center
radius
angular_velocity
phase
```

Where:

```text
center = (cx, cy)

radius = r

angular_velocity = ω

phase = φ
```

The phase parameter defaults to:

```text
0.0
```

---

## 6.3 Mathematical Model

The position at time `t` is defined as:

```text
x(t) = cx + r cos(ωt + φ)

y(t) = cy + r sin(ωt + φ)
```

Where:

- `cx`, `cy` are the center coordinates,
- `r` is the radius,
- `ω` is angular velocity in radians per second,
- `φ` is the initial phase in radians,
- `t` is simulation time.

---

## 6.4 Direction of Movement

Angular velocity determines rotation direction.

```text
ω > 0
```

produces counter-clockwise movement.

```text
ω < 0
```

produces clockwise movement.

```text
ω = 0
```

is valid and represents a stationary target at a fixed position on the
circle.

---

## 6.5 Conceptual API

```python
CircularTrajectory(
    center: tuple[float, float],
    radius: float,
    angular_velocity: float,
    phase: float = 0.0,
)
```

Position retrieval:

```python
trajectory.get_position(time)
```

---

## 6.6 Validation

The radius must satisfy:

```text
radius > 0
```

A zero or negative radius must raise:

```text
ValueError
```

Simulation time must not be negative.

A negative time value must raise:

```text
ValueError
```

Negative angular velocity is valid.

Zero angular velocity is valid.

---

# 7. Determinism

Trajectory implementations must be deterministic.

For identical:

- trajectory parameters,
- simulation time,

the returned position must always be identical.

The MVP must not introduce random trajectory behaviour.

Random movement may be added in a future extension but must support
explicit reproducibility.

---

# 8. Error Handling

The trajectory module must explicitly reject invalid input.

Examples include:

- negative simulation time,
- invalid circular trajectory radius.

Invalid input must not be silently corrected.

The module should raise `ValueError` for invalid trajectory parameters
or invalid time values.

---

# 9. Module Boundaries

The trajectory module must not contain:

- drone state,
- sensor logic,
- measurement noise,
- robot logic,
- inverse or forward kinematics,
- PID control,
- simulation loop logic,
- visualization logic.

The module is responsible only for:

```text
trajectory parameters + time
            │
            ▼
       position (x, y)
```

---

# 10. Expected Implementations

The MVP requires:

```text
trajectory.py
│
├── Trajectory
│
├── LinearTrajectory
│
└── CircularTrajectory
```

Additional trajectory types may be introduced later.

Potential future implementations include:

- sinusoidal trajectory,
- elliptical trajectory,
- waypoint trajectory,
- Bézier trajectory,
- random trajectory.

These are outside the current MVP scope.

---

# 11. Testing Requirements

The trajectory module must be independently testable.

## LinearTrajectory

Tests should cover:

- position at time zero,
- positive velocity,
- negative velocity,
- movement in both axes,
- stationary trajectory,
- negative time validation.

---

## CircularTrajectory

Tests should cover:

- position at time zero,
- quarter rotation,
- half rotation,
- custom center position,
- custom phase,
- clockwise rotation,
- zero angular velocity,
- invalid radius,
- negative time validation.

Floating-point comparisons must use appropriate approximate comparison.

For pytest:

```python
pytest.approx()
```

should be used where necessary.

---

# 12. Acceptance Criteria

The module is considered complete when:

- `Trajectory` protocol is defined,
- `LinearTrajectory` is implemented,
- `CircularTrajectory` is implemented,
- all required validation is implemented,
- unit tests cover normal and boundary behaviour,
- the module has no dependencies on higher-level system components,
- the complete project test suite passes,
- formatting and static checks pass.

---

# 13. Future Integration

The trajectory module will be used by the drone component.

The expected relationship is:

```text
Simulation Time
       │
       ▼
   Trajectory
       │
       ▼
 position(t)
       │
       ▼
     Drone
```

The drone component will own its current state.

The trajectory component remains stateless and is responsible only for
calculating positions.

This separation allows new trajectory implementations to be introduced
without modifying the drone component.
