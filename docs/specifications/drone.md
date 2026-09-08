# Drone Module Specification

## 1. Purpose

The drone module represents the moving aerial target within the
simulation environment.

The drone owns the current state of the simulated target and updates its
position according to an assigned trajectory.

The drone does not calculate its movement independently.

Instead, the movement path is provided by a trajectory object.

The relationship between the components is:

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
       │
       ▼
Current Drone Position
```

The drone module is responsible for maintaining the current true
position of the target in the simulation.

---

## 2. Module Responsibilities

The drone module is responsible for:

- storing the assigned trajectory,
- maintaining the current drone position,
- updating the position using simulation time,
- exposing the current true position.

The drone module must not contain:

- sensor logic,
- measurement noise,
- target detection logic,
- robot control,
- inverse kinematics,
- forward kinematics,
- PID control,
- simulation loop logic,
- visualization logic.

The drone represents the target state only.

---

## 3. Dependencies

The drone module depends on the trajectory contract defined in:

```text
docs/specifications/trajectory.md
```

The drone must work with any object implementing the `Trajectory`
protocol.

The drone must not depend on specific trajectory implementations.

For example, the following objects should be compatible:

```text
LinearTrajectory
CircularTrajectory
FutureTrajectory
```

The drone interacts only through the common interface:

```python
get_position(time: float) -> tuple[float, float]
```

---

## 4. Data Model

The drone maintains the following conceptual state:

```text
Drone
│
├── trajectory
│
└── position
```

Where:

```text
trajectory
```

is an object implementing the `Trajectory` protocol.

And:

```text
position
```

represents the current true position of the drone.

The position format is:

```python
tuple[float, float]
```

representing:

```text
(x, y)
```

All coordinates use the global Cartesian coordinate system defined in:

```text
docs/architecture.md
```

---

## 5. Initialization

A drone requires a trajectory during initialization.

Conceptually:

```python
drone = Drone(trajectory)
```

The drone must initialize its position using the trajectory at simulation
time:

```text
t = 0.0
```

Conceptually:

```python
initial_position = trajectory.get_position(0.0)
```

Therefore, immediately after initialization:

```python
drone.position
```

must represent the valid initial position of the assigned trajectory.

The drone must not initialize with:

```python
position = None
```

The object should always remain in a valid state.

---

## 6. Position Update

The drone position is updated using simulation time.

Conceptually:

```python
drone.update(time)
```

The update operation must perform:

```text
trajectory.get_position(time)
```

and assign the returned value as the current drone position.

Conceptually:

```text
time
 │
 ▼
Drone.update(time)
 │
 ▼
Trajectory.get_position(time)
 │
 ▼
new position
 │
 ▼
Drone.position
```

The drone must not modify the trajectory.

The drone must not calculate movement independently.

The trajectory remains the source of truth for the target position.

---

## 7. Update Behaviour

For a given simulation time:

```text
t
```

the drone position must satisfy:

```text
drone.position = trajectory.get_position(t)
```

The update operation must therefore be deterministic when used with a
deterministic trajectory.

Calling:

```python
drone.update(5.0)
```

multiple times must produce the same drone position, assuming the
trajectory parameters remain unchanged.

---

## 8. Time Handling

The drone module does not manage simulation time.

Time is provided externally by the simulation system.

The drone receives an absolute simulation time value.

Example:

```text
Simulation Time

0.0
0.1
0.2
0.3
0.4
...
```

The simulation system may call:

```python
drone.update(current_time)
```

The drone must not:

- increment time internally,
- store simulation time as its own clock,
- calculate time steps,
- manage frame rate,
- control the simulation loop.

Time management belongs to the simulation module.

---

## 9. True Position

The position maintained by the drone represents the true position of the
simulated target.

Conceptually:

```text
Drone
  │
  ▼
True Position
```

This value represents the actual target location before any sensor
measurement error is introduced.

The true position must not contain:

- sensor noise,
- measurement delay,
- detection errors.

These effects belong to the sensor module.

The expected future data flow is:

```text
Drone
  │
  ▼
True Position
  │
  ▼
Sensor
  │
  ▼
Measured Position
```

The distinction between true position and measured position must be
maintained throughout the system.

---

## 10. Public Interface

The conceptual public interface of the module is:

```python
class Drone:
    def __init__(self, trajectory: Trajectory) -> None:
        ...

    def update(self, time: float) -> None:
        ...

    @property
    def position(self) -> tuple[float, float]:
        ...
```

The exact internal implementation is not prescribed.

The public behaviour must satisfy this specification.

---

## 11. Immutability and State

The trajectory object defines the movement model.

The drone owns the current position state.

The relationship is:

```text
Trajectory
    │
    │ defines position as a function of time
    ▼
Drone
    │
    │ stores current state
    ▼
Current Position
```

The drone must not modify trajectory parameters.

For example, calling:

```python
drone.update(time)
```

must not modify:

```text
trajectory parameters
```

The update operation modifies only the drone's current position.

---

## 12. Error Handling

The drone module should not duplicate validation already guaranteed by
the trajectory module.

If the trajectory rejects an invalid time value, the resulting error may
propagate through the drone update operation.

For example:

```python
drone.update(-1.0)
```

may result in:

```text
ValueError
```

originating from the trajectory implementation.

The drone must not silently correct invalid simulation time.

The drone should not introduce unnecessary duplicate validation when the
underlying trajectory contract already defines the behaviour.

---

## 13. Module Boundaries

The drone module represents only the simulated target.

The following responsibilities belong to other modules:

```text
Trajectory
    Defines movement paths.

Sensor
    Measures the target position.

Kinematics
    Calculates robot geometry.

Controller
    Calculates control output.

Robot
    Maintains manipulator state and motion.

Simulation
    Coordinates the complete system.
```

The drone module must remain independent from all of these components
except the trajectory interface.

---

## 14. Expected File Structure

The implementation must be located in:

```text
src/anti_drone/drone.py
```

The module should contain:

```text
drone.py
│
└── Drone
```

No additional drone-related abstractions are required for the MVP.

---

## 15. Testing Requirements

The drone module must be independently testable.

Tests must be located in:

```text
tests/test_drone.py
```

The test suite should verify the following behaviour.

### Initialization

Tests should verify:

- a drone accepts a valid trajectory,
- the initial position equals the trajectory position at `t = 0.0`,
- the drone is immediately in a valid state after initialization.

---

### Position Updates

Tests should verify:

- position updates correctly for a given simulation time,
- the drone follows a linear trajectory,
- the drone follows a circular trajectory,
- multiple updates correctly replace the current position,
- repeated updates using the same time produce the expected position.

---

### Trajectory Abstraction

Tests should verify that the drone works with the trajectory contract
rather than depending on a specific trajectory implementation.

The drone should be testable using different valid trajectory objects.

---

### Invalid Time

Tests should verify that invalid time values are not silently accepted.

If invalid time handling is delegated to the trajectory implementation,
the corresponding exception should propagate correctly.

---

## 16. Acceptance Criteria

The drone module is considered complete when:

- `Drone` is implemented in `src/anti_drone/drone.py`,
- the drone accepts an object implementing the `Trajectory` contract,
- the initial position is derived from `trajectory.get_position(0.0)`,
- the drone exposes its current position,
- the drone updates its position using external simulation time,
- the drone does not manage time internally,
- the drone does not modify trajectory parameters,
- the drone does not contain sensor or robot logic,
- unit tests cover initialization and position updates,
- the complete project test suite passes,
- linting passes,
- formatting checks pass.

---

## 17. Future Extensions

The MVP drone model intentionally remains simple.

Potential future extensions include:

- drone velocity state,
- acceleration,
- orientation,
- altitude,
- flight dynamics,
- multiple drones,
- trajectory switching,
- target disappearance,
- target failure states.

These features are outside the scope of the current MVP.

The current implementation should not introduce abstractions for these
future features before they are required.

---

## 18. Integration

The drone module will be integrated into the simulation loop.

The expected data flow is:

```text
Simulation
    │
    │ current time
    ▼
Drone.update(time)
    │
    ▼
Trajectory.get_position(time)
    │
    ▼
Drone.position
    │
    ▼
Sensor.measure(position)
```

The drone provides the true target position to the rest of the system.

The sensor module will later transform the true position into a measured
position, potentially introducing detection limits and measurement noise.

This separation maintains a clear distinction between:

```text
Target Reality
     │
     ▼
Drone True Position
```

and:

```text
Sensor Observation
     │
     ▼
Measured Position
```

The drone module therefore acts as the bridge between the mathematical
trajectory definition and the simulated physical world.
