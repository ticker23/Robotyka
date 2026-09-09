# Sensor Module Specification

## 1. Purpose

The sensor module simulates observation of the drone within the
two-dimensional simulation environment.

The sensor receives the true position of a target and attempts to produce
a measured position.

The sensor may:

- detect the target,
- fail to detect the target when it is outside its detection range,
- introduce measurement noise.

The sensor does not know about the drone object itself.

It operates only on target positions.

The expected data flow is:

```text
Drone
  │
  ▼
True Position
  │
  ▼
Sensor
  │
  ├── Detection Check
  │
  ├── Range Validation
  │
  └── Measurement Noise
        │
        ▼
Measured Position
```

The sensor module represents the difference between the true state of the
simulated world and the state observed by the control system.

---

## 2. Module Responsibilities

The sensor module is responsible for:

- storing its position,
- defining a detection range,
- determining whether a target is detectable,
- measuring target positions,
- optionally introducing measurement noise,
- returning the measured position.

The sensor module must not contain:

- drone state management,
- trajectory logic,
- robot logic,
- inverse kinematics,
- forward kinematics,
- PID control,
- simulation loop logic,
- visualization logic.

The sensor is an observation component.

It does not control the target or the robot.

---

## 3. Sensor Model

The MVP uses a simple two-dimensional position sensor.

The sensor is located at a fixed position:

```text
sensor_position = (sx, sy)
```

The sensor has a circular detection range:

```text
detection_range = r
```

The sensor detects a target when the Euclidean distance between the sensor
and the target is less than or equal to the configured detection range.

Conceptually:

```text
                    Detection Range

                .-----------------.
             .-'                   '-.
           /                           \
          |                             |
          |          Target ●           |
          |                             |
          |                             |
          |              ● Sensor       |
           \                           /
             '-.                   .-'
                '-----------------'
```

The sensor detection condition is:

```text
distance(sensor_position, target_position) <= detection_range
```

---

## 4. Coordinate System

All positions use the global two-dimensional Cartesian coordinate system
defined in:

```text
docs/architecture.md
```

Positions are represented as:

```python
tuple[float, float]
```

representing:

```text
(x, y)
```

The sensor position and target position must use the same coordinate
system.

---

## 5. Public Interface

The conceptual public interface is:

```python
class Sensor:
    def __init__(
        self,
        position: tuple[float, float],
        detection_range: float,
        noise_std: float = 0.0,
    ) -> None:
        ...

    def is_detected(
        self,
        target_position: tuple[float, float],
    ) -> bool:
        ...

    def measure(
        self,
        target_position: tuple[float, float],
    ) -> tuple[float, float] | None:
        ...
```

The exact internal implementation is not prescribed.

The public behaviour must follow this specification.

---

# 6. Sensor Initialization

A sensor requires:

```text
position
detection_range
```

The measurement noise standard deviation is optional.

The default value is:

```text
noise_std = 0.0
```

This represents an ideal sensor without measurement noise.

Conceptually:

```python
sensor = Sensor(
    position=(0.0, 0.0),
    detection_range=5.0,
)
```

A sensor with noise:

```python
sensor = Sensor(
    position=(0.0, 0.0),
    detection_range=5.0,
    noise_std=0.1,
)
```

---

# 7. Detection Range

The detection range defines the maximum distance at which a target can be
detected.

The distance is calculated using Euclidean distance.

For:

```text
sensor_position = (sx, sy)

target_position = (tx, ty)
```

the distance is:

```text
distance = sqrt(
    (tx - sx)^2 +
    (ty - sy)^2
)
```

A target is detected when:

```text
distance <= detection_range
```

A target exactly on the detection boundary is considered detected.

---

## 7.1 Detection Behaviour

The `is_detected()` method determines whether the target is inside the
sensor detection range.

Conceptually:

```text
Target Position
       │
       ▼
Calculate Distance
       │
       ▼
distance <= detection_range
       │
   ┌───┴────┐
   │        │
 False      True
   │         │
   ▼         ▼
Not       Detected
Detected
```

The detection operation must not modify sensor state.

The detection operation must be deterministic.

For identical sensor parameters and target position, the result must
always be identical.

---

# 8. Measurement

The `measure()` method attempts to measure the supplied target position.

The method performs the following logical steps:

```text
Target Position
       │
       ▼
Detection Check
       │
 ┌─────┴─────┐
 │           │
Not        Detected
Detected       │
 │             ▼
 ▼        Add Measurement Noise
None             │
                 ▼
          Measured Position
```

If the target is outside the detection range:

```python
sensor.measure(target_position)
```

must return:

```python
None
```

If the target is inside the detection range, the sensor must return a
measured position.

---

# 9. Ideal Measurement

When:

```text
noise_std = 0.0
```

the sensor behaves as an ideal position sensor.

For a detected target:

```text
true_position = (x, y)
```

the measurement must equal:

```text
measured_position = (x, y)
```

No modification to the coordinates is introduced.

Therefore:

```python
sensor.measure(position) == position
```

for any detected target when noise is zero.

---

# 10. Measurement Noise

The MVP sensor uses Gaussian measurement noise.

Noise is independently applied to both Cartesian coordinates.

Conceptually:

```text
measured_x = true_x + noise_x

measured_y = true_y + noise_y
```

Where:

```text
noise_x ~ N(0, noise_std)

noise_y ~ N(0, noise_std)
```

The noise distribution has:

```text
mean = 0
standard deviation = noise_std
```

The X and Y noise values must be generated independently.

---

## 10.1 Noise Behaviour

When:

```text
noise_std > 0
```

the measured position may differ from the true position.

The true target position must never be modified.

Only the returned measurement contains noise.

Conceptually:

```text
True Position
     │
     ├──────────────► remains unchanged
     │
     ▼
Sensor Noise
     │
     ▼
Measured Position
```

The sensor must not store noisy measurements as modifications of the
target position.

---

# 11. Randomness and Reproducibility

Measurement noise introduces randomness.

The sensor module may use the Python standard library random facilities.

The module must not introduce external dependencies for noise generation.

Tests involving random behaviour must be reproducible.

Unit tests should not depend on the probability of a particular random
value occurring.

Tests may:

- use `noise_std = 0.0` when deterministic measurements are required,
- patch or control the random generator when testing noise application.

The implementation must not rely on global randomness in a way that makes
tests unreliable.

---

# 12. Validation

The sensor constructor must validate its configuration.

## Detection Range

The detection range must satisfy:

```text
detection_range > 0
```

The following values are invalid:

```text
detection_range = 0

detection_range < 0
```

Invalid detection ranges must raise:

```python
ValueError
```

---

## Noise Standard Deviation

The noise standard deviation must satisfy:

```text
noise_std >= 0
```

The following value is valid:

```text
noise_std = 0
```

This represents an ideal sensor.

Negative noise values are invalid.

Invalid values must raise:

```python
ValueError
```

---

## Position Validation

The MVP assumes positions follow the project-wide coordinate format:

```python
tuple[float, float]
```

The sensor implementation is not required to introduce complex runtime
type validation beyond what is necessary for normal operation.

The module should remain lightweight.

---

# 13. Error Handling

Invalid sensor configuration must not be silently corrected.

The following examples must raise `ValueError`:

```text
detection_range <= 0

noise_std < 0
```

The sensor must not:

- clamp invalid ranges,
- convert negative noise values to positive values,
- silently replace invalid values with defaults.

Invalid configuration should fail explicitly.

---

# 14. Module Boundaries

The sensor operates independently from higher-level system components.

The sensor receives only a target position.

The sensor must not require:

```text
Drone object
Robot object
Simulation object
PID controller
Kinematics module
```

The expected interaction is:

```python
measurement = sensor.measure(drone.position)
```

However, the sensor itself does not know whether the supplied position
originates from:

- a drone,
- another target,
- a test object,
- a future simulation component.

This keeps the module reusable and independently testable.

---

# 15. Sensor Position

The MVP sensor position is fixed after initialization.

The sensor does not move.

The position represents the origin of the sensor detection area.

Conceptually:

```text
                   Detection Area

                     radius

                .---------------.
             .-'                 '-.
           /                         \
          |                           |
          |                           |
          |             ●             |
          |        Sensor Position    |
          |                           |
           \                         /
             '-.                 .-'
                '---------------'
```

Future implementations may introduce:

- moving sensors,
- sensors mounted on the robot,
- sensors attached to the end effector,
- directional sensors,
- field-of-view limitations.

These features are outside the MVP scope.

---

# 16. Expected File Structure

The implementation must be located in:

```text
src/anti_drone/sensor.py
```

The module should contain:

```text
sensor.py
│
└── Sensor
```

No additional sensor abstractions are required for the MVP.

---

# 17. Testing Requirements

The sensor module must be independently testable.

Tests must be located in:

```text
tests/test_sensor.py
```

The test suite should cover detection, measurement, noise behaviour and
validation.

---

## 17.1 Detection Tests

Tests should verify:

- a target at the sensor position is detected,
- a target inside the detection range is detected,
- a target exactly on the detection boundary is detected,
- a target outside the detection range is not detected,
- detection works correctly with non-zero sensor positions.

---

## 17.2 Ideal Measurement Tests

Tests should verify:

- a detected target is measured correctly when noise is zero,
- the returned measurement equals the true target position,
- an undetected target returns `None`,
- measurement does not modify the input position.

---

## 17.3 Noise Tests

Tests should verify:

- noise is applied when `noise_std > 0`,
- noise is applied independently to X and Y coordinates,
- the true input position remains unchanged.

Tests must not depend on random chance.

Random behaviour should be controlled using deterministic techniques such
as mocking the random generator where necessary.

---

## 17.4 Validation Tests

Tests should verify:

- zero detection range raises `ValueError`,
- negative detection range raises `ValueError`,
- negative noise standard deviation raises `ValueError`,
- zero noise standard deviation is valid.

---

# 18. Acceptance Criteria

The sensor module is considered complete when:

- `Sensor` is implemented in `src/anti_drone/sensor.py`,
- the sensor stores a fixed position,
- the sensor stores a detection range,
- targets inside the detection range are detected,
- targets outside the detection range are not detected,
- targets exactly on the detection boundary are detected,
- `measure()` returns `None` when a target is not detected,
- `measure()` returns the true position when noise is zero,
- Gaussian noise is applied when noise is greater than zero,
- noise is independently applied to X and Y coordinates,
- invalid sensor configuration raises `ValueError`,
- the module remains independent from the drone implementation,
- unit tests cover detection, measurement, noise and validation,
- the complete project test suite passes,
- linting passes,
- formatting checks pass.

---

# 19. Future Extensions

The MVP sensor model intentionally remains simple.

Potential future extensions include:

- measurement delay,
- limited update frequency,
- sensor failure,
- probabilistic detection,
- directional field of view,
- occlusion,
- moving sensors,
- sensors mounted on the robot,
- multiple sensors,
- sensor fusion.

These features must not be implemented until required by the project.

The MVP should remain focused on deterministic range detection with
optional Gaussian measurement noise.

---

# 20. Future Integration

The sensor will be integrated into the simulation loop.

The expected data flow is:

```text
Simulation Time
       │
       ▼
   Trajectory
       │
       ▼
     Drone
       │
       ▼
True Target Position
       │
       ▼
     Sensor
       │
       ├── Detection Check
       │
       ├── Range Check
       │
       └── Noise Model
              │
              ▼
      Measured Position
              │
              ▼
      Reachability Check
              │
              ▼
    Inverse Kinematics
              │
              ▼
     Target Joint Angles
```

The sensor acts as the boundary between:

```text
True World State
```

and:

```text
Observed World State
```

The rest of the control system should operate on the measured position
rather than directly accessing the true drone position.

This separation allows measurement imperfections to influence the system
without modifying the drone or robot modules.

---

# 21. MVP Summary

The minimum sensor implementation consists of:

```text
Target Position
       │
       ▼
Distance Calculation
       │
       ▼
Detection Range Check
       │
   ┌───┴────┐
   │        │
Outside    Inside
   │         │
   ▼         ▼
 None    Add Noise
              │
              ▼
      Measured Position
```

The MVP sensor provides:

- fixed sensor position,
- circular detection range,
- deterministic range detection,
- optional Gaussian measurement noise,
- explicit invalid configuration handling.

This provides sufficient realism for the anti-drone simulation while
remaining small, testable and independent from the rest of the system.
