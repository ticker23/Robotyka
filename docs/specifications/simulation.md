# Simulation Module Specification

## 1. Purpose

The simulation module coordinates the components of the Anti-Drone System.

Unlike the lower-level modules, the simulation does not introduce new control
or kinematic algorithms.

Its responsibility is to connect the existing components and execute them in
the correct order during each simulation timestep.

The simulation coordinates:

- Drone,
- Sensor,
- Robot,
- PID controllers,
- reachability checking,
- inverse kinematics,
- forward kinematics,
- simulation time.

The expected high-level data flow is:

```text
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
    ▼
Measured Target Position
    │
    ▼
Reachability Check
    │
    ▼
Inverse Kinematics
    │
    ▼
Target Joint Angles
    │
    ▼
PID Controllers
    │
    ▼
Joint Velocity Commands
    │
    ▼
Robot
    │
    ▼
Actual Joint Angles
    │
    ▼
Forward Kinematics
    │
    ▼
End Effector Position
```

---

## 2. Module Responsibilities

The simulation module is responsible for:

- storing the current simulation time,
- advancing simulation time,
- updating the drone,
- requesting sensor measurements,
- handling missing target detections,
- checking whether a measured target is reachable,
- calculating target joint angles using inverse kinematics,
- updating both PID controllers,
- passing velocity commands to the robot,
- calculating the current end-effector position,
- returning a snapshot describing the completed simulation step.

The simulation module is an orchestrator.

It must reuse the behaviour already implemented by lower-level modules instead
of duplicating it.

---

## 3. Module Boundaries

The simulation module must not reimplement:

- trajectory equations,
- sensor detection logic,
- sensor noise generation,
- forward kinematics equations,
- inverse kinematics equations,
- reachability equations,
- PID equations,
- robot state update equations.

These responsibilities belong to their respective modules.

Conceptually:

```text
Simulation
│
├── coordinates Drone
├── coordinates Sensor
├── calls Kinematics
├── coordinates PID Controllers
├── coordinates Robot
└── manages simulation time
```

The simulation decides when components are called.

The components themselves decide how their own operations are performed.

---

## 4. Public Interface

The conceptual public interface is:

```python
class Simulation:
    def __init__(
        self,
        drone: Drone,
        sensor: Sensor,
        robot: Robot,
        pid_q1: PIDController,
        pid_q2: PIDController,
    ) -> None:
        ...

    def step(self, dt: float) -> SimulationStep:
        ...
```

The exact internal implementation is not prescribed.

The public behaviour must follow this specification.

---

## 5. Simulation Dependencies

A Simulation instance receives existing component instances.

Example:

```python
simulation = Simulation(
    drone=drone,
    sensor=sensor,
    robot=robot,
    pid_q1=pid_q1,
    pid_q2=pid_q2,
)
```

The simulation must use these supplied instances.

It must not silently create replacement:

- drones,
- sensors,
- robots,
- PID controllers.

This allows configuration and component state to remain explicit.

---

## 6. Simulation Time

The simulation stores its current absolute simulation time.

Initial simulation time is:

```text
time = 0.0
```

Each valid call:

```python
simulation.step(dt)
```

advances time according to:

```text
time_new = time_old + dt
```

For example:

```text
initial:

time = 0.0

step(dt=0.1)

time = 0.1

step(dt=0.1)

time = 0.2

step(dt=0.2)

time = 0.4
```

The resulting absolute time is passed to the Drone.

This is consistent with the trajectory contract, where trajectory position is
calculated for an absolute simulation time.

---

## 7. Time Step Validation

Every simulation step requires:

```text
dt > 0
```

The following are invalid:

```text
dt = 0

dt < 0
```

Invalid timestep values must raise:

```python
ValueError
```

Validation must happen before simulation state is modified.

For an invalid timestep:

- simulation time must remain unchanged,
- the drone must not be updated,
- the sensor must not be queried,
- PID controllers must not be updated,
- the robot must not be updated.

The simulation must not:

- replace invalid dt values,
- clamp dt,
- assume a default dt.

---

## 8. Simulation States

Each completed simulation step has one of three logical states:

```text
NO_TARGET

TARGET_UNREACHABLE

TRACKING
```

A dedicated enum should represent these states.

Conceptually:

```python
class SimulationState(Enum):
    NO_TARGET = auto()
    TARGET_UNREACHABLE = auto()
    TRACKING = auto()
```

The exact enum values are not important.

The semantic states are.

---

## 9. NO_TARGET State

The simulation enters:

```text
NO_TARGET
```

when:

```python
sensor.measure(drone.position)
```

returns:

```python
None
```

This means that the target is not currently detected by the sensor.

In this state:

- the drone has already been updated,
- the sensor has been queried,
- no inverse kinematics is calculated,
- PID controllers are not updated,
- the robot is not updated,
- target joint angles are unavailable.

The robot remains in its current joint configuration.

Conceptually:

```text
Drone
  │
  ▼
Sensor
  │
  ▼
None
  │
  ▼
NO_TARGET

Robot unchanged
```

---

## 10. TARGET_UNREACHABLE State

The simulation enters:

```text
TARGET_UNREACHABLE
```

when:

1. the sensor returns a measured target position,
2. the measured position is outside the robot workspace.

The reachability check must use the existing kinematics implementation.

Conceptually:

```text
Measured Position
       │
       ▼
is_reachable(...)
       │
       ▼
     False
       │
       ▼
TARGET_UNREACHABLE
```

In this state:

- the measurement is retained in the result,
- inverse kinematics is not calculated,
- PID controllers are not updated,
- the robot is not updated,
- target joint angles are unavailable.

The robot remains in its current configuration.

---

## 11. TRACKING State

The simulation enters:

```text
TRACKING
```

when:

1. the target is detected,
2. the measured target position is reachable.

Conceptually:

```text
Sensor Measurement
       │
       ▼
Reachability
       │
       ▼
      True
       │
       ▼
Inverse Kinematics
       │
       ▼
Target q1 / q2
       │
       ▼
PID Controllers
       │
       ▼
Joint Velocities
       │
       ▼
Robot.update(...)
       │
       ▼
Updated Robot State
```

In this state the simulation performs the complete tracking pipeline.

---

## 12. Simulation Step Order

A valid call:

```python
simulation.step(dt)
```

must conceptually perform the following operations in order.

### Step 1 — Validate dt

Verify:

```text
dt > 0
```

If validation fails, raise `ValueError` without modifying simulation state.

### Step 2 — Advance Simulation Time

Calculate:

```text
time += dt
```

### Step 3 — Update Drone

Update the Drone using absolute simulation time:

```python
drone.update(time)
```

After this operation:

```python
drone.position
```

represents the true target position at the current simulation time.

### Step 4 — Sensor Measurement

Request a measurement:

```python
measurement = sensor.measure(
    drone.position,
)
```

The Sensor is responsible for:

- detection range,
- measurement noise,
- returning `None` when the target is not detected.

The Simulation must not duplicate these calculations.

### Step 5 — Handle Missing Detection

If:

```python
measurement is None
```

the simulation state is:

```text
NO_TARGET
```

The tracking control pipeline is skipped.

### Step 6 — Check Reachability

If a measurement exists, use the existing kinematics module to determine
whether the measured position is reachable by the Robot.

The Robot link lengths must be used.

Conceptually:

```python
reachable = is_reachable(
    measurement,
    robot.link_1,
    robot.link_2,
)
```

The exact argument order must follow the existing kinematics API.

### Step 7 — Handle Unreachable Target

If the measured target is not reachable:

```text
state = TARGET_UNREACHABLE
```

The simulation must skip:

- inverse kinematics,
- PID updates,
- Robot update.

### Step 8 — Calculate Target Joint Angles

For a reachable target, use the existing inverse kinematics implementation.

Conceptually:

```python
q1_target, q2_target = inverse_kinematics(
    measurement,
    robot.link_1,
    robot.link_2,
)
```

The exact argument order must follow the existing kinematics API.

The Simulation must not implement inverse kinematics equations itself.

### Step 9 — Update q1 PID Controller

Calculate the first joint velocity command:

```python
q1_velocity = pid_q1.update(
    target=q1_target,
    current=robot.q1,
    dt=dt,
)
```

### Step 10 — Update q2 PID Controller

Calculate the second joint velocity command:

```python
q2_velocity = pid_q2.update(
    target=q2_target,
    current=robot.q2,
    dt=dt,
)
```

### Step 11 — Update Robot

Apply the velocity commands:

```python
robot.update(
    q1_velocity=q1_velocity,
    q2_velocity=q2_velocity,
    dt=dt,
)
```

After this operation, the Robot stores its new actual joint angles.

### Step 12 — Calculate End-Effector Position

Calculate the end-effector position using the existing forward kinematics
implementation and the current Robot state.

Conceptually:

```python
end_effector_position = forward_kinematics(
    robot.link_1,
    robot.link_2,
    robot.q1,
    robot.q2,
)
```

The exact argument order must follow the existing kinematics API.

### Step 13 — Return Step Result

Return an immutable snapshot describing the completed simulation step.

---

## 13. Simulation Step Result

Each call to:

```python
simulation.step(dt)
```

returns a structured snapshot.

The conceptual structure is:

```python
@dataclass(frozen=True, slots=True)
class SimulationStep:
    time: float
    state: SimulationState
    drone_position: tuple[float, float]
    measured_position: tuple[float, float] | None
    target_joint_angles: tuple[float, float] | None
    robot_joint_angles: tuple[float, float]
    end_effector_position: tuple[float, float]
```

The result represents the state of the system after the simulation step has
completed.

It should be suitable for:

- testing,
- visualization,
- logging,
- experiment analysis.

The result must represent a snapshot.

Later modifications to the Robot or Drone must not conceptually change an
already returned step result.

---

## 14. Result Semantics

### NO_TARGET

For:

```text
state = NO_TARGET
```

the result contains:

```text
time                    = current simulation time
drone_position          = current true drone position
measured_position       = None
target_joint_angles     = None
robot_joint_angles      = current unchanged robot angles
end_effector_position   = current robot end-effector position
```

---

### TARGET_UNREACHABLE

For:

```text
state = TARGET_UNREACHABLE
```

the result contains:

```text
time                    = current simulation time
drone_position          = current true drone position
measured_position       = sensor measurement
target_joint_angles     = None
robot_joint_angles      = current unchanged robot angles
end_effector_position   = current robot end-effector position
```

---

### TRACKING

For:

```text
state = TRACKING
```

the result contains:

```text
time                    = current simulation time
drone_position          = current true drone position
measured_position       = sensor measurement
target_joint_angles     = result of inverse kinematics
robot_joint_angles      = robot angles after control update
end_effector_position   = position after robot update
```

---

## 15. End-Effector Position

Every successful simulation step must return an end-effector position,
including:

```text
NO_TARGET
TARGET_UNREACHABLE
TRACKING
```

The end-effector position represents the Robot's current configuration after
all state changes applicable to that step.

For `NO_TARGET` and `TARGET_UNREACHABLE`, the Robot is not updated, so the
position corresponds to its unchanged state.

For `TRACKING`, the position corresponds to the Robot state after the PID
velocity commands have been applied.

The simulation must use the existing forward kinematics implementation.

---

## 16. PID State Behaviour

PID controllers are stateful components.

The Simulation must update them only while actively tracking a reachable
target.

Therefore:

```text
TRACKING
    -> PID controllers updated

NO_TARGET
    -> PID controllers not updated

TARGET_UNREACHABLE
    -> PID controllers not updated
```

The Simulation must not automatically reset PID controllers when:

- the target is lost,
- the target becomes unreachable.

Automatic PID reset policy is outside the MVP scope.

If such behaviour is required later, it must be introduced explicitly.

---

## 17. Robot Behaviour Without Tracking

When the simulation state is:

```text
NO_TARGET
```

or:

```text
TARGET_UNREACHABLE
```

the Robot receives no new velocity command from the Simulation.

The Robot remains at its current joint angles.

The MVP therefore uses a simple hold-position behaviour when active tracking
cannot be performed.

---

## 18. Measured Position vs True Position

The simulation must preserve the distinction between:

```text
drone_position
```

and:

```text
measured_position
```

`drone_position` represents the true simulated target position.

`measured_position` represents the Sensor output.

With an ideal sensor:

```text
noise_std = 0
```

these values may be equal.

With sensor noise enabled, they may differ.

Control calculations must use:

```text
measured_position
```

not the true drone position.

This is essential to preserve the Sensor abstraction.

---

## 19. Reachability Input

Reachability must be evaluated using the measured target position.

The expected pipeline is:

```text
True Drone Position
       │
       ▼
     Sensor
       │
       ▼
Measured Position
       │
       ▼
Reachability Check
```

The Simulation must not bypass the Sensor by checking reachability directly
against the true Drone position.

This ensures that sensor error can influence tracking behaviour in future
experiments.

---

## 20. Controller Input

The PID controllers operate in joint space.

They do not directly receive Cartesian target coordinates.

The expected transformation is:

```text
Measured Cartesian Position
          │
          ▼
  Inverse Kinematics
          │
          ▼
Target Joint Angles
          │
          ▼
    PID Controllers
```

For the first joint:

```text
error_q1 =
    q1_target
    -
    robot.q1
```

For the second joint:

```text
error_q2 =
    q2_target
    -
    robot.q2
```

The PID implementation itself remains responsible for calculating its output.

---

## 21. Controller Output Interpretation

The output of each PID controller is interpreted by the Simulation as a joint
angular velocity command.

Conceptually:

```text
PID output
    │
    ▼
joint velocity [rad/s]
    │
    ▼
Robot.update(...)
```

Therefore:

```python
q1_velocity = pid_q1.update(...)
q2_velocity = pid_q2.update(...)
```

are passed directly to:

```python
robot.update(...)
```

The Simulation must not apply additional control equations.

The MVP does not introduce:

- velocity saturation,
- actuator dynamics,
- acceleration limits,
- torque conversion.

---

## 22. Component State Ownership

Each component owns its own state.

### Drone owns:

```text
current true position
trajectory reference
```

### Sensor owns:

```text
sensor configuration
```

### PID Controller owns:

```text
integral state
previous error
controller gains
```

### Robot owns:

```text
link lengths
current q1
current q2
```

### Simulation owns:

```text
simulation time
component references
```

The Simulation must not duplicate lower-level component state unnecessarily.

---

## 23. No Internal Main Loop

The initial Simulation implementation should focus on:

```python
step(dt)
```

A long-running:

```python
run(...)
```

method is not required for the MVP implementation of this module.

Higher-level code may later perform:

```python
for _ in range(number_of_steps):
    result = simulation.step(dt)
```

This keeps the core Simulation API:

- simple,
- deterministic,
- independently testable.

---

## 24. Example Usage

Conceptually:

```python
simulation = Simulation(
    drone=drone,
    sensor=sensor,
    robot=robot,
    pid_q1=pid_q1,
    pid_q2=pid_q2,
)

result = simulation.step(dt=0.1)
```

The returned result may then be inspected:

```python
result.time
result.state
result.drone_position
result.measured_position
result.target_joint_angles
result.robot_joint_angles
result.end_effector_position
```

Repeated calls advance the system:

```python
for _ in range(100):
    result = simulation.step(dt=0.01)
```

---

## 25. Determinism

The Simulation itself must not introduce randomness.

Any randomness in the complete system must originate from components that
explicitly model randomness, such as Sensor measurement noise.

With:

- deterministic trajectory,
- sensor noise disabled or controlled,
- identical initial component state,
- identical dt sequence,

the Simulation must produce deterministic results.

---

## 26. Error Handling

The Simulation must explicitly reject:

```text
dt <= 0
```

using:

```python
ValueError
```

Errors raised by lower-level components should generally propagate unless this
specification explicitly defines handling for them.

The Simulation must not broadly suppress exceptions.

In particular, programming or configuration errors must not be silently
converted into simulation states.

`NO_TARGET` and `TARGET_UNREACHABLE` are normal simulation states, not
exceptions.

---

## 27. Expected File Structure

The implementation must be located in:

```text
src/anti_drone/simulation.py
```

Tests must be located in:

```text
tests/test_simulation.py
```

The module is expected to contain conceptually:

```text
simulation.py
│
├── SimulationState
├── SimulationStep
└── Simulation
```

No additional orchestration framework is required for the MVP.

---

## 28. Testing Strategy

The Simulation tests verify orchestration and integration behaviour.

Lower-level mathematical behaviour is already tested in the individual module
test suites.

Simulation tests should therefore focus on:

- correct component interaction,
- correct state transitions,
- correct update order,
- correct data flow,
- correct resulting state.

Do not unnecessarily retest every mathematical edge case already covered by
lower-level unit tests.

---

## 29. Time Tests

Tests must verify:

- initial simulation time is `0.0`,
- one valid step advances time by `dt`,
- repeated steps accumulate time,
- different dt values accumulate correctly,
- `dt == 0` raises `ValueError`,
- `dt < 0` raises `ValueError`,
- invalid dt does not modify simulation time.

---

## 30. Drone Update Tests

Tests must verify that:

```python
simulation.step(dt)
```

updates the Drone using the new absolute simulation time.

For example:

```text
initial time = 0

dt = 0.5

expected Drone update time = 0.5
```

After another:

```text
dt = 0.5
```

the expected Drone update time is:

```text
1.0
```

---

## 31. NO_TARGET Tests

Tests must cover a target outside Sensor detection range.

Expected behaviour:

```text
state = NO_TARGET
```

and:

```text
measured_position = None
target_joint_angles = None
```

The test must also verify that:

- Robot joint angles remain unchanged,
- PID state is not updated,
- end-effector position still corresponds to current Robot state.

---

## 32. TARGET_UNREACHABLE Tests

Tests must cover a target that:

- is detected by the Sensor,
- lies outside the Robot workspace.

Expected behaviour:

```text
state = TARGET_UNREACHABLE
```

and:

```text
measured_position != None
target_joint_angles = None
```

The test must verify that:

- Robot joint angles remain unchanged,
- PID controllers are not updated,
- the result still contains the current end-effector position.

---

## 33. TRACKING Tests

Tests must cover a target that:

- is detected,
- is reachable.

Expected behaviour:

```text
state = TRACKING
```

The result must contain:

```text
measured_position != None
target_joint_angles != None
```

The test must verify that:

- inverse kinematics produces the target joint angles,
- PID controllers are updated,
- Robot joint state changes according to the controller outputs,
- returned robot joint angles match the Robot state after update,
- returned end-effector position matches forward kinematics of the updated
  Robot state.

---

## 34. Measurement Data Flow Tests

Tests should verify that control calculations use the Sensor measurement rather
than bypassing the Sensor and using the true Drone position.

This is particularly important when Sensor noise is controlled in a test.

Conceptually:

```text
true position
     │
     ▼
sensor returns intentionally different measurement
     │
     ▼
IK must use measurement
```

The expected target joint angles should therefore correspond to the measured
position.

---

## 35. PID Update Behaviour Tests

Tests must verify:

```text
TRACKING
    -> PID state changes as expected
```

and:

```text
NO_TARGET
    -> PID state remains unchanged

TARGET_UNREACHABLE
    -> PID state remains unchanged
```

Tests should use observable controller behaviour rather than depending on
private implementation details where possible.

---

## 36. Snapshot Tests

Tests must verify that `SimulationStep` contains values corresponding to the
completed simulation step.

At minimum verify:

```text
time
state
drone_position
measured_position
target_joint_angles
robot_joint_angles
end_effector_position
```

A previously returned snapshot must not change when later simulation steps are
executed.

---

## 37. End-to-End Integration Test

At least one test should exercise the complete deterministic tracking pipeline:

```text
Trajectory
    ↓
Drone
    ↓
Sensor
    ↓
Reachability
    ↓
Inverse Kinematics
    ↓
PID
    ↓
Robot
    ↓
Forward Kinematics
```

Use:

- a deterministic trajectory,
- sensor noise disabled,
- simple PID gains,
- known Robot geometry,
- a simple positive timestep.

The expected result should be mathematically understandable and deterministic.

Avoid tests that depend on probabilistic sensor noise.

---

## 38. Floating-Point Comparisons

Tests should use approximate floating-point comparison where appropriate.

For pytest:

```python
pytest.approx(...)
```

is preferred for:

- simulation time after repeated additions,
- joint angles,
- Cartesian positions,
- inverse kinematics results,
- forward kinematics results.

Tests should use analytically understandable expected values where practical.

---

## 39. Acceptance Criteria

The Simulation module is complete when:

- `SimulationState` exists,
- `SimulationStep` exists,
- `Simulation` exists,
- initial simulation time is zero,
- `step(dt)` validates `dt`,
- valid steps advance simulation time,
- Drone is updated using absolute simulation time,
- Sensor receives the true Drone position,
- missing Sensor detection produces `NO_TARGET`,
- measured unreachable targets produce `TARGET_UNREACHABLE`,
- measured reachable targets produce `TRACKING`,
- reachability uses the measured position,
- inverse kinematics uses the measured position,
- PID controllers receive target and current joint angles,
- PID outputs are interpreted as joint velocity commands,
- Robot is updated only during `TRACKING`,
- PID controllers are updated only during `TRACKING`,
- PID controllers are not automatically reset,
- forward kinematics calculates the current end-effector position,
- every step returns a complete snapshot,
- returned snapshots remain independent of future state changes,
- no lower-level mathematical algorithms are duplicated,
- deterministic integration tests pass,
- the complete project test suite passes,
- linting passes,
- formatting checks pass.

---

## 40. Out of Scope

The initial Simulation module must not introduce:

- visualization,
- plotting,
- animation,
- real-time sleeping,
- wall-clock synchronization,
- threading,
- asynchronous execution,
- physics engines,
- actuator dynamics,
- collision detection,
- obstacle avoidance,
- automatic PID tuning,
- PID reset policies,
- target prediction,
- filtering,
- multi-target tracking,
- 3D simulation,
- more than two robot joints,
- persistent result history,
- file logging.

These may be introduced later if required.

---

## 41. Future Extensions

Potential future extensions include:

```text
Simulation.run(...)
        │
        ▼
History Collection
        │
        ├── tracking error
        ├── joint angles
        ├── target positions
        └── measurements
        │
        ▼
Visualization / Analysis
```

Other possible extensions include:

- configurable controller behaviour after target loss,
- actuator limits,
- measurement filtering,
- multiple targets,
- target prediction,
- more advanced robot dynamics,
- real-time visualization.

These features are outside the current MVP.

---

## 42. Complete MVP Data Flow

The complete intended MVP pipeline is:

```text
                         Simulation
                             │
                             │ dt
                             ▼
                     Simulation Time
                             │
                             ▼
                         Trajectory
                             │
                             ▼
                           Drone
                             │
                             │ true position
                             ▼
                           Sensor
                             │
                ┌────────────┴─────────────┐
                │                          │
              None                    measurement
                │                          │
                ▼                          ▼
           NO_TARGET                 Reachability
                                           │
                              ┌────────────┴────────────┐
                              │                         │
                            False                      True
                              │                         │
                              ▼                         ▼
                  TARGET_UNREACHABLE            Inverse Kinematics
                                                        │
                                                        ▼
                                              target q1 / target q2
                                                        │
                                              ┌─────────┴─────────┐
                                              ▼                   ▼
                                           PID q1               PID q2
                                              │                   │
                                              ▼                   ▼
                                         q1 velocity         q2 velocity
                                              │                   │
                                              └─────────┬─────────┘
                                                        ▼
                                                      Robot
                                                        │
                                                        ▼
                                                actual q1 / q2
                                                        │
                                                        ▼
                                               Forward Kinematics
                                                        │
                                                        ▼
                                               End Effector Position
```

---

## 43. MVP Summary

The Simulation module is the orchestration layer of the Anti-Drone System.

Its fundamental operation is:

```text
step(dt)
```

A simulation step performs:

```text
validate dt
    ↓
advance time
    ↓
update Drone
    ↓
measure target
    ↓
classify target state
    ↓
if reachable:
    IK
    ↓
    PID
    ↓
    Robot update
    ↓
calculate current end-effector position
    ↓
return SimulationStep
```

The Simulation does not replace the existing modules.

It connects them.

After this module is implemented, the project has a complete headless MVP
pipeline capable of simulating target movement, sensing, kinematic target
conversion, feedback control and robot state evolution.
