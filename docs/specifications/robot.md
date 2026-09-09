# Robot Module Specification

## 1. Purpose

The robot module represents the current state of the planar 2-DOF manipulator
used in the Anti-Drone System simulation.

The robot stores:

- link lengths,
- current joint angles.

The robot receives joint velocity commands and updates its joint angles over a
simulation timestep.

The robot module does not calculate control commands.

It does not determine where the end effector should move.

It only updates its internal state according to supplied velocity commands.

The expected conceptual data flow is:

```text
Joint Velocity Commands
        │
        ▼
      Robot
        │
        ▼
Updated Joint Angles
```

Within the complete system:

```text
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
```

---

## 2. Module Responsibilities

The robot module is responsible for:

- storing link lengths,
- storing current joint angles,
- updating joint angles from velocity commands,
- validating robot geometry,
- validating simulation timestep values.

The robot module must not contain:

- trajectory logic,
- drone logic,
- sensor logic,
- inverse kinematics,
- forward kinematics,
- PID control,
- reachability calculations,
- simulation time management,
- visualization logic.

The robot represents state.

Higher-level modules are responsible for deciding what the robot should do.

---

## 3. Robot Model

The MVP uses a planar two-link manipulator.

The robot has two revolute joints:

```text
                 End Effector
                      ●
                     /
                    /
               L2  /
                  /
             ● q2
            /
           /
      L1  /
         /
        ● q1
        |
        |
      Base
     (0, 0)
```

The robot contains:

```text
link_1
link_2
q1
q2
```

Where:

```text
link_1 = length of the first robot link

link_2 = length of the second robot link

q1 = current angle of the first joint

q2 = current angle of the second joint
```

Joint angles are represented in radians.

---

## 4. Coordinate Convention

The robot follows the project-wide coordinate convention defined in:

```text
docs/architecture.md
```

The robot base is conceptually located at:

```text
(0, 0)
```

Positive joint angles rotate counter-clockwise.

All internal angular values use radians.

---

## 5. Public Interface

The conceptual public interface is:

```python
class Robot:
    def __init__(
        self,
        link_1: float,
        link_2: float,
        q1: float = 0.0,
        q2: float = 0.0,
    ) -> None:
        ...

    def update(
        self,
        q1_velocity: float,
        q2_velocity: float,
        dt: float,
    ) -> None:
        ...
```

The exact internal implementation is not prescribed.

The public behaviour must follow this specification.

---

## 6. Initialization

A robot requires two positive link lengths.

Example:

```python
robot = Robot(
    link_1=2.0,
    link_2=1.5,
)
```

The default initial joint angles are:

```text
q1 = 0.0

q2 = 0.0
```

Therefore:

```python
robot = Robot(
    link_1=2.0,
    link_2=1.5,
)
```

represents a robot initialized with both joints at zero radians.

Custom initial joint angles are also allowed:

```python
robot = Robot(
    link_1=2.0,
    link_2=1.5,
    q1=0.5,
    q2=-0.25,
)
```

---

## 7. Link Length Validation

Both robot links must have positive lengths.

The required conditions are:

```text
link_1 > 0

link_2 > 0
```

The following values are invalid:

```text
link_1 <= 0

link_2 <= 0
```

Invalid link lengths must raise:

```python
ValueError
```

The implementation must not:

- convert negative lengths to positive values,
- clamp lengths,
- silently replace invalid values with defaults.

Invalid robot geometry must fail explicitly.

---

## 8. Joint Angles

The robot stores two current joint angles:

```text
q1
q2
```

The MVP allows joint angles to be:

- positive,
- zero,
- negative.

The robot must not apply angle wrapping.

The robot must not automatically normalize angles to:

```text
[-π, π]
```

or:

```text
[0, 2π]
```

The robot must not introduce joint position limits in the MVP.

These features may be introduced later if required.

---

## 9. Joint Velocity Commands

The robot update method receives two joint velocity commands:

```text
q1_velocity

q2_velocity
```

These values represent angular velocities.

For future robot integration, the intended unit is:

```text
radians per second
```

Velocity values may be:

- positive,
- zero,
- negative.

A positive velocity increases the corresponding joint angle.

A negative velocity decreases the corresponding joint angle.

Zero velocity leaves the corresponding joint angle unchanged.

---

## 10. State Update Model

The MVP uses a first-order kinematic state update.

For each joint:

```text
new_angle =
    current_angle
    +
    velocity × dt
```

Therefore:

```text
q1_new =
    q1_old
    +
    q1_velocity × dt
```

and:

```text
q2_new =
    q2_old
    +
    q2_velocity × dt
```

This corresponds to a simple explicit Euler integration step.

Conceptually:

```text
Current Joint Angle
        │
        │
        ├──────────────┐
        │              │
        ▼              ▼
Joint Velocity        dt
        │              │
        └──────┬───────┘
               ▼
        velocity × dt
               │
               ▼
        Updated Angle
```

---

## 11. Update Behaviour

The primary state update operation is:

```python
robot.update(
    q1_velocity=q1_velocity,
    q2_velocity=q2_velocity,
    dt=dt,
)
```

The method updates both joint angles.

Conceptually:

```text
Robot State Before Update

q1
q2

    │
    │ q1_velocity
    │ q2_velocity
    │ dt
    ▼

Robot.update()

    │
    ▼

Robot State After Update

q1_new
q2_new
```

The update method modifies:

```text
q1
q2
```

The update method must not modify:

```text
link_1
link_2
```

---

## 12. Example State Update

Given:

```text
q1 = 0.5

q2 = 1.0
```

and:

```text
q1_velocity = 2.0

q2_velocity = -1.0

dt = 0.25
```

the first joint update is:

```text
q1_new =
    0.5
    +
    2.0 × 0.25

q1_new = 1.0
```

The second joint update is:

```text
q2_new =
    1.0
    +
    (-1.0) × 0.25

q2_new = 0.75
```

The final robot state is:

```text
q1 = 1.0

q2 = 0.75
```

---

## 13. Time Step Validation

The robot requires a positive timestep.

The required condition is:

```text
dt > 0
```

The following values are invalid:

```text
dt = 0

dt < 0
```

Invalid timestep values must raise:

```python
ValueError
```

The robot must not:

- assume a default timestep,
- replace invalid timesteps,
- clamp invalid values,
- manage simulation time internally.

Time management belongs to the simulation module.

---

## 14. Repeated Updates

Robot state persists between update calls.

For example:

```python
robot.update(
    q1_velocity=1.0,
    q2_velocity=0.0,
    dt=0.5,
)
```

followed by another identical update should continue from the previously
updated joint state.

Conceptually:

```text
Initial q1 = 0

update #1
    │
    ▼
q1 = 0.5

update #2
    │
    ▼
q1 = 1.0
```

The robot must not reset its joint state automatically between updates.

---

## 15. Independent Joint Updates

The two robot joints must update independently.

For example:

```text
q1_velocity = 1.0

q2_velocity = 0.0
```

must modify only `q1`.

Similarly:

```text
q1_velocity = 0.0

q2_velocity = -1.0
```

must modify only `q2`.

There must be no unintended coupling between the joint state variables.

---

## 16. Zero Velocity Behaviour

A zero velocity command represents no movement.

For:

```text
q1_velocity = 0

q2_velocity = 0
```

the joint angles must remain unchanged.

The robot still requires:

```text
dt > 0
```

even when both velocities are zero.

---

## 17. Robot State

The robot's persistent state consists conceptually of:

```text
Robot
│
├── link_1
├── link_2
├── q1
└── q2
```

The link lengths represent robot geometry.

The joint angles represent dynamic simulation state.

Different Robot instances must maintain independent state.

Updating one Robot instance must not affect another Robot instance.

---

## 18. Robot and Kinematics Separation

The robot module stores robot state.

The kinematics module performs geometric calculations.

These responsibilities must remain separate.

The robot must not directly calculate:

```text
end effector position
```

The future simulation layer may obtain the end effector position using:

```python
forward_kinematics(
    robot.link_1,
    robot.link_2,
    robot.q1,
    robot.q2,
)
```

Conceptually:

```text
Robot State
   │
   ├── link_1
   ├── link_2
   ├── q1
   └── q2
   │
   ▼
Forward Kinematics
   │
   ▼
End Effector Position
```

The robot must not duplicate forward kinematics calculations.

---

## 19. Robot and Controller Separation

The robot must not calculate PID outputs.

The expected interaction is:

```text
Target Joint Angle
        │
        ▼
PID Controller
        │
        ▼
Joint Velocity Command
        │
        ▼
      Robot
```

Conceptually:

```python
q1_velocity = pid_q1.update(
    target=q1_target,
    current=robot.q1,
    dt=dt,
)

q2_velocity = pid_q2.update(
    target=q2_target,
    current=robot.q2,
    dt=dt,
)

robot.update(
    q1_velocity=q1_velocity,
    q2_velocity=q2_velocity,
    dt=dt,
)
```

The robot itself must not know where these velocity commands originate.

---

## 20. Module Independence

The robot module must remain independent from higher-level system components.

The robot must not depend on:

```text
Drone
Sensor
Trajectory
PIDController
Simulation
```

It must also not perform inverse or forward kinematics internally.

The expected abstraction is:

```text
Current Robot State
        +
Velocity Commands
        +
       dt
        │
        ▼
      Robot
        │
        ▼
Updated Robot State
```

---

## 21. Simplified Physical Model

The MVP intentionally uses a simplified kinematic model.

The robot does not model:

- mass,
- inertia,
- torque,
- force,
- acceleration,
- gravity,
- friction,
- motor dynamics,
- actuator delay.

The control output is interpreted directly as a joint velocity command.

The state model is therefore:

```text
velocity
   │
   ▼
integrate over dt
   │
   ▼
angle
```

This simplification is intentional.

The project focuses on:

- target tracking,
- sensing,
- kinematics,
- feedback control,
- simulation architecture.

A full dynamic manipulator model is outside the MVP scope.

---

## 22. Joint Limits

The MVP does not implement joint position limits.

The robot may therefore accumulate angles outside:

```text
[-π, π]
```

For example:

```text
q1 = 7.0 rad
```

is allowed by the robot state model.

Potential future extensions may introduce:

```text
q1_min
q1_max

q2_min
q2_max
```

These features must not be implemented during the initial robot task.

---

## 23. Velocity Limits

The MVP does not implement maximum joint velocities.

The robot accepts the velocity command supplied by the controller.

The robot must not:

- clamp velocity commands,
- saturate velocity commands,
- introduce hidden maximum velocities.

Future versions may introduce actuator constraints.

These features are outside the current scope.

---

## 24. Error Handling

Invalid robot configuration must fail explicitly.

The following must raise `ValueError`:

```text
link_1 <= 0

link_2 <= 0
```

Invalid update timestep must also raise `ValueError`:

```text
dt <= 0
```

Valid values must not be unnecessarily rejected.

In particular:

```text
q1 < 0

q2 < 0

q1_velocity < 0

q2_velocity < 0
```

are valid.

---

## 25. Expected File Structure

The implementation must be located in:

```text
src/anti_drone/robot.py
```

The module should contain:

```text
robot.py
│
└── Robot
```

Tests must be located in:

```text
tests/test_robot.py
```

No additional robot abstractions are required for the MVP.

---

## 26. Testing Requirements

The robot module must be independently testable.

Tests must verify robot initialization, state updates, validation and state
independence.

---

### 26.1 Initialization Tests

Tests should verify:

- valid link lengths are accepted,
- default `q1` equals zero,
- default `q2` equals zero,
- custom initial joint angles are stored correctly,
- negative initial joint angles are valid.

---

### 26.2 Link Validation Tests

Tests should verify:

- zero `link_1` raises `ValueError`,
- negative `link_1` raises `ValueError`,
- zero `link_2` raises `ValueError`,
- negative `link_2` raises `ValueError`.

---

### 26.3 Basic Update Tests

Tests should verify:

- positive velocity increases joint angle,
- negative velocity decreases joint angle,
- zero velocity leaves joint angle unchanged,
- both joints update correctly during the same call.

Expected values must be calculated analytically.

Example:

```text
initial:

q1 = 0
q2 = 0

command:

q1_velocity = 2
q2_velocity = -1
dt = 0.5

expected:

q1 = 1
q2 = -0.5
```

---

### 26.4 Timestep Tests

Tests should verify:

- different timestep values produce the correct state changes,
- zero timestep raises `ValueError`,
- negative timestep raises `ValueError`.

---

### 26.5 Repeated Update Tests

Tests should verify that state accumulates correctly across multiple calls.

Example:

```text
initial q1 = 0

q1_velocity = 1
dt = 0.5
```

After the first update:

```text
q1 = 0.5
```

After the second identical update:

```text
q1 = 1.0
```

---

### 26.6 Independent Joint Tests

Tests should verify:

- updating only `q1` does not modify `q2`,
- updating only `q2` does not modify `q1`.

---

### 26.7 Geometry Preservation Tests

Tests should verify that calls to `update()` do not modify:

```text
link_1
link_2
```

The robot geometry must remain constant during normal updates.

---

### 26.8 Instance Independence Tests

Tests should verify that two Robot instances maintain independent joint states.

Updating one robot must not affect the state of another robot.

---

## 27. Acceptance Criteria

The robot module is considered complete when:

- `Robot` is implemented in `src/anti_drone/robot.py`,
- positive link lengths are required,
- default joint angles are zero,
- custom initial joint angles are supported,
- positive and negative joint angles are supported,
- joint velocities correctly update joint angles,
- updates use `velocity × dt`,
- repeated updates accumulate state correctly,
- both joints update independently,
- zero velocity leaves the corresponding joint unchanged,
- invalid link lengths raise `ValueError`,
- invalid timestep values raise `ValueError`,
- robot geometry remains unchanged during updates,
- separate Robot instances maintain independent state,
- no kinematics calculations are duplicated,
- no PID logic is introduced,
- no physical dynamics are introduced,
- unit tests cover initialization, updates, validation and state behaviour,
- the complete project test suite passes,
- linting passes,
- formatting checks pass.

---

## 28. Future Extensions

Possible future robot model extensions include:

- joint position limits,
- maximum joint velocities,
- acceleration limits,
- actuator saturation,
- motor dynamics,
- torque control,
- link masses,
- moments of inertia,
- gravity,
- friction,
- dynamic equations of motion.

These extensions are outside the current MVP.

They should only be introduced when required by project requirements or future
experiments.

---

## 29. Future Simulation Integration

The robot will be integrated into the complete simulation pipeline.

The expected data flow is:

```text
Trajectory
    │
    ▼
Drone
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
Target q1 / q2
    │
    ├──────────────────┐
    ▼                  ▼
 PID q1              PID q2
    │                  │
    ▼                  ▼
q1 velocity         q2 velocity
    │                  │
    └─────────┬────────┘
              ▼
            Robot
              │
              ▼
        Actual q1 / q2
              │
              ▼
     Forward Kinematics
              │
              ▼
     End Effector Position
```

The simulation module will be responsible for coordinating these components.

The robot itself remains unaware of the complete system architecture.

---

## 30. MVP Summary

The MVP robot model can be summarized as:

```text
Robot State

link_1
link_2
q1
q2

    │
    │
    │ q1_velocity
    │ q2_velocity
    │ dt
    ▼

Robot.update()

    │
    ▼

q1 += q1_velocity × dt

q2 += q2_velocity × dt

    │
    ▼

Updated Robot State
```

The robot is intentionally a small stateful component.

It represents a planar 2-DOF manipulator whose joint angles evolve according
to externally supplied velocity commands.

This provides the minimum robot model required to connect the PID controller
with the kinematic simulation.
