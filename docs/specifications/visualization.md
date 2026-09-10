# Visualization Module Specification

## 1. Purpose

The visualization module provides a simple 2D graphical representation of the Anti-Drone System simulation.

Its responsibility is presentation only.

The visualization must display the current state of the simulation without implementing or modifying:

* trajectory logic,
* drone movement,
* sensor detection,
* sensor noise,
* reachability,
* inverse kinematics,
* PID control,
* robot state updates,
* simulation time management.

The visualization consumes simulation state and presents it graphically.

The intended architecture is:

```text
Simulation
    │
    │ step(dt)
    ▼
SimulationStep
    │
    ▼
Visualization
```

The visualization is a passive consumer of simulation results.

---

## 2. Scope

The initial visualization is deliberately simple.

It should display:

* the 2-DOF planar robot,
* the robot base,
* the intermediate joint,
* the end effector,
* the true drone position,
* the measured drone position when available,
* the sensor position,
* the sensor detection range,
* the current simulation time,
* the current simulation state.

Optional lightweight trajectory history may also be displayed.

The visualization is intended to make the operation of the MVP understandable during demonstrations.

---

## 3. Technology

Use:

```text
matplotlib
```

for the initial implementation.

The module should not introduce heavier graphical frameworks.

Do not use:

* pygame,
* tkinter,
* Qt,
* OpenGL,
* ROS RViz,
* external visualization engines.

The MVP requires only a simple technical 2D visualization.

---

## 4. Architectural Role

The visualization must not control the simulation.

The preferred relationship is:

```text
run_simulation.py
       │
       ├───────────────┐
       │               │
       ▼               ▼
   Simulation      Visualization
       │               ▲
       │ step(dt)      │
       ▼               │
 SimulationStep ───────┘
```

The execution script coordinates both components.

The visualization should not own the Simulation instance unless there is a strong implementation reason.

Dynamic state should primarily be supplied through:

```python
SimulationStep
```

---

## 5. Expected File Structure

The visualization implementation should be located in:

```text
src/anti_drone/visualization.py
```

Tests should be located in:

```text
tests/test_visualization.py
```

The executable demonstration should use:

```text
scripts/run_simulation.py
```

---

## 6. Visualization Inputs

The visualization requires two categories of data.

### Static configuration

Static geometry comes from:

* Robot link lengths,
* Sensor position,
* Sensor detection range.

Conceptually:

```text
Robot
├── link_1
└── link_2

Sensor
├── position
└── detection_range
```

These values remain constant during the simulation.

### Dynamic state

Dynamic state comes from `SimulationStep`.

The current SimulationStep provides:

```text
time
state
drone_position
measured_position
target_joint_angles
robot_joint_angles
end_effector_position
```

The visualization must not recalculate control state.

---

## 7. Public Interface

The conceptual interface is:

```python
class Visualization:
    def __init__(
        self,
        robot: Robot,
        sensor: Sensor,
    ) -> None:
        ...

    def update(
        self,
        step: SimulationStep,
    ) -> None:
        ...

    def show(self) -> None:
        ...
```

The exact internal structure may differ if required by Matplotlib.

However, the responsibilities must remain equivalent.

---

## 8. Robot Visualization

The robot is a planar 2-DOF manipulator.

The graphical representation consists of:

```text
base
  │
  ▼
joint 1 / base point
  │
  │ link 1
  ▼
intermediate joint
  │
  │ link 2
  ▼
end effector
```

The base is located at:

```text
(0.0, 0.0)
```

The robot links should be drawn as two connected line segments.

Conceptually:

```text
             ● end effector
            /
           / link 2
          ● intermediate joint
         /
        / link 1
       ● base
```

---

## 9. Intermediate Joint Position

The simulation currently provides:

```text
q1
q2
end_effector_position
```

but not the Cartesian position of the intermediate joint.

For visualization purposes, the intermediate joint position may be calculated using:

```text
x1 = link_1 * cos(q1)
y1 = link_1 * sin(q1)
```

This calculation is allowed because it is required only to determine the graphical position of the first link endpoint.

The visualization must not reimplement full forward kinematics for the entire manipulator.

The end-effector position should use:

```python
step.end_effector_position
```

rather than independently recomputing the complete robot position.

---

## 10. Robot Geometry

The robot drawing consists of three Cartesian points:

```text
base = (0.0, 0.0)

joint = (
    link_1 * cos(q1),
    link_1 * sin(q1),
)

end_effector = step.end_effector_position
```

These points define two line segments:

```text
base -> joint

joint -> end_effector
```

The current joint angles must come from:

```python
step.robot_joint_angles
```

The visualization must not use target joint angles to draw the actual robot.

---

## 11. Drone Visualization

The true drone position comes from:

```python
step.drone_position
```

It should be shown as a clearly visible point or marker.

Conceptually:

```text
● Drone
```

The visualization must treat this as the true simulated target position.

---

## 12. Sensor Measurement Visualization

If:

```python
step.measured_position is not None
```

the measured position should be displayed separately from the true drone position.

Conceptually:

```text
● true drone position

× measured position
```

When sensor noise is disabled, both markers may overlap.

When sensor noise is enabled, they may appear at different positions.

This distinction is useful for demonstrating the Sensor model.

---

## 13. Missing Measurement

If:

```python
step.measured_position is None
```

no measurement marker should be displayed.

The visualization must not invent a position.

This normally corresponds to:

```text
SimulationState.NO_TARGET
```

---

## 14. Sensor Position

The Sensor has a fixed configured position.

The visualization should display the sensor location.

Conceptually:

```text
◇ Sensor
```

The visualization must obtain this position from the Sensor configuration.

It must not assume the Sensor is always located at `(0.0, 0.0)` even though this may be a common configuration.

---

## 15. Sensor Detection Range

The detection range should be represented as a circle centered at:

```text
sensor.position
```

with radius:

```text
sensor.detection_range
```

Conceptually:

```text
             .----------------.
          .-'                  '-.
        .'                        '.
       /                            \
      |            Sensor            |
       \                            /
        '.                        .'
          '-.                  .-'
             '----------------'
```

The circle is informational only.

It must not participate in detection logic.

Detection decisions remain the responsibility of the Sensor module.

---

## 16. Robot Workspace

The initial visualization does not need to draw the Robot workspace.

The robot workspace is already defined mathematically by the kinematics module.

If it is later useful for demonstrations, a workspace boundary may be added.

This is optional and outside the minimum visualization scope.

---

## 17. Simulation State Display

The visualization should display the current:

```text
SimulationState
```

Possible values are:

```text
NO_TARGET
TARGET_UNREACHABLE
TRACKING
```

Example:

```text
State: TRACKING
```

The displayed state must come directly from:

```python
step.state
```

The visualization must not independently determine the state.

---

## 18. Simulation Time Display

The visualization should display:

```python
step.time
```

in seconds.

Example:

```text
Time: 2.40 s
```

The visualization must not maintain an independent simulation clock.

---

## 19. Display Layout

A simple single 2D coordinate system is sufficient.

Example:

```text
 y
 ↑
 │
 │                         ● Drone
 │                        × Measurement
 │
 │
 │                   ● End effector
 │                  /
 │                 /
 │                ● Joint
 │               /
 │              /
 │             ● Base
 │
 └────────────────────────────────────→ x
```

Additional text may show:

```text
Time: 2.40 s
State: TRACKING
```

The visualization should prioritize clarity over appearance.

---

## 20. Coordinate System

The visualization must follow the same coordinate convention as the rest of the project:

```text
+x -> right

+y -> up

angles -> radians

positive rotation -> counter-clockwise
```

The Robot base remains:

```text
(0.0, 0.0)
```

---

## 21. Equal Axis Scaling

The graphical coordinate system must preserve geometric proportions.

Use equivalent Matplotlib behaviour to:

```python
ax.set_aspect("equal")
```

One unit in the X direction must visually represent the same physical distance as one unit in the Y direction.

Without equal scaling, robot geometry may appear distorted.

---

## 22. Axis Limits

Axis limits should be selected so that the important simulation area remains visible.

A reasonable display range may be based on:

```text
robot maximum reach = link_1 + link_2
```

and:

```text
sensor detection range
```

Conceptually:

```text
display_range =
    max(
        robot.link_1 + robot.link_2,
        sensor.detection_range,
    )
```

A small margin should be added.

If the Sensor is not located at the Robot origin, its configured position must also be considered.

The implementation should remain simple.

---

## 23. Dynamic Update

The main update operation is:

```python
visualization.update(step)
```

Each call updates graphical elements to represent the supplied `SimulationStep`.

The method should update:

* robot links,
* robot joints,
* end effector,
* drone marker,
* measurement marker,
* time text,
* state text,
* optional trajectory history.

The method must not advance the Simulation.

---

## 24. Separation Between Simulation and Visualization

The correct data flow is:

```text
Simulation.step(dt)
        │
        ▼
SimulationStep
        │
        ▼
Visualization.update(step)
```

Incorrect designs include:

```text
Visualization calculates IK
```

or:

```text
Visualization updates PID
```

or:

```text
Visualization moves Robot directly
```

or:

```text
Visualization determines whether target is reachable
```

All such behaviour belongs outside the visualization module.

---

## 25. Animation Loop

The visualization module does not need to implement simulation logic.

A higher-level script may perform:

```python
while simulation.time < duration:
    step = simulation.step(dt)
    visualization.update(step)
```

Matplotlib animation tools may later call equivalent logic.

A suitable implementation may use:

```text
matplotlib.animation.FuncAnimation
```

if this keeps the code simple.

However, the visualization design should remain based on individual `SimulationStep` updates.

---

## 26. Demonstration Script

The project demonstration should use:

```text
scripts/run_simulation.py
```

The script is responsible for creating and configuring:

* Trajectory,
* Drone,
* Sensor,
* Robot,
* PID controllers,
* Simulation,
* Visualization.

Conceptually:

```python
trajectory = ...
drone = ...
sensor = ...
robot = ...
pid_q1 = ...
pid_q2 = ...

simulation = Simulation(
    drone=drone,
    sensor=sensor,
    robot=robot,
    pid_q1=pid_q1,
    pid_q2=pid_q2,
)

visualization = Visualization(
    robot=robot,
    sensor=sensor,
)
```

The script then coordinates:

```text
Simulation
    ↓
SimulationStep
    ↓
Visualization
```

---

## 27. Optional Drone History

The visualization may maintain a small history of previous true drone positions.

Conceptually:

```text
. . . . . . . ●
trajectory      drone
```

This history belongs to the visualization layer.

It must not be added to Simulation state only for graphical purposes.

Conceptually:

```python
self.drone_history.append(
    step.drone_position
)
```

History may be unlimited for a short demonstration or limited to a reasonable number of points.

This feature is optional.

---

## 28. Measurement History

Measurement history is not required for the initial MVP.

Only the current measured position needs to be shown.

A future version may optionally display measurement history.

---

## 29. Plot Refresh

The visualization should refresh efficiently enough for a simple interactive animation.

Where practical, existing Matplotlib graphical objects should be updated rather than clearing and recreating the entire plot for every frame.

For example:

```text
robot line objects
drone marker
measurement marker
text labels
```

may be created once and updated.

Do not introduce complex optimization for the MVP.

---

## 30. Initialization

When a Visualization instance is created, it should prepare:

* Matplotlib figure,
* axis,
* robot graphical elements,
* drone marker,
* measurement marker,
* sensor marker,
* sensor detection circle,
* time text,
* state text.

The exact Matplotlib implementation is not prescribed.

The resulting object should be ready to accept:

```python
visualization.update(step)
```

---

## 31. Measurement Marker Behaviour

When a measurement exists:

```python
step.measured_position != None
```

the measurement marker is visible.

When:

```python
step.measured_position is None
```

the marker should be hidden or moved to an empty-data state.

The visualization must not display the previous measurement as if it were still current.

This is particularly important when transitioning from:

```text
TRACKING
```

to:

```text
NO_TARGET
```

---

## 32. State Transition Visualization

The visualization should correctly handle transitions such as:

```text
TRACKING
    ↓
NO_TARGET
```

or:

```text
TRACKING
    ↓
TARGET_UNREACHABLE
```

or:

```text
NO_TARGET
    ↓
TRACKING
```

Graphical state must reflect the current `SimulationStep`.

No stale information should remain visible incorrectly.

---

## 33. Actual vs Target Robot State

The robot drawing must use:

```python
step.robot_joint_angles
```

not:

```python
step.target_joint_angles
```

`target_joint_angles` represent the desired joint configuration calculated by inverse kinematics.

`robot_joint_angles` represent the actual current robot configuration after control update.

The visualization must display the actual Robot state.

---

## 34. End Effector

The end-effector position must use:

```python
step.end_effector_position
```

The visualization should not recalculate the full forward kinematics independently.

This ensures the graphical representation corresponds directly to the state returned by Simulation.

---

## 35. No Mutation

The visualization must not modify:

```text
SimulationStep
Robot
Sensor
Drone
PIDController
Simulation
```

It reads state only.

Calling:

```python
visualization.update(step)
```

must not modify simulation behaviour.

---

## 36. Testing Philosophy

Visualization tests should focus on deterministic graphical state preparation and simple geometry.

Do not attempt pixel-perfect image testing.

Do not test:

```text
exact anti-aliasing
exact rendered pixels
exact colors
window manager behaviour
GUI backend behaviour
```

The goal is to verify the code that transforms simulation state into plotting data.

---

## 37. Intermediate Joint Geometry Tests

The intermediate joint calculation must be tested.

For:

```text
link_1 = 1
q1 = 0
```

expected:

```text
joint = (1, 0)
```

For:

```text
link_1 = 1
q1 = π / 2
```

expected:

```text
joint = (0, 1)
```

For:

```text
link_1 = 2
q1 = π
```

expected approximately:

```text
joint = (-2, 0)
```

Use:

```python
pytest.approx(...)
```

for floating-point comparisons.

---

## 38. Update Tests

Tests should verify that a supplied SimulationStep updates the internal graphical data corresponding to:

* robot base,
* intermediate joint,
* end effector,
* drone position,
* measured position,
* time,
* simulation state.

The exact test strategy may depend on how Matplotlib artists are stored.

Prefer testing data associated with plot objects rather than rendered pixels.

---

## 39. NO_TARGET Visualization Test

Create a SimulationStep with:

```text
state = NO_TARGET
measured_position = None
```

Verify:

* drone remains visible,
* Robot remains represented,
* measurement marker is hidden,
* state text represents `NO_TARGET`.

---

## 40. TARGET_UNREACHABLE Visualization Test

Create a SimulationStep with:

```text
state = TARGET_UNREACHABLE
measured_position != None
target_joint_angles = None
```

Verify:

* drone is represented,
* measurement is represented,
* Robot current state is represented,
* state text represents `TARGET_UNREACHABLE`.

---

## 41. TRACKING Visualization Test

Create a SimulationStep with:

```text
state = TRACKING
measured_position != None
target_joint_angles != None
```

Verify:

* robot geometry corresponds to actual robot joint angles,
* drone marker corresponds to true drone position,
* measurement marker corresponds to measured position,
* end-effector marker corresponds to `step.end_effector_position`,
* state text represents `TRACKING`.

---

## 42. Smoke Test

At least one lightweight test should verify that:

```python
visualization.update(step)
```

can execute for a valid SimulationStep without raising an exception.

The test should use a non-interactive Matplotlib backend if required by the test environment.

The test must not require a visible GUI window.

---

## 43. Integration With Simulation

At least one test may use a real SimulationStep returned by:

```python
simulation.step(dt)
```

and pass it to:

```python
visualization.update(step)
```

The purpose is to verify that the interfaces integrate correctly.

Do not attempt to test the full visual appearance.

---

## 44. Dependencies

The only new external dependency expected for this module is:

```text
matplotlib
```

Do not add other graphical dependencies.

If Matplotlib is not already part of the project dependencies, it may be added explicitly to the project configuration.

No additional numerical dependency such as NumPy is required solely for this visualization.

---

## 45. Error Handling

The visualization should assume it receives a valid `SimulationStep`.

It should not duplicate Simulation validation.

The visualization should not attempt to repair malformed simulation state.

Configuration errors that prevent construction should fail clearly.

Avoid broad exception suppression.

---

## 46. Out of Scope

The initial visualization must not introduce:

* 3D graphics,
* realistic drone models,
* realistic robot models,
* textures,
* lighting,
* camera controls,
* GUI menus,
* configuration panels,
* interactive PID tuning,
* user-controlled targets,
* mouse-based robot control,
* physics,
* collision rendering,
* obstacle rendering,
* multiple targets,
* video export,
* complex dashboards,
* live performance graphs,
* network communication.

These features are unnecessary for the MVP demonstration.

---

## 47. Future Extensions

Possible future additions include:

```text
Visualization
│
├── robot and drone animation
├── tracking error graph
├── joint angle graph
├── PID output graph
├── measured vs true trajectory
├── robot workspace
└── experiment result plots
```

These should only be added after the basic visualization is stable.

---

## 48. Acceptance Criteria

The Visualization module is complete when:

* `src/anti_drone/visualization.py` exists,
* `tests/test_visualization.py` exists,
* Matplotlib is used for the graphical representation,
* the Robot base is displayed,
* both Robot links are displayed,
* the intermediate joint is displayed,
* the end effector is displayed,
* actual robot joint angles are used for robot geometry,
* true Drone position is displayed,
* measured position is displayed when available,
* measurement marker disappears when measurement is unavailable,
* Sensor position is displayed,
* Sensor detection range is displayed,
* current simulation time is displayed,
* current SimulationState is displayed,
* axes preserve equal geometric scale,
* Visualization consumes SimulationStep,
* Visualization does not update Simulation,
* Visualization does not calculate IK,
* Visualization does not calculate PID control,
* Visualization does not perform Sensor detection,
* Visualization does not update Robot state,
* full forward kinematics is not duplicated,
* intermediate joint geometry is correct,
* tests are deterministic,
* tests do not require a visible graphical display,
* the complete project test suite passes,
* linting passes,
* formatting checks pass.

---

## 49. MVP Visualization Summary

The initial visualization represents:

```text
                 ● Drone
                × Measurement


                  ● End Effector
                 /
                /
               ● Joint
              /
             /
            ● Base


        Sensor detection range
```

and displays:

```text
Time: <simulation time>
State: <simulation state>
```

The data flow is:

```text
Simulation.step(dt)
        │
        ▼
SimulationStep
        │
        ▼
Visualization.update(step)
        │
        ▼
Matplotlib representation
```

The visualization is intentionally simple.

Its purpose is to make the working Anti-Drone System MVP easy to understand and demonstrate without introducing new simulation or control logic.

