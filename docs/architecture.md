# System Architecture

## 1. Overview

The Anti-Drone System is a software-based simulation of a robotic
system designed to track a moving aerial target.

The system operates entirely in a two-dimensional simulated environment.
A simulated drone moves through the workspace according to a predefined
trajectory. Its position is measured by a simulated sensor and processed
by the control system.

A planar robotic manipulator with two degrees of freedom attempts to
track the measured target position using inverse kinematics and
independent PID controllers for each joint.

The project is designed as a modular simulation environment that allows
individual components to be developed, tested and extended independently.

---

## 2. Architectural Goals

The system architecture is designed around the following principles:

- modularity,
- separation of responsibilities,
- independent testability,
- reproducibility,
- explicit data flow,
- minimal dependencies,
- extensibility without premature complexity.

The architecture should allow individual components to be modified or
extended without requiring changes across the entire system.

The MVP focuses on a minimal but complete closed-loop robotic system.

---

## 3. System Scope

The MVP system consists of:

- a simulated moving drone,
- predefined target trajectories,
- a simulated position sensor,
- a 2-DOF planar robotic manipulator,
- forward kinematics,
- inverse kinematics,
- PID-based joint control,
- a simulation loop,
- system state visualization.

## Simulation Dimension

The initial implementation operates in a two-dimensional workspace.

The initial MVP uses:

- two Cartesian coordinates (x, y),
- a 2-DOF planar manipulator.

The architecture should allow future extension to additional degrees of
freedom and potentially a three-dimensional workspace.

The MVP implementation should not introduce unnecessary abstractions for
future scenarios that are not yet required.

Real hardware, physical drones and camera-based detection are outside
the scope of the MVP.

---

# 4. High-Level Architecture

The system is composed of several independent components connected
through the simulation loop.

```text
                           SIMULATION WORLD

                           ┌─────────────┐
                           │    Drone    │
                           │             │
                           │   x(t), y(t)│
                           └──────┬──────┘
                                  │
                                  │ true position
                                  ▼
                           ┌─────────────┐
                           │   Sensor    │
                           │             │
                           │ noise model │
                           └──────┬──────┘
                                  │
                                  │ measured position
                                  ▼
                    ┌─────────────────────────┐
                    │   Inverse Kinematics    │
                    │                         │
                    │      (x, y → q1, q2)    │
                    └────────────┬────────────┘
                                 │
                                 │ target joint angles
                    ┌────────────┴────────────┐
                    ▼                         ▼
             ┌─────────────┐            ┌─────────────┐
             │ PID Joint 1 │            │ PID Joint 2 │
             └──────┬──────┘            └──────┬──────┘
                    │                          │
                    │ velocity command         │ velocity command
                    └────────────┬─────────────┘
                                 ▼
                           ┌─────────────┐
                           │    Robot    │
                           │             │
                           │ current q1  │
                           │ current q2  │
                           └──────┬──────┘
                                  │
                                  ▼
                    ┌─────────────────────────┐
                    │   Forward Kinematics    │
                    │                         │
                    │      (q1, q2 → x, y)    │
                    └────────────┬────────────┘
                                 │
                                 ▼
                           End Effector




The simulation component coordinates all components and advances the
system state over discrete time steps.

5. Component Architecture

The application is divided into independent modules.

src/anti_drone/

├── drone.py
├── robot.py
├── sensor.py
├── controller.py
├── kinematics.py
├── trajectory.py
├── simulation.py
├── types.py
└── __init__.py

Each module has a clearly defined responsibility.

5.1 Drone
Responsibility

The drone component represents the moving target within the simulation.

It is responsible for maintaining the current drone state and updating
its position over time.

Input
- simulation time,
- trajectory definition.
Output
- current drone position.
Responsibilities

Drone
│
├── current position
├── current trajectory
└── state update

The drone component should not contain:

- sensor logic,
- robot control logic,
- inverse kinematics,
- visualization logic.


The drone component should not contain:

sensor logic,
robot control logic,
inverse kinematics,
visualization logic.

5.2 Trajectory
Responsibility

The trajectory module defines how the drone moves through the simulated
workspace.

Trajectories should be independent of the drone implementation.

Initial trajectory types

The MVP should support at least:

- linear trajectory,
- circular trajectory.

Additional trajectories may be introduced later.

Conceptual interface

time
 │
 ▼
Trajectory
 │
 ▼
position (x, y)

The trajectory module is responsible only for generating target
positions.

It does not manage the drone state.

5.3 Sensor
Responsibility

The sensor simulates measurement of the drone position.

The initial sensor model represents a position sensor operating in a
two-dimensional workspace.

Input

true drone position

Output

measured position

Sensor model

The sensor may introduce measurement noise.

True Position
     │
     ▼
 ┌─────────┐
 │ Sensor  │
 └────┬────┘
      │
      ▼
Measured Position

The difference between true and measured position allows the simulation
to demonstrate the influence of imperfect measurements on the control
system.

Future extensions

Possible extensions include:

- measurement delay,
- variable noise,
- sensor failure,
- limited measurement range.

These are not required for the MVP.

5.4 Kinematics
Responsibility

The kinematics module contains mathematical functions describing the
relationship between the robotic manipulator joint configuration and
end-effector position.

The module should remain independent from the simulation and robot state.

Forward Kinematics

Forward kinematics calculates the end-effector position from the joint
configuration.

q1, q2
   │
   ▼
Forward Kinematics
   │
   ▼
x, y

For a two-link planar manipulator:

x = L1 cos(q1) + L2 cos(q1 + q2)

y = L1 sin(q1) + L2 sin(q1 + q2)

Where:

L1 is the length of the first link,
L2 is the length of the second link,
q1 is the first joint angle,
q2 is the second joint angle.

Inverse Kinematics

Inverse kinematics calculates a valid joint configuration for a
requested end-effector position.

x, y
 │
 ▼
Inverse Kinematics
 │
 ▼
q1, q2

The implementation must detect unreachable target positions.

A target is reachable when:

|L1 - L2| ≤ distance ≤ L1 + L2

Where

distance = sqrt(x² + y²)

The system must not generate arbitrary joint angles when the target is
outside the robot workspace.

5.5 Controller
Responsibility

The controller module implements feedback control algorithms.

For the MVP, the primary controller is a PID controller.

The controller is intentionally independent from the robotic
manipulator.

It should operate on generic numeric values.

Input
target value
current value
time step
Output
control output
PID control

The controller calculates:

error = target - current

The PID output consists of:

P = proportional component
I = integral component
D = derivative component

The controller parameters are:

Kp
Ki
Kd

The controller module must not contain:

robot geometry,
inverse kinematics,
sensor logic,
trajectory logic.
5.6 Robot
Responsibility

The robot module represents the state and motion of the simulated
2-DOF robotic manipulator.

Robot parameters

The robot contains:

L1
L2

q1
q2

joint velocity q1
joint velocity q2

Where:

L1, L2 are link lengths,
q1, q2 are current joint angles.
Motion model

The MVP uses a simplified kinematic motion model.

PID controllers produce joint velocity commands.

The robot state is updated using:

q = q + velocity × dt

This provides continuous motion without requiring a full dynamic model
based on mass, torque and acceleration.

Not included in MVP

The initial robot model does not simulate:

joint torque,
motor dynamics,
link mass,
inertia,
friction.

These may be introduced in future versions.

5.7 Simulation
Responsibility

The simulation module acts as the central system orchestrator.

It coordinates all components and advances the simulation state.

The simulation is the only component that directly integrates:

drone,
sensor,
inverse kinematics,
controllers,
robot.

Individual components should not directly control each other.

6. Simulation Loop

The simulation operates using discrete time steps.

For each simulation step:

1. Update simulation time

2. Update drone position

3. Measure drone position using sensor

4. Check whether target is reachable

5. Calculate target joint angles using inverse kinematics

6. Calculate control output for joint 1

7. Calculate control output for joint 2

8. Update robot joint states

9. Calculate end-effector position

10. Record simulation state

The complete control flow:

                   ┌──────────────────────┐
                   │ Simulation Time Step │
                   └──────────┬───────────┘
                              │
                              ▼
                        Update Drone
                              │
                              ▼
                        Sensor Measure
                              │
                              ▼
                       Reachability Check
                              │
                    ┌─────────┴─────────┐
                    │                   │
                Reachable           Unreachable
                    │                   │
                    ▼                   ▼
             Inverse Kinematics     Report State
                    │
                    ▼
               Target q1, q2
                    │
            ┌───────┴────────┐
            ▼                ▼
          PID q1           PID q2
            │                │
            └───────┬────────┘
                    ▼
                Update Robot
                    │
                    ▼
             Forward Kinematics
                    │
                    ▼
              Record System State


7. Data Flow

The primary data flow through the system is:


Drone State
    │
    ▼
True Position
    │
    ▼
Sensor
    │
    ▼
Measured Position
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
Robot State Update
    │
    ▼
Current Joint Angles
    │
    ▼
Forward Kinematics
    │
    ▼
End-Effector Position

8. Coordinate System

The simulation uses a two-dimensional Cartesian coordinate system.


               y
               ↑
               │
               │
               │
               │
               ●──────────────→ x
             Robot Base

The robot base is located at:
(0, 0)

All drone and end-effector positions are represented relative to this
coordinate system.

Joint angles are represented in radians internally.

Degrees may be used for visualization and debugging purposes.


9. Robot Workspace

The workspace of the robotic manipulator is determined by the link
lengths.

The maximum reachable distance is:

L1 + L2

The minimum reachable distance is:

|L1 - L2|

The workspace can therefore be represented as:

             Maximum Reach

                 ●●●●●
             ●           ●
          ●                 ●

        ●       WORKSPACE      ●

          ●                 ●
             ●           ●
                 ●●●●●

                 Robot Base


Targets outside the workspace are considered unreachable.

10. Control Architecture

The control system uses two independent PID controllers.

Measured Target Position
          │
          ▼
   Inverse Kinematics
          │
          ▼
     q1_target
     q2_target
          │
      ┌───┴───┐
      ▼       ▼
    PID 1   PID 2
      │       │
      ▼       ▼
     q1̇      q2̇
      │       │
      └───┬───┘
          ▼
        Robot

Each controller calculates the error independently:

error_q1 = target_q1 - current_q1

error_q2 = target_q2 - current_q2

This approach separates Cartesian target processing from joint-level
control.

11. System States

The simulation may expose high-level system states.

Initial states include:

TRACKING
TARGET_UNREACHABLE
TRACKING

The target is inside the robot workspace and valid inverse kinematics
solutions can be calculated.

TARGET_UNREACHABLE

The target is outside the reachable workspace.

The system does not generate invalid joint commands.

Additional states may be added if required.

12. Error Handling

The system should explicitly handle invalid or impossible states.

Examples include:

unreachable target position,
invalid robot dimensions,
invalid simulation timestep,
invalid PID parameters,
invalid trajectory parameters.

Errors should not be silently ignored.

13. Module Dependency Rules

The following dependency direction should be maintained:

                simulation
                     │
       ┌─────────────┼─────────────┐
       │             │             │
       ▼             ▼             ▼
     drone         sensor       robot
       │                           │
       ▼                           ▼
  trajectory                   kinematics
                                   │
                                   ▼
                              controller


Conceptually, lower-level modules should not depend on the complete
simulation system.

For example:

PID Controller
    │
    └── must not import Robot

Kinematics
    │
    └── must not import Simulation

Sensor
    │
    └── must not import Controller


The simulation module performs integration between components.

14. Testing Strategy

Core modules should be independently testable.

Initial test coverage should include:

tests/

├── test_controller.py
│   └── PID behaviour
│
├── test_kinematics.py
│   ├── forward kinematics
│   ├── inverse kinematics
│   └── unreachable positions
│
├── test_sensor.py
│   ├── ideal measurements
│   └── noisy measurements
│
└── test_trajectory.py
    ├── linear trajectory
    └── circular trajectory

Integration testing may later validate the complete simulation loop.

15. Configuration

Simulation parameters should be configurable.

Important parameters include:

Robot:
    L1
    L2

PID:
    Kp
    Ki
    Kd

Sensor:
    noise level

Simulation:
    timestep
    duration

Trajectory:
    type
    speed
    dimensions

Configuration should be separated from core algorithms where practical.

16. Extensibility

The architecture is intentionally designed to allow future extensions.

Potential extensions include:

2-DOF Robot
      │
      ▼
3-DOF Robot
      │
      ▼
3D Workspace
      │
      ▼
Camera Sensor
      │
      ▼
Computer Vision


Other possible extensions:

multiple targets,
multiple sensors,
sensor delay,
actuator limits,
robot dynamics,
advanced control algorithms,
predictive tracking,
obstacle avoidance.

These extensions should not be implemented until the MVP is complete.

17. Architectural Principles

The project follows the following engineering principles.

Separation of Concerns

Each module should have one clearly defined responsibility.

Minimal Dependencies

External dependencies should only be introduced when required by the
project.

Explicit Data Flow

Important system state transitions should be visible and understandable.

Hidden communication between components should be avoided.

Testability

Core mathematical and control components should be testable without
starting the complete simulation.

Reproducibility

The project environment should be reproducible from the project
configuration.

Incremental Complexity

The project should begin with the smallest complete working system.

Additional realism should only be introduced after the MVP functions
correctly.

18. MVP Architecture Summary

The minimum complete system is:

                  Moving Drone
                       │
                       ▼
                    Sensor
                       │
                       ▼
              Inverse Kinematics
                       │
                       ▼
                 PID Controllers
                       │
                       ▼
                  2-DOF Robot
                       │
                       ▼
              Forward Kinematics
                       │
                       ▼
                 End Effector

This architecture provides a complete closed-loop simulation while
remaining small enough to understand, test and extend.

The MVP establishes the foundation for future improvements without
introducing unnecessary complexity during initial development.



