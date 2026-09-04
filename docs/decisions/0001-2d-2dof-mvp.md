# ADR-0001: Use a 2D Workspace and 2-DOF Manipulator for the MVP

## Status

Accepted

## Context

The project is an anti-drone system simulation developed for a robotics
course.

The final project requirements do not strictly define the required number
of degrees of freedom or whether the simulation must operate in two or
three dimensions.

A more complex implementation using a 3D workspace and additional degrees
of freedom would increase the mathematical, simulation, and implementation
complexity of the project.

The primary educational objectives of the project are to demonstrate:

- robot kinematics,
- target tracking,
- simulated sensing,
- feedback control,
- PID control,
- closed-loop simulation.

## Decision

The Minimum Viable Product will use:

- a two-dimensional Cartesian workspace,
- a planar robotic manipulator,
- two revolute joints,
- two degrees of freedom,
- a simulated moving target representing a drone.

The robot will operate in the Cartesian plane using coordinates:

(x, y)

The manipulator configuration will be represented by two joint angles:

(q1, q2)

The implementation will support:

- forward kinematics,
- inverse kinematics,
- target reachability checks,
- trajectory tracking,
- joint-level PID control.

## Rationale

A 2D 2-DOF manipulator is sufficient to demonstrate the complete control
pipeline:

Target
→ Sensor
→ Inverse Kinematics
→ Joint Reference
→ PID Controllers
→ Robot Dynamics / State Update
→ Forward Kinematics
→ Updated Robot Position

This configuration allows the project to focus on understanding and
validating the complete system rather than increasing complexity through
additional dimensions or degrees of freedom.

## Consequences

### Positive

- Lower implementation complexity.
- Easier mathematical verification.
- Easier visualization.
- Faster development.
- Suitable for incremental development.
- Allows focus on control and simulation architecture.

### Negative

- The simulation does not represent full three-dimensional anti-drone
  operation.
- Robot movement is limited to a planar workspace.
- Future expansion to 3D will require additional kinematic and
  visualization work.

## Future Extensions

Possible future extensions include:

- a 3-DOF planar manipulator,
- additional joint constraints,
- a three-dimensional workspace,
- 3D target trajectories,
- obstacle avoidance,
- more realistic sensor models.

These extensions are explicitly outside the MVP scope.
