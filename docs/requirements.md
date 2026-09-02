# Anti-Drone System — Requirements

## 1. Purpose

The project implements a simulated anti-drone robotic system.

The system consists of a simulated drone, a sensor model, a
2-DOF robotic manipulator and a feedback controller.

The main purpose of the project is to demonstrate selected
robotics and control concepts in a reproducible simulation
environment.

---

## 2. Scope

The MVP system shall simulate:

- a moving drone,
- a 2-DOF robotic manipulator,
- drone position sensing,
- sensor measurement noise,
- trajectory generation,
- inverse kinematics,
- PID-based control,
- closed-loop feedback,
- system visualization.

The system will initially operate entirely in simulation.

---

## 3. Functional Requirements

### FR-01 — Drone simulation

The system shall simulate a drone moving in a 2D workspace.

The drone position shall be represented as:

    x(t), y(t)

The drone shall support predefined movement trajectories.

---

### FR-02 — Drone trajectories

The system shall provide at least two predefined drone trajectories.

Examples may include:

- linear trajectory,
- circular trajectory.

Additional trajectories may be added later.

---

### FR-03 — Robotic manipulator

The system shall simulate a robotic manipulator with two
degrees of freedom.

The manipulator shall be represented using joint coordinates:

    q1
    q2

The physical parameters of the manipulator shall be configurable.

---

### FR-04 — Forward kinematics

The system shall calculate the end-effector position from
the joint configuration.

Input:

    q1, q2

Output:

    x, y

---

### FR-05 — Inverse kinematics

The system shall calculate a valid joint configuration
for a requested end-effector position.

Input:

    x, y

Output:

    q1, q2

The implementation shall handle unreachable target positions.

---

### FR-06 — Sensor

The system shall provide a simulated sensor for measuring
the drone position.

The sensor shall support configurable measurement noise.

The sensor measurement shall differ from the true drone
position when noise is enabled.

---

### FR-07 — Controller

The system shall implement a feedback controller.

The controller shall calculate control commands based on
the difference between the desired and measured target state.

---

### FR-08 — PID controller

The system shall provide a PID controller with configurable:

- Kp
- Ki
- Kd

The implementation shall allow the controller behaviour
to be evaluated for different parameter values.

---

### FR-09 — Closed-loop control

The system shall operate as a closed-loop control system.

The control loop shall consist of:

    target
       ↓
    sensor
       ↓
    controller
       ↓
    robot
       ↓
    measurement
       ↓
    feedback

---

### FR-10 — Simulation

The system shall provide a simulation loop that updates
the state of the drone and robotic manipulator over time.

---

### FR-11 — Visualization

The system shall provide a visualization of:

- drone position,
- robot configuration,
- end-effector position,
- target trajectory.

Where practical, the visualization should also display
the tracking error.

---

## 4. Non-Functional Requirements

### NFR-01 — Modularity

The system shall be divided into independent components.

At minimum:

- drone,
- sensor,
- robot,
- kinematics,
- trajectory,
- controller,
- simulation.

---

### NFR-02 — Testability

Core components shall be independently testable.

---

### NFR-03 — Configuration

Simulation parameters shall not be unnecessarily hardcoded.

Parameters such as:

- robot dimensions,
- PID gains,
- sensor noise,
- simulation timestep,
- trajectory parameters

should be configurable.

---

### NFR-04 — Reproducibility

The project shall provide a reproducible development
environment and documented installation procedure.

---

### NFR-05 — Code quality

The project should follow consistent Python formatting,
linting and testing practices.

---

## 5. Constraints

The MVP will initially use:

- 2D simulation,
- 2-DOF manipulator,
- simulated sensor,
- software-based control,
- Python.

Physical hardware is not required for the MVP.

---

## 6. Out of Scope

The following are outside the MVP:

- real drone interception,
- physical drone hardware,
- physical robotic arm,
- real camera-based detection,
- machine-learning-based object detection,
- autonomous navigation of a real drone,
- 3D simulation.

These may be considered future extensions.

---

## 7. Future Extensions

Potential future extensions include:

- 3-DOF manipulator,
- 3D simulation,
- camera-based detection,
- computer vision,
- more realistic sensor models,
- actuator limitations,
- communication delays,
- multiple drones,
- more advanced controllers.
