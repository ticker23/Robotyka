# PID Controller Module Specification

## 1. Purpose

The controller module implements feedback control algorithms used by the
Anti-Drone System simulation.

The MVP uses a PID controller to generate control commands for individual
robot joints.

The controller operates on generic scalar numeric values.

It does not know whether the controlled value represents:

- a robot joint angle,
- velocity,
- position,
- or another physical quantity.

The expected conceptual interface is:

```text
Target Value
     │
     ▼
Calculate Error
     │
     ▼
PID Controller
     │
     ▼
Control Output
```

Within the Anti-Drone System, two independent PID controllers will
eventually be used:

```text
q1_target                           q2_target
    │                                   │
    ▼                                   ▼
PID Controller 1                  PID Controller 2
    │                                   │
    ▼                                   ▼
q1 velocity command              q2 velocity command
```

The controller module is responsible only for feedback control
calculation.

---

## 2. Module Responsibilities

The controller module is responsible for:

- calculating control error,
- calculating the proportional term,
- accumulating the integral term,
- calculating the derivative term,
- maintaining internal PID state,
- producing a control output,
- resetting controller state when requested.

The controller module must not contain:

- robot geometry,
- robot state management,
- inverse kinematics,
- forward kinematics,
- trajectory logic,
- drone logic,
- sensor logic,
- simulation loop logic,
- visualization logic.

The controller operates only on numeric values.

---

## 3. PID Control Model

The PID controller combines three components:

```text
P = Proportional

I = Integral

D = Derivative
```

The final control output is:

```text
output = P + I + D
```

The continuous PID equation is conceptually:

```text
u(t) =
    Kp × e(t)
    +
    Ki × ∫e(t)dt
    +
    Kd × de(t)/dt
```

Where:

```text
u(t) = controller output

e(t) = control error

Kp = proportional gain

Ki = integral gain

Kd = derivative gain
```

The project uses a discrete-time implementation suitable for the
simulation loop.

---

## 4. Control Error

The control error is defined as:

```text
error = target - current
```

Where:

```text
target
```

is the desired value.

And:

```text
current
```

is the currently observed value.

Example:

```text
target = 10

current = 6

error = 4
```

A positive error should produce a positive proportional contribution when
`Kp > 0`.

A negative error should produce a negative proportional contribution.

---

## 5. Proportional Term

The proportional term is calculated as:

```text
P = Kp × error
```

The proportional term reacts directly to the current error.

A larger absolute error produces a larger proportional response.

Example:

```text
Kp = 2

error = 4

P = 8
```

When:

```text
Kp = 0
```

the proportional component is disabled.

---

## 6. Integral Term

The integral term represents accumulated control error over time.

For each update:

```text
integral =
    integral
    +
    error × dt
```

The integral contribution is:

```text
I = Ki × integral
```

Where:

```text
dt
```

is the simulation timestep.

The accumulated integral is part of the internal controller state.

Example:

```text
error = 2

dt = 0.1
```

After one update:

```text
integral = 0.2
```

After another identical update:

```text
integral = 0.4
```

The integral term allows persistent error to influence the control output
over time.

When:

```text
Ki = 0
```

the integral contribution to the output is zero.

The MVP does not require integral clamping or anti-windup.

---

## 7. Derivative Term

The derivative term represents the rate of change of the control error.

For updates after the first update:

```text
derivative =
    (error - previous_error) / dt
```

The derivative contribution is:

```text
D = Kd × derivative
```

The derivative term reacts to changes in error rather than directly to
the error magnitude.

Example:

```text
previous_error = 10

error = 8

dt = 0.5
```

Then:

```text
derivative =
    (8 - 10) / 0.5
    =
    -4
```

If:

```text
Kd = 0.25
```

then:

```text
D = -1
```

When:

```text
Kd = 0
```

the derivative component is disabled.

---

## 8. First Update Behaviour

The derivative term requires a previous error value.

When a controller is newly created or has been reset:

```text
previous_error = None
```

Therefore, during the first update:

```text
derivative = 0.0
```

The controller must not attempt to calculate a derivative using an
undefined previous error.

Conceptually:

```text
First Update
     │
     ▼
previous_error is None
     │
     ▼
derivative = 0
```

After calculating the output, the current error becomes:

```text
previous_error
```

for the next update.

---

## 9. Discrete PID Algorithm

For each controller update, the following algorithm is used:

```text
1. Validate dt.

2. Calculate control error.

   error = target - current

3. Calculate proportional term.

   proportional = Kp × error

4. Update accumulated integral.

   integral += error × dt

5. Calculate integral term.

   integral_term = Ki × integral

6. Calculate derivative.

   If previous_error is None:

       derivative = 0

   Otherwise:

       derivative =
           (error - previous_error) / dt

7. Calculate derivative term.

   derivative_term = Kd × derivative

8. Calculate output.

   output =
       proportional
       +
       integral_term
       +
       derivative_term

9. Store current error as previous_error.

10. Return output.
```

---

## 10. Public Interface

The conceptual public interface is:

```python
class PIDController:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
    ) -> None:
        ...

    def update(
        self,
        target: float,
        current: float,
        dt: float,
    ) -> float:
        ...

    def reset(self) -> None:
        ...
```

The exact internal representation is not prescribed.

The public behaviour must follow this specification.

---

## 11. Controller Parameters

The PID controller requires three gain parameters:

```text
kp
ki
kd
```

The MVP requires:

```text
kp >= 0

ki >= 0

kd >= 0
```

Zero values are valid.

For example:

```text
kp = 1
ki = 0
kd = 0
```

represents a proportional-only controller.

Similarly:

```text
kp = 0
ki = 1
kd = 0
```

allows isolated testing of the integral term.

Negative gain values are outside the intended MVP control convention and
must raise:

```python
ValueError
```

---

## 12. Internal State

The PID controller maintains state between update calls.

The required conceptual state is:

```text
PIDController
│
├── kp
├── ki
├── kd
│
├── integral
└── previous_error
```

Initial state:

```text
integral = 0.0

previous_error = None
```

The gain parameters remain unchanged during normal controller operation.

The state values:

```text
integral
previous_error
```

are updated by calls to:

```python
update()
```

---

## 13. Time Step Validation

The PID controller operates on a positive timestep.

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

The controller must not:

- silently replace invalid timesteps,
- clamp invalid timesteps,
- assume a default timestep.

Time management belongs to the simulation module.

---

## 14. Update Behaviour

The main controller operation is:

```python
output = controller.update(
    target=target,
    current=current,
    dt=dt,
)
```

Conceptually:

```text
target
current
dt
 │
 ▼
PIDController.update()
 │
 ├── calculate error
 ├── calculate P
 ├── update I
 ├── calculate D
 ├── update internal state
 │
 ▼
control output
```

Calling `update()` modifies:

```text
integral
previous_error
```

but must not modify:

```text
kp
ki
kd
```

---

## 15. Reset Behaviour

The controller must provide:

```python
controller.reset()
```

Reset restores the dynamic PID state to its initial condition.

After reset:

```text
integral = 0.0

previous_error = None
```

The configured gain parameters:

```text
kp
ki
kd
```

must remain unchanged.

Conceptually:

```text
PID State
   │
   │ reset()
   ▼

integral = 0
previous_error = None
```

The next update after reset must behave like the first update of a newly
created controller.

---

## 16. Output

The PID controller returns a scalar numeric control output.

Conceptually:

```python
float
```

For the robot integration planned in this project, the output will later
be interpreted as a joint velocity command.

For example:

```text
PID 1 output
     │
     ▼
q1 velocity command
```

However, this interpretation does not belong to the PID controller
itself.

The controller simply returns a numeric control value.

---

## 17. Robot Integration

The future robot control architecture will use two separate controller
instances:

```text
                   Inverse Kinematics
                           │
                    ┌──────┴──────┐
                    │             │
                    ▼             ▼
                q1_target     q2_target
                    │             │
                    ▼             ▼
                 PID 1          PID 2
                    │             │
                    ▼             ▼
                  q1_dot        q2_dot
                    │             │
                    └──────┬──────┘
                           ▼
                         Robot
```

Each controller maintains independent state.

Conceptually:

```python
pid_q1 = PIDController(...)
pid_q2 = PIDController(...)
```

The controller module itself must not create these instances on behalf of
the robot or simulation.

Instance management belongs to higher-level components.

---

## 18. Controller Independence

The controller must remain independent from all higher-level system
components.

The controller must not import:

```text
Drone
Sensor
Trajectory
Robot
Simulation
Kinematics
```

The expected abstraction is:

```text
numbers
   │
   ▼
PID Controller
   │
   ▼
number
```

This allows the PID controller to be independently tested and reused.

---

## 19. Error Handling

Invalid controller configuration must fail explicitly.

The following values must raise `ValueError`:

```text
kp < 0

ki < 0

kd < 0
```

Invalid update timestep must also raise `ValueError`:

```text
dt <= 0
```

Invalid values must not be silently corrected.

The controller must not:

- convert negative gains to positive values,
- replace invalid gains with defaults,
- replace invalid timestep values,
- silently skip invalid updates.

---

## 20. MVP Limitations

The initial PID implementation intentionally remains simple.

The MVP does not require:

- output saturation,
- integral limits,
- anti-windup,
- derivative filtering,
- derivative-on-measurement,
- feedforward control,
- gain scheduling,
- automatic tuning.

These features may be introduced later if required by simulation
behaviour.

They must not be implemented during the initial controller task.

---

## 21. Expected File Structure

The implementation must be located in:

```text
src/anti_drone/controller.py
```

The module should contain:

```text
controller.py
│
└── PIDController
```

Tests must be located in:

```text
tests/test_controller.py
```

No additional controller abstractions are required for the MVP.

---

## 22. Testing Requirements

The PID controller must be independently testable.

Tests must verify the individual PID components and controller state
behaviour.

---

### 22.1 Proportional Tests

Tests should verify:

- proportional-only control,
- positive error,
- negative error,
- zero error,
- proportional gain equal to zero.

Example:

```text
kp = 2
ki = 0
kd = 0

target = 10
current = 5

error = 5

expected output = 10
```

---

### 22.2 Integral Tests

Tests should verify:

- integral accumulation across updates,
- integral behaviour with different timestep values,
- zero integral gain,
- persistent error accumulation.

The tests should calculate expected values analytically.

---

### 22.3 Derivative Tests

Tests should verify:

- derivative term is zero on the first update,
- derivative responds to changing error,
- derivative responds correctly to decreasing error,
- derivative calculation uses `dt`,
- zero derivative gain disables derivative contribution.

Expected values should be calculated analytically.

---

### 22.4 Combined PID Tests

Tests should verify behaviour when all three gains are enabled.

A known sequence of:

```text
target
current
dt
```

values should be used.

The expected PID output must be calculated independently from the
implementation.

Tests must not reproduce the implementation algorithm in a way that could
hide an implementation error.

---

### 22.5 Reset Tests

Tests should verify:

- reset clears accumulated integral,
- reset clears previous error,
- gain parameters remain unchanged,
- the next update behaves like the first update.

---

### 22.6 Validation Tests

Tests should verify:

- negative `kp` raises `ValueError`,
- negative `ki` raises `ValueError`,
- negative `kd` raises `ValueError`,
- zero gains are valid,
- zero `dt` raises `ValueError`,
- negative `dt` raises `ValueError`.

---

## 23. Acceptance Criteria

The controller module is considered complete when:

- `PIDController` is implemented,
- proportional control behaves correctly,
- integral error accumulates correctly,
- derivative error is calculated correctly,
- the first derivative calculation is zero,
- controller state persists correctly between updates,
- `reset()` restores the dynamic state,
- gain parameters remain unchanged during reset,
- invalid gains raise `ValueError`,
- invalid timestep values raise `ValueError`,
- the module has no dependencies on higher-level components,
- no unnecessary PID extensions are introduced,
- unit tests cover P, I, D, combined operation, reset and validation,
- the complete project test suite passes,
- linting passes,
- formatting checks pass.

---

## 24. Future Extensions

Possible future PID improvements include:

```text
Output Limits
      │
      ▼
Actuator Saturation
      │
      ▼
Integral Windup
      │
      ▼
Anti-Windup
```

Additional potential extensions include:

- integral clamping,
- output clamping,
- derivative filtering,
- configurable reset behaviour,
- gain tuning tools,
- PID performance metrics,
- feedforward control.

These extensions are outside the current MVP.

They should only be introduced after observing an actual need during
simulation integration.

---

## 25. Future Simulation Integration

The controller will eventually be integrated into the complete simulation
pipeline:

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
Measured Position
    │
    ▼
Inverse Kinematics
    │
    ▼
Target Joint Angles
    │
 ┌──┴──────────────┐
 ▼                 ▼
PID q1           PID q2
 │                 │
 ▼                 ▼
q1 velocity      q2 velocity
 │                 │
 └───────┬─────────┘
         ▼
       Robot
```

The simulation module will be responsible for supplying:

```text
target value
current value
dt
```

to each controller.

The PID controller itself remains unaware of the complete simulation
architecture.

---

## 26. MVP Summary

The MVP PID controller consists of:

```text
Target
  │
  ▼
Error = Target - Current
  │
  ├─────────────┐
  │             │
  ▼             ▼
P Term        I Term
  │             │
  │             └──────┐
  │                    │
  ▼                    ▼
             D Term
                │
       ┌────────┴────────┐
       │                 │
       ▼                 ▼
   Internal State     PID Output
```

In simplified form:

```text
P = Kp × error

I = Ki × accumulated_error

D = Kd × error_change / dt

output = P + I + D
```

The implementation provides a small, deterministic and independently
testable feedback controller suitable for joint-level control in the
Anti-Drone System MVP.
