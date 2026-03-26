# ActuatorFrc + JointActuatorFrc Sensors

Actuator force measurement from two different code paths.

## What it demonstrates

- `<actuatorfrc>` reads `actuator_force[id]` (the actuator's own force output)
- `<jointactuatorfrc>` reads `qfrc_actuator[dof]` (net actuator force at the joint DOF)
- For a single motor with `gear="1"`, both should be identical (cross-path check)
- A custom `set_control` system injects `ctrl[0] = 5.0` every frame, chained
  before `step_physics_realtime` to ensure correct ordering

## Expected visual behavior

A hinge arm with light damping (0.1) is driven by a constant 5 Nm torque. The
arm accelerates, spinning faster over time. The orange tip sphere traces arcs
as the arm rotates.

## Expected console output

```
t=  1.0s  angle=+XXX.X°  afrc=+5.000  jfrc=+5.000  diff=0.00e0
t=  2.0s  angle=+XXX.X°  afrc=+5.000  jfrc=+5.000  diff=0.00e0
...
```

Both force readings match exactly at 5.0 (ctrl * gain * gear = 5 * 1 * 1).
The angle increases steadily as the motor drives the arm.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| AfrcSensor == data | sensor matches `actuator_force[0]` | < 1e-12 |
| Afrc ≈ ctrl | force equals the control input (5.0 Nm) | < 1e-6 |
| Afrc == Jfrc | both code paths agree (gear=1 ⟹ moment=1) | < 1e-12 |
| \|Afrc\| > 1.0 | force is nonzero at all times after t=0.01s | 0 violations |

## Run

```bash
cargo run -p example-sensor-actuator-force --release
```
