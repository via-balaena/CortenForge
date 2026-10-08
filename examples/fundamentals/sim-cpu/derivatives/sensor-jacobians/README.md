# Sensor Jacobians — C, D Matrices

When `compute_sensor_derivatives` is enabled, `mjd_transition_fd()` produces
two extra matrices alongside A and B: **C** maps state perturbations to sensor
output changes, and **D** maps control perturbations to sensor output changes,
both at the current state, as MuJoCo's `mjd_transitionFD`. Together they form
the observation equation of a discrete linear system:

```
y_t = C * x_t + D * u_t
```

## What you see

- **Heavy metal rod** (5 kg, damped) with an **orange tip sphere** — tilted
  0.5 rad from vertical
- Frozen for 3 seconds showing the C and D matrices on the HUD
- Released with a brief 30 N·m torque kick (0.5s), then coasts under gravity
- The pendulum swings out and back without spinning — no runaway rotation
- HUD shows **predicted** sensor values (from C*dx + D*du) alongside
  **actual** sensor readings
- **Per-swing average error** updates every full oscillation cycle (detected
  via velocity zero-crossings) — error grows when the arm is far from the
  linearization point, shrinks when it swings back

## Physics

The model has three sensors: the length (`tendonpos`) and speed
(`tendonvel`) of a cable running from a fixed anchor to the rod's tip — a
spatial tendon with no stiffness, damping or limit, so it exerts no force —
and the motor's force (`actuatorfrc`), plus one motor actuator. This gives:

- C: 3x2 (nsensordata x 2*nv) — how sensor readings change with state
- D: 3x1 (nsensordata x nu) — how sensor readings change with control

The cable's length is a nonlinear function of the joint angle, so C*dx
predicts it well near the linearization point and drifts away from it as the
pendulum swings out. The cable's rows of D are zero; the force sensor's row
is the motor's gear (1): the force follows the control in the same step.

| Parameter | Value |
|-----------|-------|
| DOF | 1 (hinge, damping=0.5) |
| Actuators | 1 (motor) |
| Sensors | 3 (tendonpos, tendonvel, actuatorfrc) |
| State dim | 2 (dq, qvel) |
| Body mass | 5.0 kg (CoM at 0.5 m) |
| Linearization | qpos=0.5, ctrl=0.1 |
| Torque kick | 30 N·m for 0.5s, then coast |
| FD method | centered, eps = 1e-6 |

## Validation

Five automated checks at startup:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **C is Some** | sensor derivatives requested | exact |
| **C is 3x2** | nsensordata x 2*nv | exact |
| **D is 3x1** | nsensordata x nu | exact |
| **C not all zeros** | sensors respond to state | any nonzero |
| **D not all zeros** | the force sensor responds to control | any nonzero |

## Run

```
cargo run -p example-derivatives-sensor-jacobians --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
