# Sensor Jacobians — C, D Matrices

When `compute_sensor_derivatives` is enabled, `mjd_transition_fd()` produces
two extra matrices alongside A and B: **C** maps state perturbations to sensor
output changes, and **D** maps control perturbations to sensor output changes.
Together they form the observation equation of a discrete linear system:

```
y_{t+1} = C * x_t + D * u_t
```

## What you see

- **Metal rod** with an **orange tip sphere** — a single-link pendulum
  tilted 0.5 rad from vertical with 0.1 N·m applied torque
- Frozen for 3 seconds showing the C and D matrices on the HUD
- Released with a 2.0 N·m torque pulse — the pendulum swings
- HUD shows **predicted** sensor values (from C*dx + D*du) alongside
  **actual** sensor readings, with prediction error
- Near the linearization point the error is tiny; as the pendulum swings
  further away, the linear prediction diverges — showing where the
  linearization stops being trustworthy

## Physics

The model has two sensors: `jointpos` (position) and `jointvel` (velocity),
plus one motor actuator. This gives:

- C: 2x2 (nsensordata x 2*nv) — how sensor readings change with state
- D: 2x1 (nsensordata x nu) — how sensor readings change with control

C is nearly diagonal: the position sensor responds mainly to dq, the velocity
sensor responds mainly to qvel. D is small — control affects sensors only
indirectly through one timestep of dynamics.

| Parameter | Value |
|-----------|-------|
| DOF | 1 (hinge) |
| Actuators | 1 (motor) |
| Sensors | 2 (jointpos, jointvel) |
| State dim | 2 (dq, qvel) |
| Linearization | qpos=0.5, ctrl=0.1 |
| Torque pulse | 2.0 N·m at release |
| FD method | centered, eps = 1e-6 |

## Validation

Five automated checks at startup:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **C is Some** | sensor derivatives requested | exact |
| **C is 2x2** | nsensordata x 2*nv | exact |
| **D is 2x1** | nsensordata x nu | exact |
| **C not all zeros** | sensors respond to state | any nonzero |
| **D not all zeros** | sensors respond to control | any nonzero |

## Run

```
cargo run -p example-derivatives-sensor-jacobians --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
