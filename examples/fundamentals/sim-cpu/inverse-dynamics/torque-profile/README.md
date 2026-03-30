# Torque Profile — Dynamic Inverse Dynamics

Compute the torques required to drive a two-link arm along a prescribed
sinusoidal trajectory. This is inverse dynamics in the dynamic case —
with non-zero velocity and acceleration, the torques must account for
gravity, Coriolis/centrifugal forces, and inertial loads simultaneously.

## What you see

- The same two-link arm as gravity-compensation, but now it's **moving**
  — both joints follow smooth sinusoidal paths at different frequencies
- The arm traces a complex Lissajous-like pattern as the shoulder and
  elbow oscillate out of phase
- The HUD displays the **computed torque values** updating in real time
  — these are the torques that `inverse()` says are needed to produce
  this exact motion
- The torques oscillate smoothly, showing how gravity, Coriolis, and
  inertial terms combine at different points in the trajectory

## Physics

At each timestep, the example sets the arm to the prescribed state
(position, velocity, acceleration) and calls `forward()` then
`inverse()`:

```
qfrc_inverse = M * qacc + qfrc_bias - qfrc_passive - qfrc_constraint
```

The three terms that contribute:

- **M * qacc** — inertial force: how much torque is needed to accelerate
  the links. Dominates at trajectory extremes where acceleration is
  maximum and velocity is zero.
- **qfrc_bias (gravity)** — constant background load. The shoulder
  always fights gravity; the elbow less so.
- **qfrc_bias (Coriolis/centrifugal)** — velocity-dependent coupling
  between joints. When one joint moves fast, it creates reaction forces
  on the other. Dominates at zero-crossings where velocity is maximum
  and acceleration is zero.

The trajectory:

```
shoulder: theta(t) = 0.8 * sin(2.0 * t)
elbow:    theta(t) = 0.6 * sin(3.0 * t + 1.0)
```

| Parameter | Shoulder | Elbow |
|-----------|----------|-------|
| Amplitude | 0.8 rad (46°) | 0.6 rad (34°) |
| Frequency | 2.0 rad/s | 3.0 rad/s |
| Phase | 0 | 1.0 rad |
| Peak velocity | 1.6 rad/s | 1.8 rad/s |
| Peak acceleration | 3.2 rad/s² | 5.4 rad/s² |

## Expected console output

```
t=  1.0s  shoulder=0.727  elbow=-0.544
t=  2.0s  shoulder=0.589  elbow=0.024
t=  3.0s  shoulder=-0.224  elbow=0.570
...
```

The HUD shows torques oscillating smoothly between positive and negative
values. Shoulder torques are larger in magnitude because the shoulder
supports both links.

## Validation

Four automated checks at t=6s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Shoulder smooth** | No step-to-step jumps > 3x RMS torque | ratio < 3.0 |
| **Elbow smooth** | No step-to-step jumps > 3x RMS torque | ratio < 3.0 |
| **Shoulder peak > elbow peak** | Shoulder supports more mass | qualitative |
| **Non-trivial torques** | Shoulder peak > 1 N·m | > 1.0 |

## Run

```
cargo run -p example-inverse-torque-profile --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
