# Gravity Compensation — Static Holding Torques

The simplest use of inverse dynamics: compute the torques that keep a
two-link arm stationary at a non-vertical pose. With zero velocity and
zero desired acceleration, `data.inverse()` returns the pure gravity
compensation torques — exactly the forces needed to counteract gravity
at that configuration.

## What you see

- **Two capsule links** connected by hinge joints (shoulder + elbow),
  hanging in the X-Z plane
- The arm is positioned at shoulder=45°, elbow=-30° — a non-trivial
  pose where both joints need torque to resist gravity
- **Nothing moves.** The arm holds its pose perfectly because the
  inverse-dynamics-computed torques exactly cancel gravity
- The HUD shows joint positions, drift from target, and the holding
  torque values

## Physics

Inverse dynamics computes: `qfrc_inverse = M * qacc + qfrc_bias - qfrc_passive - qfrc_constraint`

In the static case (qvel=0, qacc=0, no passive forces, no constraints):

```
qfrc_inverse = qfrc_bias = gravity torques
```

The holding torques are computed once at startup and applied as constant
motor `ctrl`. This is open-loop gravity compensation — no feedback, no
recomputation. It works because the torques perfectly cancel gravity at
the initial pose, so the arm never moves, so the torques remain correct.

```
shoulder torque ≈ +5.93 N·m   (supports both links)
elbow torque    ≈ +0.38 N·m   (supports only lower link)
```

The shoulder torque is ~15x larger because it must support the entire
arm (3 kg total at a 45° moment arm), while the elbow only supports the
lower link (1 kg at a smaller moment arm at -30°).

| Parameter | Value |
|-----------|-------|
| Upper arm | 0.4 m, 2.0 kg |
| Lower arm | 0.3 m, 1.0 kg |
| Shoulder pose | 0.785 rad (45°) |
| Elbow pose | -0.524 rad (-30°) |
| Integrator | RK4, dt = 2 ms |

## Expected console output

```
  Inverse dynamics at static pose:
    shoulder torque = +5.9269 N·m
    elbow torque    = +0.3797 N·m
    |inv - bias|    = [0.00e0, 0.00e0]

t=  1.0s  shoulder=0.7850 elbow=-0.5240  drift=0.0e0/0.0e0
t=  2.0s  shoulder=0.7850 elbow=-0.5240  drift=0.0e0/0.0e0
...
t=  5.0s  shoulder=0.7850 elbow=-0.5240  drift=0.0e0/0.0e0
```

Zero drift — the arm holds perfectly. The gravity compensation torques
are exact because this is a constraint-free, damping-free system where
the inverse dynamics formula reduces to a pure gravity projection.

## Validation

Five automated checks at t=6s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **inv == bias (static)** | `qfrc_inverse` equals `qfrc_bias` exactly | < 1e-8 |
| **Shoulder drift** | Joint position stays at target | < 0.001 rad |
| **Elbow drift** | Joint position stays at target | < 0.001 rad |
| **Shoulder > elbow torque** | Shoulder supports more mass | qualitative |
| **Energy constant** | No work done by gravity compensation | < 0.1% |

## Run

```
cargo run -p example-inverse-gravity-compensation --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
