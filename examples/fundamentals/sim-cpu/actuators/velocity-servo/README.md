# Velocity Servo — Constant Speed Despite Gravity

A velocity-tracking controller that maintains constant angular speed while
gravity loads the arm sinusoidally each revolution. The `<velocity>` actuator
shortcut wires up a proportional velocity law: the control signal is a desired
speed, and the actuator computes whatever force is needed to maintain it.

This is the **exact opposite** of the motor actuator: the motor applies
constant force and lets speed wobble; the velocity servo holds constant speed
and lets force oscillate.

## What you see

- **Metal rod** with a **blue tip sphere** — a single-link arm spinning
  continuously, viewed head-on
- The arm spins at a visually steady rate (~0.3 rev/s) — no visible speed
  wobble despite gravity pulling on it every revolution
- The HUD tells the real story: `omega` holds near 2.0 while `force` swings
  between roughly +2.5 and -2.5 each revolution — the servo brakes on the
  downswing (gravity tries to speed it up) and pushes on the upswing (gravity
  tries to slow it down)

## Physics

The `<velocity>` shortcut expands to:

```
gain  = kv                         (fixed gain)
bias  = [0, 0, -kv]               (affine bias)
force = gain * ctrl + bias · [1, q, qdot]
      = kv * ctrl + 0 + 0 - kv * qdot
      = kv * (target_vel - qdot)
```

This is a proportional velocity controller. When the arm is too slow, the
servo pushes; when too fast, it brakes. With kv=20, the servo is stiff enough
to keep speed tight despite the sinusoidal gravity disturbance.

The full equation of motion:

```
I_eff * dω/dt = kv * (target - ω) - m*g*d*sin(θ)
                = 20 * (2.0 - ω)  - 2.45 * sin(θ)
```

At quasi-steady state (`dω/dt ≈ 0`):

```
ω ≈ target - m*g*d*sin(θ) / kv
  ≈ 2.0   - 2.45*sin(θ) / 20
  ≈ 2.0   ± 0.12 rad/s
```

The speed ripple is `gravity_peak / kv`. Higher kv = tighter tracking.

### Motor vs Velocity Servo

| | Motor | Velocity Servo |
|---|---|---|
| **Control signal** | Torque (N·m) | Speed (rad/s) |
| **Force** | Constant (5.0 N·m) | Oscillates (±2.5 N·m) |
| **Speed** | Wobbles (±2.5 rad/s) | Locked (±0.12 rad/s) |
| **The point** | "I push with X torque" | "I spin at X speed" |

| Parameter | Value |
|-----------|-------|
| Gain (kv) | 20 N·m·s/rad |
| Target velocity | 2.0 rad/s (~0.32 rev/s) |
| Body mass | 1.0 kg |
| Arm length | 0.5 m (CoM at 0.25 m) |
| Peak gravity torque | 2.45 N·m |
| Speed ripple | ±0.12 rad/s (±6%) |
| Integrator | RK4, dt = 1 ms |

**Key distinction:** The velocity servo's `actuator_force` oscillates
sinusoidally — it's doing gravity compensation in real time. The motor's
`actuator_force` is constant (it doesn't know about gravity). Same arm, same
gravity, opposite control strategies.

## Expected console output

```
t=  1.0s  force=+2.32  ω=1.88  rev=0.3
t=  4.0s  force=+2.44  ω=1.88  rev=1.3
t=  7.0s  force=+2.42  ω=1.88  rev=2.2
t= 10.0s  force=+2.25  ω=1.89  rev=3.2
```

Force swings ±2.5 each revolution (gravity compensation). Omega stays near 2.0
with ±0.12 ripple. Revolutions climb at a steady ~0.32 rev/s.

## Validation

Four automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Average ω** | Mean velocity in [5-15s] matches target | < 1% error |
| **Speed regulation** | Max \|ω - target\| bounded | < 0.5 rad/s |
| **Force oscillates** | Max \|force\| > 1 N·m (gravity compensation) | > 1.0 |
| **Force formula** | `force = kv*(target - qdot)` at every step | < 1% relative |

## Run

```
cargo run -p example-actuator-velocity-servo --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
