# Motor — Direct Torque Control

The simplest actuator in CortenForge: a constant torque applied directly to a
hinge joint. No dynamics, no filtering, no bias — the control signal maps
straight to joint torque. Joint damping provides viscous friction, and gravity
provides a real load.

## What you see

- **Metal rod** with a **red tip sphere** — a single-link arm hanging from a
  pivot, viewed head-on in the X-Z plane
- The arm winds up from rest, fighting gravity on the upswing
- It settles into a steady continuous spin with a visible **speed wobble** —
  faster swinging down (gravity assists), slower swinging up (gravity resists)
- The spin never stops: motor torque always exceeds what's needed to overcome
  gravity and friction combined

## Physics

The `<motor>` shortcut is the most direct actuator type:

```
actuator_force = gain * ctrl = 1 * 5.0 = 5.0 N·m  (constant)
```

No gain shaping, no bias, no activation state. The motor is a pure torque
source. The full equation of motion:

```
I_eff * dw/dt = tau_motor - damping * w - m*g*d*sin(theta)
      0.0825  =   5.0     -  0.5 * w   -  2.45 * sin(theta)
```

Three forces compete: the motor drives, viscous damping resists proportional to
speed, and gravity loads the arm sinusoidally. At terminal velocity the motor
exactly balances damping (on average), giving `w_terminal = tau / damping = 10
rad/s`. The ±2.5 rad/s speed wobble comes from gravity adding and subtracting
2.45 N·m of torque each revolution.

| Parameter | Value |
|-----------|-------|
| Motor torque | 5.0 N·m (constant) |
| Joint damping | 0.5 N·m·s/rad |
| Body mass | 1.0 kg |
| Arm length | 0.5 m (CoM at 0.25 m) |
| Effective inertia | 0.0825 kg·m² |
| Peak gravity torque | 2.45 N·m |
| Terminal velocity | ~10 rad/s (~1.6 rev/s) |
| Integrator | RK4, dt = 1 ms |

**Key distinction:** `actuator_force` is always exactly 5.0 — the motor
doesn't know about damping or gravity. Those are passive forces handled
separately in `qfrc_passive`. The motor just pushes.

## Expected console output

```
t=  1.0s  force=5.000  rev=1.2  ω=10.1
t=  5.0s  force=5.000  rev=7.3  ω=7.5
t= 10.0s  force=5.000  rev=15.1  ω=10.7
```

Force is locked at 5.000. Velocity oscillates around 10 rad/s (gravity wobble).
Revolutions climb steadily at ~1.6 rev/s — the arm never reverses.

## Validation

Four automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Force == ctrl** | `actuator_force = ctrl` exactly (gain=1) | < 1e-15 |
| **Sensor pipeline** | `ActuatorFrc` sensor matches `data.actuator_force` | < 1e-15 |
| **Terminal velocity** | Average w in [10-15s] near tau/damping = 10 | < 15% |
| **Continuous rotation** | Theta never decreases (0 reversals) | 0 |

## Run

```
cargo run -p example-actuator-motor --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
