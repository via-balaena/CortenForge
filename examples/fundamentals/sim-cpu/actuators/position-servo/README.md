# Position Servo — PD Position Control

A proportional-derivative controller that drives a hinge joint to a target
angle. The `<position>` actuator shortcut wires up a PD law with no activation
dynamics: the control signal is a desired angle, and the actuator computes the
torque needed to get there. With `dampratio=1` the response is critically
damped — the arm swings to the target in one smooth motion and holds.

## What you see

- **Metal rod** with a **green tip sphere** — a single-link arm hanging from a
  pivot, viewed head-on in the X-Z plane
- The arm hangs straight down at rest for 3 seconds (0° position)
- At t=3 s the servo commands 45° — the arm swings smoothly to the right,
  decelerating as it approaches the target with no overshoot or oscillation
  (critically damped)
- It settles within ~0.3 s and holds steady, tilted ~44° from vertical
- The HUD shows a **0.98° steady-state error** — the PD controller can't
  quite reach 45° because it has no integral term, so gravity pulls it ~1°
  short

## Physics

The `<position>` shortcut expands to:

```
gain  = kp                         (fixed gain)
bias  = [0, -kp, -kv]             (affine bias)
force = gain * ctrl + bias · [1, q, qdot]
      = kp * ctrl + 0 - kp * q - kv * qdot
      = kp * (target - q) - kv * qdot
```

This is a textbook PD controller. The proportional term pulls toward the
target, the derivative term damps oscillation. With `dampratio=1`, the engine
computes `kv` from the effective inertia to achieve critical damping:

```
kv = 2 * dampratio * sqrt(kp * I_eff)
   = 2 * 1 * sqrt(100 * 0.0825)
   = 5.74 N·m·s/rad
```

The full equation of motion:

```
I_eff * ddq = kp * (target - q) - kv * qdot - m*g*d*sin(q)
    0.0825  = 100 * (0.785 - q) - 5.74 * qdot - 2.45 * sin(q)
```

At steady state (`qdot = 0`), the servo force must balance gravity:

```
kp * (target - q_ss) = m * g * d * sin(q_ss)
100 * (45° - 44°)    ≈ 1.70 N·m gravity torque
```

This yields a steady-state error of ~1° because the PD law has no integral
term. The residual force of 1.70 N·m is pure gravity compensation. A higher
`kp` shrinks this offset but can never eliminate it entirely — that would
require a PID controller.

| Parameter | Value |
|-----------|-------|
| Gain (kp) | 100 N·m/rad |
| Damping ratio | 1.0 (critically damped) |
| Derived kv | 5.74 N·m·s/rad |
| Target angle | π/4 = 45° |
| Body mass | 1.0 kg |
| Arm length | 0.5 m (CoM at 0.25 m) |
| Effective inertia | 0.0825 kg·m² |
| Peak gravity torque | 2.45 N·m |
| Steady-state error | 0.98° (gravity offset) |
| Control delay | 3 s (arm at rest before command) |
| Integrator | RK4, dt = 1 ms |

**Key distinction:** Unlike the motor actuator, the position servo's
`actuator_force` changes over time — it's large during the initial swing and
drops to a small gravity-compensation value at rest. The control signal is an
angle (radians), not a torque.

## Expected console output

```
t=  1.0s  force=0.000  theta=0.0000  omega=0.0000
t=  2.0s  force=0.000  theta=0.0000  omega=0.0000
t=  3.0s  force=17.816  theta=0.0936  omega=8.9417
t=  4.0s  force=1.704  theta=0.7684  omega=-0.0000
t= 10.0s  force=1.704  theta=0.7684  omega=-0.0000
```

The arm rests at 0° for 3 seconds. At t=3 s the command fires — a large
initial force (42.9 N·m peak) drives the swing. By t=4 s the arm has settled:
force at 1.70 N·m (gravity compensation), theta at 0.7684 rad (44°), velocity
zero.

## Validation

Four automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Reaches target** | \|theta - target\| < 0.05 rad after command + 0.5 s | < 0.05 rad |
| **No overshoot** | Theta never exceeds target (critically damped) | < 0.001 rad |
| **Force formula** | `force = kp*(target - q) - kv*qdot` matches | < 1% relative |
| **No activation** | `na = 0`, `data.act` is empty | exact |

## Run

```
cargo run -p example-actuator-position-servo --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
