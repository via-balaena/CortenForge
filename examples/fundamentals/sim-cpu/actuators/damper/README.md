# Damper — Viscous Braking

A velocity-dependent actuator that brakes a spinning arm. Unlike a constant-
force brake (linear slowdown), a viscous damper pushes proportional to speed —
fast deceleration at high velocity, then asymptotically approaching rest. The
result is textbook exponential decay.

## What you see

- **Metal rod** with a **red tip sphere** — a single-link arm spinning around
  a pivot in zero gravity, viewed head-on
- The arm starts at 10 rad/s (~1.6 rev/s) and visibly decelerates
- Fast spin-down in the first few seconds, then slower and slower — the
  exponential "long tail" where the arm creeps toward rest but never quite stops
- The HUD tracks `omega` against `expected` (analytical) — they match exactly

## Physics

The `<damper>` shortcut is a pure velocity-proportional brake:

```
actuator_force = -kv * velocity * ctrl
               = -0.03 * omega * 1.0
```

This maps to `GainType::Affine(0, 0, -kv)` — only the velocity term is
non-zero. No bias, no activation dynamics. The ctrl signal scales damping
strength (0 = coasting, 1 = full brake).

In zero gravity with no other forces, the equation of motion reduces to:

```
I_eff * dw/dt = -kv * w
```

This is a first-order linear ODE with the exact solution:

```
w(t) = w0 * e^(-t/tau)    where tau = I_eff / kv = 2.75 s
```

At one time constant (t = 2.75s), velocity has dropped to 36.8% of its initial
value. At two time constants (t = 5.5s), it's at 13.5%. The decay is the same
shape regardless of initial speed — double the starting velocity and the curve
just scales up.

| Parameter | Value |
|-----------|-------|
| Damping coefficient (kv) | 0.03 N·m·s/rad |
| Body mass | 1.0 kg |
| Arm length | 0.5 m (CoM at 0.25 m) |
| Effective inertia | 0.0825 kg·m² |
| Initial velocity | 10 rad/s |
| Time constant (tau) | 2.75 s |
| Integrator | RK4, dt = 1 ms |

**Key distinction from motor:** the motor applies constant torque regardless of
state. The damper's force depends on velocity — it's a feedback actuator that
automatically reduces effort as the system slows down.

## Expected console output

```
t=  1.0s  force=-0.208  w=6.94
t=  3.0s  force=-0.100  w=3.34
t=  5.0s  force=-0.048  w=1.61
t= 10.0s  force=-0.008  w=0.26
```

Force and velocity decay together — both follow the same exponential envelope.
By t=10s the arm is nearly stopped (97% decayed).

## Validation

Four automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Exponential decay** | w(tau)/w0 = e^-1 = 0.368 | < 5% |
| **Time constant** | 36.8% velocity at t = 2.75s | < 5% |
| **Force proportional to velocity** | force = -kv * omega | < 5% |
| **Affine gain path** | `GainType::Affine` (structural) | exact |

## Run

```
cargo run -p example-actuator-damper --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
