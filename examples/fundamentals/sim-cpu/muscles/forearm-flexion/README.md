# Forearm Flexion — Muscle Lifting a Load

A forearm hanging from gravity, driven by a single MuJoCo `<muscle>` actuator.
The muscle activates, curls the arm upward against gravity, then releases —
demonstrating the full muscle lifecycle: activation rise, force generation
through FL/FV curves, and deactivation decay.

## What you see

- **Upper arm** (dark metal capsule) — fixed to the world, serves as the
  anchor point
- **Forearm** (lighter metal capsule with red tip) — hangs from a hinge joint
  at the elbow, free to swing in the X-Z plane
- **Tendon** (3D cylinder between upper arm and forearm) — color reflects
  actual muscle tension: **blue** at rest, **red** under peak force
- At t=0.5s, the muscle activates and the forearm curls upward
- At t=3.0s, the muscle releases and gravity pulls the arm back down
- The tendon flares red during peak contraction, fades back to blue at rest

## Physics

The `<muscle>` actuator computes force from three piecewise-quadratic curves:

```
active_force  = -F0 x FL(norm_len) x FV(norm_vel) x activation
passive_force = -F0 x FP(norm_len)
total_force   = active_force + passive_force
```

Where:
- **FL** = force-length curve (peak at optimal fiber length, zero outside range)
- **FV** = force-velocity curve (reduces force during shortening)
- **FP** = passive force (resists over-stretch beyond optimal length)

Length normalization uses `lengthrange` (computed from joint limits) so the
curves work correctly across the full joint range. F0 is auto-computed from
`scale / acc0`.

As the muscle shortens (arm curls up), the normalized length moves away
from optimal, reducing FL. The shortening velocity makes FV < 1. Both
effects reduce force as the arm approaches full flexion. Joint damping
provides viscous resistance that slows the motion visibly.

| Parameter | Value |
|-----------|-------|
| F0 (peak isometric force) | ~7.5 N (auto: scale/acc0) |
| Scale | 200 |
| Activation time constant | 10 ms |
| Deactivation time constant | 40 ms |
| Joint damping | 2.0 N-m-s/rad |
| Body mass | 1.0 kg |
| Forearm length | 0.3 m (CoM at 0.15 m) |
| Integrator | RK4, dt = 1 ms |

## Tendon visualization

The tendon is a 3D cylinder mesh stretched between two attachment points
derived from the joint's parent and child body transforms — no manual site
placement needed. Each frame, the cylinder's position, rotation, and length
update from the FK pipeline. Color is driven by actual force magnitude
normalized against F0, not by activation level.

## Expected behavior

| Phase | Time | What happens |
|-------|------|-------------|
| Rest | 0–0.5s | Arm hangs vertical, tendon blue |
| Contraction | 0.5–3.0s | Muscle activates, arm curls up, tendon turns red |
| Release | 3.0s+ | Muscle deactivates, gravity pulls arm back down, tendon fades to blue |

## Validation

Three automated checks at t=10s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Activation reached 1.0** | act > 0.99 during contraction phase | > 0.99 |
| **Muscle produced force** | max \|force\| > 1 N during simulation | > 1 N |
| **Arm moved** | angle differs from initial by > 0.05 rad | > 0.05 rad |

## Run

```
cargo run -p example-muscle-forearm-flexion --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
