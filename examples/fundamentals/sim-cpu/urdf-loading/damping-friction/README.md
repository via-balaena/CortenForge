# Joint Dynamics — Damping and Friction

Three identical pendulums side by side with different dynamics:
- **Left (blue):** no loss — swings forever
- **Center (green):** damping=0.5 — velocity-dependent, decays smoothly
- **Right (red):** friction=0.5 — velocity-independent frictionloss, stops

## What you see

All three start at 45°. The blue arm keeps swinging. The green arm
(damped) quickly decays to stillness. The red arm (friction) decays
and stops at a slight offset from vertical.

## What it tests

URDF `<dynamics>` has two attributes:

| URDF attribute | MJCF attribute | Force law |
|----------------|----------------|-----------|
| `damping="0.5"` | `damping="0.5"` | tau = -b*qdot (viscous) |
| `friction="0.5"` | `frictionloss="0.5"` | tau = -f*sign(qdot) (Coulomb) |

Note: this example uses a combined MJCF model for rendering (three
pendulums in one scene). The URDF→frictionloss conversion is verified
separately.

## Validation

| Check | Source |
|-------|--------|
| URDF friction → frictionloss | `print_report` (checks URDF converter output) |
| Damping=0.5 in model | `print_report` |
| Frictionloss=0.5 in model | `print_report` |
| No-loss still swinging | `print_report` (peak_vel > 0.5) |
| Damped arm decayed | `print_report` (peak < 50% of no-loss) |
| Friction arm decayed | `print_report` (peak < 50% of no-loss) |

## Run

```
cargo run -p example-urdf-damping-friction --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
