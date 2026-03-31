# Joint Dynamics — Damping and Friction

Three identical pendulums with different dynamics parameters:

- **(A) No loss** — damping=0, friction=0. Conservative system, energy
  is preserved indefinitely.
- **(B) Damping** — damping=0.5, friction=0. Velocity-dependent
  dissipation (τ = -b*qdot). Higher velocity → more dissipation.
- **(C) Friction** — damping=0, friction=0.5. Velocity-independent
  frictionloss (constant resistive torque opposing motion).

## What it tests

URDF `<dynamics>` has two attributes:

| URDF attribute | MJCF attribute | Force law |
|----------------|----------------|-----------|
| `damping="0.5"` | `damping="0.5"` | τ = -b*qdot (viscous) |
| `friction="0.5"` | `frictionloss="0.5"` | τ = -f*sign(qdot) (Coulomb) |

Both propagate through the URDF → MJCF → Model pipeline. The example
verifies the values arrive in the model and produce measurably different
energy decay profiles.

## Physics

All three pendulums start at 0.5 rad. After 5000 steps:

- **No loss:** peak velocity unchanged (ratio ≈ 1.0)
- **Damped:** peak velocity reduced (damping extracts energy proportional
  to velocity squared)
- **Friction:** peak velocity reduced differently (friction extracts
  energy at a constant rate regardless of velocity)

The damped and friction pendulums have different peak velocities because
their dissipation mechanisms are fundamentally different.

## Checks

| # | Check | Tolerance |
|---|-------|-----------|
| 1 | Damping=0.5 in model | 0.001 |
| 2 | Frictionloss=0.5 in model | 0.001 |
| 3 | MJCF output contains `frictionloss` | exact |
| 4 | No-loss pendulum: peak velocity stable (ratio > 0.95) | 5% |
| 5 | Damped pendulum: peak < 80% of no-loss | 20% |
| 6 | Friction pendulum: peak < 80% of no-loss | 20% |
| 7 | Damping and friction produce different peaks | diff > 0.001 |

## Run

```
cargo run -p example-urdf-damping-friction --release
```
