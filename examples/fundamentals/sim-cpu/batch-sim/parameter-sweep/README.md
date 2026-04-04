# Parameter Sweep — BatchSim Construction and Parallel Stepping

Eight pendulums share one `Model` and step in parallel via
`BatchSim::step_all()`. Each has a different effective damping coefficient,
applied as a velocity-proportional control torque (`ctrl = -D * qvel`).

## What you see

- **Eight pendulums** in a row, all starting at 90° tilt
- Color gradient: green (undamped) to red (highest damping)
- The green pendulum swings forever — full energy conservation
- The red pendulum barely swings — energy dissipated almost immediately
- HUD shows per-env energy and percentage remaining

## Design decision: damping via ctrl

All environments share one `Model`, so `dof_damping` cannot vary per-env.
Instead, each env applies a velocity-proportional torque through a motor
actuator: `ctrl[0] = -D_i * qvel[0]`. This produces identical physics to
joint damping (`τ = -D * ω`).

Motor actuators default to `ctrllimited=false` (unbounded ctrlrange), so
the torque is never clipped.

## Physics

| Env | D | Expected behavior |
|-----|---|-------------------|
| 0 | 0.0 | Perpetual swing, energy conserved |
| 1 | 0.1 | Slow decay |
| 2 | 0.2 | Moderate decay |
| 3 | 0.3 | Moderate decay |
| 4 | 0.4 | Noticeable settling |
| 5 | 0.5 | Mostly settled by t=10 |
| 6 | 0.6 | Nearly stopped |
| 7 | 0.7 | Overdamped, barely swings |

Energy dissipation rate: `dE/dt = -D * ω²`

## Validation

Four automated checks at t=10s:

| Check | Expected |
|-------|----------|
| Undamped energy conserved | drift < 1% for D=0.0 |
| Damped energy decays | < 5% remaining for D=0.7 |
| Energy monotonically ordered | E_0 > E_1 > ... > E_7 |
| All envs stepped | all 8 at t ≈ 10.0s |

## Run

```
cargo run -p example-batch-sim-parameter-sweep --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
