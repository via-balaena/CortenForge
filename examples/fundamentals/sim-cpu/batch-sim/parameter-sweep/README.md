# Parameter Sweep — BatchSim Construction and Parallel Stepping

Eight pendulums share one `Model` and step in parallel via
`BatchSim::step_all()`. Each has a different effective damping coefficient,
applied as a velocity-proportional control torque (`ctrl = -D * qvel`).

## What you see

- **Eight pendulums** in a row, each with a bracket and socket at the pivot
- Color gradient: green (undamped) to red (highest damping)
- The green pendulum swings forever — perfect energy conservation (RK4)
- Each subsequent pendulum decays faster — a smooth gradient from perpetual
  swing to fully settled
- HUD shows per-env energy and percentage lost

## Design decision: damping via ctrl

All environments share one `Model`, so `dof_damping` cannot vary per-env.
Instead, each env applies a velocity-proportional torque through a motor
actuator: `ctrl[0] = -D_i * qvel[0]`. This produces identical physics to
joint damping (`tau = -D * omega`).

Motor actuators default to `ctrllimited=false` (unbounded ctrlrange), so
the torque is never clipped.

## Physics

Damping coefficients use exponential doubling for visual variety — low
values keep pendulums swinging, high values settle quickly.

| Env | D | Energy at t=10 |
|-----|-------|----------------|
| 0 | 0.000 | ~0 J (conserved) |
| 1 | 0.005 | ~-0.8 J (16% lost) |
| 2 | 0.010 | ~-1.5 J (30% lost) |
| 3 | 0.020 | ~-2.5 J (51% lost) |
| 4 | 0.040 | ~-3.8 J (78% lost) |
| 5 | 0.080 | ~-4.7 J (95% lost) |
| 6 | 0.160 | ~-4.9 J (99.8% lost) |
| 7 | 0.320 | ~-4.9 J (100% lost) |

Energy dissipation rate: `dE/dt = -D * omega^2`

## Validation

Four automated checks at t=10s:

| Check | Expected |
|-------|----------|
| Undamped energy conserved | drift < 0.1% of mgl (RK4) |
| Damped energy dissipated | E7 lost > 50% of mgl vs E0 |
| Energy monotonically ordered | E_0 > E_1 > ... > E_7 |
| All envs stepped | all 8 at t ~ 10.0s |

## Run

```
cargo run -p example-batch-sim-parameter-sweep --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
