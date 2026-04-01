# Hinge Limits — Solver Tuning Comparison

Three pendulums with the same range (`-45..45` degrees) released from 60
degrees (beyond the upper limit). Each has a different `solreflimit` setting,
demonstrating how solver parameters control the stiffness of the limit
response.

## Concept

When a hinge joint exceeds its range, the solver generates a one-sided
constraint that pushes it back. The `solreflimit` attribute controls the
stiffness and damping of this constraint via `[timeconst, dampratio]`:

- **Shorter timeconst** = stiffer limit = faster correction, sharper bounce
- **Longer timeconst** = softer limit = slower correction, visible penetration

## What you see

| Pendulum | Color | solreflimit | Behavior |
|----------|-------|-------------|----------|
| Left | Blue | `0.005 1.0` | Stiff — bounces off limit sharply |
| Center | Green | `0.02 1.0` (default) | Standard — moderate bounce |
| Right | Red | `0.08 1.0` | Soft — visible penetration past limit |

All three start at 60 degrees, 15 degrees beyond the 45 degree limit.
Watch how the blue pendulum rebounds fastest while the red one sinks past
the limit before slowly correcting.

## What to look for

- Blue pendulum stays closest to (or within) the 45 degree limit
- Red pendulum visibly exceeds 45 degrees before correcting
- All three eventually settle near the limit
- HUD shows real-time angle and limit force for each

## Run

```
cargo run -p example-joint-limits-hinge-limits --release
```
