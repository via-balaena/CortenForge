# Sub-Stepping and Action Rate

Two pendulums under the same sinusoidal torque policy. Left updates
the action every physics step (500 Hz). Right updates every 10 steps
(50 Hz). Same physics, different action bandwidth.

## What you see
- Left (blue tip): smooth tracking of the sinusoidal torque
- Right (orange tip): diverges over time — different action timing
  accumulates through the pendulum's nonlinear dynamics

## Expected behavior
- Both pendulums start identically, then diverge within a few seconds
- The divergence comes from nonlinear sensitivity: different action
  update timing creates different integration paths through the
  `sin(theta)` gravity term, and small differences compound
- Both stay at the same sim time (same physics rate)

## Validation
| Check | Expected |
|-------|----------|
| Sim time match | left.time == right.time |
| Action step ratio | 10:1 (left:right) |
| Trajectory divergence | |qpos_left - qpos_right| > 0.01 |
| Stability | no NaN in observations |

## Run
```
cargo run -p example-ml-sim-env-sub-stepping --release
```
