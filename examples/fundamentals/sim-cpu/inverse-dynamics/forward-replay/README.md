# Forward Replay — Inverse/Forward Round-Trip

The verification step for inverse dynamics: take the torque profile
computed in the torque-profile example and actually apply those torques
as motor inputs in a forward simulation. If the inverse and forward
pipelines are consistent, the motor-driven arm should follow the same
trajectory as the prescribed one.

## What you see

- **Two arms side by side**, both following the same sinusoidal trajectory
- **Left arm (blue ghost)** — follows the prescribed trajectory directly
  by setting qpos each frame. This is the "ground truth."
- **Right arm (solid metal)** — driven by motor actuators whose `ctrl`
  values come from the inverse-computed torque profile. This is the
  "proof" that inverse dynamics got the right answer.
- The two arms should be **visually indistinguishable** — same position
  at every instant
- The HUD shows tracking error (difference between the two arms) which
  should stay near zero
- **At t=5s, torques disengage.** The console prints
  `*** TORQUES DISENGAGED ***`, the HUD switches to show zero torques,
  and the solid arm falls under gravity while the ghost freezes in place
  — a dramatic visual confirmation that the torques were doing real work

## Physics

The round-trip works like this:

```
Phase 1 (startup): Compute torque profile
  for each timestep in [0, 5s]:
    set qpos, qvel to prescribed trajectory
    forward()     → compute M, qfrc_bias
    set qacc to prescribed acceleration
    inverse()     → qfrc_inverse = required torques
    store torques

Phase 2 (runtime): Replay torques
  for each timestep:
    ctrl[0] = stored_torque[step].shoulder
    ctrl[1] = stored_torque[step].elbow
    step()        → forward dynamics + integration
    compare qpos against prescribed trajectory
```

This is open-loop replay — the torques are applied without feedback.
Tracking error accumulates from integration discretization (torques are
piecewise-constant between steps). With RK4 at dt=0.002 and moderate
trajectory speeds, the error stays small.

The formula residual verifies internal consistency:
```
qfrc_inverse == M * qacc + qfrc_bias - qfrc_passive - qfrc_constraint
```
This should hold to machine precision at every step.

## Expected console output

```
  Computed 2500 torque samples over 5s (dt=0.002)
  Max formula residual: 0.00e0

t=  1.0s  s_err=0.0001  e_err=0.0002
t=  2.0s  s_err=0.0003  e_err=0.0004
...
t=  5.0s  s_err=0.0005  e_err=0.0006

=== Forward Replay (t=5.5s) ===
  Formula residual < 1e-6:       max=0.00e0  PASS
  Shoulder tracking < 0.02 rad:  max=X.XXXXe-X rad  PASS
  Elbow tracking < 0.02 rad:     max=X.XXXXe-X rad  PASS
  Combined tracking < 0.02 rad:  max=X.XXXXe-X rad over 2500 steps  PASS
```

Tracking errors are small but non-zero — they come from the
piecewise-constant torque approximation, not from any engine bug.

## Validation

Four automated checks at t=5.5s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Formula residual** | `qfrc_inverse` matches explicit formula | < 1e-6 |
| **Shoulder tracking** | Replay position matches prescribed | < 0.02 rad |
| **Elbow tracking** | Replay position matches prescribed | < 0.02 rad |
| **Combined tracking** | Max error across both joints | < 0.02 rad |

## Run

```
cargo run -p example-inverse-forward-replay --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
