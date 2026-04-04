# Soft Landing — Selective Reset with `reset_where`

Twelve landers descend under gravity from 3 m, each with a different
constant thrust. A motor actuator on a vertical slide joint provides
upward force. Too little thrust and they crash; too much and they hover
forever. After each failure, `BatchSim::reset_where(mask)` snaps the
lander back to 3 m and nudges its thrust toward the sweet spot
(`m·g ≈ 9.81 N`). Over ~30 seconds every lander converges and lands
softly — turning green on touchdown.

This is the RL "done + adapt" pattern: selectively reset failed
environments, adjust their parameters, and let them try again — all
while successful environments continue uninterrupted.

## What you see

- **Twelve landers** in parallel lanes, silver while in flight
- Low-thrust landers plummet, crash, snap back to the top, and try again
  with slightly more thrust
- High-thrust landers hover uselessly, snap back, and try again with
  slightly less thrust
- One by one, each lane finds the right touch and turns **green** on landing
- The convergence takes ~30 seconds — you watch the learning happen

## Physics

For mass `m = 1 kg` on a vertical slide joint:

- Net acceleration: `a = thrust/m − g`
- Soft landing requires: `|v_impact| < 2.0 m/s`
- Landing window: thrust ∈ [9.1, 9.8] N (narrow — most envs miss on first try)

Thrust adaptation after each failure:

- **Crash:** proportional nudge `+= clamp(|v| × 0.08, 0.08, 0.4)` —
  small corrections for gradual convergence
- **Hover:** fixed nudge `−= 0.25 N`

## Key API demonstrated

```rust
// Evaluate: which envs crashed or hovered?
let mask: Vec<bool> = (0..12)
    .map(|i| status[i] == Crashed || status[i] == Hovering)
    .collect();

// Adapt thrust BEFORE reset (uses pre-reset velocity)
for i in 0..12 {
    if mask[i] && status[i] == Crashed {
        thrusts[i] += (impact_vel[i].abs() * 0.08).clamp(0.08, 0.4);
    }
}

// Reset only failed envs — landed envs continue uninterrupted
batch.reset_where(&mask);
```

## Validation

Four automated checks at t=35s:

| Check | Expected |
|-------|----------|
| Low-thrust envs crashed and reset | Envs 0–1 have reset_count > 0 |
| High-thrust envs hovered and reset | Envs 10–11 have reset_count > 0 |
| All envs eventually landed | 12/12 in LANDED state |
| Landed envs untouched by reset_where | env.time > 1.0 for all landed |

## Run

```
cargo run -p example-batch-sim-reset-subset --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
