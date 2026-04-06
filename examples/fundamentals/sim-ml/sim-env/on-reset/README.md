# Domain Randomization via on_reset

Pendulum under pure gravity, no motor action. Each episode starts at a
different angle from the cycling sequence [0.5, 1.0, 1.5, 2.0] rad.
The pendulum swings freely for 2 seconds, then truncates and resets at
the next angle in the sequence.

## What you see
- The pendulum snaps to a new starting angle each episode
- Four distinct starting positions cycle in order
- Gravity pulls it down from each starting angle — no motor torque
- Higher starting angles produce wider swings

## Expected behavior
- Each episode lasts exactly 2 seconds (truncation at time limit)
- The cycle repeats: 0.5 → 1.0 → 1.5 → 2.0 → 0.5 → ...
- At least 7 episodes complete within the 15-second validation window

## Validation
| Check | Expected |
|-------|----------|
| Episodes completed | >= 4 (one full cycle) |
| All 4 angles seen | 0.5, 1.0, 1.5, 2.0 all appear |
| Angles cycle in order | deterministic sequence |
| Sensor reflects start | sensor("angle") ≈ qpos after reset |
| Stability | no NaN in observations |

## Run
```
cargo run -p example-ml-sim-env-on-reset --release
```
