# Episode Lifecycle

SimEnv as a Bevy Resource with the episode loop running live.
This is the **Pattern B spike** — the first example where SimEnv owns
the physics and we copy poses to PhysicsData for rendering.

## What you see

- A pendulum driven by a sinusoidal torque (near resonance)
- The pendulum builds energy until it swings past the done threshold
  (|angle| > 2.0 rad), then snaps back to vertical for a new episode
- If the sinusoidal phase can't reach the threshold within 5 s, the
  episode truncates and resets instead
- HUD shows episode count, step count, cumulative reward,
  done/truncated flags, and per-step reward

## Expected behavior

- Episodes last 1–3 seconds (resonant driving builds energy quickly)
- Multiple episodes visible within the first 15 seconds
- After each reset the pendulum starts from vertical (angle = 0)
- Done episodes end with large swing angle; truncated episodes end
  at the 5 s time limit

## Pattern B details

After each `env.step()`, we call `data_mut().forward(&model)` to
recompute kinematics (geom_xpos, geom_xmat, sensordata), then clone
the full `Data` into `PhysicsData`. `sync_geom_transforms` reads
from `PhysicsData` as usual — no changes needed downstream.

## Validation

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Episodes completed (15 s wall clock) | >= 2 | — |
| Reset obs near initial | qpos, qvel ~ 0 | 1e-4 |
| Done fires at threshold | \|qpos\| > 2.0 when done=true | — |
| Truncated fires at time limit | time > 5.0 when truncated=true | — |
| Reward matches manual | reward == -qpos^2 | 1e-10 |

## Run

```
cargo run -p example-ml-sim-env-episode-loop --release
```
