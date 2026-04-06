# Parallel Stepping

One `VecEnv::step()` call advances eight pendulums simultaneously. Each
pendulum receives a different constant torque, linearly spaced from
-1.0 to +1.0. This is the core VecEnv operation — a single batched
action tensor in, a single batched observation tensor out.

## What you see

- 8 pendulums in a row, all sharing one physics Model
- Color encodes the action: blue (left, act=-1.0) through red (right,
  act=+1.0). The gradient makes it easy to see which pendulum gets
  which torque
- The HUD shows per-env action values, joint angles (qpos), and
  rewards, plus the shape of the batched observation tensor

## Expected behavior

- **Outer pendulums swing hardest** — they receive the strongest torques
  (+-1.0) and build up the most angular displacement over time
- **Inner pendulums barely move** — near-zero torque can't overcome
  gravity much, so they hang nearly straight down with only a slight
  wobble
- **Mirror symmetry** — each +/- pair (env 0 vs 7, env 1 vs 6, etc.)
  swings to equal magnitude in opposite directions, confirming per-env
  action isolation. The middle env (act ~0) drifts imperceptibly — just
  floating-point noise at a stable equilibrium
- **No resets** — done and truncated are disabled. This is a pure
  stepping demo; the pendulums swing indefinitely

## Validation

| Check | Expected | Detail |
|-------|----------|--------|
| Observations shape | [8, 2] | n_envs=8, obs_dim=2 (qpos + qvel) |
| Per-env observations differ | All distinct | Different actions produce different states |
| Rewards length | 8 | One reward per env |
| Bit-exact vs SimEnv | Match | First ~10 physics steps match sequential SimEnv stepping |

## Run

```
cargo run -p example-ml-vec-env-parallel-step --release
```
