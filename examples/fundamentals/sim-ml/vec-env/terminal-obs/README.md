# Terminal Observations — Done vs Truncated

Four pendulums driven by different constant torques. When an env auto-resets
(done or truncated), the VecEnv provides `terminal_observations` — the
observation from the final state *before* the reset. This matters for value
bootstrapping in RL:

- **Done** (task completed) — do NOT bootstrap value from terminal state
- **Truncated** (time limit) — DO bootstrap value from terminal state

The HUD shows the terminal obs vs post-reset obs for each env, with a
"Bootstrap?" column that makes the distinction visible.

See also: [Parallel Stepping](../parallel-step/) — simpler VecEnv demo
(no resets, no terminal obs).

## What you see

- **4 pendulums** in a row, each with a colored tip (blue, green, orange,
  red). Each driven by a different constant torque
- **Strong-torque pendulums** (env 0: +0.8, env 3: -0.7) swing past the
  done threshold (|qpos| > 2.0) and auto-reset — you see them snap back
  to vertical. These show "done" in the HUD
- **Weak-torque pendulums** (env 1: +0.3, env 2: -0.2) oscillate without
  reaching the threshold and timeout at 2.0 seconds — they also snap
  back. These show "truncated" in the HUD
- **HUD** (bottom-left):
  - Per-env action and episode time
  - **Last Reset Per Env** table showing terminal qpos, initial qpos,
    type (done/truncated), and Bootstrap? (No/Yes)

## How it works

### Terminal observations in VecEnv

When `VecEnv::step()` auto-resets an environment, it returns:
- `observations[i]` — the **post-reset** state (qpos near zero)
- `terminal_observations[i]` — the **pre-reset** state (at the
  done/truncated threshold), wrapped in `Some(...)`
- For non-reset envs: `terminal_observations[i]` is `None`

### Why this matters for RL

In value-based RL (PPO, TD3, SAC), the value function estimates future
reward. When an episode ends:

- **Done** → the task is truly over → future reward is zero →
  `V(terminal) = 0`, no bootstrapping needed
- **Truncated** → the task was artificially cut short → there IS future
  reward → `V(terminal)` should be bootstrapped from the terminal state

Without `terminal_observations`, the post-reset state (qpos ≈ 0) would
be used for bootstrapping — a completely different state that gives the
wrong value estimate.

### Physics

| Parameter | Value |
|-----------|-------|
| Pendulum | L=1.0 m, mass=1.0 kg, damping=0.2 |
| Motor | gear=10, ctrl range [-1, 1] |
| Done | |qpos| > 2.0 rad |
| Truncated | episode time > 2.0 s |
| Integrator | RK4, dt=0.002 s, no contacts |

### Per-env actions

| Env | Action | Torque | Expected behavior |
|-----|--------|--------|-------------------|
| 0 | +0.8 | 8.0 Nm | Swings past +2.0 rad → done |
| 1 | +0.3 | 3.0 Nm | Oscillates, doesn't reach threshold → truncated |
| 2 | -0.2 | -2.0 Nm | Oscillates gently → truncated |
| 3 | -0.7 | -7.0 Nm | Swings past -2.0 rad → done |

## Validation (at t=15s)

| Check | Expected |
|-------|----------|
| Done: terminal_obs beyond threshold | qpos > 2.0 in terminal obs |
| Truncated: terminal_obs populated | `Some(...)` on truncated envs |
| Non-reset envs: terminal_obs == None | No spurious terminal obs |
| Done precedence over truncated | done=true, truncated=false when both could fire |
| Post-reset env continues stepping | > 10 steps after first reset |

## Run

```
cargo run -p example-ml-vec-env-terminal-obs --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
