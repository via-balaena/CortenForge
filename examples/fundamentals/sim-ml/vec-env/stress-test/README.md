# VecEnv Stress Test

Headless validation of VecEnv edge cases and correctness invariants
that the crate's 111 unit tests don't cover.

## What it tests

1. **Bit-exact parity (100 steps)** — VecEnv(4).step() with different
   per-env actions matches 4 sequential SimEnv.step() calls for 100
   consecutive steps. Observations, rewards, dones, and truncateds all
   compared.
2. **100-step endurance + auto-resets** — 64 envs, 100 steps with strong
   torque triggering frequent auto-resets. No panics, no NaN in any
   observation.
3. **Auto-reset terminal_obs** — push one env past the done threshold.
   After step, that env has terminal_observations populated (qpos beyond
   threshold) while observations row shows post-reset state (qpos near
   zero). Other envs have terminal_observations = None.
4. **Non-reset terminal_obs = None** — 10 steps with a high done
   threshold that never triggers. All terminal_observations are None for
   every step.
5. **on_reset env_index** — on_reset sets qpos = env_idx * 0.1. After
   reset_all(), each env's observation reflects its unique index.
6. **Sub-steps all run** — sub_steps=10 with timestep=0.01. After one
   step, time = 0.10 for all envs (no early exit in VecEnv).
7. **Wrong action shape** — [N-1, dim] and [N, dim+1] both return
   VecStepError::ActionShapeMismatch.
8. **Per-env action isolation (50 steps)** — 4 envs with different
   constant actions. After 50 steps, all observations are distinct and
   the env with the largest action has the largest qpos magnitude.
9. **Done precedence** — set state where both done and truncated
   conditions are met. With the recommended truncated_fn guard,
   done=true, truncated=false.
10. **reset_all correctness** — step to non-trivial state, reset_all(),
    verify shape is [N, obs_dim] and all qpos are near zero.
11. **VecStepResult field lengths** — 16 envs: rewards, dones,
    truncateds, terminal_observations, and errors all have length 16;
    observations shape is [16, 2].

## Expected output

All checks PASS.

```
=== VecEnv Stress Test ===
  Bit-exact parity (100 steps):         4 envs x 100 steps, mismatches=0  PASS
  100-step endurance + auto-resets:     64 envs, 100 steps, resets=256, nan=no  PASS
  Auto-reset terminal_obs:              term_some=true, term_qpos_ok=true, post_reset_ok=true, others_none=true  PASS
  Non-reset terminal_obs = None:        10 steps, 4 envs, all terminal_obs None = yes  PASS
  on_reset env_index:                   8 envs, qpos[i] = i*0.1, match = yes  PASS
  Sub-steps all run:                    sub_steps=10, expected time=0.1, match = yes  PASS
  Wrong action shape:                   wrong_n_envs=true, wrong_dim=true  PASS
  Per-env action isolation (50 steps):  all_distinct=true, max_mag_env=3 (expected 3), qpos=[0.00, 0.78, -0.78, 1.68]  PASS
  Done precedence:                      done=true, truncated=false (expected true, false)  PASS
  reset_all correctness:                shape=[8, 2] (expected [8,2]), all_near_zero=true  PASS
  VecStepResult field lengths:          obs=true, rew=true, don=true, tru=true, ter=true, err=true (all == 16)  PASS
==========================
```

## Run

```
cargo run -p example-ml-vec-env-stress-test --release
```
