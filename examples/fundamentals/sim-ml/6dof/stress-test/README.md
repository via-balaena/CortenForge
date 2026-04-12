# 6-DOF Stress Test

Headless validation of 6-DOF reaching task infrastructure. Run before
investing in visual examples — if the task, policies, or VecEnv have
issues, this catches them cheaply.

## What it tests

1. **VecEnv parity** — 50-env VecEnv matches 50 sequential SimEnvs
   stepping identically for 20 steps (bit-exact observations)
2. **Obs scaling sanity** — observation values stay finite and qpos
   scaled values stay < 10 after 100 steps with random actions
3. **Done condition reachable** — setting qpos to target_joints with
   zero velocity triggers done (fingertip within 5cm)
4. **Truncated timing** — episode truncates at exactly step 500
   (5.0s at 0.01s/step)
5. **Reward gradient** — reward at qpos=target is higher than at
   qpos=0 (reward function correctly penalizes distance)
6. **Auto-reset terminal_obs** — terminal observations populated when
   done/truncated fires during VecEnv stepping
7. **Linear policy** — 78 params (6x12+6), forward produces valid
   actions in [-1,1]
8. **MLP policy** — 614 params (hidden=32), forward produces valid
   actions
9. **Autograd policy** — 2-layer [64,64] ReLU with Xavier init,
   5388 params, gradient finite after backward pass
10. **Endurance** — 500 steps with 4 envs and random actions, no NaN

## Expected output

All 10 checks PASS.

## Run

```
cargo run -p example-ml-6dof-stress-test --release
```
