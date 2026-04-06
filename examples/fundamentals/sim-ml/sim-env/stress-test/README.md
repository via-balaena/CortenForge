# SimEnv Stress Test

Headless validation of SimEnv edge cases and correctness invariants
that the crate's 111 unit tests don't cover.

## What it tests

1. **1000-step episode** — step a SimEnv for 1000 steps with sinusoidal
   actions. No panics, no NaN in observations.
2. **Multi-episode endurance** — run 50 episodes (reset + step until
   done/truncated). Verify qpos resets to initial value each time (no
   state leaks between episodes).
3. **Early termination timing** — sub_steps=100 with a low done
   threshold. Verify the step returns early and sim time is well below
   sub_steps * timestep.
4. **on_reset stateful closure** — on_reset closure owns a counter.
   After 20 episodes, verify the counter incremented exactly 20 times
   (once per reset, no double-fires).
5. **on_reset + forward() interaction** — on_reset sets qpos to a
   known value. Verify that sensordata (a derived quantity recomputed
   by forward()) reflects the modified qpos, not stale pre-reset data.
6. **data_mut() round-trip** — modify qpos via data_mut(), verify
   observe() reflects the change, verify step() runs from the modified
   state (not the pre-modification state).
7. **Reward sign under known state** — set qpos to a known positive
   value via data_mut(), step, verify reward = -qpos^2 is negative and
   matches manual calculation on the post-step state.
8. **Done precedence over truncated** — set state where both done and
   truncated conditions are met. With the recommended truncated_fn
   pattern (guard on !done_condition), verify done=true, truncated=false.
9. **Sub-steps=1 vs sub_steps=N parity** — 5 individual steps with
   sub_steps=1 produce the same final state as 1 step with sub_steps=5.
   qpos, qvel, and time match to machine precision.
10. **observe() == extract(data())** — after stepping, env.observe()
    produces the exact same tensor as obs_space.extract(env.data()).

## Expected output

All checks PASS.

```
=== SimEnv Stress Test ===
  1000-step episode:         1000 steps, no panic/NaN  PASS
  Multi-episode endurance:   50/50 episodes, state_leaks=none  PASS
  Early termination:         done=true, time=0.05s (< 1.0s = early)  PASS
  on_reset counter:          counter=20, expected=20  PASS
  on_reset + forward():      sensor=1.5000, expected=1.5000  PASS
  data_mut() round-trip:     obs_qpos=0.7500 (expect 0.75), stepped_from_modified=true  PASS
  Reward sign:               reward=-1.0032, manual=-1.0032, sign_ok=true  PASS
  Done precedence:           done=true, truncated=false (expected true, false)  PASS
  Sub-steps parity:          qpos_diff=0.00e+0, qvel_diff=0.00e+0, time_diff=0.00e+0  PASS
  observe() consistency:     observe() == extract(): true  PASS
==============================
```

## Run

```
cargo run -p example-ml-sim-env-stress-test --release
```
