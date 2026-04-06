# Spaces Stress Test

Headless validation of observation and action space edge cases and
correctness invariants that the crate's 111 unit tests don't cover.

## What it tests

1. **Realistic model** — cart-pole with contacts, sensors, 2 joints, 3
   bodies. Builds obs + act spaces and runs 100 steps without panic.
2. **All 13 extractors in one space** — qpos, qvel, qacc, ctrl,
   sensordata, actuator_force, qfrc_constraint, xpos, xquat, cvel,
   sensor (by name), contact_count, time, energy. Verifies dim accounting
   matches the sum of every extractor's contribution.
3. **All 5 injectors in one space** — ctrl, qfrc_applied, xfrc_applied,
   mocap_pos, mocap_quat on a mocap-enabled model. Verifies dim and
   successful apply.
4. **1000-step endurance** — extract obs + apply actions in a loop for
   1000 steps. No panics, no NaN, no drift in tensor shapes.
5. **Overlapping ranges** — qpos(0..2) + qpos(1..2) produces correct
   concatenation with duplicated elements (not an error).
6. **Empty space** — zero extractors, dim=0, extract produces empty
   tensor.
7. **Round-trip fidelity** — extract obs, write a known value into ctrl
   via action space, extract again, verify the ctrl value propagated.
8. **Batch parity** — extract_batch(N) produces the same result as N
   individual extract() calls. apply_batch(N) matches N individual
   apply() calls.
9. **Builder errors on realistic model** — out-of-bounds qpos range on
   actual nq, out-of-bounds xpos range on actual nbody, nonexistent
   sensor name on a model with real sensors.

## Expected output

All checks PASS.

```
=== Spaces Stress Test ===
  Realistic model:      100 steps, no panic/NaN  PASS
  All 13 extractors:    dim=58, expected=58, shape_ok=true, no_nan=true  PASS
  All 5 injectors:      dim=21, expected=21, apply succeeded  PASS
  1000-step endurance:  1000 steps, shapes stable, no NaN  PASS
  Overlapping ranges:   [1.11, 2.22, 2.22] (expected [1.11, 2.22, 2.22])  PASS
  Empty space:          dim=0, shape=[0]  PASS
  Round-trip fidelity:  before=0.0000, after=3.1250 (expected 3.125)  PASS
  Batch parity:         8 envs, obs_match=true, act_match=true  PASS
  Builder errors:       range_oob=true, body_oob=true, sensor_missing=true (3/3 caught)  PASS
==========================
```

## Run

```
cargo run -p example-ml-spaces-stress-test --release
```
