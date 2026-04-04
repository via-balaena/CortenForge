# Stress Test — BatchSim Invariants

Headless validation of the `BatchSim` parallel multi-environment API:
construction, independent state, parallel stepping, cross-contamination
isolation, bitwise determinism, all reset variants, shared model access,
and single-env parity.

## Checks (10)

| # | Check | What it validates |
|---|-------|-------------------|
| 1 | N environments created | `new(model, 8)` → `len() == 8`, `!is_empty()`; `new(model, 0)` → empty |
| 2 | Independent state | Perturbing env 0 does not affect env 1 after stepping |
| 3 | step_all advances all | 10 steps → all 8 envs at `time == 10 * dt` |
| 4 | No cross-contamination | ctrl on env 0 only; env 1 matches standalone (bitwise) |
| 5 | Batch matches sequential | 4 envs, 50 steps, bitwise identical to sequential stepping |
| 6 | reset(i) resets only i | reset(2) → env 2 at t=0/qpos0/qvel=0; envs 0,3 untouched |
| 7 | reset_where selective | Even-indexed reset, odd-indexed untouched |
| 8 | reset_all resets everything | All 8 envs back to t=0 and qpos0 |
| 9 | Shared model | `batch.model().nq/nv/nu/nbody` match original |
| 10 | Single-env matches standalone | `BatchSim::new(model, 1)` vs `make_data()`, 50 steps, bitwise |

## Run

```
cargo run -p example-batch-sim-stress-test --release
```
