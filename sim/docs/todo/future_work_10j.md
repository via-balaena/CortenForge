# Future Work 10j — Deferred Item Tracker: Group 9 — Misc Pipeline & API

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 9 — Misc Pipeline & API (13 items)

**Spec approach:** DT-74/75 need individual specs (T3 — Jacobian correctness bugs,
need formula derivation). DT-79/82/83 each need individual specs (T3 — API design
or data layout architecture). DT-77/78 share a "Length-Range Estimation" spec with
DT-59 (T2). The rest (DT-76/80/81/84/91/92) implement directly (T1). Totals:
6 T1, 2 T2, 5 T3.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-74 | §4 | `compute_body_jacobian_at_point()` incomplete — only x-component, `#[allow(dead_code)]` | Medium | T3 |
| DT-75 | §4 | `add_body_jacobian` free joint bug — world-frame unit vectors instead of body-frame `R*e_i` | Medium | T3 |
| DT-76 | §8 | Pre-allocated `efc_lambda_saved` for RK4 — avoid `efc_lambda.clone()` per step | Low | T1 |
| DT-77 | §5 | Length-range auto-estimation for site-transmission muscle actuators (no-op stub) | Low | T2 |
| DT-78 | §4 | `actuator_lengthrange` for unlimited spatial tendons — wrap-array DOF lookup wrong | Low | T2 |
| DT-79 | §14 | User callbacks `mjcb_*` Rust equivalents — closures vs trait objects, thread safety | Medium | T3 |
| DT-80 | §14 | Mocap body + equality weld constraint integration testing | Low | T1 |
| DT-81 | §14 | `key_userdata` support — no `userdata` concept in CortenForge | Low | T1 |
| DT-82 | §9 | SoA layout across environments for cache locality — deferred to GPU work | Low | T3 |
| DT-83 | §9 | Multi-model batching — different robots in same batch (per-env dimensions) | Low | T3 |
| DT-84 | §32 | `mju_encodePyramid` utility not implemented — API compatibility only | Low | T1 |
| DT-91 | §2 | Warmstart `Vec<f64>` → `SmallVec<[f64; 6]>` — avoid heap allocation in warmstart vectors | Low | T1 |
| DT-92 | §9 | Parallel reset for `BatchSim` — sequential O(nq+nv+nu+na) reset deferred | Low | T1 |
