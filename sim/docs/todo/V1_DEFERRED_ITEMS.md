# v1.0 Deferred Items — Medium Priority

8 items promoted from the deferred item tracker (119 total, audited 2026-04-11).
48 confirmed DONE. 58 LOW deferred to post-v1.0. These 8 are the only MEDIUM
items still open. Full tracker detail preserved in git history
(`sim/docs/todo/future_work_10b.md` through `10j.md`).

---

| §DT | Domain | Description | Tier |
|-----|--------|-------------|------|
| DT-107 | Actuator/Sensor | Runtime interpolation logic — model/data exist, runtime wiring missing in `mj_forward` + `reset_data` | T2 |
| DT-97 | Conformance | Golden file generation for per-flag trajectory conformance (AC18) — `.npy` reference data from MuJoCo Python for 25 flags | T2 |
| DT-23 | Contact | Per-DOF friction solver params (`dof_solref_fri`/`dof_solimp_fri`) | T2 |
| DT-19 | Contact | QCQP-based cone projection — joint normal+friction projection (MuJoCo PGS style) | T3 |
| DT-32 | Tendon | Per-tendon `solref_limit`/`solimp_limit` constraint solver params | T2 |
| DT-33 | Tendon | Tendon `margin` attribute — limit activation distance (4 sites in `assembly.rs` still hardcode `< 0.0`) | T2 |
| DT-38 | Solver | Implicit matrix-vector products for CG — avoid dense Delassus assembly for large contact counts | T3 |
| DT-70 | Flex | Deformable-vs-Mesh/Hfield/SDF narrowphase — only primitives currently supported | T3 |
