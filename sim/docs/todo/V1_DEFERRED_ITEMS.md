# v1.0 Deferred Items — Medium Priority

Promoted from the [Deferred Item Tracker](./archived/future_work_10b.md).
These 8 items are the only MEDIUM-priority open items across all 119 tracked
deferred tasks. Everything else is LOW or already DONE.

**Source:** Repo audit 2026-04-11. 48 of 119 items confirmed DONE (already
marked in source files). 66 items remain open — 8 MEDIUM (this file), 58 LOW
(stay in archived tracker as post-v1.0 backlog).

---

## Items

| §DT | Domain | Description | File | Tier |
|-----|--------|-------------|------|------|
| DT-107 | Actuator/Sensor | Runtime interpolation logic — model/data exist, runtime wiring missing in `mj_forward` + `reset_data` | [10g](./archived/future_work_10g.md) | T2 |
| DT-97 | Conformance | Golden file generation for per-flag trajectory conformance (AC18) — `.npy` reference data from MuJoCo Python for 25 flags | [10j](./archived/future_work_10j.md) | T2 |
| DT-23 | Contact | Per-DOF friction solver params (`dof_solref_fri`/`dof_solimp_fri`) | [10c](./archived/future_work_10c.md) | T2 |
| DT-19 | Contact | QCQP-based cone projection — joint normal+friction projection (MuJoCo PGS style) | [10c](./archived/future_work_10c.md) | T3 |
| DT-32 | Tendon | Per-tendon `solref_limit`/`solimp_limit` constraint solver params | [10d](./archived/future_work_10d.md) | T2 |
| DT-33 | Tendon | Tendon `margin` attribute — limit activation distance (4 sites in `assembly.rs` still hardcode `< 0.0`) | [10d](./archived/future_work_10d.md) | T2 |
| DT-38 | Solver | Implicit matrix-vector products for CG — avoid dense Delassus assembly for large contact counts | [10e](./archived/future_work_10e.md) | T3 |
| DT-70 | Flex | Deformable-vs-Mesh/Hfield/SDF narrowphase — only primitives currently supported | [10i](./archived/future_work_10i.md) | T3 |

## Post-v1.0 Backlog (58 LOW items)

Grouped by domain, staying in the archived tracker files:

| Domain | Items | File |
|--------|-------|------|
| MJCF parsing & defaults | 10 open | [10b](./archived/future_work_10b.md) |
| Contact & collision | 8 open | [10c](./archived/future_work_10c.md) |
| Tendon system | 5 open | [10d](./archived/future_work_10d.md) |
| Solver optimizations | 8 open | [10e](./archived/future_work_10e.md) |
| Derivatives & analytical | 9 open | [10f](./archived/future_work_10f.md) |
| Actuator & dynamics | 7 open | [10g](./archived/future_work_10g.md) |
| Sensor gaps | 1 open (DT-65, post-v1.0) + 5 extensions | [10h](./archived/future_work_10h.md) |
| Flex / deformable | 21 open | [10i](./archived/future_work_10i.md) |
| Misc pipeline & API | 9 open | [10j](./archived/future_work_10j.md) |

## Completed Items Summary

48 items confirmed DONE across Phases 4-13:

- **Defaults & Parsing (10b):** DT-2, 3, 6, 8, 9 (partial), 11, 13, 14, 16
- **Contact (10c):** DT-21, 25 (partial), 94, 95, 99, 100
- **Tendon (10d):** DT-35
- **Solver (10e):** DT-41
- **Derivatives (10f):** DT-47, 51, 52, 53, 54
- **Actuator (10g):** DT-56, 57, 58, 59, 60, 61
- **Sensor (10h):** DT-62, 63, 64, 102, 103, 109
- **Flex (10i):** DT-85, 88, 90
- **Misc (10j):** DT-74, 75, 77, 78, 79, 93, 98 (retired)
