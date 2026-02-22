# Future Work 10d — Deferred Item Tracker: Group 3 — Tendon System

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 3 — Tendon System (8 items)

**Spec approach:** DT-30/35 each need individual specs (T3). DT-28/31 share a
"Tendon Joint Type Completeness" spec (T2). DT-32/33 join the cross-file
"Solver Param Completeness" spec with DT-23 (T2). DT-29/34 implement directly
(T1). Totals: 2 T1, 4 T2, 2 T3.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-28 | §4 | Ball/Free joints in fixed tendons — validation + qvel DOF index mapping | Low | T2 |
| DT-29 | §4 | Spatial tendon dense J^T multiplication path (vs sparse wrap-array path) | Low | T1 |
| DT-30 | §4 | Compound pulley physics — capstan friction, pulley inertia (`sim-tendon/pulley.rs`) | Low | T3 |
| DT-31 | §4 | `WrapType::Joint` inside spatial tendons (`mjWRAP_JOINT`) — uncommon, parser rejects | Low | T2 |
| DT-32 | §4 | Per-tendon `solref_limit`/`solimp_limit` — constraint solver params (pre-existing gap) | Medium | T2 |
| DT-33 | §4 | Tendon `margin` attribute — limit activation distance (pre-existing gap) | Medium | T2 |
| DT-34 | §4 | Sparse Jacobian representation for spatial tendons — cache nonzero DOF indices | Low | T1 |
| DT-35 | §4 | Tendon spring/damper forces produce zero in implicit mode — non-diagonal K coupling needed | Medium | T3 |
