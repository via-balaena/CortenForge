# Future Work 10c — Deferred Item Tracker: Group 2 — Contact & Collision System

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 2 — Contact & Collision System (15 items)

**Spec approach:** DT-19/21/25 each need individual specs (T3). DT-20/24 share
a "Contact Force Cleanup" spec (T2). DT-23 joins the cross-file "Solver Param
Completeness" spec with DT-32/33 (T2). DT-26/27 share an "XPBD Improvements"
spec (T2). DT-18/22 implement directly (T1).

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-18 | §2 | Zero-friction condim downgrade optimization — detect `mu[0..dim-1] == 0` and downgrade to condim=1 | Low | T1 |
| DT-19 | §2 | QCQP-based cone projection — jointly project normal+friction forces (MuJoCo PGS style) | Medium | T3 |
| DT-20 | §2 | `J^T * lambda` vs manual chain-walk force application — unify contact force application | Low | T2 |
| DT-21 | §15 | `xfrc_applied` support in `qfrc_smooth` — external Cartesian body forces | Medium | T3 |
| DT-22 | §15 | `efc_impP` — impedance derivative field for external API introspection | Low | T1 |
| DT-23 | §15 | `dof_solref_fri` / `dof_solimp_fri` — per-DOF friction loss solver params | Medium | T2 |
| DT-24 | §16 | Incremental collision detection on tree wake — only re-collide woken geoms | Low | T2 |
| DT-25 | §11 | Deformable-rigid friction cone projection — normal-only solver is current scaffold | Medium | T3 |
| DT-26 | §11 | Contact re-detect + re-solve iteration loop after XPBD projection | Low | T2 |
| DT-27 | §11 | XPBD solver cross-iteration lambda accumulation fix | Low | T2 |
| ~~DT-94~~ | §41 | ~~BVH midphase integration into rigid body collision pipeline~~ **Subsumed by §41 S9** | Medium | T3 |
| ~~DT-95~~ | §41 | ~~Global contact parameter override~~ **Subsumed by §41 S10** | Medium | T2 |
| ~~DT-99~~ | §41 | ~~BVH midphase integration into collision pipeline (S9-full) — per-mesh BVH storage, build-phase construction, midphase dispatch~~ **Done** — `use_bvh` param on 5 mesh functions + `DISABLE_MIDPHASE` guard in `collide_with_mesh()`, AC31/AC33 tests | Medium | T3 |
| ~~DT-100~~ | §41 | ~~Global contact parameter override guard sites (S10-full) — assignment helpers, 6 guard sites in broadphase/narrowphase/constraint~~ **Done** — `assign_margin`/`assign_solref`/`assign_solimp`/`assign_friction`/`assign_solreffriction` helpers, 6 guard sites, AC34–AC37 tests | Medium | T2 |
| DT-101 | §41 | Implement `mj_contactPassive()` — viscous contact damping forces. Guard site (`DISABLE_CONTACT` early return) already specified in §41 S4.7d | Low | T2 |
