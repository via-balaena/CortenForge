# Future Work 10c — Deferred Item Tracker: Group 2 — Contact & Collision System

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 2 — Contact & Collision System (19 items)

**Spec approach:** DT-19/21/25 each need individual specs (T3). DT-20/24 share
a "Contact Force Cleanup" spec (T2). DT-23 joins the cross-file "Solver Param
Completeness" spec with DT-32/33 (T2). DT-26/27 share an "XPBD Improvements"
spec (T2). DT-18/22 implement directly (T1).

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-18 | §2 | Zero-friction condim downgrade optimization — detect `mu[0..dim-1] == 0` and downgrade to condim=1 | Low | T1 |
| DT-19 | §2 | QCQP-based cone projection — jointly project normal+friction forces (MuJoCo PGS style) | Medium | T3 |
| DT-20 | §2 | `J^T * lambda` vs manual chain-walk force application — unify contact force application | Low | T2 |
| ~~DT-21~~ | §15 | ~~`xfrc_applied` support in `qfrc_smooth` — external Cartesian body forces~~ **Done** — projection in `compute_qacc_smooth()` (acceleration stage), 4 tests | Medium | T3 |
| DT-22 | §15 | `efc_impP` — impedance derivative field for external API introspection | Low | T1 |
| DT-23 | §15 | `dof_solref_fri` / `dof_solimp_fri` — per-DOF friction loss solver params | Medium | T2 |
| DT-24 | §16 | Incremental collision detection on tree wake — only re-collide woken geoms | Low | T2 |
| ~~DT-25~~ | §11 | ~~Deformable-rigid friction cone projection — normal-only solver is current scaffold~~ **Partial** — Phase 8 Session 13 verified condim=3 works correctly (sliding friction, QCQP cone projection, R-scaling with asymmetric Jacobians, 7 tests). Remaining: condim=6 downgrades to 3 (DT-131), bodyweight diagApprox double-counts rigid body (DT-132). | Medium | T3 |
| DT-26 | §11 | Contact re-detect + re-solve iteration loop after XPBD projection | Low | T2 |
| DT-27 | §11 | XPBD solver cross-iteration lambda accumulation fix | Low | T2 |
| ~~DT-94~~ | §41 | ~~BVH midphase integration into rigid body collision pipeline~~ **Subsumed by §41 S9** | Medium | T3 |
| ~~DT-95~~ | §41 | ~~Global contact parameter override~~ **Subsumed by §41 S10** | Medium | T2 |
| ~~DT-99~~ | §41 | ~~BVH midphase integration into collision pipeline (S9-full) — per-mesh BVH storage, build-phase construction, midphase dispatch~~ **Done** — `use_bvh` param on 5 mesh functions + `DISABLE_MIDPHASE` guard in `collide_with_mesh()`, AC31/AC33 tests | Medium | T3 |
| ~~DT-100~~ | §41 | ~~Global contact parameter override guard sites (S10-full) — assignment helpers, 6 guard sites in broadphase/narrowphase/constraint~~ **Done** — `assign_margin`/`assign_solref`/`assign_solimp`/`assign_friction`/`assign_solreffriction` helpers, 6 guard sites, AC34–AC37 tests | Medium | T2 |
| DT-101 | §41 | Implement `mj_contactPassive()` — viscous contact damping forces. Guard site (`DISABLE_CONTACT` early return) already specified in §41 S4.7d | Low | T2 |
| DT-127 | Phase 8 Spec A | Mixed-sign `solref` validation — `(solref[0] > 0) ^ (solref[1] > 0)` triggers MuJoCo warning and replaces with default `[0.02, 1.0]` via `getsolparam()`. Affects ALL constraint types (limits, friction, equality, contact). CortenForge's `compute_kbip()` does not validate. Rubric gap R11. | Low | T1 |
| DT-131 | Phase 8 DT-25 | Flex condim=6 silently downgrades to condim=3 — `make_contact_flex_rigid()` in `flex_collide.rs` maps `_ => 3`, so rolling friction (condim=6) is unsupported for deformable contacts. MuJoCo C also doesn't support condim=6 for flex, so this is conformant but should emit a warning. | Low | T1 |
| DT-132 | Phase 8 DT-25 | Bodyweight diagApprox double-counts rigid body for flex contacts — `make_contact_flex_rigid()` sets `geom1==geom2==rigid_geom_idx`, so `compute_diag_approx_bodyweight()` sums `invweight0` from the same rigid body twice, ignoring flex vertex mass. Exact diagonal (`compute_diag_approx_exact()`) is correct. | Low | T2 |
| DT-133 | Phase 8 DT-25 | Bodyweight diagApprox uses rotational weight for flex friction rows — `compute_diag_approx_bodyweight()` selects `body_invweight0[b][1]` (rotational) for friction rows (`row_offset != 0`), but the flex vertex side has no angular DOFs. The translational component should be used instead, making R-scaling suboptimal for flex friction rows. Distinct from DT-132 (wrong body) — this is wrong *component*. | Low | T2 |
