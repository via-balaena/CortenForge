# CortenForge Sim — v1.0 Roadmap

> **Status**: Draft — 2026-03-02
> **Scope**: All remaining work from `future_work_10.md` (§41+) through `future_work_17.md`,
> plus the ~112 deferred tasks in `future_work_10b.md`–`10j` (DT-1 through DT-116).
> DT-93/94/95 were added during §41 spec and subsumed into §41.
> DT-96 (lazy energy eval) and DT-97 (golden file conformance) added during §41 audit.
> ~~DT-99~~ (BVH midphase, §41 S9-full — **done**), ~~DT-100~~ (global override, §41 S10-full — **done**),
> DT-101 (`mj_contactPassive()`) added during §41 spec expansion.
> ~~DT-98~~ retired — `passive` dropped entirely pre-v1.0 (no shim needed).
> DT-107 (runtime interpolation), DT-108 (dyntype gating), DT-109 (sensor history),
> DT-110 (actuator_plugin) added during Spec D review. ~~DT-9~~ partially done (parsing landed).
> ~~DT-58~~ done (Phase 5 Spec C). DT-111–116 (HillMuscle extension sub-items) added
> during Spec C review. DT-123/124 added during Phase 7 Spec A review.
> ~~§60~~ dropped (nonexistent in MuJoCo). DT-125 (`mj_setConst()` runtime
> `qpos_spring` recomputation) added during Phase 7 Spec B review.
> DT-128 (PGS early termination) added during Phase 8 Spec B rubric stress-test.
> DT-129 (PGS warmstart two-phase projection), DT-130 (dense AR optimization)
> added during Phase 8 Spec B review.
> DT-134 (mesh-primitive hull dispatch), DT-135 (`needhull_` trigger),
> DT-136 (GPU convex hull) added during Phase 9 Spec A review.
>
> **Current position**: Phases 1–7 complete. Next: Phases 8–11 (parallel).

---

## What is v1.0?

v1.0 is a Rust implementation of MuJoCo's core simulation pipeline that:

1. Parses standard MJCF models correctly
2. Produces physics results that match MuJoCo 3.4.0 within tolerance
3. Exposes the standard MuJoCo public API surface (step, forward, inverse, derivatives, name lookup, callbacks)
4. Passes the conformance test suite (§45)

v1.0 is **not**: GPU acceleration, trait-based solver extensibility, NeoHookean soft bodies,
URDF completeness, XPBD, or any feature that doesn't exist in the MuJoCo C library.

---

## Summary

| Category | Core v1.0 | Extra / Post-v1.0 |
|----------|----------:|-----------:|
| future_work_10 (§41+) | 5 | 8 |
| future_work_10b–10j (DT-1–92) | ~40 | ~52 |
| future_work_11 (§43–45) | 2 | 1 |
| future_work_12 (§46–50) | 3 | 2 |
| future_work_13 (§51–55) | 5 | 0 |
| future_work_14 (§56–59) | 4 | 0 |
| future_work_15 (§60–64a) | 6 | 0 |
| future_work_16 (§65–66) | 2 | 0 |
| future_work_17 (§67–71) | 0 | 5 |
| **Total** | **~67** | **~68** |

---

## Spec Tiers

Each DT item has a **Tier** governing the spec approach:

- **T1** — Plan + implement directly. Mechanical work; parent spec defines the "what."
- **T2** — Grouped spec. Related items share one spec; each gets a "Step N" section. (~15 spec groups)
- **T3** — Individual spec. Algorithmic complexity or architectural decisions needing dedicated design.
- **—** — Non-DT tasks (§-numbered). Specced in their own future_work_*.md files.

See [future_work_10b.md](./todo/future_work_10b.md) for the full T2 group listing.

---

## v1.0 Critical Path

Ordered by priority. Each phase should be completed and committed before
moving to the next. Within a phase, tasks are independent and can be
parallelized.

---

### Phase 1 — Correctness Bugs

Fix known physics bugs before building more on top of them. These are small,
high-value, and reduce the chance of compounding errors in later work.

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| ~~DT-74~~ | 10j | T3 | ~~`compute_body_jacobian_at_point()` incomplete — only x-component implemented~~ **DONE** — canonical `mj_jac` API, dead code deleted, 14 tests |
| ~~DT-75~~ | 10j | T3 | ~~`add_body_jacobian` free joint bug — world-frame vectors instead of body-frame `R*e_i`~~ **DONE** — body-frame axes fix in 3 locations, 6 tests |
| ~~DT-35~~ | 10d | T3 | ~~Tendon spring/damper forces produce zero in implicit mode — missing non-diagonal K coupling~~ **DONE** — implicit tendon K/D via `accumulate_tendon_kd`, Newton `M_impl`/`qfrc_eff`, `ten_force` diagnostic always populated, 18 tests |
| ~~DT-16~~ | 10b | T1 | ~~Flex `density` attribute location wrong in parser vs MuJoCo spec~~ **DONE** — removed non-conformant density parsing from `parse_flex_attrs()` |
| ~~DT-90~~ | 10i | T1 | ~~`flex_friction` scalar should be `Vector3<f64>` — torsional/rolling friction data lost~~ **DONE** — `flex_friction` upgraded to `Vec<Vector3<f64>>` end-to-end (parser → builder → model → collision) |
| ~~DT-78~~ | 10j | T2 | ~~`actuator_lengthrange` DOF lookup wrong for unlimited spatial tendons~~ **DONE** — spatial tendon guard in `muscle.rs`, landed in §4 step 5 |

---

### Phase 2 — Runtime Flag Wiring ✅

> **Complete.** Full 7-phase audit passed (Phases 1–7, all A on R1–R9 rubric).
> 13 discrepancies found and fixed during audit. 2,107 tests pass, 0 fail.
> See [audit plan](todo/spec_fleshouts/s41_runtime_flags/S41_AUDIT_PLAN.md) and [rubric](todo/spec_fleshouts/s41_runtime_flags/S41_AUDIT_RUBRIC.md).

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| ~~§41~~ | 10 | — | ~~Wire all 19 `disableflags` and 6 `enableflags` end-to-end. Subsumes DT-60, DT-61, DT-93, DT-94, DT-95.~~ **Done** — all AC1–AC48 conformance tests pass. See [spec](todo/spec_fleshouts/s41_runtime_flags/S41_RUNTIME_FLAGS_SPEC.md). |
| ~~DT-99~~ | 10c | T2 | ~~BVH midphase integration into collision pipeline (§41 S9-full, post-§41 commit)~~ **Done** — `use_bvh` param + `DISABLE_MIDPHASE` guard in `collide_with_mesh()`, AC31/AC33 conformance tests |
| ~~DT-100~~ | 10c | T2 | ~~Global contact parameter override guard sites (§41 S10-full, post-§41 commit)~~ **Done** — `assign_margin`/`assign_solref`/`assign_solimp`/`assign_friction`/`assign_solreffriction` helpers, 6 guard sites in broadphase/narrowphase/constraint, AC34–AC37 conformance tests |

---

### Phase 3 — Core API Gaps

Public API functions that MuJoCo exposes and users/conformance tests expect.

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| ~~DT-21~~ | 10c | T3 | ~~`xfrc_applied` support in `qfrc_smooth` — external Cartesian body forces~~ **Done** — projection in `mj_fwd_passive()`, 4 tests |
| ~~DT-41~~ | 10e | T3 | ~~Newton solver for implicit integrators (currently warns + falls back to PGS)~~ **Done** |
| ~~§51~~ | 13 | — | ~~`cacc`, `cfrc_int`, `cfrc_ext` — per-body 6D force/acceleration accumulators~~ **Done** — RNE forward+backward pass, 4 tests |
| ~~§52~~ | 13 | — | ~~`mj_inverse()` — inverse dynamics API computing `qfrc_inverse`~~ **Done** — `M*qacc + qfrc_bias - qfrc_passive`, 3 tests |
| ~~§53~~ | 13 | — | ~~`step1()`/`step2()` split stepping API for control injection between forward and integrate~~ **Done** — 2 tests |
| ~~§59~~ | 14 | — | ~~`mj_name2id`/`mj_id2name` — bidirectional name-index lookup~~ **Done** — `ElementType` enum, O(1) HashMap, 4 tests |
| ~~DT-79~~ | 10j | T3 | ~~User callbacks `mjcb_*` Rust equivalents~~ **Done** — `Callback<F>` Arc wrapper, 5 tests |

---

### Phase 4 — Core Data Fields ✅

> **Complete (2026-02-26).** All 5 deliverables shipped and audited: CVEL reference
> point fixes (commit `444046e`), §56 persistent subtree fields (commit `503ac6d`),
> lazy evaluation gates (commit `8e8f5f7`), acc-stage sensor refactor (commit
> `16cfcb3`), DT-103 spatial transport helpers (commit `29501df`). 41 acceptance
> criteria, 39 new tests, 2,148+ domain tests pass. All specs A+, implementation
> review 6/6 A. See [Phase 4 audit](todo/spec_fleshouts/phase4_lazy_eval/PHASE4_AUDIT.md)
> and [DT-103 review](todo/spec_fleshouts/phase4_lazy_eval/DT103_IMPLEMENTATION_REVIEW.md).

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| ~~§51~~ | 13 | — | ~~`cacc`, `cfrc_int`, `cfrc_ext` — per-body 6D force/acceleration accumulators~~ **Done** — moved to Phase 3 |
| ~~§56~~ | 14 | — | ~~`subtree_linvel`, `subtree_angmom` — promote from sensor helpers to persistent fields~~ **Done** — persistent fields + lazy gates + sensor refactor + DT-103 transport helpers |

---

### Phase 5 — Actuator Completeness ✅

> **Complete (2026-02-27).** All 10 originally-scoped tasks ship-complete across 19
> sessions (4 sub-specs + 2 T1 items). 2,238+ domain tests pass, 0 fail, clippy
> clean, fmt clean. See [SESSION_PLAN.md](todo/spec_fleshouts/phase5_actuator_completeness/SESSION_PLAN.md)
> for full session history and commit hashes.
>
> Deferred items discovered during implementation (DT-104, DT-105, DT-107, DT-108,
> DT-110, DT-111–116) tracked in post-v1.0 sections below.

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| ~~DT-6~~ | 10b | T1 | ~~`actearly` attribute wired to runtime~~ **Done** — Phase 5 Session 1 (commit `dc12b8b`). Already wired; verified + 4 tests added. |
| ~~§63~~ | 15 | — | ~~`dynprm` array 3→10 elements to match MuJoCo~~ **Done** — Phase 5 Session 1 (commit `d4db634`). |
| ~~DT-57~~ | 10g | T2 | ~~`acc0` computation for non-muscle actuators~~ **Done** — Spec A (Phase 5 Session 3, commit `a1cbbba`). Extended `compute_actuator_params()` to all actuator types. |
| ~~DT-56~~ | 10g | T2 | ~~`dampratio` for position actuators~~ **Done** — Spec A (Phase 5 Session 3, commit `a1cbbba`). Dampratio-to-damping conversion using `acc0`. |
| ~~DT-59~~ | 10g | T2 | ~~Bisection-based `lengthrange` for unlimited slide joints~~ **Done** — Spec A (Phase 5 Session 3, commit `a1cbbba`). Simulation-based LR estimation. |
| ~~DT-77~~ | 10j | T2 | ~~Length-range auto-estimation for site-transmission actuators~~ **Done** — Spec A (Phase 5 Session 3, commit `a1cbbba`). Site-transmission LR via actuator_moment build. |
| ~~DT-8~~ | 10b | T2 | ~~Transmission types: `cranksite`, `slidersite`, `jointinparent`~~ **Done** — Spec B (Phase 5 Session 7, commit `aa87169`). |
| ~~§61~~ | 15 | — | ~~`slidercrank` actuator transmission~~ **Done** — Spec B (Phase 5 Session 7, commit `aa87169`). |
| ~~DT-9~~ | 10b | T2 | ~~`nsample`, `interp`, `delay` — MuJoCo 3.x interpolation attributes~~ **Partially done** — parsing + model/data storage landed in Spec D (Phase 5 Session 12, commit `cc996c4`). Runtime → DT-107, dyntype gating → DT-108. |
| ~~DT-58~~ | 10g | T3 | ~~sim-muscle Hill-type model as `ActuatorDynamics::HillMuscle` variant~~ **Done** — Spec C (Phase 5 Session 17, commit `c64bab1`). Rigid tendon only; extension sub-items → DT-111–116. |
| ~~DT-60~~ | 10g | T1 | ~~`jnt_actgravcomp` routing to `qfrc_actuator`~~ **Done** — subsumed by §41 S4.2a. |
| ~~DT-106~~ | Spec A | T1 | ~~Gear-scaling in `uselimit` lengthrange~~ **Done** — intentional MuJoCo deviation documented (commit `456fc9e`). |

---

### Phase 6 — Sensor Completeness ✅

> **Complete (2026-03-01).** All 6 tasks ship-complete across 20 sessions (4 sub-specs:
> A, B, C, D). 1,900+ domain tests pass, 0 fail, clippy clean, fmt clean. See
> [SESSION_PLAN.md](todo/spec_fleshouts/phase6_sensor_completeness/SESSION_PLAN.md)
> for full session history and commit hashes.
>
> Deferred items discovered during implementation (DT-118, DT-119, DT-120,
> DT-121, DT-122) tracked in post-v1.0 sections below. DT-65 (User sensor dim)
> already tracked. DT-107/DT-108 updated to cover sensor-side runtime scope.

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| ~~§62~~ | 15 | — | ~~Missing sensor types: `clock`, `jointactuatorfrc`, `camprojection`, `geomdist`, `geompoint`, `geomnormal`~~ **Done** — Spec C (Phase 6 Sessions 11–15). 6 new `MjSensorType` variants + evaluation arms. |
| ~~DT-62~~ | 10h | T2 | ~~Frame sensor `objtype` attribute not parsed — fallback heuristic used~~ **Done** — Spec A (Phase 6 Sessions 1–5). Explicit `objtype` parsing + dispatch. |
| ~~DT-63~~ | 10h | T2 | ~~Frame sensor `reftype`/`refid` — relative-frame measurements~~ **Done** — Spec B (Phase 6 Sessions 6–10). Reference-frame transforms in all 9 frame sensor arms. |
| ~~DT-64~~ | 10h | T2 | ~~Multi-geom touch sensor aggregation (currently only first geom matched)~~ **Done** — Spec A (Phase 6 Sessions 1–5). Body-level aggregation matching MuJoCo. |
| ~~DT-102~~ | 10h | T1 | ~~Geom-attached acc-stage sensors (FrameLinAcc/FrameAngAcc with `MjObjectType::Geom`). Depends on DT-62.~~ **Done** — Spec A (Phase 6 Sessions 1–5). Full `mj_objectAcceleration()` at geom position. |
| ~~DT-109~~ | 10h | T2 | ~~Sensor history attributes — `nsample`/`interp`/`delay`/`interval` on sensors, contributes to `nhistory`.~~ **Done** — Spec D (Phase 6 Sessions 16–20). 5 model fields, historyadr computation, validation. Runtime → DT-107. |

---

### Phase 7 — MJCF Parsing & Defaults Gaps ✅

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| ~~DT-2~~ | 10b | T2 | ~~Equality constraint defaults: `solref`/`solimp` in defaults structs~~ **Done** — Phase 7 Spec A (commit `01ae59f`). `MjcfEqualityDefaults` struct, `apply_to_equality()` cascade for all 5 equality types. |
| ~~DT-3~~ | 10b | T1 | ~~File-based hfield loading from PNG~~ **Done** — Phase 7 T1 (commit `cea5f4c`). `file` attribute on `<hfield>`, PNG grayscale loading via `image` crate. |
| ~~DT-11~~ | 10b | T2 | ~~`range` as defaultable attribute in `MjcfJointDefaults`~~ **Already implemented** — verified during Phase 7 Spec A review (EGT-4). Dropped from spec scope. |
| ~~DT-13~~ | 10b | T2 | ~~`qpos_spring` — distinct from `qpos0`, not yet implemented~~ **Done** — Phase 7 Spec A (commit `3f70616`). `qpos_spring: Vec<f64>` on Model, populated from `qpos0`/`springref`. Runtime consumers in `passive.rs`/`energy.rs`. |
| ~~DT-14~~ | 10b | T2 | ~~Actuator type-specific defaults (cylinder area/timeconst, muscle params)~~ **Done** — Phase 7 Spec A (commit `01ae59f`). Shortcut names in `parse_default()`, cylinder/muscle/adhesion fields on `MjcfActuatorDefaults`. |
| ~~DT-85~~ | 10i | T1 | ~~Flex `<contact>` runtime attributes: `internal`, `activelayers`, `vertcollide`, `passive`~~ **Done** — Phase 7 T1 (commit `cf76731`). Parse + store + wire to Model arrays. |
| ~~DT-88~~ | 10i | T2 | ~~`<flexcomp>` attributes: `inertiabox`, `scale`, `quat`, `file`~~ **Done** — Phase 7 Spec C (commit `05ee0a5`). Documentation-fidelity (not in MuJoCo 3.5.0 binary). |
| ~~§55~~ | 13 | — | ~~Per-element `*_user` custom data arrays from MJCF~~ **Done** — Phase 7 Spec C (commit `05ee0a5`). 7 element types, `<size>` nuser_*, auto-sizing, zero-padding, validation, defaults cascade. |
| ~~§60~~ | 15 | — | ~~`springinertia` joint attribute — inertia-spring coupling in CRBA diagonal~~ **DROPPED** — Verified nonexistent in MuJoCo (Phase 7 Spec B rubric EGT-1: zero GitHub results, no `mjmodel.h` field, no XML attribute). |
| ~~§64~~ | 15 | — | ~~Ball/free joint spring force and energy (quaternion geodesic). Depends on Phase 7 Spec A `qpos_spring` array.~~ **Done** — Phase 7 Spec B (commit `3f70616`). Ball/free spring force in `passive.rs`, spring energy in `energy.rs`, quaternion geodesic via `subquat()`. |
| ~~§64a~~ | 15 | — | ~~`jnt_margin` for joint limit activation and constraint row margin~~ **Done** — Phase 7 Spec B (commit `3f70616`). `margin` parsed from `<joint>`, `jnt_margin: Vec<f64>` on Model, 9 sites in `assembly.rs` replaced. |

---

### Phase 8 — Constraint & Solver Gaps

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| DT-19 | 10c | T3 | QCQP-based cone projection for normal+friction force projection (MuJoCo PGS style) |
| DT-128 | 10e | T2 | PGS early termination — accumulate `improvement` from `costChange()`, break when `improvement * scale < tolerance`. Currently always runs `max_iters`. MuJoCo's `mj_solPGS` has this; CortenForge PGS does not. Discovered during Phase 8 Spec B rubric stress-test. |
| DT-23 | 10c | T2 | Per-DOF friction loss solver params (`dof_solref_fri`/`dof_solimp_fri`) |
| ~~DT-25~~ | 10c | T3 | ~~Deformable-rigid friction cone projection (currently normal-only)~~ **Partial** — Phase 8 Session 13 verification: condim=3 fully works (QCQP cone projection, R-scaling, Jacobian). Remaining gaps: condim=6 silently downgrades to 3 (DT-131), bodyweight diagApprox double-counts rigid body (DT-132), bodyweight uses rotational weight for flex friction rows (DT-133). 7 integration tests. |
| DT-28 | 10d | T2 | Ball/free joints in fixed tendons — validation + qvel DOF index mapping |
| ~~DT-32~~ | 10d | T2 | ~~Per-tendon `solref_limit`/`solimp_limit` constraint solver params~~ — **Done** (Phase 8: naming conformance) |
| DT-33 | 10d | T2 | Tendon `margin` attribute for limit activation distance. Phase 7 Spec B (§64a) implemented joint `jnt_margin`; tendon limit sites (`assembly.rs:148,152,540,564`) still hardcode `< 0.0`. |
| DT-39 | 10e | T2 | Body-weight diagonal approximation (`diagApprox`) |
| DT-129 | 10e | T3 | PGS warmstart two-phase projection — use ray+QCQP projection on warmstart forces for better initial guess. Currently warmstart comes from `classify_constraint_states` which derives forces from `qacc_warmstart`. Discovered during Phase 8 Spec B review. |
| DT-130 | 10e | T3 | Dense AR matrix optimization — PGS currently computes full nefc×nefc `efc_AR` matrix. MuJoCo uses sparse row-level operations (`ARblock`). Performance optimization, not conformance. Discovered during Phase 8 Spec B review. |

---

### Phase 9 — Collision Completeness

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| §43 | 11 | — | Mesh inertia modes: exact, shell, convex, legacy |
| §50 | 12 | — | Continuous Collision Detection (conservative-advancement CCD, tunneling prevention) |
| §54 | 13 | — | Missing heightfield collision pairs: hfield-mesh, hfield-plane, hfield-hfield |
| §57 | 14 | — | `sdf_iterations`/`sdf_initpoints` from `<option>` (replace hardcoded values) |
| ~~§65~~ | 16 | — | ~~Mesh convex hull auto-computation (Quickhull at build time for GJK/EPA)~~ **Done** — Phase 9 Spec A |
| DT-70 | 10i | T3 | Deformable-vs-mesh/hfield/SDF narrowphase (only primitives currently) |
| DT-134 | 16 | T2 | Mesh-primitive dispatch to GJK/EPA on convex hulls — mesh-sphere, mesh-capsule, mesh-box pairs currently use per-triangle BVH; should route through `convex_mesh_from_hull()` + `gjk_epa_contact()` when hull available (AD-1 option a). Deferred from Phase 9 Spec A. |

---

### Phase 10 — Flex Pipeline

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| §42A-i | 10 | — | Sparse flex edge Jacobian (`flexedge_J`) — force projection through body Jacobians |
| §42A-ii | 10 | — | `flex_rigid`/`flexedge_rigid` boolean arrays — skip rigid bodies/edges |
| §42A-iii | 10 | — | `flexedge_length`/`flexedge_velocity` pre-computed Data fields |
| §42A-iv | 10 | — | Flex self-collision dispatch (BVH/SAP midphase + narrowphase) |
| §42A-v | 10 | — | Flex-flex cross-object collision filtering (contype/conaffinity) |
| §42B | 10 | — | Flex bending: cotangent Laplacian (MuJoCo's actual bending formulation) |

---

### Phase 11 — Derivatives

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| §58 | 14 | — | `mjd_smooth_pos` analytical position derivatives |
| DT-47 | 10f | T2 | Sensor derivatives (C, D matrices) for `TransitionMatrices` |
| DT-51 | 10f | T2 | `mjd_inverseFD` — inverse dynamics derivatives |
| DT-52 | 10f | T2 | `mjd_subQuat` — quaternion subtraction Jacobians |
| DT-53 | 10f | T2 | `mj_forwardSkip` — skip-stage optimization for ~50% FD cost reduction |
| DT-54 | 10f | T2 | Muscle actuator velocity derivatives — piecewise FLV curve gradients |

---

### Phase 12 — Conformance Test Suite

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| §45 | 11 | — | Four-layer conformance test suite: self-consistency, per-stage reference, trajectory comparison, property/invariant tests against MuJoCo 3.4.0 |
| DT-97 | 10j | T2 | Golden file generation for per-flag trajectory conformance (AC18 of §41 — bootstrap pattern reused by §45) |

This is the gate. Run it, identify failures, iterate on phases 1–11 until
green.

---

### Phase 13 — Remaining Core

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| §46 | 12 | — | `<composite>` procedural body generation (grid, rope, cable, cloth, box, cylinder, ellipsoid) |
| §66 | 16 | — | Plugin/extension system — `<plugin>`/`<extension>` MJCF parsing + Rust trait dispatch |

---

## Post-v1.0 — Extra Features & Extensions

Everything below is explicitly **out of scope for v1.0**. These are tracked
and valued, but shipping them before conformance is wasted effort if the
foundation isn't right.

### Trait Architecture & Solver Extensibility
| Task | Source | Tier | Description |
|------|--------|------|-------------|
| §42C | 10 | — | `FlexElasticityModel` trait — NeoHookean hyperelastic model |
| §42D | 10 | — | `ActuatorGainModel` trait — Series Elastic Actuator model |
| §42E | 10 | — | Contact Solver trait — XPBD, impulse-based formulations |
| §42F | 10 | — | `SimBuilder` generic composition API |

### GPU Pipeline (future_work_17)
| Task | Source | Tier | Description |
|------|--------|------|-------------|
| DT-136 | 16 | T3 | GPU convex hull computation — move Quickhull to compute shader for large meshes. Pure performance optimization, no conformance impact. Deferred from Phase 9 Spec A. |
| §67 | 17 | — | GPU forward kinematics (level-set parallel tree traversal) |
| §68 | 17 | — | GPU collision broad-phase (parallel SAP / spatial hashing) |
| §69 | 17 | — | GPU collision narrow-phase (GJK/EPA on compute shaders) |
| §70 | 17 | — | GPU constraint solver (Jacobi-style parallel PGS) |
| §71 | 17 | — | Full GPU step chaining (single command buffer submission) |

### Performance Optimizations
| Task | Source | Tier | Description |
|------|--------|------|-------------|
| §40d | 10 | — | Sparse Jacobian for fluid derivatives (nv > 200) |
| §40e | 10 | — | Refactor `mj_jac_site` to use `mj_jac_point` kernel |
| §48 | 12 | — | SIMD batch audit for hot paths |
| DT-18 | 10c | T1 | Zero-friction condim downgrade optimization |
| DT-20 | 10c | T2 | Unify `J^T * lambda` vs chain-walk contact force application |
| DT-24 | 10c | T2 | Incremental collision detection on tree wake |
| DT-29 | 10d | T1 | Spatial tendon dense J^T multiplication path |
| DT-34 | 10d | T1 | Sparse Jacobian for spatial tendons |
| DT-36 | 10e | T2 | Flat CSR for `qLD_L` (replace `Vec<Vec<…>>`) |
| DT-37 | 10e | T2 | Sparse factorization for implicit integrator |
| DT-38 | 10e | T3 | Implicit matrix-vector products for CG |
| DT-40 | 10e | T2 | LDL^T factorization for Hessian robustness |
| DT-42 | 10e | T3 | Per-island Newton solver dispatch |
| DT-43 | 10e | T1 | Selective CRBA backward-pass per island |
| DT-44 | 10e | T2 | Sparse mass matrix for island extraction |
| DT-48 | 10f | T2 | Sparse derivative storage (nv > 100) |
| DT-49 | 10f | T2 | Parallel FD computation |
| DT-55 | 10f | T1 | `skipfactor` / factorization reuse |
| DT-76 | 10j | T1 | Pre-allocated `efc_lambda_saved` for RK4 |
| DT-91 | 10j | T1 | Warmstart `SmallVec` optimization |
| DT-92 | 10j | T1 | Parallel reset for `BatchSim` |
| DT-105 | 10e | T3 | Sparse `actuator_moment` compression (CSR) — numerically equivalent to current dense storage. Deferred from Phase 5 Spec B. |

### Advanced Differentiation
| Task | Source | Tier | Description |
|------|--------|------|-------------|
| DT-45 | 10f | T3 | Full position-analytical derivatives (`dFK/dq`, `dM/dq`) |
| DT-46 | 10f | T3 | Contact-analytical derivatives via implicit function theorem |
| DT-50 | 10f | T3 | Automatic differentiation (dual numbers / enzyme) |

### Non-MuJoCo Extensions
| Task | Source | Tier | Description |
|------|--------|------|-------------|
| DT-4 | 10b | T1 | `<sdf>` inline distance grid asset element |
| DT-111 | 10g | T3 | HillMuscle compliant tendon mode — persistent fiber state (`act_num ≥ 2` or separate state array) + custom integration |
| DT-112 | 10g | T1 | HillMuscle named MJCF attributes (`optlen`, `slacklen`, `pennation`) — convenience UX over raw `gainprm` indices |
| DT-113 | 10g | T1 | `<hillmuscle>` shortcut element — analogous to `<muscle>`, auto-sets dyntype/gaintype/biastype |
| DT-114 | 10g | T2 | HillMuscle variable pennation angle — `α = asin(w / L_fiber)` as function of fiber length |
| DT-115 | 10g | T2 | HillMuscle configurable curve parameters — Gaussian FL widths, FV curvature, FP shape via `gainprm`/`biasprm` |
| DT-116 | 10g | T3 | Per-actuator `GainType::User` / `BiasType::User` callback infrastructure — depends on §66 (plugin system) |
| DT-26 | 10c | T2 | Contact re-detect + re-solve iteration after XPBD |
| DT-27 | 10c | T2 | XPBD cross-iteration lambda accumulation fix |
| DT-30 | 10d | T3 | Compound pulley physics (capstan friction, pulley inertia) |
| DT-66 | 10i | T3 | `<equality><flex>` constraints — flex-flex coupling |
| DT-67 | 10i | T3 | GPU flex pipeline |
| DT-68 | 10i | T1 | Per-vertex material variation for flex |
| DT-73 | 10i | T3 | Volume constraints for flex bodies |
| DT-82 | 10j | T3 | SoA layout across environments |
| DT-83 | 10j | T3 | Multi-model batching |
| DT-86 | 10i | T1 | `elastic2d` keyword on `<flex><elasticity>` |
| DT-87 | 10i | T2 | Shared-body flex vertices |

### §41 Follow-Ons (Post-§41, Low Priority)
| Task | Source | Tier | Description |
|------|--------|------|-------------|
| DT-96 | 10j | T1 | Lazy energy evaluation (`flg_energypos`/`flg_energyvel` — only matters with plugins or energy-dependent sensors) |
| DT-101 | 10c | T2 | `mj_contactPassive()` — viscous contact damping forces (must be called after `qfrc_passive` aggregation, see §41 S4.7e) |

### Low-Priority MuJoCo Compat
| Task | Source | Tier | Description |
|------|--------|------|-------------|
| DT-1 | 10b | T1 | Mesh defaults `apply_to_mesh()` |
| DT-5 | 10b | T2 | `gaintype/biastype/dyntype="user"` callback-based types |
| DT-7 | 10b | T1 | `actdim` explicit override |
| DT-10 | 10b | T1 | Deferred `<compiler>` attributes (`fitaabb`, `usethread`, etc.) |
| DT-12 | 10b | T1 | Programmatic enforcement that `worldbody.childclass` = None |
| DT-15 | 10b | T1 | Sentinel-value detection → `Option<T>` migration. Phase 7 Spec A added 14 sentinel-detection fields in `apply_to_actuator()` (defaults.rs:441-512) — primary migration candidates. |
| DT-17 | 10b | T1 | Global `<option o_margin>` override. Phase 7 Spec B (§64a) implemented per-joint `jnt_margin`; this covers the separate global option. |
| DT-22 | 10c | T1 | `efc_impP` impedance derivative field |
| DT-31 | 10d | T2 | `WrapType::Joint` inside spatial tendons |
| DT-135 | 16 | T1 | `needhull_` collision-only hull trigger — compute convex hulls only for meshes used in collision (matching MuJoCo's `needhull_` flag) instead of all meshes unconditionally (current AD-3 option A). Conformance-neutral — no behavioral difference. Deferred from Phase 9 Spec A. |
| DT-65 | 10h | T1 | User sensor `dim` attribute. Also requires `sensor_intprm` array (`mjmodel.h:1213`). |
| DT-69 | 10i | T2 | SAP for flex broadphase (currently brute-force) |
| DT-72 | 10i | T1 | Flex contacts wired for adhesion |
| DT-80 | 10j | T1 | Mocap body + equality weld integration testing |
| DT-81 | 10j | T1 | `key_userdata` support |
| DT-84 | 10j | T1 | `mju_encodePyramid` utility |
| DT-89 | 10i | T1 | `<flexcomp>` rendering attributes |
| DT-104 | 10b | T2 | Ball/free joint transmission — `nv == 3` and `nv == 6` sub-paths in `mj_transmission()`. Deferred from Phase 5 Spec B. |
| DT-107 | 10g | T2 | Runtime interpolation logic — `mj_forward` reads history buffer for delayed ctrl, `mj_step` writes circular buffer. Covers both actuators (Phase 5 Spec D) and sensors (Phase 6 Spec D): structures exist for both, runtime missing for both. Includes sensor history pre-population in `reset_data()`. |
| DT-108 | 10g | T1 | `dyntype` enum gating interpolation eligibility — restrict which `ActuatorDynamics` variants may use history buffer. Deferred from Phase 5 Spec D. |
| DT-110 | 10g | T1 | `actuator_plugin` model array — per-actuator plugin ID (`int[nu]`, -1 sentinel). Depends on §66. Deferred from Phase 5 Spec D. |
| DT-118 | 15 | T2 | `mj_contactForce()` — touch sensor force reconstruction via full contact force vector. Deferred from Phase 6 Spec A. |
| DT-119 | 15 | T2 | Ray-geom intersection filter for touch sensor — filter contacts by surface normal alignment. Depends on DT-118. Deferred from Phase 6 Spec A. |
| DT-120 | 15 | T1 | `MjObjectType::Camera` — frame sensor camera support (reftype="camera" currently warns + ignores). Deferred from Phase 6 Spec B. |
| DT-121 | 15 | T1 | `InsideSite` sensor (`mjSENS_INSIDESITE`) — MuJoCo 3.x sensor type not yet supported. Deferred from Phase 6 Spec C. |
| DT-122 | 15 | T2 | Mesh/Hfield/SDF geom distance support for `GeomDist`/`GeomPoint`/`GeomNormal` sensors. Deferred from Phase 6 Spec C. |
| DT-123 | 10b | T1 | `IntVelocity` enum variant — concrete `<intvelocity>` elements not yet supported (defaults parsing works). Deferred from Phase 7 Spec A. |
| DT-124 | 10b | T1 | Muscle sentinel detection for `<general dyntype="muscle">` path (`gainprm[0]==1` quirk). Known conformance divergence. Deferred from Phase 7 Spec A. |
| DT-125 | 15 | T2 | `mj_setConst()` runtime `qpos_spring` recomputation — when `mj_setConst()` is called at runtime, `qpos_spring` must be recomputed from current `qpos0`/`springref` via `setSpring()` logic (`engine_setconst.c`). Currently `qpos_spring` is set at build time and static. Deferred from Phase 7 Spec B. |

### Code Quality
| Task | Source | Tier | Description |
|------|--------|------|-------------|
| DT-117 | xtask check | T2 | Eliminate `unwrap()`/`expect()` from library code (~1,085 call sites). Convert to `?` propagation, `Result` returns, or `unsafe { get_unchecked() }` with safety invariant comments where bounds are structurally guaranteed. Prevents unrecoverable panics for library consumers. |

### Other Non-Critical
| Task | Source | Tier | Description |
|------|--------|------|-------------|
| §42 | 10 | — | `<flex>`/`<flexcomp>` parsing (subsumed by §6B) |
| §44 | 11 | — | Deprecate legacy standalone crates |
| §47 | 12 | — | URDF loader completeness (mesh collision, mimic joints, etc.) |
| §49 | 12 | — | Non-physics MJCF elements (`<visual>`, `<statistic>`, `<custom>`, `<size>`) |
| DT-71 | 10i | T1 | Behavioral friction tests for deformables |

---

## Decision Log

| Date | Decision |
|------|----------|
| 2026-02-21 | Triaged all ~135 remaining tasks into core v1.0 (~67) vs extra (~68). Core defined as "matches MuJoCo C library behavior, passes conformance." |
| 2026-02-21 | Split `future_work_10b.md` into 9 files (10b–10j), one per thematic group. Updated all ~75 back-references across future_work_1–9, index.md, and ROADMAP. |
| 2026-02-21 | Classified all 92 DT items into spec tiers: T1 (32 plan+implement), T2 (39 grouped into ~15 specs), T3 (21 individual specs). Added Tier column to all 10b–10j tables and ROADMAP. |
