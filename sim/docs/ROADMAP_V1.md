# CortenForge Sim — v1.0 Roadmap

> **Status**: **Complete** — 2026-03-11
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
> DT-137 (concave mesh test), DT-138 (GPU inertia), DT-139
> (`exactmeshinertia` removal) added during Phase 9 Spec B review.
> DT-140 (prism BVH acceleration) added during Phase 9 Spec C review.
> DT-141 (GJK/EPA cross-frame warm-start) added during Phase 9 Spec D review.
> DT-142 (flex self-collision), DT-143 (flex-flex cross-body filtering),
> DT-144 (prism-based hfield for flex) added during Phase 9 Spec E review.
> DT-148 (hinge topology optimization) added during Phase 10 Spec B review.
> DT-150 (activelayers filtering), DT-151 (edge-edge tet self-collision),
> DT-152 (barycentric face distribution) added during Phase 10 Spec C review.
> DT-153 (island flex contact assignment), DT-154 (flex condim=6 mapping),
> DT-155 (S10 override test for flex-flex), DT-156 (narrowphase contact count gap)
> added during Phase 10 Spec D review.
> DT-161 (pyramidal diagApprox factor-of-2) added during Phase 13 Spec A review.
> DT-162 (PGS nactive/nchange counting), DT-163 (PGS warmstart primal cost gate)
> added during Phase 13 Spec B implementation.
> DT-164 (Newton solver golden flag convergence) added during Phase 13 Session 8 review.
> DT-165 (cable skin), DT-166 (composite custom text), DT-167 (flexcomp element),
> DT-168 (replicate element), DT-169 (cable site template) added during Phase 13 Spec C review.
> DT-170 (SDF collision dispatch), DT-171 (resource providers/decoders),
> DT-172 (plugin copy/destroy callbacks), DT-173 (Data clone plugin_data),
> DT-174 (sensor cutoff stage check) added during Phase 13 Spec D review.
> ~~§66~~ done (Phase 13 Spec D). DT-110 dependency on §66 satisfied.
>
> **Current position**: All 13 phases complete. v1.0 shipped. Conformance gate: 79/79.

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

### Phase 1 — Correctness Bugs ✅

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

### Phase 3 — Core API Gaps ✅

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

### Phase 8 — Constraint & Solver Gaps ✅

> **Complete (2026-03-04).** All conformance-relevant tasks shipped across 13 sessions
> (Spec A + Spec B + T1). DT-28 (ball/free tendon fix, commit `22c7c3d`), DT-33
> (tendon margin, commit `ac54666`), DT-25 condim=3 path verified. DT-130 (dense AR
> optimization) deferred to post-v1.0 — performance only, zero conformance impact.

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| ~~DT-19~~ | 10c | T3 | ~~QCQP-based cone projection for normal+friction force projection (MuJoCo PGS style)~~ — **Moved to Phase 13** (surfaced by Phase 12 gate triage) |
| ~~DT-128~~ | 10e | T2 | ~~PGS early termination~~ — **Moved to Phase 13** (surfaced by Phase 12 gate triage) |
| ~~DT-23~~ | 10c | T2 | ~~Per-DOF friction loss solver params (`dof_solref_fri`/`dof_solimp_fri`)~~ — **Moved to Phase 13** (surfaced by Phase 12 gate triage) |
| ~~DT-25~~ | 10c | T3 | ~~Deformable-rigid friction cone projection (currently normal-only)~~ **Partial** — Phase 8 Session 13 verification: condim=3 fully works (QCQP cone projection, R-scaling, Jacobian). Remaining gaps: condim=6 silently downgrades to 3 (DT-131), bodyweight diagApprox double-counts rigid body (DT-132), bodyweight uses rotational weight for flex friction rows (DT-133). 7 integration tests. |
| ~~DT-28~~ | 10d | T2 | ~~Ball/free joints in fixed tendons — validation + qvel DOF index mapping~~ **Done** — Phase 8 T1 (commit `22c7c3d`). qpos/dof address mapping fix for ball (nq=4/nv=3) and free (nq=7/nv=6) joints, 5 tests. |
| ~~DT-32~~ | 10d | T2 | ~~Per-tendon `solref_limit`/`solimp_limit` constraint solver params~~ — **Done** (Phase 8: naming conformance) |
| ~~DT-33~~ | 10d | T2 | ~~Tendon `margin` attribute for limit activation distance~~ **Done** — Phase 8 Spec A (commit `ac54666`). `tendon_margin` model field, all 6 hardcoded activation sites replaced, island/sleep wakeup margin-aware, 12 tests. |
| ~~DT-39~~ | 10e | T2 | ~~Body-weight diagonal approximation (`diagApprox`)~~ — **Moved to Phase 13** (partially fixed in Phase 12 Session 15; remaining paths surfaced by gate triage) |
| ~~DT-129~~ | 10e | T3 | ~~PGS warmstart two-phase projection~~ — **Moved to Phase 13** (surfaced by Phase 12 gate triage) |

---

### Phase 9 — Collision Completeness ✅

> **Complete (2026-03-07).** All conformance-relevant collision tasks shipped across
> 5 specs (A–E) + T1. DT-134 (mesh-primitive GJK/EPA dispatch) deferred to post-v1.0 —
> per-triangle BVH produces correct results, hull dispatch is a performance optimization.

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| ~~§43~~ | 11 | — | ~~Mesh inertia modes: exact, shell, convex, legacy~~ **Done** — Phase 9 Spec B |
| ~~§50~~ | 12 | — | ~~Continuous Collision Detection (conservative-advancement CCD, tunneling prevention)~~ **Done** — Phase 9 Spec D. GJK distance query + conservative advancement + MULTICCD multi-contact. |
| ~~§54~~ | 13 | — | ~~Missing heightfield collision pairs~~ **Done** — Phase 9 Spec C. Prism-based `mjc_ConvexHField` for all convex-vs-hfield pairs (hfield-mesh, cylinder, ellipsoid now exact + multi-contact). Hfield-plane/hfield-hfield dropped (not in MuJoCo). |
| ~~§57~~ | 14 | — | ~~`sdf_iterations`/`sdf_initpoints` from `<option>` (replace hardcoded values)~~ **Done** — Phase 9 T1 (Session 2) |
| ~~§65~~ | 16 | — | ~~Mesh convex hull auto-computation (Quickhull at build time for GJK/EPA)~~ **Done** — Phase 9 Spec A |
| ~~DT-70~~ | 10i | T3 | ~~Deformable-vs-mesh/hfield/SDF narrowphase (only primitives currently)~~ **Done** — Phase 9 Spec E. Flex-vs-mesh (convex hull GJK + per-triangle BVH fallback), flex-vs-hfield, flex-vs-SDF. |

---

### Phase 10 — Flex Pipeline ✅

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| ~~§42A-i~~ | 10 | — | ~~Sparse flex edge Jacobian (`flexedge_J`) — force projection through body Jacobians~~ **DONE** — Spec A (23 sessions). |
| ~~§42A-ii~~ | 10 | — | ~~`flex_rigid`/`flexedge_rigid` boolean arrays — skip rigid bodies/edges~~ **DONE** — T1 (S2). |
| ~~§42A-iii~~ | 10 | — | ~~`flexedge_length`/`flexedge_velocity` pre-computed Data fields~~ **DONE** — T1 (S2). |
| ~~§42A-iv / DT-142~~ | 10 | — | ~~Flex self-collision dispatch (BVH/SAP midphase + narrowphase)~~ **DONE** — Spec C (S14–S18). |
| ~~§42A-v / DT-143~~ | 10 | — | ~~Flex-flex cross-object collision filtering (contype/conaffinity)~~ **DONE** — Spec D (S19–S23). |
| ~~§42B~~ | 10 | — | ~~Flex bending: cotangent Laplacian (MuJoCo's actual bending formulation)~~ **DONE** — Spec B (S8–S13). |

---

### Phase 11 — Derivatives ✅

> **Complete (2026-03-09).** All 6 tasks shipped (Spec A + Spec B + sessions).
> Merged in PR #100 (commit `39b8945`).

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| ~~§58~~ | 14 | — | ~~`mjd_smooth_pos` analytical position derivatives~~ **DONE** — Spec A, analytical position force derivatives (`mjd_smooth_pos`) + hybrid integration, 18 tests |
| ~~DT-47~~ | 10f | T2 | ~~Sensor derivatives (C, D matrices) for `TransitionMatrices`~~ **DONE** — Spec B (Session 11, commit `7bf1043`). C/D matrices via FD, opt-in `compute_sensor_derivatives` flag, 11 tests. |
| ~~DT-51~~ | 10f | T2 | ~~`mjd_inverseFD` — inverse dynamics derivatives~~ **DONE** — Session 3, FD wrapper around `mj_inverse()` |
| ~~DT-52~~ | 10f | T2 | ~~`mjd_subQuat` — quaternion subtraction Jacobians~~ **DONE** — Session 2, analytical 3×3 Jacobians |
| ~~DT-53~~ | 10f | T2 | ~~`mj_forwardSkip` — skip-stage optimization for ~50% FD cost reduction~~ **DONE** — Session 3, `MjStage` enum + `forward_skip()` |
| ~~DT-54~~ | 10f | T2 | ~~Muscle actuator velocity derivatives — piecewise FLV curve gradients~~ **DONE** — Session 2, Hill-type + MuJoCo muscle velocity derivatives |

---

### Phase 12 — Conformance Test Suite ✅

> **Complete (2026-03-10).** Four-layer conformance test suite: 79/79 tests pass.
> 102 golden reference .npy files generated from MuJoCo 3.4.0. 26 golden flag tests
> generated (DT-97). Merged in PR #101 (commit `001a7fd`). Gate triage identified
> remaining conformance gaps → routed to Phase 13.

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| ~~§45~~ | 11 | — | ~~Four-layer conformance test suite: self-consistency, per-stage reference, trajectory comparison, property/invariant tests against MuJoCo 3.4.0~~ **Done** — 79/79 tests pass (Layer A: 13, Layer B: 43, Layer C: 8, Layer D: 15). |
| ~~DT-97~~ | 10j | T2 | ~~Golden file generation for per-flag trajectory conformance (AC18 of §41 — bootstrap pattern reused by §45)~~ **Done** — 26 .npy files (1 baseline + 19 disable + 6 enable flags), `gen_flag_golden.py` + `gen_conformance_reference.py`. |

---

### Phase 13 — Remaining Core ✅

> **Complete (2026-03-11).** All 7 tasks shipped across 4 specs (A–D).
> Merged in PR #102 (commit `bd75fdf`).

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| ~~DT-19~~ | 10c | T3 | ~~QCQP-based cone projection for normal+friction force projection (MuJoCo PGS style)~~ **Done** — Phase 13 Spec B (Session 6). Verified correct via line-by-line comparison with MuJoCo `mju_QCQP2/3/N()`. No code changes needed; 14/14 unit tests pass. |
| ~~DT-23~~ | 10c | T2 | ~~Per-DOF friction loss solver params (`dof_solref_fri`/`dof_solimp_fri`)~~ **Done** — verified correct in Phase 13 Spec A (Session 2/4). Assembly routes per-DOF `dof_solref`/`dof_solimp` and per-tendon `tendon_solref_fri`/`tendon_solimp_fri`. |
| ~~DT-39~~ | 10e | T2 | ~~Body-weight diagonal approximation (`diagApprox`) — remaining code paths~~ **Done** — Phase 13 Spec A (Session 2). Replaced diagonal-only `tendon_invweight0` with full `J·M⁻¹·J^T` solve. diagApprox/R/D match MuJoCo. Golden flags still blocked by Newton solver convergence (DT-128/129). |
| ~~DT-128~~ | 10e | T2 | ~~PGS early termination~~ **Done** — Phase 13 Spec B (Session 6). Refactored PGS loop: `while iter < max_iters` with `improvement -= costChange()` accumulation, `improvement *= scale`, `if improvement < tolerance { break }`. `solver_niter` reports actual count. `solver_stat` populated per iteration. MuJoCo conformance verified (T7: efc_force, solver_niter, qacc within 1e-10). |
| ~~DT-129~~ | 10e | T3 | ~~PGS warmstart two-phase projection~~ **Done** — Phase 13 Spec B (Session 6). Verified existing warmstart logic matches MuJoCo PGS path: `classify_constraint_states()` maps qacc_warmstart → efc_force, dual cost gate zeros forces when warmstart not beneficial. Minor primal/dual cost gate difference tracked as DT-163 (no measurable divergence). |
| ~~§46~~ | 12 | — | ~~`<composite>` procedural body generation~~ **Done** — Phase 13 Spec C (Session 10). Only `cable` type implemented (non-deprecated in MuJoCo 3.4.0). Deprecated types (particle/grid/rope/loop/cloth) return MuJoCo-matching error messages. Skin (DT-165), custom text (DT-166), `<flexcomp>` (DT-167), `<replicate>` (DT-168), `<site>` template (DT-169) deferred to post-v1.0. |
| ~~§66~~ | 16 | — | ~~Plugin/extension system — `<plugin>`/`<extension>` MJCF parsing + Rust trait dispatch~~ **Done** — Phase 13 Spec D (Session 15). Rust `Plugin` trait + `PluginRegistry`, 15 Model fields, Data plugin state, MJCF `<extension>`/`<plugin>` parsing, builder resolution, forward dispatch hooks (actuation/passive/sensor/advance), lifecycle (init/reset). SDF collision dispatch deferred (DT-170), resource providers/decoders deferred (DT-171), plugin copy/destroy callbacks deferred (DT-172), Data clone plugin_data preservation deferred (DT-173). |

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
| DT-138 | 11 | T1 | GPU-accelerated mesh inertia computation — move shell/exact/legacy inertia to compute shader for large meshes. Pure performance optimization, no conformance impact. Deferred from Phase 9 Spec B. |
| §67 | 17 | — | GPU forward kinematics (level-set parallel tree traversal) |
| §68 | 17 | — | GPU collision broad-phase (parallel SAP / spatial hashing) |
| §69 | 17 | — | GPU collision narrow-phase (GJK/EPA on compute shaders) |
| §70 | 17 | — | GPU constraint solver (Jacobi-style parallel PGS) |
| §71 | 17 | — | Full GPU step chaining (single command buffer submission) |

### Performance Optimizations
| Task | Source | Tier | Description |
|------|--------|------|-------------|
| DT-130 | 10e | T3 | Dense AR matrix optimization — PGS currently computes full nefc×nefc `efc_AR` matrix. MuJoCo uses sparse row-level operations (`ARblock`). Performance optimization, not conformance. Discovered during Phase 8 Spec B review. |
| DT-134 | 16 | T2 | Mesh-primitive dispatch to GJK/EPA on convex hulls — mesh-sphere, mesh-capsule, mesh-box pairs currently use per-triangle BVH; should route through `convex_mesh_from_hull()` + `gjk_epa_contact()` when hull available (AD-1 option a). Deferred from Phase 9 Spec A. |
| §40d | 10 | — | Sparse Jacobian for fluid derivatives (nv > 200) |
| §40e | 10 | — | Refactor `mj_jac_site` to use `mj_jac_point` kernel |
| §48 | 12 | — | SIMD batch audit for hot paths |
| DT-140 | 13 | T2 | Prism BVH acceleration for heightfield collision — `collide_hfield_multi` iterates sub-grid cells linearly; a quadtree or spatial index could skip empty cells for large hfields. Pure performance optimization, no conformance impact. Deferred from Phase 9 Spec C. |
| DT-141 | 12 | T2 | GJK/EPA cross-frame simplex warm-starting — cache previous frame's GJK simplex per geom pair and use as initial simplex for next frame's query. Within-frame vertex warm-start already implemented (Spec A). Pure performance optimization, no conformance impact. Deferred from Phase 9 Spec D. |
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
| DT-157 | 10f | T2 | Analytical sensor derivatives — per-sensor-type Jacobians (CortenForge extension, not MuJoCo conformance). Deferred from Phase 11 Spec B. |
| DT-158 | 10f | T2 | Inverse dynamics sensor derivatives (`DsDq`/`DsDv`/`DsDa` in `mjd_inverseFD`). Deferred from Phase 11 umbrella. |
| DT-159 | 10f | T1 | `step()` → `forward_skip + integrate` migration for FD loops — `skipsensor` optimization. Deferred from Phase 11 Spec B. |
| DT-76 | 10j | T1 | Pre-allocated `efc_lambda_saved` for RK4 |
| DT-91 | 10j | T1 | Warmstart `SmallVec` optimization |
| DT-92 | 10j | T1 | Parallel reset for `BatchSim` |
| DT-105 | 10e | T3 | Sparse `actuator_moment` compression (CSR) — numerically equivalent to current dense storage. Deferred from Phase 5 Spec B. |
| DT-146 | 10i | T2 | Sparse constraint assembly — scatter `flexedge_J` into sparse `efc_J` instead of dense DMatrix. Performance optimization, no conformance impact. Deferred from Phase 10 Spec A. |
| DT-147 | 10i | T1 | `flex_edgeequality` dedicated flag — per-flex boolean for skip_jacobian condition instead of conservative `solref != [0,0]` proxy. Minor optimization. Deferred from Phase 10 Spec A. |
| DT-148 | 10i | T1 | `flex_hingeadr`/`flex_hingenum` per-flex hinge index arrays — O(1) Bridson bending iteration instead of filter loop. Performance optimization, no conformance impact. Deferred from Phase 10 Spec B. |

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
| DT-116 | 10g | T3 | Per-actuator `GainType::User` / `BiasType::User` callback infrastructure — dependency on §66 satisfied (Phase 13 Spec D) |
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
| DT-139 | 11 | T1 | `exactmeshinertia` attribute full removal — match MuJoCo 3.5.0 schema rejection (currently parsed + warn for backward compat). No conformance impact since attribute has no behavioral effect. Deferred from Phase 9 Spec B. |
| DT-65 | 10h | T1 | User sensor `dim` attribute. Also requires `sensor_intprm` array (`mjmodel.h:1213`). |
| DT-69 | 10i | T2 | SAP for flex broadphase (currently brute-force) |
| DT-72 | 10i | T1 | Flex contacts wired for adhesion |
| DT-144 | 10i | T2 | Prism-based hfield collision for flex vertices — currently uses point-sampling `heightfield_sphere_contact()` which gives equivalent results for tiny flex radii (< 0.1). MuJoCo uses `mjc_ConvexHField()` (prism-based GJK) for all convex-vs-hfield pairs. Upgrade if divergence found for large-radius flex vertices. Deferred from Phase 9 Spec E. |
| DT-80 | 10j | T1 | Mocap body + equality weld integration testing |
| DT-81 | 10j | T1 | `key_userdata` support |
| DT-84 | 10j | T1 | `mju_encodePyramid` utility |
| DT-161 | 13 | T1 | Pyramidal `efc_diagApprox` bodyweight factor-of-2 — CF stores `(1+μ²)·w_tran`, MuJoCo stores `2·(1+μ²)·w_tran` for pyramidal facet rows (`assembly.rs:681`). No downstream effect: Rpy post-processing (`assembly.rs:775-795`) overwrites R/D for all facet rows. Pure conformance of a stored diagnostic field. Root cause unknown — requires reading MuJoCo `mj_diagApprox` C source. Identified during Phase 13 Spec A review. |
| DT-162 | 13 | T1 | PGS `solver_stat` `nactive`/`nchange` per-iteration counting — MuJoCo calls `dualState()` after each PGS sweep to classify active constraints and count state transitions. CortenForge uses placeholder 0 for both fields. Diagnostic only — does not affect solver forces, qacc, or convergence. Deferred from Phase 13 Spec B. |
| DT-163 | 13 | T1 | PGS warmstart primal cost gate — MuJoCo evaluates `primal_cost > 0` via `mj_constraintUpdate()`, CortenForge evaluates `dual_cost < 0` via `classify_constraint_states()`. Equivalent at optimum (strong duality); slightly more conservative before convergence (`dual ≤ primal`). No measurable divergence in T7 conformance test. Upgrade to primal cost if divergence found on complex models. Deferred from Phase 13 Spec B. |
| DT-164 | 13 | T2 | Newton solver golden flag convergence — 24/26 golden flag tests fail at ~0.002 qacc divergence. Assembly (diagApprox, R, D) and PGS solver are verified correct (Specs A+B). Residual divergence traces to Newton solver iteration: efc_force differs by ~3e-6 per row, amplified by M⁻¹ to ~0.002 qacc. Root cause is Newton convergence behavior (line search, Hessian factorization, or tolerance handling), not constraint assembly or PGS. Requires dedicated Newton solver investigation. Identified during Phase 13 Session 8 golden flag gate. |
| DT-165 | 13 | T2 | Cable skin generation — rendering-only box-geom cable skin with bicubic interpolation and subgrid support. MuJoCo `mjCComposite::MakeSkin()` in `user_composite.cc`. Visual-only feature — no physics impact. Deferred from Phase 13 Spec C. |
| DT-166 | 13 | T1 | Custom text metadata for composites — cable adds `composite_{prefix}` text element with data `rope_{prefix}` via MuJoCo's `mjsText`. Requires `<custom><text>` infrastructure which CortenForge does not yet support. Metadata-only — no physics impact. Deferred from Phase 13 Spec C. |
| DT-167 | 13 | T2 | `<flexcomp>` element — separate MJCF element from `<composite>`. Generates flex bodies from templates (box, cylinder, ellipsoid, mesh, etc.). Completely independent infrastructure from composite cable. Not in §46. Deferred from Phase 13 Spec C review. |
| DT-168 | 13 | T1 | `<replicate>` element — MuJoCo 3.4.0 replacement for deprecated `<composite type="particle">`. Repeats body templates at specified positions. Independent from cable composite. Deferred from Phase 13 Spec C review. |
| DT-169 | 13 | T1 | Cable `<site>` template customization — `<site>` child element support inside `<composite>` to customize boundary site properties (size, rgba, group, etc.). Currently boundary sites use `MjcfSite::default()`. Deferred from Phase 13 Spec C review. |
| DT-170 | 16 | T2 | SDF collision dispatch — Plugin trait defines `sdf_distance`, `sdf_gradient`, `sdf_staticdistance`, `sdf_aabb`, `sdf_attribute` callbacks, but collision pipeline does not call them. Requires `mjc_SDF()` equivalent integration into narrowphase. No known MuJoCo test models require SDF plugins for conformance. Deferred from Phase 13 Spec D (§66). |
| DT-171 | 16 | T1 | Resource providers and decoders — MuJoCo's `mjpResourceProvider` (file I/O) and `mjpDecoder` (format decoding) are separate from the 4 physics plugin types. No conformance impact — CortenForge uses Rust native I/O. Deferred from Phase 13 Spec D (§66). |
| DT-172 | 16 | T1 | Plugin copy/destroy callbacks — MuJoCo's `copy` and `destroy` plugin callbacks for lifecycle management when `mjData` is copied or freed. Currently `plugin_data` is reset to `None` on clone and not explicitly destroyed. Deferred from Phase 13 Spec D (§66). |
| DT-173 | 16 | T2 | Data clone plugin_data preservation — `Data::clone()` resets `plugin_data` to `None` (each `Option<Box<dyn Any>>` becomes `None`). Full-fidelity clone requires a `Plugin::copy()` callback to deep-copy type-erased plugin state. Depends on DT-172. Deferred from Phase 13 Spec D (§66). |
| DT-174 | 16 | T1 | Sensor cutoff stage check hardening — `compute_plugin_sensors()` omits `sensor_needstage[j] == stage` check from spec. Functionally correct (plugin needstage == sensor needstage by construction), but adding the check would match the spec exactly and be more robust against future changes. Deferred from Phase 13 Spec D review (Session 17). |
| DT-89 | 10i | T1 | `<flexcomp>` rendering attributes |
| DT-104 | 10b | T2 | Ball/free joint transmission — `nv == 3` and `nv == 6` sub-paths in `mj_transmission()`. Deferred from Phase 5 Spec B. |
| DT-107 | 10g | T2 | Runtime interpolation logic — `mj_forward` reads history buffer for delayed ctrl, `mj_step` writes circular buffer. Covers both actuators (Phase 5 Spec D) and sensors (Phase 6 Spec D): structures exist for both, runtime missing for both. Includes sensor history pre-population in `reset_data()`. |
| DT-108 | 10g | T1 | `dyntype` enum gating interpolation eligibility — restrict which `ActuatorDynamics` variants may use history buffer. Deferred from Phase 5 Spec D. |
| DT-110 | 10g | T1 | `actuator_plugin` model array — per-actuator plugin ID (`int[nu]`, -1 sentinel). Dependency on §66 satisfied (Phase 13 Spec D). Deferred from Phase 5 Spec D. |
| DT-118 | 15 | T2 | `mj_contactForce()` — touch sensor force reconstruction via full contact force vector. Deferred from Phase 6 Spec A. |
| DT-119 | 15 | T2 | Ray-geom intersection filter for touch sensor — filter contacts by surface normal alignment. Depends on DT-118. Deferred from Phase 6 Spec A. |
| DT-120 | 15 | T1 | `MjObjectType::Camera` — frame sensor camera support (reftype="camera" currently warns + ignores). Deferred from Phase 6 Spec B. |
| DT-121 | 15 | T1 | `InsideSite` sensor (`mjSENS_INSIDESITE`) — MuJoCo 3.x sensor type not yet supported. Deferred from Phase 6 Spec C. |
| DT-122 | 15 | T2 | Mesh/Hfield/SDF geom distance support for `GeomDist`/`GeomPoint`/`GeomNormal` sensors. Deferred from Phase 6 Spec C. |
| DT-123 | 10b | T1 | `IntVelocity` enum variant — concrete `<intvelocity>` elements not yet supported (defaults parsing works). Deferred from Phase 7 Spec A. |
| DT-124 | 10b | T1 | Muscle sentinel detection for `<general dyntype="muscle">` path (`gainprm[0]==1` quirk). Known conformance divergence. Deferred from Phase 7 Spec A. |
| DT-125 | 15 | T2 | `mj_setConst()` runtime `qpos_spring` recomputation — when `mj_setConst()` is called at runtime, `qpos_spring` must be recomputed from current `qpos0`/`springref` via `setSpring()` logic (`engine_setconst.c`). Currently `qpos_spring` is set at build time and static. Deferred from Phase 7 Spec B. |
| DT-150 | 10i | T2 | `activelayers` runtime filtering for flex self-collision — parsed and stored (Phase 7 T1) but not consumed at runtime. MuJoCo uses `activelayers` to filter which element layers participate in self-collision. Minimal conformance impact — affects only models using layer-based filtering. Deferred from Phase 10 Spec C. |
| DT-151 | 10i | T2 | Edge-edge tests for dim=3 tetrahedral self-collision — MuJoCo performs edge-edge proximity tests between tet edges in addition to vertex-face tests. CortenForge implements vertex-face only. Minor conformance gap for dim=3 self-collision. Deferred from Phase 10 Spec C. |
| DT-152 | 10i | T2 | Barycentric force distribution on face side for flex self-collision — current Jacobian applies force to nearest vertex rather than distributing across face vertices via barycentric weights. Force direction correct; only distribution approximate. Deferred from Phase 10 Spec C. |
| DT-153 | 10i | T1 | Island assignment for flex contacts — `island/mod.rs:297-306` and `453-467` skip flex contacts (sentinel geom indices) from island assignment and constraint-to-tree lookup. Affects island-based constraint solving only (not default mode). Deferred from Phase 10 Spec D. |
| DT-154 | 10i | T1 | Flex contact factory condim=6 mapping — all flex factories map `condim: 1→1, 4→4, _→3`. `condim=6` produces `dim=3` not `dim=6`. Pre-existing across all flex contact types. Deferred from Phase 10 Spec D. |
| DT-155 | 10i | T1 | S10 override test for flex-flex contacts (AC11/T10) — `ENABLE_OVERRIDE` test infrastructure not wired for flex contact tests. Override logic works (verified in rigid specs); only the test harness is missing. Deferred from Phase 10 Spec D review. |
| DT-156 | 10i | T2 | Narrowphase triangle-triangle contact count conformance gap — CortenForge produces 36 contacts vs MuJoCo 32 for overlapping 3×3 flex grids. Root cause in triangle-triangle intersection logic (Spec C territory). Behavior correct; only count differs. Identified during Phase 10 Spec D review. |
| DT-160 | 10j | T1 | Additional FK reference fields — `xmat` (nbody×9), `geom_xpos` (ngeom×3), `geom_xmat` (ngeom×9), `site_xpos` (nsite×3). Extend `gen_conformance_reference.py` to export these fields, then add Layer B tests. Strengthens FK conformance coverage beyond primary body pose outputs. Deferred from Phase 12 Spec A Out of Scope. |

### Code Quality
| Task | Source | Tier | Description |
|------|--------|------|-------------|
| DT-117 | xtask check | T2 | Eliminate `unwrap()`/`expect()` from library code (~1,085 call sites). Convert to `?` propagation, `Result` returns, or `unsafe { get_unchecked() }` with safety invariant comments where bounds are structurally guaranteed. Prevents unrecoverable panics for library consumers. |

### Other Non-Critical
| Task | Source | Tier | Description |
|------|--------|------|-------------|
| §42 | 10 | — | `<flex>`/`<flexcomp>` parsing (subsumed by §6B) |
| ~~§44~~ | 11 | — | ~~Deprecate legacy standalone crates~~ **Done** — crates deleted (not deprecated) per `LEGACY_CRATE_CLEANUP.md` |
| §47 | 12 | — | URDF loader completeness (mesh collision, mimic joints, etc.) |
| §49 | 12 | — | Non-physics MJCF elements (`<visual>`, `<statistic>`, `<custom>`, `<size>`) |
| DT-71 | 10i | T1 | Behavioral friction tests for deformables |
| DT-137 | 11 | T1 | Deeply concave mesh test (C/U-shape) for legacy vs exact inertia differentiation — current L-shape test has centroid inside solid so legacy==exact. Nice-to-have test enhancement. Deferred from Phase 9 Spec B. |

---

## Decision Log

| Date | Decision |
|------|----------|
| 2026-02-21 | Triaged all ~135 remaining tasks into core v1.0 (~67) vs extra (~68). Core defined as "matches MuJoCo C library behavior, passes conformance." |
| 2026-02-21 | Split `future_work_10b.md` into 9 files (10b–10j), one per thematic group. Updated all ~75 back-references across future_work_1–9, index.md, and ROADMAP. |
| 2026-02-21 | Classified all 92 DT items into spec tiers: T1 (32 plan+implement), T2 (39 grouped into ~15 specs), T3 (21 individual specs). Added Tier column to all 10b–10j tables and ROADMAP. |
| 2026-03-11 | v1.0 complete. All 13 phases shipped. Conformance gate 79/79. DT-130 (dense AR) and DT-134 (mesh-primitive GJK dispatch) moved to post-v1.0 Performance Optimizations — both zero conformance impact. Roadmap status updated from Draft to Complete. |
