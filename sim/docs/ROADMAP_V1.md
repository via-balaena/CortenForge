# CortenForge Sim — v1.0 Roadmap

> **Status**: Draft — 2026-02-21
> **Scope**: All remaining work from `future_work_10.md` (§41+) through `future_work_17.md`,
> plus the ~101 deferred tasks in `future_work_10b.md`–`10j` (DT-1 through DT-101).
> DT-93/94/95 were added during §41 spec and subsumed into §41.
> DT-96 (lazy energy eval) and DT-97 (golden file conformance) added during §41 audit.
> DT-99 (BVH midphase, §41 S9-full), DT-100 (global override, §41 S10-full),
> DT-101 (`mj_contactPassive()`) added during §41 spec expansion.
> ~~DT-98~~ retired — `passive` dropped entirely pre-v1.0 (no shim needed).
>
> **Current position**: Through §40c on `future_work_10.md`.

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

### Phase 2 — Runtime Flag Wiring

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| §41 | 10 | — | Wire all 19 `disableflags` and 6 `enableflags` end-to-end. Subsumes DT-61 (DISABLE_GRAVITY), DT-93 (auto-reset on NaN/divergence), DT-94 (BVH midphase integration), DT-95 (global contact parameter override). See [spec](todo/spec_fleshouts/S41_RUNTIME_FLAGS_SPEC.md). |
| DT-99 | 10c | T2 | BVH midphase integration into collision pipeline (§41 S9-full, post-§41 commit) |
| DT-100 | 10c | T2 | Global contact parameter override guard sites (§41 S10-full, post-§41 commit) |

---

### Phase 3 — Core API Gaps

Public API functions that MuJoCo exposes and users/conformance tests expect.

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| DT-21 | 10c | T3 | `xfrc_applied` support in `qfrc_smooth` — external Cartesian body forces |
| DT-41 | 10e | T3 | Newton solver for implicit integrators (currently warns + falls back to PGS) |
| §52 | 13 | — | `mj_inverse()` — inverse dynamics API computing `qfrc_inverse` |
| §53 | 13 | — | `step1()`/`step2()` split stepping API for control injection between forward and integrate |
| §59 | 14 | — | `mj_name2id`/`mj_id2name` — bidirectional name-index lookup |
| DT-79 | 10j | T3 | User callbacks `mjcb_*` Rust equivalents |

---

### Phase 4 — Core Data Fields

Persistent fields in `Data` that MuJoCo computes every forward step.

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| §51 | 13 | — | `cacc`, `cfrc_int`, `cfrc_ext` — per-body 6D force/acceleration accumulators |
| §56 | 14 | — | `subtree_linvel`, `subtree_angmom` — promote from sensor helpers to persistent fields |

---

### Phase 5 — Actuator Completeness

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| DT-56 | 10g | T2 | `dampratio` for position actuators (requires `acc0`) |
| DT-57 | 10g | T2 | `acc0` computation for non-muscle actuators |
| DT-58 | 10g | T3 | sim-muscle Hill-type model as `ActuatorDynamics::HillMuscle` variant |
| DT-59 | 10g | T2 | Bisection-based `lengthrange` for unlimited slide joints (`mj_setLengthRange`) |
| DT-77 | 10j | T2 | Length-range auto-estimation for site-transmission muscle actuators |
| DT-6 | 10b | T1 | `actearly` attribute wired to runtime (currently parsed, no effect) |
| DT-8 | 10b | T2 | Transmission types: `cranksite`, `slidersite`, `jointinparent` |
| DT-9 | 10b | T2 | `nsample`, `interp`, `delay` — MuJoCo 3.x interpolation actuator attributes |
| DT-60 | 10g | T1 | `jnt_actgravcomp` routing to `qfrc_actuator` instead of `qfrc_passive` |
| §61 | 15 | — | `slidercrank` actuator transmission |
| §63 | 15 | — | `dynprm` array 3→10 elements to match MuJoCo |

---

### Phase 6 — Sensor Completeness

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| §62 | 15 | — | Missing sensor types: `clock`, `jointactuatorfrc`, `camprojection`, `geomdist`, `geompoint`, `geomnormal` |
| DT-62 | 10h | T2 | Frame sensor `objtype` attribute not parsed — fallback heuristic used |
| DT-63 | 10h | T2 | Frame sensor `reftype`/`refid` — relative-frame measurements |
| DT-64 | 10h | T2 | Multi-geom touch sensor aggregation (currently only first geom matched) |

---

### Phase 7 — MJCF Parsing & Defaults Gaps

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| DT-2 | 10b | T2 | Equality constraint defaults: `solref`/`solimp` in defaults structs |
| DT-3 | 10b | T1 | File-based hfield loading from PNG |
| DT-11 | 10b | T2 | `range` as defaultable attribute in `MjcfJointDefaults` |
| DT-13 | 10b | T2 | `qpos_spring` — distinct from `qpos0`, not yet implemented |
| DT-14 | 10b | T2 | Actuator type-specific defaults (cylinder area/timeconst, muscle params) |
| DT-85 | 10i | T1 | Flex `<contact>` runtime attributes: `internal`, `activelayers`, `vertcollide`, `passive` |
| DT-88 | 10i | T2 | `<flexcomp>` attributes: `inertiabox`, `scale`, `quat`, `file` |
| §55 | 13 | — | Per-element `*_user` custom data arrays from MJCF |
| §60 | 15 | — | `springinertia` joint attribute — inertia-spring coupling in CRBA diagonal |
| §64 | 15 | — | Ball/free joint spring potential energy (quaternion geodesic) |
| §64a | 15 | — | `jnt_margin` for joint limit activation and constraint row margin |

---

### Phase 8 — Constraint & Solver Gaps

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| DT-19 | 10c | T3 | QCQP-based cone projection for normal+friction force projection (MuJoCo PGS style) |
| DT-23 | 10c | T2 | Per-DOF friction loss solver params (`dof_solref_fri`/`dof_solimp_fri`) |
| DT-25 | 10c | T3 | Deformable-rigid friction cone projection (currently normal-only) |
| DT-28 | 10d | T2 | Ball/free joints in fixed tendons — validation + qvel DOF index mapping |
| DT-32 | 10d | T2 | Per-tendon `solref_limit`/`solimp_limit` constraint solver params |
| DT-33 | 10d | T2 | Tendon `margin` attribute for limit activation distance |
| DT-39 | 10e | T2 | Body-weight diagonal approximation (`diagApprox`) |

---

### Phase 9 — Collision Completeness

| Task | Source | Tier | Description |
|------|--------|------|-------------|
| §43 | 11 | — | Mesh inertia modes: exact, shell, convex, legacy |
| §50 | 12 | — | Continuous Collision Detection (conservative-advancement CCD, tunneling prevention) |
| §54 | 13 | — | Missing heightfield collision pairs: hfield-mesh, hfield-plane, hfield-hfield |
| §57 | 14 | — | `sdf_iterations`/`sdf_initpoints` from `<option>` (replace hardcoded values) |
| §65 | 16 | — | Mesh convex hull auto-computation (Quickhull at build time for GJK/EPA) |
| DT-70 | 10i | T3 | Deformable-vs-mesh/hfield/SDF narrowphase (only primitives currently) |

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
| DT-15 | 10b | T1 | Sentinel-value detection → `Option<T>` migration |
| DT-17 | 10b | T1 | Global `<option o_margin>` override |
| DT-22 | 10c | T1 | `efc_impP` impedance derivative field |
| DT-31 | 10d | T2 | `WrapType::Joint` inside spatial tendons |
| DT-65 | 10h | T1 | User sensor `dim` attribute |
| DT-69 | 10i | T2 | SAP for flex broadphase (currently brute-force) |
| DT-72 | 10i | T1 | Flex contacts wired for adhesion |
| DT-80 | 10j | T1 | Mocap body + equality weld integration testing |
| DT-81 | 10j | T1 | `key_userdata` support |
| DT-84 | 10j | T1 | `mju_encodePyramid` utility |
| DT-89 | 10i | T1 | `<flexcomp>` rendering attributes |

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
