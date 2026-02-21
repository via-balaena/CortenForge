# CortenForge Sim — v1.0 Roadmap

> **Status**: Draft — 2026-02-21
> **Scope**: All remaining work from `future_work_10.md` (§41+) through `future_work_17.md`,
> plus the ~92 deferred tasks in `future_work_10b.md` (DT-1 through DT-92).
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
| future_work_10b (DT-1–92) | ~40 | ~52 |
| future_work_11 (§43–45) | 2 | 1 |
| future_work_12 (§46–50) | 3 | 2 |
| future_work_13 (§51–55) | 5 | 0 |
| future_work_14 (§56–59) | 4 | 0 |
| future_work_15 (§60–64a) | 6 | 0 |
| future_work_16 (§65–66) | 2 | 0 |
| future_work_17 (§67–71) | 0 | 5 |
| **Total** | **~67** | **~68** |

---

## v1.0 Critical Path

Ordered by priority. Each phase should be completed and committed before
moving to the next. Within a phase, tasks are independent and can be
parallelized.

---

### Phase 1 — Correctness Bugs

Fix known physics bugs before building more on top of them. These are small,
high-value, and reduce the chance of compounding errors in later work.

| Task | Source | Description |
|------|--------|-------------|
| DT-74 | 10b | `compute_body_jacobian_at_point()` incomplete — only x-component implemented |
| DT-75 | 10b | `add_body_jacobian` free joint bug — world-frame vectors instead of body-frame `R*e_i` |
| DT-35 | 10b | Tendon spring/damper forces produce zero in implicit mode — missing non-diagonal K coupling |
| DT-16 | 10b | Flex `density` attribute location wrong in parser vs MuJoCo spec |
| DT-90 | 10b | `flex_friction` scalar should be `Vector3<f64>` — torsional/rolling friction data lost |
| DT-78 | 10b | `actuator_lengthrange` DOF lookup wrong for unlimited spatial tendons |

---

### Phase 2 — Runtime Flag Wiring

| Task | Source | Description |
|------|--------|-------------|
| §41 | 10 | Wire all 19 `disableflags` and 6 `enableflags` from MJCF `<flag>` through to runtime pipeline gating. Conformance-critical — controls whether gravity, contacts, limits, equality, actuation, sensors, etc. are active. |
| DT-61 | 10b | Define `DISABLE_GRAVITY` flag (currently only `gravity.norm() == 0.0` check) |

---

### Phase 3 — Core API Gaps

Public API functions that MuJoCo exposes and users/conformance tests expect.

| Task | Source | Description |
|------|--------|-------------|
| DT-21 | 10b | `xfrc_applied` support in `qfrc_smooth` — external Cartesian body forces |
| DT-41 | 10b | Newton solver for implicit integrators (currently warns + falls back to PGS) |
| §52 | 13 | `mj_inverse()` — inverse dynamics API computing `qfrc_inverse` |
| §53 | 13 | `step1()`/`step2()` split stepping API for control injection between forward and integrate |
| §59 | 14 | `mj_name2id`/`mj_id2name` — bidirectional name-index lookup |
| DT-79 | 10b | User callbacks `mjcb_*` Rust equivalents |

---

### Phase 4 — Core Data Fields

Persistent fields in `Data` that MuJoCo computes every forward step.

| Task | Source | Description |
|------|--------|-------------|
| §51 | 13 | `cacc`, `cfrc_int`, `cfrc_ext` — per-body 6D force/acceleration accumulators |
| §56 | 14 | `subtree_linvel`, `subtree_angmom` — promote from sensor helpers to persistent fields |

---

### Phase 5 — Actuator Completeness

| Task | Source | Description |
|------|--------|-------------|
| DT-56 | 10b | `dampratio` for position actuators (requires `acc0`) |
| DT-57 | 10b | `acc0` computation for non-muscle actuators |
| DT-58 | 10b | sim-muscle Hill-type model as `ActuatorDynamics::HillMuscle` variant |
| DT-59 | 10b | Bisection-based `lengthrange` for unlimited slide joints (`mj_setLengthRange`) |
| DT-77 | 10b | Length-range auto-estimation for site-transmission muscle actuators |
| DT-6 | 10b | `actearly` attribute wired to runtime (currently parsed, no effect) |
| DT-8 | 10b | Transmission types: `cranksite`, `slidersite`, `jointinparent` |
| DT-9 | 10b | `nsample`, `interp`, `delay` — MuJoCo 3.x interpolation actuator attributes |
| DT-60 | 10b | `jnt_actgravcomp` routing to `qfrc_actuator` instead of `qfrc_passive` |
| §61 | 15 | `slidercrank` actuator transmission |
| §63 | 15 | `dynprm` array 3→10 elements to match MuJoCo |

---

### Phase 6 — Sensor Completeness

| Task | Source | Description |
|------|--------|-------------|
| §62 | 15 | Missing sensor types: `clock`, `jointactuatorfrc`, `camprojection`, `geomdist`, `geompoint`, `geomnormal` |
| DT-62 | 10b | Frame sensor `objtype` attribute not parsed — fallback heuristic used |
| DT-63 | 10b | Frame sensor `reftype`/`refid` — relative-frame measurements |
| DT-64 | 10b | Multi-geom touch sensor aggregation (currently only first geom matched) |

---

### Phase 7 — MJCF Parsing & Defaults Gaps

| Task | Source | Description |
|------|--------|-------------|
| DT-2 | 10b | Equality constraint defaults: `solref`/`solimp` in defaults structs |
| DT-3 | 10b | File-based hfield loading from PNG |
| DT-11 | 10b | `range` as defaultable attribute in `MjcfJointDefaults` |
| DT-13 | 10b | `qpos_spring` — distinct from `qpos0`, not yet implemented |
| DT-14 | 10b | Actuator type-specific defaults (cylinder area/timeconst, muscle params) |
| DT-85 | 10b | Flex `<contact>` runtime attributes: `internal`, `activelayers`, `vertcollide`, `passive` |
| DT-88 | 10b | `<flexcomp>` attributes: `inertiabox`, `scale`, `quat`, `file` |
| §55 | 13 | Per-element `*_user` custom data arrays from MJCF |
| §60 | 15 | `springinertia` joint attribute — inertia-spring coupling in CRBA diagonal |
| §64 | 15 | Ball/free joint spring potential energy (quaternion geodesic) |
| §64a | 15 | `jnt_margin` for joint limit activation and constraint row margin |

---

### Phase 8 — Constraint & Solver Gaps

| Task | Source | Description |
|------|--------|-------------|
| DT-19 | 10b | QCQP-based cone projection for normal+friction force projection (MuJoCo PGS style) |
| DT-23 | 10b | Per-DOF friction loss solver params (`dof_solref_fri`/`dof_solimp_fri`) |
| DT-25 | 10b | Deformable-rigid friction cone projection (currently normal-only) |
| DT-28 | 10b | Ball/free joints in fixed tendons — validation + qvel DOF index mapping |
| DT-32 | 10b | Per-tendon `solref_limit`/`solimp_limit` constraint solver params |
| DT-33 | 10b | Tendon `margin` attribute for limit activation distance |
| DT-39 | 10b | Body-weight diagonal approximation (`diagApprox`) |

---

### Phase 9 — Collision Completeness

| Task | Source | Description |
|------|--------|-------------|
| §43 | 11 | Mesh inertia modes: exact, shell, convex, legacy |
| §50 | 12 | Continuous Collision Detection (conservative-advancement CCD, tunneling prevention) |
| §54 | 13 | Missing heightfield collision pairs: hfield-mesh, hfield-plane, hfield-hfield |
| §57 | 14 | `sdf_iterations`/`sdf_initpoints` from `<option>` (replace hardcoded values) |
| §65 | 16 | Mesh convex hull auto-computation (Quickhull at build time for GJK/EPA) |
| DT-70 | 10b | Deformable-vs-mesh/hfield/SDF narrowphase (only primitives currently) |

---

### Phase 10 — Flex Pipeline

| Task | Source | Description |
|------|--------|-------------|
| §42A-i | 10 | Sparse flex edge Jacobian (`flexedge_J`) — force projection through body Jacobians |
| §42A-ii | 10 | `flex_rigid`/`flexedge_rigid` boolean arrays — skip rigid bodies/edges |
| §42A-iii | 10 | `flexedge_length`/`flexedge_velocity` pre-computed Data fields |
| §42A-iv | 10 | Flex self-collision dispatch (BVH/SAP midphase + narrowphase) |
| §42A-v | 10 | Flex-flex cross-object collision filtering (contype/conaffinity) |
| §42B | 10 | Flex bending: cotangent Laplacian (MuJoCo's actual bending formulation) |

---

### Phase 11 — Derivatives

| Task | Source | Description |
|------|--------|-------------|
| §58 | 14 | `mjd_smooth_pos` analytical position derivatives |
| DT-47 | 10b | Sensor derivatives (C, D matrices) for `TransitionMatrices` |
| DT-51 | 10b | `mjd_inverseFD` — inverse dynamics derivatives |
| DT-52 | 10b | `mjd_subQuat` — quaternion subtraction Jacobians |
| DT-53 | 10b | `mj_forwardSkip` — skip-stage optimization for ~50% FD cost reduction |
| DT-54 | 10b | Muscle actuator velocity derivatives — piecewise FLV curve gradients |

---

### Phase 12 — Conformance Test Suite

| Task | Source | Description |
|------|--------|-------------|
| §45 | 11 | Four-layer conformance test suite: self-consistency, per-stage reference, trajectory comparison, property/invariant tests against MuJoCo 3.4.0 |

This is the gate. Run it, identify failures, iterate on phases 1–11 until
green.

---

### Phase 13 — Remaining Core

| Task | Source | Description |
|------|--------|-------------|
| §46 | 12 | `<composite>` procedural body generation (grid, rope, cable, cloth, box, cylinder, ellipsoid) |
| §66 | 16 | Plugin/extension system — `<plugin>`/`<extension>` MJCF parsing + Rust trait dispatch |

---

## Post-v1.0 — Extra Features & Extensions

Everything below is explicitly **out of scope for v1.0**. These are tracked
and valued, but shipping them before conformance is wasted effort if the
foundation isn't right.

### Trait Architecture & Solver Extensibility
| Task | Source | Description |
|------|--------|-------------|
| §42C | 10 | `FlexElasticityModel` trait — NeoHookean hyperelastic model |
| §42D | 10 | `ActuatorGainModel` trait — Series Elastic Actuator model |
| §42E | 10 | Contact Solver trait — XPBD, impulse-based formulations |
| §42F | 10 | `SimBuilder` generic composition API |

### GPU Pipeline (future_work_17)
| Task | Source | Description |
|------|--------|-------------|
| §67 | 17 | GPU forward kinematics (level-set parallel tree traversal) |
| §68 | 17 | GPU collision broad-phase (parallel SAP / spatial hashing) |
| §69 | 17 | GPU collision narrow-phase (GJK/EPA on compute shaders) |
| §70 | 17 | GPU constraint solver (Jacobi-style parallel PGS) |
| §71 | 17 | Full GPU step chaining (single command buffer submission) |

### Performance Optimizations
| Task | Source | Description |
|------|--------|-------------|
| §40d | 10 | Sparse Jacobian for fluid derivatives (nv > 200) |
| §40e | 10 | Refactor `mj_jac_site` to use `mj_jac_point` kernel |
| §48 | 12 | SIMD batch audit for hot paths |
| DT-18 | 10b | Zero-friction condim downgrade optimization |
| DT-20 | 10b | Unify `J^T * lambda` vs chain-walk contact force application |
| DT-24 | 10b | Incremental collision detection on tree wake |
| DT-29 | 10b | Spatial tendon dense J^T multiplication path |
| DT-34 | 10b | Sparse Jacobian for spatial tendons |
| DT-36 | 10b | Flat CSR for `qLD_L` (replace `Vec<Vec<…>>`) |
| DT-37 | 10b | Sparse factorization for implicit integrator |
| DT-38 | 10b | Implicit matrix-vector products for CG |
| DT-40 | 10b | LDL^T factorization for Hessian robustness |
| DT-42 | 10b | Per-island Newton solver dispatch |
| DT-43 | 10b | Selective CRBA backward-pass per island |
| DT-44 | 10b | Sparse mass matrix for island extraction |
| DT-48 | 10b | Sparse derivative storage (nv > 100) |
| DT-49 | 10b | Parallel FD computation |
| DT-55 | 10b | `skipfactor` / factorization reuse |
| DT-76 | 10b | Pre-allocated `efc_lambda_saved` for RK4 |
| DT-91 | 10b | Warmstart `SmallVec` optimization |
| DT-92 | 10b | Parallel reset for `BatchSim` |

### Advanced Differentiation
| Task | Source | Description |
|------|--------|-------------|
| DT-45 | 10b | Full position-analytical derivatives (`dFK/dq`, `dM/dq`) |
| DT-46 | 10b | Contact-analytical derivatives via implicit function theorem |
| DT-50 | 10b | Automatic differentiation (dual numbers / enzyme) |

### Non-MuJoCo Extensions
| Task | Source | Description |
|------|--------|-------------|
| DT-4 | 10b | `<sdf>` inline distance grid asset element |
| DT-26 | 10b | Contact re-detect + re-solve iteration after XPBD |
| DT-27 | 10b | XPBD cross-iteration lambda accumulation fix |
| DT-30 | 10b | Compound pulley physics (capstan friction, pulley inertia) |
| DT-66 | 10b | `<equality><flex>` constraints — flex-flex coupling |
| DT-67 | 10b | GPU flex pipeline |
| DT-68 | 10b | Per-vertex material variation for flex |
| DT-73 | 10b | Volume constraints for flex bodies |
| DT-82 | 10b | SoA layout across environments |
| DT-83 | 10b | Multi-model batching |
| DT-86 | 10b | `elastic2d` keyword on `<flex><elasticity>` |
| DT-87 | 10b | Shared-body flex vertices |

### Low-Priority MuJoCo Compat
| Task | Source | Description |
|------|--------|-------------|
| DT-1 | 10b | Mesh defaults `apply_to_mesh()` |
| DT-5 | 10b | `gaintype/biastype/dyntype="user"` callback-based types |
| DT-7 | 10b | `actdim` explicit override |
| DT-10 | 10b | Deferred `<compiler>` attributes (`fitaabb`, `usethread`, etc.) |
| DT-12 | 10b | Programmatic enforcement that `worldbody.childclass` = None |
| DT-15 | 10b | Sentinel-value detection → `Option<T>` migration |
| DT-17 | 10b | Global `<option o_margin>` override |
| DT-22 | 10b | `efc_impP` impedance derivative field |
| DT-31 | 10b | `WrapType::Joint` inside spatial tendons |
| DT-65 | 10b | User sensor `dim` attribute |
| DT-69 | 10b | SAP for flex broadphase (currently brute-force) |
| DT-72 | 10b | Flex contacts wired for adhesion |
| DT-80 | 10b | Mocap body + equality weld integration testing |
| DT-81 | 10b | `key_userdata` support |
| DT-84 | 10b | `mju_encodePyramid` utility |
| DT-89 | 10b | `<flexcomp>` rendering attributes |

### Other Non-Critical
| Task | Source | Description |
|------|--------|-------------|
| §42 | 10 | `<flex>`/`<flexcomp>` parsing (subsumed by §6B) |
| §44 | 11 | Deprecate legacy standalone crates |
| §47 | 12 | URDF loader completeness (mesh collision, mimic joints, etc.) |
| §49 | 12 | Non-physics MJCF elements (`<visual>`, `<statistic>`, `<custom>`, `<size>`) |
| DT-71 | 10b | Behavioral friction tests for deformables |

---

## Decision Log

| Date | Decision |
|------|----------|
| 2026-02-21 | Triaged all ~135 remaining tasks into core v1.0 (~67) vs extra (~68). Core defined as "matches MuJoCo C library behavior, passes conformance." |
