# Future Work 9b — Deferred Item Tracker (Items from §1–§40)

Consolidated tracker for deferred sub-items identified during a systematic
audit of [future_work_1.md](./future_work_1.md) through
[future_work_9.md](./future_work_9.md). These are items that were explicitly
noted as "deferred", "out of scope", "follow-up", or "future work" within
completed tasks §1–§40 but were **not** assigned to any specific §41+ task.

Each item is numbered §DT-*XX* ("Deferred Tracker item XX") and organized
into thematic groups. Original files have been updated with back-references
to the corresponding §DT-*XX* entry.

**Items already tracked elsewhere are excluded.** If an item was later
addressed by a numbered task (e.g., pyramidal cones → §32, geom priority →
§25, flex self-collision → §42A-iv), it does not appear here.

---

## Group 1 — Defaults & MJCF Parsing Gaps (17 items)

| §DT | Origin | Description | Priority |
|-----|--------|-------------|----------|
| DT-1 | §1 | Mesh defaults — no `apply_to_mesh()` method; root-only mesh scale defaults deferred | Low |
| DT-2 | §1 | Equality constraint defaults — no `apply_to_equality()`, `solref`/`solimp` not in defaults structs | Medium |
| DT-3 | §6a | File-based hfield loading from PNG (`<hfield file="terrain.png"/>`) | Low |
| DT-4 | §6b | `<sdf>` asset element for inline distance grids (no standard MuJoCo equivalent) | Low |
| DT-5 | §8 | `gaintype/biastype/dyntype="user"` — callback-based types require plugin system | Low |
| DT-6 | §8 | `actearly` parsed + defaultable but not wired to runtime (always standard order) | Medium |
| DT-7 | §8 | `actdim` explicit override not supported (auto-detection only) | Low |
| DT-8 | §8 | Transmission types: `cranksite`, `slidersite`, `jointinparent` not supported | Low |
| DT-9 | §8 | `nsample`, `interp`, `delay` — MuJoCo 3.x interpolation actuator attributes | Low |
| DT-10 | §18 | Deferred `<compiler>` attributes: `fitaabb`, `usethread`, `alignfree`, `saveinertial`, `inertiagrouprange`, `<lengthrange>` child | Low |
| DT-11 | §20 | `range` not in `MjcfJointDefaults` as a defaultable attribute | Medium |
| DT-12 | §20 | Programmatic API enforcement that `worldbody.childclass` must be `None` | Low |
| DT-13 | §22 | `qpos_spring` not implemented — uses `qpos0` instead (equivalent only in default case) | Medium |
| DT-14 | §27 defaults | Actuator type-specific defaults not yet defaultable (cylinder area/timeconst, muscle params) | Medium |
| DT-15 | §27 defaults | Sentinel-value detection for `gear`/`kp`/`noise`/`cutoff` should migrate to `Option<T>` | Low |
| DT-16 | §27B | Flex `density` attribute location wrong — on `<flex>` in parser but not on `<flex>` in MuJoCo | Medium |
| DT-17 | §27 | Global `<option o_margin>` override deferred — per-geom margin is correct foundation | Low |

---

## Group 2 — Contact & Collision System (10 items)

| §DT | Origin | Description | Priority |
|-----|--------|-------------|----------|
| DT-18 | §2 | Zero-friction condim downgrade optimization — detect `mu[0..dim-1] == 0` and downgrade to condim=1 | Low |
| DT-19 | §2 | QCQP-based cone projection — jointly project normal+friction forces (MuJoCo PGS style) | Medium |
| DT-20 | §2 | `J^T * lambda` vs manual chain-walk force application — unify contact force application | Low |
| DT-21 | §15 | `xfrc_applied` support in `qfrc_smooth` — external Cartesian body forces | Medium |
| DT-22 | §15 | `efc_impP` — impedance derivative field for external API introspection | Low |
| DT-23 | §15 | `dof_solref_fri` / `dof_solimp_fri` — per-DOF friction loss solver params | Medium |
| DT-24 | §16 | Incremental collision detection on tree wake — only re-collide woken geoms | Low |
| DT-25 | §11 | Deformable-rigid friction cone projection — normal-only solver is current scaffold | Medium |
| DT-26 | §11 | Contact re-detect + re-solve iteration loop after XPBD projection | Low |
| DT-27 | §11 | XPBD solver cross-iteration lambda accumulation fix | Low |

---

## Group 3 — Tendon System (8 items)

| §DT | Origin | Description | Priority |
|-----|--------|-------------|----------|
| DT-28 | §4 | Ball/Free joints in fixed tendons — validation + qvel DOF index mapping | Low |
| DT-29 | §4 | Spatial tendon dense J^T multiplication path (vs sparse wrap-array path) | Low |
| DT-30 | §4 | Compound pulley physics — capstan friction, pulley inertia (`sim-tendon/pulley.rs`) | Low |
| DT-31 | §4 | `WrapType::Joint` inside spatial tendons (`mjWRAP_JOINT`) — uncommon, parser rejects | Low |
| DT-32 | §4 | Per-tendon `solref_limit`/`solimp_limit` — constraint solver params (pre-existing gap) | Medium |
| DT-33 | §4 | Tendon `margin` attribute — limit activation distance (pre-existing gap) | Medium |
| DT-34 | §4 | Sparse Jacobian representation for spatial tendons — cache nonzero DOF indices | Low |
| DT-35 | §4 | Tendon spring/damper forces produce zero in implicit mode — non-diagonal K coupling needed | Medium |

---

## Group 4 — Solver Optimizations (9 items)

| §DT | Origin | Description | Priority |
|-----|--------|-------------|----------|
| DT-36 | §2 | Flat CSR format for `qLD_L` — replace `Vec<Vec<(usize, f64)>>` with CSR | Low |
| DT-37 | §2 | Sparse factorization for implicit integrator path — currently dense `cholesky_in_place` | Low |
| DT-38 | §3 | Implicit matrix-vector products for CG — avoid dense Delassus assembly for large contact counts | Medium |
| DT-39 | §15 | Body-weight diagonal approximation for `diagApprox` — add `body_invweight0`, `dof_invweight0` | Low |
| DT-40 | §15 | LDL^T factorization for Hessian — robustness on extreme stiffness settings | Low |
| DT-41 | §15 | Newton solver for implicit integrators — currently warns + falls back to PGS | Low |
| DT-42 | §15/§16 | Per-island Newton solver dispatch — Hessian block-diagonal refactor needed | Low |
| DT-43 | §16 | Selective CRBA backward-pass optimization — defer to per-island block structure | Low |
| DT-44 | §16 | Sparse mass matrix for island extraction — currently dense `DMatrix` indexing | Low |

---

## Group 5 — Derivatives & Analytical Methods (11 items)

| §DT | Origin | Description | Priority |
|-----|--------|-------------|----------|
| DT-45 | §12 | Full position-analytical derivatives (`dFK/dq`, `dM/dq`) — massive complexity, deferred | Low |
| DT-46 | §12 | Contact-analytical derivatives — implicit function theorem through PGS/CG | Low |
| DT-47 | §12 | Sensor derivatives (C, D matrices) — `TransitionMatrices` reserves `Option` fields | Low |
| DT-48 | §12 | Sparse derivative storage — all matrices dense, follow-up for nv > 100 | Low |
| DT-49 | §12 | Parallel FD computation — each perturbation column requires sequential `step()` | Low |
| DT-50 | §12 | Automatic differentiation — dual numbers / enzyme, no scalar type genericity changes | Low |
| DT-51 | §12 | `mjd_inverseFD` — inverse dynamics derivatives deferred | Low |
| DT-52 | §12 | `mjd_subQuat` — quaternion subtraction Jacobians for constraint derivatives | Low |
| DT-53 | §12 | Skip-stage optimization (`mj_forwardSkip`) — ~50% per-column FD cost reduction | Medium |
| DT-54 | §12 | Muscle actuator velocity derivatives — piecewise FLV curve gradients, FD only | Low |
| DT-55 | §13 | `skipfactor` / factorization reuse across implicit steps | Low |

---

## Group 6 — Actuator & Dynamics (6 items)

| §DT | Origin | Description | Priority |
|-----|--------|-------------|----------|
| DT-56 | §12 | `dampratio` for position actuators — requires `acc0` at compile time | Low |
| DT-57 | §12 | `acc0` computation for non-muscle actuators — extend `compute_muscle_params()` | Low |
| DT-58 | §5 | sim-muscle Hill-type model as `ActuatorDynamics::HillMuscle` variant | Low |
| DT-59 | §5 | Bisection-based lengthrange for unlimited slide joints (`mj_setLengthRange`) | Low |
| DT-60 | §35 | `jnt_actgravcomp` routing to `qfrc_actuator` — currently all goes to `qfrc_passive` | Medium |
| DT-61 | §35 | `DISABLE_GRAVITY` flag not defined — only `gravity.norm() == 0.0` check exists | Low |

---

## Group 7 — Sensor Gaps (4 items)

| §DT | Origin | Description | Priority |
|-----|--------|-------------|----------|
| DT-62 | §6 | Frame sensor `objtype` attribute not parsed — fallback tries site → body → geom | Medium |
| DT-63 | §6 | Frame sensor `reftype`/`refid` not wired — relative-frame measurements | Low |
| DT-64 | §6 | Multi-geom Touch bodies — only first geom on body matched | Medium |
| DT-65 | §6 | User sensor `dim` attribute not parsed — `User.dim()` returns 0 | Low |

---

## Group 8 — Flex / Deformable Body (14 items)

| §DT | Origin | Description | Priority |
|-----|--------|-------------|----------|
| DT-66 | §6B | `<equality><flex>` constraints — flex-flex coupling via equality constraints | Low |
| DT-67 | §6B | GPU flex pipeline — GPU acceleration of flex constraint solve (Phase 3E) | Low |
| DT-68 | §6B | Per-vertex material variation — all vertices share single material | Low |
| DT-69 | §6B | SAP integration for flex broadphase — currently brute-force O(V*G) | Low |
| DT-70 | §11 | Deformable-vs-Mesh/Hfield/SDF narrowphase — only primitives supported | Medium |
| DT-71 | §11 | Behavioral friction tests for deformable — deferred until DT-25 lands | Low |
| DT-72 | §36 | Flex contacts not wired for adhesion — AC12 test documented skip | Low |
| DT-73 | §6B | Volume constraints — no MuJoCo equivalent; emergent from continuum model | Low |
| DT-85 | §27B | Flex `<contact>` runtime attributes not wired: `internal`, `activelayers`, `vertcollide`, `passive` | Low |
| DT-86 | §27B | `elastic2d` keyword on `<flex><elasticity>` — model selection `[none, bend, stretch, both]` | Low |
| DT-87 | §27D | Shared-body flex vertices — multiple vertices referencing same body's DOFs not implemented | Low |
| DT-88 | §27E | `<flexcomp>` deferred physics attributes: `inertiabox`, `scale`, `quat`, `file` | Low |
| DT-89 | §27E | `<flexcomp>` deferred rendering attributes: `flatskin`, `material`, `rgba` | Low |
| DT-90 | §27E/§30 | `flex_friction` scalar → `Vector3<f64>` — torsional/rolling friction data lost | Low |

---

## Group 9 — Misc Pipeline & API (13 items)

| §DT | Origin | Description | Priority |
|-----|--------|-------------|----------|
| DT-74 | §4 | `compute_body_jacobian_at_point()` incomplete — only x-component, `#[allow(dead_code)]` | Medium |
| DT-75 | §4 | `add_body_jacobian` free joint bug — world-frame unit vectors instead of body-frame `R*e_i` | Medium |
| DT-76 | §8 | Pre-allocated `efc_lambda_saved` for RK4 — avoid `efc_lambda.clone()` per step | Low |
| DT-77 | §5 | Length-range auto-estimation for site-transmission muscle actuators (no-op stub) | Low |
| DT-78 | §4 | `actuator_lengthrange` for unlimited spatial tendons — wrap-array DOF lookup wrong | Low |
| DT-79 | §14 | User callbacks `mjcb_*` Rust equivalents — closures vs trait objects, thread safety | Medium |
| DT-80 | §14 | Mocap body + equality weld constraint integration testing | Low |
| DT-81 | §14 | `key_userdata` support — no `userdata` concept in CortenForge | Low |
| DT-82 | §9 | SoA layout across environments for cache locality — deferred to GPU work | Low |
| DT-83 | §9 | Multi-model batching — different robots in same batch (per-env dimensions) | Low |
| DT-84 | §32 | `mju_encodePyramid` utility not implemented — API compatibility only | Low |
| DT-91 | §2 | Warmstart `Vec<f64>` → `SmallVec<[f64; 6]>` — avoid heap allocation in warmstart vectors | Low |
| DT-92 | §9 | Parallel reset for `BatchSim` — sequential O(nq+nv+nu+na) reset deferred | Low |

---

## Summary

| Group | Items | Count |
|-------|-------|-------|
| 1. Defaults & MJCF Parsing Gaps | DT-1 – DT-17 | 17 |
| 2. Contact & Collision System | DT-18 – DT-27 | 10 |
| 3. Tendon System | DT-28 – DT-35 | 8 |
| 4. Solver Optimizations | DT-36 – DT-44 | 9 |
| 5. Derivatives & Analytical Methods | DT-45 – DT-55 | 11 |
| 6. Actuator & Dynamics | DT-56 – DT-61 | 6 |
| 7. Sensor Gaps | DT-62 – DT-65 | 4 |
| 8. Flex / Deformable Body | DT-66 – DT-73, DT-85 – DT-90 | 14 |
| 9. Misc Pipeline & API | DT-74 – DT-84, DT-91 – DT-92 | 13 |
| **Total** | | **92** |

**Priority breakdown:** 17 Medium, 75 Low. No High — these are all sub-items
within completed tasks, not critical gaps.
