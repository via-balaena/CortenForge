# Post-v1.0 Roadmap

Consolidated tracker for all deferred tasks (DT items) discovered during
CortenForge v1.0 development. Supersedes `future_work_10b.md` through
`future_work_10j.md` (archived to `archived/`).

**Status:** v1.0 shipped 2026-03-11. All 13 phases complete. Conformance
gate 79/79. This document tracks post-v1.0 work only.

---

## Summary

| Metric | Count |
|--------|------:|
| Total DT items (DT-1 through DT-180) | 178 |
| Completed | 56 |
| Retired | 1 |
| Remaining (open) | 121 |

Numbers DT-145 and DT-149 were never assigned. DT-179/180 added by cf-design Phase 5.

### Open items by tier

| Tier | Count | Description |
|------|------:|-------------|
| T1 | 60 | Plan + implement (mechanical, parent spec defines the "what") |
| T2 | 45 | Grouped spec (related items share one spec) |
| T3 | 16 | Individual spec (algorithmic complexity, dedicated design needed) |

---

## Performance Optimizations

| DT | Tier | Description | Source |
|----|------|-------------|--------|
| DT-18 | T1 | Zero-friction condim downgrade optimization -- detect `mu[0..dim-1] == 0` and downgrade to condim=1 | 10c |
| DT-20 | T2 | Unify `J^T * lambda` vs chain-walk contact force application | 10c |
| DT-24 | T2 | Incremental collision detection on tree wake -- only re-collide woken geoms | 10c |
| DT-29 | T1 | Spatial tendon dense `J^T` multiplication path (vs sparse wrap-array path) | 10d |
| DT-34 | T1 | Sparse Jacobian representation for spatial tendons -- cache nonzero DOF indices | 10d |
| DT-36 | T2 | Flat CSR format for `qLD_L` -- replace `Vec<Vec<(usize, f64)>>` with CSR | 10e |
| DT-37 | T2 | Sparse factorization for implicit integrator path -- currently dense `cholesky_in_place` | 10e |
| DT-38 | T3 | Implicit matrix-vector products for CG -- avoid dense Delassus assembly for large contact counts | 10e |
| DT-40 | T2 | LDL^T factorization for Hessian -- robustness on extreme stiffness settings | 10e |
| DT-42 | T3 | Per-island Newton solver dispatch -- Hessian block-diagonal refactor needed | 10e |
| DT-43 | T1 | Selective CRBA backward-pass optimization -- defer to per-island block structure | 10e |
| DT-44 | T2 | Sparse mass matrix for island extraction -- currently dense `DMatrix` indexing | 10e |
| DT-48 | T2 | Sparse derivative storage -- all matrices dense, follow-up for nv > 100 | 10f |
| DT-49 | T2 | Parallel FD computation -- each perturbation column requires sequential `step()` | 10f |
| DT-55 | T1 | `skipfactor` / factorization reuse across implicit steps | 10f |
| DT-76 | T1 | Pre-allocated `efc_lambda_saved` for RK4 -- avoid `efc_lambda.clone()` per step | 10j |
| DT-91 | T1 | Warmstart `Vec<f64>` to `SmallVec<[f64; 6]>` -- avoid heap allocation | 10j |
| DT-92 | T1 | Parallel reset for `BatchSim` -- sequential O(nq+nv+nu+na) reset deferred | 10j |
| DT-105 | T3 | Sparse `actuator_moment` compression (CSR) -- numerically equivalent to current dense storage | 10g |
| DT-130 | T3 | Dense AR matrix optimization -- PGS currently computes full nefc x nefc `efc_AR`; MuJoCo uses sparse row-level `ARblock` | 10e |
| DT-134 | T2 | Mesh-primitive dispatch to GJK/EPA on convex hulls -- mesh-sphere, mesh-capsule, mesh-box pairs currently use per-triangle BVH | 10j |
| DT-179 | T2 | Mesh-plane collision per-step cost ~120ms vs ~3µs without -- adding a ground plane to a mesh-body scene makes `step()` ~40,000× slower. Blocks cf-design contact-force optimization loop. Profile broadphase rejection, narrowphase vertex iteration, and constraint assembly for mesh-plane pairs. | cf-design Phase 5 |
| DT-140 | T2 | Prism BVH acceleration for heightfield collision -- `collide_hfield_multi` iterates sub-grid cells linearly | 10i |
| DT-141 | T2 | GJK/EPA cross-frame simplex warm-starting -- cache previous frame's GJK simplex per geom pair | 10j |
| DT-146 | T2 | Sparse constraint assembly -- scatter `flexedge_J` into sparse `efc_J` instead of dense DMatrix | 10i |
| DT-147 | T1 | `flex_edgeequality` dedicated flag -- per-flex boolean for `skip_jacobian` condition instead of conservative `solref != [0,0]` proxy | 10i |
| DT-148 | T1 | `flex_hingeadr`/`flex_hingenum` per-flex hinge index arrays -- O(1) Bridson bending iteration instead of filter loop | 10i |
| DT-157 | T2 | Analytical sensor derivatives -- per-sensor-type analytical Jacobians (CortenForge extension, not MuJoCo conformance) | 10f |
| DT-158 | T2 | Inverse dynamics sensor derivatives (`DsDq`/`DsDv`/`DsDa` in `mjd_inverseFD`) | 10f |
| DT-159 | T1 | `step()` to `forward_skip + integrate` migration for FD loops -- `skipsensor` optimization | 10f |

## Advanced Differentiation

| DT | Tier | Description | Source |
|----|------|-------------|--------|
| DT-45 | T3 | Full position-analytical derivatives (`dFK/dq`, `dM/dq`) -- massive complexity | 10f |
| DT-46 | T3 | Contact-analytical derivatives -- implicit function theorem through PGS/CG | 10f |
| DT-50 | T3 | Automatic differentiation -- dual numbers / enzyme, no scalar type genericity changes | 10f |

## CortenForge Extensions Beyond MuJoCo

| DT | Tier | Description | Source |
|----|------|-------------|--------|
| DT-4 | T1 | `<sdf>` inline distance grid asset element (no standard MuJoCo equivalent) | 10b |
| DT-26 | T2 | Contact re-detect + re-solve iteration loop after XPBD projection | 10c |
| DT-27 | T2 | XPBD solver cross-iteration lambda accumulation fix | 10c |
| DT-30 | T3 | Compound pulley physics -- capstan friction, pulley inertia (`sim-tendon/pulley.rs`) | 10d |
| DT-66 | T3 | `<equality><flex>` constraints -- flex-flex coupling via equality constraints | 10i |
| DT-67 | T3 | GPU flex pipeline -- GPU acceleration of flex constraint solve | 10i |
| DT-68 | T1 | Per-vertex material variation for flex -- all vertices share single material | 10i |
| DT-73 | T3 | Volume constraints for flex bodies -- no MuJoCo equivalent; continuum model | 10i |
| DT-82 | T3 | SoA layout across environments for cache locality -- deferred to GPU work | 10j |
| DT-83 | T3 | Multi-model batching -- different robots in same batch (per-env dimensions) | 10j |
| DT-86 | T1 | `elastic2d` keyword on `<flex><elasticity>` -- model selection `[none, bend, stretch, both]` | 10i |
| DT-87 | T2 | Shared-body flex vertices -- multiple vertices referencing same body's DOFs | 10i |
| DT-111 | T3 | HillMuscle compliant tendon mode -- persistent fiber state (`act_num >= 2`) + custom integration | 10g |
| DT-112 | T1 | HillMuscle named MJCF attributes (`optlen`, `slacklen`, `pennation`) -- convenience UX over raw `gainprm` indices | 10g |
| DT-113 | T1 | `<hillmuscle>` shortcut element -- analogous to `<muscle>`, auto-sets dyntype/gaintype/biastype | 10g |
| DT-114 | T2 | HillMuscle variable pennation angle -- `alpha = asin(w / L_fiber)` as function of fiber length | 10g |
| DT-115 | T2 | HillMuscle configurable curve parameters -- Gaussian FL widths, FV curvature, FP shape via `gainprm`/`biasprm` | 10g |
| DT-116 | T3 | Per-actuator `GainType::User` / `BiasType::User` callback infrastructure -- dependency on plugin system satisfied (Phase 13 Spec D) | 10g |

## GPU Pipeline

| DT | Tier | Description | Source |
|----|------|-------------|--------|
| DT-136 | T3 | GPU convex hull computation -- move Quickhull to compute shader for large meshes | ROADMAP_V1 |
| DT-138 | T1 | GPU-accelerated mesh inertia computation -- move shell/exact/legacy inertia to compute shader | ROADMAP_V1 |

## Solver and Constraint Conformance

| DT | Tier | Description | Source |
|----|------|-------------|--------|
| DT-131 | T1 | Flex condim=6 silently downgrades to condim=3 -- `make_contact_flex_rigid()` maps `_ => 3`. Conformant with MuJoCo C but should emit warning. | 10c |
| DT-132 | T2 | Bodyweight diagApprox double-counts rigid body for flex contacts -- `make_contact_flex_rigid()` sets both geoms to rigid, ignoring flex vertex mass | 10c |
| DT-133 | T2 | Bodyweight diagApprox uses rotational weight for flex friction rows -- should use translational component | 10c |
| DT-180 | T2 | Mesh-plane contact forces ~10^16× too large vs primitive-plane -- sphere(0.05) mesh geom on plane produces 2.5e17 N (expected ~6 N from weight). Primitive sphere of same size produces correct ~5100 N. `collide_mesh_plane` vertex projection + `make_contact_from_geoms` uses same path as primitive; likely mass/inertia computation from mesh or constraint impedance scaling issue. | cf-design Phase 5 |
| DT-161 | T1 | Pyramidal `efc_diagApprox` bodyweight factor-of-2 -- CF stores `(1+mu^2)*w_tran`, MuJoCo stores `2*(1+mu^2)*w_tran` for pyramidal facet rows. No downstream effect (R_py post-processing overwrites). | ROADMAP_V1 |
| DT-162 | T1 | PGS `solver_stat` `nactive`/`nchange` per-iteration counting -- MuJoCo calls `dualState()` per sweep; CF uses placeholder 0. Diagnostic only. | Phase 13 |
| DT-163 | T1 | PGS warmstart primal cost gate -- CF uses dual cost (`< 0`), MuJoCo uses primal cost (`> 0`). Equivalent at optimum; no measurable divergence. | Phase 13 |
| DT-164 | T2 | Newton solver golden flag convergence -- 24/26 golden flag tests fail at ~0.002 qacc divergence. Assembly and PGS verified correct. Residual is Newton convergence behavior. | Phase 13 |

## Flex / Deformable Body

| DT | Tier | Description | Source |
|----|------|-------------|--------|
| DT-69 | T2 | SAP integration for flex broadphase -- currently brute-force O(V*G) | 10i |
| DT-71 | T1 | Behavioral friction tests for deformable -- additional coverage beyond DT-25 verification (friction coefficient sweep, stick-slip transition) | 10i |
| DT-72 | T1 | Flex contacts not wired for adhesion -- AC12 test documented skip | 10i |
| DT-89 | T1 | `<flexcomp>` deferred rendering attributes: `flatskin`, `material`, `rgba` | 10i |
| DT-150 | T2 | `activelayers` runtime filtering for flex self-collision -- parsed and stored but not consumed at runtime | 10i |
| DT-151 | T2 | Edge-edge tests for dim=3 tetrahedral self-collision -- MuJoCo performs edge-edge proximity tests; CortenForge vertex-face only | 10i |
| DT-152 | T2 | Barycentric force distribution on face side for flex self-collision contacts -- current Jacobian applies force to nearest vertex | 10i |
| DT-153 | T1 | Island assignment for flex contacts -- sentinel `usize::MAX` geom indices skipped from island assignment. Affects island-based solving only. | 10i |
| DT-154 | T1 | Flex contact factory condim=6 mapping -- all flex factories map `condim: 1->1, 4->4, _->3`. `condim=6` produces `dim=3`. | 10i |
| DT-155 | T1 | S10 override test for flex-flex contacts (AC11/T10) -- `ENABLE_OVERRIDE` test infrastructure not wired for flex contact tests | 10i |
| DT-156 | T2 | Narrowphase triangle-triangle contact count conformance gap -- CortenForge 36 contacts vs MuJoCo 32 for overlapping 3x3 flex grids | 10i |

## Plugin and Extension System

| DT | Tier | Description | Source |
|----|------|-------------|--------|
| DT-170 | T2 | SDF collision dispatch -- plugin trait defines SDF callbacks but collision pipeline does not call them. Requires `mjc_SDF()` equivalent. | Phase 13 |
| DT-171 | T1 | Resource providers and decoders -- `mjpResourceProvider`/`mjpDecoder` are separate from physics plugins. No conformance impact. | Phase 13 |
| DT-172 | T1 | Plugin copy/destroy callbacks -- MuJoCo's `copy` and `destroy` lifecycle callbacks. Currently `plugin_data` reset to `None` on clone. | Phase 13 |
| DT-173 | T2 | Data clone plugin_data preservation -- `Data::clone()` resets `plugin_data` to `None`. Full-fidelity clone requires `Plugin::copy()`. Depends on DT-172. | Phase 13 |
| DT-174 | T1 | Sensor cutoff stage check hardening -- `compute_plugin_sensors()` omits `sensor_needstage == stage` check. Functionally correct but does not exactly match spec. | Phase 13 |

## Tendon System

| DT | Tier | Description | Source |
|----|------|-------------|--------|
| DT-31 | T2 | `WrapType::Joint` inside spatial tendons (`mjWRAP_JOINT`) -- uncommon, parser rejects | 10d |

## Composite and Procedural Generation

| DT | Tier | Description | Source |
|----|------|-------------|--------|
| DT-165 | T2 | Cable skin generation -- rendering-only box-geom cable skin with bicubic interpolation and subgrid support. Visual-only. | Phase 13 |
| DT-166 | T1 | Custom text metadata for composites -- cable adds `composite_{prefix}` text element. Requires `<custom><text>` infrastructure. | Phase 13 |
| DT-167 | T2 | `<flexcomp>` element -- separate MJCF element from `<composite>`. Generates flex bodies from templates (box, cylinder, ellipsoid, mesh). | Phase 13 |
| DT-168 | T1 | `<replicate>` element -- MuJoCo 3.4.0 replacement for deprecated `<composite type="particle">`. Repeats body templates at specified positions. | Phase 13 |
| DT-169 | T1 | Cable `<site>` template customization -- `<site>` child element support inside `<composite>` for boundary site properties. | Phase 13 |

## Low-Priority MuJoCo Compatibility

| DT | Tier | Description | Source |
|----|------|-------------|--------|
| DT-1 | T1 | Mesh defaults `apply_to_mesh()` -- root-only mesh scale defaults deferred | 10b |
| DT-5 | T2 | `gaintype/biastype/dyntype="user"` -- callback-based types require plugin system | 10b |
| DT-7 | T1 | `actdim` explicit override not supported (auto-detection only) | 10b |
| DT-10 | T1 | Deferred `<compiler>` attributes: `fitaabb`, `usethread`, `alignfree`, `saveinertial`, `inertiagrouprange`, `<lengthrange>` child | 10b |
| DT-12 | T1 | Programmatic API enforcement that `worldbody.childclass` must be `None` | 10b |
| DT-15 | T1 | Sentinel-value detection for `gear`/`kp`/`noise`/`cutoff` should migrate to `Option<T>`. Phase 7 Spec A added 14 sentinel-detection fields -- primary migration candidates. | 10b |
| DT-17 | T1 | Global `<option o_margin>` override -- per-geom margin is correct foundation; this covers the separate global option | 10b |
| DT-22 | T1 | `efc_impP` -- impedance derivative field for external API introspection | 10c |
| DT-65 | T1 | User sensor `dim` attribute not parsed -- `User.dim()` returns 0. Also requires `sensor_intprm` array. | 10h |
| DT-80 | T1 | Mocap body + equality weld constraint integration testing | 10j |
| DT-81 | T1 | `key_userdata` support -- no `userdata` concept in CortenForge | 10j |
| DT-84 | T1 | `mju_encodePyramid` utility not implemented -- API compatibility only | 10j |
| DT-96 | T1 | Lazy energy evaluation (`flg_energypos`/`flg_energyvel`) -- only matters with plugins or energy-dependent sensors | 10j |
| DT-101 | T2 | `mj_contactPassive()` -- viscous contact damping forces. Guard site already specified in S4.7d. | 10c |
| DT-104 | T2 | Ball/free joint transmission -- `nv == 3` and `nv == 6` sub-paths in `mj_transmission()` | 10b |
| DT-107 | T2 | Runtime interpolation logic -- `mj_forward` reads history buffer for delayed ctrl, `mj_step` writes circular buffer. Covers both actuators and sensors. | 10g |
| DT-108 | T1 | `dyntype` enum gating interpolation eligibility -- restrict which `ActuatorDynamics` variants may use history buffer | 10g |
| DT-110 | T1 | `actuator_plugin` model array -- per-actuator plugin ID (`int[nu]`, -1 sentinel). Plugin system dependency satisfied. | 10g |
| DT-118 | T2 | `mj_contactForce()` -- touch sensor force reconstruction via full contact force vector | 10h |
| DT-119 | T2 | Ray-geom intersection filter for touch sensor. Depends on DT-118. | 10h |
| DT-120 | T1 | `MjObjectType::Camera` -- frame sensor camera support (reftype="camera" currently warns + ignores) | 10h |
| DT-121 | T1 | `InsideSite` sensor (`mjSENS_INSIDESITE`) -- MuJoCo 3.x sensor type not yet supported | 10h |
| DT-122 | T2 | Mesh/Hfield/SDF geom distance support for `GeomDist`/`GeomPoint`/`GeomNormal` sensors | 10h |
| DT-123 | T1 | `IntVelocity` enum variant -- concrete `<intvelocity>` elements not yet supported (defaults parsing works) | 10b |
| DT-124 | T1 | Muscle sentinel detection for `<general dyntype="muscle">` path (`gainprm[0]==1` quirk). Known conformance divergence. | 10b |
| DT-125 | T2 | `mj_setConst()` runtime `qpos_spring` recomputation -- currently `qpos_spring` is set at build time and static | ROADMAP_V1 |
| DT-126 | T1 | Camera user data (`cam_user`/`nuser_cam`) -- per-camera user data (8th element type for `user` attribute) | 10b |
| DT-127 | T1 | Mixed-sign `solref` validation -- `(solref[0] > 0) ^ (solref[1] > 0)` triggers MuJoCo warning and default replacement. Not validated in CortenForge. | 10c |
| DT-135 | T1 | `needhull_` collision-only hull trigger -- compute convex hulls only for meshes used in collision. Conformance-neutral. | ROADMAP_V1 |
| DT-139 | T1 | `exactmeshinertia` attribute full removal -- match MuJoCo 3.5.0 schema rejection (currently parsed + warn) | ROADMAP_V1 |
| DT-144 | T2 | Prism-based hfield collision for flex vertices -- currently uses point-sampling. MuJoCo uses prism-based GJK. | 10i |
| DT-160 | T1 | Additional FK reference fields -- `xmat`, `geom_xpos`, `geom_xmat`, `site_xpos`. Extend conformance reference and add Layer B tests. | 10j |
| DT-175 | T1 | Thread margin into mesh collision helpers -- `mesh_collide.rs` receives margin but does not propagate into sub-helpers | Cleanup |
| DT-176 | T1 | Thread margin into SDF collision helpers -- `sdf_collide.rs` receives margin but does not propagate into SDF distance queries | Cleanup |
| DT-177 | T2 | URDF mesh asset support -- `converter.rs` skips `<mesh>` geometry elements (lacks asset declaration infrastructure) | Cleanup |
| DT-178 | T1 | Height field visualization -- `sim-bevy` mesh conversion returns `None` for `CollisionShape::HeightField`. Requires grid-to-mesh tessellation. | Cleanup |

## Code Quality

| DT | Tier | Description | Source |
|----|------|-------------|--------|
| DT-117 | T2 | Eliminate `unwrap()`/`expect()` from library code (~1,085 call sites). Convert to `?` propagation, `Result` returns, or `unsafe { get_unchecked() }` with safety invariant comments. | ROADMAP_V1 |

## Other Non-Critical

| DT | Tier | Description | Source |
|----|------|-------------|--------|
| DT-137 | T1 | Deeply concave mesh test (C/U-shape) for legacy vs exact inertia differentiation -- nice-to-have test enhancement | ROADMAP_V1 |

---

## Completed Items

Items resolved during Phases 1-13. Listed here for traceability -- every DT
number is accounted for.

### Defaults and MJCF Parsing (Phase 7)

| DT | Description | Resolved In |
|----|-------------|-------------|
| DT-2 | Equality constraint defaults (`solref`/`solimp` in defaults structs) | Phase 7 Spec A |
| DT-3 | File-based hfield loading from PNG | Phase 7 T1 |
| DT-11 | `range` as defaultable attribute in `MjcfJointDefaults` | Phase 7 Spec A (already implemented) |
| DT-13 | `qpos_spring` -- distinct from `qpos0` | Phase 7 Spec A |
| DT-14 | Actuator type-specific defaults (cylinder area/timeconst, muscle params) | Phase 7 Spec A |
| DT-16 | Flex `density` attribute location wrong in parser vs MuJoCo spec | Phase 4 |
| DT-85 | Flex `<contact>` runtime attributes: `internal`, `activelayers`, `vertcollide`, `passive` | Phase 7 T1 |
| DT-88 | `<flexcomp>` attributes: `inertiabox`, `scale`, `quat`, `file` | Phase 7 Spec C |

### Actuator Completeness (Phase 5)

| DT | Description | Resolved In |
|----|-------------|-------------|
| DT-6 | `actearly` attribute wired to runtime | Phase 5 Session 1 |
| DT-8 | Transmission types: `cranksite`, `slidersite`, `jointinparent` | Phase 5 Spec B |
| DT-9 | `nsample`, `interp`, `delay` interpolation attributes (parsing + storage) | Phase 5 Spec D (partial -- runtime: DT-107, gating: DT-108) |
| DT-56 | `dampratio` for position actuators (requires `acc0`) | Phase 5 Spec A |
| DT-57 | `acc0` computation for non-muscle actuators | Phase 5 Spec A |
| DT-58 | Hill-type muscle model as `ActuatorDynamics::HillMuscle` variant | Phase 5 Spec C |
| DT-59 | Bisection-based `lengthrange` for unlimited slide joints | Phase 5 Spec A |
| DT-106 | Gear-scaling in `uselimit` lengthrange (intentional MuJoCo deviation) | Phase 5 Spec A |

### Sensor Completeness (Phase 6)

| DT | Description | Resolved In |
|----|-------------|-------------|
| DT-62 | Frame sensor `objtype` attribute parsing | Phase 6 Spec A |
| DT-63 | Frame sensor `reftype`/`refid` relative-frame measurements | Phase 6 Spec B |
| DT-64 | Multi-geom touch sensor aggregation | Phase 6 Spec A |
| DT-102 | Geom-attached acc-stage sensors (FrameLinAcc/FrameAngAcc) | Phase 6 Spec A |
| DT-103 | Extract `mj_objectAcceleration()` spatial transport helpers | Phase 4 |
| DT-109 | Sensor history attributes (`nsample`/`interp`/`delay`/`interval`) | Phase 6 Spec D |

### Jacobian and Dynamics (Phase 4 / Phase 8)

| DT | Description | Resolved In |
|----|-------------|-------------|
| DT-21 | `xfrc_applied` support in `qfrc_smooth` (external Cartesian body forces) | Phase 4 |
| DT-35 | Tendon spring/damper forces in implicit mode (non-diagonal K/D coupling) | Phase 8 |
| DT-74 | Canonical `mj_jac` -- full body Jacobian API | Phase 4 |
| DT-75 | `add_body_jacobian` free joint bug (world-frame vs body-frame) | Phase 4 |
| DT-78 | `actuator_lengthrange` for unlimited spatial tendons | Phase 1 |
| DT-79 | User callbacks `mjcb_*` Rust equivalents | Phase 4 |

### Tendon (Phase 8)

| DT | Description | Resolved In |
|----|-------------|-------------|
| DT-28 | Ball/free joints in fixed tendons (qpos/dof address mapping) | Phase 8 T1 |
| DT-33 | Tendon `margin` attribute for limit activation distance | Phase 8 Spec A |

### Collision (Phase 8 / Phase 9 / Phase 10)

| DT | Description | Resolved In |
|----|-------------|-------------|
| DT-25 | Deformable-rigid friction cone projection (condim=3 verified) | Phase 8 (partial -- remaining gaps: DT-131, DT-132, DT-133) |
| DT-70 | Deformable-vs-mesh/hfield/SDF narrowphase | Phase 9 Spec E |
| DT-90 | `flex_friction` scalar to `Vector3<f64>` | Phase 4 |
| DT-142 | Flex self-collision dispatch (BVH/SAP midphase + narrowphase) | Phase 10 Spec C |
| DT-143 | Flex-flex cross-body collision filtering (contype/conaffinity) | Phase 10 Spec D |

### Runtime Flags (Phase 4 / subsumed by section 41)

| DT | Description | Resolved In |
|----|-------------|-------------|
| DT-60 | `jnt_actgravcomp` routing to `qfrc_actuator` | Subsumed by section 41 S4.2a |
| DT-61 | `DISABLE_GRAVITY` flag not defined | Subsumed by section 41 S4.2 |
| DT-93 | Auto-reset on NaN/divergence | Subsumed by section 41 S8 |
| DT-94 | BVH midphase integration into rigid body collision pipeline | Subsumed by section 41 S9 |
| DT-95 | Global contact parameter override | Subsumed by section 41 S10 |
| DT-99 | BVH midphase integration (S9-full -- per-mesh BVH, build-phase construction) | Phase 4 (post-section 41) |
| DT-100 | Global contact parameter override guard sites (S10-full) | Phase 4 (post-section 41) |

### Solver (Phase 8 / Phase 13)

| DT | Description | Resolved In |
|----|-------------|-------------|
| DT-19 | QCQP-based cone projection (verified correct) | Phase 13 Spec B |
| DT-23 | Per-DOF friction loss solver params (`dof_solref_fri`/`dof_solimp_fri`) | Phase 13 Spec A |
| DT-32 | Per-tendon `solref_limit`/`solimp_limit` (naming conformance) | Phase 8 |
| DT-39 | Body-weight diagonal approximation (`diagApprox`) remaining code paths | Phase 13 Spec A |
| DT-41 | Newton solver for implicit integrators | Phase 8 |
| DT-128 | PGS early termination | Phase 13 Spec B |
| DT-129 | PGS warmstart two-phase projection | Phase 13 Spec B |

### Derivatives (Phase 11)

| DT | Description | Resolved In |
|----|-------------|-------------|
| DT-47 | Sensor derivatives (C, D matrices) for `TransitionMatrices` | Phase 11 Spec B |
| DT-51 | `mjd_inverseFD` -- inverse dynamics derivatives | Phase 11 Session 3 |
| DT-52 | `mjd_subQuat` -- quaternion subtraction Jacobians | Phase 11 Session 2 |
| DT-53 | `mj_forwardSkip` -- skip-stage optimization | Phase 11 Session 3 |
| DT-54 | Muscle actuator velocity derivatives | Phase 11 Session 2 |

### Conformance and Golden Files (Phase 12)

| DT | Description | Resolved In |
|----|-------------|-------------|
| DT-77 | Length-range auto-estimation for site-transmission actuators | Phase 5 Spec A |
| DT-97 | Golden file generation for per-flag trajectory conformance | Phase 12 |

### Retired

| DT | Description | Reason |
|----|-------------|--------|
| DT-98 | Remove `passive` backward-compatibility shim | Retired -- `passive` dropped entirely pre-v1.0 (no users to break) |

---

## DT Number Cross-Reference

Every DT number from DT-1 through DT-178 appears in this file. Numbers
DT-145 and DT-149 were never assigned.

Open items appear in the themed sections above. Completed and retired items
appear in the Completed Items section. Use text search (DT-NNN) to locate
any specific item.

---

## Source File Archive

The original `future_work_10b.md` through `future_work_10j.md` files have been
moved to `archived/` for historical reference. Each DT item's Source column
above references its original file.
