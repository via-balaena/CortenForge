# MuJoCo Gap Analysis: CortenForge Physics Stack

<!--
LLM/Agent Note: This document is ~2000 lines. To update it efficiently:
1. Use offset/limit reads to target specific sections (see ## headers below)
2. Key sections: Executive Summary (line ~18), §5 Geom Types (line ~465), §13 Model Format (line ~1180)
3. The document is well-structured - grep for "^## " to find all section headers
-->

This document provides a comprehensive comparison between MuJoCo's physics capabilities and CortenForge's current `sim-*` crate implementation.

> **Note:** All `sim-*` crates are in initial development (pre-1.0). Breaking changes to APIs are expected and acceptable. Prefer clean, correct implementations over backwards compatibility.

> **Current Roadmap:** For the authoritative list of verified gaps with file:line references, acceptance criteria, and dependency graph, see [`sim/docs/todo/index.md`](./todo/index.md).

---

## 📊 Executive Summary

**Overall completion: ~75-80%** of MuJoCo's core pipeline features are functional end-to-end. Some standalone crates exist but are not yet wired into the MuJoCo pipeline (`mujoco_pipeline.rs`). See [`sim/docs/todo/index.md`](./todo/index.md) for the full roadmap (Phase 1: 12 items complete, Phase 2: 17 items).

### Fully Implemented (in pipeline)
- Integration methods: Euler, RK4 (true 4-stage Runge-Kutta), ImplicitSpringDamper (diagonal spring/damper only — see [future_work_1 #7](./todo/future_work_1.md))
- Constraint solver: PGS (plain Gauss-Seidel, no SOR) + CG (preconditioned PGD with Barzilai-Borwein), Warm Starting via `WarmstartKey`
- Contact model (Compliant with solref/solimp, elliptic friction cones with variable condim 1/3/4/6, torsional/rolling friction, contype/conaffinity filtering, `<contact><pair>`/`<exclude>` two-mechanism architecture)
- Collision detection (All primitive shapes, GJK/EPA, Height fields, BVH, **TriangleMesh, SDF**)
- Joint types (Fixed, Revolute, Prismatic, Spherical, Universal, **Free**)
- Actuators: All 8 shortcut types (Motor, Position, Velocity, Damper, Cylinder, Adhesion, Muscle, General) with MuJoCo-compatible gain/bias force model (`force = gain * input + bias`), GainType/BiasType dispatch, FilterExact dynamics, control/force clamping; **Site transmissions** (6D gear, refsite, Jacobian-based wrench projection, common-ancestor zeroing)
- Sensors (32 pipeline types, all wired from MJCF): JointPos, JointVel, BallQuat, BallAngVel, FramePos, FrameQuat, FrameXAxis/YAxis/ZAxis, FrameLinVel, FrameAngVel, FrameLinAcc, FrameAngAcc, Accelerometer, Gyro, Velocimeter, SubtreeCom, SubtreeLinVel, SubtreeAngMom, ActuatorPos, ActuatorVel, ActuatorFrc, JointLimitFrc, TendonLimitFrc, TendonPos, TendonVel, Force, Torque, Touch, Rangefinder, Magnetometer, User (0-dim)
- Model loading (URDF, MJCF with `<default>` class resolution, **MJB binary format**) — `DefaultResolver` is wired into `model_builder.rs` for all element types (joints, geoms, sites, actuators, tendons, sensors)

### Placeholder / Stub (in pipeline)
- Pyramidal friction cones — `cone` field stored but solver uses elliptic cones (warning emitted if pyramidal requested)

### Recently Implemented (previously stubs)
- Site-transmission actuators ✅ — 6D gear, refsite, `mj_jac_site()`, `mj_transmission_site()`, `subquat()`, common-ancestor DOF zeroing, `ActuatorFrc` sensor fix, 23 integration tests + 9 unit tests ([future_work_2 #5](./todo/future_work_2.md))
- Spatial tendon pipeline ✅ — 3D routing with `mj_fwd_tendon_spatial()`, sphere/cylinder wrapping, sidesite disambiguation, pulley divisors, free-joint Jacobians, `accumulate_point_jacobian()`, 31 acceptance tests verified against MuJoCo 3.4.0 ([future_work_2 #4](./todo/future_work_2.md))
- General gain/bias actuator force model ✅ — all 8 shortcut types expanded to gain/bias/dynamics, `force = gain * input + bias`, GainType/BiasType dispatch, FilterExact integration ([future_work_1 #12](./todo/future_work_1.md))
- Muscle pipeline ✅ — MuJoCo FLV curves, activation dynamics, act_dot architecture, RK4 integration ([future_work_1 #5](./todo/future_work_1.md))
- Activation dynamics ✅ — Filter, FilterExact, Integrator, Muscle types all functional; `data.act` integrated by Euler/RK4
- Control/force clamping ✅ — `ctrlrange`/`forcerange` gated by `ctrllimited`/`forcelimited`; Damper/Adhesion force `ctrllimited=true`
- TendonPos/TendonVel sensors ✅ — now read live `ten_length`/`ten_velocity` ([future_work_1 #4](./todo/future_work_1.md))
- Tendon actuation ✅ — J^T force mapping in `mj_fwd_actuation()` ([future_work_1 #4](./todo/future_work_1.md))
- Fixed tendon pipeline ✅ — kinematics, passive forces, limit constraints ([future_work_1 #4](./todo/future_work_1.md))

### Standalone Crates (not wired into pipeline)
- sim-tendon (3,919 lines) — standalone crate; fixed tendons now implemented directly in pipeline ([future_work_1 #4](./todo/future_work_1.md) ✅)
- sim-muscle (2,550 lines) — standalone Hill model; MuJoCo FLV muscle model now implemented directly in pipeline ([future_work_1 #5](./todo/future_work_1.md) ✅)
- ~~sim-deformable (7,733 lines) — XPBD solver not called from `Data::step()`~~ ✅ **Pipeline** (deformable-rigid contact via split-solve Jacobi PGS + position correction + XPBD; see [future_work_4 #11](./todo/future_work_4.md) ✅)
- sim-sensor (rangefinder, magnetometer, force/torque) — standalone crate with own API; pipeline has independent implementations
- CGSolver in sim-constraint (1,664 lines) — standalone joint-space CG; pipeline contact CG is separate in `mujoco_pipeline.rs` ([future_work_1 #3](./todo/future_work_1.md) ✅)
- ~~`integrators.rs` trait system~~ — removed in FUTURE_WORK C1
- Pneumatic actuators in sim-constraint — standalone `PneumaticCylinderActuator`; pipeline cylinder/adhesion actuators use gain/bias model ([future_work_1 #12](./todo/future_work_1.md) ✅)
- Planar, Cylindrical joints in sim-constraint — not in pipeline `MjJointType` (MJCF model builder errors)

### Removed (Phase 3 Consolidation)
- Newton solver (`newton.rs` — deleted)
- Constraint island discovery (`islands.rs` — deleted)
- Sparse Jacobian operations (`sparse.rs` — deleted)
- PGS with SOR (`pgs.rs` — deleted; pipeline PGS in `mujoco_pipeline.rs` has no SOR)
- Island-parallel constraint solving (`parallel.rs` — deleted)

### ✅ Recently Completed (January–February 2026)

| Feature | Implementation | Section |
|---------|----------------|---------|
| Site-transmission actuators | 6D gear, refsite (Mode A/B), `mj_jac_site()`, `mj_transmission_site()`, `subquat()`, common-ancestor zeroing, `ActuatorFrc` sensor fix | [§6](#6-actuation) |
| Contact Condim (1/3/4/6) + Friction Cones | Variable-dimension contacts with elliptic friction cones, torsional friction (condim 4), rolling friction (condim 6), two-step projection, `apply_contact_torque()` | [§3](#3-contact-physics) |
| General gain/bias actuator force model | All 8 shortcut types with `GainType`/`BiasType` dispatch, `FilterExact` dynamics, `ctrllimited` enforcement for Damper/Adhesion | [§7](#7-actuators) |
| RK4 integrator | True 4-stage Runge-Kutta via `mj_runge_kutta()` with quaternion-safe position updates | [§1](#1-integration-methods) |
| Pipeline sensors (32 types) | All 32 types functional and wired from MJCF `<sensor>` elements via `model_builder.rs`; `set_options()` propagates `magnetic`/`wind`/`density`/`viscosity` | [§8](#8-sensors) |
| Non-convex mesh collision | TriangleMesh ↔ all primitives + mesh-mesh with BVH acceleration | [§5](#5-geom-types-collision-shapes) |
| SDF collision | All 10 shape combinations (Sphere, Capsule, Box, Cylinder, Ellipsoid, ConvexMesh, Plane, TriangleMesh, HeightField, Sdf↔Sdf) | [§5](#5-geom-types-collision-shapes) |
| MJCF `<default>` element | ✅ Full | `DefaultResolver` wired into `model_builder.rs` for all element types (joints, geoms, sites, actuators, tendons, sensors) | [§13](#13-model-format) |
| MJCF `<tendon>` parsing + pipeline | Fixed + spatial tendons fully wired (kinematics, wrapping, actuation, passive, constraints, sensors) | [§13](#13-model-format) |
| MJCF `<sensor>` parsing + wiring | 32 sensor types parsed; all 32 wired to pipeline via `process_sensors()` | [§13](#13-model-format) |
| Muscle pipeline | MuJoCo FLV curves, activation dynamics (Millard 2013), act_dot/integrator architecture, RK4 activation | [§6](#6-actuation) |
| Multi-threading | `parallel` feature with rayon: `BatchSim::step_all()` for cross-environment parallelism (island-parallel solving removed in Phase 3) | [§12](#12-performance-optimizations) |
| GPU acceleration (Phase 10a) | `sim-gpu` crate: wgpu compute shader Euler velocity integration, `GpuBatchSim` drop-in for `BatchSim`, transparent CPU fallback | [§12](#12-performance-optimizations) |
| SIMD optimization | `sim-simd` crate with `Vec3x4`, `Vec3x8`, batch operations | [§12](#12-performance-optimizations) |

**For typical robotics use cases**, collision detection, joint types, actuation (motors + muscles + filter/integrator dynamics + site transmissions), sensors (32 pipeline types, all wired from MJCF), fixed + spatial tendons (including sphere/cylinder wrapping, sidesite, pulley), and deformable bodies (split-solve contact with rigid geoms) are functional. See [`sim/docs/todo/index.md`](./todo/index.md) for the full gap list.

---

## Legend

| Status | Meaning |
|--------|---------|
| **Implemented** | Feature exists and is functional in the MuJoCo pipeline |
| **Standalone** | Crate/module exists but is not wired into the pipeline |
| **Placeholder** | Code path exists but produces incorrect/trivial results |
| **Stub** | Infrastructure exists, needs real implementation |
| **Removed** | Was implemented, then deleted (Phase 3 consolidation) |
| **Partial** | Core concept exists but incomplete |
| **Missing** | Not yet started |

---

## 1. Integration Methods

| Feature | MuJoCo | CortenForge | Status | Priority | Complexity |
|---------|--------|-------------|--------|----------|------------|
| Semi-implicit Euler | Default | `SemiImplicitEuler` | **Implemented** | - | - |
| RK4 | Supported | `RungeKutta4` | **Implemented** (true 4-stage RK4 via `mj_runge_kutta()`; see [future_work_1 #8](./todo/future_work_1.md) ✅) | - | - |
| Explicit Euler | Supported | - | Not implemented (removed standalone-only code) | - | - |
| Velocity Verlet | - | - | Not implemented (removed standalone-only code) | - | - |
| Implicit-in-velocity | Core feature | - | **Implemented** (pipeline `ImplicitSpringDamper` for diagonal spring/damper) | - | - |
| Implicit-fast (no Coriolis) | Optimization | - | Not separately implemented; `ImplicitSpringDamper` covers the primary use case | - | - |

> The pipeline uses a single `Integrator` enum (`mujoco_pipeline.rs:629`)
> with three variants: `Euler` (semi-implicit), `RungeKutta4` (true 4-stage),
> and `ImplicitSpringDamper` (diagonal spring/damper implicit Euler).
> A standalone trait-based integrator system was removed in FUTURE_WORK C1.

### Implementation Notes: Implicit Integration ✅ COMPLETED

MuJoCo's implicit integrators solve:
```
(M + h·D + h²·K) · v_new = M · v_old + h · f_ext - h · K · (q - q_eq)
```

Where `D` captures per-DOF damping and `K` captures per-DOF stiffness. This is critical for:
- Very stiff contacts
- Highly damped systems
- Muscle models with activation dynamics

**Implemented (pipeline):**
- `Integrator::ImplicitSpringDamper` in `mujoco_pipeline.rs` — implicit Euler
  for diagonal per-DOF spring stiffness K and damping D.
- Solves: `(M + h·D + h²·K)·v_new = M·v_old + h·f_ext - h·K·(q - q_eq)`

**Usage:**
```rust
use sim_mjcf::load_model;

let mjcf = r#"<mujoco>
    <option integrator="implicit"/>
    <worldbody>
        <body pos="0 0 1">
            <joint type="hinge" stiffness="100" damping="10"/>
            <geom type="sphere" size="0.1" mass="1"/>
        </body>
    </worldbody>
</mujoco>"#;

let model = load_model(mjcf).expect("load");
let mut data = model.make_data();
data.step(&model).expect("step");
```

**Files:** `sim-core/src/mujoco_pipeline.rs` (`Integrator::ImplicitSpringDamper`)

---

## 2. Constraint Solvers

| Feature | MuJoCo | CortenForge | Status | Priority | Complexity |
|---------|--------|-------------|--------|----------|------------|
| PGS (Gauss-Seidel) | Supported | `pgs_solve_contacts()` in pipeline (plain GS, no SOR) | **Implemented** (no SOR) | - | - |
| Newton solver | Default, 2-3 iterations | `NewtonConstraintSolver` | **Removed** (Phase 3 consolidation) | - | - |
| Conjugate Gradient | Supported | `cg_solve_contacts()` in pipeline (PGD+BB, named "CG") | **Implemented** ([future_work_1 #3](./todo/future_work_1.md) ✅) | - | - |
| Constraint islands | Auto-detected | `ConstraintIslands` | **Removed** (Phase 3 consolidation) | - | - |
| Warm starting | Supported | `WarmstartKey` spatial hash + `efc_lambda` | **Implemented** | - | - |

> **Note:** The pipeline now has two contact solvers in `mujoco_pipeline.rs`: `pgs_solve_contacts()` (PGS, default) and `cg_solve_contacts()` (preconditioned PGD with Barzilai-Borwein, selected via `solver_type: SolverType::CG`). Both share `assemble_contact_system()` for Delassus assembly. The pipeline PGS uses standard Gauss-Seidel (ω=1.0) with no SOR. `CGSolver` in `sim-constraint/src/cg.rs` remains a standalone joint-space solver unrelated to the pipeline contact CG.

### Implementation Notes: Newton Solver ⚠️ REMOVED (Phase 3 consolidation)

MuJoCo's Newton solver uses:
- Analytical second-order derivatives
- Cholesky factorization
- Exact line-search via 1D Newton iteration
- Typically converges in 2-3 iterations

**Implemented:**
- `NewtonConstraintSolver` with analytical Jacobians for all joint types
- Cholesky factorization (with LU fallback) for effective mass matrix
- Line search for improved convergence
- Baumgarte stabilization for position correction
- Configuration presets: `default()`, `robotics()`, `realtime()`

**Usage (historical — this code no longer compiles):**
```rust
use sim_constraint::{NewtonConstraintSolver, NewtonSolverConfig, RevoluteJoint};
use sim_types::BodyId;
use nalgebra::Vector3;

// Create solver (typically converges in 2-3 iterations)
let mut solver = NewtonConstraintSolver::new(NewtonSolverConfig::default());

// Or use presets
let mut robotics_solver = NewtonConstraintSolver::new(NewtonSolverConfig::robotics());

// Solve constraints
let result = solver.solve(&joints, |id| get_body_state(id), dt);

// Check convergence
if result.converged {
    println!("Converged in {} iterations", result.iterations_used);
}
```

**Files:** `sim-constraint/src/newton.rs` (removed in Phase 3 consolidation; solver types extracted to `sim-constraint/src/types.rs`)

### Implementation Notes: PGS (Gauss-Seidel) Solver ⚠️ REMOVED from sim-constraint; reimplemented in pipeline

> The `PGSSolver` described below was in `sim-constraint/src/pgs.rs` and was **deleted in Phase 3 consolidation**. The pipeline now has its own PGS in `mujoco_pipeline.rs` (`pgs_solve_contacts()`, line 8028) and CG (`cg_solve_contacts()`, line 8244). PGS uses plain Gauss-Seidel (ω=1.0) **without SOR**.

The original PGS solver implemented a full iterative Projected Gauss-Seidel method with
Successive Over-Relaxation (SOR).

**Algorithm:**
For each constraint i, we iteratively solve:
```
λ_i^{new} = (b_i - Σ_{j<i} A_ij λ_j^{new} - Σ_{j>i} A_ij λ_j^{old}) / A_ii
```

With SOR, the update becomes:
```
λ_i^{new} = (1 - ω) * λ_i^{old} + ω * λ_i^{gauss-seidel}
```

Where ω is the relaxation factor:
- ω = 1.0: Standard Gauss-Seidel
- ω < 1.0: Under-relaxation (more stable, slower convergence)
- ω > 1.0: Over-relaxation (faster convergence, typically 1.2-1.8)

**Implemented:**
- `PGSSolver` - Full iterative Gauss-Seidel solver
- `PGSSolverConfig` - Configurable tolerance, max iterations, SOR factor
- SOR support with configurable relaxation factor (0 < ω < 2)
- Warm starting from previous frame's solution
- Convergence tracking with optional residual history
- Configuration presets: `default()`, `realtime()`, `mujoco()`, `fast()`, `high_accuracy()`

**Usage (historical — this code no longer compiles):**
```rust
use sim_constraint::{PGSSolver, PGSSolverConfig, RevoluteJoint};
use sim_types::BodyId;
use nalgebra::Vector3;

// Create PGS solver with MuJoCo-compatible settings
let mut solver = PGSSolver::new(PGSSolverConfig::mujoco());

// Or customize with SOR for faster convergence
let config = PGSSolverConfig {
    max_iterations: 100,
    tolerance: 1e-6,
    sor_factor: 1.3,  // Over-relaxation
    warm_starting: true,
    warm_start_factor: 0.9,
    ..Default::default()
};
let mut solver = PGSSolver::new(config);

// Solve constraints
let result = solver.solve(&joints, |id| get_body_state(id), dt);

// Check convergence
if result.converged {
    println!("Converged in {} iterations (residual: {:.2e})",
        result.iterations_used, result.residual_norm);
}

// Access statistics
let stats = solver.last_stats();
println!("Used warm start: {}, SOR factor: {}",
    stats.used_warm_start, stats.sor_factor);
```

**Files:** `sim-constraint/src/pgs.rs` (removed in Phase 3 consolidation)

### Implementation Notes: Constraint Islands ⚠️ REMOVED (Phase 3 consolidation)

Constraint islands are groups of bodies connected by constraints that can be
solved independently. This enables significant performance optimizations:

- **Smaller linear systems**: Each island solves a smaller matrix equation
- **Static island skipping**: Islands where all bodies are static can be skipped
- **Parallel solving potential**: Independent islands can be solved concurrently

**Algorithm:**
- Union-Find (Disjoint-Set Union) with path compression and union by rank
- Time complexity: O(n × α(n)) ≈ O(n) where α is inverse Ackermann

**Implemented:**
- `ConstraintIslands` - Automatic island detection from joints
- `Island` - Single island with body list and constraint indices
- `IslandStatistics` - Analysis of island distribution
- `NewtonConstraintSolver::solve_with_islands()` - Island-aware solving
- `NewtonConstraintSolver::solve_islands()` - Solve with pre-computed islands

**Usage (historical — this code no longer compiles):**
```rust
use sim_constraint::{ConstraintIslands, NewtonConstraintSolver, RevoluteJoint};
use sim_types::BodyId;
use nalgebra::Vector3;

// Create joints (some bodies may be disconnected)
let joints = vec![
    RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
    RevoluteJoint::new(BodyId::new(1), BodyId::new(2), Vector3::z()),
    // Separate island
    RevoluteJoint::new(BodyId::new(10), BodyId::new(11), Vector3::z()),
];

// Automatic island detection and solving
let mut solver = NewtonConstraintSolver::default();
let result = solver.solve_with_islands(&joints, get_body_state, dt);

// Or pre-compute islands for caching between frames
let islands = ConstraintIslands::build(&joints);
let result = solver.solve_islands(&joints, &islands, &get_body_state, dt);
```

**Files:** `sim-constraint/src/islands.rs` (removed in Phase 3 consolidation), `sim-constraint/src/newton.rs` (removed in Phase 3 consolidation)

---

## 3. Contact Model

| Feature | MuJoCo | CortenForge | Status | Priority | Complexity |
|---------|--------|-------------|--------|----------|------------|
| Compliant contacts | Core | Inline in PGS solver (`mujoco_pipeline.rs`) | **Implemented** | - | - |
| Spring-damper (F = k*d^p + c*v) | Core | `compute_normal_force_magnitude` | **Standalone** (in sim-simd `batch_ops.rs` only; pipeline uses constraint-based PGS, not penalty spring-damper) | - | - |
| Nonlinear stiffness (d^p) | Core | `stiffness_power` param | **Standalone** (same as above — sim-simd only) | - | - |
| Contact margin | Supported | `contact_margin` param | **Implemented** (field stored; effect on collision thresholds TBD) | - | - |
| Elliptic friction cones | Default | `Model.cone = 1`, `project_elliptic_cone()` | **Implemented** (two-step projection: unilateral constraint + friction scaling) | - | - |
| Pyramidal friction cones | Alternative | `Model.cone` field | **Stub** (warning emitted, falls back to elliptic — requires different system size) | - | - |
| Variable condim (1/3/4/6) | Core | `Contact.dim`, `efc_offsets`, variable-dim solvers | **Implemented** (full variable-dimension support in PGS/CG) | - | - |
| Torsional friction | condim ≥ 4 | `Contact.mu[2]`, angular Jacobian row 3 | **Implemented** (Jacobian + solver + `apply_contact_torque()`) | - | - |
| Rolling friction | condim = 6 | `Contact.mu[3..5]`, angular Jacobian rows 4-5 | **Implemented** (Jacobian + solver + `apply_contact_torque()`) | - | - |
| Complete friction model | condim 6 | All components integrated | **Implemented** (condim 1, 3, 4, 6 all functional) | - | - |
| Contact pairs filtering | Supported | `contype`/`conaffinity` bitmasks, `<exclude>` body-pair exclusions, `<pair>` explicit pairs | **Implemented** (two-mechanism architecture) | - | - |
| solref/solimp params | MuJoCo-specific | `jnt_solref`, `geom_solimp`, `eq_solref`, etc. in `Model` | **Implemented** (in MuJoCo pipeline) | - | - |

### Implementation Notes: Elliptic Friction Cones ✅ IMPLEMENTED

MuJoCo uses elliptic cones by default:
```
f_n ≥ 0, ||(f_t1/μ_1, f_t2/μ_2, ...)|| ≤ f_n
```

**Implemented (February 2026):**
- `project_elliptic_cone()` — Two-step physical projection:
  1. Unilateral constraint: if λ_n < 0, contact releases (all forces = 0)
  2. Friction scaling: if friction exceeds cone, scale by λ_n/s
- `project_friction_cone()` — Dispatcher for condim 1/3/4/6
- `Model.cone = 1` (elliptic) as default; pyramidal emits warning and falls back
- Per-contact `mu: [f64; 5]` for [sliding1, sliding2, torsional, rolling1, rolling2]

**Files modified:** `sim-core/src/mujoco_pipeline.rs` (lines 8248-8312)

### Implementation Notes: Variable Condim + Advanced Friction Models ✅ IMPLEMENTED

MuJoCo uses contact dimensionality (`condim`) to specify which friction components are active:
- **condim 1**: Normal force only (frictionless) — `lambda[0]` clamped ≥ 0
- **condim 3**: Normal + 2D tangential friction (sliding) — elliptic cone projection
- **condim 4**: condim 3 + torsional friction (spinning resistance) — angular Jacobian row 3
- **condim 6**: condim 4 + 2D rolling friction (rolling resistance) — angular Jacobian rows 4-5

**Implemented (February 2026):**
- `compute_efc_offsets()` — Per-contact row offsets for variable-dimension system
- `compute_contact_jacobian()` — Returns `dim×nv` matrix with angular rows for condim ≥ 4
- `add_angular_jacobian()` — Helper for torsional/rolling Jacobian rows
- `apply_contact_torque()` — Maps world torque to generalized forces via angular Jacobian transpose
- `compute_block_jacobi_preconditioner()` — Variable `dim×dim` blocks
- `Data.efc_lambda: HashMap<WarmstartKey, Vec<f64>>` — Variable-length warmstart
- PGS and CG solvers updated for variable-dimension indexing

**Torsional Friction (condim ≥ 4):**
Opposes rotation about the contact normal (spinning). Applied via `apply_contact_torque()`:
```rust
let torsional_torque = normal * lambda[3];
apply_contact_torque(model, data, body1, -torsional_torque);
apply_contact_torque(model, data, body2, torsional_torque);
```

**Rolling Friction (condim = 6):**
Opposes rotation perpendicular to the contact normal. Applied via `apply_contact_torque()`:
```
τ_roll = -μ_roll * r_roll * F_n * normalize(ω_tangent)
```

**Pyramidal Friction Cones:**
Linearized approximation of circular/elliptic cones using flat faces. Useful for:
- LCP/QP constraint solvers requiring linear constraints
- Exact projection (no iteration needed)
- Complementarity-based contact methods

The pyramid circumscribes the circular cone (vertices touch the circle). More faces = better accuracy but more constraints.

---

## 4. Collision Detection

| Feature | MuJoCo | CortenForge | Status | Priority | Complexity |
|---------|--------|-------------|--------|----------|------------|
| Sphere-sphere | GJK/EPA | `ContactPoint::sphere_sphere` | **Implemented** | - | - |
| Sphere-plane | Native | `ContactPoint::sphere_plane` | **Implemented** | - | - |
| Box-box | GJK/EPA | `ContactPoint::box_box` | **Implemented** | - | - |
| Box-sphere | GJK/EPA | `ContactPoint::box_sphere` | **Implemented** | - | - |
| Box-plane | Native | `ContactPoint::box_plane` | **Implemented** | - | - |
| Capsule-plane | Native | `ContactPoint::capsule_plane` | **Implemented** | - | - |
| Capsule-sphere | Native | `ContactPoint::capsule_sphere` | **Implemented** | - | - |
| Capsule-capsule | Native | `ContactPoint::capsule_capsule` | **Implemented** | - | - |
| Cylinder | Native | `CollisionShape::Cylinder` | **Implemented** | - | - |
| Ellipsoid | Native | `CollisionShape::Ellipsoid` | **Implemented** | - | - |
| Convex mesh | GJK/EPA | `CollisionShape::ConvexMesh` | **Implemented** | - | - |
| Height field | Native | `CollisionShape::HeightField` | **Implemented** | - | - |
| SDF (signed distance) | Native | `CollisionShape::Sdf` | **Implemented** | - | - |
| Broad-phase (sweep-prune) | Native | `SweepAndPrune` | **Implemented** | - | - |
| Mid-phase (BVH per body) | Static AABB | `Bvh` | **Implemented** | - | - |
| Narrow-phase (GJK/EPA) | Default | `gjk_epa` module | **Implemented** | - | - |

### Implementation Notes: Collision Pipeline

The collision detection pipeline now uses:
1. ✅ **Broad-phase**: Sweep-and-prune with automatic axis selection (O(n log n))
   - `SweepAndPrune` for scenes with >32 bodies
   - `BruteForce` fallback for small scenes
   - Configurable via `BroadPhaseConfig`
2. ✅ **Mid-phase**: AABB tree per complex body for mesh primitive culling
   - `Bvh` - Top-down BVH with median splitting
   - `BvhPrimitive` - Triangle/primitive storage with AABB
   - `bvh_from_triangle_mesh()` - Helper for building BVH from mesh
3. ✅ **Narrow-phase**: GJK for intersection testing, EPA for penetration depth

**Files:** `sim-core/src/mid_phase.rs`, `sim-core/src/gjk_epa.rs`, `sim-core/src/collision_shape.rs`

### Implementation Notes: GJK/EPA ✅ COMPLETED

GJK (Gilbert-Johnson-Keerthi) and EPA (Expanding Polytope Algorithm) provide
general convex collision detection for arbitrary convex shapes.

**GJK Algorithm:**
- Works in Minkowski difference space (A - B)
- Iteratively builds a simplex to enclose the origin
- Returns true if shapes intersect (origin inside Minkowski difference)
- Complexity: O(n) iterations, each O(v) where v is vertex count

**EPA Algorithm:**
- Runs after GJK confirms intersection
- Expands the GJK simplex into a polytope
- Finds the closest face to the origin
- Returns penetration depth and contact normal

**Implemented:**
- `CollisionShape::ConvexMesh` - Convex hull defined by vertices
- `CollisionShape::tetrahedron()` - Helper for regular tetrahedra
- Support functions for all shape types (sphere, box, capsule, plane, convex mesh)
- `gjk_intersection()` - Fast intersection test
- `gjk_epa_contact()` - Full contact information (point, normal, depth)
- Automatic GJK/EPA fallback for capsule-box and other mixed collisions

**Usage:**
```rust
use sim_core::CollisionShape;
use nalgebra::{Point3, Vector3};

// Create a convex mesh (e.g., from a convex hull)
let vertices = vec![
    Point3::new(-0.5, -0.5, -0.5),
    Point3::new(0.5, -0.5, -0.5),
    Point3::new(0.5, 0.5, -0.5),
    Point3::new(-0.5, 0.5, -0.5),
    Point3::new(-0.5, -0.5, 0.5),
    Point3::new(0.5, -0.5, 0.5),
    Point3::new(0.5, 0.5, 0.5),
    Point3::new(-0.5, 0.5, 0.5),
];
let shape = CollisionShape::convex_mesh(vertices);

// Or use the tetrahedron helper
let tetra = CollisionShape::tetrahedron(0.5); // circumradius 0.5
```

**Files:** `sim-core/src/gjk_epa.rs`, `sim-core/src/collision_shape.rs`

---

## 5. Geom Types (Collision Shapes)

| Shape | MuJoCo | CortenForge | Status |
|-------|--------|-------------|--------|
| Plane | Yes | `CollisionShape::Plane` | **Implemented** |
| Sphere | Yes | `CollisionShape::Sphere` | **Implemented** |
| Box | Yes | `CollisionShape::Box` | **Implemented** |
| Capsule | Yes | `CollisionShape::Capsule` | **Implemented** |
| Convex Mesh | Yes (convexified) | `CollisionShape::ConvexMesh` | **Implemented** |
| Cylinder | Yes | `CollisionShape::Cylinder` | **Implemented** |
| Ellipsoid | Yes | `CollisionShape::Ellipsoid` | **Implemented** |
| Height field | Yes | `CollisionShape::HeightField` | **Implemented** ✅ |
| **Mesh (non-convex)** | Yes | `CollisionShape::TriangleMesh` | **Implemented** ✅ |
| **SDF** | Yes | `CollisionShape::Sdf` | **Implemented** ✅ |

### Implementation Notes: Non-Convex Mesh ✅ COMPLETED (January 2026)

Full triangle mesh collision support is now implemented:
- `CollisionShape::TriangleMesh` variant with embedded BVH
- Triangle-primitive collision: Sphere, Capsule, Box, Cylinder, Ellipsoid, Plane
- Mesh-mesh collision with dual BVH traversal
- Contact manifold generation with multiple contact points
- Performance: 172 µs for 36k triangle pairs (well under 5ms target)

**Implementation files:**
- `sim-core/src/mesh.rs` - Triangle mesh collision and mesh-mesh BVH traversal
- `sim-core/src/mid_phase.rs` - BVH for collision culling

### Implementation Notes: SDF Collision ✅ COMPLETED

Full SDF (Signed Distance Field) collision support is implemented in sim-core
and wired through the MJCF pipeline. `GeomType::Sdf` is dispatched first in
`collide_geoms()` to `collide_with_sdf()`, which handles all 10 shape
combinations. SDF geoms are programmatic (no MJCF asset element).
- `CollisionShape::Sdf` variant with 3D grid storage
- Trilinear interpolation for distance queries
- Gradient-based normal computation
- All 10 shape combinations implemented:
  - Sdf ↔ Sphere, Capsule, Box (original)
  - Sdf ↔ Cylinder, Ellipsoid (point sampling)
  - Sdf ↔ ConvexMesh (vertex sampling)
  - Sdf ↔ Plane (grid sampling)
  - Sdf ↔ TriangleMesh (vertex + edge sampling)
  - Sdf ↔ HeightField (grid point sampling)
  - Sdf ↔ Sdf (dual implicit surface sampling)

**Implementation files:**
- `sim-core/src/sdf.rs` - SDF types, queries, and all SDF collision functions

---

## 6. Joint Types

| Joint | MuJoCo | CortenForge | Status | Notes |
|-------|--------|-------------|--------|-------|
| Fixed (weld) | Yes | `FixedJoint` | **Implemented** | |
| Hinge (revolute) | Yes | `RevoluteJoint` | **Implemented** | |
| Slide (prismatic) | Yes | `PrismaticJoint` | **Implemented** | |
| Ball (spherical) | Yes | `SphericalJoint` | **Implemented** | |
| Universal | Yes | `UniversalJoint` | **Implemented** | |
| Free (6 DOF) | Yes | `FreeJoint` | **Implemented** | Full constraint solver support |
| Planar | Yes | `PlanarJoint` | **Standalone** | In sim-constraint; MJCF model builder errors (not in pipeline `MjJointType`) |
| Cylindrical | Yes | `CylindricalJoint` | **Standalone** | In sim-constraint; MJCF model builder errors (not in pipeline `MjJointType`) |

### Implementation Notes: Free/Planar/Cylindrical Joints ✅ COMPLETED (Free only; Planar/Cylindrical are Standalone)

FreeJoint is fully implemented in the pipeline. PlanarJoint and CylindricalJoint exist in sim-constraint but are not in the pipeline's `MjJointType` enum (MJCF model builder errors on them):

**FreeJoint (6 DOF floating bodies):**
- Used for floating-base robots (quadrupeds, humanoids, drones)
- Zero constraints (all 6 DOF free)
- Includes linear and angular damping
- Methods: `set_position()`, `set_rotation()`, `compute_damping_force()`

**PlanarJoint (3 DOF: x, y translation + rotation):**
- Used for mobile robots on flat surfaces
- 3 constraints: 1 translation (perpendicular to plane) + 2 rotation (tilt)
- Configurable plane normal
- Methods: `set_position()`, `set_angle()`, `translation()`, `rotation()`

**CylindricalJoint (2 DOF: rotation + translation along axis):**
- Combination of revolute + prismatic along same axis
- 4 constraints: 2 translation + 2 rotation (perpendicular to axis)
- Supports separate rotation and translation limits, motors, and damping
- Methods: `set_angle()`, `set_displacement()`, `compute_joint_forces()`

**Constraint solver support added to:**
- `ConstraintSolver` (Gauss-Seidel): `solve_free_constraint()`, `solve_planar_constraint()`, `solve_cylindrical_constraint()` (removed in Phase 3 consolidation — `ConstraintSolver` was in `solver.rs`; `BodyState`/`JointForce` extracted to `types.rs`)
- `NewtonConstraintSolver`: Jacobian and error computation for all three types (removed in Phase 3 consolidation — was in `newton.rs`)
- `CGSolver`: Jacobian and error computation for all three types (kept — `cg.rs`)
- MJCF loader: Updated to parse `cylindrical` and `planar` joint types

---

## 7. Actuators

| Actuator | MuJoCo | CortenForge | Status | Priority | Complexity |
|----------|--------|-------------|--------|----------|------------|
| Motor (direct torque) | Yes | `mj_fwd_actuation()` (gain/bias: `force = 1.0 * ctrl`) | **Implemented** | - | - |
| Position servo | Yes | `mj_fwd_actuation()` (gain/bias: `force = kp*input - kp*length - kv*velocity`; FilterExact dynamics when `timeconst > 0`) | **Implemented** ([future_work_1 #12](./todo/future_work_1.md) ✅) | - | - |
| Velocity servo | Yes | `mj_fwd_actuation()` (gain/bias: `force = kv*ctrl - kv*velocity`) | **Implemented** ([future_work_1 #12](./todo/future_work_1.md) ✅) | - | - |
| PD control | Yes | Position servo with `kp` + `kv` (equivalent to PD control) | **Implemented** (via Position actuator with explicit `kv`) | - | - |
| Integrated velocity | Yes | `IntegratedVelocityActuator` | **Standalone** (in sim-constraint, not in pipeline) | - | - |
| Damper | Yes | `mj_fwd_actuation()` (gain/bias: Affine gain `= -kv*velocity`, `force = gain * ctrl`; `ctrllimited` enforced) | **Implemented** ([future_work_1 #12](./todo/future_work_1.md) ✅) | - | - |
| Cylinder (pneumatic) | Yes | `mj_fwd_actuation()` (gain/bias: `force = area*act + bias[0] + bias[1]*length + bias[2]*velocity`; Filter dynamics with `timeconst`) | **Implemented** ([future_work_1 #12](./todo/future_work_1.md) ✅) | - | - |
| Muscle (MuJoCo FLV) | Yes | MuJoCo FLV in pipeline + `HillMuscle` (standalone via sim-muscle) | **Implemented** (MuJoCo-compatible FLV curves, activation dynamics, act_dot architecture; [future_work_1 #5](./todo/future_work_1.md) ✅) | - | - |
| Adhesion | Yes | `mj_fwd_actuation()` (gain/bias: `force = gain * ctrl`; `ctrllimited` enforced) | **Implemented** ([future_work_1 #12](./todo/future_work_1.md) ✅) | - | - |
| General (custom) | Yes | MJCF `gaintype`/`biastype`/`dyntype`/`gainprm`/`biasprm`/`dynprm` parsed on `<general>`, wired through defaults and model builder to runtime dispatch | **Implemented** ([future_work_3 #8](./todo/future_work_3.md) ✅) | - | - |

> **Pipeline vs standalone actuators.** The MuJoCo pipeline's `mj_fwd_actuation()` implements a 3-phase architecture: (1) compute `act_dot` per `ActuatorDynamics` type (None → ctrl passthrough, Muscle → Millard activation dynamics, Filter/FilterExact → first-order filter, Integrator → ctrl), (2) compute force via general gain/bias formula `force = gain * input + bias` dispatched on `GainType`/`BiasType` (Fixed, Affine, Muscle), (3) map force to generalized coordinates via transmission J^T. All 8 MJCF shortcut actuator types (Motor, Position, Velocity, Damper, Cylinder, Adhesion, Muscle, General) are expanded to their general gain/bias/dynamics representation in the model builder, matching MuJoCo's `user_api.cc`. Control clamping (`ctrlrange`, gated by `ctrllimited`) and force clamping (`forcerange`, gated by `forcelimited`) are enforced. Damper and Adhesion actuators force `ctrllimited=true`. `data.act` is integrated by the Euler/RK4 integrator using `act_dot` (activation is never modified inside `mj_fwd_actuation` — matching MuJoCo's `mjData.act_dot` convention). FilterExact uses exact discrete integration `act += act_dot * tau * (1 - exp(-h/tau))` while Filter uses Euler. `ActuatorTransmission::Site` is fully implemented with 6D gear vector, refsite (Mode A/B), `mj_jac_site()`, `mj_transmission_site()`, `subquat()`, and common-ancestor zeroing ([future_work_2 §5](./todo/future_work_2.md) ✅). The sim-muscle crate (2,550 lines) provides a richer Hill-type model as a standalone alternative; sim-constraint actuators (`JointMotor`, `IntegratedVelocityActuator`, `PneumaticCylinderActuator`, `AdhesionActuator`, `CustomActuator`) remain standalone.

### Implementation Notes: Muscle Model ✅ COMPLETED (pipeline + standalone)

MuJoCo's muscle model includes:
- Activation dynamics (1st-order system)
- Force-length-velocity relationships
- Pennation angle

**Pipeline implementation (MuJoCo FLV model in `mujoco_pipeline.rs`):**

MuJoCo's simplified FLV muscle model is implemented directly in the pipeline,
matching `mju_muscleGain`, `mju_muscleBias`, and `mju_muscleDynamics` from
`engine_util_misc.c`. Key components:

- `muscle_gain_length()` — piecewise-quadratic active force-length curve
- `muscle_gain_velocity()` — piecewise-quadratic force-velocity curve
- `muscle_passive_force()` — passive force (quadratic onset, linear beyond midpoint)
- `muscle_activation_dynamics()` — Millard et al. (2013) with activation-dependent
  time constants: `τ_act_eff = τ_act * (0.5 + 1.5*act)`,
  `τ_deact_eff = τ_deact / (0.5 + 1.5*act)`
- `sigmoid()` — quintic C2-continuous smoothstep for optional smooth blending
- `compute_muscle_params()` — build-time computation of `lengthrange`, `acc0`,
  and auto-resolved `F0 = scale / acc0`

Model fields: `actuator_dynprm[3]`, `actuator_gainprm[9]`, `actuator_biasprm[9]`,
`actuator_lengthrange`, `actuator_acc0`. Data fields: `actuator_length`,
`actuator_velocity`, `actuator_force`, `act_dot`, RK4 scratch buffers.

17 tests covering all 15 acceptance criteria (see [future_work_1 #5](./todo/future_work_1.md)).

**Standalone implementation in `sim-muscle` crate (richer Hill-type model):**

Created a comprehensive Hill-type muscle-tendon unit (MTU) model for biomechanical simulation:

**Activation Dynamics (`activation.rs`):**
- First-order excitation-to-activation filter with asymmetric time constants
- Faster activation (τ_act ≈ 10-20 ms) than deactivation (τ_deact ≈ 40-80 ms)
- Presets for fast-twitch (`ActivationDynamics::fast_twitch()`) and slow-twitch (`ActivationDynamics::slow_twitch()`)
- Semi-implicit Euler integration for unconditional stability

**Force Curves (`curves.rs`):**
- `ActiveForceLengthCurve` - Bell-shaped curve centered at optimal fiber length
- `PassiveForceLengthCurve` - Exponential rise at long lengths (connective tissue)
- `ForceVelocityCurve` - Hill hyperbolic relationship for concentric/eccentric
- `MuscleForceCurves` - Combined evaluation of all three relationships

**Hill Muscle Model (`hill.rs`):**
- `HillMuscle` - Complete muscle-tendon unit with state management
- `HillMuscleConfig` - Configurable parameters (F_max, L_opt, L_slack, etc.)
- Pennation angle effects with constant-width assumption
- Rigid tendon (fast) and compliant tendon (accurate) modes
- Predefined configurations: `biceps()`, `quadriceps()`, `gastrocnemius()`, `soleus()`

**Kinematics (`kinematics.rs`):**
- `ConstantMomentArm` - Fixed lever arm
- `PolynomialMomentArm` - r(θ) as polynomial function
- `SplineMomentArm` - Interpolation from measured data
- `BiarticularMuscleConfig` - Two-joint muscles (e.g., rectus femoris)
- `MusclePath` - Via points and wrapping support (geometry only)

**`MuscleActuator` trait** (in sim-muscle `lib.rs`, not sim-constraint):
- Common interface for muscle models

**Integration with sim-constraint (optional `muscle` feature):**
- `MuscleJoint` - RevoluteJoint with muscle group actuation
- `MuscleJointBuilder` - Builder pattern for agonist/antagonist pairs
- `MuscleCommands` - Command storage for RL interfaces

**Usage:**
```rust
use sim_muscle::{HillMuscle, HillMuscleConfig, MuscleGroup};
use sim_constraint::{MuscleJoint, RevoluteJoint}; // with "muscle" feature

// Create biceps muscle
let biceps = HillMuscle::new(HillMuscleConfig::biceps())
    .with_name("biceps_brachii");

// Create triceps muscle
let triceps = HillMuscle::new(HillMuscleConfig::default())
    .with_name("triceps_brachii");

// Build muscle group (agonist/antagonist pair)
let muscles = MuscleGroup::new()
    .with_flexor(biceps)
    .with_extensor(triceps);

// Attach to joint
let base_joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());
let mut elbow = MuscleJoint::new(base_joint, muscles);

// Control via excitation (0-1)
elbow.set_muscle_excitation(0, 0.8);  // Activate biceps
elbow.set_muscle_excitation(1, 0.2);  // Partial triceps

// Compute joint torque
let torque = elbow.compute_joint_force(velocity, dt);
```

**Files:**
- `sim-muscle/src/activation.rs` - Activation dynamics
- `sim-muscle/src/curves.rs` - Force-length-velocity relationships
- `sim-muscle/src/hill.rs` - Hill muscle model
- `sim-muscle/src/kinematics.rs` - Moment arm models
- `sim-muscle/src/lib.rs` - Crate root with MuscleActuator trait
- `sim-constraint/src/muscle.rs` - Joint integration (requires `muscle` feature)

**New crate:** `sim-muscle/`

---

## 8. Sensors

| Sensor | MuJoCo | CortenForge | Status | Priority |
|--------|--------|-------------|--------|----------|
| Joint position | Yes | `MjSensorType::JointPos` | **Implemented** (hinge/slide scalar) | - |
| Joint velocity | Yes | `MjSensorType::JointVel` | **Implemented** (hinge/slide scalar) | - |
| Ball joint quaternion | Yes | `MjSensorType::BallQuat` | **Implemented** (4D normalized quaternion) | - |
| Ball joint angular velocity | Yes | `MjSensorType::BallAngVel` | **Implemented** (3D angular velocity) | - |
| Body position/rotation | Yes | `MjSensorType::FramePos` | **Implemented** | - |
| Body velocity | Yes | `MjSensorType::FrameLinVel` | **Implemented** | - |
| Accelerometer | Yes | `MjSensorType::Accelerometer` | **Implemented** | - |
| Gyro | Yes | `MjSensorType::Gyro` | **Implemented** | - |
| IMU (combined) | Yes | Accelerometer + Gyro | **Implemented** | - |
| Velocimeter | Yes | `MjSensorType::Velocimeter` | **Implemented** | - |
| FrameQuat | Yes | `MjSensorType::FrameQuat` | **Implemented** | - |
| FrameXAxis / YAxis / ZAxis | Yes | `MjSensorType::FrameXAxis` etc. | **Implemented** | - |
| FrameAngVel | Yes | `MjSensorType::FrameAngVel` | **Implemented** | - |
| FrameLinAcc | Yes | `MjSensorType::FrameLinAcc` | **Implemented** | - |
| FrameAngAcc | Yes | `MjSensorType::FrameAngAcc` | **Implemented** | - |
| SubtreeCom | Yes | `MjSensorType::SubtreeCom` | **Implemented** | - |
| SubtreeLinVel | Yes | `MjSensorType::SubtreeLinVel` | **Implemented** | - |
| ActuatorFrc | Yes | `MjSensorType::ActuatorFrc` | **Implemented** | - |
| Force | Yes | `MjSensorType::Force` | **Implemented** (inverse dynamics via `compute_site_force_torque()`) | - |
| Torque | Yes | `MjSensorType::Torque` | **Implemented** (inverse dynamics via `compute_site_force_torque()`) | - |
| Touch | Yes | `MjSensorType::Touch` | **Implemented** (sums `efc_lambda` normal forces on attached geom) | - |
| Rangefinder | Yes | `MjSensorType::Rangefinder` | **Implemented** (ray-cast along +Z with mesh support) | - |
| Magnetometer | Yes | `MjSensorType::Magnetometer` | **Implemented** (`model.magnetic` transformed to sensor frame) | - |
| ActuatorPos / ActuatorVel | Yes | `MjSensorType::ActuatorPos/Vel` | **Implemented** (joint, tendon, and site transmissions) | - |
| SubtreeAngMom | Yes | `MjSensorType::SubtreeAngMom` | **Implemented** (subtree angular momentum) | - |
| TendonPos / TendonVel | Yes | `MjSensorType::TendonPos/Vel` | ✅ **Implemented** (reads `ten_length`/`ten_velocity`; [future_work_1 #4](./todo/future_work_1.md)) | - |
| Camera (rendered) | Yes | Out of scope | N/A | - |

> **Two sensor systems exist.** The sim-sensor crate has standalone `Imu`, `ForceTorqueSensor`, `TouchSensor`, `Rangefinder`, and `Magnetometer` implementations that operate on `RigidBodyState` objects. The MuJoCo pipeline has its own sensor readout in `mj_sensor_pos()`/`mj_sensor_vel()`/`mj_sensor_acc()` within `mujoco_pipeline.rs`. All 32 pipeline sensor types are functional and fully wired from MJCF `<sensor>` elements via `process_sensors()` in `model_builder.rs` ([future_work_1 #6](./todo/future_work_1.md)). The MJCF parser recognizes 32 `MjcfSensorType` variants; all 32 map to pipeline `MjSensorType`. JointLimitFrc and TendonLimitFrc read cached penalty force magnitudes from `mj_fwd_constraint()`. `set_options()` propagates `magnetic`, `wind`, `density`, and `viscosity` from MJCF `<option>`. Magnetometer is evaluated in the Position stage (depends only on `site_xmat` from FK). ActuatorVel reads from pre-computed `data.actuator_velocity`.

### Implementation Notes: Sensors ✅ COMPLETED (both standalone sim-sensor crate and pipeline sensors)

Created `sim-sensor` crate with:
- `Imu` - Accelerometer + gyroscope combined sensor with configurable noise, bias, and gravity
- `ForceTorqueSensor` - 6-axis force/torque measurement with saturation and deadband
- `TouchSensor` - Contact detection with force thresholds and body filtering
- `SensorObservation` in sim-types for integration with observation system

**Example usage:**
```rust
use sim_sensor::{Imu, ImuConfig, TouchSensor, TouchSensorConfig};
use sim_types::{BodyId, ContactInfo, RigidBodyState, Pose};

// Create an IMU on body 1
let imu = Imu::new(BodyId::new(1), ImuConfig::default());
let state = RigidBodyState::at_rest(Pose::identity());
let reading = imu.read_from_state(&state, 0.001);

// Create a touch sensor
let touch = TouchSensor::new(BodyId::new(1), TouchSensorConfig::default());
let contacts: Vec<ContactInfo> = vec![];
let obs = touch.read_as_observation(&contacts, 0.001);
```

**Files:** `sim-sensor/src/imu.rs`, `sim-sensor/src/force_torque.rs`, `sim-sensor/src/touch.rs`, `sim-sensor/src/rangefinder.rs`, `sim-sensor/src/magnetometer.rs`

**New crate:** `sim-sensor/`

---

## 9. Tendons

| Feature | MuJoCo | CortenForge | Status | Priority | Complexity |
|---------|--------|-------------|--------|----------|------------|
| Fixed tendons | Yes | `FixedTendon` | ✅ **Pipeline** — `mj_fwd_tendon_fixed()`, full kinematics/actuation/passive/constraints | - | - |
| Spatial tendons | Yes | `SpatialTendon` | ✅ **Pipeline** — `mj_fwd_tendon_spatial()`, 3D routing, Jacobian via `accumulate_point_jacobian()`, passive/constraints/actuation | - | - |
| Wrapping (sphere/cylinder) | Yes | `sphere_wrap`, `cylinder_wrap` | ✅ **Pipeline** — dual-candidate tangent selection, sidesite disambiguation, helical arc length | - | - |
| Pulley systems | Yes | Pulley divisor | ✅ **Pipeline** — `WrapType::Pulley` pair-skip, length/Jacobian scaling by divisor | - | - |

> **Both fixed and spatial tendons are fully integrated into the pipeline.** The model builder (`process_tendons()`) converts MJCF `<tendon><fixed>` and `<tendon><spatial>` elements into pipeline Model arrays. `mj_fwd_tendon()` computes tendon lengths and Jacobians in `mj_fwd_position()` (fixed via linear coupling, spatial via 3D pairwise routing with `accumulate_point_jacobian()`), tendon velocities in `mj_fwd_velocity()`, passive forces (spring/damper/friction) in `mj_fwd_passive()`, limit constraints in `mj_fwd_constraint()`, and actuation via J^T mapping in `mj_fwd_actuation()`. Spatial tendons support sphere and cylinder wrapping (`sphere_wrap`, `cylinder_wrap`), sidesite disambiguation, pulley divisors, and free-joint Jacobians. All tendon sensor types (TendonPos/TendonVel/ActuatorPos/ActuatorVel) read live tendon data. See [future_work_1 #4](./todo/future_work_1.md), [future_work_2 #4](./todo/future_work_2.md).

### Implementation Notes: Tendons ✅ COMPLETED (fixed + spatial tendons in pipeline; sim-tendon crate standalone)

Created `sim-tendon` crate with comprehensive tendon/cable modeling for cable-driven robots and biomechanics:

**Fixed Tendons (`fixed.rs`) - MuJoCo-style:**
- `FixedTendon` - Linear coupling of joints: `L = L₀ + Σᵢ cᵢ qᵢ`
- `TendonCoefficient` - Moment arm coupling coefficients
- `FixedTendonGroup` - Multiple tendons controlled together
- Presets: `differential()`, `parallel()` for common configurations
- Support for range limits with soft limit forces

**Spatial Tendons (`spatial.rs`):**
- `SpatialTendon` - 3D cable routing through body attachment points
- `SpatialTendonConfig` - Material properties, friction, range limits
- Integration with wrapping geometries
- Automatic force computation on connected bodies

**Tendon Path (`path.rs`):**
- `TendonPath` - Origin → via points → insertion path representation
- `AttachmentPoint` - Attachment to bodies with local position and tangent
- `TendonSegment` - Cached segment length and direction
- Force distribution computation for body forces/torques

**Cable Properties (`cable.rs`):**
- `CableProperties` - Stiffness, damping, rest length, max tension
- One-way spring model (cables can only pull, not push)
- Presets: `steel_cable()`, `dyneema_cable()`, `biological_tendon()`, `soft_cable()`
- `CableState` - Current length, velocity, tension, slack status

**Wrapping Geometry (`wrapping.rs`):**
- `SphereWrap` - Tendon wrapping over spherical surfaces (joints)
- `CylinderWrap` - Tendon wrapping around cylindrical surfaces (pulleys)
- `WrappingGeometry` - Enum for both types with transform support
- `WrapResult` - Tangent points, arc length, wrap normal

**Pulley Systems (`pulley.rs`):**
- `Pulley` - Fixed or moving pulley with friction
- `PulleySystem` - Multiple pulleys with mechanical advantage
- `PulleyConfig` - Radius, bearing friction, wrap friction (Capstan effect)
- `PulleyBuilder` - Presets for block-and-tackle, compound systems

**Usage:**
```rust
use sim_tendon::{FixedTendon, SpatialTendon, TendonActuator, CableProperties};
use sim_tendon::path::{TendonPath, AttachmentPoint};
use sim_tendon::pulley::{PulleySystem, PulleyBuilder};
use sim_types::{JointId, BodyId};
use nalgebra::Point3;

// Fixed tendon (MuJoCo-style joint coupling)
let tendon = FixedTendon::new("differential")
    .with_coefficient(JointId::new(0), 0.05)   // 5cm moment arm
    .with_coefficient(JointId::new(1), -0.05)  // Opposite direction
    .with_rest_length(0.5)
    .with_cable(CableProperties::steel_cable(0.002));

let force = tendon.compute_force(&[0.5, -0.5], &[0.0, 0.0]);

// Spatial tendon (3D cable routing)
let path = TendonPath::straight(
    BodyId::new(0), Point3::new(0.0, 0.0, 0.1),
    BodyId::new(1), Point3::new(0.0, 0.0, -0.1),
);
let mut spatial = SpatialTendon::new("biceps", path)
    .with_cable(CableProperties::biological_tendon(20e-6));

// Pulley system (2:1 mechanical advantage)
let pulley = PulleyBuilder::block_and_tackle_2_1(
    Point3::new(0.0, 0.0, 2.0),
    BodyId::new(1),
    Point3::origin(),
    0.05,
);
```

**Files:**
- `sim-tendon/src/lib.rs` - Crate root with `TendonActuator` trait
- `sim-tendon/src/cable.rs` - Cable material properties
- `sim-tendon/src/fixed.rs` - MuJoCo-style fixed tendons
- `sim-tendon/src/spatial.rs` - 3D spatial tendons
- `sim-tendon/src/path.rs` - Tendon path geometry
- `sim-tendon/src/wrapping.rs` - Sphere/cylinder wrapping
- `sim-tendon/src/pulley.rs` - Pulley systems
- `sim-tendon/src/error.rs` - Error types

**New crate:** `sim-tendon/`

---

## 10. Equality Constraints

| Constraint | MuJoCo | CortenForge | Status | Priority |
|------------|--------|-------------|--------|----------|
| Connect (ball) | Yes | Pipeline `EqualityType::Connect` + `apply_connect_constraint()` | **Implemented** (in pipeline via penalty forces; standalone `ConnectConstraint` in sim-constraint is unused) | - |
| Weld | Yes | Pipeline `EqualityType::Weld` + `apply_weld_constraint()` | **Implemented** (in pipeline; standalone `WeldConstraint` in sim-constraint is unused) | - |
| Distance | Yes | Pipeline `EqualityType::Distance` + `apply_distance_constraint()` | **Implemented** (in pipeline; standalone `DistanceConstraint` in sim-constraint is unused) | - |
| Joint coupling | Yes | Pipeline `EqualityType::Joint` + `apply_joint_equality_constraint()` | **Implemented** (in pipeline; standalone `JointCoupling`/`GearCoupling`/`DifferentialCoupling` in sim-constraint are unused) | - |
| Tendon coupling | Yes | `TendonConstraint`, `TendonNetwork` | **Standalone** (in sim-constraint; pipeline uses `EqualityType::Tendon` warning — tendon *equality* constraints not yet implemented) | - |
| Flex (edge length) | Yes | `FlexEdgeConstraint` | ✅ **Pipeline** (XPBD solver called from `mj_deformable_step()` in `integrate()` and `mj_runge_kutta()`; see [future_work_4 #11](./todo/future_work_4.md) ✅) | - |

### Implementation Notes: Connect (Ball) Constraint ✅ COMPLETED

The connect (ball) constraint enforces that two attachment points (one on each body) coincide in 3D space. It acts like a ball-and-socket joint without any rotational constraints - both bodies can rotate freely around the connection point.

**Constraint Formulation:**
```
p1 + R1 * anchor - p2 = 0
```

Where:
- `p1`, `p2` are the body positions
- `R1` is the rotation matrix of body1
- `anchor` is the local anchor point in body1's frame

This results in 3 scalar constraints (one per axis).

**Implementation:**
- `ConnectConstraint` - Ball constraint between two bodies or body-to-world
- `MjcfConnect` - MJCF representation for `<connect>` element parsing
- `MjcfEquality` - Container for equality constraints in MJCF model
- Full MJCF parser support for `<equality><connect>` elements
- Baumgarte stabilization for position correction
- Compliance and damping parameters for soft constraints
- Solver reference (`solref`) and impedance (`solimp`) parameters

**Usage:**
```rust
use sim_constraint::ConnectConstraint;
use sim_types::BodyId;
use nalgebra::Vector3;

// Connect body1's tip to body2's origin
let constraint = ConnectConstraint::new(
    BodyId::new(0),
    BodyId::new(1),
    Vector3::new(0.0, 0.0, 1.0),  // anchor at body1's +Z tip
);

// Connect body to world (fixed point constraint)
let world_constraint = ConnectConstraint::to_world(
    BodyId::new(0),
    Vector3::new(0.0, 0.0, 0.0),
);

// Via MJCF
let mjcf = r#"
    <mujoco model="test">
        <worldbody>
            <body name="body1"/>
            <body name="body2"/>
        </worldbody>
        <equality>
            <connect name="ball" body1="body1" body2="body2" anchor="0.5 0 0"/>
        </equality>
    </mujoco>
"#;
let model = sim_mjcf::parse_mjcf_str(mjcf).expect("should parse");
// model.equality.connects contains the parsed constraints
```

**Files:**
- `sim-constraint/src/equality.rs` - `ConnectConstraint` implementation
- `sim-mjcf/src/types.rs` - `MjcfConnect`, `MjcfEquality` types
- `sim-mjcf/src/parser.rs` - `<equality>` and `<connect>` element parsing
- `sim-mjcf/src/model_builder.rs` - Conversion to `ConnectConstraint`

---

## 11. Deformables (Flex)

| Feature | MuJoCo | CortenForge | Status | Priority | Complexity |
|---------|--------|-------------|--------|----------|------------|
| 1D (capsule chains) | Yes | `CapsuleChain` | ✅ **Pipeline** (XPBD + deformable-rigid contact) | - | - |
| 2D (triangle shells) | Yes | `Cloth` | ✅ **Pipeline** (XPBD + deformable-rigid contact) | - | - |
| 3D (tetrahedra) | Yes | `SoftBody` | ✅ **Pipeline** (XPBD + deformable-rigid contact) | - | - |
| Skinned meshes | Yes | `SkinnedMesh` | **Standalone** (visual deformation only, not in contact pipeline) | - | - |
| Deformable-rigid collision | Yes | `CollisionConstraint`, `DeformableContact` | ✅ **Implemented** (vertex-vs-geom narrowphase: plane/sphere/box/capsule/cylinder/ellipsoid) | - | - |

> ✅ **Pipeline-integrated via split-solve approach.** Deformable bodies registered via `Data::register_deformable()` participate in collision detection (`mj_deformable_collision()`) and contact resolution (`solve_deformable_contacts()`) with rigid geoms. XPBD internal constraints solved via `mj_deformable_step()` in both Euler and RK4 integrators. Feature-gated behind `#[cfg(feature = "deformable")]` for zero overhead when disabled. See [future_work_4 #11](./todo/future_work_4.md) ✅.

### Implementation Notes: Skinned Meshes ✅ COMPLETED (standalone only)

Skinned meshes provide visual deformation for rendering soft bodies:
- Vertex skinning with bone weights
- Linear blend skinning (LBS) or dual quaternion skinning (DQS)
- Maps physics particles to visual mesh vertices

**Implemented in `sim-deformable/src/skinning.rs`:**
- `Skeleton` - Hierarchical bone structure with world-space pose computation
- `Bone` - Individual bone with bind pose and local/world transforms
- `SkinnedMesh` - Mesh with per-vertex bone weights
- `BoneWeight` / `VertexWeights` - Per-vertex bone influence weights
- `SkinningMethod` - LBS (linear blend) and DQS (dual quaternion) algorithms
- `SkinnedMeshBuilder` - Convenient builder with distance-based weight assignment

**MJCF support in `sim-mjcf/src/parser.rs`:**
- `MjcfSkin` - Skin element from `<deformable>` section
- `MjcfSkinBone` - Bone references with bind poses
- `MjcfSkinVertex` - Per-vertex bone weights
- Parser support for `<skin>`, `<bone>`, and `<vertex>` elements
4. Export skinned vertex positions for rendering

### Implementation Notes: Deformables ✅ COMPLETED (pipeline-integrated)

Created `sim-deformable` crate using XPBD (Extended Position-Based Dynamics) for stable, physically-accurate soft body simulation:

**1D Deformables - Capsule Chains (`capsule_chain.rs`):**
- `CapsuleChain` - Rope/cable simulation with configurable segments
- Distance constraints for stretch resistance
- Bending constraints for flexibility control
- Presets: `rope()`, `steel_cable()`, `soft_cable()`, `hair()`, `chain()`
- Builder pattern: `CapsuleChain::new()` or `CapsuleChain::from_points()`

**2D Deformables - Cloth (`cloth.rs`):**
- `Cloth` - Triangle mesh cloth/membrane simulation
- Distance constraints along edges
- Dihedral bending constraints for folding resistance
- Grid cloth with `Cloth::grid()` and edge pinning
- Presets: `cotton()`, `silk()`, `leather()`, `rubber()`, `paper()`, `membrane()`
- Wind force simulation with `apply_wind()`

**3D Deformables - Soft Bodies (`soft_body.rs`):**
- `SoftBody` - Tetrahedral mesh volumetric simulation
- Distance constraints for edge stiffness
- Volume constraints for incompressibility
- Shape primitives: `SoftBody::cube()`, `SoftBody::sphere()`
- Presets: `rubber()`, `gelatin()`, `soft_tissue()`, `muscle()`, `foam()`
- Surface triangle extraction for rendering/collision

**XPBD Solver (`solver.rs`):**
- `XpbdSolver` - Position-based dynamics solver
- Unconditionally stable for any time step
- Configurable iterations and substeps
- Velocity damping and clamping
- Presets: `realtime()`, `accurate()`, `soft()`, `stiff()`

**Constraints (`constraints.rs`):**
- `DistanceConstraint` - Maintains distance between particles
- `BendingConstraint` - Maintains angles (chain or dihedral)
- `VolumeConstraint` - Preserves tetrahedron volume

**Material Model (`material.rs`):**
- `Material` - Young's modulus, Poisson's ratio, density
- Compliance computation for XPBD
- Presets: `Rubber`, `Cloth`, `SoftTissue`, `Muscle`, `Gelatin`, `Foam`, etc.

**Usage:**
```rust
use sim_deformable::{
    CapsuleChain, CapsuleChainConfig,
    Cloth, ClothConfig,
    SoftBody, SoftBodyConfig,
    XpbdSolver, SolverConfig,
    DeformableBody,
};
use nalgebra::{Point3, Vector3};

// 1D: Create a rope
let mut rope = CapsuleChain::new(
    "rope",
    Point3::new(0.0, 0.0, 2.0),
    Point3::new(5.0, 0.0, 2.0),
    20,
    CapsuleChainConfig::rope(0.01),
);
rope.pin_vertex(0);  // Fix start

// 2D: Create a cloth flag
let mut cloth = Cloth::grid(
    "flag",
    Point3::origin(),
    Vector3::new(2.0, 0.0, 0.0),
    Vector3::new(0.0, 0.0, -1.5),
    20, 15,
    ClothConfig::cotton(),
);
cloth.pin_edge("left");  // Pin left edge

// 3D: Create a soft cube
let mut jelly = SoftBody::cube(
    "jelly",
    Point3::new(0.0, 0.0, 1.0),
    0.3,
    2,  // subdivisions
    SoftBodyConfig::gelatin(),
);
jelly.pin_bottom(0.05);  // Pin bottom vertices

// Simulate
let mut solver = XpbdSolver::new(SolverConfig::default());
let gravity = Vector3::new(0.0, 0.0, -9.81);

for _ in 0..100 {
    solver.step(&mut rope, gravity, 1.0 / 60.0);
    solver.step(&mut cloth, gravity, 1.0 / 60.0);
    solver.step(&mut jelly, gravity, 1.0 / 60.0);
}
```

**Files:**
- `sim-deformable/src/lib.rs` - Crate root, `DeformableBody` trait
- `sim-deformable/src/types.rs` - `DeformableId`, `Vertex`, `VertexFlags`
- `sim-deformable/src/mesh.rs` - `Edge`, `Triangle`, `Tetrahedron`, `DeformableMesh`
- `sim-deformable/src/material.rs` - Material properties and presets
- `sim-deformable/src/constraints.rs` - XPBD constraints
- `sim-deformable/src/solver.rs` - `XpbdSolver`
- `sim-deformable/src/capsule_chain.rs` - 1D ropes/cables
- `sim-deformable/src/cloth.rs` - 2D cloth/membranes
- `sim-deformable/src/soft_body.rs` - 3D volumetric soft bodies
- `sim-deformable/src/error.rs` - Error types

**New crate:** `sim-deformable/`

---

## 12. Performance Optimizations

| Feature | MuJoCo | CortenForge | Status | Priority |
|---------|--------|-------------|--------|----------|
| Sparse matrix ops | Native | `SparseJacobian`, `JacobianBuilder` (was in `sparse.rs`) | **Removed** (Phase 3 consolidation) | - |
| Sleeping bodies | Native | — | **Not implemented** (removed with World/Stepper) | - |
| Constraint islands | Auto | `ConstraintIslands` (was in `islands.rs`) | **Removed** (Phase 3 consolidation) | - |
| **Multi-threading** | Model-data separation | `parallel` feature with rayon | **Active** — `BatchSim::step_all()` uses `par_iter_mut` for cross-environment parallelism (`batch.rs`); see [future_work_3 #9](./todo/future_work_3.md) | - |
| **GPU acceleration** | MuJoCo MJX (JAX) | `sim-gpu` crate (wgpu) | **Active (Phase 10a)** — `GpuBatchSim` offloads Euler velocity integration to GPU compute shader; full pipeline stays on CPU. See [future_work_3 #10](./todo/future_work_3.md) | - |
| SIMD | Likely | `sim-simd` crate | **Partial** (only `find_max_dot()` is used by sim-core GJK; all other batch ops have zero callers outside benchmarks) | - |

### Implementation Notes: SIMD Optimization ⚠️ PARTIAL (crate complete; only `find_max_dot` has production callers)

The `sim-simd` crate provides explicit SIMD vectorization for hot paths in the physics
simulation stack. It processes multiple vectors simultaneously using batch operations
optimized for modern CPU SIMD instructions (AVX/AVX2, AVX-512, NEON).

**Key Types:**
- `Vec3x4` - Process 4 `Vector3<f64>` values simultaneously (256-bit)
- `Vec3x8` - Process 8 `Vector3<f64>` values simultaneously (512-bit)

**Hot Paths Optimized:**

1. **GJK Support Vertex Search** (`sim-core/src/gjk_epa.rs`):
   - Batch dot products for finding support vertices in convex meshes
   - Uses `find_max_dot()` to process vertices in groups of 4-8
   - 2-4x speedup for meshes with >16 vertices

2. **Contact Force Computation** (removed — formerly `sim-contact/src/batch.rs`):
   - `BatchContactProcessor` has been removed (was in `sim-contact`)
   - Contact types (`ContactPoint`, `ContactManifold`, `ContactForce`) now live in `sim-core`

3. **AABB Overlap Testing** (`sim-simd/src/batch_ops.rs`):
   - `batch_aabb_overlap_4()` tests 4 AABB pairs at once
   - Useful for broad-phase collision detection

**Usage:**

```rust
use sim_simd::{Vec3x4, find_max_dot, batch_dot_product_4};
use nalgebra::Vector3;

// Batch dot products
let vectors = [
    Vector3::new(1.0, 0.0, 0.0),
    Vector3::new(0.0, 1.0, 0.0),
    Vector3::new(0.0, 0.0, 1.0),
    Vector3::new(1.0, 1.0, 1.0),
];
let direction = Vector3::new(1.0, 2.0, 3.0);
let dots = batch_dot_product_4(&vectors, &direction);

// Find support vertex (GJK)
let vertices = vec![/* many vertices */];
let (idx, max_dot) = find_max_dot(&vertices, &direction);

// Contact types now in sim-core
use sim_core::{ContactPoint, ContactManifold, ContactForce};
```

**Performance Notes:**
- On modern x86-64 (AVX2): expect 2-4x speedup for batch operations
- On Apple Silicon (NEON): expect 2-3x speedup
- Scalar fallback available when remainder doesn't fill a batch

**Files:** `sim-simd/src/`, `sim-core/src/gjk_epa.rs` (batch contact processor removed; contact types now in `sim-core`)

### Implementation Notes: Multi-threading ✅ ACTIVE (cross-environment parallelism via BatchSim)

> The island-parallel constraint solving described below was **deleted in Phase 3 consolidation** along with the Newton solver and `islands.rs`. The `parallel` feature and rayon dependency remain available in sim-core (`core/Cargo.toml:17,31`). The `parallel` feature is now used by `BatchSim::step_all()` (`batch.rs`) for cross-environment parallelism via `par_iter_mut`. See [future_work_3 #9](./todo/future_work_3.md).

The original `parallel` feature enabled multi-threaded constraint solving using rayon.
Before the Model/Data refactor, the original design used a snapshot-based approach
with island-parallel constraint solving.

**Key Design Decisions:**

1. **Disabled by default**: Parallel constraint solving is disabled by default
   (`ParallelConfig::default().parallel_constraints == false`) because it uses
   the Newton solver internally, which may produce slightly different results
   than the default `ConstraintSolver` (removed in Phase 3 consolidation — was in `solver.rs`). Enable explicitly for performance.

2. **Uses Newton solver**: The parallel implementation used `NewtonConstraintSolver`
   (removed in Phase 3 consolidation — was in `newton.rs`)
   rather than `ConstraintSolver` because Newton already had island-based solving
   infrastructure and provided fast convergence (2-3 iterations vs 8-16 for Gauss-Seidel).

3. **Body integration was sequential**: `integrate_bodies_parallel()` was
   actually sequential because hashbrown's `HashMap` does not support `par_iter_mut()`.
   The Model/Data refactor now uses `Vec`-based storage, making parallel integration feasible.

4. **Minimum thresholds**: Parallel solving only activates when there are at least
   `min_islands_for_parallel` independent islands (default: 2), avoiding rayon overhead
   for simple single-island scenes like a single articulated robot.

**Key types and methods:**

- `ParallelConfig` in `sim-types/src/config.rs` - Configuration for parallel thresholds
- `sim_constraint::parallel::solve_islands_parallel()` — removed in Phase 3 consolidation
- `NewtonConstraintSolver::solve_islands_parallel()` — removed in Phase 3 consolidation

**Usage:**

```rust
// Enable parallel (disabled by default, must opt-in)
let mut config = SimulationConfig::default();
config.solver.parallel.parallel_constraints = true;
config.solver.parallel.min_islands_for_parallel = 2;

// Or use the preset for multi-island scenes
config.solver.parallel = ParallelConfig::many_islands();

// Note: Island-parallel constraint solving was removed in Phase 3.
// The parallel feature is now used by BatchSim::step_all() for
// cross-environment parallelism (see batch.rs).
```

**Performance notes:**

- Parallel constraint solving benefits scenes with 2+ independent islands
- Falls back to sequential for small scenes to avoid parallel overhead
- Use `ParallelConfig::sequential()` for deterministic results matching default solver

**Files:** `sim-types/src/config.rs` (ParallelConfig remains), removed: `sim-constraint/src/parallel.rs`, `sim-constraint/src/newton.rs`

### Implementation Notes: GPU Acceleration ✅ ACTIVE (Phase 10a — Euler velocity integration)

The `sim-gpu` crate (`sim/L0/gpu/`) provides GPU-accelerated batched simulation
via wgpu compute shaders. `GpuBatchSim` is a drop-in replacement for `BatchSim`
with the same API surface (step_all, reset, env/env_mut, len, model).

**Phase 10a scope:** Only the Euler velocity integration (`qvel += qacc * h`)
runs on GPU. All other pipeline stages (FK, collision, constraints, dynamics)
remain on CPU. This validates the full GPU infrastructure with minimal physics
complexity.

**Architecture:**
- `GpuSimContext` — `OnceLock<Option<>>` singleton for wgpu device/queue (follows `mesh-gpu` pattern)
- `GpuEnvBuffers` — Structure-of-Arrays f32 GPU buffers (qvel, qacc, staging) with overflow-safe size validation
- `GpuParams` — `#[repr(C)]` Pod+Zeroable uniform buffer matching WGSL struct layout (16 bytes)
- `euler_integrate.wgsl` — compute shader with pipeline-overridable `wg_size` and strided loop for nv > wg_size

**5-step pipeline per timestep:**
1. CPU: `data.forward(&model)` for each env (rayon-parallel with `parallel` feature)
2. CPU→GPU: Upload qvel + qacc as f32 SoA buffers (`queue.write_buffer`)
3. GPU: `euler_integrate` compute shader (one workgroup per env, one thread per DOF)
4. GPU→CPU: Staging buffer readback via `map_async` + `device.poll(Wait)`
5. CPU: Activation integration + position integration + time advance (`integrate_without_velocity`)

**sim-core integration:** Three feature-gated helpers exposed behind `gpu-internals`:
- `Data::integrate_without_velocity()` — skips velocity update (done on GPU)
- `BatchSim::envs_as_mut_slice()` — for rayon `par_iter_mut` in GPU forward pass
- `BatchSim::model_arc()` — for `Arc::clone` in parallel forward pass

**Fallback:** `GpuBatchSim::try_new()` returns `Ok(None)` when no GPU is available;
other errors (`UnsupportedIntegrator`, `BatchTooLarge`) are propagated.

**Remaining phases (10b–10f):** FK, collision broad/narrow phase, constraint solver,
full GPU step. See [future_work_3 #10](./todo/future_work_3.md).

**Files:** `sim-gpu/src/` (pipeline.rs, context.rs, buffers.rs, error.rs, shaders/euler_integrate.wgsl), `sim-gpu/tests/integration.rs`, `sim-gpu/benches/gpu_benchmarks.rs`

### Implementation Notes: Sleeping Bodies — REMOVED

> **Note:** Sleeping (deactivation) was implemented in the old World/Stepper architecture
> (`Body::is_sleeping`, `put_to_sleep()`, `wake_up()`). It was **removed** along with
> the `World` and `Stepper` types during the Model/Data refactor. Re-implementing sleeping
> in the MuJoCo pipeline has not yet been prioritized.

---

## 13. Model Format

| Feature | MuJoCo | CortenForge | Status | Priority |
|---------|--------|-------------|--------|----------|
| URDF loading | Supported | `sim-urdf` crate | **Implemented** | - |
| MJCF loading | Native | `sim-mjcf` crate | **Implemented** | - |
| MJB (binary) | Native | `sim-mjcf` crate (mjb feature) | **Implemented** | - |

### Implementation Notes: MJB Binary Format ✅ COMPLETED

MJB is a binary serialization format for MJCF models providing:
- Faster loading than XML parsing
- Pre-serialized model data ready for deserialization
- Reduced file sizes through binary encoding

**Implementation:**
- `save_mjb_file()` / `save_mjb_bytes()` / `save_mjb_writer()` - Serialize `MjcfModel` to binary
- `load_mjb_file()` / `load_mjb_bytes()` / `load_mjb_reader()` - Deserialize from binary
- `is_mjb_file()` / `is_mjb_bytes()` - Check if data is valid MJB format
- `MjbHeader` - File header with magic bytes, version, and flags

**File Format:**
1. Magic bytes: `MJB1` (4 bytes)
2. Version: `u32` little-endian (4 bytes) - currently version 1
3. Flags: `u32` little-endian (4 bytes) - reserved for future use
4. Payload: bincode-encoded `MjcfModel` data

**Usage:**
```rust
use sim_mjcf::{parse_mjcf_str, load_mjb_file, save_mjb_file};

// Parse MJCF and save to binary for faster loading later
let model = parse_mjcf_str("<mujoco><worldbody/></mujoco>").unwrap();
save_mjb_file(&model, "model.mjb").unwrap();

// Load from binary (much faster than XML parsing)
let loaded = load_mjb_file("model.mjb").unwrap();
```

**Feature Flag:** Requires `mjb` feature to be enabled:
```toml
[dependencies]
sim-mjcf = { workspace = true, features = ["mjb"] }
```

**Files:** `sim-mjcf/src/mjb.rs`, `sim-mjcf/src/error.rs`

### Implementation Notes: MJCF Support ✅ COMPLETED

Created `sim-mjcf` crate for MuJoCo XML format compatibility.

**Supported Elements:**

| Element | Support | Notes |
|---------|---------|-------|
| `<mujoco>` | Full | Root element, model name |
| `<option>` | Full | All attributes, flags, solver params, collision options |
| `<default>` | Full | `DefaultResolver` wired into `model_builder.rs` — class defaults applied to joints, geoms, sites, actuators, tendons, sensors before per-element attributes |
| `<worldbody>` | Full | Body tree root |
| `<body>` | Full | Hierarchical bodies with pos, quat, euler |
| `<inertial>` | Full | mass, diaginertia, fullinertia |
| `<joint>` | Full | hinge, slide, ball, free types |
| `<geom>` | Full | sphere, box, capsule, cylinder, ellipsoid, plane, mesh (convex + non-convex), hfield, sdf |
| `<site>` | Parsed | Markers (not used in physics) |
| `<actuator>` | Full | motor, position, velocity, cylinder, muscle, adhesion, damper, general |
| `<tendon>` | Full | Fixed + spatial tendons fully wired into pipeline via `process_tendons()`; spatial tendons include sphere/cylinder wrapping, sidesite, pulley |
| `<sensor>` | Full | 32 sensor types parsed (all MuJoCo types); all 32 wired into pipeline via `process_sensors()` |
| `<contact>` | Full | `<pair>` (explicit geom pairs with per-pair condim/friction/solref/solimp overrides), `<exclude>` (body-pair exclusions), contype/conaffinity bitmasks; two-mechanism collision architecture |

**Supported Joint Types:**
- `hinge` → `RevoluteJoint` (1 DOF rotation)
- `slide` → `PrismaticJoint` (1 DOF translation)
- `ball` → `SphericalJoint` (3 DOF rotation)
- `free` → `FreeJoint` (6 DOF)

**Supported Geom Types:**
- `sphere` → `CollisionShape::Sphere`
- `box` → `CollisionShape::Box`
- `capsule` → `CollisionShape::Capsule`
- `cylinder` → `CollisionShape::Cylinder`
- `ellipsoid` → `CollisionShape::Ellipsoid`
- `plane` → `CollisionShape::Plane`
- `mesh` → `CollisionShape::ConvexMesh` (convex hull) or `CollisionShape::TriangleMesh` (non-convex)
- `hfield` → `GeomType::Hfield`, fully wired to `collide_with_hfield()` pipeline
- `sdf` → `GeomType::Sdf`, fully wired to `collide_with_sdf()` pipeline (programmatic data)

**Usage:**
```rust
use sim_mjcf::{parse_mjcf_str, load_model};
use sim_core::Model;

// Parse MJCF to get a MjcfModel (intermediate representation)
let mjcf = r#"
    <mujoco model="robot">
        <worldbody>
            <body name="base" pos="0 0 1">
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
"#;
let mjcf_model = parse_mjcf_str(mjcf).expect("should parse");

// Or load directly into a sim-core Model (for simulation)
let model: Model = load_model(mjcf).expect("should load");
let mut data = model.make_data();
data.step(&model).expect("should step");
```

**Limitations:**
- Composite bodies not supported
- Include files not supported
- Textures and materials parsed but not loaded (meshes are loaded)

**Files:** `sim-mjcf/src/lib.rs`, `parser.rs`, `types.rs`, `model_builder.rs`, `validation.rs`, `config.rs`

### Implementation Notes: MJCF `<option>` Element ✅ COMPLETED

Full support for MuJoCo's `<option>` element with all simulation configuration:

**Solver Configuration:**
- `timestep` - Simulation time step (default: 0.002)
- `integrator` - Euler, RK4, implicit, implicitfast
- `solver` - PGS, CG, Newton solver types
- `iterations` - Solver iterations (default: 100)
- `tolerance` - Solver convergence tolerance
- `ls_iterations` - Line search iterations for CG/Newton
- `noslip_iterations` - No-slip solver iterations
- `ccd_iterations` - Continuous collision detection iterations

**Contact Model:**
- `cone` - Friction cone type (pyramidal, elliptic)
- `jacobian` - Jacobian type (dense, sparse, auto)
- `impratio` - Friction-to-normal impedance ratio
- `nconmax` - Maximum contacts (0 = unlimited)
- `njmax` - Maximum constraint rows

**Physics Environment:**
- `gravity` - 3D gravity vector (default: 0 0 -9.81)
- `wind` - Wind velocity for aerodynamic effects
- `magnetic` - Magnetic field direction
- `density` - Medium density for drag
- `viscosity` - Medium viscosity

**Override Parameters:**
- `o_margin` - Global contact margin override
- `o_solimp` - Global solimp override [5 values]
- `o_solref` - Global solref override [2 values]
- `o_friction` - Global friction override [5 values]

**Flags (`<flag>` child element):**
All 20 MuJoCo flags supported:
- `constraint`, `equality`, `frictionloss`, `limit`, `contact`
- `passive`, `gravity`, `clampctrl`, `warmstart`, `filterparent`
- `actuation`, `refsafe`, `sensor`, `midphase`, `eulerdamp`
- `override`, `energy`, `multiccd`, `island`, `nativeccd`

**Configuration Types:**
- `MjcfOption` - Complete option parsing with defaults
- `MjcfFlag` - All 20 boolean flags
- `MjcfIntegrator` - Euler, RK4, ImplicitSpringDamper, ImplicitFast
- `MjcfConeType` - Pyramidal, Elliptic
- `MjcfSolverType` - PGS, CG, Newton
- `MjcfJacobianType` - Dense, Sparse, Auto
- `ExtendedSolverConfig` - Conversion to sim-types with extended settings

**Usage:**
```rust
use sim_mjcf::{parse_mjcf_str, ExtendedSolverConfig};

let mjcf = r#"
    <mujoco model="test">
        <option timestep="0.001" integrator="RK4" gravity="0 0 -10">
            <flag warmstart="true" contact="true"/>
        </option>
        <worldbody>
            <body name="ball">
                <geom type="sphere" size="0.1"/>
            </body>
        </worldbody>
    </mujoco>
"#;

let model = parse_mjcf_str(mjcf).expect("should parse");

// Access simulation config
let sim_config = model.simulation_config();
assert_eq!(sim_config.timestep, 0.001);

// Access extended config with MJCF-specific settings
let ext_config = &model.solver_config;
assert!(ext_config.warmstart_enabled());
assert!(ext_config.flags.contact);
```

**Files:** `sim-mjcf/src/types.rs`, `parser.rs`, `config.rs`, `model_builder.rs`, `validation.rs`

---

## Priority Roadmap

### ⚠️ Status: Partially Complete (January 2026)

> **The authoritative roadmap is [`sim/docs/todo/index.md`](./todo/index.md)**, which covers Phase 1 (12 items, all complete) and Phase 2 (17 items) with verified code references and acceptance criteria. The phases below reflect historical development milestones. Features marked with ⚠️ were later removed or remain standalone (not wired into the pipeline).

The following were completed in January 2026:

| Feature | Section | Notes |
|---------|---------|-------|
| Non-convex mesh collision | §5 Geoms | `CollisionShape::TriangleMesh` with BVH acceleration |
| SDF collision | §5 Geoms | All 10 shape combinations implemented |
| MJCF `<default>` element | §13 Model Format | ✅ `DefaultResolver` wired into `model_builder.rs` for all element types |
| MJCF `<tendon>` parsing | §13 Model Format | Spatial and fixed tendons |
| MJCF `<sensor>` parsing + wiring | §13 Model Format | 32 `MjcfSensorType` variants parsed; all 32 wired to pipeline via `process_sensors()` |

See [future_work_1.md](./todo/future_work_1.md) for remaining items.

### ✅ Multi-threading (constraint-parallel removed; cross-env parallelism active)

The `parallel` feature originally enabled multi-threaded constraint solving and body integration.
Island-parallel constraint solving (`solve_islands_parallel()`) and its dependencies (Newton solver, `islands.rs`) were **removed in Phase 3 consolidation**. The rayon dependency and `parallel` feature flag are now used by `BatchSim::step_all()` for cross-environment parallelism ([future_work_3 #9](./todo/future_work_3.md)).

**Files:** `sim-core/src/batch.rs` (BatchSim), `sim-types/src/config.rs` (ParallelConfig). Removed: `sim-constraint/src/parallel.rs`, `sim-core/src/world.rs`, `sim-core/src/stepper.rs`

### ✅ Recently Completed: MJB Binary Format

MJB binary format support added to `sim-mjcf` crate (requires `mjb` feature):
- `save_mjb_file()` / `load_mjb_file()` - Serialize/deserialize models to/from binary files
- `save_mjb_bytes()` / `load_mjb_bytes()` - In-memory binary serialization
- `is_mjb_file()` / `is_mjb_bytes()` - Format detection
- File format: magic bytes (`MJB1`) + version + flags + bincode-encoded `MjcfModel`

### ✅ Previously Completed: Free/Planar/Cylindrical Joint Solvers

These joint types now have full constraint solver support:
- **FreeJoint**: 6 DOF floating base for quadrupeds, humanoids, drones
- **PlanarJoint**: 3 DOF for mobile robots on flat surfaces
- **CylindricalJoint**: 2 DOF for screw mechanisms (rotation + translation)

---

### ✅ Phase 1: Core Parity (COMPLETED)

1. ~~**Collision shapes**: Box-box, box-sphere, capsule detection~~ ✅
2. ~~**Broad-phase**: Sweep-and-prune or BVH integration~~ ✅
3. ~~**Elliptic friction cones**: Replace circular with elliptic~~ ✅ (full elliptic cones with variable condim 1/3/4/6 in PGS/CG solvers)
4. ~~**Sensors**: IMU, force/torque, touch sensors~~ ✅ (sim-sensor crate standalone + 27 pipeline sensor types functional)
5. ~~**Implicit integration**: Implicit-in-velocity method~~ ✅

### ⚠️ Phase 2: Solver Improvements (built then partially removed)

1. ~~**Newton solver**: For faster convergence~~ ✅ → ⚠️ **Removed** (Phase 3 consolidation — dead code, never called by pipeline)
2. ~~**Constraint islands**: For performance~~ ✅ → ⚠️ **Removed** (Phase 3 consolidation — depended on Newton solver)
3. ~~**Sleeping**: Deactivate stationary bodies~~ ✅
4. ~~**GJK/EPA**: For convex mesh collision~~ ✅

### ⚠️ Phase 3: Extended Features (standalone crates built, not in pipeline)

1. ~~**MJCF loading**: For MuJoCo model compatibility~~ ✅
2. ~~**Muscle actuators**: For biomechanics~~ ✅ **Pipeline** (MuJoCo FLV model in `mj_fwd_actuation()` with activation dynamics, control/force clamping; sim-muscle crate provides richer standalone Hill-type model; see [future_work_1 #5](./todo/future_work_1.md) ✅)
3. ~~**Tendons**: For cable robots~~ ✅ **Pipeline** (fixed + spatial tendons fully integrated, including sphere/cylinder wrapping, sidesite, pulley; see [future_work_1 #4](./todo/future_work_1.md), [future_work_2 #4](./todo/future_work_2.md))
4. ~~**Deformables**: For soft body simulation~~ ✅ **Pipeline** (split-solve deformable-rigid contact + XPBD; see [future_work_4 #11](./todo/future_work_4.md) ✅)

### ⚠️ Phase 4: Solver & Performance (built then partially removed)

Focus: Internal solver improvements for better performance.

1. ~~**Sparse matrix operations**: CSR/CSC matrices for constraint Jacobians~~ ✅ → ⚠️ **Removed** (Phase 3 consolidation — `sparse.rs` deleted, never used by pipeline)
2. ~~**Warm starting**: Initialize from previous frame's solution~~ ✅ (pipeline PGS has its own warm starting)
3. ~~**Implicit-fast (no Coriolis)**: Skip Coriolis terms for performance~~ — Removed (standalone `integrators.rs` deleted in FUTURE_WORK C1; pipeline uses `ImplicitSpringDamper`)

**Implemented:**

**Sparse Matrix Operations (`sim-constraint/src/sparse.rs` — removed in Phase 3 consolidation):**
- `SparseJacobian` - CSR format for efficient J*v and J^T*v operations
- `SparseEffectiveMass` - CSC format for Cholesky factorization
- `JacobianBuilder` - Triplet accumulation for building sparse matrices
- `InvMassBlock` - Efficient 6x6 inverse mass/inertia storage
- Automatic dense/sparse switching based on system size (threshold: 16 bodies)

**Warm Starting (`sim-constraint/src/newton.rs` — removed in Phase 3 consolidation):**
- `NewtonSolverConfig::warm_starting` - Enable/disable warm starting
- `NewtonSolverConfig::warm_start_factor` - Scaling factor (0.8-0.95 typical)
- `SolverStats` - Track warm start usage and convergence metrics
- Lambda values cached between frames, scaled by warm start factor

**Implicit-Fast Integration (removed):**
- The standalone `ImplicitFast` integrator and its `IntegrationMethod::ImplicitFast`
  dispatch variant were removed in FUTURE_WORK C1 (dead code — not called by
  the pipeline). The pipeline's `Integrator::ImplicitSpringDamper` covers the
  primary use case (diagonal spring/damper implicit Euler).

**Files (removed in consolidation):**
- `sim-constraint/src/sparse.rs` — removed in Phase 3
- `sim-constraint/src/newton.rs` — removed in Phase 3
- `sim-core/src/integrators.rs` — removed in FUTURE_WORK C1

### Phase 5: Collision Completeness ✅ COMPLETED

Focus: All collision detection improvements, primarily in sim-core.

| Feature | Section | Complexity | Notes |
|---------|---------|------------|-------|
| ~~Cylinder collision shape~~ | §4 Collision, §5 Geoms | Medium | ✅ Native cylinder support with GJK/EPA |
| ~~Ellipsoid collision shape~~ | §4 Collision, §5 Geoms | Medium | ✅ Native ellipsoid support with GJK/EPA |
| ~~Mid-phase BVH per body~~ | §4 Collision | Medium | ✅ AABB tree for complex meshes |
| ~~Contact pairs filtering~~ | §3 Contact | Low | ✅ contype/conaffinity bitmasks |

**Implemented:**

**Cylinder Collision Shape (`sim-core/src/collision_shape.rs`, `gjk_epa.rs`):**
- `CollisionShape::Cylinder { half_length, radius }` - Flat-capped cylinder
- `CollisionShape::cylinder(half_length, radius)` - Constructor
- `support_cylinder()` - GJK support function with proper local direction handling
- Tight AABB computation accounting for rotation
- All collision pairs routed through GJK/EPA

**Ellipsoid Collision Shape (`sim-core/src/collision_shape.rs`, `gjk_epa.rs`):**
- `CollisionShape::Ellipsoid { radii }` - Axis-aligned ellipsoid
- `CollisionShape::ellipsoid(radii)`, `ellipsoid_xyz(rx, ry, rz)` - Constructors
- `support_ellipsoid()` - GJK support function using normalized gradient
- AABB computation via rotation matrix column projection
- All collision pairs routed through GJK/EPA

**Mid-Phase BVH (`sim-core/src/mid_phase.rs`):**
- `Bvh` - Top-down AABB tree for triangle meshes
- `BvhPrimitive` - Stores AABB, index, and user data per primitive
- `BvhNode` - Internal (with children) or leaf (with primitives)
- `Bvh::build()` - Top-down construction with median splitting
- `Bvh::query()` - Find all primitives overlapping an AABB
- `Bvh::query_pairs()` - Self-intersection query
- `bvh_from_triangle_mesh()` - Helper for building from triangle soup
- Configurable `max_primitives_per_leaf` (default: 4)

**Contact Pairs Filtering (MuJoCo pipeline):**
- `geom_contype: Vec<u32>` - Geom collision type bitmask (in `Model`)
- `geom_conaffinity: Vec<u32>` - Geom affinity bitmask (in `Model`)
- Check: `(a.contype & b.conaffinity) != 0 && (b.contype & a.conaffinity) != 0`
- Defaults: `contype = 1`, `conaffinity = 1` (everything collides)

**MJCF Model Builder Updates (`sim-mjcf/src/model_builder.rs`):**
- Transfer `contype`/`conaffinity` from MJCF geoms to Model arrays
- Native `CollisionShape::Cylinder` for MJCF cylinders (no longer approximated)
- Native `CollisionShape::Ellipsoid` for MJCF ellipsoids

**Files:** `sim-core/src/mujoco_pipeline.rs`, `sim-core/src/gjk_epa.rs`, `sim-core/src/mid_phase.rs`, `sim-mjcf/src/model_builder.rs`

### Phase 6: Contact Physics ✅ RE-IMPLEMENTED

Focus: Advanced friction models — **originally implemented in sim-contact, removed in Phase 3 consolidation, then re-implemented directly in the pipeline** with full variable-dimension support.

| Feature | Section | Complexity | Notes |
|---------|---------|------------|-------|
| ~~Torsional friction~~ | §3 Contact | Medium | ✅ **Implemented** (`Contact.mu[2]`, angular Jacobian row 3, `apply_contact_torque()`) |
| ~~Rolling friction~~ | §3 Contact | High | ✅ **Implemented** (`Contact.mu[3..5]`, angular Jacobian rows 4-5, `apply_contact_torque()`) |
| ~~Pyramidal friction cones~~ | §3 Contact | Low | ⚠️ **Stub** (warning emitted, falls back to elliptic — requires different system size) |

**Implemented (in `sim-core/src/mujoco_pipeline.rs`):**

**Variable Contact Dimensions:**
- `Contact.dim: usize` — Contact dimensionality (1, 3, 4, or 6)
- `Contact.mu: [f64; 5]` — Friction coefficients [sliding1, sliding2, torsional, rolling1, rolling2]
- `efc_offsets: Vec<usize>` — Starting row for each contact in the constraint system
- Variable-length warmstart with `HashMap<WarmstartKey, Vec<f64>>`

**Elliptic Friction Cone Projection (`project_elliptic_cone()`):**
- Two-step physically-correct projection:
  1. Enforce unilateral constraint (λ_n ≥ 0, else release contact)
  2. Scale friction components to cone boundary if `||(λ_i/μ_i)|| > λ_n`
- Handles per-direction friction coefficients for anisotropic friction
- Zero-mu components clamped without affecting others

**Dispatcher (`project_friction_cone()`):**
- Routes to appropriate projection based on contact dimension
- condim 1: Normal only (frictionless)
- condim 3: Normal + 2D tangential (sliding friction)
- condim 4: condim 3 + torsional (spinning resistance)
- condim 6: condim 4 + rolling (full MuJoCo model)

**Torsional Friction (condim ≥ 4):**
- Angular Jacobian row 3: `J_ω[0..3] = contact_normal`
- Torque applied via `apply_contact_torque()` using `Contact.mu[2]`
- Opposes rotation about the contact normal (spinning)

**Rolling Friction (condim = 6):**
- Angular Jacobian rows 4-5: perpendicular to contact normal
- Torques applied via `apply_contact_torque()` using `Contact.mu[3..5]`
- Opposes rotation perpendicular to the contact normal

**Torque Application (`apply_contact_torque()`):**
```rust
// Apply torsional/rolling friction torques to body angular accelerations
fn apply_contact_torque(data: &mut Data, contact: &Contact, lambda: &[f64]) {
    // Torsional (row 3): torque about contact normal
    if contact.dim >= 4 && lambda.len() > 3 {
        let torque = contact.normal * lambda[3];
        // Apply to both bodies with opposite signs...
    }
    // Rolling (rows 4-5): torque perpendicular to normal
    if contact.dim == 6 && lambda.len() > 5 {
        let tangent1 = contact.frame.col(0);
        let tangent2 = contact.frame.col(1);
        let torque = tangent1 * lambda[4] + tangent2 * lambda[5];
        // Apply to both bodies with opposite signs...
    }
}
```

**Files:**
- `sim-core/src/mujoco_pipeline.rs` — `project_elliptic_cone()`, `project_friction_cone()`, `apply_contact_torque()`, PGS/CG solvers
- `sim-core/src/contact.rs` — `Contact` struct with `dim`, `mu`, `frame` fields
- Formerly `sim-contact/src/friction.rs`, `sim-contact/src/model.rs` — removed in Phase 3 consolidation, re-implemented in pipeline

### Phase 7: Actuators & Control ⚠️ STANDALONE (sim-constraint only — not in pipeline)

Focus: New actuator types and joint coupling in sim-constraint. **All items are standalone** — pipeline `mj_fwd_actuation()` has full gain/bias processing for all actuator types (GainType/BiasType dispatch, FLV curves for muscles, Fixed/Affine gain/bias for non-muscle types, FilterExact dynamics).

| Feature | Section | Complexity | Notes |
|---------|---------|------------|-------|
| ~~Integrated velocity actuator~~ | §7 Actuators | Low | ✅ → ⚠️ **Standalone** (sim-constraint; pipeline has its own velocity actuator via gain/bias) |
| ~~General custom actuator~~ | §7 Actuators | Medium | ✅ → ⚠️ **Standalone** (sim-constraint; pipeline has its own general actuator via GainType/BiasType) |
| ~~Pneumatic cylinder actuator~~ | §7 Actuators | Medium | ✅ → ⚠️ **Standalone** (sim-constraint; pipeline has its own cylinder actuator via gain/bias + Filter dynamics) |
| ~~Adhesion actuator~~ | §7 Actuators | Medium | ✅ → ⚠️ **Standalone** (sim-constraint; pipeline has its own adhesion actuator via gain/bias) |
| ~~Joint coupling constraints~~ | §10 Equality | Medium | ✅ → ⚠️ **Standalone** (sim-constraint; pipeline has its own equality constraint solver) |

**Implemented:**

**Actuator Trait (`sim-constraint/src/actuator.rs`):**
- `Actuator` trait - Common interface for all actuator types
- `set_command()`, `compute_force()`, `reset()`, `max_force()`, `has_dynamics()`
- `BoxedActuator` type alias for trait objects
- `IntoBoxedActuator` extension trait

**Integrated Velocity Actuator:**
- `IntegratedVelocityActuator` - MuJoCo-style velocity integration
- Integrates velocity commands into internal position target
- PD control to track integrated target position
- Position limits with clamping
- Configurable gains (kp, kd) and max velocity/force

**Pneumatic Cylinder Actuator:**
- `PneumaticCylinderActuator` - Double-acting cylinder with pressure dynamics
- Separate extend/retract chambers with pressure state
- Fill, exhaust, and leak rate constants
- Coulomb and viscous friction modeling
- Presets: `mckibben_small()`, `industrial_medium()`, `hydraulic()`

**Adhesion Actuator:**
- `AdhesionActuator` - Controllable adhesion for gripping/climbing
- First-order activation dynamics with separate on/off time constants
- Contact-dependent force (requires contact ratio input)
- Shear adhesion ratio for sliding resistance
- Presets: `electroadhesion()`, `gecko_adhesion()`, `suction_cup()`, `magnetic()`

**Custom Actuator:**
- `CustomActuator<F>` - User-defined actuator via closure
- Generic over `FnMut(command, position, velocity, dt) -> force`
- Automatic force clamping to max_force

**Joint Coupling Constraints (`sim-constraint/src/equality.rs`):**
- `JointCoupling` - Linear constraint: `Σᵢ cᵢ · qᵢ = offset`
- `CouplingCoefficient` - Per-joint coefficient storage
- `GearCoupling` - Specialized two-joint gear ratio
- `DifferentialCoupling` - Two inputs to one output (averaging, difference, weighted)
- `CouplingGroup` - Collection of couplings solved together
- Factory methods: `gear()`, `mimic()`, `parallel()`, `anti_parallel()`
- Baumgarte stabilization for position drift correction
- Compliance and damping for soft constraints

**Usage:**
```rust
use sim_constraint::{
    Actuator, IntegratedVelocityActuator, PneumaticCylinderActuator,
    AdhesionActuator, CustomActuator,
    JointCoupling, GearCoupling, DifferentialCoupling, CouplingGroup,
};
use sim_types::JointId;

// Integrated velocity actuator
let mut actuator = IntegratedVelocityActuator::new(2.0, 100.0)  // max vel, max force
    .with_position_limits(-1.5, 1.5)
    .with_gains(1000.0, 100.0);

actuator.set_command(0.5);  // 50% of max velocity
let force = actuator.compute_force(position, velocity, dt);

// Pneumatic cylinder
let mut cylinder = PneumaticCylinderActuator::mckibben_small();
cylinder.set_command(1.0);  // Full extend
let force = cylinder.compute_force(position, velocity, dt);

// Adhesion gripper
let mut gripper = AdhesionActuator::gecko_adhesion(100.0);
gripper.set_contact_ratio(1.0);  // Full contact
gripper.set_command(1.0);  // Activate adhesion
let adhesion_force = gripper.compute_force(position, velocity, dt);

// Custom actuator (spring-damper)
let custom = CustomActuator::new("spring_damper", 100.0, |cmd, pos, vel, _dt| {
    500.0 * (cmd - pos) - 50.0 * vel  // PD controller
});

// Gear coupling (10:1 reduction)
let gear = GearCoupling::reduction(JointId::new(0), JointId::new(1), 10.0);
let output_pos = gear.output_from_input(input_pos);

// Differential drive (average of two motors)
let diff = DifferentialCoupling::averaging(
    JointId::new(0),  // left motor
    JointId::new(1),  // right motor
    JointId::new(2),  // output wheel
);
let output = diff.compute_output(left_pos, right_pos);

// Mimic joint (URDF-style)
let mimic = JointCoupling::mimic(
    JointId::new(0),  // leader
    JointId::new(1),  // follower
    2.0,              // multiplier
    0.5,              // offset
);

// Coupling group for batch solving
let group = CouplingGroup::new()
    .with_gear(JointId::new(0), JointId::new(1), 2.0)
    .with_mimic(JointId::new(2), JointId::new(3), 1.0, 0.0);

let forces = group.compute_all_forces(get_position, get_velocity, dt);
```

**Files:**
- `sim-constraint/src/actuator.rs` - Actuator trait and implementations
- `sim-constraint/src/equality.rs` - Joint coupling constraints
- `sim-constraint/src/lib.rs` - Public exports

### Phase 8: Sensors ✅ IMPLEMENTED (sim-sensor crate standalone + pipeline sensors functional)

Focus: Additional sensor types in sim-sensor. **Both standalone and pipeline implementations are now functional.**

| Feature | Section | Complexity | Notes |
|---------|---------|------------|-------|
| ~~Rangefinder sensor~~ | §8 Sensors | Medium | ✅ **Implemented** in both sim-sensor crate and pipeline (ray-cast along +Z with mesh support) |
| ~~Magnetometer sensor~~ | §8 Sensors | Low | ✅ **Implemented** in both sim-sensor crate and pipeline (`model.magnetic` in sensor frame) |

**Implemented:**

**Rangefinder Sensor (`sim-sensor/src/rangefinder.rs`):**
- `Rangefinder` - Ray-based distance measurement sensor
- `RangefinderConfig` - Configurable local position, direction, min/max range, beam width, noise
- `RangefinderReading` - Distance and hit status output
- `RayCaster` trait - Interface for integrating with physics world ray casting
- `RayHit` - Ray cast result with distance, point, normal, body ID
- Presets: `height_sensor()`, `forward_proximity()`, `lidar()`, `ultrasonic()`
- Supports clamping to min/max range, optional infinity on no hit

**Magnetometer Sensor (`sim-sensor/src/magnetometer.rs`):**
- `Magnetometer` - Magnetic field measurement sensor for compass heading
- `MagnetometerConfig` - Earth field vector, noise, hard-iron bias, soft-iron distortion
- `MagnetometerReading` - Magnetic field vector with heading computation
- `for_location()` - Configure Earth field based on declination, inclination, intensity
- Hard-iron (constant bias) and soft-iron (scale distortion) calibration modeling
- `heading()` and `heading_degrees()` - Compute yaw from magnetic field

**Usage:**
```rust
use sim_sensor::{Rangefinder, RangefinderConfig, Magnetometer, MagnetometerConfig};
use sim_types::{BodyId, Pose};
use nalgebra::{Point3, Vector3};

// Create a downward-facing height sensor
let height_sensor = Rangefinder::new(
    BodyId::new(1),
    RangefinderConfig::height_sensor(10.0),
);
let pose = Pose::from_position(Point3::new(0.0, 0.0, 1.5));
let reading = height_sensor.read_with_distance(&pose, Some(1.5));
assert!(reading.is_hit());
assert!((reading.distance - 1.5).abs() < 0.001);

// Create a magnetometer for heading estimation
let compass = Magnetometer::new(
    BodyId::new(1),
    MagnetometerConfig::default(),
);
let reading = compass.read(&Pose::identity());
let heading = reading.heading_degrees();
println!("Heading: {:.1}°", heading);
```

**Files:**
- `sim-sensor/src/rangefinder.rs` - Rangefinder sensor implementation
- `sim-sensor/src/magnetometer.rs` - Magnetometer sensor implementation
- `sim-sensor/src/types.rs` - `SensorType::Rangefinder`, `SensorType::Magnetometer`, `SensorData` variants
- `sim-types/src/observation.rs` - `SensorObservation::rangefinder()`, `SensorObservation::magnetometer()`

### Phase 9: Advanced Features (Partially Complete)

Focus: Large standalone features, each potentially its own PR.

| Feature | Section | Complexity | Status | Notes |
|---------|---------|------------|--------|-------|
| ~~Height field collision~~ | §4 Collision, §5 Geoms | High | ✅ COMPLETED | Terrain simulation |
| ~~Conjugate Gradient solver~~ | §2 Solvers | Medium | ✅ COMPLETED | Pipeline CG in `mujoco_pipeline.rs` (`cg_solve_contacts()`); `CGSolver` in sim-constraint remains standalone joint-space solver (see FUTURE_WORK #3 ✅) |
| ~~Tendon coupling constraints~~ | §10 Equality | Low | ✅ COMPLETED → ⚠️ **Standalone** | In sim-constraint; pipeline tendon *equality* constraints not yet implemented (tendon kinematics/actuation/limits ARE in pipeline) |
| ~~Flex edge constraints~~ | §10 Equality | Low | ✅ **COMPLETED** | XPBD called from pipeline via `mj_deformable_step()` |
| ~~SDF collision~~ | §4 Collision, §5 Geoms | High | ✅ **COMPLETED** | All 10 shape combinations implemented (see §5 notes) |
| ~~Skinned meshes~~ | §11 Deformables | High | ✅ **COMPLETED** | Visual deformation for rendering |
| ~~Multi-threading~~ | §12 Performance | Medium | ✅ **Active** | Island-parallel solving removed in Phase 3; `BatchSim::step_all()` now uses rayon `par_iter_mut` for cross-environment parallelism; see [future_work_3 #9](./todo/future_work_3.md) |
| ~~MJB binary format~~ | §13 Model Format | Low | ✅ **COMPLETED** | Faster loading via bincode serialization |

**Implemented (some now standalone or removed — see table above for current status):**

> **⚠️ Note:** The `CGSolver` below is standalone (`sim-constraint`), operating on joint-space constraints via the `Joint` trait. The pipeline's contact CG solver (`cg_solve_contacts()` in `mujoco_pipeline.rs`) is a separate implementation using PGD with Barzilai-Borwein step. Tendon *equality* constraints (`TendonConstraint`) are in `sim-constraint` and not used by the pipeline (the pipeline handles tendon kinematics, actuation, limits, and passive forces directly). Island-parallel solving was removed in Phase 3 consolidation.

**Conjugate Gradient Solver (`sim-constraint/src/cg.rs`) — ⚠️ Standalone:**
- `CGSolver` - Conjugate gradient method for constraint solving
- `CGSolverConfig` - Configurable tolerance, max iterations, Baumgarte stabilization
- `Preconditioner` - None, Jacobi, or BlockJacobi preconditioning
- `CGSolverResult` - Solution with convergence statistics
- Presets: `high_accuracy()`, `realtime()`, `large_system()`
- Optimal for systems with 100+ constraints

**Usage:**
```rust
use sim_constraint::{CGSolver, CGSolverConfig, Preconditioner};

let config = CGSolverConfig::realtime()
    .with_preconditioner(Preconditioner::Jacobi);
let mut solver = CGSolver::new(config);

let result = solver.solve(&joints, get_body_state, dt);
println!("Converged in {} iterations", result.iterations_used);
```

**Tendon Coupling Constraints (`sim-constraint/src/equality.rs`) — ⚠️ Standalone:**
- `TendonConstraint` - Joint-to-joint coupling via tendon mechanics
- Moment arm modeling for each connected joint
- Rest length and target length with slack/taut states
- Stiffness and damping for compliant constraints
- `TendonNetwork` - Collection of tendons for batch solving
- Presets: `two_joint()`, `finger()`

**Usage:**
```rust
use sim_constraint::{TendonConstraint, TendonNetwork};
use sim_types::JointId;

let tendon = TendonConstraint::two_joint(
    "finger_flexor",
    JointId::new(0), 0.01,  // MCP joint, 1cm moment arm
    JointId::new(1), 0.008, // PIP joint, 0.8cm moment arm
    0.12,                    // 12cm rest length
);

let mut network = TendonNetwork::new();
network.add_tendon(tendon);
let forces = network.compute_all_forces(&get_position, &get_velocity, dt);
```

**Flex Edge Constraints (`sim-deformable/src/constraints.rs`) — ✅ Pipeline-integrated:**
- `FlexEdgeType` - Stretch, Shear, StretchShear, or Twist constraint types
- `FlexEdgeConstraint` - XPBD constraint for deformable edge behavior
- Stretch constraint for distance maintenance (2 vertices)
- Shear constraint for angular stiffness (3 vertices)
- Twist constraint for torsion resistance
- Damping support for energy dissipation
- Compatible with `XpbdSolver` for cloth and soft body simulation

**Usage:**
```rust
use sim_deformable::{FlexEdgeConstraint, FlexEdgeType, XpbdSolver};

// Stretch constraint between two vertices
let stretch = FlexEdgeConstraint::stretch(0, 1, rest_length, compliance);

// Shear constraint for angular stiffness
let shear = FlexEdgeConstraint::shear([0, 1, 2], rest_angle, compliance);

// Combined stretch-shear
let combined = FlexEdgeConstraint::stretch_shear(
    [0, 1, 2], rest_length, rest_angle,
    stretch_compliance, shear_compliance,
);
```

**Height Field Collision (`sim-core/src/heightfield.rs`):**
- `HeightFieldData` - 2D grid of height values for terrain
- `HeightFieldContact` - Contact result with point, normal, penetration, cell
- Bilinear interpolation for smooth height sampling
- Normal computation via finite differences
- `CollisionShape::HeightField` - Integration with collision pipeline
- Collision detection with spheres, capsules, and boxes
- AABB computation for broad-phase integration
- Presets: `flat_terrain()`, `terrain_from_fn()`

**Usage:**
```rust
use sim_core::{CollisionShape, HeightFieldData};
use std::sync::Arc;

// Create height field from function
let terrain = CollisionShape::terrain_from_fn(100, 100, 1.0, |x, y| {
    (x * 0.1).sin() * (y * 0.1).cos() * 2.0  // Wavy terrain
});

// Or load from data
let heights: Vec<f64> = load_terrain_data();
let data = HeightFieldData::new(heights, 256, 256, 0.5);
let terrain = CollisionShape::heightfield(Arc::new(data));
```

**Files:**
- `sim-constraint/src/cg.rs` - Conjugate Gradient solver
- `sim-constraint/src/equality.rs` - Tendon coupling constraints (additions)
- `sim-deformable/src/constraints.rs` - Flex edge constraints (additions)
- `sim-core/src/heightfield.rs` - Height field collision
- `sim-core/src/collision_shape.rs` - CollisionShape::HeightField variant
- `sim-core/src/gjk_epa.rs` - HeightField support function

> **📌 Note:** For the current authoritative roadmap with verified code references, see [`sim/docs/todo/index.md`](./todo/index.md). The priority roadmap above is historical and partially outdated.

---

## File Reference

| Crate | Purpose | Key Files |
|-------|---------|-----------|
| `sim-types` | Data structures | `dynamics.rs`, `joint.rs`, `observation.rs`, `body.rs`, `config.rs` |
| `sim-core` | Integration, MuJoCo pipeline, Collision | `mujoco_pipeline.rs`, `collision_shape.rs`, `contact.rs`, `gjk_epa.rs`, `mid_phase.rs`, `heightfield.rs`, `sdf.rs`, `mesh.rs`, `raycast.rs` (removed: `world.rs`, `stepper.rs`, `broad_phase.rs`, `integrators.rs`) |
| `sim-constraint` | Joint constraints (⚠️ standalone) | `joint.rs`, `types.rs`, `actuator.rs`, `equality.rs`, `cg.rs`, `limits.rs`, `motor.rs`, `muscle.rs` (removed in Phase 3: `solver.rs`, `newton.rs`, `islands.rs`, `sparse.rs`, `pgs.rs`, `parallel.rs`) |
| `sim-sensor` | Sensor simulation (⚠️ standalone) | `imu.rs`, `force_torque.rs`, `touch.rs`, `rangefinder.rs`, `magnetometer.rs` |
| `sim-urdf` | URDF loading | `lib.rs`, `parser.rs`, `converter.rs`, `types.rs`, `validation.rs`, `error.rs` |
| `sim-mjcf` | MJCF loading | `lib.rs`, `parser.rs`, `types.rs`, `validation.rs`, `model_builder.rs`, `defaults.rs`, `config.rs`, `error.rs`, `mjb.rs` |
| `sim-muscle` | Muscle actuators (⚠️ standalone) | `activation.rs`, `curves.rs`, `hill.rs`, `kinematics.rs` |
| `sim-tendon` | Tendon/cable systems (⚠️ standalone) | `fixed.rs`, `spatial.rs`, `path.rs`, `wrapping.rs`, `pulley.rs`, `cable.rs` |
| `sim-deformable` | Soft body simulation (✅ pipeline-integrated) | `capsule_chain.rs`, `cloth.rs`, `soft_body.rs`, `solver.rs`, `constraints.rs`, `skinning.rs`, `material.rs`, `mesh.rs` |
| `sim-physics` | Umbrella re-export | `lib.rs` |

---

## How to Use This Document

1. **Pick a feature** from the tables above based on your needs
2. **Check the status** to understand current implementation state
3. **Read implementation notes** for guidance on approach
4. **Modify indicated files** or create new crates as specified
5. **Update this document** when features are completed

Each session, tell the assistant:
> "Implement [feature] per the gap analysis. See docs/MUJOCO_GAP_ANALYSIS.md for context."

The assistant can read this file and the relevant source files to implement the feature efficiently.
