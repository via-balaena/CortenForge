# MuJoCo Gap Analysis: CortenForge Physics Stack

<!--
LLM/Agent Note: This document is ~2000 lines. To update it efficiently:
1. Use offset/limit reads to target specific sections (see ## headers below)
2. Key sections: Executive Summary (line ~18), §5 Geom Types (line ~465), §13 Model Format (line ~1180)
3. The document is well-structured - grep for "^## " to find all section headers
-->

This document provides a comprehensive comparison between MuJoCo's physics capabilities and CortenForge's current `sim-*` crate implementation.

> **Note:** All `sim-*` crates are in initial development (pre-1.0). Breaking changes to APIs are expected and acceptable. Prefer clean, correct implementations over backwards compatibility.

> **Current Roadmap:** For the authoritative list of verified gaps with file:line references, acceptance criteria, and dependency graph, see [`sim/docs/FUTURE_WORK.md`](./FUTURE_WORK.md).

---

## 📊 Executive Summary

**Overall completion: ~65-70%** of MuJoCo's core pipeline features are functional end-to-end. Many standalone crates exist but are not yet wired into the MuJoCo pipeline (`mujoco_pipeline.rs`). See `sim/docs/FUTURE_WORK.md` for the roadmap (11 items total, 4 completed, 7 remaining).

### Fully Implemented (in pipeline)
- Integration methods: Euler, RK4 (true 4-stage Runge-Kutta), ImplicitSpringDamper (diagonal spring/damper only — see [FUTURE_WORK #7](./FUTURE_WORK.md))
- Constraint solver: PGS (plain Gauss-Seidel, no SOR), Warm Starting
- Contact model (Compliant with solref/solimp, circular friction cone condim 3, contype/conaffinity filtering)
- Collision detection (All primitive shapes, GJK/EPA, Height fields, BVH, **TriangleMesh, SDF**)
- Joint types (Fixed, Revolute, Prismatic, Spherical, Universal, **Free**)
- Actuators: Motor (`ctrl * gear` only — no gain/bias for position/velocity/damper servos)
- Sensors (27 functional in pipeline): JointPos, JointVel, BallQuat, BallAngVel, FramePos, FrameQuat, FrameXAxis/YAxis/ZAxis, FrameLinVel, FrameAngVel, FrameLinAcc, FrameAngAcc, Accelerometer, Gyro, Velocimeter, SubtreeCom, SubtreeLinVel, SubtreeAngMom, ActuatorPos, ActuatorVel, ActuatorFrc, Force, Torque, Touch, Rangefinder, Magnetometer
- Model loading (URDF, MJCF with `<default>` parsing, **MJB binary format**) — note: `DefaultResolver` is implemented but **not called** by `model_builder.rs`; defaults are parsed then dropped

### Placeholder / Stub (in pipeline)
- Elliptic/Pyramidal friction cones — `cone` field stored but solver hardcodes circular cone
- Torsional/Rolling friction — `Contact.dim` and `geom_friction.y/.z` stored but solver is condim 3 only
- TendonPos/TendonVel sensors — write `0.0`, pending tendon pipeline integration ([FUTURE_WORK #4](./FUTURE_WORK.md))
- Tendon/Site actuation — placeholder comment in `mj_fwd_actuation()` ([FUTURE_WORK #4](./FUTURE_WORK.md))

### Standalone Crates (not wired into pipeline)
- sim-tendon (3,919 lines) — zero pipeline coupling ([FUTURE_WORK #4](./FUTURE_WORK.md))
- sim-muscle (2,550 lines) — zero pipeline coupling ([FUTURE_WORK #5](./FUTURE_WORK.md))
- sim-deformable (7,733 lines) — XPBD solver not called from `Data::step()` ([FUTURE_WORK #9](./FUTURE_WORK.md))
- sim-sensor (rangefinder, magnetometer, force/torque) — standalone crate with own API; pipeline has independent implementations
- CGSolver in sim-constraint (1,664 lines) — 0 pipeline callers ([FUTURE_WORK #3](./FUTURE_WORK.md))
- ~~`integrators.rs` trait system~~ — removed in FUTURE_WORK C1
- Pneumatic, Adhesion, Muscle actuators in sim-constraint — not in `mj_fwd_actuation()`
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
| RK4 integrator | True 4-stage Runge-Kutta via `mj_runge_kutta()` with quaternion-safe position updates | [§1](#1-integration-methods) |
| Pipeline sensors (27 types) | Force, Torque, Touch, Rangefinder, Magnetometer, SubtreeAngMom, ActuatorPos/Vel all functional | [§8](#8-sensors) |
| Non-convex mesh collision | TriangleMesh ↔ all primitives + mesh-mesh with BVH acceleration | [§5](#5-geom-types-collision-shapes) |
| SDF collision | All 10 shape combinations (Sphere, Capsule, Box, Cylinder, Ellipsoid, ConvexMesh, Plane, TriangleMesh, HeightField, Sdf↔Sdf) | [§5](#5-geom-types-collision-shapes) |
| MJCF `<default>` element | ⚠️ Parsed only — `DefaultResolver` implemented but **not called** by `model_builder.rs`; defaults are silently dropped | [§13](#13-model-format) |
| MJCF `<tendon>` parsing | Spatial and fixed tendons with site/joint references | [§13](#13-model-format) |
| MJCF `<sensor>` parsing | 24 sensor types (position, velocity, force, IMU, etc.) | [§13](#13-model-format) |
| Multi-threading | `parallel` feature with rayon (island-parallel solving removed in Phase 3) | [§12](#12-performance-optimizations) |
| SIMD optimization | `sim-simd` crate with `Vec3x4`, `Vec3x8`, batch operations | [§12](#12-performance-optimizations) |

**For typical robotics use cases**, collision detection, joint types, basic actuation, and sensors (27 types) are functional. Tendons, muscles, and deformable bodies require pipeline integration before they produce correct results. See `sim/docs/FUTURE_WORK.md` for the full gap list.

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
| RK4 | Supported | `RungeKutta4` | **Implemented** (true 4-stage RK4 via `mj_runge_kutta()`; see [FUTURE_WORK #8](./FUTURE_WORK.md) ✅) | - | - |
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
| Conjugate Gradient | Supported | `CGSolver` | **Standalone** (0 pipeline callers; see [FUTURE_WORK #3](./FUTURE_WORK.md)) | - | - |
| Constraint islands | Auto-detected | `ConstraintIslands` | **Removed** (Phase 3 consolidation) | - | - |
| Warm starting | Supported | `warm_starting` in pipeline PGS | **Implemented** | - | - |

> **Note:** The pipeline's PGS solver lives in `mujoco_pipeline.rs:7081` (`pgs_solve_contacts()`), not in `sim-constraint/src/pgs.rs` (which was deleted in Phase 3). The pipeline PGS uses standard Gauss-Seidel (ω=1.0) with no SOR relaxation factor. CGSolver exists in `sim-constraint/src/cg.rs` (1,664 lines) but has zero callers in the pipeline — see [FUTURE_WORK #3](./FUTURE_WORK.md) for the plan to wire it in.

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

> The `PGSSolver` described below was in `sim-constraint/src/pgs.rs` and was **deleted in Phase 3 consolidation**. The pipeline now has its own PGS in `mujoco_pipeline.rs:7081` (`pgs_solve_contacts()`) which uses plain Gauss-Seidel (ω=1.0) **without SOR**.

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
| Elliptic friction cones | Default | `Model.cone` field | **Stub** (`cone` field parsed and stored but PGS solver ignores it — hardcodes circular cone) | - | - |
| Pyramidal friction cones | Alternative | `Model.cone` field | **Stub** (`cone` field stored but solver ignores it) | - | - |
| Torsional friction | condim 4-6 | `Contact.dim`, `Contact.mu[1]` | **Stub** (data fields exist but solver is hardcoded to 3 rows per contact — condim 3 only) | - | - |
| Rolling friction | condim 6-10 | `geom_friction.y`/`.z` | **Stub** (data stored but only `.x` sliding friction is extracted for contacts) | - | - |
| Complete friction model | condim 6 | — | **Stub** (consequence of above — only condim 3 circular cone is functional) | - | - |
| Contact pairs filtering | Supported | `contype`/`conaffinity` bitmasks | **Implemented** | - | - |
| solref/solimp params | MuJoCo-specific | `jnt_solref`, `geom_solimp`, `eq_solref`, etc. in `Model` | **Implemented** (in MuJoCo pipeline) | - | - |

### Implementation Notes: Elliptic Friction Cones ⚠️ STUB (types exist but solver ignores cone field)

MuJoCo uses elliptic cones by default:
```
f_n ≥ 0, (f_t1/μ_1)² + (f_t2/μ_2)² ≤ f_n²
```

**Implemented:**
- `EllipticFrictionCone` struct with anisotropic friction coefficients (μ_1, μ_2)
- Newton-based projection onto ellipse boundary for forces outside the cone
- `FrictionModelType::Elliptic` for ContactModel integration
- `ContactParams::friction_anisotropy` parameter for anisotropic surfaces
- Presets: `ContactParams::treaded()`, `ContactParams::brushed_metal()`
- Conversions between circular and elliptic cones

**Files modified:** `sim-core/src/contact.rs`, `sim-core/src/mujoco_pipeline.rs` (formerly in `sim-contact`, now consolidated into `sim-core`)

### Implementation Notes: Advanced Friction Models ⚠️ STUB (data fields exist; solver hardcodes condim 3; former sim-contact code removed)

MuJoCo uses contact dimensionality (`condim`) to specify which friction components are active:
- **condim 1**: Normal force only (frictionless)
- **condim 3**: Normal + 2D tangential friction (sliding)
- **condim 4**: condim 3 + torsional friction (spinning resistance)
- **condim 6**: condim 4 + 2D rolling friction (rolling resistance)

**Torsional Friction:**
Opposes rotation about the contact normal (spinning). Torque is:
```
τ_torsion = -μ_torsion * r_contact * F_n * sign(ω_n)
```

**Rolling Friction:**
Opposes rotation perpendicular to the contact normal (rolling). Torque is:
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
| Height field | Native | `CollisionShape::HeightField` | **Implemented** (sim-core only; MJCF falls back to Box) | - | - |
| SDF (signed distance) | Native | `CollisionShape::Sdf` | **Implemented** (sim-core only; MJCF falls back to Box) | - | - |
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
| Height field | Yes | `CollisionShape::HeightField` | **Implemented** (sim-core; MJCF→Box fallback) |
| **Mesh (non-convex)** | Yes | `CollisionShape::TriangleMesh` | **Implemented** ✅ |
| **SDF** | Yes | `CollisionShape::Sdf` | **Implemented** (sim-core; MJCF→Box fallback) |

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

### Implementation Notes: SDF Collision ✅ COMPLETED (sim-core only — not wired through MJCF model builder)

Full SDF (Signed Distance Field) collision support is implemented in sim-core.
The MJCF model builder (`model_builder.rs`) currently falls back to Box for
`hfield` and `sdf` geom types. Wiring these through to the sim-core collision
shapes is a remaining integration task.
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
| Motor (direct torque) | Yes | `mj_fwd_actuation()` (`ctrl * gear`) | **Implemented** | - | - |
| Position servo | Yes | `JointMotor::position` | **Standalone** (in sim-constraint; pipeline has no gain/bias logic — treats as raw motor) | - | - |
| Velocity servo | Yes | `JointMotor::velocity` | **Standalone** (in sim-constraint; pipeline has no gain/bias logic — treats as raw motor) | - | - |
| PD control | Yes | `JointMotor::compute_force` with Kp/Kd | **Standalone** (in sim-constraint, not called by pipeline) | - | - |
| Integrated velocity | Yes | `IntegratedVelocityActuator` | **Standalone** (in sim-constraint, not in pipeline) | - | - |
| Damper | Yes | Passive joint damping in `mj_fwd_passive()` | **Partial** (passive damping works; MuJoCo `<damper>` actuator type not implemented — pipeline applies `ctrl * gear` instead of `-kv * qvel`) | - | - |
| Cylinder (pneumatic) | Yes | `PneumaticCylinderActuator` | **Standalone** (in sim-constraint, not in pipeline) | - | - |
| Muscle (Hill-type) | Yes | `HillMuscle` (via sim-muscle) | **Standalone** (not wired into `mj_fwd_actuation()`; see [FUTURE_WORK #5](./FUTURE_WORK.md)) | - | - |
| Adhesion | Yes | `AdhesionActuator` | **Standalone** (in sim-constraint, not in pipeline) | - | - |
| General (custom) | Yes | `CustomActuator<F>` | **Standalone** (in sim-constraint, not in pipeline) | - | - |

> **Pipeline vs standalone actuators.** The MuJoCo pipeline's `mj_fwd_actuation()` (`mujoco_pipeline.rs:5997`) only computes `ctrl * gear` for `ActuatorTransmission::Joint` — no gain/bias processing, so Position/Velocity/Damper actuators are functionally identical to Motor. `ActuatorTransmission::Tendon` and `::Site` are placeholders ([FUTURE_WORK #4](./FUTURE_WORK.md)). All `ActuatorDynamics` variants (`Filter`, `Integrator`, `Muscle`) are dead code — the enum and `actuator_dyntype` field are populated by the model builder but never read by any pipeline function; `data.act` is never updated. The sim-muscle crate (2,550 lines) and sim-constraint actuators (`JointMotor`, `IntegratedVelocityActuator`, `PneumaticCylinderActuator`, `AdhesionActuator`, `CustomActuator`) exist as standalone implementations not called by the pipeline.

### Implementation Notes: Muscle Model ✅ COMPLETED (standalone only — not in pipeline)

MuJoCo's muscle model includes:
- Activation dynamics (1st-order system)
- Force-length-velocity relationships
- Pennation angle

**Implemented in `sim-muscle` crate:**

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
| ActuatorPos / ActuatorVel | Yes | `MjSensorType::ActuatorPos/Vel` | **Implemented** (joint transmission `gear * qpos`/`gear * qvel`) | - |
| SubtreeAngMom | Yes | `MjSensorType::SubtreeAngMom` | **Implemented** (subtree angular momentum) | - |
| TendonPos / TendonVel | Yes | `MjSensorType::TendonPos/Vel` | **Stub** (writes `0.0`; requires [FUTURE_WORK #4](./FUTURE_WORK.md)) | - |
| Camera (rendered) | Yes | Out of scope | N/A | - |

> **Two sensor systems exist.** The sim-sensor crate has standalone `Imu`, `ForceTorqueSensor`, `TouchSensor`, `Rangefinder`, and `Magnetometer` implementations that operate on `RigidBodyState` objects. The MuJoCo pipeline has its own sensor readout in `mj_sensor_pos()`/`mj_sensor_vel()`/`mj_sensor_acc()` within `mujoco_pipeline.rs`. All 27 pipeline sensor types are now functional; only TendonPos/TendonVel remain as stubs (pending tendon pipeline integration, [FUTURE_WORK #4](./FUTURE_WORK.md)).

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
| Fixed tendons | Yes | `FixedTendon` | **Standalone** (sim-tendon crate, not in pipeline) | - | - |
| Spatial tendons | Yes | `SpatialTendon` | **Standalone** (sim-tendon crate, not in pipeline) | - | - |
| Wrapping (sphere/cylinder) | Yes | `SphereWrap`, `CylinderWrap` | **Standalone** (sim-tendon crate, not in pipeline) | - | - |
| Pulley systems | Yes | `PulleySystem` | **Standalone** (sim-tendon crate, not in pipeline) | - | - |

> **Standalone crate with zero pipeline coupling.** sim-tendon is a 3,919-line crate with path routing, wrapping geometry, and a `TendonActuator` trait. The MuJoCo pipeline declares Model fields (`tendon_stiffness`, `tendon_damping`, etc. at `mujoco_pipeline.rs:907-929`) and Data scaffolds (`ten_length`, `ten_velocity`, `ten_force`, `ten_J` at `mujoco_pipeline.rs:1362-1371`) but these are initialized to defaults and **never populated**. The `ActuatorTransmission::Tendon` placeholder at `mujoco_pipeline.rs:6017-6020` is a no-op. See [FUTURE_WORK #4](./FUTURE_WORK.md) for the tendon pipeline integration spec.

### Implementation Notes: Tendons ✅ COMPLETED (standalone sim-tendon crate only — not in pipeline)

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
| Tendon coupling | Yes | `TendonConstraint`, `TendonNetwork` | **Standalone** (in sim-constraint, not wired into pipeline tendon system; see [FUTURE_WORK #4](./FUTURE_WORK.md)) | - |
| Flex (edge length) | Yes | `FlexEdgeConstraint` | **Standalone** (in sim-deformable, XPBD not called from pipeline; see [FUTURE_WORK #9](./FUTURE_WORK.md)) | - |

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
| 1D (capsule chains) | Yes | `CapsuleChain` | **Standalone** (sim-deformable crate, not in pipeline) | - | - |
| 2D (triangle shells) | Yes | `Cloth` | **Standalone** (sim-deformable crate, not in pipeline) | - | - |
| 3D (tetrahedra) | Yes | `SoftBody` | **Standalone** (sim-deformable crate, not in pipeline) | - | - |
| Skinned meshes | Yes | `SkinnedMesh` | **Standalone** (sim-deformable crate, not in pipeline) | - | - |
| Deformable-rigid collision | Yes | `ConstraintType::Collision` | **Missing** (enum variant defined but unimplemented) | - | - |

> **Standalone crate with zero pipeline coupling.** sim-deformable is a 7,733-line crate (86 tests) with `XpbdSolver`, `Cloth`, `SoftBody`, `CapsuleChain`, and `DeformableBody` trait. The XPBD solver works standalone but is **not called from `Data::step()`**. No deformable-rigid collision detection exists — `ConstraintType::Collision` is an enum variant defined at `constraints.rs:42-43` but unimplemented. `Material.friction` and per-body `radius`/`thickness` fields are declared but unused by any collision system. See [FUTURE_WORK #9](./FUTURE_WORK.md) for the pipeline integration spec.

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

### Implementation Notes: Deformables ✅ COMPLETED (standalone sim-deformable crate only — not in pipeline)

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
| **Multi-threading** | Model-data separation | `parallel` feature with rayon | **Reserved** (Cargo.toml wiring exists but zero `#[cfg(feature = "parallel")]` or rayon usage in code; see [FUTURE_WORK #10](./FUTURE_WORK.md)) | - |
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

### Implementation Notes: Multi-threading ⚠️ PARTIALLY REMOVED

> The island-parallel constraint solving described below was **deleted in Phase 3 consolidation** along with the Newton solver and `islands.rs`. The `parallel` feature and rayon dependency remain available in sim-core (`core/Cargo.toml:17,31`) but have no active constraint-parallelism callers. See [FUTURE_WORK #10](./FUTURE_WORK.md) for the batched simulation plan which will use rayon for cross-environment parallelism.

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
// The parallel feature and rayon dependency remain for future use.
```

**Performance notes:**

- Parallel constraint solving benefits scenes with 2+ independent islands
- Falls back to sequential for small scenes to avoid parallel overhead
- Use `ParallelConfig::sequential()` for deterministic results matching default solver

**Files:** `sim-types/src/config.rs` (ParallelConfig remains), removed: `sim-constraint/src/parallel.rs`, `sim-constraint/src/newton.rs`

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
| `<default>` | ⚠️ Parsed only | `DefaultResolver` implemented in `defaults.rs` but **never called** by `model_builder.rs` — defaults are silently dropped during Model conversion |
| `<worldbody>` | Full | Body tree root |
| `<body>` | Full | Hierarchical bodies with pos, quat, euler |
| `<inertial>` | Full | mass, diaginertia, fullinertia |
| `<joint>` | Full | hinge, slide, ball, free types |
| `<geom>` | Partial | sphere, box, capsule, cylinder, ellipsoid, plane, mesh (convex + non-convex); hfield/sdf parsed but fall back to Box |
| `<site>` | Parsed | Markers (not used in physics) |
| `<actuator>` | Full | motor, position, velocity, cylinder, muscle, adhesion, damper, general |
| `<tendon>` | Parsed | Spatial and fixed tendons parsed but **dropped by model builder** (not carried to sim-core Model) |
| `<sensor>` | Parsed | 24 sensor types parsed but **dropped by model builder** (pipeline sensors populated from model builder defaults only) |
| `<contact>` | Partial | contype/conaffinity bitmasks on geoms work; `<pair>` and `<exclude>` sub-elements are **not parsed** |

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
- `hfield` → parsed, falls back to `CollisionShape::Box` (sim-core `HeightField` exists but not wired)
- `sdf` → parsed, falls back to `CollisionShape::Box` (sim-core `Sdf` exists but not wired)

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

> **The authoritative roadmap is [`sim/docs/FUTURE_WORK.md`](./FUTURE_WORK.md)**, which lists 11 items (4 completed, 7 remaining) with verified code references and acceptance criteria. The phases below reflect historical development milestones. Features marked with ⚠️ were later removed or remain standalone (not wired into the pipeline).

The following were completed in January 2026:

| Feature | Section | Notes |
|---------|---------|-------|
| Non-convex mesh collision | §5 Geoms | `CollisionShape::TriangleMesh` with BVH acceleration |
| SDF collision | §5 Geoms | All 10 shape combinations implemented |
| MJCF `<default>` element | §13 Model Format | ⚠️ `DefaultResolver` implemented but not called by `model_builder.rs` — defaults are parsed then dropped |
| MJCF `<tendon>` parsing | §13 Model Format | Spatial and fixed tendons |
| MJCF `<sensor>` parsing | §13 Model Format | 24 sensor types |

See [FUTURE_WORK.md](./FUTURE_WORK.md) for remaining items.

### ⚠️ Recently Completed then Partially Removed: Multi-threading

The `parallel` feature originally enabled multi-threaded constraint solving and body integration.
Island-parallel constraint solving (`solve_islands_parallel()`) and its dependencies (Newton solver, `islands.rs`) were **removed in Phase 3 consolidation**. The rayon dependency and `parallel` feature flag remain available for future batched simulation ([FUTURE_WORK #10](./FUTURE_WORK.md)).

**Files:** `sim-types/src/config.rs` (ParallelConfig). Removed: `sim-constraint/src/parallel.rs`, `sim-core/src/world.rs`, `sim-core/src/stepper.rs`

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
3. ~~**Elliptic friction cones**: Replace circular with elliptic~~ ✅ → ⚠️ **Stub** (`cone` field stored but solver hardcodes circular cone condim 3; see §3)
4. ~~**Sensors**: IMU, force/torque, touch sensors~~ ✅ (sim-sensor crate standalone + 27 pipeline sensor types functional)
5. ~~**Implicit integration**: Implicit-in-velocity method~~ ✅

### ⚠️ Phase 2: Solver Improvements (built then partially removed)

1. ~~**Newton solver**: For faster convergence~~ ✅ → ⚠️ **Removed** (Phase 3 consolidation — dead code, never called by pipeline)
2. ~~**Constraint islands**: For performance~~ ✅ → ⚠️ **Removed** (Phase 3 consolidation — depended on Newton solver)
3. ~~**Sleeping**: Deactivate stationary bodies~~ ✅
4. ~~**GJK/EPA**: For convex mesh collision~~ ✅

### ⚠️ Phase 3: Extended Features (standalone crates built, not in pipeline)

1. ~~**MJCF loading**: For MuJoCo model compatibility~~ ✅
2. ~~**Muscle actuators**: For biomechanics~~ ✅ → ⚠️ **Standalone** (sim-muscle crate exists, not wired into pipeline; see [FUTURE_WORK #5](./FUTURE_WORK.md))
3. ~~**Tendons**: For cable robots~~ ✅ → ⚠️ **Standalone** (sim-tendon crate exists, zero pipeline coupling; see [FUTURE_WORK #4](./FUTURE_WORK.md))
4. ~~**Deformables**: For soft body simulation~~ ✅ → ⚠️ **Standalone** (sim-deformable crate exists, not called from `Data::step()`; see [FUTURE_WORK #9](./FUTURE_WORK.md))

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

### Phase 6: Contact Physics ⚠️ REMOVED (Phase 3 consolidation)

Focus: Advanced friction models — **all items were implemented in sim-contact then removed** in Phase 3 consolidation. Pipeline PGS solver hardcodes circular cone condim 3.

| Feature | Section | Complexity | Notes |
|---------|---------|------------|-------|
| ~~Torsional friction~~ | §3 Contact | Medium | ✅ → ⚠️ **Removed** (was in `sim-contact/src/friction.rs`; pipeline is condim 3 only) |
| ~~Rolling friction~~ | §3 Contact | High | ✅ → ⚠️ **Removed** (was in `sim-contact/src/friction.rs`; pipeline is condim 3 only) |
| ~~Pyramidal friction cones~~ | §3 Contact | Low | ✅ → ⚠️ **Removed** (was in `sim-contact/src/friction.rs`; pipeline uses circular cone) |

**Implemented:**

**Torsional Friction (removed — formerly `sim-contact/src/friction.rs`):**
- `TorsionalFriction` - Resistance to spinning (rotation about contact normal)
- Configurable friction coefficient and contact radius
- Regularization for smooth force response near zero velocity
- `compute_torque()` - Compute torque opposing spinning motion
- `max_torque()`, `project_torque()` - Friction limit helpers

**Rolling Friction (removed — formerly `sim-contact/src/friction.rs`):**
- `RollingFriction` - Resistance to rolling (rotation perpendicular to normal)
- Configurable friction coefficient and rolling radius
- `compute_torque()` - Compute torque opposing rolling motion
- `compute_resistance_force()` - Equivalent linear resistance force
- Regularization for smooth force response

**Pyramidal Friction Cones (removed — formerly `sim-contact/src/friction.rs`, `model.rs`):**
- `PyramidalFrictionCone` - Linearized approximation to circular/elliptic cones
- Configurable number of faces (3-64, typical: 4, 8, or 16)
- Presets: `box_approximation()`, `octagonal()`, `high_accuracy()`
- `contains()`, `project()` - Cone containment and projection
- `constraint_matrix()` - Generate linear constraints for LCP/QP solvers
- `FrictionModelType::Pyramidal { num_faces }` - ContactModel integration

**Complete Friction Model (removed — formerly `sim-contact/src/friction.rs`):**
- `CompleteFrictionModel` - Unified tangential + torsional + rolling friction
- MuJoCo-style `condim()` method returning contact dimensionality (1, 3, 4, or 6)
- Factory methods: `tangential_only()`, `with_torsional()`, `complete()`
- Presets: `rubber_ball()`, `wheel()`, `sliding_box()`
- `CompleteFrictionResult` - Combined force and torque output

**Usage:**
```rust
// NOTE: These types were in sim-contact and have been removed.
// ContactPoint, ContactManifold, ContactForce now live in sim-core.
use sim_core::{ContactPoint, ContactManifold, ContactForce};
use nalgebra::Vector3;

// Torsional friction (spinning resistance)
let torsional = TorsionalFriction::new(0.05, 0.01); // μ=0.05, radius=1cm
let normal = Vector3::z();
let angular_vel = Vector3::new(0.0, 0.0, 5.0); // Spinning at 5 rad/s
let torque = torsional.compute_torque(&normal, &angular_vel, 100.0);

// Rolling friction
let rolling = RollingFriction::new(0.02, 0.05); // μ=0.02, radius=5cm
let angular_vel = Vector3::new(0.0, 10.0, 0.0); // Rolling about Y
let torque = rolling.compute_torque(&normal, &angular_vel, 100.0);

// Pyramidal friction cone (for LCP/QP solvers)
let pyramid = PyramidalFrictionCone::octagonal(0.5);
let force = Vector3::new(100.0, 0.0, 0.0);
let projected = pyramid.project(force, 100.0); // Project to cone
let (a, b) = pyramid.constraint_matrix(100.0); // Linear constraints

// Complete friction model (MuJoCo condim 6)
let model = CompleteFrictionModel::complete(
    0.5,   // tangential μ
    0.03,  // torsional μ
    0.01,  // contact radius
    0.02,  // rolling μ
    0.05,  // rolling radius
);
assert_eq!(model.condim(), 6);

let result = model.compute_force_and_torque(
    &normal,
    &Vector3::new(0.1, 0.0, 0.0), // tangent velocity
    &Vector3::new(0.0, 5.0, 2.0), // angular velocity (rolling + spinning)
    100.0, // normal force
);

// Use pyramidal cones in ContactModel
let contact_model = ContactModel::default()
    .with_pyramidal_friction(8); // 8-face pyramid
```

**Files:**
- Formerly `sim-contact/src/friction.rs`, `sim-contact/src/model.rs`, `sim-contact/src/lib.rs` — all removed
- Contact types (`ContactPoint`, `ContactManifold`, `ContactForce`) now in `sim-core/src/contact.rs`

### Phase 7: Actuators & Control ⚠️ STANDALONE (sim-constraint only — not in pipeline)

Focus: New actuator types and joint coupling in sim-constraint. **All items are standalone** — pipeline `mj_fwd_actuation()` only computes `ctrl * gear`.

| Feature | Section | Complexity | Notes |
|---------|---------|------------|-------|
| ~~Integrated velocity actuator~~ | §7 Actuators | Low | ✅ → ⚠️ **Standalone** (sim-constraint; pipeline uses `ctrl * gear` only) |
| ~~General custom actuator~~ | §7 Actuators | Medium | ✅ → ⚠️ **Standalone** (sim-constraint; not called by pipeline) |
| ~~Pneumatic cylinder actuator~~ | §7 Actuators | Medium | ✅ → ⚠️ **Standalone** (sim-constraint; not called by pipeline) |
| ~~Adhesion actuator~~ | §7 Actuators | Medium | ✅ → ⚠️ **Standalone** (sim-constraint; not called by pipeline) |
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
| ~~Conjugate Gradient solver~~ | §2 Solvers | Medium | ✅ COMPLETED → ⚠️ **Standalone** | Built in sim-constraint, 0 pipeline callers (see FUTURE_WORK #3) |
| ~~Tendon coupling constraints~~ | §10 Equality | Low | ✅ COMPLETED → ⚠️ **Standalone** | In sim-constraint, not wired into pipeline tendon system |
| ~~Flex edge constraints~~ | §10 Equality | Low | ✅ COMPLETED → ⚠️ **Standalone** | In sim-deformable, XPBD not called from pipeline |
| ~~SDF collision~~ | §4 Collision, §5 Geoms | High | ✅ **COMPLETED** | All 10 shape combinations implemented (see §5 notes) |
| ~~Skinned meshes~~ | §11 Deformables | High | ✅ **COMPLETED** | Visual deformation for rendering |
| ~~Multi-threading~~ | §12 Performance | Medium | ⚠️ **Reserved** | Island-parallel solving removed in Phase 3; rayon is an optional dep with no callers in code; see [FUTURE_WORK #10](./FUTURE_WORK.md) |
| ~~MJB binary format~~ | §13 Model Format | Low | ✅ **COMPLETED** | Faster loading via bincode serialization |

**Implemented (some now standalone or removed — see table above for current status):**

> **⚠️ Note:** The CG solver below is standalone (`sim-constraint`), not called by the pipeline. Tendon constraints are in `sim-constraint`, not wired into the pipeline tendon system. Island-parallel solving was removed in Phase 3 consolidation.

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

**Flex Edge Constraints (`sim-deformable/src/constraints.rs`) — ⚠️ Standalone:**
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

> **📌 Note:** For the current authoritative roadmap with verified code references, see [`sim/docs/FUTURE_WORK.md`](./FUTURE_WORK.md). The priority roadmap above is historical and partially outdated.

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
| `sim-deformable` | Soft body simulation (⚠️ standalone) | `capsule_chain.rs`, `cloth.rs`, `soft_body.rs`, `solver.rs`, `constraints.rs`, `skinning.rs`, `material.rs`, `mesh.rs` |
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
