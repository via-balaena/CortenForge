# Simulation Domain Architecture

## Overview

The `sim-*` crates provide a MuJoCo-aligned physics simulation stack for:

- **Robotics training** (reinforcement learning with sim-to-real transfer)
- **Headless simulation** (no Bevy dependency — Layer 0)
- **Deterministic execution** (fixed iteration counts, reproducible results)
- **Domain randomization** (tunable contact parameters for robust policies)

The primary API is a `Model`/`Data` architecture modeled after MuJoCo:
`Model` is static (immutable after loading), `Data` is dynamic (`qpos`/`qvel`
are the source of truth), and body poses are computed via forward kinematics.

## Directory Structure

```
sim/
├── L0/                    # Layer 0: Bevy-free simulation
│   ├── types/             # sim-types — Pure data types
│   ├── simd/              # sim-simd — SIMD batch operations
│   ├── core/              # sim-core — Pipeline, collision, integration
│   ├── constraint/        # sim-constraint — Joint types, motors, limits, CGSolver
│   ├── sensor/            # sim-sensor — IMU, F/T, touch, rangefinder
│   ├── muscle/            # sim-muscle — Hill-type muscles
│   ├── tendon/            # sim-tendon — Cable/tendon routing
│   ├── gpu/               # sim-gpu — GPU-accelerated batched sim (wgpu)
│   ├── mjcf/              # sim-mjcf — MuJoCo format parser
│   ├── urdf/              # sim-urdf — URDF parser
│   ├── physics/           # sim-physics — Unified L0 API
│   └── tests/             # sim-conformance-tests — Model loading tests
├── L1/                    # Layer 1: Bevy integration
│   └── bevy/              # sim-bevy — Visualization
└── docs/                              # Simulation documentation
    ├── ARCHITECTURE.md                # This file
    ├── todo/                           # Roadmap and remaining work
    │   ├── index.md                   # Priority table, dependency graph, file map
    │   ├── future_work_1.md           # Phase 1 (complete)
    │   ├── future_work_2.md           # Phase 2: Correctness #1–5
    │   ├── future_work_3.md           # Phase 2: Correctness + Scaling #6–10
    │   ├── future_work_4.md           # Phase 2: Physics Completeness #11–14
    │   ├── future_work_5.md           # Phase 2: Quality of Life #15–16
    │   ├── future_work_6.md           # Phase 3A-i: Parser Fundamentals #18–22
    │   ├── future_work_6b_precursor_to_7.md # Flex Solver Unification #6B
    │   ├── future_work_7.md           # Phase 3A-ii: Inertia + Contact Parameters #23–27
    │   ├── future_work_8.md           # Phase 3A-iii: Constraint System Overhaul #28–32
    │   ├── future_work_9.md           # Phase 3A-iv: Noslip + Actuator/Dynamics #33–37
    │   ├── future_work_10.md          # Phase 3A-v: Constraint/Joint + Physics #38–42
    │   ├── future_work_11.md          # Phase 3A-vi: Cleanup + Conformance #43–45
    │   ├── future_work_12.md          # Phase 3B: Format Completeness + Performance #46–50
    │   ├── future_work_13.md          # Phase 3C: API + Pipeline Completeness #51–55
    │   ├── future_work_14.md          # Phase 3C: Data Fields + Derivatives + API #56–59
    │   ├── future_work_15.md          # Phase 3D: Edge-Case Features #60–64
    │   ├── future_work_16.md          # Phase 3D: Mesh + Plugin Infrastructure #65–66
    │   └── future_work_17.md          # Phase 3E: GPU Pipeline #67–71
    ├── TRAIT_ARCHITECTURE.md            # Trait boundary vision (modeling choices vs solver variants vs core math)
    ├── MUJOCO_CONFORMANCE.md          # MuJoCo conformance testing plan
    ├── MUJOCO_GAP_ANALYSIS.md         # Feature-by-feature gap analysis
    ├── MUJOCO_REFERENCE.md            # Pipeline algorithms and pseudocode
    └── SIM_BEVY_IMPLEMENTATION_PLAN.md # Bevy visualization layer plan
```

## Core Architecture: Model/Data

The simulation engine lives in `sim-core/src/mujoco_pipeline.rs` (~24,800 lines).
It implements MuJoCo's computation pipeline end-to-end.

### Model (static)

Immutable after loading. Contains the kinematic tree, joint definitions,
geometry specifications, actuator configuration, and solver parameters.

Key fields:
- `nq`/`nv` — position coordinates vs velocity DOFs (differ for quaternion joints); includes flex vertex DOFs when `nflex > 0`
- `nq_rigid`/`nv_rigid` — rigid-only DOF counts (before flex DOFs in the state vector)
- `nflex`/`nflexvert`/`nflexedge`/`nflexelem`/`nflexhinge` — flex body dimensions
- `nu`/`na` — actuator count, total activation dimension
- `body_parent[i]` — kinematic tree topology
- `body_pos[i]`/`body_quat[i]` — local frame offsets
- `body_mass[i]`/`body_inertia[i]` — mass properties
- `jnt_type[i]` — `MjJointType` (Hinge, Slide, Ball, Free)
- `jnt_stiffness[i]`/`jnt_damping[i]`/`jnt_springref[i]` — passive dynamics
- `jnt_solref[i]`/`jnt_solimp[i]` — constraint solver parameters
- `actuator_dyntype[i]` — `ActuatorDynamics` (None, Filter, FilterExact, Integrator, Muscle)
- `actuator_gaintype[i]` — `GainType` (Fixed, Affine, Muscle) — dispatches Phase 2 gain computation
- `actuator_biastype[i]` — `BiasType` (None, Affine, Muscle) — dispatches Phase 2 bias computation
- `actuator_dynprm[i]`/`actuator_gainprm[i]`/`actuator_biasprm[i]` — dynamics and force parameters
- `timestep`, `gravity`, `integrator`, `solver_type`, `solver_iterations`, `solver_tolerance`
- `nmocap`, `body_mocapid[i]` — mocap body count and body→mocap index mapping
- `nkeyframe`, `keyframes` — named state snapshots for quick reset
- `ntree`, `tree_body_adr[t]`/`tree_body_num[t]`/`tree_dof_adr[t]`/`tree_dof_num[t]` — kinematic tree enumeration
- `body_treeid[i]`, `dof_treeid[d]` — body→tree and DOF→tree mapping
- `tree_sleep_policy[t]` — per-tree sleep policy (`Auto`→`AutoNever`/`AutoAllowed`, or user `Never`/`Allowed`/`Init`)
- `sleep_tolerance` — velocity threshold for sleeping (default `1e-4` m/s)
- `dof_length[d]` — per-DOF length scale for threshold normalization

Constructed from MJCF via `sim-mjcf` or URDF via `sim-urdf`.

### Data (mutable)

Pre-allocated state and computed quantities. Most buffers are pre-allocated;
residual heap allocation occurs for contact vector growth and RK4 warmstart save.

| Category | Fields | Description |
|----------|--------|-------------|
| State | `qpos`, `qvel`, `act`, `ctrl`, `time` | Source of truth (act = activation states for muscles/filters) |
| Mocap state | `mocap_pos`, `mocap_quat` | User-settable kinematic input for mocap bodies (length nmocap); FK overrides body pose |
| Body poses | `xpos`, `xquat`, `xmat` | Computed by forward kinematics |
| Mass matrix | `qM`, `qLD_data`, `qLD_diag_inv` | Computed by CRBA; sparse L^T D L factorization in flat CSR storage (diagonal D[i,i] stored as the last element of each CSR row in `qLD_data`; `qLD_diag_inv` stores precomputed 1/D[i,i] for fast solves) |
| Forces | `qfrc_bias`, `qfrc_passive`, `qfrc_actuator`, `qfrc_applied`, `qfrc_constraint` | Generalized force components |
| Actuation | `actuator_length`, `actuator_velocity`, `actuator_force`, `act_dot` | Actuator-space state and activation derivatives |
| Acceleration | `qacc` | Computed as `M^-1 * f_total` |
| Contacts | `contacts` | Active contacts (constraint forces in `efc_force` array) |
| Derivatives | `qDeriv`, `deriv_Dcvel`, `deriv_Dcacc`, `deriv_Dcfrc` | Analytical `∂(qfrc_smooth)/∂qvel` (nv×nv) and per-body chain-rule Jacobians (6×nv each) |
| Sleep state | `tree_asleep`, `tree_awake`, `body_sleep_state`, `ntree_awake`, `nv_awake` | Per-tree sleep timer/flag, per-body sleep state, awake counts |
| Awake indices | `body_awake_ind`, `dof_awake_ind`, `parent_awake_ind` | Sorted index arrays for cache-friendly awake-only iteration |
| Islands | `nisland`, `tree_island`, `island_ntree`, `dof_island`, `contact_island` | Constraint island structure from DFS flood-fill |

### Stepping

```rust
use sim_core::{Model, Data};
use sim_mjcf::{load_model, load_model_from_file};

let model = load_model(mjcf_string)?;  // Static (from MJCF XML string)
// Or from file (supports <include> file resolution):
// let model = load_model_from_file("robot.xml")?;
let mut data = model.make_data();      // Pre-allocated

// Simulation loop
loop {
    data.step(&model)?;
    // data.qpos, data.xpos, etc. now updated
}
```

`Data::step()` dispatches by integrator: for Euler, ImplicitSpringDamper,
ImplicitFast, and Implicit it calls `forward()` then `integrate()`; for RK4 it
calls `forward()` then `mj_runge_kutta()` (a true 4-stage Runge-Kutta that
re-evaluates dynamics at each stage). See `MUJOCO_REFERENCE.md` for the
complete pipeline algorithm.

## Physics Pipeline

Each timestep executes these stages in order:

```
forward():
  Sleep        mj_wake                Check user forces on sleeping bodies
               mj_wake_collision      Check contacts between sleeping/awake bodies
               mj_wake_tendon         Tendons coupling sleeping ↔ awake trees
               mj_wake_equality       Equality constraints to awake trees
               mj_sleep               Sleep state machine (countdown → sleep transition)
               mj_island              Island discovery (DFS flood-fill over constraints)
  Position     mj_fwd_position       FK from qpos → body poses (skips sleeping bodies)
               mj_fwd_position_flex  Flex vertex positions from qpos
               mj_fwd_tendon         Tendon lengths + Jacobians (fixed + spatial)
               mj_collision           Broad + narrow phase contacts (skips sleeping pairs)
                                     + mj_collision_flex (vertex-vs-geom, brute-force O(V*G))
  Velocity     mj_fwd_velocity        Body spatial velocities (skips sleeping DOFs)
               mj_actuator_length     Actuator length/velocity from transmission
  Actuation    mj_fwd_actuation       act_dot computation + gain/bias force + clamping
  Dynamics     mj_crba                Selective CRBA (skips sleeping subtrees)
               mj_crba_flex           Flex diagonal mass matrix
               mj_factor_flex         Flex LDL factorization (diagonal)
               mj_rne                 Bias forces (Recursive Newton-Euler) + flex gravity
               mj_fwd_passive         Springs, dampers, friction loss, flex bending + vertex damping
  Constraints  mj_fwd_constraint      Unified constraint assembly + PGS/CG/Newton solve
               mj_fwd_constraint_islands  Per-island block-diagonal solving (when islands > 1)
  Solve        mj_fwd_acceleration    qacc = M^-1 * f  (or implicit solve)
integrate() [Euler / ImplicitFast / Implicit / ImplicitSpringDamper]:
  Activation integration (act += dt * act_dot, muscle clamp to [0,1])
  Semi-implicit Euler (velocity first, then position with new velocity)
  Quaternion integration on SO(3) for ball/free joints (skips sleeping joints)
  ImplicitFast: (M − h·D_sym) · qacc = f, Cholesky (D = passive + actuator vel)
  Implicit: (M − h·D) · qacc = f, LU with partial pivot (D includes Coriolis)
mj_runge_kutta() [RungeKutta4]:
  True 4-stage RK4 with Butcher tableau [1/6, 1/3, 1/3, 1/6]
  Integrates activation alongside qpos/qvel with same RK4 weights
  Stage 0 reuses initial forward(); stages 1-3 call forward_skip_sensors()
  Uses mj_integrate_pos_explicit() for quaternion-safe position updates
  Sleep is disabled for RK4 (warning emitted if both enabled)
```

**Derivative computation** (optional, after `forward()`):

```
mjd_smooth_vel():
  Zeros data.qDeriv, then accumulates:
    mjd_passive_vel        ∂(qfrc_passive)/∂qvel (diagonal damping + tendon rank-1)
    mjd_actuator_vel       ∂(qfrc_actuator)/∂qvel (affine gain/bias velocity terms)
    mjd_rne_vel            −∂(qfrc_bias)/∂qvel (chain-rule RNE + direct gyroscopic)
mjd_transition_fd():
  Pure FD Jacobians: A = ∂x⁺/∂x, B = ∂x⁺/∂u via centered/forward differences
  Tangent-space perturbation for quaternion joints (Ball, Free)
mjd_transition_hybrid():
  Analytical velocity/activation columns + FD position columns
  Euler: I + h·M⁻¹·qDeriv via sparse LDL
  Implicit: (M+hD+h²K)⁻¹·(M+h·(qDeriv+D)) via Cholesky
  ~nv FD calls (position columns only) vs 2·(2nv+na+nu) for pure FD
mjd_transition():
  Public dispatch: FD-only or hybrid based on DerivativeConfig.use_analytical
```

**Integration methods** (`Integrator` enum):
- `Euler` (default) — semi-implicit Euler, velocity-first
- `RungeKutta4` — true 4th-order Runge-Kutta (O(h⁴) global error);
  uses `forward_skip_sensors()` for intermediate stage evaluations and
  `mj_integrate_pos_explicit()` for quaternion-safe position updates
- `ImplicitSpringDamper` — unconditionally stable for stiff springs/dampers; solves
  `(M + h*D + h^2*K) v_new = M*v_old + h*f_ext - h*K*(q - q_eq)`
- `Implicit` — full implicit with asymmetric D (includes Coriolis velocity derivatives)
  and LU factorization with partial pivoting; maximum accuracy
- `ImplicitFast` — fast implicit with symmetric D (skips Coriolis terms) and Cholesky
  factorization; faster than `Implicit` with nearly identical results for most models

**Constraint enforcement (unified):**
All constraint types (equality, friction loss, limits, contacts, flex edge) are
routed through unified solver rows assembled by `assemble_unified_constraints()`.
Every solver operates on the same constraint Jacobian + RHS, selectable via
`<option solver="Newton"/>` in MJCF or `model.solver_type = SolverType::Newton`:
- **PGS** — Dual-space Gauss-Seidel on regularized Delassus matrix AR
- **CG** — Primal Polak-Ribiere conjugate gradient sharing `mj_sol_primal` with
  Newton, M^-1 preconditioner
- **Newton** — Primal solver with H^-1 preconditioner, analytical second-order
  derivatives, sparse LDL factorization

## Collision Detection

### Pipeline

MuJoCo's two-mechanism architecture:

**Mechanism 1 (automatic pipeline):**

| Phase | Method | Output |
|-------|--------|--------|
| Broad | Sweep-and-prune on AABBs | Candidate pairs |
| Filter | Body-pair excludes, explicit pair-set suppression, same-body, parent-child, contype/conaffinity bitmasks | Filtered pairs |
| Narrow | Per-pair geometry tests | Contact points |

**Mechanism 2 (explicit `<pair>` pipeline):**

Explicit `<pair>` entries bypass all kinematic and bitmask filters. They go
through a bounding-sphere distance cull and narrow-phase, then
`apply_pair_overrides` applies per-pair condim/friction/solref/solimp.

### Narrow Phase Dispatch

Analytical algorithms for common pairs (sphere-sphere, sphere-capsule,
capsule-capsule, box-box via SAT, sphere-box, capsule-box, etc.).
GJK/EPA fallback for remaining convex pairs. BVH-accelerated tests for
triangle meshes.

### Supported Geometry Types

| Type | `GeomType` | Notes |
|------|------------|-------|
| Sphere | `Sphere` | Analytical collisions |
| Box | `Box` | SAT for box-box |
| Capsule | `Capsule` | Analytical line-line distance |
| Cylinder | `Cylinder` | GJK/EPA fallback for some pairs |
| Ellipsoid | `Ellipsoid` | GJK/EPA |
| Plane | `Plane` | Infinite half-space (ground) |
| Mesh | `Mesh` | BVH-accelerated triangle tests |

sim-core also provides collision primitives usable outside the pipeline:

| Shape | Description |
|-------|-------------|
| `CollisionShape::HeightField` | Terrain (2D height grid) |
| `CollisionShape::Sdf` | Signed distance field queries |
| `CollisionShape::TriangleMesh` | Non-convex triangle soup |
| `CollisionShape::ConvexMesh` | GJK/EPA-based convex hull |

Supporting modules: `mid_phase.rs` (BVH construction and traversal),
`gjk_epa.rs` (convex collision), `mesh.rs` (triangle tests), `heightfield.rs`,
`sdf.rs`, `raycast.rs`.

## Crate Reference

### sim-types

Pure data structures with no physics logic. Minimal dependencies: nalgebra, glam, thiserror.

`RigidBodyState`, `Pose`, `Twist`, `MassProperties`, `Action`, `ExternalForce`,
`JointState`, `JointCommand`, `SimulationConfig`, `BodyId`, `JointId`.

### sim-simd

SIMD-optimized batch operations for performance-critical paths.
`Vec3x4`/`Vec3x8` batched vectors, `find_max_dot`, `batch_dot_product_4`,
`batch_aabb_overlap_4`, `batch_normal_force_4`, `batch_integrate_position_4`/
`batch_integrate_velocity_4`. **Note:** only `find_max_dot` is currently called
by sim-core (GJK); all other batch ops are benchmarked but have no callers.
2-4x speedup on x86-64, 2-3x on Apple Silicon.

### sim-core

The physics engine. Depends on sim-types and sim-simd. Contains:

- `mujoco_pipeline.rs` — `Model`, `Data`, full MuJoCo-aligned pipeline
- `derivatives.rs` — Simulation transition derivatives (FD, analytical qDeriv, hybrid FD+analytical, SO(3) Jacobians, validation utilities)
- `collision_shape.rs` — `CollisionShape` enum, `Aabb`
- `mid_phase.rs` — BVH tree (median-split construction, traversal, ray queries)
- `gjk_epa.rs` — GJK/EPA for convex shapes
- `mesh.rs` — Triangle mesh collision functions
- `heightfield.rs`, `sdf.rs` — Terrain and implicit surface collision
- `raycast.rs` — Ray-shape intersection
- `batch.rs` — `BatchSim`: N independent `Data` environments sharing one `Arc<Model>`, parallel stepping via rayon (`parallel` feature)

### Contact Types (in sim-core)

`ContactPoint`, `ContactManifold`, and `ContactForce` live in
`sim-core/src/contact.rs`. These represent collision geometry output
(position, normal, penetration, body pair) and resulting forces.
The MuJoCo pipeline uses unified PGS/CG/Newton solvers in `mujoco_pipeline.rs`
with variable condim (1/3/4/6) and elliptic friction cones.

### sim-constraint

Joint types, motors, limits, and constraint solver for articulated body simulation:

**Joint types** (all implement `Joint` trait):

| Joint | DOF | qpos | qvel |
|-------|-----|------|------|
| Fixed | 0 | — | — |
| Revolute (Hinge) | 1 | angle | angular vel |
| Prismatic (Slide) | 1 | displacement | linear vel |
| Universal | 2 | 2 angles | 2 angular vel |
| Cylindrical | 2 | angle + disp | angular + linear |
| Planar | 3 | x, y, angle | vx, vy, omega |
| Spherical (Ball) | 3 | quaternion (4) | angular vel (3) |
| Free | 6 | pos + quat (7) | lin + ang vel (6) |

Also provides: `JointLimits`, `JointMotor`, `MotorMode`, equality constraints
(connect, gear coupling, differential, tendon networks), actuator types,
and `CGSolver` (Conjugate Gradient solver with Block Jacobi preconditioner —
standalone library for joint-space constraints; see [future_work_1 #3](./todo/future_work_1.md)).
The pipeline's unified constraint solver (PGS/CG/Newton) lives in
`mujoco_pipeline.rs` and operates on unified constraint rows assembled by
`assemble_unified_constraints()`.

### sim-sensor

Simulated sensor suite: `Imu` (6-axis accel + gyro), `ForceTorqueSensor`
(6-axis), `TouchSensor` (binary/pressure), `Rangefinder` (ray-cast),
`Magnetometer` (heading).

### Flex (Deformable) Bodies

Deformable bodies are unified into the rigid pipeline as flex bodies (matching
MuJoCo's flex architecture). No separate crate — all flex code lives in
`sim-core/mujoco_pipeline.rs` and `sim-mjcf/model_builder.rs`.

| Dimension | Use Cases | MJCF Element |
|-----------|-----------|--------------|
| 1D (cable) | Ropes, cables | `<flexcomp type="cable">` |
| 2D (shell) | Membranes, cloth, shells | `<flexcomp type="grid">` |
| 3D (solid) | Tetrahedral volumetric meshes | `<flexcomp type="box">` |

**Architecture:** Flex vertex DOFs are appended to `qpos`/`qvel`/`qacc` after
rigid DOFs. Edge constraints enter the unified Jacobian as `FlexEdge` rows.
Bending acts as passive spring-damper forces in `mj_fwd_passive()` (matching
MuJoCo `engine_passive.c`). Flex-rigid contacts are regular `Contact` entries
(discriminated by `flex_vertex: Option<usize>`).

**Key pipeline functions:** `mj_collision_flex()` (brute-force vertex-vs-geom),
`mj_crba_flex()` / `mj_factor_flex()` (diagonal mass + LDL),
`mj_integrate_pos_flex()` (position integration),
`mj_fwd_position_flex()` (vertex position update from qpos).

**Material model:** Young's modulus, Poisson's ratio, density, thickness,
damping. Bending stiffness derived from material (Kirchhoff-Love for shells,
characteristic stiffness for solids). Pinned vertices use `mass = 1e20`,
`invmass = 0`.

**Parsed from MJCF:** `<flex>` (direct vertex/element specification) and
`<flexcomp>` (procedural generation: grid, box, cable types).

See [future_work_6b](./todo/future_work_6b_precursor_to_7.md) for the full
specification. The previous `sim-deformable` crate (XPBD solver) has been
deleted — useful code migrated to `model_builder.rs`. The bending and
elasticity models are being refactored into trait boundaries — see
[TRAIT_ARCHITECTURE.md](./TRAIT_ARCHITECTURE.md) for the vision.

### sim-muscle

Hill-type muscle model:

```
F = activation * F_max * f_length(l) * f_velocity(v) + F_passive(l)
```

Three-element model: contractile (CE), parallel elastic (PE), series elastic (SE).
Activation dynamics with asymmetric time constants, pennation angle support.
Predefined configs: biceps, quadriceps, gastrocnemius, soleus.
`MuscleGroup` for antagonist pairs.

### sim-tendon

Standalone cable-driven actuation and routing library:

- **Fixed tendons** — MuJoCo-style linear joint couplings
- **Spatial tendons** — 3D routing through attachment points with wrapping
  geometry (sphere, cylinder) and pulley systems

`TendonActuator` trait: `rest_length`, `compute_length`, `compute_velocity`,
`compute_force`, `jacobian`, `num_joints` (6 methods).

**Note:** Both fixed and spatial tendons are implemented directly in the MuJoCo
pipeline (`mj_fwd_tendon` in sim-core). Spatial tendons include sphere and
cylinder wrapping, sidesite disambiguation, pulley divisors, and Jacobian
computation via `accumulate_point_jacobian()`. This crate remains a standalone
reference library for advanced tendon analysis.

### sim-mjcf

MuJoCo XML format parser. Supports: bodies, joints (hinge, slide, ball, free),
geoms (sphere, box, capsule, cylinder, ellipsoid, plane, mesh), actuators
(motor, position, velocity, general, muscle, cylinder, damper, adhesion),
contype/conaffinity contact bitmasks, `<contact>` `<pair>`/`<exclude>` elements
(two-mechanism collision architecture with per-pair parameter overrides),
default class inheritance with full Option<T> defaults system (`DefaultResolver`
with four-stage pipeline: types → parser → merge → apply; 91+ defaultable
fields across 8 element types), `childclass` attribute (body/frame recursive
propagation with undefined-class validation), `<frame>` element (pose
composition, childclass inheritance, recursive nesting),
`<include>` file support, `<compiler>` element, and MJB binary format.
`<include>` resolves file references as a pre-parse XML expansion step with
recursive nested includes, duplicate file detection, and path resolution
relative to the main model file. Works inside any MJCF section (`<worldbody>`,
`<asset>`, `<actuator>`, `<default>`, etc.); duplicate top-level sections are
merged. `<compiler>` controls angle units (`angle`), Euler sequence
(`eulerseq`), asset path resolution (`meshdir`/`texturedir`/`assetdir`),
automatic limit inference (`autolimits`), inertia computation
(`inertiafromgeom`), mass post-processing (`boundmass`/`boundinertia`,
`balanceinertia`, `settotalmass`), and model simplification (`strippath`,
`discardvisual`, `fusestatic`).
`<tendon>` and `<sensor>` elements are parsed and wired into the pipeline
(fixed and spatial tendons fully supported, including sphere/cylinder wrapping,
sidesite disambiguation, and pulley divisors;
all 32 pipeline sensor types functional and wired from MJCF via
`process_sensors()` in `model_builder.rs`). The model builder expands all 8 actuator
shortcut types to their general gain/bias/dynamics representation (matching
MuJoCo's `user_api.cc`), populating `actuator_gaintype`, `actuator_biastype`,
`actuator_gainprm`, `actuator_biasprm`, and `actuator_dynprm` per actuator.
`<general>` actuators support explicit `gaintype`, `biastype`, `dyntype`,
`gainprm`, `biasprm`, `dynprm` attributes with default class inheritance.
Muscle parameters are parsed and transferred; `compute_muscle_params()`
resolves `lengthrange`, `acc0`, and auto-computes `F0` at model build time.
`<flex>` and `<flexcomp>` elements are parsed via `parse_flex()` /
`parse_flexcomp()` and processed by `process_flex()` in the model builder,
which computes edge/hinge topology, bending stiffness from material properties,
and element rest volumes.

### sim-urdf

URDF robot description parser. Converts URDF → MJCF via sim-mjcf. Supports:
links with mass, joints (fixed, revolute, continuous, prismatic, floating),
geometry (box, sphere, cylinder; mesh parsed but not converted), dynamics
(damping; friction parsed but not converted), limits. **Planar joints are
lossy** — approximated as a single hinge (loses 2 of 3 DOF).

### sim-gpu

GPU-accelerated batched simulation via wgpu compute shaders. Drop-in
replacement for `BatchSim` that offloads the integration step to GPU.
Dependencies: sim-core (with `gpu-internals` feature), wgpu, bytemuck,
pollster, thiserror, tracing. Optional rayon (behind `parallel` feature).

- `GpuBatchSim` — wraps `BatchSim` with GPU Euler velocity integration
- `GpuSimContext` — `OnceLock<Option<>>` singleton for device/queue init
- `GpuEnvBuffers` — Structure-of-Arrays f32 GPU buffers with size validation
- `GpuParams` — `#[repr(C)]` uniform buffer matching WGSL struct layout
- `euler_integrate.wgsl` — compute shader: `qvel += qacc * h` per DOF per env

**Phase 10a scope:** Only Euler velocity integration on GPU. FK, collision,
constraints, dynamics remain on CPU. Transparent fallback via `try_new()`.
See [future_work_3 #10](./todo/future_work_3.md) for remaining phases.

### sim-physics

Unified L0 API re-exporting all simulation crates:

```rust
use sim_physics::prelude::*;
// Gives access to: Model, Data, CollisionShape, MjJointType,
// all joint types, contact types, muscle/tendon/sensor types, etc.
```

### sim-bevy (Layer 1)

Bevy visualization layer for physics debugging:

| Component | Description |
|-----------|-------------|
| `SimViewerPlugin` | Main plugin — orbit camera, lighting, mesh generation |
| `ModelDataPlugin` | Syncs `Model`/`Data` into Bevy ECS |
| `BodyEntityMap` | Bidirectional physics-body ↔ Bevy-entity mapping |
| `CachedContacts` | Last frame's contacts for debug rendering |
| `ViewerConfig` | Toggle visibility of contacts, forces, axes, etc. |

**Coordinate system:** sim-core uses Z-up (robotics), Bevy uses Y-up (graphics).
Conversions centralized in `convert.rs`: `(x, y, z)_physics → (x, z, y)_bevy`.

Debug gizmos: contact points, contact normals, muscle/tendon paths, sensor
readings. All toggleable via `ViewerConfig`. Force vectors and joint axes are
declared in `ViewerConfig` but not yet implemented (no drawing systems).

## Sleeping / Body Deactivation

Tree-based sleeping system matching MuJoCo's deactivation model. Stationary
bodies are detected, grouped into islands, and excluded from computation.

### Architecture

Bodies are organized into **kinematic trees** (connected components of the
`body_parent` graph). Trees are the unit of sleep: all DOFs in a tree sleep
or wake together. Trees are grouped into **constraint islands** via DFS
flood-fill over contact/tendon/equality coupling. Islands are the unit of
sleep *decisions*: if any tree in an island must wake, all trees wake.

### Sleep Policy

Per-tree policy resolved at model build time:

| Policy | Source | Behavior |
|--------|--------|----------|
| `AutoNever` | Compiler: actuated tree or multi-tree tendon | Never sleeps |
| `AutoAllowed` | Compiler: no actuators or coupling | May sleep |
| `Never` | MJCF: `sleep="never"` | User override: never sleeps |
| `Allowed` | MJCF: `sleep="allowed"` | User override: may sleep |
| `Init` | MJCF: `sleep="init"` | Starts asleep; validated via union-find |

Enabled via `<option><flag sleep="enable"/>` (maps to `ENABLE_SLEEP` bit).

### Sleep State Machine

Each tree tracks a countdown timer (`tree_asleep`):

1. **Awake** (`tree_asleep < 0`): velocity checked each step against
   `sleep_tolerance * dof_length[d]`. If all DOFs below threshold,
   countdown advances toward `-1`.
2. **Transition** (`tree_asleep == -1`): after `MIN_AWAKE` (10) consecutive
   sub-threshold steps, tree enters sleep. Velocities, accelerations, and
   force caches are zeroed.
3. **Asleep** (`tree_asleep >= 0`): tree participates in sleep-cycle
   linked list (Phase B). No computation until woken.

### Wake Detection

Sleeping bodies are woken by:
- **User forces**: nonzero `xfrc_applied` or `qfrc_applied` (bytewise check)
- **Contact**: sleeping body contacts awake body
- **Tendon**: active limited tendon coupling sleeping ↔ awake trees
- **Equality**: active constraint to an awake tree
- **qpos change**: external modification of sleeping body's `qpos`

Wake propagates to all trees in the same constraint island.

### Pipeline Skip Logic

When sleep is enabled, pipeline stages skip sleeping bodies/DOFs:
- FK: poses frozen (not recomputed)
- Collision: narrow-phase skipped when both geoms are asleep
- Velocity kinematics: sleeping DOFs skipped
- Passive forces: skipped when all target DOFs are asleep
- Position/velocity integration: sleeping joints skipped
- Sensors: return frozen values (not zeroed)

### Performance Optimizations (Phase C)

Three optimizations reduce work proportional to the awake fraction:

1. **Awake-index iteration**: `body_awake_ind`, `dof_awake_ind`,
   `parent_awake_ind` arrays enable O(awake) loops instead of O(total)
   with per-body branch skipping.
2. **Island-local Delassus**: when multiple islands exist,
   `mj_fwd_constraint_islands` builds small per-island mass matrices
   and solves independently via block-diagonal decomposition.
3. **Selective CRBA + Partial LDL**: `mj_factor_sparse_selective`
   skips sleeping subtrees in composite-inertia accumulation and
   factorizes only awake DOF blocks. Sleeping DOFs retain their
   last-awake `qM`/`qLD` values (tree independence guarantees no
   cross-contamination).

### MJCF Configuration

```xml
<option sleep_tolerance="1e-4">
  <flag sleep="enable"/>
</option>
<body name="box" sleep="allowed">
  ...
</body>
```

### Tests

93 integration tests in `sleeping.rs` covering all three phases:
Phase A (per-tree sleeping), Phase B (island discovery + cross-tree coupling),
Phase C (selective CRBA, partial LDL, awake-index iteration, island-local solving).

## Design Principles

### Contact Solver

The pipeline supports three constraint solvers selectable via `model.solver_type`.
All solvers share `assemble_unified_constraints()` which builds a unified constraint
Jacobian + RHS for ALL constraint types (equality, friction loss, limits, contacts,
flex edge). There is no separate contact-only assembly path.

- **PGS** (programmatic default) — Dual-space Gauss-Seidel on regularized Delassus
  matrix AR. Uses per-row inline elliptic friction cone projection inside the GS
  sweep. Warmstarts from `qacc_warmstart` via `classify_constraint_states`.
- **CG** — Primal Polak-Ribiere conjugate gradient sharing `mj_sol_primal` with
  Newton. Uses M^-1 preconditioner. Falls back to PGS on non-convergence,
  reusing the pre-assembled constraint system.
- **Newton** (MJCF default, matching MuJoCo) — Primal solver with H^-1
  preconditioner and analytical second-order derivatives. Sparse LDL
  factorization. Shares `mj_sol_primal` infrastructure with CG. Converges in
  2-3 iterations vs PGS's 20+. Falls back to PGS on Cholesky failure or
  non-convergence.

**Variable Contact Dimensions (condim):**
- condim 1: Normal force only (frictionless contact)
- condim 3: Normal + 2D tangential (sliding friction)
- condim 4: condim 3 + torsional friction (spinning resistance via `apply_contact_torque()`)
- condim 6: condim 4 + rolling friction (full MuJoCo model)

Each contact stores `Contact.dim` and `Contact.mu: [f64; 5]` (sliding1, sliding2, torsional,
rolling1, rolling2). The `efc_offsets` array tracks each contact's starting row in the
variable-size constraint system.

**Elliptic Friction Cones:**
Projection uses a two-step physically-correct algorithm: (1) enforce unilateral constraint
lambda_n >= 0 (else release contact), (2) scale friction components to cone boundary if
`||(lambda_i/mu_i)|| > lambda_n`. This handles anisotropic friction (different coefficients
per direction).

Contact geometry types (`ContactPoint`, etc.) are in `sim-core/src/contact.rs`.

Solver selection: `<option solver="CG"/>` in MJCF or `model.solver_type = SolverType::CG`.

### Determinism

All solvers use a fixed iteration cap (`solver_iterations`, default 100) with an
early-exit tolerance. Same inputs produce same outputs. Fixed computational cost is
essential for RL training.

### Layer 0 (No Bevy)

All `sim-*` crates in `L0/` have zero Bevy dependencies. They run in headless
training loops, deploy to hardware, and integrate with other engines. `sim-bevy`
is Layer 1 only.

## Feature Flags

| Flag | Crates | Description |
|------|--------|-------------|
| `parallel` | sim-core, sim-gpu | Rayon-based parallelization for `BatchSim::step_all()` and `GpuBatchSim::step_all()` CPU forward pass. Sequential fallback when disabled. See [future_work_3 #9](./todo/future_work_3.md) |
| `gpu-internals` | sim-core | Exposes internal helpers (`integrate_without_velocity`, `envs_as_mut_slice`, `model_arc`) for sim-gpu. No additional deps. |
| `serde` | Most crates | Serialization support |
| `mjb` | sim-mjcf | Binary MuJoCo format |
| `muscle` | sim-constraint | Hill-type muscle integration |

## References

- [MuJoCo Documentation](https://mujoco.readthedocs.io/en/stable/modeling.html#contact)
- [MuJoCo Technical Notes](https://mujoco.readthedocs.io/en/stable/computation.html)
- Todorov, E. (2014). "Convex and analytically-invertible dynamics with contacts and constraints"
- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer.
- Bridson, R. et al. (2003). "Simulation of Clothing with Folds and Wrinkles" (dihedral bending gradient)
- Hill, A.V. (1938). "The heat of shortening and the dynamic constants of muscle"
