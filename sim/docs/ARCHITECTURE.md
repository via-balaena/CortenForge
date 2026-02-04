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
│   ├── deformable/        # sim-deformable — XPBD soft bodies
│   ├── muscle/            # sim-muscle — Hill-type muscles
│   ├── tendon/            # sim-tendon — Cable/tendon routing
│   ├── mjcf/              # sim-mjcf — MuJoCo format parser
│   ├── urdf/              # sim-urdf — URDF parser
│   ├── physics/           # sim-physics — Unified L0 API
│   └── tests/             # sim-conformance-tests — Model loading tests
├── L1/                    # Layer 1: Bevy integration
│   └── bevy/              # sim-bevy — Visualization
└── docs/                              # Simulation documentation
    ├── ARCHITECTURE.md                # This file
    ├── FUTURE_WORK.md                 # Roadmap and remaining work
    ├── MUJOCO_CONFORMANCE.md          # MuJoCo conformance testing plan
    ├── MUJOCO_GAP_ANALYSIS.md         # Feature-by-feature gap analysis
    ├── MUJOCO_REFERENCE.md            # Pipeline algorithms and pseudocode
    └── SIM_BEVY_IMPLEMENTATION_PLAN.md # Bevy visualization layer plan
```

## Core Architecture: Model/Data

The simulation engine lives in `sim-core/src/mujoco_pipeline.rs` (~13,900 lines).
It implements MuJoCo's computation pipeline end-to-end.

### Model (static)

Immutable after loading. Contains the kinematic tree, joint definitions,
geometry specifications, actuator configuration, and solver parameters.

Key fields:
- `nq`/`nv` — position coordinates vs velocity DOFs (differ for quaternion joints)
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

Constructed from MJCF via `sim-mjcf` or URDF via `sim-urdf`.

### Data (mutable)

Pre-allocated state and computed quantities. Most buffers are pre-allocated;
residual heap allocation occurs for contact vector growth and RK4 warmstart save.

| Category | Fields | Description |
|----------|--------|-------------|
| State | `qpos`, `qvel`, `act`, `ctrl`, `time` | Source of truth (act = activation states for muscles/filters) |
| Body poses | `xpos`, `xquat`, `xmat` | Computed by forward kinematics |
| Mass matrix | `qM`, `qLD_diag`, `qLD_L` | Computed by CRBA; sparse L^T D L factorization cached |
| Forces | `qfrc_bias`, `qfrc_passive`, `qfrc_actuator`, `qfrc_applied`, `qfrc_constraint` | Generalized force components |
| Actuation | `actuator_length`, `actuator_velocity`, `actuator_force`, `act_dot` | Actuator-space state and activation derivatives |
| Acceleration | `qacc` | Computed as `M^-1 * f_total` |
| Contacts | `contacts`, `efc_lambda` | Active contacts and warmstart cache (`HashMap<WarmstartKey, [f64; 3]>`) |

### Stepping

```rust
use sim_core::{Model, Data};
use sim_mjcf::load_model;

let model = load_model(mjcf_string)?;  // Static (from MJCF XML string)
let mut data = model.make_data();      // Pre-allocated

// Simulation loop
loop {
    data.step(&model)?;
    // data.qpos, data.xpos, etc. now updated
}
```

`Data::step()` dispatches by integrator: for Euler and ImplicitSpringDamper it
calls `forward()` then `integrate()`; for RK4 it calls `forward()` then
`mj_runge_kutta()` (a true 4-stage Runge-Kutta that re-evaluates dynamics at
each stage). See `MUJOCO_REFERENCE.md` for the complete pipeline algorithm.

## Physics Pipeline

Each timestep executes these stages in order:

```
forward():
  Position     mj_fwd_position       FK from qpos → body poses
               mj_fwd_tendon         Tendon lengths + Jacobians (fixed tendons)
               mj_collision           Broad + narrow phase contacts
  Velocity     mj_fwd_velocity        Body spatial velocities + tendon velocities
               mj_actuator_length     Actuator length/velocity from transmission
  Actuation    mj_fwd_actuation       act_dot computation + gain/bias force + clamping
  Dynamics     mj_crba                Mass matrix (Composite Rigid Body)
               mj_rne                 Bias forces (Recursive Newton-Euler)
               mj_fwd_passive         Springs, dampers, friction loss (joints + tendons)
  Constraints  mj_fwd_constraint      Joint/tendon limits + equality + contact PGS/CG
  Solve        mj_fwd_acceleration    qacc = M^-1 * f  (or implicit solve)
integrate() [Euler / ImplicitSpringDamper]:
  Activation integration (act += dt * act_dot, muscle clamp to [0,1])
  Semi-implicit Euler (velocity first, then position with new velocity)
  Quaternion integration on SO(3) for ball/free joints
mj_runge_kutta() [RungeKutta4]:
  True 4-stage RK4 with Butcher tableau [1/6, 1/3, 1/3, 1/6]
  Integrates activation alongside qpos/qvel with same RK4 weights
  Stage 0 reuses initial forward(); stages 1-3 call forward_skip_sensors()
  Uses mj_integrate_pos_explicit() for quaternion-safe position updates
```

**Integration methods** (`Integrator` enum):
- `Euler` (default) — semi-implicit Euler, velocity-first
- `RungeKutta4` — true 4th-order Runge-Kutta (O(h⁴) global error);
  uses `forward_skip_sensors()` for intermediate stage evaluations and
  `mj_integrate_pos_explicit()` for quaternion-safe position updates
- `ImplicitSpringDamper` — unconditionally stable for stiff springs/dampers; solves
  `(M + h*D + h^2*K) v_new = M*v_old + h*f_ext - h*K*(q - q_eq)`

**Constraint enforcement:**
- Joint/tendon limits and equality constraints use penalty + Baumgarte stabilization
  with configurable stiffness via `solref`/`solimp` parameters
- Contacts use PGS (Projected Gauss-Seidel) or CG (preconditioned projected gradient
  descent with Barzilai-Borwein step) with friction cones and warmstart, selectable
  via `<option solver="CG"/>` in MJCF or `model.solver_type = SolverType::CG`

## Collision Detection

### Pipeline

| Phase | Method | Output |
|-------|--------|--------|
| Broad | Sweep-and-prune on AABBs | Candidate pairs |
| Filter | contype/conaffinity bitmasks, parent-child exclusion | Filtered pairs |
| Narrow | Per-pair geometry tests | Contact points |

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
- `collision_shape.rs` — `CollisionShape` enum, `Aabb`
- `mid_phase.rs` — BVH tree (median-split construction, traversal, ray queries)
- `gjk_epa.rs` — GJK/EPA for convex shapes
- `mesh.rs` — Triangle mesh collision functions
- `heightfield.rs`, `sdf.rs` — Terrain and implicit surface collision
- `raycast.rs` — Ray-shape intersection

### Contact Types (in sim-core)

`ContactPoint`, `ContactManifold`, and `ContactForce` live in
`sim-core/src/contact.rs`. These represent collision geometry output
(position, normal, penetration, body pair) and resulting forces.
The MuJoCo pipeline uses its own PGS contact solver in `mujoco_pipeline.rs`.

### sim-constraint

Joint types, motors, limits, and CGSolver for articulated body simulation:

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
standalone library for joint-space constraints, distinct from the pipeline's CG
contact solver in `mujoco_pipeline.rs`; see `FUTURE_WORK.md`).

### sim-sensor

Simulated sensor suite: `Imu` (6-axis accel + gyro), `ForceTorqueSensor`
(6-axis), `TouchSensor` (binary/pressure), `Rangefinder` (ray-cast),
`Magnetometer` (heading).

### sim-deformable

Soft body simulation using XPBD (Extended Position-Based Dynamics):

| Type | Dimension | Use Cases |
|------|-----------|-----------|
| `CapsuleChain` | 1D | Ropes, cables, hair |
| `Cloth` | 2D | Membranes, shells, fabric |
| `SoftBody` | 3D | Tetrahedral volumetric meshes |
| `SkinnedMesh` | — | Bone-driven mesh deformation |

Material model: Young's modulus, Poisson's ratio, density, damping.
Presets: rubber, tendon, gelatin, foam, cloth, soft-tissue, muscle, cartilage,
leather, rope, steel-cable, paper, flexible-plastic, soft-wood (14 total).

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

**Note:** Fixed tendons are implemented directly in the MuJoCo pipeline
(`mj_fwd_tendon` in sim-core) and do not use sim-tendon. This crate remains
a standalone reference library for advanced tendon analysis.

### sim-mjcf

MuJoCo XML format parser. Supports: bodies, joints (hinge, slide, ball, free),
geoms (sphere, box, capsule, cylinder, ellipsoid, plane, mesh), actuators
(motor, position, velocity, general, muscle, cylinder, damper, adhesion),
contype/conaffinity contact bitmasks, default class inheritance, and MJB binary
format. **Note:** `<contact>` `<pair>`/`<exclude>` elements are not parsed.
`<tendon>` and `<sensor>` elements are parsed and wired into the pipeline
(fixed tendons fully supported; spatial tendons scaffolded but deferred;
all 30 pipeline sensor types functional and wired from MJCF via
`process_sensors()` in `model_builder.rs`). The model builder expands all 8 actuator
shortcut types to their general gain/bias/dynamics representation (matching
MuJoCo's `user_api.cc`), populating `actuator_gaintype`, `actuator_biastype`,
`actuator_gainprm`, `actuator_biasprm`, and `actuator_dynprm` per actuator.
Muscle parameters are parsed and transferred; `compute_muscle_params()`
resolves `lengthrange`, `acc0`, and auto-computes `F0` at model build time.

### sim-urdf

URDF robot description parser. Converts URDF → MJCF via sim-mjcf. Supports:
links with mass, joints (fixed, revolute, continuous, prismatic, floating),
geometry (box, sphere, cylinder; mesh parsed but not converted), dynamics
(damping; friction parsed but not converted), limits. **Planar joints are
lossy** — approximated as a single hinge (loses 2 of 3 DOF).

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

## Design Principles

### Contact Solver

The pipeline supports two contact solvers selectable via `model.solver_type`:

- **PGS** (default) — Projected Gauss-Seidel. Matches MuJoCo's default. Uses
  per-contact inline friction cone projection inside the GS sweep.
- **CG** — Preconditioned projected gradient descent (PGD) with Barzilai-Borwein
  adaptive step size. Named "CG" for MuJoCo API compatibility. Falls back to PGS
  on non-convergence, reusing the pre-assembled Delassus matrix.
- **CGStrict** — Same as CG but returns zero forces on non-convergence instead of
  falling back. Use in tests to detect CG regressions.

Both solvers share `assemble_contact_system()` for Delassus matrix + RHS construction,
`WarmstartKey`-based contact correspondence, and friction cone projection. The warmstart
key combines canonical geom pair IDs with a discretized 1 cm spatial grid cell, so
multiple contacts within the same geom pair (e.g., box-on-plane corners) each get
their own cached lambda. Contact geometry types (`ContactPoint`, etc.) are in
`sim-core/src/contact.rs`.

Solver selection: `<option solver="CG"/>` in MJCF or `model.solver_type = SolverType::CG`.

### Determinism

Both solvers use a fixed iteration cap (`solver_iterations`, default 100) with an
early-exit tolerance. Same inputs produce same outputs. Fixed computational cost is
essential for RL training.

### Layer 0 (No Bevy)

All `sim-*` crates in `L0/` have zero Bevy dependencies. They run in headless
training loops, deploy to hardware, and integrate with other engines. `sim-bevy`
is Layer 1 only.

## Feature Flags

| Flag | Crates | Description |
|------|--------|-------------|
| `parallel` | sim-core | Rayon-based parallelization. **Reserved** — declared but no `#[cfg]` guards yet; see [future_work_2.md #9](./future_work_2.md) |
| `serde` | Most crates | Serialization support |
| `mjb` | sim-mjcf | Binary MuJoCo format |
| `muscle` | sim-constraint | Hill-type muscle integration |
| `deformable` | sim-physics | XPBD soft body support |

## References

- [MuJoCo Documentation](https://mujoco.readthedocs.io/en/stable/modeling.html#contact)
- [MuJoCo Technical Notes](https://mujoco.readthedocs.io/en/stable/computation.html)
- Todorov, E. (2014). "Convex and analytically-invertible dynamics with contacts and constraints"
- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer.
- Macklin, M. et al. (2016). "XPBD: Position-Based Simulation of Compliant Constrained Dynamics"
- Hill, A.V. (1938). "The heat of shortening and the dynamic constants of muscle"
