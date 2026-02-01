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
│   └── physics/           # sim-physics — Unified L0 API
├── L1/                    # Layer 1: Bevy integration
│   └── bevy/              # sim-bevy — Visualization
└── docs/                  # Simulation documentation
    ├── ARCHITECTURE.md    # This file
    ├── FUTURE_WORK.md     # Roadmap and remaining work
    └── MUJOCO_REFERENCE.md # Pipeline algorithms and pseudocode
```

## Core Architecture: Model/Data

The simulation engine lives in `sim-core/src/mujoco_pipeline.rs` (~8500 lines).
It implements MuJoCo's computation pipeline end-to-end.

### Model (static)

Immutable after loading. Contains the kinematic tree, joint definitions,
geometry specifications, actuator configuration, and solver parameters.

Key fields:
- `nq`/`nv` — position coordinates vs velocity DOFs (differ for quaternion joints)
- `body_parent[i]` — kinematic tree topology
- `body_pos[i]`/`body_quat[i]` — local frame offsets
- `body_mass[i]`/`body_inertia[i]` — mass properties
- `jnt_type[i]` — `MjJointType` (Hinge, Slide, Ball, Free)
- `jnt_stiffness[i]`/`jnt_damping[i]`/`jnt_springref[i]` — passive dynamics
- `jnt_solref[i]`/`jnt_solimp[i]` — constraint solver parameters
- `timestep`, `gravity`, `integrator`, `solver_iterations`, `solver_tolerance`

Constructed from MJCF via `sim-mjcf` or URDF via `sim-urdf`.

### Data (mutable)

Pre-allocated state and computed quantities. No heap allocation during stepping.

| Category | Fields | Description |
|----------|--------|-------------|
| State | `qpos`, `qvel`, `time` | Source of truth |
| Body poses | `xpos`, `xquat`, `xmat` | Computed by forward kinematics |
| Mass matrix | `qM`, `qM_cholesky` | Computed by CRBA, Cholesky cached |
| Forces | `qfrc_bias`, `qfrc_passive`, `qfrc_actuator`, `qfrc_applied`, `qfrc_constraint` | Generalized force components |
| Acceleration | `qacc` | Computed as `M^-1 * f_total` |
| Contacts | `contacts`, `efc_lambda` | Active contacts and warmstart cache |

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

`Data::step()` calls `forward()` then `integrate()`. See `MUJOCO_REFERENCE.md`
for the complete pipeline algorithm.

## Physics Pipeline

Each timestep executes these stages in order:

```
forward():
  Position     mj_fwd_position       FK from qpos → body poses
               mj_collision           Broad + narrow phase contacts
  Velocity     mj_fwd_velocity        Body spatial velocities from qvel
  Actuation    mj_fwd_actuation       Control → joint forces
  Dynamics     mj_crba                Mass matrix (Composite Rigid Body)
               mj_rne                 Bias forces (Recursive Newton-Euler)
               mj_fwd_passive         Springs, dampers, friction loss
  Constraints  mj_fwd_constraint      Joint limits + equality + contact PGS
  Solve        mj_fwd_acceleration    qacc = M^-1 * f  (or implicit solve)
integrate():
  Semi-implicit Euler (velocity first, then position with new velocity)
  Quaternion integration on SO(3) for ball/free joints
```

**Integration methods** (`Integrator` enum):
- `Euler` (default) — semi-implicit Euler, velocity-first
- `RungeKutta4` — fourth-order accuracy
- `Implicit` — unconditionally stable for stiff springs/dampers; solves
  `(M + h*D + h^2*K) v_new = M*v_old + h*f_ext - h*K*(q - q_eq)`

**Constraint enforcement:**
- Joint limits and equality constraints use penalty + Baumgarte stabilization
  with configurable stiffness via `solref`/`solimp` parameters
- Contacts use PGS (Projected Gauss-Seidel) with friction cones and warmstart

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

Pure data structures with no physics logic. Zero dependencies beyond nalgebra.

`RigidBodyState`, `Pose`, `Twist`, `MassProperties`, `Action`, `ExternalForce`,
`JointState`, `JointCommand`, `SimulationConfig`, `BodyId`, `JointId`.

### sim-simd

SIMD-optimized batch operations for performance-critical paths.
`Vec3x4`/`Vec3x8` batched vectors, `batch_dot_product`, `batch_aabb_overlap`,
`batch_normal_force`, `batch_integrate_position/velocity`.
2-4x speedup on x86-64, 2-3x on Apple Silicon.

### sim-core

The physics engine. Contains:

- `mujoco_pipeline.rs` — `Model`, `Data`, full MuJoCo-aligned pipeline
- `integrators.rs` — 6 integrators: ExplicitEuler, SemiImplicitEuler,
  VelocityVerlet, RungeKutta4, ImplicitVelocity, ImplicitFast
- `collision_shape.rs` — `CollisionShape` enum, `Aabb`
- `mid_phase.rs` — BVH tree (SAH construction, traversal, ray queries)
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
and `CGSolver` (Conjugate Gradient solver with Block Jacobi preconditioner,
retained for future pipeline integration — see `FUTURE_WORK.md`).

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
Presets: rubber, tendon, gelatin, foam.

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

Cable-driven actuation and routing:

- **Fixed tendons** — MuJoCo-style linear joint couplings
- **Spatial tendons** — 3D routing through attachment points with wrapping
  geometry (sphere, cylinder) and pulley systems

`TendonActuator` trait: `compute_force`, `compute_length`, `jacobian`.

### sim-mjcf

MuJoCo XML format parser. Supports: bodies, joints (hinge, slide, ball, free),
geoms (sphere, box, capsule, cylinder, ellipsoid, plane, mesh), actuators
(motor, position, velocity, general), contact filtering, default class
inheritance, and MJB binary format.

### sim-urdf

URDF robot description parser. Supports: links with mass, joints (fixed,
revolute, continuous, prismatic, floating, planar), geometry (box, sphere,
cylinder), dynamics (damping, friction), limits.

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

Debug gizmos: contact points, force vectors, joint axes, muscle/tendon paths,
sensor readings. All toggleable via `ViewerConfig`.

## Design Principles

### Contact Solver

The production contact solver is the PGS (Projected Gauss-Seidel) solver
in `mujoco_pipeline.rs`. It uses Lagrange multiplier contacts with
solref/solimp parameters and warmstarting. Contact geometry types
(`ContactPoint`, etc.) are in `sim-core/src/contact.rs`.

### Determinism

The PGS solver uses a fixed iteration cap (`solver_iterations`, default 100)
with an early-exit tolerance. Same inputs produce same outputs. Fixed
computational cost is essential for RL training.

### Layer 0 (No Bevy)

All `sim-*` crates in `L0/` have zero Bevy dependencies. They run in headless
training loops, deploy to hardware, and integrate with other engines. `sim-bevy`
is Layer 1 only.

## Feature Flags

| Flag | Crates | Description |
|------|--------|-------------|
| `parallel` | sim-core | Rayon-based parallelization |
| `serde` | Most crates | Serialization support |
| `sensor` | sim-core | Enable sensor pipeline integration |
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
