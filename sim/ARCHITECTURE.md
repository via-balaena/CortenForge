# Simulation Domain Architecture

## Overview

The `sim-*` crates provide a MuJoCo-inspired physics simulation stack designed for:

- **Robotics training** (reinforcement learning with sim-to-real transfer)
- **Headless simulation** (no Bevy dependency - Layer 0)
- **Deterministic execution** (fixed iteration counts, reproducible results)
- **Domain randomization** (tunable contact parameters for robust policies)

## Directory Structure

```
sim/
├── L0/                    # Layer 0: Bevy-free simulation
│   ├── types/             # sim-types - Pure data types
│   ├── simd/              # sim-simd - SIMD batch operations
│   ├── core/              # sim-core - Integrators, world, collision
│   ├── contact/           # sim-contact - Compliant contact model
│   ├── constraint/        # sim-constraint - Joint constraints, solvers
│   ├── sensor/            # sim-sensor - IMU, F/T, touch, rangefinder
│   ├── deformable/        # sim-deformable - XPBD soft bodies
│   ├── muscle/            # sim-muscle - Hill-type muscles
│   ├── tendon/            # sim-tendon - Cable/tendon routing
│   ├── mjcf/              # sim-mjcf - MuJoCo format parser
│   ├── urdf/              # sim-urdf - URDF parser
│   └── physics/           # sim-physics - Unified L0 API
├── L1/                    # Layer 1: Bevy integration
│   └── bevy/              # sim-bevy - Visualization
├── docs/                  # Simulation documentation
└── ARCHITECTURE.md
```

## Crate Hierarchy

```
sim/L0/
├── types      (L0)  ─────►  Pure data types, zero dependencies
├── simd       (L0)  ─────►  SIMD-optimized batch operations
├── contact    (L0)  ─────►  Compliant contact model, friction, solver
├── constraint (L0)  ─────►  Joint constraints, PGS/Newton/CG solvers
├── core       (L0)  ─────►  Integrators, world, stepper, collision detection
├── sensor     (L0)  ─────►  IMU, force/torque, touch, rangefinder, magnetometer
├── deformable (L0)  ─────►  XPBD soft bodies, cloth, ropes
├── muscle     (L0)  ─────►  Hill-type muscle models
├── tendon     (L0)  ─────►  Cable/tendon routing and actuation
├── mjcf       (L0)  ─────►  MuJoCo XML/binary format parser
├── urdf       (L0)  ─────►  URDF robot description parser
└── physics    (L0)  ─────►  Unified API combining all L0 crates

sim/L1/
└── bevy       (L1)  ─────►  Bevy integration layer (visualization)
```

## Current Implementation

### sim-types (v0.7.0)

Pure data structures with no physics logic:

| Type | Purpose |
|------|---------|
| `RigidBodyState` | Position, orientation, velocity (6-DOF) |
| `Pose` | Position + quaternion rotation |
| `Twist` | Linear + angular velocity |
| `MassProperties` | Mass, COM, inertia tensor |
| `Action` / `ExternalForce` | Control inputs |
| `JointState` / `JointCommand` | Articulated body support |
| `SimulationConfig` | Timestep, gravity, solver settings |

### sim-simd (v0.7.0)

SIMD-optimized batch operations for performance-critical paths:

| Component | Description |
|-----------|-------------|
| `Vec3x4` / `Vec3x8` | Batched 3D vectors (SSE/AVX) |
| `batch_dot_product` | Parallel dot products |
| `batch_cross_product` | Parallel cross products |

Performance gains: 2-4x on x86-64, 2-3x on Apple Silicon. Used in contact force computation, GJK support search, AABB tests, and solver operations.

### sim-core (v0.7.0)

Numerical integration, world management, and collision detection:

| Component | Description |
|-----------|-------------|
| **Integrators** | Euler, Semi-Implicit Euler, Velocity Verlet, RK4 |
| **World** | Body/joint storage, entity management |
| **Stepper** | Simulation loop orchestration |
| **Collision** | Multi-phase detection pipeline |

**Collision Detection Pipeline:**

| Phase | Implementation |
|-------|----------------|
| **Broad Phase** | BruteForce, Sweep-and-Prune |
| **Mid Phase** | BVH tree for triangle meshes |
| **Narrow Phase** | GJK/EPA for convex shapes, specialized mesh algorithms |

**Supported Collision Shapes:**

| Shape | Type |
|-------|------|
| Sphere, Box, Capsule, Cylinder, Ellipsoid | Primitives |
| Plane | Half-space |
| ConvexMesh | GJK/EPA-based |
| TriangleMesh | BVH-accelerated, non-convex |
| HeightField | Terrain (2D height grid) |
| SignedDistanceField | Implicit surface queries |

### sim-contact (v0.1.0)

MuJoCo-inspired compliant contact model:

```
F_normal = k * d^p + c * ḋ
F_friction = project_to_cone(F_tangent, μ * F_normal)
```

| Module | Contents |
|--------|----------|
| `contact.rs` | `ContactPoint`, `ContactManifold`, `ContactForce` |
| `friction.rs` | `FrictionCone`, `FrictionModel`, `RegularizedFriction` |
| `model.rs` | `ContactModel` (spring-damper computation) |
| `params.rs` | `ContactParams`, `DomainRandomization`, material presets |
| `solver.rs` | `ContactSolver` with deterministic fixed iterations |
| `batch.rs` | `BatchContactProcessor` for parallel handling |

Key features:
- Nonlinear stiffness (d^p) for stability at varying penetrations
- Regularized friction for implicit integration compatibility
- Elliptic and pyramidal friction cone approximations
- Rolling and torsional friction support
- Domain randomization ranges for sim-to-real transfer
- Material presets: rubber, metal, plastic, wood, etc.

### sim-constraint (v0.7.0)

Joint constraints and articulated body dynamics:

**Solvers:**

| Solver | Description |
|--------|-------------|
| `ConstraintSolver` | Basic Gauss-Seidel (8-16 iterations) |
| `PGSSolver` | Projected Gauss-Seidel with SOR, warm-starting |
| `NewtonConstraintSolver` | Newton-Raphson with analytical Jacobians (2-3 iterations) |
| `CGSolver` | Conjugate Gradient for sparse systems |

**Joint Types:**

| Joint | DOF |
|-------|-----|
| Fixed | 0 |
| Revolute (Hinge) | 1 |
| Prismatic (Slide) | 1 |
| Universal | 2 |
| Cylindrical | 2 |
| Planar | 3 |
| Spherical (Ball) | 3 |
| Free | 6 |

**Features:**
- Joint limits (position, velocity)
- Motors (position control, velocity control)
- Spring-damper dynamics
- Constraint islands for independent group optimization
- Parallel solving (rayon, optional)
- Muscle actuator integration (optional)
- Equality constraints: connect, gear coupling, differential coupling, tendon networks

### sim-sensor (v0.7.0)

Simulated sensor suite for robot perception:

| Sensor | Measurements |
|--------|--------------|
| `Imu` | Accelerometer + gyroscope (6-axis) |
| `ForceTorqueSensor` | 6-axis force/torque |
| `TouchSensor` | Binary/pressure contact detection |
| `Rangefinder` | Ray-cast distance (LIDAR/ultrasonic) |
| `Magnetometer` | Magnetic heading |

### sim-deformable (v0.1.0)

Soft body simulation using Extended Position-Based Dynamics (XPBD):

| Type | Description |
|------|-------------|
| `CapsuleChain` | 1D deformables (ropes, cables) |
| `Cloth` | 2D deformables (membranes, shells) |
| `SoftBody` | 3D deformables (tetrahedral meshes) |
| `SkinnedMesh` | Bone-driven mesh deformation |

**Material Model:**
- Young's modulus, Poisson's ratio
- Density, damping coefficients
- Presets: Rubber, Tendon, Gelatin, Foam

**Features:**
- Pinned vertices (fixed constraints)
- External force application
- Bounding box queries

### sim-muscle (v0.1.0)

Hill-type muscle model for biomechanical simulation:

```
F_muscle = a(t) * F_max * f_l(l) * f_v(v) + F_passive(l)
```

| Component | Description |
|-----------|-------------|
| Contractile Element (CE) | Active force generation |
| Parallel Elastic (PE) | Passive tissue stiffness |
| Series Elastic (SE) | Tendon compliance |

**Features:**
- Activation dynamics with asymmetric time constants
- Force-length relationship (Gaussian curve)
- Force-velocity relationship (Hill curve)
- Pennation angle support
- Predefined configs: Biceps, Quadriceps, Gastrocnemius, Soleus
- MuscleGroup for antagonist pairs

### sim-tendon (v0.1.0)

Cable-driven actuation and routing:

| Type | Description |
|------|-------------|
| Fixed Tendon | MuJoCo-style linear joint couplings |
| Spatial Tendon | 3D routing through attachment points |

**Features:**
- Wrapping geometry (sphere, cylinder)
- Pulley systems with mechanical advantage
- Cable properties: stiffness, damping, tension limits
- `TendonActuator` trait: `compute_force`, `compute_length`, `jacobian`

### sim-mjcf (v0.7.0)

MuJoCo XML format parser:

**Supported Elements:**
- Bodies with hierarchical definition
- Joints: hinge, slide, ball, free
- Geoms: sphere, box, capsule, cylinder, ellipsoid, plane, mesh
- Actuators: motor, position, velocity, general
- Contact filtering
- Default class inheritance system

**Features:**
- Binary MJB format support (fast loading)
- Defaults system with class inheritance
- Kinematic tree validation

### sim-urdf (v0.7.0)

URDF robot description parser:

**Supported Elements:**
- Links with mass properties
- Joints: fixed, revolute, continuous, prismatic, floating, planar
- Geometry: box, sphere, cylinder
- Dynamics: damping, friction
- Limits: position, velocity, effort

**Features:**
- `SpawnedRobot` for world integration
- Kinematic validation

### sim-physics (v0.7.0)

Unified API re-exporting all simulation components:

```rust
use sim_physics::prelude::*;
```

Bridges sim-urdf, sim-constraint, sim-contact, sim-core, and sim-types into a single coherent interface.

### sim-bevy (v0.1.0) - Layer 1

Bevy visualization layer for physics debugging and demonstration:

| Component | Description |
|-----------|-------------|
| `SimViewerPlugin` | Main plugin with orbit camera and lighting |
| `SimulationHandle` | Resource holding sim-core `World` |
| `BodyEntityMap` | Maps physics body IDs to Bevy entities |
| `MjcfModel` / `UrdfModel` | Model loading from robot descriptions |

**Coordinate System Conversion:**

sim-core uses Z-up (robotics convention), Bevy uses Y-up (graphics convention).
All conversions are centralized in `convert.rs`:

| Physics (Z-up) | Bevy (Y-up) |
|----------------|-------------|
| X (forward) | X (right) |
| Y (left) | Z (forward) |
| Z (up) | Y (up) |

```rust
// Position: swap Y and Z
(x, y, z)_physics -> (x, z, y)_bevy

// Quaternion: conjugate by coordinate rotation
q_bevy = R * q_physics * R^-1
```

**Features:**
- Automatic mesh generation for all collision shapes
- Transform synchronization (physics → Bevy)
- Debug gizmos (contact points, forces, axes)
- Orbit camera with pan/zoom controls
- MJCF and URDF model loading

**Example:**

```rust
use bevy::prelude::*;
use sim_bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin::default())
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    // Load and spawn MJCF model
    let model = MjcfModel::from_file("humanoid.xml").unwrap();
    commands.spawn(model);
}
```

## Design Principles

### 1. Compliant vs Impulse-Based Contacts

We use **compliant (spring-damper) contacts** rather than impulse-based:

| Aspect | Compliant | Impulse-Based |
|--------|-----------|---------------|
| Force response | Smooth, continuous | Discontinuous at contact |
| Parameters | Physical (stiffness, damping) | Abstract (iterations, ERP) |
| Stability | Better for stiff contacts | Sensitive to timestep |
| Sim-to-real | Tunable to match real materials | Harder to transfer |

### 2. Determinism

All solvers use **fixed iteration counts** (not convergence-based):

```rust
ContactSolverConfig {
    velocity_iterations: 4,  // Always 4, regardless of convergence
    position_iterations: 2,
}
```

This ensures:
- Same inputs → same outputs (essential for RL)
- Predictable performance (fixed computational cost)
- Reproducible debugging

### 3. Domain Randomization

Contact parameters can be randomized for robust policy learning:

```rust
let randomizer = DomainRandomization::default();
// stiffness_range: (0.5, 1.5)  -- ±50% variation
// friction_range: (0.2, 1.0)   -- wide friction range
// etc.

let params = randomizer.sample(&base_params, rng_samples);
```

### 4. Layer 0 (No Bevy)

All `sim-*` crates are Layer 0:
- Zero Bevy dependencies
- Can run in headless training loops
- Can deploy to hardware
- Can integrate with other engines

### 5. SIMD Optimization

Performance-critical operations use batched SIMD:

```rust
use sim_simd::{Vec3x4, batch_dot_product};

let a = Vec3x4::splat(vec_a);
let b = Vec3x4::load(&vectors);
let dots = batch_dot_product(&a, &b);  // 4 dot products in parallel
```

### 6. Parallel Solving

Constraint islands enable independent parallel solving:

```rust
let islands = find_constraint_islands(&world);
islands.par_iter().for_each(|island| {
    solver.solve_island(island);
});
```

## Feature Flags

| Flag | Crates | Description |
|------|--------|-------------|
| `parallel` | sim-core, sim-constraint | Rayon-based parallelization |
| `serde` | Most crates | Serialization support |
| `mjb` | sim-mjcf | Binary MuJoCo format |
| `muscle` | sim-constraint | Hill-type muscle integration |

## Implementation Status

### Completed ✓

- [x] Compliant contact force model
- [x] Nonlinear stiffness (d^p)
- [x] Spring-damper normal forces
- [x] Friction cone with projection
- [x] Regularized friction for smooth gradients
- [x] Rolling and torsional friction
- [x] Domain randomization infrastructure
- [x] Deterministic solvers (Gauss-Seidel, PGS, Newton, CG)
- [x] Collision detection (primitives, convex meshes, triangle meshes, SDF)
- [x] Broad-phase culling (sweep-and-prune, BVH)
- [x] Articulated bodies (joint limits, motors, constraints)
- [x] SIMD optimization
- [x] Parallel constraint solving
- [x] Sensor simulation (IMU, F/T, touch, rangefinder, magnetometer)
- [x] Soft body dynamics (XPBD)
- [x] Hill-type muscle models
- [x] Tendon/cable simulation
- [x] MJCF parser with defaults system
- [x] URDF parser
- [x] Height field terrain
- [x] Signed distance field collision
- [x] Bevy visualization layer (sim-bevy)

### Future Considerations

- [ ] **Kinematic loops** (currently tree-only articulations)
- [ ] **Non-convex mesh decomposition** (automatic convex hull splitting)
- [ ] **Fluid coupling** (buoyancy, drag)
- [ ] **Include file support** in MJCF/URDF parsers
- [ ] **GPU acceleration** (CUDA/Metal compute)

## Usage Example

```rust
use sim_physics::prelude::*;
use sim_core::{World, Stepper, collision::{CollisionShape, detect_contacts}};
use sim_contact::{ContactModel, ContactParams, ContactSolver};
use sim_constraint::PGSSolver;

// Create world
let mut world = World::default();
let sphere = world.add_body(
    RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 1.0))),
    MassProperties::sphere(1.0, 0.1),
);

// Add collision shape
world.set_collision_shape(sphere, CollisionShape::Sphere { radius: 0.1 });

// Create solvers
let contact_model = ContactModel::new(ContactParams::rubber_on_concrete());
let mut contact_solver = ContactSolver::with_model(contact_model);
let mut constraint_solver = PGSSolver::new(100); // 100 iterations

// Simulation loop
loop {
    // 1. Detect contacts (broad + narrow phase)
    let contacts = detect_contacts(&world);

    // 2. Solve contact forces
    let contact_result = contact_solver.solve(&contacts, |body_id| {
        world.body(body_id).map(|b| b.state.twist.linear).unwrap_or_default()
    });

    // 3. Apply contact forces
    for force_result in &contact_result.forces {
        world.apply_force(force_result.body_a, force_result.force.total());
        world.apply_force(force_result.body_b, -force_result.force.total());
    }

    // 4. Solve joint constraints
    constraint_solver.solve(&mut world);

    // 5. Integrate
    stepper.step(&mut world)?;
}
```

## References

- [MuJoCo Documentation](https://mujoco.readthedocs.io/en/stable/modeling.html#contact)
- [MuJoCo Technical Notes](https://mujoco.readthedocs.io/en/stable/computation.html)
- Todorov, E. (2014). "Convex and analytically-invertible dynamics with contacts and constraints"
- Macklin, M. et al. (2016). "XPBD: Position-Based Simulation of Compliant Constrained Dynamics"
- Hill, A.V. (1938). "The heat of shortening and the dynamic constants of muscle"
