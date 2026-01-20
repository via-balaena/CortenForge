# MuJoCo Gap Analysis: CortenForge Physics Stack

This document provides a comprehensive comparison between MuJoCo's physics capabilities and CortenForge's current `sim-*` crate implementation. Use this as a roadmap for bridging the gap.

## Legend

| Status | Meaning |
|--------|---------|
| **Implemented** | Feature exists and is functional |
| **Partial** | Core concept exists but incomplete |
| **Stub** | Infrastructure exists, needs implementation |
| **Missing** | Not yet started |

---

## 1. Integration Methods

| Feature | MuJoCo | CortenForge | Status | Priority | Complexity |
|---------|--------|-------------|--------|----------|------------|
| Semi-implicit Euler | Default | `SemiImplicitEuler` | **Implemented** | - | - |
| RK4 | Supported | `RungeKutta4` | **Implemented** | - | - |
| Explicit Euler | Supported | `ExplicitEuler` | **Implemented** | - | - |
| Velocity Verlet | - | `VelocityVerlet` | **Implemented** | - | - |
| Implicit-in-velocity | Core feature | `ImplicitVelocity` | **Implemented** | - | - |
| Implicit-fast (no Coriolis) | Optimization | Missing | **Missing** | Medium | Medium |

### Implementation Notes: Implicit Integration ✅ COMPLETED

MuJoCo's implicit integrators solve:
```
(M - h*D) * v_{t+h} = M * v_t + h * f
```

Where `D` captures velocity-dependent force derivatives (damping). This is critical for:
- Very stiff contacts
- Highly damped systems
- Muscle models with activation dynamics

**Implemented:**
- `ImplicitVelocity` integrator with unconditional stability
- `integrate_with_damping()` method for built-in damping support
- `integrate_with_method_and_damping()` dispatch function
- `IntegrationMethod::ImplicitVelocity` enum variant
- `IntegrationMethod::is_implicit()` helper method

**Usage:**
```rust
use sim_core::integrators::{ImplicitVelocity, integrate_with_method_and_damping};
use sim_types::IntegrationMethod;

// Direct usage with damping
ImplicitVelocity::integrate_with_damping(
    &mut state,
    linear_accel,
    angular_accel,
    linear_damping,  // e.g., 10.0 for heavy damping
    angular_damping,
    dt,
);

// Via dispatch (for damped systems)
integrate_with_method_and_damping(
    IntegrationMethod::ImplicitVelocity,
    &mut state,
    linear_accel,
    angular_accel,
    linear_damping,
    angular_damping,
    dt,
);
```

**Files modified:** `sim-core/src/integrators.rs`, `sim-types/src/config.rs`

---

## 2. Constraint Solvers

| Feature | MuJoCo | CortenForge | Status | Priority | Complexity |
|---------|--------|-------------|--------|----------|------------|
| PGS (Gauss-Seidel) | Supported | Partial (relaxation param) | **Partial** | Medium | Low |
| Newton solver | Default, 2-3 iterations | `NewtonConstraintSolver` | **Implemented** | - | - |
| Conjugate Gradient | Supported | Missing | **Missing** | Low | Medium |
| Constraint islands | Auto-detected | `ConstraintIslands` | **Implemented** | - | - |
| Warm starting | Supported | Stub (`enable_warm_starting`) | **Stub** | Medium | Low |

### Implementation Notes: Newton Solver ✅ COMPLETED

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

**Usage:**
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

**Files:** `sim-constraint/src/newton.rs`

### Implementation Notes: Constraint Islands ✅ COMPLETED

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

**Usage:**
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

**Files:** `sim-constraint/src/islands.rs`, `sim-constraint/src/newton.rs`

---

## 3. Contact Model

| Feature | MuJoCo | CortenForge | Status | Priority | Complexity |
|---------|--------|-------------|--------|----------|------------|
| Compliant contacts | Core | `ContactModel` | **Implemented** | - | - |
| Spring-damper (F = k*d^p + c*v) | Core | `compute_normal_force_magnitude` | **Implemented** | - | - |
| Nonlinear stiffness (d^p) | Core | `stiffness_power` param | **Implemented** | - | - |
| Contact margin | Supported | `contact_margin` param | **Implemented** | - | - |
| Elliptic friction cones | Default | `EllipticFrictionCone` | **Implemented** | - | - |
| Pyramidal friction cones | Alternative | Implicit (Coulomb) | **Partial** | Low | Low |
| Torsional friction | condim 4-6 | Missing | **Missing** | Medium | Medium |
| Rolling friction | condim 6-10 | Missing | **Missing** | Low | High |
| Contact pairs filtering | Supported | Missing | **Missing** | Medium | Low |
| solref/solimp params | MuJoCo-specific | Different params | N/A | - | - |

### Implementation Notes: Elliptic Friction Cones ✅ COMPLETED

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

**Files modified:** `sim-contact/src/friction.rs`, `sim-contact/src/model.rs`, `sim-contact/src/params.rs`

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
| Cylinder | Native | Missing | **Missing** | Medium | Medium |
| Ellipsoid | Native | Missing | **Missing** | Low | Medium |
| Convex mesh | GJK/EPA | `CollisionShape::ConvexMesh` | **Implemented** | - | - |
| Height field | Native | Missing | **Missing** | Low | High |
| SDF (signed distance) | Native | Missing | **Missing** | Low | High |
| Broad-phase (sweep-prune) | Native | `SweepAndPrune` | **Implemented** | - | - |
| Mid-phase (BVH per body) | Static AABB | Missing | **Missing** | Medium | Medium |
| Narrow-phase (GJK/EPA) | Default | `gjk_epa` module | **Implemented** | - | - |

### Implementation Notes: Collision Pipeline

The collision detection pipeline now uses:
1. ✅ **Broad-phase**: Sweep-and-prune with automatic axis selection (O(n log n))
   - `SweepAndPrune` for scenes with >32 bodies
   - `BruteForce` fallback for small scenes
   - Configurable via `BroadPhaseConfig`
2. Mid-phase: AABB tree per complex body (not yet implemented)
3. ✅ **Narrow-phase**: GJK for intersection testing, EPA for penetration depth

**Files:** `sim-core/src/broad_phase.rs`, `sim-core/src/world.rs`, `sim-core/src/gjk_epa.rs`

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

**Files:** `sim-core/src/gjk_epa.rs`, `sim-core/src/world.rs`

---

## 5. Geom Types (Collision Shapes)

| Shape | MuJoCo | CortenForge | Status |
|-------|--------|-------------|--------|
| Plane | Yes | `CollisionShape::Plane` | **Implemented** |
| Sphere | Yes | `CollisionShape::Sphere` | **Implemented** |
| Box | Yes | `CollisionShape::Box` | **Implemented** |
| Capsule | Yes | `CollisionShape::Capsule` | **Implemented** |
| Convex Mesh | Yes (convexified) | `CollisionShape::ConvexMesh` | **Implemented** |
| Cylinder | Yes | Missing | **Missing** |
| Ellipsoid | Yes | Missing | **Missing** |
| Mesh | Yes (convexified) | Missing | **Missing** |
| Height field | Yes | Missing | **Missing** |
| SDF | Yes | Missing | **Missing** |

---

## 6. Joint Types

| Joint | MuJoCo | CortenForge | Status | Notes |
|-------|--------|-------------|--------|-------|
| Fixed (weld) | Yes | `FixedJoint` | **Implemented** | |
| Hinge (revolute) | Yes | `RevoluteJoint` | **Implemented** | |
| Slide (prismatic) | Yes | `PrismaticJoint` | **Implemented** | |
| Ball (spherical) | Yes | `SphericalJoint` | **Implemented** | |
| Universal | Yes | `UniversalJoint` | **Implemented** | |
| Free (6 DOF) | Yes | `JointType::Free` | **Partial** | No constraint solver |
| Planar | Yes | `JointType::Planar` | **Partial** | No constraint solver |
| Cylindrical | Yes | `JointType::Cylindrical` | **Partial** | No constraint solver |

### Implementation Notes: Free/Planar Joints

Free joints (6 DOF floating bodies) need:
- Quaternion integration for orientation
- No positional constraints, but need mass matrix handling

**Files to modify:** `sim-constraint/src/solver.rs`

---

## 7. Actuators

| Actuator | MuJoCo | CortenForge | Status | Priority | Complexity |
|----------|--------|-------------|--------|----------|------------|
| Motor (direct torque) | Yes | `JointMotor` | **Implemented** | - | - |
| Position servo | Yes | `JointMotor::position` | **Implemented** | - | - |
| Velocity servo | Yes | `JointMotor::velocity` | **Implemented** | - | - |
| PD control | Yes | `compute_force` with Kp/Kd | **Implemented** | - | - |
| Integrated velocity | Yes | Missing | **Missing** | Medium | Low |
| Damper | Yes | Joint damping | **Implemented** | - | - |
| Cylinder (pneumatic) | Yes | Missing | **Missing** | Low | Medium |
| Muscle (Hill-type) | Yes | Missing | **Missing** | Low | High |
| Adhesion | Yes | Missing | **Missing** | Low | Medium |
| General (custom) | Yes | Missing | **Missing** | Medium | Medium |

### Implementation Notes: Muscle Model

MuJoCo's muscle model includes:
- Activation dynamics (3rd-order system)
- Force-length-velocity relationships
- Pennation angle

This is specialized for biomechanics. Consider as optional extension.

---

## 8. Sensors

| Sensor | MuJoCo | CortenForge | Status | Priority |
|--------|--------|-------------|--------|----------|
| Joint position | Yes | `JointState` | **Implemented** | - |
| Joint velocity | Yes | `JointState` | **Implemented** | - |
| Body position/rotation | Yes | `Observation::BodyStates` | **Implemented** | - |
| Body velocity | Yes | `Observation::BodyStates` | **Implemented** | - |
| Accelerometer | Yes | `Imu` (via sim-sensor) | **Implemented** | - |
| Gyro | Yes | `Imu` (via sim-sensor) | **Implemented** | - |
| Force/torque | Yes | `ForceTorqueSensor` | **Implemented** | - |
| Touch | Yes | `TouchSensor` | **Implemented** | - |
| IMU (combined) | Yes | `Imu` | **Implemented** | - |
| Rangefinder | Yes | Missing | **Missing** | Medium |
| Magnetometer | Yes | Missing | **Missing** | Low |
| Camera (rendered) | Yes | Out of scope | N/A | - |

### Implementation Notes: Sensors ✅ COMPLETED

Created `sim-sensor` crate with:
- `Imu` - Accelerometer + gyroscope combined sensor with configurable noise, bias, and gravity
- `ForceTorqueSensor` - 6-axis force/torque measurement with saturation and deadband
- `TouchSensor` - Contact detection with force thresholds and body filtering
- `SensorObservation` in sim-types for integration with observation system

**Example usage:**
```rust
use sim_sensor::{Imu, ImuConfig, TouchSensor, TouchSensorConfig};
use sim_types::{BodyId, RigidBodyState, Pose};

// Create an IMU on body 1
let imu = Imu::new(BodyId::new(1), ImuConfig::default());
let state = RigidBodyState::at_rest(Pose::identity());
let reading = imu.read_from_state(&state, 0.001);

// Create a touch sensor
let touch = TouchSensor::new(BodyId::new(1), TouchSensorConfig::default());
let obs = touch.read_as_observation(&contacts, 0.001);
```

**Files:** `sim-sensor/src/imu.rs`, `sim-sensor/src/force_torque.rs`, `sim-sensor/src/touch.rs`

**New crate:** `sim-sensor/`

---

## 9. Tendons

| Feature | MuJoCo | CortenForge | Status | Priority | Complexity |
|---------|--------|-------------|--------|----------|------------|
| Fixed tendons | Yes | Missing | **Missing** | Low | Medium |
| Spatial tendons | Yes | Missing | **Missing** | Low | High |
| Wrapping (sphere/cylinder) | Yes | Missing | **Missing** | Low | High |
| Pulley systems | Yes | Missing | **Missing** | Low | Medium |

### Implementation Notes

Tendons are primarily for biomechanics and cable-driven robots. Lower priority unless specifically needed.

---

## 10. Equality Constraints

| Constraint | MuJoCo | CortenForge | Status | Priority |
|------------|--------|-------------|--------|----------|
| Connect (ball) | Yes | Via SphericalJoint | **Partial** | Medium |
| Weld | Yes | Via FixedJoint | **Implemented** | - |
| Joint coupling | Yes | Missing | **Missing** | Medium |
| Tendon coupling | Yes | Missing | **Missing** | Low |
| Flex (edge length) | Yes | Missing | **Missing** | Low |

---

## 11. Deformables (Flex)

| Feature | MuJoCo | CortenForge | Status | Priority | Complexity |
|---------|--------|-------------|--------|----------|------------|
| 1D (capsule chains) | Yes | Missing | **Missing** | Low | High |
| 2D (triangle shells) | Yes | Missing | **Missing** | Low | Very High |
| 3D (tetrahedra) | Yes | Missing | **Missing** | Low | Very High |
| Skinned meshes | Yes | Missing | **Missing** | Low | High |

### Implementation Notes

Deformable simulation is a major undertaking. Consider using position-based dynamics (PBD) or XPBD for cables/cloth if needed.

---

## 12. Performance Optimizations

| Feature | MuJoCo | CortenForge | Status | Priority |
|---------|--------|-------------|--------|----------|
| Sparse matrix ops | Native | Missing | **Missing** | High |
| Sleeping bodies | Native | `Body::is_sleeping`, `put_to_sleep()`, `wake_up()` | **Implemented** | - |
| Constraint islands | Auto | `ConstraintIslands` | **Implemented** | - |
| Multi-threading | Model-data separation | Not designed for | **Missing** | Low |
| SIMD | Likely | nalgebra SIMD | **Partial** | Low |

### Implementation Notes: Sleeping Bodies ✅ COMPLETED

Sleeping (deactivation) is a performance optimization that skips simulation for
stationary bodies. This significantly reduces computational cost for scenes with
many resting objects.

**How it works:**
1. Bodies track `sleep_time` - how long they've been below the velocity threshold
2. When `sleep_time >= sleep_time_threshold`, the body is put to sleep
3. Sleeping bodies are skipped during integration, gravity, and damping
4. Bodies wake up automatically when:
   - Force or torque is applied (`apply_force`, `apply_torque`, `apply_force_at_point`)
   - Contact forces are applied
   - Joint constraint forces are applied

**Configuration (in `SolverConfig`):**
- `allow_sleeping: bool` - Enable/disable sleeping globally (default: `true`)
- `sleep_threshold: f64` - Velocity threshold in m/s and rad/s (default: `0.01`)
- `sleep_time_threshold: f64` - Time in seconds before sleeping (default: `0.5`)

**Usage:**
```rust
use sim_core::{World, Stepper};
use sim_types::SimulationConfig;

// Default config has sleeping enabled
let mut config = SimulationConfig::default();

// Customize sleeping behavior
config.solver.sleep_threshold = 0.05;      // More aggressive sleeping
config.solver.sleep_time_threshold = 0.2;  // Sleep faster

// Or disable sleeping for high-accuracy simulations
config.solver.allow_sleeping = false;

// Manual control
if let Some(body) = world.body_mut(body_id) {
    body.put_to_sleep();  // Force body to sleep
    body.wake_up();       // Force body to wake
}
```

**Files:** `sim-core/src/world.rs`, `sim-core/src/stepper.rs`, `sim-types/src/config.rs`

---

## 13. Model Format

| Feature | MuJoCo | CortenForge | Status | Priority |
|---------|--------|-------------|--------|----------|
| URDF loading | Supported | `sim-urdf` crate | **Implemented** | - |
| MJCF loading | Native | Missing | **Missing** | Medium |
| MJB (binary) | Native | Missing | **Missing** | Low |

### Implementation Notes: MJCF Support

MJCF is MuJoCo's native XML format with richer features than URDF. Consider adding `sim-mjcf` crate for compatibility.

---

## Priority Roadmap

### Phase 1: Core Parity (High Priority)

1. ~~**Collision shapes**: Box-box, box-sphere, capsule detection~~ ✅ COMPLETED
2. ~~**Broad-phase**: Sweep-and-prune or BVH integration~~ ✅ COMPLETED
3. ~~**Elliptic friction cones**: Replace circular with elliptic~~ ✅ COMPLETED
4. ~~**Sensors**: IMU, force/torque, touch sensors~~ ✅ COMPLETED
5. ~~**Implicit integration**: Implicit-in-velocity method~~ ✅ COMPLETED

### Phase 2: Solver Improvements (Medium Priority)

1. ~~**Newton solver**: For faster convergence~~ ✅ COMPLETED
2. ~~**Constraint islands**: For performance~~ ✅ COMPLETED
3. ~~**Sleeping**: Deactivate stationary bodies~~ ✅ COMPLETED
4. ~~**GJK/EPA**: For convex mesh collision~~ ✅ COMPLETED

### Phase 3: Extended Features (Lower Priority)

1. **MJCF loading**: For MuJoCo model compatibility
2. **Muscle actuators**: For biomechanics
3. **Tendons**: For cable robots
4. **Deformables**: For soft body simulation

---

## File Reference

| Crate | Purpose | Key Files |
|-------|---------|-----------|
| `sim-types` | Data structures | `dynamics.rs`, `joint.rs`, `observation.rs` |
| `sim-core` | Integration, World | `integrators.rs`, `world.rs`, `stepper.rs`, `broad_phase.rs`, `gjk_epa.rs` |
| `sim-contact` | Contact physics | `model.rs`, `friction.rs`, `solver.rs` |
| `sim-constraint` | Joint constraints | `joint.rs`, `solver.rs`, `newton.rs`, `islands.rs` |
| `sim-sensor` | Sensor simulation | `imu.rs`, `force_torque.rs`, `touch.rs` |
| `sim-urdf` | Robot loading | `loader.rs`, `parser.rs` |
| `sim-physics` | Umbrella | `lib.rs` |

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
