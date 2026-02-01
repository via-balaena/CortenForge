# sim-bevy Implementation Plan

> **Target:** Bevy 0.18 | **Layer:** 1 (Bevy allowed) | **Standard:** A-Grade

A high-quality physics visualization layer for CortenForge's simulation stack.

---

## Overview

sim-bevy bridges the headless sim-* crates (Layer 0) to Bevy's rendering and ECS (Layer 1). It provides:

- Real-time visualization of physics simulations
- Debug overlays for contacts, forces, and constraints
- MJCF/URDF model loading and inspection
- Optional ray-traced rendering via Solari

**Design Philosophy:**
- The viewer is a *window into* sim-core, not a replacement
- sim-core remains the source of truth for physics state
- All rendering is derived from sim-core data structures
- Zero physics logic in sim-bevy
- **Bevy-upgrade-resilient:** Minimize API surface, isolate Bevy types

---

## Bevy Upgrade Strategy

Bevy releases major versions every 3-4 months with breaking changes. This section defines how sim-bevy stays maintainable across upgrades.

### The Problem

| Version | Release | Notable Breaks |
|---------|---------|----------------|
| 0.15 | ~Aug 2025 | Required States rework |
| 0.16 | ~Oct 2025 | Camera API changes |
| 0.17 | ~Dec 2025 | Solari introduced, system ordering changes |
| 0.18 | Jan 2026 | Camera controllers, feature collections |

Each upgrade typically requires 2-8 hours of migration work for a medium-sized plugin.

### Mitigation Strategies

#### 1. Thin Bevy Layer

Keep Bevy types at the boundary. Internal logic uses sim-core types.

```rust
// ❌ Bad: Bevy types throughout
pub fn sync_transforms(
    query: Query<(&ModelBodyIndex, &mut Transform)>,
    data: Res<PhysicsData>,
) {
    for (body_idx, mut transform) in &query {
        // Bevy's Transform used directly with physics data
        transform.translation = data.xpos[body_idx.0].into();
    }
}

// ✅ Good: Bevy at boundary, conversion functions isolate types
pub fn sync_transforms(
    query: Query<(&ModelBodyIndex, &mut Transform)>,
    data: Res<PhysicsData>,
) {
    for (body_idx, mut transform) in &query {
        // Convert at boundary only via convert.rs helpers
        let pos = &data.0.xpos[body_idx.0];
        let quat = &data.0.xquat[body_idx.0];
        transform.translation = vec3_from_vector(pos);  // Convert once
        transform.rotation = quat_from_unit_quaternion(quat);
    }
}
```

#### 2. Conversion Traits in One Place

All Bevy ↔ sim-core conversions in a single module:

```rust
// src/convert.rs - THE ONLY FILE THAT KNOWS BOTH TYPES

use bevy::math::{Vec3, Quat};
use sim_types::{Point3, UnitQuaternion};

pub trait IntoBevy<T> {
    fn into_bevy(self) -> T;
}

pub trait FromBevy<T> {
    fn from_bevy(value: T) -> Self;
}

impl IntoBevy<Vec3> for Point3<f64> {
    fn into_bevy(self) -> Vec3 {
        Vec3::new(self.x as f32, self.y as f32, self.z as f32)
    }
}

impl IntoBevy<Quat> for UnitQuaternion<f64> {
    fn into_bevy(self) -> Quat {
        let q = self.quaternion();
        Quat::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32)
    }
}

// When Bevy changes Vec3/Quat, only this file changes
```

#### 3. Avoid Unstable Bevy Features

| Feature | Stability | Use? |
|---------|-----------|------|
| ECS basics (Entity, Component, Resource) | Stable | ✅ Yes |
| System scheduling (Update, PostUpdate) | Stable | ✅ Yes |
| Transform, GlobalTransform | Stable | ✅ Yes |
| Mesh, Material | Mostly stable | ✅ Yes |
| Gizmos | Stable since 0.14 | ✅ Yes |
| Camera controllers | New in 0.18 | ⚠️ Wrap in abstraction |
| Solari | Experimental | ⚠️ Feature-gate, expect breaks |
| UI/egui | Changes frequently | ⚠️ Isolate completely |

#### 4. Abstract Volatile APIs

Wrap Bevy APIs that change frequently:

```rust
// src/camera/orbit.rs

/// Orbit camera abstraction.
///
/// Wraps Bevy's camera system to isolate from API changes.
/// When Bevy's camera API changes, only this module updates.
pub struct OrbitCameraController {
    // Our own state, not Bevy's
    pub target: Vec3,
    pub distance: f32,
    pub azimuth: f32,
    pub elevation: f32,
}

impl OrbitCameraController {
    /// Apply this controller's state to a Bevy Transform.
    ///
    /// This is the only function that touches Bevy's Transform directly.
    pub fn apply_to_transform(&self, transform: &mut Transform) {
        let x = self.distance * self.azimuth.cos() * self.elevation.cos();
        let y = self.distance * self.elevation.sin();
        let z = self.distance * self.azimuth.sin() * self.elevation.cos();

        transform.translation = self.target + Vec3::new(x, y, z);
        transform.look_at(self.target, Vec3::Y);
    }
}
```

#### 5. Version-Specific Modules (If Needed)

For major incompatibilities, use conditional compilation:

```rust
// src/compat/mod.rs

#[cfg(feature = "bevy_0_18")]
mod bevy_0_18;
#[cfg(feature = "bevy_0_18")]
pub use bevy_0_18::*;

#[cfg(feature = "bevy_0_19")]
mod bevy_0_19;
#[cfg(feature = "bevy_0_19")]
pub use bevy_0_19::*;
```

**Use sparingly.** This adds maintenance burden. Prefer migration over multi-version support.

#### 6. Upgrade Checklist Template

When Bevy releases a new version:

```markdown
## Bevy 0.X → 0.Y Migration

- [ ] Read [Bevy 0.Y Migration Guide](https://bevy.org/news/bevy-0-Y/)
- [ ] Update `Cargo.toml`: `bevy = "0.Y"`
- [ ] Run `cargo check` - note all errors
- [ ] Update `src/convert.rs` for type changes
- [ ] Update camera module for camera API changes
- [ ] Update plugin registration for system changes
- [ ] Run all tests
- [ ] Run all examples visually
- [ ] Update docs for any API changes
- [ ] Commit: "chore: upgrade to Bevy 0.Y"
```

### Upgrade Budget

Plan for Bevy upgrades in the development cycle:

| Bevy Release Cycle | sim-bevy Effort | Notes |
|--------------------|-----------------|-------|
| Minor (0.18.1) | ~0 hours | Usually compatible |
| Major (0.19) | 4-8 hours | Expect API breaks |
| With Solari changes | +2-4 hours | Experimental feature |

**Recommendation:** Upgrade within 2-4 weeks of Bevy release. Staying too far behind makes jumps painful.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         sim-bevy (L1)                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ SimPlugin   │  │ DebugPlugin │  │ SolariPlugin (optional) │  │
│  └──────┬──────┘  └──────┬──────┘  └────────────┬────────────┘  │
│         │                │                      │               │
│  ┌──────▼──────┐  ┌──────▼──────┐  ┌───────────▼───────────┐   │
│  │   Systems   │  │  Overlays   │  │   Ray-traced render   │   │
│  │  - sync     │  │  - contacts │  │   (RTX required)      │   │
│  │  - render   │  │  - forces   │  └───────────────────────┘   │
│  │  - camera   │  │  - joints   │                               │
│  └──────┬──────┘  └─────────────┘                               │
└─────────┼───────────────────────────────────────────────────────┘
          │ reads
┌─────────▼───────────────────────────────────────────────────────┐
│                    sim-core World (L0)                          │
│  Bodies, Joints, Contacts, Collision Shapes                     │
└─────────────────────────────────────────────────────────────────┘
```

---

## Crate Structure

```
sim/L1/bevy/
├── Cargo.toml
├── src/
│   ├── lib.rs                 # Public API, plugin re-exports, prelude
│   ├── plugin.rs              # SimViewerPlugin, OrbitCameraPlugin
│   ├── components.rs          # CollisionShapeVisual, ShapeType
│   ├── resources.rs           # ViewerConfig, DebugColors, BodyEntityMap, CachedContacts,
│   │                          #   MuscleVisualization, TendonVisualization, SensorVisualization
│   ├── model_data.rs          # Model/Data integration: PhysicsModel, PhysicsData,
│   │                          #   ModelDataPlugin, ModelBodyIndex, ModelGeomIndex,
│   │                          #   step_model_data, sync_model_data_to_bevy
│   ├── systems.rs             # Core ECS systems
│   ├── convert.rs             # Z-up ↔ Y-up coordinate conversion helpers
│   ├── gizmos.rs              # Contact, force, joint debug visualization
│   ├── camera.rs              # OrbitCamera component + input systems
│   └── mesh.rs                # CollisionShape → Bevy Mesh generation
│
├── examples/
│   ├── falling_sphere.rs      # Minimal: one sphere, gravity
│   ├── collision_shapes.rs    # All primitive shapes
│   ├── contact_debug.rs       # Contact point visualization
│   ├── simple_pendulum.rs     # MuJoCo-exact single pendulum
│   ├── double_pendulum.rs     # Two-link pendulum
│   ├── spherical_pendulum.rs  # 3D pendulum motion
│   ├── nlink_pendulum.rs      # Multi-link pendulum chain
│   ├── model_data_demo.rs     # Model/Data architecture demo
│   └── mjcf_humanoid.rs       # MJCF model loading (humanoid)
│
└── (tests inline in modules)
```

---

## L0 Dependency API Reference

This section documents the actual APIs available from Layer 0 crates, verified against the codebase.

### sim-core: Model/Data Architecture

The physics engine uses MuJoCo's Model/Data split: a static `Model` (kinematic tree, parameters) and a mutable `Data` (state, computed quantities).

```rust
use sim_core::{Model, Data};

// Model - static, immutable after loading
model.nbody                    // Number of bodies
model.njnt                     // Number of joints
model.ngeom                    // Number of geoms
model.nq                       // Position coordinates count
model.nv                       // Velocity DOFs count
model.gravity                  // Gravity vector (Vector3<f64>)
model.timestep                 // Simulation step size
model.body_name[i]             // Body names
model.jnt_type[i]              // Joint types (MjJointType)
model.geom_type[i]             // Geom types
model.geom_size[i]             // Geom sizes

// Data - dynamic, mutable state
let mut data = model.make_data();

// Simulation stepping
data.forward(&model)?;         // Forward kinematics only
data.step(&model)?;            // Full simulation step

// Source-of-truth state
data.qpos                      // Joint positions (DVector<f64>)
data.qvel                      // Joint velocities (DVector<f64>)
data.time                      // Simulation time

// Computed quantities (populated by forward/step)
data.xpos[i]                   // World-frame body positions (Vector3<f64>)
data.xquat[i]                  // World-frame body quaternions (UnitQuaternion<f64>)
data.geom_xpos[i]              // Geom positions
data.geom_xmat[i]              // Geom orientation matrices
data.site_xpos[i]              // Site positions
data.site_xmat[i]              // Site orientation matrices
data.qacc                      // Joint accelerations
data.energy_kinetic             // Kinetic energy
data.energy_potential           // Potential energy
data.contacts                  // Active contact points
```

### sim-types: Core Types

All fundamental types for physics simulation.

```rust
use sim_types::{BodyId, Pose, RigidBodyState, Twist, MassProperties};

// BodyId - simple wrapper
BodyId::new(u64) -> BodyId
body_id.raw() -> u64

// Pose - position + orientation
Pose {
    position: Point3<f64>,
    rotation: UnitQuaternion<f64>,
}
Pose::identity() -> Pose
Pose::from_position(Point3<f64>) -> Pose
Pose::from_position_rotation(Point3<f64>, UnitQuaternion<f64>) -> Pose
pose.transform_point(&Point3<f64>) -> Point3<f64>
pose.transform_vector(&Vector3<f64>) -> Vector3<f64>

// RigidBodyState - full kinematic state
RigidBodyState {
    pose: Pose,
    twist: Twist,
}
RigidBodyState::at_rest(pose: Pose) -> RigidBodyState

// Twist - velocities
Twist {
    linear: Vector3<f64>,
    angular: Vector3<f64>,
}

// MassProperties
MassProperties::sphere(mass: f64, radius: f64) -> MassProperties
MassProperties::cuboid(mass: f64, half_extents: Vector3<f64>) -> MassProperties
// ... other factory methods
```

### sim-core: CollisionShape Enum

```rust
use sim_core::CollisionShape;

pub enum CollisionShape {
    Sphere { radius: f64 },
    Box { half_extents: Vector3<f64> },
    Capsule { half_length: f64, radius: f64 },
    Cylinder { half_length: f64, radius: f64 },
    Ellipsoid { radii: Vector3<f64> },
    Plane { normal: Vector3<f64>, distance: f64 },
    ConvexMesh { vertices: Vec<Point3<f64>> },
    TriangleMesh { data: Arc<TriangleMeshData> },
    HeightField { data: Arc<HeightFieldData> },
    Sdf { data: Arc<SdfCollisionData> },
}

// Factory methods
CollisionShape::sphere(radius: f64) -> CollisionShape
CollisionShape::box_shape(half_extents: Vector3<f64>) -> CollisionShape
CollisionShape::capsule(half_length: f64, radius: f64) -> CollisionShape
CollisionShape::cylinder(half_length: f64, radius: f64) -> CollisionShape
CollisionShape::ellipsoid(radii: Vector3<f64>) -> CollisionShape
CollisionShape::ground_plane(height: f64) -> CollisionShape
CollisionShape::convex_mesh(vertices: Vec<Point3<f64>>) -> CollisionShape
shape.bounding_radius() -> f64
```

### sim-core: Contact Types

```rust
use sim_core::{ContactPoint, ContactManifold, ContactForce};

// ContactPoint - single contact
pub struct ContactPoint {
    pub position: Point3<f64>,      // World coordinates
    pub normal: Vector3<f64>,       // Unit normal, B→A direction
    pub penetration: f64,           // Positive when overlapping
    pub body_a: BodyId,
    pub body_b: BodyId,
}

// ContactManifold - collection of contacts between two bodies
pub struct ContactManifold {
    pub points: Vec<ContactPoint>,
    pub body_a: BodyId,
    pub body_b: BodyId,
}
manifold.iter() -> impl Iterator<Item = &ContactPoint>
manifold.average_normal() -> Vector3<f64>
manifold.max_penetration() -> f64
manifold.centroid() -> Option<Point3<f64>>

// ContactForce - resolved contact force
pub struct ContactForce {
    pub normal: Vector3<f64>,       // Normal force component
    pub friction: Vector3<f64>,     // Tangential friction force
    pub position: Point3<f64>,      // Application point
}
force.total() -> Vector3<f64>
force.torque_about(point: Point3<f64>) -> Vector3<f64>
```

### sim-mjcf: MJCF Loading

```rust
use sim_mjcf::load_model;

// Load from string — returns sim_core::Model directly
let model: Model = load_model(mjcf_xml_str)?;

// Load from file
let model: Model = sim_mjcf::load_model_from_file("path/to/model.xml")?;

// Lower-level: parse then convert separately
let mjcf: MjcfModel = sim_mjcf::parse_mjcf_str(xml)?;
let model: Model = sim_mjcf::model_from_mjcf(&mjcf)?;

// MJB binary format (requires "mjb" feature)
let model = sim_mjcf::load_mjb_file("model.mjb")?;
sim_mjcf::save_mjb_file(&model, "model.mjb")?;

// Create mutable data from model
let mut data = model.make_data();
```

**Supported geometry:** sphere, box, capsule, cylinder, ellipsoid, plane, mesh (convex + non-convex), sdf

### sim-urdf: URDF Loading

```rust
use sim_urdf::load_urdf_model;

// Load from string — returns sim_core::Model directly
let model: Model = load_urdf_model(urdf_xml_str)?;

// Lower-level: parse then convert via MJCF intermediate
let robot: UrdfRobot = sim_urdf::parse_urdf_str(xml)?;
let mjcf_xml: String = sim_urdf::robot_to_mjcf(&robot)?;

// Create mutable data from model
let mut data = model.make_data();
```

**Supported joints:** fixed, revolute, continuous, prismatic, floating, planar

### sim-constraint: Joint Types

```rust
use sim_constraint::{
    RevoluteJoint, PrismaticJoint, FixedJoint, SphericalJoint,
    JointLimits, JointMotor,
};

// Joint construction (builder pattern)
let joint = RevoluteJoint::new(parent_id, child_id, axis)
    .with_limits(JointLimits::symmetric(range))
    .with_motor(JointMotor::velocity(target, max_torque))
    .with_damping(coefficient);

// Available joint types
RevoluteJoint     // Single-axis rotation (hinge)
PrismaticJoint    // Single-axis translation (slider)
FixedJoint        // Rigid connection
SphericalJoint    // Ball-and-socket (3 DOF)
UniversalJoint    // Two perpendicular rotation axes
FreeJoint         // 6 DOF floating base
PlanarJoint       // XY translation + Z rotation
CylindricalJoint  // Translation + rotation on same axis
```

---

## Dependencies

```toml
[package]
name = "sim-bevy"
version = "0.1.0"
edition = "2021"
license = "MIT OR Apache-2.0"
description = "Bevy visualization for CortenForge physics simulation"
keywords = ["physics", "simulation", "visualization", "bevy", "robotics"]
categories = ["simulation", "visualization", "game-development"]

[dependencies]
# Layer 0 simulation crates
sim-core = { path = "../sim-core" }
sim-types = { path = "../sim-types" }
sim-constraint = { path = "../sim-constraint" }
sim-mjcf = { path = "../sim-mjcf" }
sim-urdf = { path = "../sim-urdf" }

# Bevy 0.18 - use feature collections for clean compilation
bevy = { version = "0.18", default-features = false, features = [
    "3d",                    # 3D rendering, cameras, lights
    "bevy_pbr",              # PBR materials
    "bevy_gizmos",           # Debug line/shape drawing
    "bevy_winit",            # Window management
    "bevy_state",            # State management
    "x11",                   # Linux display (or wayland)
    "wayland",
] }

# Math compatibility
nalgebra = { workspace = true }
glam = { workspace = true }

# Error handling
thiserror = { workspace = true }

[dev-dependencies]
bevy = { version = "0.18", features = ["dynamic_linking"] }

[features]
default = []
# Ray-traced rendering (requires RTX GPU)
solari = ["bevy/bevy_solari"]
# GPU profiling
trace = ["bevy/trace"]
```

---

## Core Components

### Model/Data Components

Link Bevy entities to bodies, geoms, and sites in the physics Model/Data system.

```rust
/// Component linking a Bevy entity to a body in the Model/Data system.
///
/// Uses a direct index into the Model's body arrays. Body transforms
/// are synchronized from `data.xpos[index]` and `data.xquat[index]`.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ModelBodyIndex(pub usize);

/// Component linking a Bevy entity to a geom in the Model/Data system.
///
/// Used for entities representing collision geometries. Transforms are
/// synchronized from `data.geom_xpos[index]` and `data.geom_xmat[index]`.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ModelGeomIndex(pub usize);

/// Component linking a Bevy entity to a site in the Model/Data system.
///
/// Sites are attachment points for sensors, actuators, etc.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ModelSiteIndex(pub usize);

/// Marker component for the root entity of a Model/Data physics scene.
#[derive(Component, Debug, Default, Clone, Copy)]
pub struct ModelDataRoot;
```

### CollisionShapeVisual

Marks an entity as a collision shape visualization.

```rust
/// Visual representation of a collision shape.
///
/// This component is added to child entities to render collision geometry.
#[derive(Component, Debug, Clone)]
pub struct CollisionShapeVisual {
    /// The shape type being visualized.
    pub shape_type: ShapeType,
    /// Whether to render as wireframe.
    pub wireframe: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum ShapeType {
    Sphere,
    Box,
    Capsule,
    Cylinder,
    Ellipsoid,
    Plane,
    ConvexMesh,
    TriangleMesh,
    HeightField,
    Sdf,
}

impl ShapeType {
    /// Convert from a sim-core CollisionShape to ShapeType.
    pub fn from_collision_shape(shape: &sim_core::CollisionShape) -> Self {
        match shape {
            sim_core::CollisionShape::Sphere { .. } => Self::Sphere,
            sim_core::CollisionShape::Box { .. } => Self::Box,
            // ... etc for each variant
        }
    }
}
```

### ViewerConfig

Runtime configuration for the viewer. Controls what debug information is displayed and how it is rendered.

```rust
/// Configuration for the physics viewer.
#[derive(Resource, Debug, Clone)]
pub struct ViewerConfig {
    // Visibility toggles
    pub show_collision_shapes: bool,
    pub show_contacts: bool,
    pub show_contact_normals: bool,
    pub show_forces: bool,
    pub show_joint_axes: bool,
    pub show_joint_limits: bool,
    pub show_velocities: bool,
    pub show_muscles: bool,
    pub show_tendons: bool,
    pub show_sensors: bool,

    // Scale factors
    pub force_scale: f32,
    pub velocity_scale: f32,
    pub contact_marker_radius: f32,
    pub contact_normal_length: f32,
    pub joint_axis_length: f32,
    pub joint_limit_arc_radius: f32,
    pub joint_limit_arc_segments: u32,
    pub joint_marker_radius: f32,

    // Thresholds
    pub force_display_threshold: f32,
    pub velocity_display_threshold: f32,
    pub joint_line_distance_threshold: f32,

    // Musculoskeletal
    pub muscle_line_radius: f32,
    pub tendon_line_radius: f32,
    pub sensor_force_scale: f32,
    pub sensor_axis_length: f32,

    /// Color scheme for debug visualization.
    pub colors: DebugColors,
}
```

Additional visualization resources:
- `CachedContacts` — stores contact points from the last physics step
- `BodyEntityMap` — bidirectional `HashMap<BodyId, Entity>` for O(1) lookups
- `MuscleVisualization` — user-provided muscle state for rendering
- `TendonVisualization` — user-provided tendon paths for rendering
- `SensorVisualization` — user-provided sensor state for rendering

---

## Core Systems

### Transform Synchronization

```rust
/// Synchronize body transforms from physics Data to Bevy entities.
///
/// Reads xpos and xquat from the physics data (computed by forward
/// kinematics) and updates Bevy Transform components.
/// Coordinate system conversion (Z-up → Y-up) is handled automatically.
///
/// Runs in PostUpdate after physics stepping.
pub fn sync_model_data_to_bevy(
    data: Res<PhysicsData>,
    mut bodies: Query<(&ModelBodyIndex, &mut Transform)>,
) {
    for (body_idx, mut transform) in &mut bodies {
        let idx = body_idx.0;
        if idx < data.0.xpos.len() {
            let pos = &data.0.xpos[idx];
            let quat = &data.0.xquat[idx];
            transform.translation = vec3_from_vector(pos);
            transform.rotation = quat_from_unit_quaternion(quat);
        }
    }
}

/// Synchronize geom transforms from physics Data to Bevy entities.
pub fn sync_geom_transforms(
    data: Res<PhysicsData>,
    mut geoms: Query<(&ModelGeomIndex, &mut Transform)>,
) { /* reads data.geom_xpos[idx] and data.geom_xmat[idx] */ }

/// Synchronize site transforms from physics Data to Bevy entities.
pub fn sync_site_transforms(
    data: Res<PhysicsData>,
    mut sites: Query<(&ModelSiteIndex, &mut Transform)>,
) { /* reads data.site_xpos[idx] and data.site_xmat[idx] */ }
```

### Shape Mesh Generation

```rust
/// Generates Bevy meshes from sim-core collision shapes.
pub fn generate_shape_mesh(shape: &CollisionShape) -> Mesh {
    match shape {
        CollisionShape::Sphere { radius } => {
            Sphere::new(*radius as f32).mesh().build()
        }
        CollisionShape::Box { half_extents } => {
            Cuboid::new(
                half_extents.x as f32 * 2.0,
                half_extents.y as f32 * 2.0,
                half_extents.z as f32 * 2.0,
            ).mesh().build()
        }
        CollisionShape::Capsule { half_height, radius } => {
            Capsule3d::new(*radius as f32, *half_height as f32 * 2.0)
                .mesh()
                .build()
        }
        CollisionShape::Cylinder { half_height, radius } => {
            Cylinder::new(*radius as f32, *half_height as f32 * 2.0)
                .mesh()
                .build()
        }
        CollisionShape::ConvexMesh { vertices, indices } => {
            mesh_from_convex(vertices, indices)
        }
        CollisionShape::TriangleMesh { mesh } => {
            mesh_from_trimesh(mesh)
        }
        // ... other shapes
    }
}
```

### Contact Visualization

Contact gizmos read from the `CachedContacts` resource (updated each frame from
the physics step) rather than re-detecting contacts or cloning the world.

```rust
/// Draw contact point markers as small spheres.
pub fn draw_contact_points(
    mut gizmos: Gizmos,
    cached_contacts: Res<CachedContacts>,
    config: Res<ViewerConfig>,
) {
    if !config.show_contacts { return; }

    for contact in cached_contacts.contacts() {
        let pos = vec3_from_point(&contact.position);
        gizmos.sphere(
            Isometry3d::from_translation(pos),
            config.contact_marker_radius,
            config.colors.contact_point,
        );
    }
}

/// Draw contact normal arrows.
pub fn draw_contact_normals(
    mut gizmos: Gizmos,
    cached_contacts: Res<CachedContacts>,
    config: Res<ViewerConfig>,
) {
    if !config.show_contact_normals { return; }

    for contact in cached_contacts.contacts() {
        let start = vec3_from_point(&contact.position);
        let normal = vec3_from_vector(&contact.normal);
        let end = start + normal * config.contact_normal_length;
        gizmos.arrow(start, end, config.colors.contact_normal);
    }
}
```

---

## Plugin Structure

### SimViewerPlugin

The main entry point for visualization. Read-only with respect to physics state.

```rust
pub struct SimViewerPlugin {
    pub config: ViewerConfig,
    pub spawn_camera: bool,
    pub spawn_lighting: bool,
    pub enable_camera_input: bool,
    pub enable_debug_gizmos: bool,
}

impl SimViewerPlugin {
    /// Full-featured viewer with camera, lighting, and gizmos.
    pub fn new() -> Self { /* all enabled */ }

    /// Headless mode: no camera, lighting, input, or gizmos.
    pub fn headless() -> Self { /* all disabled */ }

    pub fn with_config(mut self, config: ViewerConfig) -> Self { .. }
    pub fn without_camera(mut self) -> Self { .. }
    pub fn without_lighting(mut self) -> Self { .. }
}

impl Plugin for SimViewerPlugin {
    fn build(&self, app: &mut App) {
        // Resources
        app.insert_resource(self.config.clone())
            .init_resource::<BodyEntityMap>()
            .init_resource::<CachedContacts>()
            .init_resource::<MuscleVisualization>()
            .init_resource::<TendonVisualization>()
            .init_resource::<SensorVisualization>();

        // Core systems
        app.add_systems(PostUpdate, update_shape_visibility);

        // Debug gizmos (if enabled)
        if self.enable_debug_gizmos {
            app.add_systems(PostUpdate, (
                draw_contact_points,
                draw_contact_normals,
                draw_muscles,
                draw_tendons,
                draw_sensors,
            ).in_set(DebugGizmosSet));
        }

        // Camera
        if self.enable_camera_input { app.add_plugins(OrbitCameraPlugin); }
        if self.spawn_camera { app.add_systems(Startup, spawn_orbit_camera); }
        if self.spawn_lighting { app.add_systems(Startup, spawn_default_lighting); }
    }
}
```

### ModelDataPlugin

Handles physics stepping and transform sync for the Model/Data pipeline.

```rust
pub struct ModelDataPlugin {
    /// Whether to automatically step physics in Update.
    pub auto_step: bool,
}

impl Plugin for ModelDataPlugin {
    fn build(&self, app: &mut App) {
        app.configure_sets(Update, (ModelDataSet::Step, ModelDataSet::Sync).chain());

        if self.auto_step {
            app.add_systems(Update,
                step_model_data
                    .in_set(ModelDataSet::Step)
                    .run_if(resource_exists::<PhysicsModel>)
                    .run_if(resource_exists::<PhysicsData>),
            );
        }

        app.add_systems(PostUpdate, (
            sync_model_data_to_bevy,
            sync_geom_transforms,
            sync_site_transforms,
        ).run_if(resource_exists::<PhysicsData>));
    }
}
```

### OrbitCameraPlugin

```rust
/// Orbit camera controller for physics inspection.
///
/// Provides mouse-based orbit, pan, and zoom controls.
pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app
            .add_systems(Startup, spawn_orbit_camera)
            .add_systems(Update, (
                orbit_camera_input,
                update_orbit_camera,
            ).chain());
    }
}

/// Orbit camera component.
#[derive(Component, Debug)]
pub struct OrbitCamera {
    /// Target point to orbit around.
    pub target: Vec3,
    /// Distance from target.
    pub distance: f32,
    /// Horizontal angle (radians).
    pub azimuth: f32,
    /// Vertical angle (radians).
    pub elevation: f32,
    /// Orbit speed (radians per pixel).
    pub orbit_speed: f32,
    /// Pan speed (units per pixel).
    pub pan_speed: f32,
    /// Zoom speed (multiplier per scroll unit).
    pub zoom_speed: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: Vec3::ZERO,
            distance: 5.0,
            azimuth: 0.0,
            elevation: 0.5,
            orbit_speed: 0.005,
            pan_speed: 0.01,
            zoom_speed: 0.1,
        }
    }
}
```

---

## Solari Integration (Optional)

Feature-gated ray-traced rendering for high-quality visualization.

```rust
#[cfg(feature = "solari")]
pub mod solari {
    use bevy::prelude::*;
    use bevy_solari::prelude::*;

    /// Ray-traced rendering plugin for physics visualization.
    ///
    /// Requires an RTX-capable GPU. Falls back gracefully if unavailable.
    pub struct SolariViewerPlugin;

    impl Plugin for SolariViewerPlugin {
        fn build(&self, app: &mut App) {
            app
                .add_plugins(SolariPlugin::default())
                .add_systems(Startup, setup_solari_environment);
        }
    }

    fn setup_solari_environment(mut commands: Commands) {
        // Spawn directional light with soft shadows
        commands.spawn((
            DirectionalLight {
                illuminance: 10000.0,
                shadows_enabled: true,
                ..default()
            },
            Transform::from_rotation(Quat::from_euler(
                EulerRot::XYZ, -0.5, 0.5, 0.0
            )),
        ));

        // Enable Solari for the camera
        // (specific API depends on Bevy 0.18 Solari implementation)
    }
}
```

---

## Implementation Phases

### Phase 1: Foundation (MVP) ✅ COMPLETE

**Goal:** Render a sim-core World with basic shapes and camera control.

**Status:** Completed 2026-01-23

**Bevy Version:** 0.18.0 (upgraded from 0.15)

**Tasks:**
- [x] Create crate structure and Cargo.toml
- [x] Implement `SimViewerPlugin` skeleton
- [x] Implement `ModelBodyIndex` component and transform sync
- [x] Implement primitive mesh generation (Sphere, Box, Capsule, Cylinder, Ellipsoid)
- [x] Implement `OrbitCamera` with mouse controls (using Bevy 0.18 `AccumulatedMouseMotion`)
- [x] Create `falling_sphere.rs` example
- [x] Create `collision_shapes.rs` example
- [x] Write unit tests for mesh generation (5 tests)
- [x] Write unit tests for transform sync (3 tests)
- [x] Write unit tests for camera controls (3 tests)
- [x] Write unit tests for type conversions (2 tests)
- [x] Write integration tests for entity spawning (6 tests)
- [x] Write integration tests for transform sync (5 tests)
- [x] All 25 tests pass, clippy clean (now 32 with Phase 2)

**Test Coverage:**
- 14 unit tests (mesh, camera, convert, systems, plugin, gizmos)
- 6 spawn integration tests (`tests/spawn_tests.rs`)
- 5 sync integration tests (`tests/sync_tests.rs`)
- 7 gizmo integration tests (`tests/gizmo_tests.rs`) [Phase 2]

**Implementation Notes:**
- Module structure: `convert`, `components`, `resources`, `systems`, `mesh`, `camera`, `plugin`
- Type conversions isolated in `convert.rs` for Bevy upgrade resilience
- `BodyEntityMap` provides O(1) lookups for entity lifecycle management
- Ellipsoid mesh generation includes proper normal recalculation
- ConvexMesh falls back to bounding box visualization (proper hull computation deferred)
- Added `SimViewerPlugin::headless()` for testing without input/render resources
- Bevy 0.18 migration: `EventReader` → `AccumulatedMouseMotion`, `despawn_recursive` → `despawn`, `Entity::from_raw` → `Entity::from_bits`

**Deliverable:** Can spawn a sim-core World and watch bodies fall. ✅

### Phase 2: Debug Visualization ✅ COMPLETE

**Goal:** Add contact and joint visualization.

**Status:** Completed 2026-01-23

**Tasks:**
- [x] Implement contact point gizmos
- [x] Implement contact normal arrows
- [x] Implement force vector visualization
- [x] Implement joint axis visualization
- [x] Implement joint limit arc visualization
- [x] Add `ViewerConfig` resource for runtime toggles
- [x] Add `DebugColors` configuration
- [x] Add `enable_debug_gizmos` plugin option
- [x] Create `contact_debug.rs` example with keyboard toggles
- [x] Write gizmo integration tests (7 tests)
- [x] All 32 tests pass, clippy clean

**Implementation Notes:**
- New `gizmos.rs` module with 5 gizmo drawing systems
- Contact detection clones world for snapshot (TODO: cache contacts for performance)
- `SimViewerPlugin::headless()` disables gizmos (requires `bevy_gizmos`)
- Joint axes drawn at midpoint between parent/child bodies
- Joint limits visualized as arc segments with limit markers
- Keyboard toggles: C=contacts, N=normals, F=forces, J=joints, L=limits

**Test Coverage:**
- 7 gizmo configuration tests (`tests/gizmo_tests.rs`)
- ViewerConfig default values and modification
- DebugColors distinctiveness
- Plugin headless/new configuration

**Deliverable:** Can see contacts and forces in real-time. ✅

### Phase 3: Model Loading ✅ COMPLETE

**Goal:** Load and visualize MJCF/URDF models.

**Status:** Completed 2026-01-24

**Tasks:**
- [x] Create `model_data.rs` module with `PhysicsModel`, `PhysicsData`, and `ModelDataPlugin`
- [x] Implement MJCF → Bevy entity spawning via sim-mjcf integration
- [x] Implement URDF → Bevy entity spawning via sim-urdf integration
- [x] Handle mesh geometry (TriangleMesh → Bevy Mesh with proper normals)
- [x] Create `mjcf_viewer.rs` example (simple humanoid)
- [x] Create `urdf_viewer.rs` example (robot arm with gripper)
- [x] Add model loading unit tests (6 tests)
- [x] All 40 tests pass, clippy clean

**Implementation Notes:**
- `model_data.rs` module provides `PhysicsModel`, `PhysicsData`, `ModelBodyIndex`, `ModelGeomIndex`, `ModelSiteIndex`, `ModelDataRoot` types
- `sim_mjcf::load_model()` returns `Model` directly; `sim_urdf::load_urdf_model()` returns `Model` directly
- `spawn_model_bodies()` utility creates Bevy entities for all bodies with proper transforms
- `ModelDataPlugin` provides auto-step and transform sync systems
- `triangle_mesh()` function in `mesh.rs` converts `TriangleMeshData` to Bevy mesh with computed normals
- Examples demonstrate keyboard toggles and runtime info printing

**Test Coverage:**
- 6 new model loading tests in `model_data.rs`:
  - `model_body_index_accessors`, `model_geom_index_accessors`
  - `model_site_index_accessors`, `physics_model_deref`
  - `physics_data_deref_mut`, `file_not_found_error`

**Deliverable:** Can load MJCF/URDF and visualize articulated robots. ✅

### Phase 4: Polish and Performance

**Goal:** Production-quality viewer.

**Tasks:**
- [ ] Profile and optimize transform sync (batch updates)
- [ ] Implement mesh caching (don't regenerate static shapes)
- [ ] Add material presets for different body types
- [ ] Implement follow camera mode
- [ ] Add keyboard shortcuts for debug toggles
- [ ] Benchmark: 1000 bodies at 60fps target
- [ ] Complete documentation (all public items)
- [ ] Achieve ≥75% test coverage

**Deliverable:** Performant, documented, A-grade crate.

### Phase 5: Solari (Optional)

**Goal:** Ray-traced rendering for high-quality visualization.

**Tasks:**
- [ ] Add `solari` feature flag
- [ ] Implement `SolariViewerPlugin`
- [ ] Setup ray-traced materials
- [ ] Handle graceful fallback on non-RTX hardware
- [ ] Create `solari_demo.rs` example
- [ ] Document GPU requirements

**Deliverable:** Optional ray-traced rendering for capable hardware.

---

## Quality Checklist (A-Grade)

Before shipping, all criteria must pass:

| Criterion | Target | Measurement |
|-----------|--------|-------------|
| Test Coverage | ≥75% | `cargo tarpaulin -p sim-bevy` |
| Documentation | 0 warnings | `RUSTDOCFLAGS="-D warnings" cargo doc` |
| Clippy | 0 warnings | `cargo clippy -p sim-bevy -- -D warnings` |
| Safety | 0 unwrap in lib | `grep -r "\.unwrap()" src/` |
| Dependencies | Minimal | Only bevy + sim-* crates |
| API Design | Idiomatic | Manual review |

**Layer 1 specific:**
- [ ] Bevy dependencies use feature collections (not individual features)
- [ ] Plugin follows Bevy conventions (systems in appropriate schedules)
- [ ] Components are small and focused
- [ ] Resources are documented with usage examples

---

## API Examples

### Basic Usage (Model/Data)

```rust
use bevy::prelude::*;
use sim_bevy::prelude::*;
use sim_core::Model;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin::new())
        .add_plugins(ModelDataPlugin::new().with_auto_step())
        .add_systems(Startup, setup_physics)
        .run();
}

fn setup_physics(mut commands: Commands) {
    // Load model from MJCF
    let model = sim_mjcf::load_model("assets/humanoid.xml")
        .expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward kinematics failed");

    // Spawn Bevy entities for each body
    spawn_model_bodies(&mut commands, &model, &data);

    // Insert resources
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}
```

### Loading MJCF

```rust
use bevy::prelude::*;
use sim_bevy::prelude::*;
use sim_core::Model;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin::new())
        .add_plugins(ModelDataPlugin::new().with_auto_step())
        .add_systems(Startup, load_humanoid)
        .run();
}

fn load_humanoid(mut commands: Commands) {
    // load_model() returns a Model directly
    let model = sim_mjcf::load_model("assets/humanoid.xml")
        .expect("Failed to load MJCF");
    let mut data = model.make_data();
    data.forward(&model).expect("forward kinematics failed");

    spawn_model_bodies(&mut commands, &model, &data);
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}
```

### Custom Debug Configuration

```rust
use bevy::prelude::*;
use sim_bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin::new().with_config(ViewerConfig {
            show_contacts: true,
            show_contact_normals: true,
            show_forces: true,
            show_muscles: true,
            force_scale: 0.1,
            colors: DebugColors {
                contact_point: Color::srgb(1.0, 0.0, 0.0),
                contact_normal: Color::srgb(0.0, 1.0, 0.0),
                force_vector: Color::srgb(1.0, 1.0, 0.0),
                ..default()
            },
            ..default()
        }))
        .add_plugins(ModelDataPlugin::new().with_auto_step())
        .run();
}
```

---

## Architectural Decisions

### 1. Simulation Stepping: Read-Only Viewer

**Decision:** sim-bevy does NOT step the simulation. It is a read-only visualization layer.

**Rationale:**
- **Separation of concerns** — sim-bevy is a visualization layer, not a simulation runner. Mixing stepping logic with rendering creates coupling that makes both harder to reason about.
- **Flexibility** — Users may want different stepping strategies: fixed timestep, variable, substeps, paused with manual advance, or running headless and only visualizing occasionally. If sim-bevy owned stepping, we'd have to expose all these options.
- **Testing** — A read-only viewer is trivially testable. Create a World in any state, pass it in, verify rendering. No temporal concerns.
- **Replay/scrubbing** — If the viewer just reads state, timeline scrubbing becomes trivial by swapping which frame's state you point it at.

**Implementation:** The user steps via `data.step(&model)` (or uses `ModelDataPlugin::with_auto_step()`), and sim-bevy syncs transforms in `PostUpdate`.

```rust
// User code - they own the stepping
fn step_physics(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>) {
    data.step(&model).expect("step failed");
}

// sim-bevy - runs after user's Update systems
// PostUpdate: sync_model_data_to_bevy reads data.xpos/xquat, updates Bevy Transforms
```

### 2. Entity Lifecycle: Polling with HashMap Tracking

**Decision:** Use frame-by-frame polling with a `HashMap<BodyId, Entity>` for efficient tracking.

**Rationale:**
- **Simplicity** — Polling is straightforward to implement and debug. No event system needed in sim-core.
- **Performance** — For typical body counts (hundreds to low thousands), O(n) comparison per frame is negligible. The HashMap makes individual lookups O(1).
- **Robustness** — No risk of missed events or ordering issues. The viewer always reflects ground truth.

**Implementation:**

```rust
/// Tracks the mapping between sim-core bodies and Bevy entities.
#[derive(Resource, Default)]
pub struct BodyEntityMap {
    body_to_entity: HashMap<BodyId, Entity>,
    entity_to_body: HashMap<Entity, BodyId>,
}
```

In the Model/Data architecture, the body set is static (defined by the Model), so entities
are spawned once at setup via `spawn_model_bodies()` rather than polled each frame. The
`BodyEntityMap` is still useful for O(1) lookups when correlating physics events with entities.

**Future optimization:** If profiling shows this is a bottleneck, sim-core could expose a generation counter or change events. But start simple.

### 3. Multiple Worlds: Not V1, Designed for Extensibility

**Decision:** V1 supports a single world. The design does not preclude multiple worlds later.

**Rationale:**
- **Simplicity** — Most use cases involve one world: a humanoid simulation, a robot arm, a physics sandbox.
- **Complexity avoidance** — Multiple worlds raise questions: which world does the camera follow? How do you position them visually? Do they interact?
- **Alternative solutions** — The common "multiple world" case is A/B comparison (e.g., two policy rollouts side by side). That's better served by two separate app instances or a split-screen feature.

**Design for extensibility:**
- `PhysicsModel`/`PhysicsData` are single-instance resources, but this isn't hardcoded throughout
- `ModelBodyIndex` uses just an index for V1 (could add a model/world discriminator later)
- `spawn_model_bodies()` takes `&Model` and `&Data` references, so calling with different models would work

```rust
// V1: Single model
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ModelBodyIndex(pub usize);

// Future V2 (if needed): Multi-model support
#[derive(Component, Debug, Clone, Copy)]
pub struct ModelBodyIndex {
    pub model_id: ModelId,  // Added later
    pub body_index: usize,
}
```

If someone needs multiple models later, they can:
1. Run multiple Bevy apps (simplest)
2. Contribute the multi-model feature when they need it

### 4. Headless Testing

**Decision:** Use Bevy's headless mode for CI; visual tests require screenshot comparison or manual verification.

**Implementation:**
- Unit tests for mesh generation, transform math, etc. run without GPU
- Integration tests use `MinimalPlugins` instead of `DefaultPlugins`
- Visual regression tests (if desired) use screenshot comparison tools

```rust
#[cfg(test)]
mod tests {
    use bevy::prelude::*;

    fn test_app() -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(SimViewerPlugin::headless());
        app.add_plugins(ModelDataPlugin::new());
        app
    }

    #[test]
    fn body_transforms_sync_correctly() {
        let mut app = test_app();
        // ... test logic without rendering
    }
}
```

---

## References

- [Bevy 0.18 Release Notes](https://bevy.org/news/bevy-0-18/)
- [Bevy Plugin Guide](https://bevy-cheatbook.github.io/programming/plugins.html)
- [Bevy ECS Guide](https://bevy.org/learn/quick-start/getting-started/ecs/)
- [Solari Ray Tracing](https://jms55.github.io/posts/2025-12-27-solari-bevy-0-18/)
- [CortenForge STANDARDS.md](./STANDARDS.md)
