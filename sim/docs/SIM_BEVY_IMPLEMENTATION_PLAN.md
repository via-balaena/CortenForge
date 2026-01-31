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
    query: Query<(&PhysicsBody, &mut Transform)>,
    world: Res<SimWorld>,
) {
    for (body, mut transform) in &query {
        // Bevy's Transform used directly
        transform.translation = world.get_position(body.id);
    }
}

// ✅ Good: Bevy at boundary, logic uses sim-core types
pub fn sync_transforms(
    query: Query<(&PhysicsBody, &mut Transform)>,
    world: Res<SimWorld>,
) {
    for (body, mut transform) in &query {
        // Convert at boundary only
        let pose = world.get_pose(body.id);  // sim-core Pose
        transform.translation = pose.position.into();  // Convert once
        transform.rotation = pose.orientation.into();
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
sim/sim-bevy/
├── Cargo.toml
├── src/
│   ├── lib.rs                 # Public API, plugin re-exports
│   ├── plugin.rs              # SimViewerPlugin composition
│   │
│   ├── components/
│   │   ├── mod.rs
│   │   ├── body.rs            # PhysicsBody, BodyLink
│   │   ├── shape.rs           # CollisionShape marker, ShapeMesh
│   │   ├── joint.rs           # JointVisual, JointAxis
│   │   └── debug.rs           # ContactMarker, ForceVector
│   │
│   ├── resources/
│   │   ├── mod.rs
│   │   ├── config.rs          # ViewerConfig
│   │   ├── state.rs           # SimulationHandle, PlaybackState
│   │   └── materials.rs       # DebugMaterials (cached handles)
│   │
│   ├── systems/
│   │   ├── mod.rs
│   │   ├── sync.rs            # World → Transform synchronization
│   │   ├── spawn.rs           # Spawn entities from sim-core World
│   │   ├── shapes.rs          # CollisionShape → Mesh generation
│   │   ├── contacts.rs        # Contact point visualization
│   │   ├── forces.rs          # Force/torque vector rendering
│   │   └── joints.rs          # Joint axis/limit visualization
│   │
│   ├── camera/
│   │   ├── mod.rs
│   │   ├── orbit.rs           # OrbitCamera component + systems
│   │   └── follow.rs          # FollowCamera for tracking bodies
│   │
│   ├── mesh/
│   │   ├── mod.rs
│   │   ├── primitives.rs      # Sphere, Box, Capsule, Cylinder meshes
│   │   ├── convex.rs          # ConvexMesh → Bevy Mesh
│   │   └── trimesh.rs         # TriangleMesh → Bevy Mesh
│   │
│   └── solari/                # Optional: Ray-traced rendering
│       ├── mod.rs
│       ├── plugin.rs          # SolariViewerPlugin
│       └── materials.rs       # RT-compatible materials
│
├── examples/
│   ├── falling_sphere.rs      # Minimal: one sphere, gravity
│   ├── collision_shapes.rs    # All primitive shapes
│   ├── mjcf_viewer.rs         # Load and display MJCF model
│   ├── urdf_viewer.rs         # Load and display URDF robot
│   ├── contact_debug.rs       # Contact point visualization
│   └── solari_demo.rs         # Ray-traced rendering (feature-gated)
│
└── tests/
    ├── spawn_tests.rs         # Entity spawning correctness
    ├── sync_tests.rs          # Transform sync accuracy
    ├── gizmo_tests.rs         # Debug gizmo configuration tests
    └── mesh_tests.rs          # Mesh generation tests
```

---

## L0 Dependency API Reference

This section documents the actual APIs available from Layer 0 crates, verified against the codebase.

### sim-core: World Management

The `World` struct is the central simulation container.

```rust
use sim_core::World;

// Body management
world.add_body(state: RigidBodyState, mass: MassProperties) -> BodyId
world.add_static_body(pose: Pose) -> BodyId
world.remove_body(id: BodyId) -> Option<Body>
world.body(id: BodyId) -> Option<&Body>
world.body_mut(id: BodyId) -> Option<&mut Body>
world.body_by_name(name: &str) -> Option<&Body>
world.bodies() -> impl Iterator<Item = &Body>
world.body_ids() -> impl Iterator<Item = BodyId>
world.body_count() -> usize

// Body struct fields
body.state: RigidBodyState      // Contains pose + twist
body.collision_shape: Option<CollisionShape>
body.name: Option<String>

// Joint management
world.add_joint(...) -> JointId
world.joint(id: JointId) -> Option<&Joint>
world.joints() -> impl Iterator<Item = &Joint>

// Simulation stepping (via Stepper)
let mut stepper = Stepper::new();
stepper.step(&mut world);
stepper.run_for(&mut world, duration: f64);

// Direct physics operations
world.detect_contacts() -> Vec<ContactPoint>
world.solve_contacts() -> usize
world.solve_constraints() -> usize
world.apply_gravity()
world.clear_forces()

// Utilities
world.total_kinetic_energy() -> f64
world.total_linear_momentum() -> Vector3<f64>
world.center_of_mass() -> Option<Point3<f64>>
world.validate() -> Result<()>
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
use sim_mjcf::{load_mjcf_file, load_mjcf_str, LoadedModel, SpawnedModel};

// Load and parse
let model: LoadedModel = load_mjcf_file("path/to/model.xml")?;
let model: LoadedModel = load_mjcf_str(xml_content)?;

// Spawn into World
let spawned: SpawnedModel = model.spawn_at_origin(&mut world)?;

// Access spawned entities by name
spawned.body_id("torso") -> Option<BodyId>
spawned.joint_id("hip") -> Option<JointId>
```

**Supported geometry:** sphere, box, capsule, cylinder, ellipsoid, plane, mesh (STL/OBJ/PLY)

### sim-urdf: URDF Loading

```rust
use sim_urdf::{load_urdf_file, load_urdf_str, LoadedRobot, SpawnedRobot};

// Load and parse
let robot: LoadedRobot = load_urdf_file("path/to/robot.urdf")?;
let robot: LoadedRobot = load_urdf_str(xml_content)?;

// Spawn into World
let spawned: SpawnedRobot = robot.spawn_at_origin(&mut world)?;

// Access spawned entities by name
spawned.link_id("base_link") -> Option<BodyId>
spawned.joint_id("joint1") -> Option<JointId>
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

### PhysicsBody

Links a Bevy entity to a sim-core body.

```rust
/// Links this entity to a body in the physics simulation.
///
/// The transform of this entity will be synchronized with the body's pose
/// each frame.
#[derive(Component, Debug, Clone, Copy)]
pub struct PhysicsBody {
    /// The body ID in the sim-core World.
    pub body_id: BodyId,
}

/// Marker for the root entity of a spawned physics world.
#[derive(Component, Debug, Default)]
pub struct PhysicsWorldRoot;
```

### CollisionShapeVisual

Marks an entity as a collision shape visualization.

```rust
/// Visual representation of a collision shape.
///
/// This component is added to child entities of [`PhysicsBody`] entities
/// to render their collision geometry.
#[derive(Component, Debug, Clone)]
pub struct CollisionShapeVisual {
    /// The shape type being visualized.
    pub shape_type: ShapeType,
    /// Whether to render as wireframe.
    pub wireframe: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ShapeType {
    Sphere,
    Box,
    Capsule,
    Cylinder,
    Ellipsoid,
    ConvexMesh,
    TriangleMesh,
    HeightField,
    Sdf,
}
```

### ViewerConfig

Runtime configuration for the viewer.

```rust
/// Configuration for the physics viewer.
#[derive(Resource, Debug, Clone)]
pub struct ViewerConfig {
    /// Whether to show collision shapes.
    pub show_collision_shapes: bool,
    /// Whether to show contact points.
    pub show_contacts: bool,
    /// Whether to show contact normals.
    pub show_contact_normals: bool,
    /// Whether to show applied forces.
    pub show_forces: bool,
    /// Whether to show joint axes.
    pub show_joint_axes: bool,
    /// Whether to show joint limits.
    pub show_joint_limits: bool,
    /// Scale factor for force vectors.
    pub force_scale: f32,
    /// Radius of contact point markers.
    pub contact_marker_radius: f32,
    /// Color scheme for debug visualization.
    pub color_scheme: ColorScheme,
}

impl Default for ViewerConfig {
    fn default() -> Self {
        Self {
            show_collision_shapes: true,
            show_contacts: true,
            show_contact_normals: true,
            show_forces: false,
            show_joint_axes: true,
            show_joint_limits: false,
            force_scale: 0.01,
            contact_marker_radius: 0.02,
            color_scheme: ColorScheme::default(),
        }
    }
}
```

---

## Core Systems

### Transform Synchronization

```rust
/// Synchronizes Bevy transforms with sim-core body poses.
///
/// This system runs in `PostUpdate` to ensure physics has stepped
/// before transforms are updated for rendering.
pub fn sync_body_transforms(
    sim_world: Res<SimulationHandle>,
    mut bodies: Query<(&PhysicsBody, &mut Transform)>,
) {
    let world = match sim_world.world() {
        Some(w) => w,
        None => return,
    };

    for (physics_body, mut transform) in &mut bodies {
        if let Some(body) = world.body(physics_body.body_id) {
            let pose = &body.state.pose;
            transform.translation = vec3_from_nalgebra(pose.position);
            transform.rotation = quat_from_nalgebra(pose.orientation);
        }
    }
}
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

```rust
/// Spawns gizmos for active contact points.
pub fn draw_contact_points(
    sim_world: Res<SimulationHandle>,
    config: Res<ViewerConfig>,
    mut gizmos: Gizmos,
) {
    if !config.show_contacts {
        return;
    }

    let contacts = match sim_world.contacts() {
        Some(c) => c,
        None => return,
    };

    for contact in contacts {
        let pos = vec3_from_nalgebra(contact.point);

        // Draw contact point sphere
        gizmos.sphere(
            pos,
            config.contact_marker_radius,
            config.color_scheme.contact_point,
        );

        // Draw contact normal
        if config.show_contact_normals {
            let normal = vec3_from_nalgebra(contact.normal);
            gizmos.arrow(
                pos,
                pos + normal * 0.1,
                config.color_scheme.contact_normal,
            );
        }
    }
}
```

---

## Plugin Structure

### SimViewerPlugin

The main entry point.

```rust
/// Physics visualization plugin for Bevy.
///
/// This plugin provides real-time visualization of sim-core physics
/// simulations, including collision shapes, contacts, forces, and joints.
///
/// # Example
///
/// ```no_run
/// use bevy::prelude::*;
/// use sim_bevy::SimViewerPlugin;
///
/// fn main() {
///     App::new()
///         .add_plugins(DefaultPlugins)
///         .add_plugins(SimViewerPlugin::default())
///         .run();
/// }
/// ```
pub struct SimViewerPlugin {
    /// Initial viewer configuration.
    pub config: ViewerConfig,
}

impl Default for SimViewerPlugin {
    fn default() -> Self {
        Self {
            config: ViewerConfig::default(),
        }
    }
}

impl Plugin for SimViewerPlugin {
    fn build(&self, app: &mut App) {
        app
            // Resources
            .insert_resource(self.config.clone())
            .init_resource::<DebugMaterials>()

            // Core systems
            .add_systems(Update, (
                spawn_physics_entities,
                sync_body_transforms,
                update_shape_visibility,
            ).chain())

            // Debug visualization (uses gizmos)
            .add_systems(PostUpdate, (
                draw_contact_points,
                draw_force_vectors,
                draw_joint_axes,
            ))

            // Camera
            .add_plugins(OrbitCameraPlugin);
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
- [x] Implement `PhysicsBody` component and transform sync
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
- [x] Create `models.rs` module with `MjcfModel` and `UrdfModel` wrappers
- [x] Implement MJCF → Bevy entity spawning via sim-mjcf integration
- [x] Implement URDF → Bevy entity spawning via sim-urdf integration
- [x] Handle mesh geometry (TriangleMesh → Bevy Mesh with proper normals)
- [x] Create `mjcf_viewer.rs` example (simple humanoid)
- [x] Create `urdf_viewer.rs` example (robot arm with gripper)
- [x] Add model loading unit tests (6 tests)
- [x] All 40 tests pass, clippy clean

**Implementation Notes:**
- New `models.rs` module provides `MjcfModel`, `UrdfModel`, `SpawnedMjcf`, `SpawnedUrdf` types
- `MjcfModel::from_file()` / `from_xml()` for loading from path or string
- `UrdfModel::from_file()` / `from_xml()` for loading from path or string
- `spawn_into()` / `spawn_at()` methods spawn into `SimulationHandle`
- Models export to prelude: `MjcfModel`, `UrdfModel`, `SpawnedMjcf`, `SpawnedUrdf`, `ModelError`, `ModelSource`, `ModelType`
- `ModelSource` component tracks which model a body came from
- `triangle_mesh()` function in `mesh.rs` converts `TriangleMeshData` to Bevy mesh with computed normals
- Examples demonstrate keyboard toggles and runtime info printing

**Test Coverage:**
- 6 new model loading tests in `models.rs`:
  - `mjcf_model_from_str`, `urdf_model_from_str`
  - `mjcf_spawn_into_world`, `urdf_spawn_into_world`
  - `spawn_fails_without_world`, `file_not_found_error`
  - `model_source_component`

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

### Basic Usage

```rust
use bevy::prelude::*;
use sim_bevy::prelude::*;
use sim_core::World;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin::default())
        .add_systems(Startup, setup_physics)
        .run();
}

fn setup_physics(mut commands: Commands) {
    // Create a sim-core world
    let mut world = World::default();

    // Add a sphere
    let sphere = world.add_body(
        RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 5.0, 0.0))),
        MassProperties::sphere(1.0, 0.5),
    );
    world.set_collision_shape(sphere, CollisionShape::Sphere { radius: 0.5 });

    // Add ground plane
    let ground = world.add_static_body(Pose::identity());
    world.set_collision_shape(ground, CollisionShape::Plane { normal: Vector3::y() });

    // Spawn the world for visualization
    commands.spawn_physics_world(world);
}
```

### Loading MJCF

```rust
use bevy::prelude::*;
use sim_bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin::default())
        .add_systems(Startup, load_humanoid)
        .run();
}

fn load_humanoid(mut commands: Commands) {
    // Load MJCF model
    let mjcf = sim_mjcf::load("assets/humanoid.xml")
        .expect("Failed to load MJCF");

    // Convert to sim-core world
    let world = sim_mjcf::to_world(&mjcf)
        .expect("Failed to convert MJCF");

    // Spawn for visualization
    commands.spawn_physics_world(world);
}
```

### Custom Debug Configuration

```rust
use bevy::prelude::*;
use sim_bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin {
            config: ViewerConfig {
                show_contacts: true,
                show_contact_normals: true,
                show_forces: true,
                force_scale: 0.1,
                color_scheme: ColorScheme {
                    contact_point: Color::srgb(1.0, 0.0, 0.0),
                    contact_normal: Color::srgb(0.0, 1.0, 0.0),
                    force_vector: Color::srgb(1.0, 1.0, 0.0),
                    ..default()
                },
                ..default()
            },
        })
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

**Implementation:** The user steps the simulation themselves (likely in `Update` or `FixedUpdate`), and sim-bevy syncs transforms in `PostUpdate`.

```rust
// User code - they own the stepping
fn step_physics(mut sim_world: ResMut<SimulationHandle>, time: Res<Time>) {
    sim_world.step(time.delta_secs_f64());
}

// sim-bevy - runs after user's Update systems
// PostUpdate: sync_body_transforms reads from sim_world, updates Bevy Transforms
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

/// Synchronizes Bevy entities with sim-core bodies.
///
/// Spawns entities for new bodies, despawns entities for removed bodies.
pub fn sync_physics_entities(
    mut commands: Commands,
    sim_world: Res<SimulationHandle>,
    mut body_map: ResMut<BodyEntityMap>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let Some(world) = sim_world.world() else { return };

    // Collect current body IDs
    let current_bodies: HashSet<BodyId> = world.body_ids().collect();
    let tracked_bodies: HashSet<BodyId> = body_map.body_to_entity.keys().copied().collect();

    // Spawn new bodies
    for &body_id in current_bodies.difference(&tracked_bodies) {
        let entity = spawn_body_entity(&mut commands, world, body_id, &mut meshes, &mut materials);
        body_map.body_to_entity.insert(body_id, entity);
        body_map.entity_to_body.insert(entity, body_id);
    }

    // Despawn removed bodies
    for &body_id in tracked_bodies.difference(&current_bodies) {
        if let Some(entity) = body_map.body_to_entity.remove(&body_id) {
            body_map.entity_to_body.remove(&entity);
            commands.entity(entity).despawn_recursive();
        }
    }
}
```

**Future optimization:** If profiling shows this is a bottleneck, sim-core could expose a generation counter or change events. But start simple.

### 3. Multiple Worlds: Not V1, Designed for Extensibility

**Decision:** V1 supports a single world. The design does not preclude multiple worlds later.

**Rationale:**
- **Simplicity** — Most use cases involve one world: a humanoid simulation, a robot arm, a physics sandbox.
- **Complexity avoidance** — Multiple worlds raise questions: which world does the camera follow? How do you position them visually? Do they interact?
- **Alternative solutions** — The common "multiple world" case is A/B comparison (e.g., two policy rollouts side by side). That's better served by two separate app instances or a split-screen feature.

**Design for extensibility:**
- `SimulationHandle` stores a single world, but this isn't hardcoded throughout
- `PhysicsBody` uses just `body_id: BodyId` for V1 (could add `world_id` later)
- Spawning logic lives in functions that take a world reference, so calling them with different worlds would work

```rust
// V1: Single world
#[derive(Component, Debug, Clone, Copy)]
pub struct PhysicsBody {
    pub body_id: BodyId,
}

// Future V2 (if needed): Multi-world support
#[derive(Component, Debug, Clone, Copy)]
pub struct PhysicsBody {
    pub world_id: WorldId,  // Added later
    pub body_id: BodyId,
}
```

If someone needs multiple worlds later, they can:
1. Run multiple Bevy apps (simplest)
2. Contribute the multi-world feature when they need it

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
        app.add_plugins(SimViewerPlugin::default());
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
