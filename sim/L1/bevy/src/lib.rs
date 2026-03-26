//! Bevy visualization for CortenForge physics simulation.
//!
//! This crate is **Layer 1** - it depends on Bevy and provides visualization
//! for the Layer 0 simulation crates (`sim-core`, `sim-types`, etc.).
//!
//! # Architecture
//!
//! ```text
//! ┌──────────────────────────────────────────────────────────┐
//! │                      sim-bevy (L1)                        │
//! │  ┌─────────────────┐  ┌────────────────┐                 │
//! │  │ SimViewerPlugin │  │ ModelDataPlugin │                 │
//! │  └───────┬─────────┘  └───────┬────────┘                 │
//! │          │                    │                           │
//! │  ┌───────▼────────┐  ┌───────▼────────┐  ┌───────────┐  │
//! │  │    Systems      │  │    Gizmos      │  │  Camera   │  │
//! │  │ - sync bodies   │  │ - contacts     │  │ - orbit   │  │
//! │  │ - sync geoms    │  │ - muscles      │  └───────────┘  │
//! │  │ - sync sites    │  │ - tendons      │                  │
//! │  │ - step physics  │  │ - sensors      │                  │
//! │  └───────┬─────────┘  └────────────────┘                  │
//! └──────────┼────────────────────────────────────────────────┘
//!            │ reads
//! ┌──────────▼────────────────────────────────────────────────┐
//! │                  sim-core Model/Data (L0)                  │
//! │  Bodies, Joints, Contacts, Collision Shapes                │
//! └───────────────────────────────────────────────────────────┘
//! ```
//!
//! # Design Philosophy
//!
//! - The viewer is a *window into* sim-core, not a replacement
//! - sim-core remains the source of truth for physics state
//! - All rendering is derived from sim-core data structures
//! - Zero physics logic in sim-bevy
//! - **Bevy-upgrade-resilient:** Minimize API surface, isolate Bevy types
//!
//! # Example
//!
//! ```no_run,ignore
//! use bevy::prelude::*;
//! use sim_bevy::prelude::*;
//!
//! fn main() {
//!     App::new()
//!         .add_plugins(DefaultPlugins)
//!         .add_plugins(SimViewerPlugin::default())
//!         .run();
//! }
//! ```
//!
//! # Performance Tuning
//!
//! ## Collision Filtering
//!
//! Use MuJoCo-compatible collision filtering via `contype`/`conaffinity`
//! bitmasks on geoms in the Model (set via MJCF attributes).
//!
//! ## Mesh Performance
//!
//! For convex mesh collision, keep vertex counts low:
//! - Under 32 vertices: optimal for convex-convex collision
//! - Under 64 vertices: acceptable performance

#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![warn(clippy::all, clippy::pedantic)]
#![allow(clippy::module_name_repetitions)]

pub mod camera;
pub mod components;
pub mod convert;
pub mod examples;
pub mod gizmos;
pub mod materials;
pub mod mesh;
pub mod model_data;
pub mod plugin;
pub mod resources;
pub mod scene;
pub mod sensor_viz;
pub mod systems;

/// Prelude module for convenient imports.
pub mod prelude {
    pub use crate::camera::OrbitCamera;
    pub use crate::components::{CollisionShapeVisual, ShapeType, VisGroup};
    pub use crate::convert::{
        bevy_to_physics, physics_pos, transform_from_physics, transform_from_physics_pose,
    };
    pub use crate::examples::{
        DiagTimer, ValidationHarness, spawn_example_camera, validation_system,
    };
    pub use crate::gizmos::{DebugGizmosSet, TrailGizmo, draw_trails, sample_trails};
    // Model/Data architecture (MuJoCo-style) - PREFERRED API
    pub use crate::materials::{
        MetalPreset, override_geom_material_by_index, override_geom_material_by_name,
        override_geom_materials_by_name, override_geom_materials_by_name_with,
    };
    pub use crate::mesh::{
        SpringCoilParams, spawn_design_mesh, spring_coil, triangle_mesh_from_indexed,
    };
    pub use crate::model_data::{
        GeomMaterialOverride, ModelBodyIndex, ModelDataPlugin, ModelDataRoot, ModelDataSet,
        ModelGeomIndex, ModelSiteIndex, PhysicsAccumulator, PhysicsData, PhysicsModel,
        spawn_model_geoms, spawn_model_geoms_with, step_model_data, step_physics_realtime,
        sync_geom_transforms, sync_model_data_to_bevy,
    };
    pub use crate::plugin::SimViewerPlugin;
    pub use crate::resources::{
        BodyEntityMap, CachedContacts, DebugColors, MuscleVisualData, MuscleVisualization,
        TendonVisualData, TendonVisualization, ViewerConfig,
    };
    pub use crate::scene::ExampleScene;
    pub use crate::sensor_viz::{
        SensorGizmo, SensorVisualization, SensorVizEntry, draw_sensor_gizmos, sensor_type_to_gizmo,
        update_sensor_visualization,
    };
    pub use crate::systems::update_cached_contacts;

    // Re-export Model/Data types from sim-core for convenience
    pub use sim_core::{Data, Model};
    // Re-export MJCF loader for convenient Model creation
    pub use sim_mjcf::load_model;
    // Re-export URDF loader
    pub use sim_urdf::load_urdf_model;
}
