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
pub mod gizmos;
pub mod mesh;
pub mod model_data;
pub mod plugin;
pub mod resources;
pub mod systems;

/// Prelude module for convenient imports.
pub mod prelude {
    pub use crate::camera::OrbitCamera;
    pub use crate::components::{CollisionShapeVisual, ShapeType};
    pub use crate::gizmos::DebugGizmosSet;
    // Model/Data architecture (MuJoCo-style) - PREFERRED API
    pub use crate::model_data::{
        step_model_data, sync_model_data_to_bevy, ModelBodyIndex, ModelDataPlugin, ModelDataRoot,
        ModelDataSet, ModelGeomIndex, ModelSiteIndex, PhysicsData, PhysicsModel,
    };
    pub use crate::plugin::SimViewerPlugin;
    pub use crate::resources::{
        BodyEntityMap, CachedContacts, DebugColors, MuscleVisualData, MuscleVisualization,
        SensorVisualData, SensorVisualType, SensorVisualization, TendonVisualData,
        TendonVisualization, ViewerConfig,
    };
    pub use crate::systems::update_cached_contacts;

    // Re-export Model/Data types from sim-core for convenience
    pub use sim_core::{Data, Model};
    // Re-export MJCF loader for convenient Model creation
    pub use sim_mjcf::load_model;
    // Re-export URDF loader
    pub use sim_urdf::load_urdf_model;
}
