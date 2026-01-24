//! Bevy visualization for CortenForge physics simulation.
//!
//! This crate is **Layer 1** - it depends on Bevy and provides visualization
//! for the Layer 0 simulation crates (`sim-core`, `sim-types`, etc.).
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                         sim-bevy (L1)                           │
//! │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
//! │  │ SimPlugin   │  │ DebugPlugin │  │ SolariPlugin (optional) │  │
//! │  └──────┬──────┘  └──────┬──────┘  └────────────┬────────────┘  │
//! │         │                │                      │               │
//! │  ┌──────▼──────┐  ┌──────▼──────┐  ┌───────────▼───────────┐   │
//! │  │   Systems   │  │  Overlays   │  │   Ray-traced render   │   │
//! │  │  - sync     │  │  - contacts │  │   (RTX required)      │   │
//! │  │  - render   │  │  - forces   │  └───────────────────────┘   │
//! │  │  - camera   │  │  - joints   │                               │
//! │  └──────┬──────┘  └─────────────┘                               │
//! └─────────┼───────────────────────────────────────────────────────┘
//!           │ reads
//! ┌─────────▼───────────────────────────────────────────────────────┐
//! │                    sim-core World (L0)                          │
//! │  Bodies, Joints, Contacts, Collision Shapes                     │
//! └─────────────────────────────────────────────────────────────────┘
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

#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![warn(clippy::all, clippy::pedantic)]
#![allow(clippy::module_name_repetitions)]

pub mod camera;
pub mod components;
pub mod convert;
pub mod gizmos;
pub mod mesh;
pub mod plugin;
pub mod resources;
pub mod systems;

/// Prelude module for convenient imports.
pub mod prelude {
    pub use crate::camera::OrbitCamera;
    pub use crate::components::{CollisionShapeVisual, PhysicsBody, PhysicsWorldRoot, ShapeType};
    pub use crate::gizmos::DebugGizmosSet;
    pub use crate::plugin::SimViewerPlugin;
    pub use crate::resources::{BodyEntityMap, DebugColors, SimulationHandle, ViewerConfig};
}
