//! Shared Bevy-0.18 helpers for workspace consumers.
//!
//! Houses the axis-aware coordinate convention ([`axis::UpAxis`]) and the
//! mouse-driven orbit camera ([`camera::OrbitCamera`]) that three consumers
//! need: `cf-viewer` (PLY visual review), `sim-bevy-soft` (deforming-tet-mesh
//! visualization), and `sim-bevy` (rigid-body scenes). Factoring keeps the
//! shared abstractions in one place so consumers don't duplicate or drift.
//!
//! Submodules:
//!
//! - [`axis`] — [`axis::UpAxis`] enum + [`axis::UpAxis::to_bevy_point`] /
//!   [`axis::UpAxis::to_bevy_normal`] coordinate-swap methods that project
//!   an input-frame `(x, y, z)` to Bevy's Y-up `[f32; 3]`.
//! - [`camera`] — [`camera::OrbitCamera`] component +
//!   [`camera::OrbitCameraPlugin`] that wires `orbit_camera_input` →
//!   `update_orbit_camera` with explicit `.after()` ordering.
//! - [`mesh`] — [`mesh::triangle_mesh_with_crease_splitting`]: shared
//!   `IndexedMesh` → Bevy `Mesh` conversion with cos(30°) crease-angle
//!   vertex splitting, optional per-vertex colors, and `UpAxis`-aware
//!   coordinate swap + winding flip.
//! - [`prelude`] — convenience re-exports for consumer wiring.

pub mod axis;
pub mod camera;
pub mod mesh;

/// Convenience re-exports. `use cf_bevy_common::prelude::*;` brings in
/// the load-bearing types (`UpAxis`, `OrbitCamera`, `OrbitCameraPlugin`)
/// plus the public system functions for consumers that prefer manual
/// wiring over the plugin.
pub mod prelude {
    pub use crate::axis::UpAxis;
    pub use crate::camera::{
        OrbitCamera, OrbitCameraPlugin, orbit_camera_input, update_orbit_camera,
    };
}
