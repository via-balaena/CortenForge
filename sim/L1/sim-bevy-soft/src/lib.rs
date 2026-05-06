//! Soft-body Bevy visualization — deformed-tet-mesh trajectory replay
//! for sim-soft.
//!
//! Companion to `sim-soft` (L0): the solver runs headless, captures a
//! [`Trajectory`](trajectory::Trajectory) of frame-by-frame deformed
//! positions, then a Bevy app replays it by computing a frame index from
//! [`Time<Real>`](bevy::time::Time)'s elapsed wall-clock each tick and
//! writing that frame's positions + smooth normals into the soft-mesh
//! entity's `Mesh3d` asset. The solver is NEVER invoked from inside
//! Bevy's update loop — replaying captured frames decouples solver step
//! time from rendering frame time, deterministic across CI headless and
//! user windowed runs.
//!
//! Sister support crate to `sim-bevy` (rigid-body sim-core integration).
//! Consumes [`cf_bevy_common::axis::UpAxis`] for the input → Bevy frame
//! swap; the orbit camera ([`cf_bevy_common::camera::OrbitCameraPlugin`])
//! is wired by consumers, not this crate, mirroring the cf-bevy-common
//! precedent (separation of up-axis convention, camera input, and viz
//! systems).
//!
//! # Authoring shape
//!
//! Per-example `main.rs` runs the headless solver harness unconditionally
//! (asserts always run, JSON always emitted), then conditionally spins up
//! a Bevy `App` for replay via env var / CLI flag. The library crate
//! itself does NOT gate on visual mode — that's example-shape, not
//! crate-shape.
//!
//! Submodules:
//!
//! - [`mesh`] — [`mesh::build_soft_mesh`] one-shot Bevy `Mesh` build from
//!   a sim-soft positions slice + boundary-face triangulation;
//!   [`mesh::apply_soft_positions`] per-frame position + smooth-normal
//!   write into an existing Mesh asset.
//! - [`trajectory`] — [`trajectory::Trajectory`] Bevy `Component` carrying
//!   the captured frames; [`trajectory::step_replay`] system that, per
//!   soft-body entity, looks up the active frame under `Time<Real>` and
//!   writes its positions + normals via `apply_soft_positions`.
//! - [`plugin`] — [`plugin::SoftBodyVisualPlugin`] wires `step_replay`
//!   into Bevy's `Update` schedule and inits the [`UpAxis`] resource.
//! - [`prelude`] — convenience re-exports for consumer wiring.
//!
//! [`UpAxis`]: cf_bevy_common::axis::UpAxis

pub mod mesh;
pub mod plugin;
pub mod trajectory;

/// Convenience re-exports. `use sim_bevy_soft::prelude::*;` brings in the
/// load-bearing types ([`Trajectory`](trajectory::Trajectory),
/// [`ReplayEpoch`](trajectory::ReplayEpoch),
/// [`SoftBodyVisualPlugin`](plugin::SoftBodyVisualPlugin)) plus the public
/// build / apply / replay functions for consumers that prefer manual
/// wiring over the plugin.
pub mod prelude {
    pub use crate::mesh::{apply_soft_positions, build_soft_mesh};
    pub use crate::plugin::SoftBodyVisualPlugin;
    pub use crate::trajectory::{ReplayEpoch, Trajectory, step_replay};
}
