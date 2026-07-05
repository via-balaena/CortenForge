//! # CortenForge SDK
//!
//! The single, stable public surface of the CortenForge SDK. Applications
//! depend on this one crate and reach the whole SDK through it — so the SDK's
//! internal crate structure can evolve behind one contract. See `MISSION.md`
//! and the app-vs-SDK boundary.
//!
//! This is a **facade**: each item below re-exports a constituent SDK crate (or
//! a domain umbrella) verbatim. It adds no logic of its own, and it is
//! deliberately **headless** — no Bevy, GUI, or GPU crate is pulled in, so
//! every app compiling against it stays headless too.
//!
//! ```
//! // The two domain umbrellas give you the whole toolkit through one dep:
//! use cortenforge::sim;   // rigid + soft physics, coupling, RL/opt
//! use cortenforge::mesh;  // load/repair/measure/print meshes
//! ```
//!
//! ## Capability map — start here
//!
//! ### Simulation & co-design — [`sim`]
//! The flagship headless spine, one umbrella (`cortenforge::sim`):
//! - [`sim::core`] — rigid-body dynamics (MuJoCo-compatible).
//! - [`sim::soft`] — backward-Euler soft-body FEM with IFT gradients.
//! - [`sim::coupling`] — the L1 keystone: differentiable soft↔rigid coupling.
//! - [`sim::mjcf`] / [`sim::urdf`] — model I/O.
//! - [`sim::ml_chassis`] / [`sim::rl`] / [`sim::opt`] — the learning + optimization stack.
//! - [`sim::thermostat`] / [`sim::therm_env`] — thermodynamic-computing track.
//!
//! ### Mesh processing — [`mesh`]
//! The full mesh toolkit, one umbrella (`cortenforge::mesh`):
//! - [`mesh::types`] — core mesh types (`IndexedMesh`, `Triangle`, `Aabb`).
//! - [`mesh::io`] — mesh I/O (STL/OBJ/PLY/3MF; `threemf` enabled).
//! - [`mesh::repair`] / [`mesh::sdf`] / [`mesh::offset`] / [`mesh::shell`] — core operations.
//! - [`mesh::measure`] / [`mesh::printability`] / [`mesh::lattice`] — analysis + 3D-printing.
//!
//! ### Foundation kernels
//! - [`cf_geometry`] — geometry primitives (`Aabb`, `Shape`, `Sdf`).
//! - [`cf_spatial`] — spatial-math primitives.
//!
//! ### Design & fabrication
//! - [`cf_design`] — implicit-surface design kernel.
//! - [`cf_cap_planes`] — cap-plane parsing.
//! - [`cf_device_types`] — shared device-design domain types.
//! - [`cf_scan_prep_core`] — headless scan-prep (repair / cap / centerline / trim).
//! - [`cf_cast`] — multi-material mold generation.
//! - [`cf_cast_cli`] — the scan→cast bridge (`run_with_config`, `CastConfig`).

// =============================================================================
// Simulation & co-design
// =============================================================================

/// Headless simulation & co-design umbrella — `sim::core`, `sim::soft`,
/// `sim::coupling`, `sim::rl`, and the rest of the headless spine.
///
/// Requires the `sim` feature (on by default).
#[cfg(feature = "sim")]
pub use sim;

// =============================================================================
// Mesh processing
// =============================================================================

/// Mesh-processing umbrella — `mesh::io`, `mesh::repair`, `mesh::sdf`, …
/// (`mesh-io/threemf` is enabled, so 3MF scans load through `mesh::io`).
///
/// Requires the `mesh` feature (on by default).
#[cfg(feature = "mesh")]
pub use mesh;

// =============================================================================
// Foundation kernels (shared by `sim` + `fabrication`)
// =============================================================================

/// Geometry primitives (`Aabb`, `Shape`, `Sdf`).
///
/// Available with either the `sim` or `fabrication` feature (both on by default).
#[cfg(any(feature = "sim", feature = "fabrication"))]
pub use cf_geometry;

/// Spatial-math primitives.
///
/// Available with either the `sim` or `fabrication` feature (both on by default).
#[cfg(any(feature = "sim", feature = "fabrication"))]
pub use cf_spatial;

// =============================================================================
// Design & fabrication (requires the `fabrication` feature, on by default)
// =============================================================================

/// Implicit-surface design kernel.
#[cfg(feature = "fabrication")]
pub use cf_design;

/// Cap-plane parsing.
#[cfg(feature = "fabrication")]
pub use cf_cap_planes;

/// Shared device-design domain types.
#[cfg(feature = "fabrication")]
pub use cf_device_types;

/// Headless scan-prep (repair / cap / centerline / trim).
#[cfg(feature = "fabrication")]
pub use cf_scan_prep_core;

/// Multi-material mold generation.
#[cfg(feature = "fabrication")]
pub use cf_cast;

/// The scan→cast bridge (`run_with_config`, `CastConfig`).
#[cfg(feature = "fabrication")]
pub use cf_cast_cli;

#[cfg(test)]
mod tests {
    // The facade is pure re-exports with no executable lines of its own; this
    // smoke test gives coverage something to measure and guards the headline
    // public paths (one per enabled domain) against silent breakage. Each
    // assertion is gated on the feature that re-exports its crate, so the test
    // compiles under any feature combination (including `--no-default-features`).
    #[test]
    fn public_paths_are_reachable() {
        #[cfg(feature = "sim")]
        assert!(std::mem::size_of::<crate::sim::types::SimulationConfig>() < usize::MAX);
        #[cfg(feature = "mesh")]
        assert!(std::mem::size_of::<crate::mesh::types::IndexedMesh>() < usize::MAX);
        #[cfg(any(feature = "sim", feature = "fabrication"))]
        assert!(std::mem::size_of::<crate::cf_geometry::Aabb>() < usize::MAX);
        #[cfg(feature = "fabrication")]
        assert!(std::mem::size_of::<crate::cf_design::InfillKind>() < usize::MAX);
    }
}
