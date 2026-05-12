//! cf-cast — physical-cast tooling for layered silicone devices.
//!
//! Composes [`cf_design::Solid`] (typed SDF kernel), [`mesh_offset`]
//! (marching cubes + scalar grid), [`mesh_io::save_stl`] (STL writer),
//! and [`mesh_printability::validate_for_printing`] (F4 gate) into a
//! per-layer mold/plug exporter for the layered-silicone-device v1.0
//! cast workflow.
//!
//! # Stage 1 scope
//!
//! Single layer only: one mold cup STL + one plug STL written to a
//! caller-supplied output directory, with the F4 printability gate
//! firing at write time. Multi-layer composition, pour-volume budgeting
//! (F2), and the procedure-spec generator (F3) arrive in Stage 2.
//!
//! # Unit boundary
//!
//! [`cf_design::Solid`] works in **meters**. STL files and
//! [`mesh_printability::PrinterConfig::fdm_default`]'s build-volume
//! check work in **millimeters**. The [`CastSpec::export_molds`]
//! pipeline performs the `×1000` scale at the marching-cubes →
//! validate/save boundary exactly once, so caller-facing geometry stays
//! in meters and printer/STL-facing geometry stays in mm.
//!
//! # Demolding axis
//!
//! Stage 1 hardcodes `+z` as the demolding axis. The exporter
//! internally subtracts a clip cuboid covering the half-space above
//! the body's `z_max`, producing a one-piece cup with an opening at
//! the top. Stage 2 will expose the axis as a parameter for arbitrary
//! orientations (and Stage 4+ for multi-piece molds if iter-1 surfaces
//! demolding-undercut issues).
//!
//! See the [casting roadmap][rmp] for the full Track F trajectory.
//!
//! [rmp]: ../../../docs/CASTING_ROADMAP.md

mod error;
mod mesher;
mod spec;

pub use error::CastError;
pub use spec::{CastSpec, MeshSummary, MoldExportReport};
