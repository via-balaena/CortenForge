//! cf-cast — physical-cast tooling for layered silicone devices.
//!
//! Composes [`cf_design::Solid`] (typed SDF kernel), [`mesh_offset`]
//! (marching cubes + scalar grid), [`mesh_io::save_stl`] (STL writer),
//! and [`mesh_printability::validate_for_printing`] (F4 gate) into a
//! per-layer mold/plug exporter for the layered-silicone-device v1.0
//! cast workflow.
//!
//! # Stage 2 scope
//!
//! Multi-layer cast spec: [`CastSpec::layers`] carries N
//! [`CastLayer`]s in **innermost-first** cast order. `layers[0]` is
//! cast first into the bounding region with the shared printed
//! [`CastSpec::plug`] shaping its inner cavity; subsequent layers
//! pour around the previously cured layer (no additional printed
//! plug), so the artifact count is **N mold cup STLs + 1 plug STL**
//! per the casting roadmap.
//!
//! Per-layer geometry is the **cumulative** outer-surface positive
//! after that pour cures: `layers[0].body` is the innermost shell
//! only; `layers[N].body` for `N > 0` is the cured-inner-plus-this-
//! pour fused solid. Each mold cup is
//! `bounding_region ∖ layers[i].body ∖ clip_above(layers[i].body)`.
//!
//! Stage 2's F1 leaf shipped the multi-layer geometry pipeline; F2
//! added per-layer pour-volume integration and the
//! [`CastSpec::mass_budget_kg`] gate. The F3 leaf (this commit) adds
//! the [`CastSpec::write_procedure`] workshop-Markdown generator
//! backed by the cf-cast-local [`mod@cure`] table.
//!
//! # Unit boundary
//!
//! [`cf_design::Solid`] works in **meters**. STL files and
//! [`mesh_printability::PrinterConfig::fdm_default`]'s build-volume
//! check work in **millimeters**. The [`CastSpec::export_molds`]
//! pipeline performs the `×1000` scale at the marching-cubes →
//! validate/save boundary exactly once per mesh, so caller-facing
//! geometry stays in meters and printer/STL-facing geometry stays in
//! mm.
//!
//! # Demolding axis
//!
//! Stage 2 hardcodes `+z` as the demolding axis (unchanged from
//! Stage 1). The exporter internally subtracts a clip cuboid covering
//! the half-space above each layer body's `z_max`, producing a
//! one-piece cup with an opening at the top. Stage 3+ may expose the
//! axis as a parameter (and multi-piece molds if iter-1 surfaces
//! demolding-undercut issues).
//!
//! See the [casting roadmap][rmp] for the full Track F trajectory.
//!
//! [rmp]: ../../../docs/CASTING_ROADMAP.md

pub mod cure;
mod error;
mod material;
mod mesher;
pub mod piece;
pub mod plug;
pub mod pour;
mod pour_volume;
mod procedure;
pub mod registration;
mod ribbon;
mod spec;

pub use cure::CureProtocol;
pub use error::{CastError, CastTarget};
pub use material::MoldingMaterial;
pub use piece::{RIBBON_PIECE_OVERLAP_M, compose_piece_solid};
pub use plug::{
    PlugPinKind, PlugPinSpec, add_plug_pins, build_plug_pin_solid, build_plug_socket_solid,
};
pub use pour::{PourGateKind, PourGateSpec, build_pour_gate_solid};
pub use pour_volume::{DEFAULT_MASS_BUDGET_KG, PourVolume};
pub use procedure::{generate_procedure_markdown, generate_procedure_markdown_v2};
pub use registration::{PinSpec, RegistrationKind, build_registration_solid};
pub use ribbon::{PieceSide, Ribbon, RibbonError, RibbonSegment, SplitNormal};
pub use spec::{
    CastLayer, CastSpec, MeshSummary, MoldArtifact, MoldExportReport, PieceArtifact, PlugArtifact,
    V2LayerReport, V2MoldExportReport,
};
