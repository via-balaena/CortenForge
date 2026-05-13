//! cf-cast — physical-cast tooling for layered silicone devices.
//!
//! Composes [`cf_design::Solid`] (typed SDF kernel), [`mesh_offset`]
//! (marching cubes + scalar grid), [`mesh_io::save_stl`] (STL writer),
//! and [`mesh_printability::validate_for_printing`] (F4 gate) into
//! per-layer mold + plug exporters for the layered-silicone-device
//! cast workflow.
//!
//! # Two export pipelines
//!
//! cf-cast ships two pipelines that share the [`CastSpec`] data
//! carrier:
//!
//! - [`CastSpec::export_molds`] — **v1 single-piece cup** (Stage 2).
//!   Each layer's mold is one `bounding_region ∖ layer_body ∖
//!   clip_above(layer_body)` cup; the shared [`CastSpec::plug`]
//!   shapes the innermost layer's cavity. `+z` is the hardcoded
//!   demolding axis. Artifact count: **N mold cup STLs + 1 plug
//!   STL = N + 1**.
//! - [`CastSpec::export_molds_v2`] — **v2 curve-following multi-
//!   piece** + **v2.1 detachable-shell**. Each layer's mold cup
//!   is split into 2 pieces along a curve-following [`Ribbon`]
//!   surface (per `docs/CURVE_FOLLOWING_DESIGN.md`), with optional
//!   inter-piece registration pins ([`PinSpec`]), side-mounted
//!   pour gate + apex air vent ([`PourGateSpec`]), and per-piece
//!   plug-anchor pin sockets ([`PlugPinSpec`]). Each layer is
//!   cast independently against its own plug — layer 0's plug
//!   derives from [`CastSpec::plug`], layer N>0's derives from
//!   `layers[N-1].body`. Artifact count: **2L mold piece STLs +
//!   L plug STLs = 3L**.
//!
//! # Cast layer convention
//!
//! [`CastSpec::layers`] carries N [`CastLayer`]s in
//! **innermost-first** cast order. Each `body` field is the
//! **cumulative SOLID outer-surface positive** of the cured
//! silicone after that pour: `layers[0].body` is a solid pipe (or
//! arbitrary [`cf_design::Solid`]) at the innermost layer's outer
//! radius; `layers[N].body` for `N > 0` is the cumulative outer
//! solid of all inner-plus-this-pour shells fused. Pour-volume
//! integration computes each silicone shell as `layer.body ∖
//! previous.body` (or `∖ spec.plug` for layer 0) inside
//! [`CastSpec::compute_pour_volumes`], so callers pass cumulative
//! solid bodies (NOT annular shells — annular bodies break the
//! `bounding ∖ body` mold-piece composition by re-introducing the
//! plug-cavity region as "inside the mold piece").
//!
//! # Unit boundary
//!
//! [`cf_design::Solid`] works in **meters**. STL files and
//! [`mesh_printability::PrinterConfig::fdm_default`]'s build-volume
//! check work in **millimeters**. Both pipelines perform the
//! `×1000` scale at the marching-cubes → validate/save boundary
//! exactly once per mesh, so caller-facing geometry stays in meters
//! and printer/STL-facing geometry stays in mm.
//!
//! # Workshop orientation convention (v2.1)
//!
//! [`CastSpec::export_molds_v2`] places the apex vent on the
//! polyline's argmax-z vertex with axis `+Z`; v2.1 callers orient
//! the assembled mold **`+Z` up** during pour + cure so trapped
//! air rises into the vent. The side-mounted pour gate exits on
//! the `±Y` face (binormal direction at the centerline midpoint).
//! v1's [`CastSpec::export_molds`] uses `+z` as the demolding
//! axis (clip cuboid above each layer body's `z_max`).
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
