//! `cf-studio-engine` — the SDK-boundary orchestrator for CortenForge Studio.
//!
//! This crate sits on top of the headless [`cf_studio_core`] spine and
//! executes the workflow's steps by calling the SDK crates. The spine
//! stays pure (serde + thiserror only, so it builds and tests fast);
//! **this** is the layer where the SDK dependencies live, and where the
//! conversions between the spine's owned artifacts and the SDK's types
//! happen.
//!
//! It is still a *library* — frontends (the `cf-studio` CLI and the
//! Slint GUI) call into it; it holds no UI.
//!
//! Today it implements the design-build boundary ([`design`]) — turning
//! a [`cf_studio_core::DesignDraft`] into a validated `.design.toml`
//! via the now-headless `cf-device-types`. Scan loading, cleaning, mold
//! generation, and the structured pour plan land here as the workflow
//! is built up, one proven layer at a time.

pub mod design;
pub mod edit;
pub mod error;
pub mod mold;
pub mod pour;
pub mod prep;
pub mod preview;
pub mod print;
pub mod scan;

pub use design::{
    design_toml_from_draft, draft_from_design_toml, save_design_from_draft, silicone_catalog,
};
pub use edit::{EditSession, run_simplify};
// The reconstruct-shape choice an `EditSession::apply_reconstruct` caller
// (the GUI) needs to name — re-exported through the engine front door.
pub use cf_scan_prep_core::ReconstructShape;
pub use error::{EngineError, Result};
pub use mold::{generate_molds, generate_molds_for_design};
// Re-export the config type `generate_molds` takes + the part-selection
// types `generate_molds_for_design` takes, so a frontend can build/parse
// them without a separate cf-cast-cli dependency.
pub use cf_cast_cli::{CastConfig, CastMode, PartId, PartSelection, PieceSide};
pub use pour::{LayerPour, build_pour_plan};
pub use prep::accept_prep;
pub use preview::{PreviewShowing, texture_preview_mesh};
pub use print::{PrintExportReport, export_print_package};
pub use scan::{LoadedScan, load_scan};

// Re-export the spine so a frontend needs only this one crate to reach
// the workflow types (`Project`, `Step`, `DesignDraft`, …): the engine
// is the single front door.
pub use cf_studio_core;
