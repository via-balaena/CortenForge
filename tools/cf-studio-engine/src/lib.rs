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
pub mod error;
pub mod prep;
pub mod scan;

pub use design::{design_toml_from_draft, save_design_from_draft};
pub use error::{EngineError, Result};
pub use prep::accept_prep;
pub use scan::{LoadedScan, load_scan};

// Re-export the spine so a frontend needs only this one crate to reach
// the workflow types (`Project`, `Step`, `DesignDraft`, …): the engine
// is the single front door.
pub use cf_studio_core;
