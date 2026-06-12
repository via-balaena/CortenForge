//! `cf-studio-core` — the headless workflow engine for CortenForge Studio.
//!
//! CortenForge Studio is the guided "wizard" app that walks a person
//! through making a custom multi-layer silicone-cast piece, one step
//! at a time. This crate is its **headless core**: a UI-agnostic Rust
//! library that owns the workflow as a state machine plus the project
//! it operates on. Frontends — the `cf-studio` CLI and the Slint GUI —
//! are thin clients over this; they hold no business logic, so the two
//! cannot drift apart.
//!
//! The workflow has six ordered steps ([`Step`]):
//!
//! 1. **Add scan** — load a 3-D scan of the area.
//! 2. **Clean scan** — repair + seal it into a castable, watertight body.
//! 3. **Design layers** — cavity inset + the soft→firm layer stack.
//! 4. **Make molds** — generate the printable mold pieces + plugs.
//! 5. **Print** — export the files for the 3-D printer.
//! 6. **Pour** — the guided silicone-pour assistant.
//!
//! # Foundation layer (this crate, today)
//!
//! This is the *spine*: the [`Project`] model (serializable, with
//! autosave/resume) and the step [state machine](Project) that enforces
//! the workflow's invariants — steps run in order, you cannot advance
//! past an incomplete step, and editing an earlier step invalidates
//! everything downstream of it. It deliberately depends on nothing but
//! `serde` + `thiserror`; the SDK crates that actually clean scans and
//! generate molds are wired in a layer built *on top of* this one,
//! once this layer is proven rock-solid.
//!
//! # Example
//!
//! ```
//! use cf_studio_core::{Project, Step, ScanInput};
//! use std::path::PathBuf;
//!
//! # fn main() -> Result<(), cf_studio_core::StudioError> {
//! let mut project = Project::new("Wrist brace");
//! assert_eq!(project.current_step(), Step::AddScan);
//!
//! // Completing the current step's action, then advancing, is gated:
//! project.set_scan(ScanInput { source_path: PathBuf::from("wrist.stl") });
//! let now_on = project.advance()?; // allowed: the scan is present
//! assert_eq!(now_on, Step::CleanScan);
//! # Ok(())
//! # }
//! ```

pub mod error;
pub mod progress;
pub mod project;
pub mod step;

pub use error::{Result, StudioError};
pub use progress::{NoopSink, Progress, ProgressSink};
pub use project::{
    DesignDraft, LayerDraft, MoldOutputs, PROJECT_SCHEMA_VERSION, PlugDraft, PourPlan, PourRecord,
    PourStep, PrepInput, PrintExport, Project, RidgeOptions, RidgeRing, ScanInput,
};
pub use step::Step;
