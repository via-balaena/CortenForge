//! The engine's error type. Like the spine, the engine returns errors
//! as values — no `unwrap`/`expect`/`panic` in library code.

use thiserror::Error;

/// Result alias for `cf-studio-engine` operations.
pub type Result<T> = std::result::Result<T, EngineError>;

/// Everything that can go wrong executing a workflow step against the
/// SDK. (Grows one variant per executor as later steps land.)
#[derive(Debug, Error)]
pub enum EngineError {
    /// A design layer named a silicone that isn't in the catalog.
    #[error("layer {index} names unknown silicone {key:?} — not in the material catalog")]
    UnknownMaterial {
        /// Zero-based layer index, innermost first.
        index: usize,
        /// The offending material key.
        key: String,
    },

    /// The assembled design failed `cf-device-types` validation.
    #[error("design is invalid: {0}")]
    InvalidDesign(String),

    /// Writing the `.design.toml` to disk failed.
    #[error("could not write design file: {0}")]
    WriteDesign(String),

    /// The chosen scan file could not be loaded as a mesh.
    #[error("could not load scan at {path}: {reason}")]
    ScanLoad {
        /// The scan path.
        path: String,
        /// Why the load failed.
        reason: String,
    },

    /// The scan loaded but has no geometry (no vertices or no faces).
    #[error("scan at {path} has no geometry — it is empty or not a surface mesh")]
    EmptyScan {
        /// The scan path.
        path: String,
    },

    /// The `.prep.toml` could not be read or parsed.
    #[error("prep file at {path} is unreadable or invalid: {reason}")]
    PrepInvalid {
        /// The prep-file path.
        path: String,
        /// Why it failed.
        reason: String,
    },

    /// The `.prep.toml` has no usable centerline (the cast needs ≥2 points).
    #[error(
        "prep file at {path} has no usable centerline (needs at least 2 points) — re-run cf-scan-prep to compute the centerline polyline"
    )]
    NoCenterline {
        /// The prep-file path.
        path: String,
    },

    /// A layer's silicone has no cure-protocol entry (mix ratio / pot
    /// life / cure time), so the pour assistant has no timing to guide it.
    #[error(
        "no pour data for silicone {key:?} — missing a cure-protocol entry (mix ratio / pot life / cure time)"
    )]
    PourDataUnavailable {
        /// The material key with no cure data.
        key: String,
    },

    /// The cast / mold-generation run failed (validation, printability
    /// gate, mass budget, or reading back the outputs).
    #[error("mold generation failed: {0}")]
    MoldGen(String),

    /// Writing the cleaned scan + `.prep.toml` from the step-2 editor failed
    /// (non-finite transform, TOML serialization, or the atomic file write).
    #[error("could not save the cleaned scan: {0}")]
    Save(String),

    /// Copying the printable files into the step-5 export folder failed
    /// (the destination couldn't be created, or a file copy errored).
    #[error("could not export the print files: {0}")]
    ExportPrint(String),

    /// Building the live "Shape your piece" preview failed (the cleaned scan
    /// or its `.prep.toml` could not be read / parsed, or the flood-fill SDF
    /// build failed). The frontend falls back to the proxy preview.
    #[error("could not build the shape preview: {0}")]
    Preview(String),
}
