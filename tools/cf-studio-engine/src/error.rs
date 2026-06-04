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

    /// The `.prep.toml` has no centerline — the cast pipeline needs one.
    #[error(
        "prep file at {path} has no centerline — re-run cf-scan-prep to compute the centerline polyline"
    )]
    NoCenterline {
        /// The prep-file path.
        path: String,
    },
}
