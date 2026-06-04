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
}
