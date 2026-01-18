//! Error types for assembly operations.

use std::path::PathBuf;
use thiserror::Error;

/// Result type for assembly operations.
pub type AssemblyResult<T> = Result<T, AssemblyError>;

/// Errors that can occur during assembly operations.
#[derive(Debug, Error)]
pub enum AssemblyError {
    /// Part with the given ID already exists.
    #[error("Part '{id}' already exists in assembly")]
    DuplicatePart {
        /// The duplicate part ID.
        id: String,
    },

    /// Part with the given ID was not found.
    #[error("Part '{id}' not found in assembly")]
    PartNotFound {
        /// The missing part ID.
        id: String,
    },

    /// Parent part does not exist.
    #[error("Parent part '{parent_id}' does not exist for part '{child_id}'")]
    ParentNotFound {
        /// The child part ID.
        child_id: String,
        /// The missing parent ID.
        parent_id: String,
    },

    /// Assembly is empty.
    #[error("Cannot perform operation on empty assembly")]
    EmptyAssembly,

    /// Circular parent reference detected.
    #[error("Circular parent reference detected for part '{id}'")]
    CircularReference {
        /// The part ID with circular reference.
        id: String,
    },

    /// I/O error during file operations.
    #[error("I/O error at '{path}': {source}")]
    Io {
        /// The path where the error occurred.
        path: PathBuf,
        /// The underlying I/O error.
        #[source]
        source: std::io::Error,
    },

    /// Zip error during 3MF export.
    #[cfg(feature = "export-3mf")]
    #[error("Zip error: {message}")]
    Zip {
        /// Error message.
        message: String,
    },

    /// Mesh I/O error.
    #[error("Mesh I/O error: {0}")]
    MeshIo(#[from] mesh_io::IoError),
}
