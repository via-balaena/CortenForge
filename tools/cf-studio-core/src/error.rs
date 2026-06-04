//! The crate's error type. Library code returns errors as values —
//! there are no `unwrap`/`expect`/`panic` paths in this crate.

use thiserror::Error;

use crate::step::Step;

/// Result alias for `cf-studio-core` operations.
pub type Result<T> = std::result::Result<T, StudioError>;

/// Everything that can go wrong driving a [`Project`](crate::Project).
#[derive(Debug, Error)]
pub enum StudioError {
    /// Tried to advance, or to complete a step, before its
    /// prerequisite step was finished. The wizard runs in order.
    #[error("cannot proceed from {from:?}: {reason}")]
    StepNotReady {
        /// The step that was not ready.
        from: Step,
        /// Human-readable reason, safe to surface to a user.
        reason: String,
    },

    /// Tried to go back from the first step.
    #[error("already at the first step")]
    AlreadyAtStart,

    /// Tried to advance past the final step.
    #[error("already at the final step")]
    AlreadyAtEnd,

    /// A loaded project violated the workflow invariants (artifacts
    /// must form a contiguous prefix; the current step must be
    /// reachable). The string names the specific violation.
    #[error("project file is invalid: {0}")]
    InvalidProject(String),

    /// A loaded project declared a schema version newer than this
    /// build understands. Refusing is safer than misreading it.
    #[error("project schema version {found} is newer than this build supports ({supported})")]
    UnsupportedSchema {
        /// The version found on disk.
        found: u32,
        /// The newest version this build can read.
        supported: u32,
    },

    /// Serializing the project to its on-disk form failed.
    #[error("could not serialize project: {0}")]
    Serialize(String),

    /// Parsing a project from its on-disk form failed.
    #[error("could not parse project: {0}")]
    Deserialize(String),

    /// A filesystem operation failed. `path` is the file involved.
    #[error("I/O error at {path}: {source}")]
    Io {
        /// The path the operation targeted.
        path: String,
        /// The underlying OS error.
        #[source]
        source: std::io::Error,
    },
}
