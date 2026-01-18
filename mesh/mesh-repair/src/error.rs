//! Error types for mesh repair operations.

use thiserror::Error;

/// Result type for repair operations.
pub type RepairResult<T> = Result<T, RepairError>;

/// Errors that can occur during mesh repair.
#[derive(Debug, Error)]
pub enum RepairError {
    /// Mesh is empty (no vertices or faces).
    #[error("mesh is empty")]
    EmptyMesh,

    /// Mesh has invalid indices.
    #[error("invalid vertex index {index} (mesh has {vertex_count} vertices)")]
    InvalidIndex {
        /// The invalid index.
        index: u32,
        /// Total number of vertices in the mesh.
        vertex_count: usize,
    },

    /// Not enough points for the operation.
    #[error("insufficient points: need at least {required}, got {actual}")]
    InsufficientPoints {
        /// Minimum number of points required.
        required: usize,
        /// Actual number of points provided.
        actual: usize,
    },

    /// Mesh is not manifold.
    #[error("mesh is not manifold: {details}")]
    NonManifold {
        /// Description of the non-manifold condition.
        details: String,
    },

    /// Winding order repair failed.
    #[error("failed to fix winding order: {reason}")]
    WindingRepairFailed {
        /// Reason for failure.
        reason: String,
    },

    /// Hole filling failed.
    #[error("failed to fill holes: {reason}")]
    HoleFillFailed {
        /// Reason for failure.
        reason: String,
    },
}
