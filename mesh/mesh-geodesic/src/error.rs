//! Error types for geodesic distance computation.

use thiserror::Error;

/// Result type for geodesic operations.
pub type GeodesicResult<T> = Result<T, GeodesicError>;

/// Errors that can occur during geodesic computation.
#[derive(Debug, Error)]
pub enum GeodesicError {
    /// Invalid vertex index.
    #[error("invalid vertex index: {index} (mesh has {vertex_count} vertices)")]
    InvalidVertex {
        /// The invalid index.
        index: usize,
        /// Total number of vertices in the mesh.
        vertex_count: usize,
    },

    /// Empty mesh (no vertices or faces).
    #[error("mesh is empty")]
    EmptyMesh,

    /// Disconnected mesh (some vertices unreachable).
    #[error("mesh is disconnected: {unreachable_count} vertices unreachable from sources")]
    DisconnectedMesh {
        /// Number of unreachable vertices.
        unreachable_count: usize,
    },
}
