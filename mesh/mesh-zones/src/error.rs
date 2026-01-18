//! Error types for zone operations.

use thiserror::Error;

/// Result type for zone operations.
pub type ZoneResult<T> = Result<T, ZoneError>;

/// Errors that can occur during zone operations.
#[derive(Debug, Error)]
pub enum ZoneError {
    /// Mesh is empty (no vertices or faces).
    #[error("mesh is empty")]
    EmptyMesh,

    /// Zone ID is invalid.
    #[error("invalid zone id: {0}")]
    InvalidZoneId(u32),

    /// Face index is out of bounds.
    #[error("face index {face_idx} out of bounds (mesh has {face_count} faces)")]
    FaceOutOfBounds {
        /// The invalid face index.
        face_idx: usize,
        /// Total number of faces in the mesh.
        face_count: usize,
    },

    /// Vertex index is out of bounds.
    #[error("vertex index {vertex_idx} out of bounds (mesh has {vertex_count} vertices)")]
    VertexOutOfBounds {
        /// The invalid vertex index.
        vertex_idx: usize,
        /// Total number of vertices in the mesh.
        vertex_count: usize,
    },

    /// No seed faces provided for region growing.
    #[error("no seed faces provided for region growing")]
    NoSeeds,
}
