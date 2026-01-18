//! Error types for mesh region operations.

use thiserror::Error;

/// Result type for region operations.
pub type RegionResult<T> = Result<T, RegionError>;

/// Errors that can occur during region operations.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum RegionError {
    /// A region with this name already exists.
    #[error("region '{name}' already exists")]
    DuplicateRegion {
        /// The duplicate region name.
        name: String,
    },

    /// The specified region was not found.
    #[error("region '{name}' not found")]
    RegionNotFound {
        /// The missing region name.
        name: String,
    },

    /// An invalid face index was provided.
    #[error("invalid face index {index} (mesh has {face_count} faces)")]
    InvalidFaceIndex {
        /// The invalid index.
        index: u32,
        /// Total number of faces in the mesh.
        face_count: usize,
    },

    /// An invalid vertex index was provided.
    #[error("invalid vertex index {index} (mesh has {vertex_count} vertices)")]
    InvalidVertexIndex {
        /// The invalid index.
        index: u32,
        /// Total number of vertices in the mesh.
        vertex_count: usize,
    },

    /// The mesh is empty.
    #[error("mesh is empty")]
    EmptyMesh,
}
