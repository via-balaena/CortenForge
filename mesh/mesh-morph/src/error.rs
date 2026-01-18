//! Error types for mesh morphing operations.

use thiserror::Error;

/// Errors that can occur during mesh morphing.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum MorphError {
    /// The mesh has no vertices.
    #[error("mesh has no vertices")]
    EmptyMesh,

    /// No constraints were provided.
    #[error("no constraints provided for morphing")]
    NoConstraints,

    /// The RBF interpolation matrix is singular or degenerate.
    #[error("RBF system is degenerate: {0}")]
    DegenerateSystem(String),

    /// Invalid FFD lattice dimensions.
    #[error("invalid FFD lattice dimensions: {0}")]
    InvalidLatticeDimensions(String),

    /// A constraint references an invalid vertex index.
    #[error("constraint references invalid vertex index {index} (mesh has {vertex_count} vertices)")]
    InvalidVertexIndex {
        /// The invalid index.
        index: usize,
        /// The number of vertices in the mesh.
        vertex_count: usize,
    },

    /// Region mask contains an invalid face index.
    #[error("region mask contains invalid face index {index} (mesh has {face_count} faces)")]
    InvalidFaceIndex {
        /// The invalid index.
        index: u32,
        /// The number of faces in the mesh.
        face_count: usize,
    },
}

/// Result type for morphing operations.
pub type MorphResult<T> = Result<T, MorphError>;
