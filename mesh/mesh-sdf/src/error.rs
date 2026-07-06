//! Error types for SDF operations.

use thiserror::Error;

/// Result type for SDF operations.
pub type SdfResult<T> = Result<T, SdfError>;

/// Errors that can occur during SDF computation.
#[derive(Debug, Error)]
pub enum SdfError {
    /// Mesh is empty (no vertices or faces).
    #[error("mesh is empty")]
    EmptyMesh,

    /// Grid dimensions are invalid.
    #[error("invalid grid dimensions: {0}")]
    InvalidDimensions(String),

    /// Point is outside the SDF bounds.
    #[error("point ({x}, {y}, {z}) is outside SDF bounds")]
    OutOfBounds {
        /// X coordinate.
        x: f64,
        /// Y coordinate.
        y: f64,
        /// Z coordinate.
        z: f64,
    },

    /// A face references a vertex index outside the mesh's vertex list.
    #[error("face references vertex index {index}, but the mesh has only {vertex_count} vertices")]
    FaceIndexOutOfRange {
        /// The out-of-range vertex index.
        index: u32,
        /// Number of vertices in the mesh.
        vertex_count: usize,
    },

    /// A sampled deviation was non-finite (NaN or infinite) — the input
    /// mesh vertices or the reference field are corrupt, so no meaningful
    /// fidelity score exists.
    #[error(
        "non-finite deviation sample (NaN or infinite): corrupt mesh vertices or reference field"
    )]
    NonFiniteDeviation,
}
