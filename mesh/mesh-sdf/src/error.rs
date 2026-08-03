//! Error types for SDF operations.

use thiserror::Error;

/// Result type for SDF operations.
pub type SdfResult<T> = Result<T, SdfError>;

/// Errors that can occur during SDF computation.
#[derive(Debug, Error)]
pub enum SdfError {
    /// Mesh has no faces (nothing to build a surface or BVH from).
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

    /// No internal scale satisfies both ends of the normalisation rule, so this mesh
    /// cannot be given a trustworthy sign.
    ///
    /// The rule is two-sided: the smallest triangle must be lifted clear of parry's f32
    /// area floor (below it the pseudo-normal is zeroed and the oracle reports "inside"
    /// at any distance), while the largest coordinate must stay where f32 cross products
    /// and squared norms remain finite. A mesh whose triangle areas span more dynamic
    /// range than f32 can hold has no scale that satisfies both.
    ///
    /// **This is a real answer, not a failure to cope.** The alternative — clamping to
    /// the nearest feasible scale — hands back a signed oracle for a surface that cannot
    /// be signed, which is the defect this whole arc exists to remove.
    #[error(
        "mesh cannot be internally normalised (smallest triangle area {min_area:.3e}, \
         coordinate extent {extent:.3e}): {reason}"
    )]
    UnscalableMesh {
        /// Smallest triangle area in the mesh, in the caller's units.
        min_area: f64,
        /// Largest absolute vertex coordinate, in the caller's units.
        extent: f64,
        /// Which end of the two-sided rule could not be satisfied.
        reason: &'static str,
    },
}
