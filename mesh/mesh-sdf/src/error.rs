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

    /// The mesh carries no geometry an internal scale could be derived from — every
    /// triangle is exactly degenerate, or a vertex is non-finite.
    ///
    /// ⚠ **Narrower than it first looks, deliberately.** An earlier version of this
    /// variant also fired when the scale rule could not reach its full area margin without
    /// passing the f32 coordinate cap. That turned out to reject perfectly signable meshes
    /// — `mesh-lattice`'s marching-cubes output has slivers around 1e-30 that still clear
    /// the floor by twenty binades once lifted — so the rule now clamps and only this
    /// genuinely-empty case remains fatal. Reporting a *partial* lift is the job of a
    /// construction-time guard that can measure the consequence, not of the scale rule.
    #[error(
        "mesh cannot be internally normalised (smallest positive triangle area \
         {min_area:.3e}, coordinate extent {extent:.3e}): {reason}"
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
