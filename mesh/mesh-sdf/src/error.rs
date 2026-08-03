//! Error types for SDF operations.

use thiserror::Error;

/// Result type for SDF operations.
pub type SdfResult<T> = Result<T, SdfError>;

/// Errors that can occur during SDF computation.
///
/// # ★ What may be an error here, and what may not
///
/// **Definitional impossibility may be fatal. A quality judgment may not.**
///
/// Ask of every variant added below: *am I saying "I cannot proceed," or am I saying "I
/// think this input is bad"?* The first is an error. The second is a **report** — it
/// belongs in a construction-time summary the caller can act on, and only something that
/// measures the actual consequence may refuse on its strength.
///
/// The rule exists because a constructor sees **proxies**. `choose_scale` sees the
/// smallest triangle area; it cannot see whether any vertex ends up with all of its
/// incident triangles skipped, which is what actually zeroes a pseudo-normal. Giving a
/// proxy the authority to fail the build means a merely-suspicious mesh gets rejected,
/// and the caller has no recourse because the thing that refused could not measure what
/// it was refusing over.
///
/// This is not hypothetical. [`SdfError::UnscalableMesh`] was first written to fire when
/// the scale rule could not reach its full area margin — a quality judgment wearing an
/// error's clothes. It rejected `mesh-lattice`'s marching-cubes output, whose slivers
/// clear the floor by twenty binades once lifted, and took six gates red. Narrowed to the
/// definitional case; the shortfall is now clamped and reported.
///
/// `mesh-sdf` already had the pattern right elsewhere, which is what makes the slip
/// instructive: `inside_components > 1` is a genuine quality signal — a flood leak, or a
/// body that is not one body — and [`crate::FloodFillReport`] **reports** it rather than
/// refusing. That is the shape to copy.
///
/// Audited against this line: every variant here and in [`crate::FloodFillError`] is
/// definitional. [`FloodFillError::NoOutsideSeed`](crate::FloodFillError::NoOutsideSeed)
/// is the closest call — it reads like "your bounds are too tight" — but mechanically the
/// BFS has no cell to start from, so there is no answer to compute rather than a poor one.
#[derive(Debug, Error)]
pub enum SdfError {
    /// Mesh has no faces (nothing to build a surface or BVH from).
    #[error("mesh is empty")]
    EmptyMesh,

    /// Grid dimensions are invalid.
    ///
    /// ⚠ **Dead: no code in the workspace constructs this.** Surfaced by the audit above.
    /// Removing it is a breaking change to a public enum, so it is left for a deliberate
    /// API pass rather than smuggled into a behaviour PR — but it is safe to remove when
    /// that happens: every downstream consumer wraps `SdfError` with `#[from]`, so nothing
    /// matches it exhaustively (`mesh-offset`, `cf-codesign`) and `mesh-lattice` keeps its
    /// own `String` variant.
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
