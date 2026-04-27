//! `Mesh` trait — tet-mesh storage abstraction.
//!
//! Seven items: counts, vertex lookup, positions, adjacency, quality,
//! and a structural-equality predicate used by change-detection. One
//! impl (`SingleTetMesh`, hand-rolled 1 tet) in skeleton; multi-tet
//! `TetMesh` lands in Phase A proper per spec §8.

use crate::Vec3;

pub mod hand_built;
pub mod quality;
pub mod single_tet;

pub use hand_built::HandBuiltTetMesh;
pub use single_tet::SingleTetMesh;

/// Tetrahedron index into a [`Mesh`]. `u32` caps at 4 B tets; skeleton
/// needs only `0`.
pub type TetId = u32;

/// Vertex index into a [`Mesh`]. `u32` caps at 4 B vertices; skeleton
/// needs `0..4`.
pub type VertexId = u32;

/// Per-element neighbour graph (tet-face adjacency). Skeleton has none
/// — single tet, no neighbours.
#[derive(Clone, Debug, Default)]
pub struct MeshAdjacency;

/// Per-tet quality metrics. All four fields are indexed by [`TetId`]
/// and have length equal to [`Mesh::n_tets`] for the owning mesh.
///
/// Populated at mesh construction time by
/// [`quality::compute_metrics`].
#[derive(Clone, Debug, Default)]
pub struct QualityMetrics {
    /// Inscribed-sphere-over-circumscribed-sphere ratio per tet.
    /// Equals 1/3 for a regular tet (theoretical max); approaches 0
    /// as tets degenerate to slivers.
    pub aspect_ratio: Vec<f64>,
    /// Minimum interior dihedral angle (radians) across the 6 edges,
    /// per tet.
    pub dihedral_min: Vec<f64>,
    /// Maximum interior dihedral angle (radians) across the 6 edges,
    /// per tet.
    pub dihedral_max: Vec<f64>,
    /// Signed tetrahedron volume per tet. Positive for right-handed
    /// orientation, negative for left-handed; magnitude equals the
    /// geometric volume.
    pub signed_volume: Vec<f64>,
}

/// Tet-mesh storage surface — read-only view of topology, positions,
/// and precomputed adjacency / quality.
pub trait Mesh: Send + Sync {
    /// Number of tetrahedra in the mesh.
    fn n_tets(&self) -> usize;

    /// Number of vertices in the mesh.
    fn n_vertices(&self) -> usize;

    /// Four-vertex indices for the given tetrahedron.
    fn tet_vertices(&self, tet: TetId) -> [VertexId; 4];

    /// Rest-configuration vertex positions, indexed by `VertexId`.
    fn positions(&self) -> &[Vec3];

    /// Neighbour graph (face-adjacency) — cold lookup.
    fn adjacency(&self) -> &MeshAdjacency;

    /// Precomputed quality metrics for the mesh.
    fn quality(&self) -> &QualityMetrics;

    /// Structural equality: two meshes are structurally equal when they
    /// share vertex count, tet count, and per-tet vertex indices (Ch 00
    /// §02 mesh claim 3).
    fn equals_structurally(&self, other: &dyn Mesh) -> bool;
}
