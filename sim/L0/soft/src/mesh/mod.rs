//! `Mesh` trait — tet-mesh storage abstraction.
//!
//! Seven items: counts, vertex lookup, positions, adjacency, quality,
//! and a structural-equality predicate used by change-detection. One
//! impl (`SingleTetMesh`, hand-rolled 1 tet) in skeleton; multi-tet
//! `TetMesh` lands in Phase A proper per spec §8.

use std::collections::BTreeSet;

use crate::Vec3;
use crate::material::{MaterialField, NeoHookean};

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

    /// Per-tet material parameters, indexed by [`TetId`]. Length equals
    /// [`Mesh::n_tets`].
    ///
    /// Populated at mesh-construction time by sampling a
    /// [`MaterialField`] at each tet's centroid (Tet4 default per Part 7
    /// §02 §00 — sub-leaf
    /// `70-sdf-pipeline/02-material-assignment/00-sampling.md`). Read by
    /// the Newton hot path; not re-sampled per iteration.
    fn materials(&self) -> &[NeoHookean];

    /// Per-tet interface flags, indexed by [`TetId`]. Length equals
    /// [`Mesh::n_tets`].
    ///
    /// Populated at mesh-construction time by the `|φ(x_c)| < L_e`
    /// rule per Part 7 §02 §01 — sub-leaf
    /// `70-sdf-pipeline/02-material-assignment/01-composition.md`. A
    /// tet whose centroid lands within one mean-edge-length of the
    /// material-interface SDF's zero set carries `true`; all other
    /// tets carry `false`. The flag tracks the optional
    /// [`MaterialField::interface_sdf`] slot attached at field
    /// construction; meshes built from a field with no interface SDF
    /// (uniform / `LayeredScalarField`-only) carry an all-`false`
    /// vector of length `n_tets`.
    ///
    /// Diagnostic-only per Phase 4 scope memo Decision K — the Newton
    /// hot path
    /// ([`assemble_global_int_force`](crate::CpuNewtonSolver) and
    /// [`assemble_free_hessian_triplets`](crate::CpuNewtonSolver))
    /// does NOT branch on this flag. Phase H adaptive refinement
    /// (Part 7 Ch 03) is the eventual consumer; the Phase 4
    /// consumer is Part 11 mesh-quality reporting.
    fn interface_flags(&self) -> &[bool];

    /// Structural equality: two meshes are structurally equal when they
    /// share vertex count, tet count, and per-tet vertex indices (Ch 00
    /// §02 mesh claim 3).
    fn equals_structurally(&self, other: &dyn Mesh) -> bool;
}

/// Sample a [`MaterialField`] at each tet's centroid, returning the
/// per-tet [`NeoHookean`] cache that backs [`Mesh::materials`].
///
/// Pins the Tet4 evaluation point to the centroid `(v0 + v1 + v2 + v3) /
/// 4` per Part 7 §02 §00 — Phase H Tet10 will sample at four Gauss
/// points instead and bypass this helper. Walked in `tet_id` order
/// (single-threaded) for I-5 determinism carry-forward.
#[must_use]
pub(crate) fn materials_from_field(
    positions: &[Vec3],
    tets: &[[VertexId; 4]],
    field: &MaterialField,
) -> Vec<NeoHookean> {
    tets.iter()
        .map(|&tv| {
            let v0 = positions[tv[0] as usize];
            let v1 = positions[tv[1] as usize];
            let v2 = positions[tv[2] as usize];
            let v3 = positions[tv[3] as usize];
            let centroid = (v0 + v1 + v2 + v3) * 0.25;
            field.sample(centroid)
        })
        .collect()
}

/// Compute the per-tet interface-flag cache that backs
/// [`Mesh::interface_flags`].
///
/// Implements the book Part 7 §02 §01 rule `|φ(x_c)| < L_e`, where
/// `φ` is the SDF attached to `field` via
/// [`MaterialField::with_interface_sdf`], `x_c` is the tet centroid,
/// and `L_e` is the arithmetic mean of the tet's six edge lengths
/// (book's "tet's representative edge length"). When `field` carries
/// no interface SDF, every tet is flagged `false` — the natural
/// answer for uniform fields and `LayeredScalarField`-only fields,
/// and a clean grep target via the all-`false` payload.
///
/// Walked in `tet_id` order (single-threaded) for I-5 determinism
/// carry-forward (sister of [`materials_from_field`]).
//
// `from_fn` over the SDF-`Some` arm needs the centroid-and-edge-length
// math inline; the helper is small and tightly read by mesh
// constructors, so factoring it into named inner helpers would add
// noise without information.
#[must_use]
pub(crate) fn interface_flags_from_field(
    positions: &[Vec3],
    tets: &[[VertexId; 4]],
    field: &MaterialField,
) -> Vec<bool> {
    let Some(sdf) = field.interface_sdf() else {
        return vec![false; tets.len()];
    };
    tets.iter()
        .map(|&tv| {
            let v0 = positions[tv[0] as usize];
            let v1 = positions[tv[1] as usize];
            let v2 = positions[tv[2] as usize];
            let v3 = positions[tv[3] as usize];
            let centroid = (v0 + v1 + v2 + v3) * 0.25;
            // Six-edge mean per Part 7 §02 §01: edges are the (4 choose 2)
            // = 6 vertex pairs (0,1)(0,2)(0,3)(1,2)(1,3)(2,3). Dividing
            // the sum by 6 once is one rounding instead of six per-edge
            // accumulator-divides.
            let l_e = ((v1 - v0).norm()
                + (v2 - v0).norm()
                + (v3 - v0).norm()
                + (v2 - v1).norm()
                + (v3 - v1).norm()
                + (v3 - v2).norm())
                / 6.0;
            sdf.eval(centroid).abs() < l_e
        })
        .collect()
}

/// Collect every vertex referenced by at least one tet, ascending.
///
/// Walks `mesh.tet_vertices(0..n_tets)`, accumulates ids into a
/// `BTreeSet` (sorted-iteration determinism per scope memo §3
/// Decision M D-3), returns the sorted `Vec<VertexId>`.
///
/// Used to filter unreferenced "orphan" lattice points out of
/// spatial-predicate boundary conditions on mesher-generated meshes:
/// `SdfMeshedTetMesh` retains every BCC lattice point in
/// `positions()`, including lattice corners in BCC tets that fell
/// entirely outside the SDF's zero set. Orphans have no
/// element-energy contribution and would silently zero a load
/// applied to them. The III-3 test
/// (`tests/sdf_forward_map_gradcheck.rs`) filters its max-z load
/// candidate through `referenced_vertices` before the argmax per
/// scope memo §3 Decision K.
//
// `as TetId` is the Mesh-trait API tax: `n_tets()` returns `usize`
// while `tet_vertices()` takes `TetId = u32`. Phase 3 meshes stay
// well below `u32::MAX`.
#[allow(clippy::cast_possible_truncation)]
#[must_use]
pub fn referenced_vertices(mesh: &dyn Mesh) -> Vec<VertexId> {
    let mut set: BTreeSet<VertexId> = BTreeSet::new();
    for tet_id in 0..mesh.n_tets() as TetId {
        for v in mesh.tet_vertices(tet_id) {
            set.insert(v);
        }
    }
    set.into_iter().collect()
}
