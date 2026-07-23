//! [`Tet10Mesh`] — the enriched quadratic (Tet10) tet mesh (ladder rung 3a).
//!
//! Wraps a linear (Tet4) mesh's four-corner connectivity with the six
//! edge-midpoint nodes a [`Tet10`](crate::element::Tet10) element needs,
//! produced by [`enrich_tet4_to_tet10`]. Built with
//! [`Tet10Mesh::from_tet4`] from any linear [`Mesh`].
//!
//! # Rung 3a: additive, and provably bit-identical Tet4
//!
//! The enrichment is *plumbing only* — it changes what the mesh stores,
//! not what any element computes:
//!
//! - **Positions grow, corners are preserved verbatim.** [`Mesh::positions`]
//!   returns the four corner positions unchanged, with the deduplicated
//!   midside positions appended after them (`positions[..n_corners]` is
//!   bit-identical to the source), and [`Mesh::n_vertices`] counts both.
//! - **[`Mesh::tet_vertices`] still returns four corners.** The midside
//!   nodes are surfaced *only* through the additive
//!   [`Mesh::tet_midside_nodes`] channel, never through `tet_vertices`.
//!   So a Tet4 solver never references them: the construction-time orphan
//!   auto-pin (`solver::backward_euler::construct`) Dirichlet-clamps every
//!   unreferenced midside, they contribute no free DOF, and the solve is
//!   bit-identical to the un-enriched Tet4 solve. Rung 3b is the first
//!   consumer that reads the midside channel to free those DOFs.
//! - **The four corner-derived caches are copied verbatim.** Materials,
//!   interface flags, quality metrics, and boundary faces all read only
//!   the four corners and the corner positions, which enrichment leaves
//!   untouched — so [`Tet10Mesh::from_tet4`] copies them straight from the
//!   source rather than re-deriving them, which keeps them bit-identical
//!   by construction. (The three-node boundary faces stay corner-only; the
//!   six-node quadratic boundary faces are ladder rung 8, deferred.)

use super::{Mesh, MeshAdjacency, QualityMetrics, TetId, VertexId, enrich::enrich_tet4_to_tet10};
use crate::Vec3;
use crate::material::NeoHookean;

/// Enriched quadratic (Tet10) tet mesh — four corners plus six
/// edge-midpoint nodes per tet.
///
/// Constructed from a linear mesh via [`Tet10Mesh::from_tet4`]. Neo-Hookean
/// per-tet materials (sibling of [`HandBuiltTetMesh`](super::HandBuiltTetMesh),
/// which is likewise NH-only); a future rung generalizes the material type if
/// a Yeoh Tet10 scene needs it. Fields are private — external code constructs
/// only via [`Tet10Mesh::from_tet4`], which preserves the corner id-space and
/// appends midside nodes after it.
#[derive(Clone, Debug)]
pub struct Tet10Mesh {
    /// Corner positions (indices `0..n_corners`, verbatim from the source
    /// mesh) followed by the deduplicated edge-midpoint positions.
    positions: Vec<Vec3>,
    /// Ten-node connectivity per tet: slots `0..4` are the corners (as
    /// [`Mesh::tet_vertices`] returns them), slots `4..10` the midside nodes
    /// (as [`Mesh::tet_midside_nodes`] returns them), in canonical
    /// [`TET10_EDGE_NODES`](crate::element::TET10_EDGE_NODES) order.
    tets: Vec<[VertexId; 10]>,
    /// Number of corner vertices (= the source mesh's vertex count). Every
    /// midside `VertexId` is `>= n_corners`.
    n_corners: usize,
    adj: MeshAdjacency,
    q: QualityMetrics,
    material_cache: Vec<NeoHookean>,
    interface_flags: Vec<bool>,
    boundary_faces: Vec<[VertexId; 3]>,
}

impl Tet10Mesh {
    /// Enrich a linear (Tet4) [`Mesh`] into a [`Tet10Mesh`].
    ///
    /// Reads the source's four-corner connectivity and corner positions,
    /// runs [`enrich_tet4_to_tet10`] to add the shared edge-midpoint nodes,
    /// and copies the four corner-derived caches (materials, interface flags,
    /// quality metrics, boundary faces) verbatim — they are enrichment-
    /// invariant, so copying keeps them bit-identical to the source (see the
    /// module docs). Works for any linear mesh, including one carrying
    /// unreferenced orphan lattice points: enrichment preserves orphans as
    /// corners and appends midsides only for referenced edges.
    ///
    /// The source must be a *linear* mesh (every tet a plain four-corner
    /// tetrahedron); passing an already-enriched [`Tet10Mesh`] would treat
    /// its midside positions as extra corners.
    //
    // `as TetId` is the Mesh-trait API tax (sister to `referenced_vertices`
    // and the other impls' `equals_structurally`): `n_tets()` returns `usize`
    // while `tet_vertices()` takes `TetId = u32`; vertex/tet counts stay far
    // below `u32::MAX`.
    #[allow(clippy::cast_possible_truncation)]
    #[must_use]
    pub fn from_tet4(mesh: &dyn Mesh) -> Self {
        // Corner-derived caches are enrichment-invariant — copy, don't
        // re-derive (see module docs). Cloning preserves them bit-for-bit.
        let q = mesh.quality().clone();
        let material_cache = mesh.materials().to_vec();
        let interface_flags = mesh.interface_flags().to_vec();
        let boundary_faces = mesh.boundary_faces().to_vec();

        let corner_tets: Vec<[VertexId; 4]> = (0..mesh.n_tets() as TetId)
            .map(|tet| mesh.tet_vertices(tet))
            .collect();

        let enriched = enrich_tet4_to_tet10(mesh.positions(), &corner_tets);

        Self {
            positions: enriched.positions,
            tets: enriched.tets,
            n_corners: enriched.n_corners,
            adj: MeshAdjacency,
            q,
            material_cache,
            interface_flags,
            boundary_faces,
        }
    }

    /// Number of corner vertices — every midside `VertexId` is `>= n_corners`.
    ///
    /// The corner/midside split rung 3b uses to keep corners free and
    /// midsides pinned.
    #[must_use]
    pub const fn n_corners(&self) -> usize {
        self.n_corners
    }
}

impl Mesh for Tet10Mesh {
    fn n_tets(&self) -> usize {
        self.tets.len()
    }

    fn n_vertices(&self) -> usize {
        self.positions.len()
    }

    fn tet_vertices(&self, tet: TetId) -> [VertexId; 4] {
        let t = &self.tets[tet as usize];
        [t[0], t[1], t[2], t[3]]
    }

    fn tet_midside_nodes(&self, tet: TetId) -> Option<[VertexId; 6]> {
        let t = &self.tets[tet as usize];
        Some([t[4], t[5], t[6], t[7], t[8], t[9]])
    }

    fn positions(&self) -> &[Vec3] {
        &self.positions
    }

    fn adjacency(&self) -> &MeshAdjacency {
        &self.adj
    }

    fn quality(&self) -> &QualityMetrics {
        &self.q
    }

    fn materials(&self) -> &[NeoHookean] {
        &self.material_cache
    }

    fn interface_flags(&self) -> &[bool] {
        &self.interface_flags
    }

    fn boundary_faces(&self) -> &[[VertexId; 3]] {
        &self.boundary_faces
    }

    // Mirror of `HandBuiltTetMesh::equals_structurally`: same vertex count,
    // same tet count, same per-tet corner indices. Midsides are excluded
    // because enrichment derives them deterministically from the corners —
    // equal corners plus equal vertex count implies equal midside
    // connectivity — and positions are the change-detection signal, not
    // structural identity (Ch 00 §02 mesh claim 3). A Tet10Mesh never
    // compares equal to a linear mesh of the same corner topology: its
    // `n_vertices` includes the midsides.
    //
    // `as TetId` cast is the Mesh-trait API tax, as in `from_tet4` above.
    #[allow(clippy::cast_possible_truncation)]
    fn equals_structurally(&self, other: &dyn Mesh) -> bool {
        if self.n_tets() != other.n_tets() {
            return false;
        }
        if self.n_vertices() != other.n_vertices() {
            return false;
        }
        for tet_id in 0..self.n_tets() as TetId {
            if self.tet_vertices(tet_id) != other.tet_vertices(tet_id) {
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    // `as TetId` casts on these small hand-built meshes are trivially in
    // range — same Mesh-trait API tax as the impl above.
    #![allow(clippy::cast_possible_truncation)]
    // `.expect()` on the midside channel surfaces a `Tet10Mesh` contract
    // violation (it must return `Some`) as a test failure — matching the
    // crate's integration-test convention (e.g. `contact_passthrough.rs`).
    #![allow(clippy::expect_used)]

    use super::*;
    use crate::element::TET10_EDGE_NODES;
    use crate::material::MaterialField;
    use crate::mesh::HandBuiltTetMesh;

    fn canonical_field() -> MaterialField {
        MaterialField::uniform(1.0e5, 4.0e5)
    }

    /// Corners + positions preserved verbatim, midsides appended after them.
    #[test]
    fn from_tet4_preserves_corners_and_appends_midsides() {
        let tet4 = HandBuiltTetMesh::two_tet_shared_face(&canonical_field());
        let tet10 = Tet10Mesh::from_tet4(&tet4);

        // Corner id-space unchanged.
        assert_eq!(tet10.n_corners(), tet4.n_vertices());
        assert_eq!(
            &tet10.positions()[..tet4.n_vertices()],
            tet4.positions(),
            "corner positions must be bit-identical to the source",
        );
        // Two tets sharing a face → 6 corner edges each, 3 shared (the
        // face's edges) → 6 + 6 - 3 = 9 unique midsides appended after the
        // 5 corners.
        assert_eq!(tet10.n_vertices(), 5 + 9);
        assert_eq!(tet10.n_tets(), tet4.n_tets());

        for tet_id in 0..tet10.n_tets() as TetId {
            assert_eq!(
                tet10.tet_vertices(tet_id),
                tet4.tet_vertices(tet_id),
                "tet_vertices must still return the four corners",
            );
            let mids = tet10
                .tet_midside_nodes(tet_id)
                .expect("Tet10Mesh surfaces the midside channel");
            for m in mids {
                assert!(
                    (m as usize) >= tet10.n_corners(),
                    "midside id {m} must be >= n_corners {}",
                    tet10.n_corners(),
                );
            }
        }
    }

    /// Mesh-level ordering detector: midside channel slot `i` is the midpoint
    /// of corner edge `TET10_EDGE_NODES[i]`. Mirrors the producer's
    /// `local_slots_match_canonical_table`, one layer up.
    #[test]
    fn tet_midside_nodes_match_canonical_table() {
        let tet4 = HandBuiltTetMesh::two_tet_shared_face(&canonical_field());
        let tet10 = Tet10Mesh::from_tet4(&tet4);
        let positions = tet10.positions();

        for tet_id in 0..tet10.n_tets() as TetId {
            let corners = tet10.tet_vertices(tet_id);
            let mids = tet10.tet_midside_nodes(tet_id).expect("midside channel");
            for (i, &(a, b)) in TET10_EDGE_NODES.iter().enumerate() {
                let expected =
                    (positions[corners[a] as usize] + positions[corners[b] as usize]) * 0.5;
                let got = positions[mids[i] as usize];
                assert!(
                    (got - expected).norm() < 1e-15,
                    "tet {tet_id} midside slot {i} (edge {a}-{b}): got {got:?}, \
                     expected {expected:?}",
                );
            }
        }
    }

    /// The four corner-derived caches are copied verbatim from the source.
    #[test]
    fn corner_caches_copied_from_source() {
        let tet4 = HandBuiltTetMesh::two_tet_shared_face(&canonical_field());
        let tet10 = Tet10Mesh::from_tet4(&tet4);

        // Per-tet caches keep their length (n_tets is unchanged).
        assert_eq!(tet10.materials().len(), tet4.materials().len());
        assert_eq!(tet10.interface_flags(), tet4.interface_flags());
        // Boundary faces stay the three-node corner faces (rung 8 upgrades
        // these to six-node quadratic faces) — bit-identical to the source.
        assert_eq!(tet10.boundary_faces(), tet4.boundary_faces());
        assert_eq!(tet10.quality().signed_volume, tet4.quality().signed_volume,);
    }

    /// A linear mesh returns `None` from the additive channel (the default),
    /// so it stays a pure Tet4 storage surface.
    #[test]
    fn linear_mesh_has_no_midside_channel() {
        let tet4 = HandBuiltTetMesh::two_tet_shared_face(&canonical_field());
        for tet_id in 0..tet4.n_tets() as TetId {
            assert_eq!(tet4.tet_midside_nodes(tet_id), None);
        }
    }
}
