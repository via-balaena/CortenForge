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
//!   by construction. (The three-node `boundary_faces` stay corner-only; the
//!   six-node quadratic boundary faces are surfaced separately via
//!   [`Mesh::boundary_faces6`], built from the ten-node connectivity — ladder
//!   rung 8b, for the surface-integrated face-contact barrier.)

use super::{
    Mesh, MeshAdjacency, QualityMetrics, TetId, VertexId, boundary_faces6_from_tet10,
    enrich::enrich_tet4_to_tet10,
};
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
    /// Six-node (P2) boundary faces, built from the ten-node connectivity
    /// (rung 8b). Corner triples match `boundary_faces` one-for-one; the
    /// three trailing midsides complete each `[c0,c1,c2,m01,m12,m02]` face.
    boundary_faces6: Vec<[VertexId; 6]>,
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
        // The source must be linear: an already-enriched mesh surfaces
        // midside nodes, and re-enriching it would treat those as corners.
        // Cheap sentinel — midside-ness is uniform, so tet 0 stands for all.
        debug_assert!(
            mesh.n_tets() == 0 || mesh.tet_midside_nodes(0).is_none(),
            "from_tet4 expects a linear (Tet4) mesh, but the source already \
             surfaces midside nodes (an enriched mesh) — re-enrichment would \
             treat its midsides as corners",
        );

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
        let boundary_faces6 = boundary_faces6_from_tet10(&enriched.tets);

        Self {
            positions: enriched.positions,
            tets: enriched.tets,
            n_corners: enriched.n_corners,
            adj: MeshAdjacency,
            q,
            material_cache,
            interface_flags,
            boundary_faces,
            boundary_faces6,
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

    /// Move the midside nodes off the straight edge midpoints, remapping each
    /// midside rest position through `project` — the seam that lets a mesh
    /// carry a *curved* (isoparametric) soft surface so the
    /// [`Tet10`](crate::element::Tet10) element honors it exactly ("exact
    /// geometry IS the exact physics").
    ///
    /// Only midside positions (indices `>= n_corners`) are remapped; corner
    /// positions and every tet's connectivity are unchanged, so a midside
    /// shared across tets stays conforming (the one node moves once). The
    /// solver picks up the curvature automatically:
    /// [`construct`](crate::solver::backward_euler) detects midsides moved off
    /// the midpoints and switches those elements to the per-Gauss-point
    /// isoparametric Jacobian; an identity `project` leaves every element
    /// straight-edged and byte-identical to the un-curved mesh.
    ///
    /// This is the geometry *carrier* only. Deciding WHERE a boundary midside
    /// belongs — projecting onto a rigid `Sdf`, boundary-only selection,
    /// inverted-element guards — is a separate mesher concern; here the caller
    /// supplies the map (the identity for any midside it wants left straight).
    #[must_use]
    pub fn with_curved_midsides(mut self, project: impl Fn(Vec3) -> Vec3) -> Self {
        for p in &mut self.positions[self.n_corners..] {
            *p = project(*p);
        }
        self
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

    fn boundary_faces6(&self) -> Option<&[[VertexId; 6]]> {
        Some(&self.boundary_faces6)
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
        // The three-node `boundary_faces` stay corner-only, bit-identical to
        // the source (the six-node faces live in the separate `boundary_faces6`
        // cache — see `boundary_faces6_corner_triples_match_boundary_faces`).
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
        // A linear mesh also exposes no six-node boundary faces.
        assert!(tet4.boundary_faces6().is_none());
    }

    /// `with_curved_midsides` remaps only the midside positions (indices
    /// `>= n_corners`), leaving corners and connectivity untouched, and a
    /// shared midside moves exactly once (conforming preserved).
    #[test]
    fn with_curved_midsides_moves_only_midsides() {
        let tet4 = HandBuiltTetMesh::two_tet_shared_face(&canonical_field());
        let straight = Tet10Mesh::from_tet4(&tet4);
        let n_corners = straight.n_corners();
        let corners_before: Vec<Vec3> = straight.positions()[..n_corners].to_vec();

        // A deterministic non-identity map (push every point out along +z).
        let curved = straight
            .clone()
            .with_curved_midsides(|p| p + Vec3::new(0.0, 0.0, 0.1));

        // Corners are byte-identical; connectivity unchanged.
        assert_eq!(&curved.positions()[..n_corners], &corners_before[..]);
        assert_eq!(curved.n_vertices(), straight.n_vertices());
        for tet_id in 0..curved.n_tets() as TetId {
            assert_eq!(curved.tet_vertices(tet_id), straight.tet_vertices(tet_id));
            assert_eq!(
                curved.tet_midside_nodes(tet_id),
                straight.tet_midside_nodes(tet_id),
                "connectivity (node ids) must be unchanged — only positions move",
            );
        }
        // Every midside moved by exactly the map (once, even if shared).
        for (before, after) in straight.positions()[n_corners..]
            .iter()
            .zip(&curved.positions()[n_corners..])
        {
            assert_eq!(*after, before + Vec3::new(0.0, 0.0, 0.1));
        }
        // The identity map is a genuine no-op (byte-identical positions).
        let noop = straight.clone().with_curved_midsides(|p| p);
        assert_eq!(noop.positions(), straight.positions());
    }

    /// The six-node boundary faces' corner triples equal the three-node
    /// `boundary_faces` one-for-one and in the same order (rung 8b builds them
    /// with the identical winding + boundary cull), and each carries three
    /// trailing midsides.
    #[test]
    fn boundary_faces6_corner_triples_match_boundary_faces() {
        let cube = HandBuiltTetMesh::uniform_block(2, 0.1, &canonical_field());
        let tet10 = Tet10Mesh::from_tet4(&cube);
        let faces3 = tet10.boundary_faces();
        let faces6 = tet10
            .boundary_faces6()
            .expect("Tet10Mesh surfaces 6-node faces");
        assert_eq!(faces6.len(), faces3.len(), "same face count");
        for (f6, f3) in faces6.iter().zip(faces3) {
            assert_eq!(&[f6[0], f6[1], f6[2]], f3, "corner triple must match");
        }
        // A closed surface (cube boundary) has more than a handful of faces.
        assert!(faces6.len() >= 12, "unexpectedly few boundary faces");
    }

    /// Oracle-matches-SUT: each six-node face's trailing midsides are the actual
    /// edge midpoints of its edges `(c0,c1)`, `(c1,c2)`, `(c0,c2)` at rest, and
    /// each midside id equals the owning tet's `tet_midside_nodes` entry for that
    /// edge — the `[c0,c1,c2,m01,m12,m02]` order the face barrier relies on.
    #[test]
    fn boundary_faces6_midsides_are_edge_midpoints_in_canonical_order() {
        let cube = HandBuiltTetMesh::uniform_block(2, 0.1, &canonical_field());
        let tet10 = Tet10Mesh::from_tet4(&cube);
        let pos = tet10.positions();
        for f in tet10.boundary_faces6().expect("6-node faces") {
            let [c0, c1, c2, m01, m12, m02] = *f;
            let mid = |a: VertexId, b: VertexId| (pos[a as usize] + pos[b as usize]) * 0.5;
            for (m, (a, b)) in [(m01, (c0, c1)), (m12, (c1, c2)), (m02, (c0, c2))] {
                let got = pos[m as usize];
                let expected = mid(a, b);
                assert!(
                    (got - expected).norm() < 1e-14,
                    "midside {m} of edge ({a},{b}): got {got:?}, expected {expected:?}",
                );
            }
        }
    }
}
