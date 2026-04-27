//! Hand-built multi-tet meshes — Phase 2 commit 2.
//!
//! Two constructors for the Phase 2 gate scenes per
//! [`phase_2_multi_element_fem_scope.md`](../../../../docs/todo/phase_2_multi_element_fem_scope.md) §1:
//!
//! - [`HandBuiltTetMesh::two_isolated_tets`] — 8 vertices, 2
//!   mechanically-isolated tets (II-1 multi-element determinism + II-2
//!   gradient aggregation).
//! - [`HandBuiltTetMesh::two_tet_shared_face`] — 5 vertices, 2 tets
//!   sharing a face (II-3 shared-vertex tangent assembly).
//!
//! Both meshes use the canonical decimeter scale (edge `L = 0.1` m)
//! per walking-skeleton scope §2. The shared-face constructor places
//! tet 1's apex asymmetrically (NOT a mirror reflection of tet 0's
//! apex) so the shared free vertex's Hessian block has non-zero
//! off-diagonal coupling — open-decision constraint per Phase 2 scope
//! §9.

use super::{Mesh, MeshAdjacency, QualityMetrics, TetId, VertexId};
use crate::Vec3;

/// Hand-built multi-tetrahedron mesh.
///
/// Phase 2's only multi-tet [`Mesh`] impl. Constructed via per-scene
/// associated functions; Phase 3+ will add `HandBuiltTetMesh::from_sdf`
/// (or similar) once the SDF→tet bridge lands. Fields are private —
/// external code constructs only via the named scene constructors,
/// which guarantee right-handed tet orientation and topological
/// validity by hand.
#[derive(Clone, Debug)]
pub struct HandBuiltTetMesh {
    vertices: Vec<Vec3>,
    tets: Vec<[VertexId; 4]>,
    adj: MeshAdjacency,
    q: QualityMetrics,
}

impl HandBuiltTetMesh {
    /// Two isolated decimeter tets — II-1 / II-2 gate scene.
    ///
    /// 8 vertices, 2 tets, no shared vertices. Tet 0 is the canonical
    /// decimeter tet at the origin (matches
    /// [`crate::mesh::SingleTetMesh::new`]); tet 1 is the same tet
    /// translated by `(0.5, 0, 0)` m — 5× edge length so the two tets
    /// have no geometric coupling, no shared vertex IDs, and no
    /// proximity that contact would notice.
    ///
    /// Per-tet connectivity: tet 0 = `[0, 1, 2, 3]`; tet 1 = `[4, 5,
    /// 6, 7]`. Both right-handed (positive signed volume).
    #[must_use]
    pub fn two_isolated_tets() -> Self {
        let l = 0.1;
        let dx = 0.5;
        Self {
            vertices: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(l, 0.0, 0.0),
                Vec3::new(0.0, l, 0.0),
                Vec3::new(0.0, 0.0, l),
                Vec3::new(dx, 0.0, 0.0),
                Vec3::new(dx + l, 0.0, 0.0),
                Vec3::new(dx, l, 0.0),
                Vec3::new(dx, 0.0, l),
            ],
            tets: vec![[0, 1, 2, 3], [4, 5, 6, 7]],
            adj: MeshAdjacency,
            q: QualityMetrics,
        }
    }

    /// Two tets sharing one face — II-3 gate scene.
    ///
    /// 5 vertices, 2 tets. Tet 0 is the canonical decimeter tet at
    /// the origin (apex `v_0`, base face `{v_1, v_2, v_3}`); tet 1
    /// glues to that base face with apex `v_4 = (0.08, 0.08, 0.08)`.
    ///
    /// **Asymmetric apex placement** is load-bearing for II-3: tet 1's
    /// apex is NOT the mirror reflection of `v_0` across the shared
    /// face. A mirror placement would produce a Hessian block at any
    /// shared free vertex with zero off-diagonal coupling (perfect
    /// symmetry cancels), which would let II-3 pass on a buggy
    /// assembly that drops shared-vertex coupling entirely. The chosen
    /// `(0.08, 0.08, 0.08)` is well off the mirror axis while keeping
    /// tet 1 right-handed and non-sliver.
    ///
    /// Per-tet connectivity: tet 0 = `[0, 1, 2, 3]`; tet 1 = `[1, 2,
    /// 3, 4]`. Both right-handed.
    #[must_use]
    pub fn two_tet_shared_face() -> Self {
        let l = 0.1;
        Self {
            vertices: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(l, 0.0, 0.0),
                Vec3::new(0.0, l, 0.0),
                Vec3::new(0.0, 0.0, l),
                Vec3::new(0.08, 0.08, 0.08),
            ],
            tets: vec![[0, 1, 2, 3], [1, 2, 3, 4]],
            adj: MeshAdjacency,
            q: QualityMetrics,
        }
    }
}

impl Mesh for HandBuiltTetMesh {
    fn n_tets(&self) -> usize {
        self.tets.len()
    }

    fn n_vertices(&self) -> usize {
        self.vertices.len()
    }

    fn tet_vertices(&self, tet: TetId) -> [VertexId; 4] {
        let idx = tet as usize;
        assert!(
            idx < self.tets.len(),
            "tet ID {idx} out of bounds for {n}-tet HandBuiltTetMesh",
            n = self.tets.len(),
        );
        self.tets[idx]
    }

    fn positions(&self) -> &[Vec3] {
        &self.vertices
    }

    fn adjacency(&self) -> &MeshAdjacency {
        &self.adj
    }

    fn quality(&self) -> &QualityMetrics {
        &self.q
    }

    // Mirror of `SingleTetMesh::equals_structurally` generalized to N
    // tets: same vertex count + same tet count + same per-tet vertex
    // indices in the same tet-id order. Positions deliberately
    // excluded — those are the change-detection signal, not structural
    // identity (matches Ch 00 §02 mesh claim 3).
    //
    // `as TetId` cast is the Mesh-trait-API tax: `n_tets()` returns
    // `usize`, `tet_vertices()` takes `TetId = u32`. Hand-built meshes
    // hold ≤ N=2 tets per Phase 2 scope §3 Decision A; cast is
    // trivially in range.
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
    // `as TetId` casts on hand-built ≤ N=2 meshes are trivially in
    // range — same Mesh-trait-API tax as `equals_structurally`.
    #![allow(clippy::cast_possible_truncation)]

    use nalgebra::Matrix3;

    use super::*;

    /// Signed volume of one tet from positions + connectivity. Used by
    /// the right-handedness checks below — a left-handed tet would
    /// silently break `CpuNewtonSolver::reference_geometry`'s `volume
    /// = |det| / 6` (correct magnitude, wrong sign on `F` at rest →
    /// inverted deformation gradient → Newton divergence). Catching
    /// at mesh-construction time is much cheaper than catching at
    /// solver time.
    fn signed_volume(positions: &[Vec3], tet: [VertexId; 4]) -> f64 {
        let v0 = positions[tet[0] as usize];
        let v1 = positions[tet[1] as usize];
        let v2 = positions[tet[2] as usize];
        let v3 = positions[tet[3] as usize];
        let m = Matrix3::from_columns(&[v1 - v0, v2 - v0, v3 - v0]);
        m.determinant() / 6.0
    }

    #[test]
    fn two_isolated_tets_has_8_vertices_and_2_tets() {
        let mesh = HandBuiltTetMesh::two_isolated_tets();
        assert_eq!(mesh.n_vertices(), 8);
        assert_eq!(mesh.n_tets(), 2);
    }

    #[test]
    fn two_isolated_tets_share_no_vertices() {
        let mesh = HandBuiltTetMesh::two_isolated_tets();
        let t0 = mesh.tet_vertices(0);
        let t1 = mesh.tet_vertices(1);
        for &v0 in &t0 {
            assert!(
                !t1.contains(&v0),
                "two_isolated_tets must share no vertex IDs (vertex {v0} appears in both tets)"
            );
        }
    }

    #[test]
    fn two_isolated_tets_are_right_handed() {
        let mesh = HandBuiltTetMesh::two_isolated_tets();
        for tet_id in 0..mesh.n_tets() as TetId {
            let vol = signed_volume(mesh.positions(), mesh.tet_vertices(tet_id));
            assert!(
                vol > 0.0,
                "tet {tet_id} must be right-handed (positive signed volume), got {vol:e}"
            );
        }
    }

    #[test]
    fn two_isolated_tets_match_canonical_decimeter_volume() {
        // Canonical decimeter tet: V = L^3 / 6 = 0.1^3 / 6 ≈ 1.667e-4 m^3.
        // Both tets are translated copies of the canonical, so both
        // must produce that exact volume (no nonlinear scaling).
        let mesh = HandBuiltTetMesh::two_isolated_tets();
        let expected = 0.1_f64.powi(3) / 6.0;
        for tet_id in 0..mesh.n_tets() as TetId {
            let vol = signed_volume(mesh.positions(), mesh.tet_vertices(tet_id));
            assert!(
                (vol - expected).abs() < 1e-15,
                "tet {tet_id} volume {vol:e} ≠ canonical decimeter {expected:e}"
            );
        }
    }

    #[test]
    fn two_tet_shared_face_has_5_vertices_and_2_tets() {
        let mesh = HandBuiltTetMesh::two_tet_shared_face();
        assert_eq!(mesh.n_vertices(), 5);
        assert_eq!(mesh.n_tets(), 2);
    }

    #[test]
    fn two_tet_shared_face_shares_exactly_three_vertices() {
        let mesh = HandBuiltTetMesh::two_tet_shared_face();
        let t0 = mesh.tet_vertices(0);
        let t1 = mesh.tet_vertices(1);
        let shared: usize = t0.iter().filter(|v| t1.contains(v)).count();
        assert_eq!(
            shared, 3,
            "two_tet_shared_face must share exactly 3 vertex IDs (one shared face)"
        );
    }

    #[test]
    fn two_tet_shared_face_are_right_handed() {
        let mesh = HandBuiltTetMesh::two_tet_shared_face();
        for tet_id in 0..mesh.n_tets() as TetId {
            let vol = signed_volume(mesh.positions(), mesh.tet_vertices(tet_id));
            assert!(
                vol > 0.0,
                "tet {tet_id} must be right-handed (positive signed volume), got {vol:e}"
            );
        }
    }

    #[test]
    fn two_tet_shared_face_apex_is_not_mirror_of_v0() {
        // Mirror of v_0 = (0,0,0) across the plane x + y + z = 0.1
        // would land at (2/3·0.1, 2/3·0.1, 2/3·0.1) ≈ (0.0667,
        // 0.0667, 0.0667). The chosen apex (0.08, 0.08, 0.08) is well
        // off that point — verifies the asymmetric-placement
        // constraint from Phase 2 scope §9 holds at the geometry
        // level. Future edits to the apex must keep this distance
        // non-trivial.
        let mesh = HandBuiltTetMesh::two_tet_shared_face();
        let apex = mesh.positions()[4];
        let mirror = Vec3::new(2.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0) * 0.1;
        let dist = (apex - mirror).norm();
        assert!(
            dist > 0.01,
            "tet-1 apex {apex:?} is too close to the mirror reflection of v_0 \
             (distance {dist:e} m, must be > 1 cm to avoid Hessian-zeroing symmetry)"
        );
    }

    #[test]
    fn equals_structurally_same_constructor() {
        let a = HandBuiltTetMesh::two_isolated_tets();
        let b = HandBuiltTetMesh::two_isolated_tets();
        assert!(a.equals_structurally(&b));
    }

    #[test]
    fn equals_structurally_different_topology() {
        let isolated = HandBuiltTetMesh::two_isolated_tets();
        let shared = HandBuiltTetMesh::two_tet_shared_face();
        assert!(
            !isolated.equals_structurally(&shared),
            "isolated and shared-face meshes must NOT be structurally equal \
             (different vertex counts: 8 vs 5)"
        );
    }

    #[test]
    #[should_panic(expected = "out of bounds")]
    fn tet_vertices_out_of_bounds_panics() {
        let mesh = HandBuiltTetMesh::two_isolated_tets();
        let _ = mesh.tet_vertices(5);
    }
}
