//! Hand-built multi-tet meshes — Phase 2 commit 2 + Phase 4 commit 8.
//!
//! Three constructors:
//!
//! - [`HandBuiltTetMesh::two_isolated_tets`] — 8 vertices, 2
//!   mechanically-isolated tets (Phase 2 II-1 multi-element determinism
//!   + II-2 gradient aggregation).
//! - [`HandBuiltTetMesh::two_tet_shared_face`] — 5 vertices, 2 tets
//!   sharing a face (Phase 2 II-3 shared-vertex tangent assembly).
//! - [`HandBuiltTetMesh::cantilever_bilayer_beam`] — `(nx, ny, nz)`
//!   hex-grid cantilever block with through-thickness bilayer material
//!   split, each cell decomposed into 6 right-handed tets via
//!   Coxeter-Freudenthal-Kuhn (Phase 4 IV-3 bonded-bilayer beam gate
//!   scene per `phase_4_multi_material_scope.md` §6 IV-3 + §8
//!   commit 8).
//!
//! The Phase 2 constructors use the canonical decimeter scale (edge
//! `L = 0.1` m) per walking-skeleton scope §2. The shared-face
//! constructor places tet 1's apex asymmetrically (NOT a mirror
//! reflection of tet 0's apex) so the shared free vertex's Hessian
//! block has non-zero off-diagonal coupling — open-decision
//! constraint per Phase 2 scope §9.

use super::{
    Mesh, MeshAdjacency, QualityMetrics, TetId, VertexId, boundary_faces_from_topology,
    interface_flags_from_field, materials_from_field, quality,
};
use crate::Vec3;
use crate::material::{MaterialField, NeoHookean};

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
    material_cache: Vec<NeoHookean>,
    interface_flags: Vec<bool>,
    boundary_faces: Vec<[VertexId; 3]>,
}

impl HandBuiltTetMesh {
    /// Two isolated decimeter tets — II-1 / II-2 gate scene, sampled
    /// against `field` at each tet centroid (Decision K).
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
    pub fn two_isolated_tets(field: &MaterialField) -> Self {
        let l = 0.1;
        let dx = 0.5;
        let vertices = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(l, 0.0, 0.0),
            Vec3::new(0.0, l, 0.0),
            Vec3::new(0.0, 0.0, l),
            Vec3::new(dx, 0.0, 0.0),
            Vec3::new(dx + l, 0.0, 0.0),
            Vec3::new(dx, l, 0.0),
            Vec3::new(dx, 0.0, l),
        ];
        let tets = vec![[0, 1, 2, 3], [4, 5, 6, 7]];
        let q = quality::compute_metrics(&vertices, &tets);
        let material_cache = materials_from_field(&vertices, &tets, field);
        let interface_flags = interface_flags_from_field(&vertices, &tets, field);
        let boundary_faces = boundary_faces_from_topology(&tets);
        Self {
            vertices,
            tets,
            adj: MeshAdjacency,
            q,
            material_cache,
            interface_flags,
            boundary_faces,
        }
    }

    /// Two tets sharing one face — II-3 gate scene, sampled against
    /// `field` at each tet centroid (Decision K).
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
    pub fn two_tet_shared_face(field: &MaterialField) -> Self {
        let l = 0.1;
        let vertices = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(l, 0.0, 0.0),
            Vec3::new(0.0, l, 0.0),
            Vec3::new(0.0, 0.0, l),
            Vec3::new(0.08, 0.08, 0.08),
        ];
        let tets = vec![[0, 1, 2, 3], [1, 2, 3, 4]];
        let q = quality::compute_metrics(&vertices, &tets);
        let material_cache = materials_from_field(&vertices, &tets, field);
        let interface_flags = interface_flags_from_field(&vertices, &tets, field);
        let boundary_faces = boundary_faces_from_topology(&tets);
        Self {
            vertices,
            tets,
            adj: MeshAdjacency,
            q,
            material_cache,
            interface_flags,
            boundary_faces,
        }
    }

    /// Cantilever beam — IV-3 bonded-bilayer gate scene.
    ///
    /// Hex-grid mesh of an `(length × breadth × height)` rectangular
    /// block, subdivided into `(nx × ny × nz)` cells. Each cell is
    /// decomposed into 6 tets via the Coxeter-Freudenthal-Kuhn
    /// triangulation: every cell shares the body diagonal `v_0 → v_6`,
    /// all 6 tets are right-handed, and the diagonal split on every
    /// shared face is the same body-diagonal direction in both
    /// neighbouring cells, so the mesh tiles conformally without
    /// parity-flips.
    ///
    /// **Cantilever orientation:** length axis is `+x̂`, breadth is
    /// `+ŷ`, thickness (bilayer-split axis) is `+ẑ`. Vertices on the
    /// `x = 0` face are the natural clamp set; vertices on the `x =
    /// length` face are the natural tip-load set. A `+ẑ` tip force
    /// drives upward bending displacement, which lets `tests/`
    /// callers re-use the existing [`LoadAxis::AxisZ`][la] convention
    /// for the per-vertex traction broadcast (`THETA = F_total /
    /// n_tip_vertices`).
    ///
    /// **Bilayer split:** the per-tet [`MaterialField`] sample at each
    /// tet centroid uses `centroid.z` as the discriminator — the
    /// caller wires a `Field<f64>` whose `sample(x_ref)` switches at
    /// `x_ref.z = height / 2`. `nz` is required to be even so the
    /// interface aligns with a cell boundary; no tet straddles the
    /// interface plane.
    ///
    /// Vertex indexing (row-major within each `k`-slice, slice-major
    /// in `k`): `(i, j, k) → i + (nx + 1) * (j + (ny + 1) * k)`,
    /// `i ∈ [0, nx]`, `j ∈ [0, ny]`, `k ∈ [0, nz]`.
    ///
    /// Per-cell local corner indexing (standard hex-element order):
    ///
    /// ```text
    /// 0 = (i,     j,     k)         4 = (i,     j,     k + 1)
    /// 1 = (i + 1, j,     k)         5 = (i + 1, j,     k + 1)
    /// 2 = (i + 1, j + 1, k)         6 = (i + 1, j + 1, k + 1)
    /// 3 = (i,     j + 1, k)         7 = (i,     j + 1, k + 1)
    /// ```
    ///
    /// Per-cell tet decomposition (6 tets, all right-handed by
    /// construction, every tet contains the body diagonal
    /// `[c[0], c[6]]`):
    ///
    /// ```text
    /// [c[0], c[1], c[2], c[6]]      [c[0], c[7], c[4], c[6]]
    /// [c[0], c[2], c[3], c[6]]      [c[0], c[4], c[5], c[6]]
    /// [c[0], c[3], c[7], c[6]]      [c[0], c[5], c[1], c[6]]
    /// ```
    ///
    /// [la]: crate::LoadAxis
    ///
    /// # Panics
    /// - `nx == 0`, `ny == 0`, or `nz == 0`.
    /// - `nz` is odd (bilayer interface must align to a cell
    ///   boundary).
    /// - any of `length`, `breadth`, `height` is non-positive.
    //
    // `usize as f64` casts on the integer cell-count and index inputs
    // sit far below `f64`'s 52-bit mantissa precision range — the
    // largest grid this crate's tests build is `(40, 16, 16)` (`< 2^16`
    // cells per axis); the casts are the construction-time arithmetic
    // the per-element FEM pipeline reads as `f64`. `usize as VertexId =
    // u32` casts in the `vid` closure cap at `~12 000` vertices for
    // the same grid, well below `u32::MAX`. Both lints are the
    // standard `Mesh`-trait API tax mirrored in `equals_structurally`
    // and `referenced_vertices` in this module.
    #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
    #[must_use]
    pub fn cantilever_bilayer_beam(
        nx: usize,
        ny: usize,
        nz: usize,
        length: f64,
        breadth: f64,
        height: f64,
        field: &MaterialField,
    ) -> Self {
        assert!(nx >= 1, "nx must be >= 1, got {nx}");
        assert!(ny >= 1, "ny must be >= 1, got {ny}");
        assert!(
            nz >= 2 && nz.is_multiple_of(2),
            "nz must be >= 2 and even (bilayer interface aligns at z = height/2), got {nz}",
        );
        assert!(length > 0.0, "length must be positive, got {length}");
        assert!(breadth > 0.0, "breadth must be positive, got {breadth}");
        assert!(height > 0.0, "height must be positive, got {height}");

        let dx = length / nx as f64;
        let dy = breadth / ny as f64;
        let dz = height / nz as f64;

        let mut vertices: Vec<Vec3> = Vec::with_capacity((nx + 1) * (ny + 1) * (nz + 1));
        for k in 0..=nz {
            for j in 0..=ny {
                for i in 0..=nx {
                    vertices.push(Vec3::new(i as f64 * dx, j as f64 * dy, k as f64 * dz));
                }
            }
        }

        // Hex-grid → CFK 6-tet decomposition. Each cell is split via
        // the body diagonal `c[0] → c[6]` into 6 right-handed tets;
        // every shared face between neighbouring cells is split with
        // the same world-space diagonal, so the mesh tiles
        // conformally without parity flips.
        let stride_y = nx + 1;
        let stride_z = (nx + 1) * (ny + 1);
        let vid = |i: usize, j: usize, k: usize| -> VertexId {
            (i + j * stride_y + k * stride_z) as VertexId
        };

        let mut tets: Vec<[VertexId; 4]> = Vec::with_capacity(6 * nx * ny * nz);
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let c = [
                        vid(i, j, k),
                        vid(i + 1, j, k),
                        vid(i + 1, j + 1, k),
                        vid(i, j + 1, k),
                        vid(i, j, k + 1),
                        vid(i + 1, j, k + 1),
                        vid(i + 1, j + 1, k + 1),
                        vid(i, j + 1, k + 1),
                    ];
                    tets.push([c[0], c[1], c[2], c[6]]);
                    tets.push([c[0], c[2], c[3], c[6]]);
                    tets.push([c[0], c[3], c[7], c[6]]);
                    tets.push([c[0], c[7], c[4], c[6]]);
                    tets.push([c[0], c[4], c[5], c[6]]);
                    tets.push([c[0], c[5], c[1], c[6]]);
                }
            }
        }

        let q = quality::compute_metrics(&vertices, &tets);
        let material_cache = materials_from_field(&vertices, &tets, field);
        let interface_flags = interface_flags_from_field(&vertices, &tets, field);
        let boundary_faces = boundary_faces_from_topology(&tets);
        Self {
            vertices,
            tets,
            adj: MeshAdjacency,
            q,
            material_cache,
            interface_flags,
            boundary_faces,
        }
    }

    /// Uniform-material cube — compressive-block gate scene.
    ///
    /// Same `(n × n × n)` hex-grid + Coxeter-Freudenthal-Kuhn 6-tets-per-cell
    /// decomposition as [`Self::cantilever_bilayer_beam`] with cube
    /// dimensions `(edge_len, edge_len, edge_len)`. The bilayer-interface
    /// constraint (`nz` even, interface aligns at `z = height/2`) is
    /// preserved by delegation, but the compressive-block fixture feeds
    /// a uniform `MaterialField` so the would-be interface is invisible
    /// — both halves carry the same `(μ, λ)` Lamé pair and
    /// centroid-sampling produces an isotropic per-tet material cache.
    ///
    /// `tests/penalty_compressive_block.rs` feeds three refinement
    /// levels at `edge_len = 0.01` m (1 cm cube): `n = 2 / 4 / 8` for
    /// cell sizes `5 / 2.5 / 1.25` mm — all even, satisfying the
    /// inherited constraint by construction. n=1 (single-cell cube) is
    /// rejected by the inherited assert; widening it would require
    /// relaxing [`Self::cantilever_bilayer_beam`]'s constraint (no
    /// fixture exercises a single-cell cube today).
    ///
    /// # Panics
    /// - `n_per_edge` is odd or zero (inherited from
    ///   [`Self::cantilever_bilayer_beam`]'s `nz` constraint).
    /// - `edge_len` is non-positive.
    #[must_use]
    pub fn uniform_block(n_per_edge: usize, edge_len: f64, field: &MaterialField) -> Self {
        Self::cantilever_bilayer_beam(
            n_per_edge, n_per_edge, n_per_edge, edge_len, edge_len, edge_len, field,
        )
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

    fn materials(&self) -> &[NeoHookean] {
        &self.material_cache
    }

    fn interface_flags(&self) -> &[bool] {
        &self.interface_flags
    }

    fn boundary_faces(&self) -> &[[VertexId; 3]] {
        &self.boundary_faces
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

    use std::collections::{BTreeMap, BTreeSet};

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

    /// Phase 2 IV-1 regression target: the Ecoflex-class `(μ, λ) =
    /// (1e5, 4e5)` previously hardcoded in the skeleton solver, threaded
    /// here as a uniform `MaterialField` so the topology/geometry tests
    /// stay material-agnostic while the new constructors require an
    /// explicit field per Decision P.
    fn canonical_field() -> MaterialField {
        MaterialField::uniform(1.0e5, 4.0e5)
    }

    #[test]
    fn two_isolated_tets_has_8_vertices_and_2_tets() {
        let mesh = HandBuiltTetMesh::two_isolated_tets(&canonical_field());
        assert_eq!(mesh.n_vertices(), 8);
        assert_eq!(mesh.n_tets(), 2);
    }

    #[test]
    fn two_isolated_tets_share_no_vertices() {
        let mesh = HandBuiltTetMesh::two_isolated_tets(&canonical_field());
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
        let mesh = HandBuiltTetMesh::two_isolated_tets(&canonical_field());
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
        let mesh = HandBuiltTetMesh::two_isolated_tets(&canonical_field());
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
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&canonical_field());
        assert_eq!(mesh.n_vertices(), 5);
        assert_eq!(mesh.n_tets(), 2);
    }

    #[test]
    fn two_tet_shared_face_shares_exactly_three_vertices() {
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&canonical_field());
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
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&canonical_field());
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
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&canonical_field());
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
        let a = HandBuiltTetMesh::two_isolated_tets(&canonical_field());
        let b = HandBuiltTetMesh::two_isolated_tets(&canonical_field());
        assert!(a.equals_structurally(&b));
    }

    #[test]
    fn equals_structurally_different_topology() {
        let isolated = HandBuiltTetMesh::two_isolated_tets(&canonical_field());
        let shared = HandBuiltTetMesh::two_tet_shared_face(&canonical_field());
        assert!(
            !isolated.equals_structurally(&shared),
            "isolated and shared-face meshes must NOT be structurally equal \
             (different vertex counts: 8 vs 5)"
        );
    }

    #[test]
    #[should_panic(expected = "out of bounds")]
    fn tet_vertices_out_of_bounds_panics() {
        let mesh = HandBuiltTetMesh::two_isolated_tets(&canonical_field());
        let _ = mesh.tet_vertices(5);
    }

    // ── cantilever_bilayer_beam (Phase 4 commit 8) ──────────────────────

    /// Sanity geometry shared by the bilayer-beam unit tests below: a
    /// `(0.5, 0.1, 0.1)` cantilever with `(nx, ny, nz) = (4, 2, 2)`.
    /// Small enough to keep these unit tests sub-millisecond, large
    /// enough to exercise multi-cell tile-conformity (12 × 6 = 72
    /// tets, 5·3·3 = 45 vertices).
    fn small_beam(field: &MaterialField) -> HandBuiltTetMesh {
        HandBuiltTetMesh::cantilever_bilayer_beam(4, 2, 2, 0.5, 0.1, 0.1, field)
    }

    #[test]
    fn cantilever_bilayer_beam_vertex_count_matches_grid() {
        // (nx + 1) * (ny + 1) * (nz + 1) = 5 * 3 * 3 = 45.
        let mesh = small_beam(&canonical_field());
        assert_eq!(mesh.n_vertices(), 5 * 3 * 3);
    }

    #[test]
    fn cantilever_bilayer_beam_tet_count_matches_six_per_cell() {
        // 6 tets per cell × nx · ny · nz = 6 · 4 · 2 · 2 = 96.
        let mesh = small_beam(&canonical_field());
        assert_eq!(mesh.n_tets(), 6 * 4 * 2 * 2);
    }

    #[test]
    fn cantilever_bilayer_beam_all_tets_right_handed() {
        // CFK decomposition: every tet must have positive signed
        // volume by construction. Catches a future edit that flips a
        // tet's vertex order without realising orientation matters.
        let mesh = small_beam(&canonical_field());
        for tet_id in 0..mesh.n_tets() as TetId {
            let vol = signed_volume(mesh.positions(), mesh.tet_vertices(tet_id));
            assert!(
                vol > 0.0,
                "tet {tet_id} must be right-handed (positive signed volume), got {vol:e}",
            );
        }
    }

    #[test]
    fn cantilever_bilayer_beam_tet_volumes_sum_to_block_volume() {
        // Six tets per cell tile that cell exactly; total signed
        // volume across the whole mesh must equal length·breadth·height
        // to within FP noise. Catches duplicated or missing tets in
        // any future edit to the per-cell tet list.
        let mesh = small_beam(&canonical_field());
        let total: f64 = (0..mesh.n_tets() as TetId)
            .map(|tid| signed_volume(mesh.positions(), mesh.tet_vertices(tid)))
            .sum();
        let expected = 0.5 * 0.1 * 0.1;
        assert!(
            (total - expected).abs() < 1e-12,
            "total signed volume {total:e} ≠ block volume {expected:e}",
        );
    }

    #[test]
    fn cantilever_bilayer_beam_material_cache_length_matches_n_tets() {
        let mesh = small_beam(&canonical_field());
        assert_eq!(mesh.materials().len(), mesh.n_tets());
    }

    #[test]
    #[should_panic(expected = "nz must be >= 2 and even")]
    fn cantilever_bilayer_beam_panics_on_odd_nz() {
        let _mesh =
            HandBuiltTetMesh::cantilever_bilayer_beam(4, 2, 3, 0.5, 0.1, 0.1, &canonical_field());
    }

    #[test]
    #[should_panic(expected = "nz must be >= 2 and even")]
    fn cantilever_bilayer_beam_panics_on_zero_nz() {
        let _mesh =
            HandBuiltTetMesh::cantilever_bilayer_beam(4, 2, 0, 0.5, 0.1, 0.1, &canonical_field());
    }

    #[test]
    #[should_panic(expected = "nx must be >= 1")]
    fn cantilever_bilayer_beam_panics_on_zero_nx() {
        let _mesh =
            HandBuiltTetMesh::cantilever_bilayer_beam(0, 2, 2, 0.5, 0.1, 0.1, &canonical_field());
    }

    #[test]
    #[should_panic(expected = "length must be positive")]
    fn cantilever_bilayer_beam_panics_on_zero_length() {
        let _mesh =
            HandBuiltTetMesh::cantilever_bilayer_beam(4, 2, 2, 0.0, 0.1, 0.1, &canonical_field());
    }

    // -- boundary_faces -------------------------------------------------

    /// Single-tet boundary faces are pinned to the exact right-handed
    /// outward winding tuple. Catches any drift in the per-face
    /// emission convention in `boundary_faces_from_topology`.
    #[test]
    fn single_tet_boundary_faces_exact_pin() {
        use crate::mesh::SingleTetMesh;

        let mesh = SingleTetMesh::new(&canonical_field());
        let faces = mesh.boundary_faces();
        assert_eq!(
            faces,
            &[
                [1u32, 2, 3], // opposite v0
                [0, 3, 2],    // opposite v1
                [0, 1, 3],    // opposite v2
                [0, 2, 1],    // opposite v3
            ],
            "single tet must emit the four right-handed outward faces in tet-id-then-face-id order",
        );
    }

    /// Two-tet shared-face mesh has the interior face (vertex set
    /// `{1, 2, 3}`) culled — both orientations cancel at the canonical
    /// counting step, leaving 6 boundary faces (3 from each tet, with
    /// the shared face dropped).
    #[test]
    fn two_tet_shared_face_culls_interior_face() {
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&canonical_field());
        let faces = mesh.boundary_faces();
        // 4 faces per tet × 2 tets − 2 (shared face, both orientations) = 6.
        assert_eq!(
            faces.len(),
            6,
            "expected 6 boundary faces, got {}",
            faces.len()
        );

        // Tet 0 = [0, 1, 2, 3]; the (1, 2, 3) face is the shared one.
        // Tet 1 = [1, 2, 3, 4]; (1, 3, 2) — the (1,2,3) canonical key —
        // also falls out. Neither orientation should appear in the
        // output.
        let shared_canonical = {
            let mut k = [1u32, 2, 3];
            k.sort_unstable();
            k
        };
        for face in faces {
            let mut k = *face;
            k.sort_unstable();
            assert_ne!(
                k, shared_canonical,
                "shared face {face:?} (canonical {k:?}) must be culled",
            );
        }
    }

    /// `uniform_block(2, 0.1, ...)` emits a 2³-cell cube. Each of the
    /// 6 cube faces tiles with 4 surface quads × 2 triangles = 8
    /// triangles, totalling 48 boundary faces. Spot-check that every
    /// face winds outward (its right-hand-rule normal points away from
    /// the cube center) — the per-face check catches any sign drift in
    /// the boundary-face winding convention against actual mesh
    /// geometry, not just topological pinning.
    #[test]
    fn uniform_block_2_cube_boundary_count_and_outward_winding() {
        let edge = 0.1;
        let mesh = HandBuiltTetMesh::uniform_block(2, edge, &canonical_field());
        let faces = mesh.boundary_faces();
        assert_eq!(
            faces.len(),
            48,
            "2³-cell cube must have 48 boundary triangles, got {}",
            faces.len(),
        );

        let positions = mesh.positions();
        let cube_center = Vec3::new(edge / 2.0, edge / 2.0, edge / 2.0);
        for face in faces {
            let v_a = positions[face[0] as usize];
            let v_b = positions[face[1] as usize];
            let v_c = positions[face[2] as usize];
            let normal = (v_b - v_a).cross(&(v_c - v_a));
            let face_centroid = (v_a + v_b + v_c) / 3.0;
            let outward_dir = face_centroid - cube_center;
            assert!(
                normal.dot(&outward_dir) > 0.0,
                "face {face:?} normal must point away from cube center {cube_center:?} \
                 (face centroid {face_centroid:?}, normal {normal:?})",
            );
        }
    }

    /// Topology of `Mesh::boundary_faces()`: the boundary of a solid tet
    /// mesh is a closed genus-0 surface, so its Euler characteristic
    /// `χ = V − E + F` is `2` and every boundary edge is shared by exactly
    /// two faces (watertight / closed-manifold). These invariants are what
    /// a well-formed boundary extraction guarantees — asserted here on the
    /// deterministic `2³`-cell cube (48 boundary triangles). No exact
    /// V/E/F counts are pinned (those are a mesh-resolution artifact); the
    /// topological invariants hold at any resolution.
    #[test]
    fn uniform_block_boundary_is_closed_genus_0_manifold() {
        let mesh = HandBuiltTetMesh::uniform_block(2, 0.1, &canonical_field());
        let faces = mesh.boundary_faces();

        // Unique boundary vertices.
        let mut verts: BTreeSet<VertexId> = BTreeSet::new();
        // Undirected edge (sorted vertex pair) -> incidence count.
        let mut edge_count: BTreeMap<[VertexId; 2], usize> = BTreeMap::new();
        for face in faces {
            verts.extend(face.iter().copied());
            for mut edge in [[face[0], face[1]], [face[1], face[2]], [face[2], face[0]]] {
                edge.sort_unstable();
                *edge_count.entry(edge).or_insert(0) += 1;
            }
        }

        // Closed-manifold: every boundary edge shared by exactly two faces
        // (a hole leaves an edge with 1 incidence; a non-manifold flap ≥ 3).
        for (edge, &count) in &edge_count {
            assert_eq!(
                count, 2,
                "boundary edge {edge:?} appears {count} times — boundary is not closed-manifold",
            );
        }

        // Euler characteristic of a closed genus-0 surface: V − E + F = 2,
        // written `V + F == E + 2` to stay in usize (a valid closed surface
        // always has V + F ≥ E + 2, so no subtraction underflow).
        let (v, e, f) = (verts.len(), edge_count.len(), faces.len());
        assert_eq!(
            v + f,
            e + 2,
            "Euler characteristic V − E + F ≠ 2 (V = {v}, E = {e}, F = {f}) — boundary is not a \
             topological sphere",
        );
    }
}
