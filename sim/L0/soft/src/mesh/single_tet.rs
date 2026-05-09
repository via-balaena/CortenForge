//! One-tet mesh — `SoftScene::one_tet_cube()` constructs this.
//!
//! Four vertices hardcoded at the canonical decimeter-edge tet per
//! spec §2: `v_0 = (0, 0, 0)`, `v_1 = (0.1, 0, 0)`, `v_2 = (0, 0.1, 0)`,
//! `v_3 = (0, 0, 0.1)`. Edge length `L = 0.1` m (soft-robotics scale).
//! Phase A proper replaces this with a general `TetMesh`.
//!
//! Generic over `M: BuildableFromField` so the same mesh shape carries
//! either NH or Yeoh per-tet materials. `SingleTetMesh` (no parameter)
//! defaults to `SingleTetMesh<NeoHookean>` for back-compat with all
//! Phase 4 NH consumers.

use super::{
    Mesh, MeshAdjacency, QualityMetrics, TetId, VertexId, boundary_faces_from_topology,
    interface_flags_from_field, materials_from_field, quality,
};
use crate::Vec3;
use crate::material::{BuildableFromField, MaterialField, NeoHookean, Yeoh};

/// Single-tetrahedron mesh. Four vertices, one tet, trivial adjacency.
#[derive(Clone, Debug)]
pub struct SingleTetMesh<M: BuildableFromField = NeoHookean> {
    vertices: [Vec3; 4],
    adj: MeshAdjacency,
    q: QualityMetrics,
    material_cache: Vec<M>,
    interface_flags: Vec<bool>,
    boundary_faces: Vec<[VertexId; 3]>,
}

/// Internal constructor body shared by `new` (NH) and `new_yeoh`. The
/// generic `M: BuildableFromField` bound dispatches to the right
/// material sampling path; explicit per-`M` entry points avoid the
/// type-inference papercut that bare `SingleTetMesh::new` triggers.
fn build<M: BuildableFromField>(field: &MaterialField) -> SingleTetMesh<M> {
    let vertices = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.1, 0.0, 0.0),
        Vec3::new(0.0, 0.1, 0.0),
        Vec3::new(0.0, 0.0, 0.1),
    ];
    let tets: [[VertexId; 4]; 1] = [[0, 1, 2, 3]];
    let q = quality::compute_metrics(&vertices, &tets);
    let material_cache: Vec<M> = materials_from_field(&vertices, &tets, field);
    let interface_flags = interface_flags_from_field(&vertices, &tets, field);
    let boundary_faces = boundary_faces_from_topology(&tets);
    SingleTetMesh {
        vertices,
        adj: MeshAdjacency,
        q,
        material_cache,
        interface_flags,
        boundary_faces,
    }
}

impl SingleTetMesh<NeoHookean> {
    /// Canonical decimeter-edge tet per walking-skeleton spec §2,
    /// sampled against the NH-variant `field` at the tet centroid
    /// (Decision K). Vertices in reference configuration; later
    /// Phase A proper replaces this with an `SDF → TetMesh` pipeline.
    ///
    /// # Panics
    ///
    /// Panics if `field` was built via [`MaterialField::from_yeoh_fields`]
    /// — call [`SingleTetMesh::<Yeoh>::new_yeoh`] instead.
    #[must_use]
    pub fn new(field: &MaterialField) -> Self {
        build(field)
    }
}

impl SingleTetMesh<Yeoh> {
    /// Canonical decimeter-edge tet sampled against the Yeoh-variant
    /// `field` at the tet centroid. Mirror of [`SingleTetMesh::new`]
    /// for the Yeoh material model (arc memo D10).
    ///
    /// # Panics
    ///
    /// Panics if `field` was built via NH constructors
    /// ([`MaterialField::from_fields`] / [`MaterialField::uniform`] /
    /// [`MaterialField::skeleton_default`]) — call
    /// [`SingleTetMesh::<NeoHookean>::new`] instead.
    #[must_use]
    pub fn new_yeoh(field: &MaterialField) -> Self {
        build(field)
    }
}

impl<M: BuildableFromField> Mesh<M> for SingleTetMesh<M> {
    fn n_tets(&self) -> usize {
        1
    }

    fn n_vertices(&self) -> usize {
        4
    }

    fn tet_vertices(&self, tet: TetId) -> [VertexId; 4] {
        assert!(tet == 0, "SingleTetMesh has only tet 0, got {tet}");
        [0, 1, 2, 3]
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

    fn materials(&self) -> &[M] {
        &self.material_cache
    }

    fn interface_flags(&self) -> &[bool] {
        &self.interface_flags
    }

    fn boundary_faces(&self) -> &[[VertexId; 3]] {
        &self.boundary_faces
    }

    // Ch 00 §02 mesh claim 3: same vertex count, same tet count, and
    // same per-tet vertex indices. Positions deliberately excluded —
    // those are the change-detection signal, not structural identity.
    fn equals_structurally(&self, other: &dyn Mesh<M>) -> bool {
        if self.n_tets() != other.n_tets() {
            return false;
        }
        if self.n_vertices() != other.n_vertices() {
            return false;
        }
        // SingleTetMesh has exactly one tet; a general multi-tet
        // `TetMesh` impl will iterate all tets in Phase A proper.
        self.n_tets() == 0 || self.tet_vertices(0) == other.tet_vertices(0)
    }
}
