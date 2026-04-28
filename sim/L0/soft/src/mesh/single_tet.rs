//! One-tet mesh — `SoftScene::one_tet_cube()` constructs this.
//!
//! Four vertices hardcoded at the canonical decimeter-edge tet per
//! spec §2: `v_0 = (0, 0, 0)`, `v_1 = (0.1, 0, 0)`, `v_2 = (0, 0.1, 0)`,
//! `v_3 = (0, 0, 0.1)`. Edge length `L = 0.1` m (soft-robotics scale).
//! Phase A proper replaces this with a general `TetMesh`.

use super::{Mesh, MeshAdjacency, QualityMetrics, TetId, VertexId, materials_from_field, quality};
use crate::Vec3;
use crate::material::{MaterialField, NeoHookean};

/// Single-tetrahedron mesh. Four vertices, one tet, trivial adjacency.
#[derive(Clone, Debug)]
pub struct SingleTetMesh {
    vertices: [Vec3; 4],
    adj: MeshAdjacency,
    q: QualityMetrics,
    material_cache: Vec<NeoHookean>,
}

impl SingleTetMesh {
    /// Canonical decimeter-edge tet per walking-skeleton spec §2,
    /// sampled against `field` at the tet centroid (Decision K).
    ///
    /// Vertices in reference configuration; later Phase A proper
    /// replaces this with an `SDF → TetMesh` pipeline.
    #[must_use]
    pub fn new(field: &MaterialField) -> Self {
        let vertices = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.1, 0.0, 0.0),
            Vec3::new(0.0, 0.1, 0.0),
            Vec3::new(0.0, 0.0, 0.1),
        ];
        let tets: [[VertexId; 4]; 1] = [[0, 1, 2, 3]];
        let q = quality::compute_metrics(&vertices, &tets);
        let material_cache = materials_from_field(&vertices, &tets, field);
        Self {
            vertices,
            adj: MeshAdjacency,
            q,
            material_cache,
        }
    }
}

impl Mesh for SingleTetMesh {
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

    fn materials(&self) -> &[NeoHookean] {
        &self.material_cache
    }

    // Ch 00 §02 mesh claim 3: same vertex count, same tet count, and
    // same per-tet vertex indices. Positions deliberately excluded —
    // those are the change-detection signal, not structural identity.
    fn equals_structurally(&self, other: &dyn Mesh) -> bool {
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
