//! One-tet mesh — `SoftScene::one_tet_cube()` constructs this.
//!
//! Four vertices hardcoded at the canonical decimeter-edge tet per
//! spec §2. Phase B populates constructors and Phase A proper replaces
//! with a general `TetMesh`.

use super::{Mesh, MeshAdjacency, QualityMetrics, TetId, VertexId};
use crate::Vec3;

/// Single-tetrahedron mesh. Four vertices, one tet, trivial adjacency.
#[derive(Clone, Debug)]
pub struct SingleTetMesh {
    vertices: [Vec3; 4],
    adj: MeshAdjacency,
    q: QualityMetrics,
}

impl Mesh for SingleTetMesh {
    fn n_tets(&self) -> usize {
        unimplemented!("skeleton phase 2")
    }

    fn n_vertices(&self) -> usize {
        unimplemented!("skeleton phase 2")
    }

    fn tet_vertices(&self, _tet: TetId) -> [VertexId; 4] {
        unimplemented!("skeleton phase 2")
    }

    fn positions(&self) -> &[Vec3] {
        unimplemented!("skeleton phase 2")
    }

    fn adjacency(&self) -> &MeshAdjacency {
        unimplemented!("skeleton phase 2")
    }

    fn quality(&self) -> &QualityMetrics {
        unimplemented!("skeleton phase 2")
    }

    fn equals_structurally(&self, _other: &dyn Mesh) -> bool {
        unimplemented!("skeleton phase 2")
    }
}
