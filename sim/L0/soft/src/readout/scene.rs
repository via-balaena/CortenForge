//! `SoftScene` — skeleton scene constructors + scene-config bundles.
//!
//! `one_tet_cube()` returns the canonical decimeter-edge tet per spec
//! §2 bundled with its boundary conditions and initial state. The
//! 3-tuple shape `(impl Mesh, BoundaryConditions, SceneInitial)` is
//! Phase 2's canonical scene-emission contract — Decision K + L of
//! [`phase_2_multi_element_fem_scope.md`](../../../../docs/todo/phase_2_multi_element_fem_scope.md).
//! Multi-tet scenes (`HandBuiltTetMesh::two_isolated_tets`,
//! `two_tet_shared_face`) land in Phase 2 commit 2.

use sim_ml_chassis::Tensor;

use crate::mesh::{Mesh, SingleTetMesh, VertexId};

/// Scene constructors. Skeleton ships one constructor for the 1-tet
/// cube; multi-tet siblings (`n_isolated_tets`, `two_tet_shared_face`)
/// land in Phase 2 commit 2 as new methods on the same `impl` block.
pub struct SoftScene;

impl SoftScene {
    /// Canonical 1-tet scene per walking-skeleton spec §2: decimeter edge
    /// (`L = 0.1` m), silicone-class density (`ρ = 1030` kg/m³ — carried
    /// on `SolverConfig`, not here), four vertices at the canonical
    /// right-handed axis-aligned placement.
    ///
    /// Returns the mesh bundled with its boundary conditions
    /// (`v_0..v_2` Dirichlet-pinned, `v_3` `+ẑ`-loaded — the Stage-1
    /// default per Decision L; Stage-2 tests build their own
    /// [`BoundaryConditions`] inline) and its initial state
    /// (rest-configuration positions, zero velocity, flattened to
    /// length-12 `Tensor<f64>` in vertex-major + xyz-inner layout —
    /// DOF `i` is vertex `i / 3`'s `i % 3` component).
    #[must_use]
    pub fn one_tet_cube() -> (SingleTetMesh, BoundaryConditions, SceneInitial) {
        let mesh = SingleTetMesh::new();
        let mut x_prev_flat = [0.0f64; 12];
        for (v, pos) in mesh.positions().iter().enumerate() {
            x_prev_flat[3 * v] = pos.x;
            x_prev_flat[3 * v + 1] = pos.y;
            x_prev_flat[3 * v + 2] = pos.z;
        }
        let bc = BoundaryConditions {
            pinned_vertices: vec![0, 1, 2],
            loaded_vertices: vec![(3, LoadAxis::AxisZ)],
        };
        let initial = SceneInitial {
            x_prev: Tensor::from_slice(&x_prev_flat, &[12]),
            v_prev: Tensor::zeros(&[12]),
        };
        (mesh, bc, initial)
    }
}

/// Boundary conditions for a soft-body scene.
///
/// Carries the Dirichlet pinned-vertex set (those whose displacement is
/// fixed at the rest configuration) plus the load-application list
/// (vertices that receive external traction, paired with the load axis
/// describing which θ component drives which DOF).
///
/// The solver consumes this at construction time (Phase 2 commit 3
/// wires it through `CpuNewtonSolver::new`); commit 1 ships the type +
/// scene emission only.
#[derive(Clone, Debug)]
pub struct BoundaryConditions {
    /// Vertex IDs whose displacement is pinned to their rest position
    /// (full Dirichlet — all three xyz DOFs constrained).
    pub pinned_vertices: Vec<VertexId>,
    /// Vertex IDs that receive external traction, paired with the load
    /// axis describing which θ component drives which DOF.
    pub loaded_vertices: Vec<(VertexId, LoadAxis)>,
}

/// How a θ component maps to a vertex's DOFs.
///
/// Stage 1 (`+ẑ` magnitude) pairs each loaded vertex with one θ scalar
/// driving its `z` DOF; Stage 2 (full traction vector) pairs each
/// loaded vertex with three θ components driving its xyz. Mixed-axis
/// scenes are out of scope for Phase 2.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadAxis {
    /// Single `+ẑ` traction component — Stage-1 θ broadcasts this
    /// magnitude to the loaded vertex's `z` DOF.
    AxisZ,
    /// Full `(t_x, t_y, t_z)` traction vector — Stage-2 θ supplies all
    /// three components to the loaded vertex's xyz DOFs.
    FullVector,
}

/// Initial state for a forward-pass rollout. Bootstrap position and
/// velocity the first Newton step consumes.
#[derive(Debug)]
pub struct SceneInitial {
    /// Rest-configuration vertex positions, flattened to shape `[3 * N]`.
    pub x_prev: Tensor<f64>,
    /// Rest-configuration vertex velocities, flattened to shape `[3 * N]`.
    pub v_prev: Tensor<f64>,
}
