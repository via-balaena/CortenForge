//! `SoftScene` — skeleton scene constructors + initial-state bundle.
//!
//! `one_tet_cube()` returns the canonical decimeter-edge tet per spec
//! §2 bundled with its initial state (rest configuration, zero velocity).
//! `SceneInitial` is the bootstrap position/velocity pair the first
//! Newton step consumes.

use sim_ml_chassis::Tensor;

use crate::mesh::{Mesh, SingleTetMesh};

/// Scene constructors. Skeleton ships one constructor for the 1-tet
/// cube; additional scenes (multi-tet, contact walls) land in Phase B+.
pub struct SoftScene;

impl SoftScene {
    /// Canonical 1-tet scene per walking-skeleton spec §2: decimeter edge
    /// (`L = 0.1` m), silicone-class density (`ρ = 1030` kg/m³ — carried
    /// on `SolverConfig`, not here), four vertices at the canonical
    /// right-handed axis-aligned placement.
    ///
    /// Returns the mesh bundled with its initial state: rest-configuration
    /// positions and zero velocity, flattened as length-12 `Tensor<f64>`
    /// in vertex-major + xyz-inner layout (DOF `i` is vertex `i / 3`'s
    /// `i % 3` component).
    #[must_use]
    pub fn one_tet_cube() -> (SingleTetMesh, SceneInitial) {
        let mesh = SingleTetMesh::new();
        let mut x_prev_flat = [0.0f64; 12];
        for (v, pos) in mesh.positions().iter().enumerate() {
            x_prev_flat[3 * v] = pos.x;
            x_prev_flat[3 * v + 1] = pos.y;
            x_prev_flat[3 * v + 2] = pos.z;
        }
        let initial = SceneInitial {
            x_prev: Tensor::from_slice(&x_prev_flat, &[12]),
            v_prev: Tensor::zeros(&[12]),
        };
        (mesh, initial)
    }
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
