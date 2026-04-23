//! `SoftScene` — skeleton scene constructors + initial-state bundle.
//!
//! `one_tet_cube()` returns the canonical decimeter-edge tet per spec
//! §2. `SceneInitial` bundles the initial position and velocity pair
//! the first Newton step consumes.

use sim_ml_chassis::Tensor;

use crate::mesh::SingleTetMesh;

/// Scene constructors. Skeleton ships one constructor for the 1-tet
/// cube; additional scenes (multi-tet, contact walls) land in Phase B+.
pub struct SoftScene;

impl SoftScene {
    /// Canonical 1-tet scene: decimeter edge (L = 0.1 m) per spec §2,
    /// four vertices at the canonical right-handed placement, ρ = 1030
    /// kg/m³, silicone-like elastic constants.
    #[must_use]
    pub fn one_tet_cube() -> SingleTetMesh {
        unimplemented!("skeleton phase 2")
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
