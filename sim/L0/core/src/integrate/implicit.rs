//! Tendon implicit stiffness/damping helpers (DT-35).
//!
//! These functions accumulate non-diagonal tendon K/D contributions into the
//! mass matrix for implicit integration. Used by both the implicit acceleration
//! path (`forward/acceleration.rs`) and the Newton constraint solver
//! (`constraint/mod.rs`).

use nalgebra::{DMatrix, DVector};

use crate::types::{Data, Model};

/// Check if all DOFs affected by a tendon's Jacobian belong to sleeping trees (§16.5a').
pub fn tendon_all_dofs_sleeping(model: &Model, data: &Data, t: usize) -> bool {
    tendon_all_dofs_sleeping_fields(model, &data.ten_J[t], &data.tree_awake)
}

/// Field-level variant of `tendon_all_dofs_sleeping` that avoids borrowing all of `Data`.
/// Used by `accumulate_tendon_kd` where `data.scratch_m_impl` is mutably borrowed.
fn tendon_all_dofs_sleeping_fields(
    model: &Model,
    ten_j: &DVector<f64>,
    tree_awake: &[bool],
) -> bool {
    for dof in 0..model.nv {
        if ten_j[dof] != 0.0 && tree_awake[model.dof_treeid[dof]] {
            return false; // At least one target DOF is awake
        }
    }
    true
}

/// Return the effective stiffness for implicit treatment (DT-35).
///
/// Returns `k` when the tendon is outside its deadband (spring engaged),
/// `0.0` when inside (spring disengaged). This gates the `h²·K` LHS
/// modification: no phantom stiffness inside the deadband.
///
/// **Note on exact boundary:** At `length == lower` or `length == upper`,
/// returns `0.0`. The displacement is also `0.0` at the boundary, so no
/// spring force exists. If velocity moves the tendon outside the deadband,
/// the spring activates in the next step (one-step delay, consistent with
/// linearization at the current state).
#[inline]
pub fn tendon_active_stiffness(k: f64, length: f64, range: [f64; 2]) -> f64 {
    if k <= 0.0 {
        return 0.0;
    }
    let [lower, upper] = range;
    if length >= lower && length <= upper {
        0.0
    } else {
        k
    }
}

/// Accumulate non-diagonal tendon K/D into a mass matrix (DT-35).
///
/// For each tendon with nonzero stiffness or damping, adds the rank-1
/// outer product `(h²·k_active + h·b) · J^T · J` to `matrix`. Uses
/// deadband-aware `k_active` (zero inside deadband, `k` outside).
///
/// Shared by `mj_fwd_acceleration_implicit` (Step 2) and
/// `build_m_impl_for_newton` (Step 2b). Both call sites must produce
/// identical mass matrix modifications — factoring this out guarantees it.
///
/// **Sleep guard:** Skips tendons whose target DOFs are all sleeping,
/// matching the guards in `mj_fwd_passive` and `mjd_passive_vel`.
pub fn accumulate_tendon_kd(
    matrix: &mut DMatrix<f64>,
    model: &Model,
    ten_j: &[DVector<f64>],
    ten_length: &[f64],
    tree_awake: &[bool],
    h: f64,
    sleep_enabled: bool,
) {
    let h2 = h * h;
    for t in 0..model.ntendon {
        if sleep_enabled && tendon_all_dofs_sleeping_fields(model, &ten_j[t], tree_awake) {
            continue;
        }
        let kt = model.tendon_stiffness[t];
        let bt = model.tendon_damping[t];
        if kt <= 0.0 && bt <= 0.0 {
            continue;
        }
        let j = &ten_j[t];
        let k_active = tendon_active_stiffness(kt, ten_length[t], model.tendon_lengthspring[t]);
        let scale = h2 * k_active + h * bt;
        // Defensive: skip if scale is non-positive. For valid models (k ≥ 0,
        // b ≥ 0) this is unreachable when the above guard passes, but protects
        // against pathological negative parameters that would break SPD.
        if scale <= 0.0 {
            continue;
        }
        // Rank-1 outer product: (h²·k_active + h·b) · J^T · J
        //
        // Sparsity skip `j[r] == 0.0`: for fixed tendons, Jacobian entries
        // are exact MJCF coefficients (parsed floats), so zero entries are
        // exactly 0.0. For spatial tendons, entries are computed from 3D
        // geometry and may be near-zero (e.g. 1e-17) rather than exact
        // zero — but the contribution of such entries is O(ε²) per matrix
        // element, which is negligible (well below f64 precision). This
        // matches MuJoCo's own sparse outer-product loops.
        for r in 0..model.nv {
            if j[r] == 0.0 {
                continue;
            }
            for c in 0..model.nv {
                if j[c] == 0.0 {
                    continue;
                }
                matrix[(r, c)] += scale * j[r] * j[c];
            }
        }
    }
}
