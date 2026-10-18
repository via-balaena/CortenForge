//! Tendon implicit stiffness/damping helpers (DT-35).
//!
//! These functions accumulate non-diagonal tendon K/D contributions into the
//! mass matrix for implicit integration. Used by both the implicit acceleration
//! path (`forward/acceleration.rs`) and the Newton constraint solver
//! (`constraint/mod.rs`).

use nalgebra::{DMatrix, DVector};

use crate::types::flags::disabled;
use crate::types::{DISABLE_DAMPER, DISABLE_SPRING, Model};

/// A tendon's Jacobian entry `j[dof]` as the implicit spring-damper takes
/// it: 0 at a sleeping dof under the sleep filter (`sleep_filter`, as
/// [`crate::island::constraint_sleep_filter`]). The matrix and the forces it
/// builds then couple no sleeping dof to an awake one (the mass matrix
/// couples dofs within one tree, and a tree's dofs sleep together), so the
/// awake dofs' solve is over the awake dofs alone, as MuJoCo's implicit
/// integrators factor (`mj_implicitSkip`); a tendon with no entry on an
/// awake dof adds nothing.
#[must_use]
pub fn isd_tendon_j(
    model: &Model,
    tree_awake: &[bool],
    sleep_filter: bool,
    j: &DVector<f64>,
    dof: usize,
) -> f64 {
    if sleep_filter && !tree_awake[model.dof_treeid[dof]] {
        0.0
    } else {
        j[dof]
    }
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
    if k == 0.0 {
        return 0.0;
    }
    let [lower, upper] = range;
    if length >= lower && length <= upper {
        0.0
    } else {
        k
    }
}

/// The stiffness implicitspringdamper puts in its implicit solve at dof `i`:
/// `implicit_stiffness[i]`, or 0 with springs disabled, when the passive pass
/// applies no spring either.
pub fn isd_stiffness(model: &Model, i: usize) -> f64 {
    if disabled(model, DISABLE_SPRING) {
        0.0
    } else {
        model.implicit_stiffness[i]
    }
}

/// The damping implicitspringdamper puts in its implicit solve at dof `i`:
/// `implicit_damping[i]`, or 0 with dampers disabled.
pub fn isd_damping(model: &Model, i: usize) -> f64 {
    if disabled(model, DISABLE_DAMPER) {
        0.0
    } else {
        model.implicit_damping[i]
    }
}

/// Tendon `t`'s stiffness in implicitspringdamper's solve, as
/// [`isd_stiffness`].
pub fn isd_tendon_stiffness(model: &Model, t: usize) -> f64 {
    if disabled(model, DISABLE_SPRING) {
        0.0
    } else {
        model.tendon_stiffness[t]
    }
}

/// Tendon `t`'s damping in implicitspringdamper's solve, as [`isd_damping`].
pub fn isd_tendon_damping(model: &Model, t: usize) -> f64 {
    if disabled(model, DISABLE_DAMPER) {
        0.0
    } else {
        model.tendon_damping[t]
    }
}

/// Accumulate non-diagonal tendon K/D into a mass matrix (DT-35).
///
/// For each tendon with nonzero stiffness or damping, adds the rank-1
/// outer product `(h²·k_active + h·b) · J^T · J` to `matrix`. Uses
/// deadband-aware `k_active` (zero inside deadband, `k` outside).
///
/// Shared by `mj_fwd_acceleration_implicit` (Step 2) and
/// `build_m_impl` (Step 2b). Both call sites must produce
/// identical mass matrix modifications — factoring this out guarantees it.
///
/// **Sleep:** each tendon's row is taken at the awake dofs only
/// ([`isd_tendon_j`]), as the other ImplicitSpringDamper paths take it.
pub fn accumulate_tendon_kd(
    matrix: &mut DMatrix<f64>,
    model: &Model,
    ten_j: &[DVector<f64>],
    ten_length: &[f64],
    tree_awake: &[bool],
    h: f64,
    sleep_filter: bool,
) {
    let h2 = h * h;
    for t in 0..model.ntendon {
        let kt = isd_tendon_stiffness(model, t);
        let bt = isd_tendon_damping(model, t);
        if kt == 0.0 && bt == 0.0 {
            continue;
        }
        let j = |dof| isd_tendon_j(model, tree_awake, sleep_filter, &ten_j[t], dof);
        let k_active = tendon_active_stiffness(kt, ten_length[t], model.tendon_lengthspring[t]);
        // A negative stiffness or damping is folded in as given, as a joint's
        // is: the passive pass leaves these forces to this solve, and a matrix
        // they make indefinite fails its factorization (`CholeskyFailed`).
        let scale = h2 * k_active + h * bt;
        if scale == 0.0 {
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
            if j(r) == 0.0 {
                continue;
            }
            for c in 0..model.nv {
                if j(c) == 0.0 {
                    continue;
                }
                matrix[(r, c)] += scale * j(r) * j(c);
            }
        }
    }
}
