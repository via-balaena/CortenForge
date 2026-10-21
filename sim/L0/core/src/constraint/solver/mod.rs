//! Constraint solvers and shared infrastructure.
//!
//! The primal solver Newton and CG share (`primal`, with Newton's sparse
//! Hessian in `hessian`), PGS, noslip, and the shared post-solve operations
//! (qfrc recovery, friction pyramid decoding). Corresponds to MuJoCo's
//! `engine_solver.c`.

pub mod hessian;
pub mod noslip;
pub mod pgs;
pub mod primal;
pub mod qcqp;

use crate::linalg::mju_norm;
use crate::types::{ConstraintState, ConstraintType, Data, Model};

/// Decode pyramidal facet forces into physical normal + friction forces (§32.6).
///
/// Matches MuJoCo's `mju_decodePyramid`:
/// - `f_normal = Σ f_facet[k]` (sum of all facets)
/// - `f_friction[d] = μ[d] · (f_pos[d] - f_neg[d])` for each friction direction
/// - `dim` 1: `f_normal` is the contact's one row (`engine_util_misc.c:1144-1147`)
///
/// `facet_forces` is a slice of `2*(dim-1)` facet force values for one contact
/// (one at `dim` 1). `mu` is the 5-element friction array. `dim` is condim (NOT
/// number of facets).
///
/// Returns `(normal_force, friction_forces)` where `friction_forces` has `dim-1` elements.
#[must_use]
pub fn decode_pyramid(facet_forces: &[f64], mu: &[f64; 5], dim: usize) -> (f64, Vec<f64>) {
    let mut out = [0.0; 6];
    decode_pyramid_into(facet_forces, mu, dim, &mut out);
    (out[0], out[1..dim].to_vec())
}

/// [`decode_pyramid`] into `out`, as `mju_decodePyramid` writes its caller's
/// buffer: the normal force in `out[0]` and the friction forces in
/// `out[1..dim]`.
pub fn decode_pyramid_into(facet_forces: &[f64], mu: &[f64; 5], dim: usize, out: &mut [f64]) {
    if dim == 1 {
        out[0] = facet_forces[0];
        return;
    }
    let n_facets = 2 * (dim - 1);
    debug_assert_eq!(facet_forces.len(), n_facets);

    let mut f_normal = 0.0;
    for &f in &facet_forces[..n_facets] {
        f_normal += f;
    }
    out[0] = f_normal;

    for (d, pair) in facet_forces[..n_facets].chunks_exact(2).enumerate() {
        out[1 + d] = mu[d] * (pair[0] - pair[1]);
    }
}

/// Each row's state from its force alone, MuJoCo's `dualState`
/// (`engine_solver.c:233-310`); returns the rows not satisfied. Equality rows
/// are quadratic; a friction-loss row is linear at its bound, the side
/// opposite the primal's; a limit or non-elliptic contact row is quadratic
/// while its force is positive; an elliptic contact's rows take its zone in
/// the regular cone, the friction's norm summed as `mju_norm` sums it. Noslip
/// and PGS read it.
pub fn dual_state(data: &Data, state: &mut [ConstraintState]) -> usize {
    let (ne, nf, nefc) = (data.ne, data.nf, data.efc_type.len());
    let force = &data.efc_force;
    let mut nactive = ne + nf;
    state[..ne].fill(ConstraintState::Quadratic);
    for i in ne..ne + nf {
        let floss = data.efc_floss[i];
        state[i] = if force[i] <= -floss {
            ConstraintState::LinearPos
        } else if force[i] >= floss {
            ConstraintState::LinearNeg
        } else {
            ConstraintState::Quadratic
        };
    }
    let mut i = ne + nf;
    while i < nefc {
        if data.efc_type[i] != ConstraintType::ContactElliptic {
            if force[i] <= 0.0 {
                state[i] = ConstraintState::Satisfied;
            } else {
                state[i] = ConstraintState::Quadratic;
                nactive += 1;
            }
            i += 1;
            continue;
        }
        let dim = data.efc_dim[i];
        let mu = data.contacts[data.efc_id[i]].regularized_mu;
        let zone = elliptic_zone(&force.as_slice()[i..i + dim], mu, &data.efc_mu[i]);
        if zone != ConstraintState::Satisfied {
            nactive += dim;
        }
        state[i..i + dim].fill(zone);
        i += dim;
    }
    nactive
}

/// The zone of an elliptic contact's forces (its `dim` rows, normal first)
/// in its regular cone of friction `mu`, `dualState`'s
/// (`engine_solver.c:268-303`): satisfied inside the cone, quadratic inside
/// its dual, in the cone between. The friction forces are scaled by their
/// coefficients and their norm summed as `mju_norm` sums it.
fn elliptic_zone(force: &[f64], mu: f64, friction: &[f64; 5]) -> ConstraintState {
    let n = force[0] / mu;
    let mut f = [0.0; 5];
    for j in 1..force.len() {
        f[j - 1] = force[j] / friction[j - 1];
    }
    let t = mju_norm(&f[..force.len() - 1]);
    if mu * n >= t {
        ConstraintState::Satisfied
    } else if n + mu * t <= 0.0 {
        ConstraintState::Quadratic
    } else {
        ConstraintState::Cone
    }
}

/// Map efc_force → qfrc_constraint via J^T · efc_force.
pub fn compute_qfrc_constraint_from_efc(model: &Model, data: &mut Data) {
    let nv = model.nv;
    data.qfrc_constraint.fill(0.0);
    let nefc = data.efc_type.len();
    for i in 0..nefc {
        let force = data.efc_force[i];
        if force.abs() > 0.0 {
            for col in 0..nv {
                data.qfrc_constraint[col] += data.efc_J[(i, col)] * force;
            }
        }
    }
}

/// Extract friction loss forces from efc_force into qfrc_frictionloss.
///
/// After the solver runs, friction loss constraint rows contain the
/// friction loss forces. Extract them via J^T · efc_force for the
/// friction loss rows only.
pub fn extract_qfrc_frictionloss(data: &mut Data, nv: usize) {
    data.qfrc_frictionloss.fill(0.0);
    let nefc = data.efc_type.len();
    for i in 0..nefc {
        if matches!(data.efc_type[i], ConstraintType::FrictionLoss) {
            let force = data.efc_force[i];
            for col in 0..nv {
                data.qfrc_frictionloss[col] += data.efc_J[(i, col)] * force;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::elliptic_zone;
    use crate::types::ConstraintState;

    /// At condim 6 the friction's norm is `mju_norm`'s, which sums in four
    /// lanes: on these forces it is one ulp below the sequential sum
    /// (0.7995754347083057 against …058), so a normal force equal to it
    /// leaves the contact satisfied in MuJoCo, where the sequential sum puts
    /// it in the cone.
    #[test]
    fn an_elliptic_zone_sums_the_friction_as_mujoco() {
        let friction = [
            1.878_710_267_871_435_3e-9,
            0.547_744_465_709_557_8,
            0.564_368_293_133_386_7,
            0.144_171_098_973_528_56,
            5.317_202_465_801_857e-9,
        ];
        let mut force = vec![0.799_575_434_708_305_7];
        force.extend(friction);
        let sequential = friction.iter().map(|x| x * x).sum::<f64>().sqrt();
        assert!(sequential > force[0], "the sums differ here");
        assert_eq!(
            elliptic_zone(&force, 1.0, &[1.0; 5]),
            ConstraintState::Satisfied
        );
        force[0] = 0.7;
        assert_eq!(elliptic_zone(&force, 1.0, &[1.0; 5]), ConstraintState::Cone);
        force[0] = -0.9;
        assert_eq!(
            elliptic_zone(&force, 1.0, &[1.0; 5]),
            ConstraintState::Quadratic
        );
    }
}
