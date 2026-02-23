//! Constraint solver dispatch and shared infrastructure.
//!
//! Routes to the configured solver algorithm (PGS, CG, Newton) and provides
//! shared post-solve operations (Delassus regularization, qfrc recovery,
//! friction pyramid decoding). Corresponds to MuJoCo's `engine_solver.c`.

pub mod cg;
pub mod pgs;
pub mod primal;

use crate::types::{ConstraintType, Data, Model};

/// Decode pyramidal facet forces into physical normal + friction forces (§32.6).
///
/// Matches MuJoCo's `mju_decodePyramid`:
/// - `f_normal = Σ f_facet[k]` (sum of all facets)
/// - `f_friction[d] = μ[d] · (f_pos[d] - f_neg[d])` for each friction direction
///
/// `facet_forces` is a slice of `2*(dim-1)` facet force values for one contact.
/// `mu` is the 5-element friction array. `dim` is condim (NOT number of facets).
///
/// Returns `(normal_force, friction_forces)` where `friction_forces` has `dim-1` elements.
#[must_use]
pub fn decode_pyramid(facet_forces: &[f64], mu: &[f64; 5], dim: usize) -> (f64, Vec<f64>) {
    let n_facets = 2 * (dim - 1);
    debug_assert_eq!(facet_forces.len(), n_facets);

    let mut f_normal = 0.0;
    for &f in &facet_forces[..n_facets] {
        f_normal += f;
    }

    let mut f_friction = Vec::with_capacity(dim - 1);
    for d in 0..(dim - 1) {
        let f_pos = facet_forces[2 * d];
        let f_neg = facet_forces[2 * d + 1];
        f_friction.push(mu[d] * (f_pos - f_neg));
    }

    (f_normal, f_friction)
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
