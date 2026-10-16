//! Contact constraint row assembly — frictionless, elliptic, and pyramidal
//! friction cone handling.
//!
//! Populates contact rows in the unified `efc_*` arrays, then makes the
//! friction cones' R and regularized μ as MuJoCo's `mj_makeImpedance` does.
//! Called from [`super::assembly::assemble_unified_constraints`] during
//! Phase 3f.

use crate::constraint::impedance::{MJ_MINVAL, checked_solreffriction};
use crate::constraint::jacobian::compute_contact_jacobian;
use crate::types::{ConstraintType, Contact, Data, Model};

use super::assembly::finalize_constraint_row;

/// Assemble contact constraint rows (Phase 3f).
///
/// Handles three contact types:
/// - **Pyramidal** (cone == 0, μ ≥ 1e-10): 2*(dim-1) facet rows per contact
/// - **Elliptic** (cone == 1, μ ≥ 1e-10): dim rows per contact
/// - **Frictionless** (μ < 1e-10): dim rows per contact
///
/// Returns the first row and the contact index of each pyramidal or
/// elliptic contact, for [`regularize_cones`].
// Contact assembly takes the full per-iteration contact state (model, data, J/M, contact lists, output buffers).
#[allow(clippy::too_many_arguments)]
pub(super) fn assemble_contact_rows(
    model: &Model,
    data: &mut Data,
    nv: usize,
    row: &mut usize,
    contacts: &[Contact],
) -> Vec<(usize, usize)> {
    let mut cones: Vec<(usize, usize)> = Vec::new();

    for (ci, contact) in contacts.iter().enumerate() {
        let dim = contact.dim;
        let cj = compute_contact_jacobian(model, data, contact);

        let sr_normal = contact.solref;
        let sr_friction = checked_solreffriction(contact.solreffriction);
        let si = contact.solimp;
        // includemargin = margin - gap, computed at contact creation.
        let margin = contact.includemargin;
        let is_elliptic = dim >= 3 && model.cone == 1 && contact.mu[0] >= 1e-10;
        let is_pyramidal = dim >= 3 && model.cone == 0 && contact.mu[0] >= 1e-10;

        // Precompute contact bodyweight for DT-39
        let bw_contact = compute_contact_bodyweight(model, contact);

        if is_pyramidal {
            assemble_pyramidal_contact(
                model, data, nv, row, &cj, contact, ci, sr_normal, si, margin, bw_contact,
                &mut cones,
            );
        } else {
            // Elliptic or frictionless: emit dim rows.
            let ctype = if is_elliptic {
                ConstraintType::ContactElliptic
            } else {
                ConstraintType::ContactFrictionless
            };

            // §31: solreffriction selection for elliptic friction rows.
            let has_solreffriction =
                is_elliptic && (sr_friction[0] != 0.0 || sr_friction[1] != 0.0);

            let normal_row = *row;
            for r in 0..dim {
                for col in 0..nv {
                    data.efc_J[(*row, col)] = cj[(r, col)];
                }

                // pos: row 0 = signed distance, rows 1+ = 0.
                let pos = if r == 0 { -contact.depth } else { 0.0 };
                let margin_r = if r == 0 { margin } else { 0.0 };

                // §31: select solref for this row.
                let sr = if r > 0 && has_solreffriction {
                    sr_friction
                } else {
                    sr_normal
                };

                // row 0 = normal (translational), rows 1+ = friction (rotational)
                let bw_c = if r == 0 { bw_contact[0] } else { bw_contact[1] };
                finalize_constraint_row(
                    model, data, nv, row, sr, si, pos, margin_r, 0.0, ctype, dim, ci, contact.mu,
                    bw_c,
                );
            }

            // An elliptic contact's rows share the normal row's impedance: MuJoCo
            // gives a contact one position, its distance (`getposdim`,
            // `engine_core_constraint.c:1366-1397`). Their R is made from the
            // normal row's in `regularize_cones`.
            if is_elliptic {
                let imp_n = data.efc_imp[normal_row];
                data.efc_imp[normal_row + 1..normal_row + dim].fill(imp_n);
                cones.push((normal_row, ci));
            }
        }
    }

    cones
}

/// The friction cones' R and regularized μ, the second pass of MuJoCo's
/// `mj_makeImpedance` (`engine_core_constraint.c:1527-1568`), over each
/// pyramidal or elliptic contact `(first row, contact index)`, its rows' R
/// first made from each row's own diagApprox: `R[1] = R[0]/impratio`, and the
/// regularized cone's `μ = friction[0]·√(R[1]/R[0])`, written to the contact.
/// An elliptic contact's other rows take `R[j+1] = R[1]·friction[0]²/friction[j]²`,
/// a pyramidal contact's every facet `2·μ²·R[0]`. The rows' D and diagApprox
/// are then made from their R ([`super::assembly::assemble_unified_constraints`]).
pub(super) fn regularize_cones(model: &Model, data: &mut Data, cones: &[(usize, usize)]) {
    for &(i, ci) in cones {
        let (friction, dim) = (data.contacts[ci].mu, data.contacts[ci].dim);
        data.efc_R[i + 1] = data.efc_R[i] / model.impratio.max(MJ_MINVAL);
        let mu = friction[0] * (data.efc_R[i + 1] / data.efc_R[i]).sqrt();
        data.contacts[ci].regularized_mu = mu;
        if data.efc_type[i] == ConstraintType::ContactElliptic {
            for j in 1..dim - 1 {
                data.efc_R[i + j + 1] =
                    data.efc_R[i + 1] * friction[0] * friction[0] / (friction[j] * friction[j]);
            }
        } else {
            let rpy = 2.0 * mu * mu * data.efc_R[i];
            data.efc_R[i..i + 2 * (dim - 1)].fill(rpy);
        }
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Compute contact bodyweight for DT-39 diagApprox.
fn compute_contact_bodyweight(model: &Model, contact: &Contact) -> [f64; 2] {
    if let (Some(vi1), Some(vi2)) = (contact.flex_vertex, contact.flex_vertex2) {
        // Self-collision: both sides are flex vertices — use vertex bodies
        let b1 = model.flexvert_bodyid[vi1];
        let b2 = model.flexvert_bodyid[vi2];
        let w1t = if b1 < model.body_invweight0.len() {
            model.body_invweight0[b1][0]
        } else {
            0.0
        };
        let w2t = if b2 < model.body_invweight0.len() {
            model.body_invweight0[b2][0]
        } else {
            0.0
        };
        let w1r = if b1 < model.body_invweight0.len() {
            model.body_invweight0[b1][1]
        } else {
            0.0
        };
        let w2r = if b2 < model.body_invweight0.len() {
            model.body_invweight0[b2][1]
        } else {
            0.0
        };
        [(w1t + w2t).max(MJ_MINVAL), (w1r + w2r).max(MJ_MINVAL)]
    } else {
        let b1 = if contact.geom1 < model.geom_body.len() {
            model.geom_body[contact.geom1]
        } else {
            0
        };
        let b2 = if contact.geom2 < model.geom_body.len() {
            model.geom_body[contact.geom2]
        } else {
            0
        };
        let w1t = if b1 < model.body_invweight0.len() {
            model.body_invweight0[b1][0]
        } else {
            0.0
        };
        let w2t = if b2 < model.body_invweight0.len() {
            model.body_invweight0[b2][0]
        } else {
            0.0
        };
        let w1r = if b1 < model.body_invweight0.len() {
            model.body_invweight0[b1][1]
        } else {
            0.0
        };
        let w2r = if b2 < model.body_invweight0.len() {
            model.body_invweight0[b2][1]
        } else {
            0.0
        };
        // [translational, rotational]
        [(w1t + w2t).max(MJ_MINVAL), (w1r + w2r).max(MJ_MINVAL)]
    }
}

/// Assemble pyramidal friction cone facet rows for a single contact.
///
/// §32: Emits 2*(dim-1) facet rows. Each friction direction d produces two facets:
///   J_pos = J_normal + μ_d · J_friction_d
///   J_neg = J_normal - μ_d · J_friction_d
// Contact assembly takes the full per-iteration contact state; bundling would obscure the math against the published references.
#[allow(clippy::too_many_arguments)]
fn assemble_pyramidal_contact(
    model: &Model,
    data: &mut Data,
    nv: usize,
    row: &mut usize,
    cj: &nalgebra::DMatrix<f64>,
    contact: &Contact,
    ci: usize,
    sr_normal: [f64; 2],
    si: [f64; 5],
    margin: f64,
    bw_contact: [f64; 2],
    cones: &mut Vec<(usize, usize)>,
) {
    let dim = contact.dim;
    let n_facets = 2 * (dim - 1);
    let start_row = *row;

    let pos = -contact.depth;

    // Pyramidal facets combine normal + mu·friction → invweight includes
    // both normal (1²) and friction (mu²) components, using translational
    // body weight.
    let mu0 = contact.mu[0];
    let bw_py = (1.0 + mu0 * mu0) * bw_contact[0];

    for d in 1..dim {
        let mu_d = contact.mu[d - 1]; // friction coefficient for direction d

        // Positive facet: J_normal + μ_d · J_friction_d
        for col in 0..nv {
            data.efc_J[(*row, col)] = cj[(0, col)] + mu_d * cj[(d, col)];
        }
        finalize_constraint_row(
            model,
            data,
            nv,
            row,
            sr_normal,
            si,
            pos,
            margin,
            0.0,
            ConstraintType::ContactPyramidal,
            n_facets,
            ci,
            contact.mu,
            bw_py,
        );

        // Negative facet: J_normal - μ_d · J_friction_d
        for col in 0..nv {
            data.efc_J[(*row, col)] = cj[(0, col)] - mu_d * cj[(d, col)];
        }
        finalize_constraint_row(
            model,
            data,
            nv,
            row,
            sr_normal,
            si,
            pos,
            margin,
            0.0,
            ConstraintType::ContactPyramidal,
            n_facets,
            ci,
            contact.mu,
            bw_py,
        );
    }

    cones.push((start_row, ci));
}
