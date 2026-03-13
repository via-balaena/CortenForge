//! Contact constraint row assembly — frictionless, elliptic, and pyramidal
//! friction cone handling.
//!
//! Populates contact rows in the unified `efc_*` arrays and performs
//! pyramidal R-scaling post-processing. Called from
//! [`super::assembly::assemble_unified_constraints`] during Phase 3f.

use nalgebra::DVector;

use crate::constraint::impedance::MJ_MINVAL;
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
/// Returns pyramidal ranges `(start_row, n_facets, contact_idx)` for R-scaling
/// post-processing.
#[allow(clippy::too_many_arguments)]
pub(super) fn assemble_contact_rows(
    model: &Model,
    data: &mut Data,
    nv: usize,
    row: &mut usize,
    qacc_smooth: &DVector<f64>,
    contacts: &[Contact],
) -> Vec<(usize, usize, usize)> {
    let mut pyramidal_ranges: Vec<(usize, usize, usize)> = Vec::new();

    for (ci, contact) in contacts.iter().enumerate() {
        let dim = contact.dim;
        let cj = compute_contact_jacobian(model, data, contact);

        let sr_normal = contact.solref;
        let si = contact.solimp;
        // includemargin = margin - gap, computed at contact creation.
        let margin = contact.includemargin;
        let is_elliptic = dim >= 3 && model.cone == 1 && contact.mu[0] >= 1e-10;
        let is_pyramidal = dim >= 3 && model.cone == 0 && contact.mu[0] >= 1e-10;

        // Precompute contact bodyweight for DT-39
        let bw_contact = compute_contact_bodyweight(model, contact);

        if is_pyramidal {
            assemble_pyramidal_contact(
                model,
                data,
                nv,
                row,
                qacc_smooth,
                &cj,
                contact,
                ci,
                sr_normal,
                si,
                margin,
                bw_contact,
                &mut pyramidal_ranges,
            );
        } else {
            // Elliptic or frictionless: emit dim rows.
            let ctype = if is_elliptic {
                ConstraintType::ContactElliptic
            } else {
                ConstraintType::ContactFrictionless
            };

            // §31: solreffriction selection for elliptic friction rows.
            let has_solreffriction = is_elliptic
                && (contact.solreffriction[0] != 0.0 || contact.solreffriction[1] != 0.0);

            for r in 0..dim {
                for col in 0..nv {
                    data.efc_J[(*row, col)] = cj[(r, col)];
                }

                // pos: row 0 = signed distance, rows 1+ = 0.
                let pos = if r == 0 { -contact.depth } else { 0.0 };
                let margin_r = if r == 0 { margin } else { 0.0 };

                // §31: select solref for this row.
                let sr = if r > 0 && has_solreffriction {
                    contact.solreffriction
                } else {
                    sr_normal
                };

                // vel: J_row · qvel
                let mut vel = 0.0;
                for col in 0..nv {
                    vel += cj[(r, col)] * data.qvel[col];
                }

                // row 0 = normal (translational), rows 1+ = friction (rotational)
                let bw_c = if r == 0 { bw_contact[0] } else { bw_contact[1] };
                finalize_constraint_row(
                    model,
                    data,
                    nv,
                    row,
                    qacc_smooth,
                    sr,
                    si,
                    pos,
                    margin_r,
                    vel,
                    0.0,
                    ctype,
                    dim,
                    ci,
                    contact.mu,
                    bw_c,
                );
            }
        }
    }

    pyramidal_ranges
}

/// Post-process R scaling for pyramidal contacts (§32).
///
/// MuJoCo's mj_makeImpedance computes R per-row from each row's own diagApprox,
/// then overrides all facet rows with Rpy = 2 · μ_reg² · R[first_facet].
pub(super) fn postprocess_pyramidal_r_scaling(
    model: &Model,
    data: &mut Data,
    contacts: &[Contact],
    pyramidal_ranges: &[(usize, usize, usize)],
) {
    for &(start_row, n_facets, ci) in pyramidal_ranges {
        let contact = &contacts[ci];

        // Use R already computed by finalize_constraint_row for the first facet row.
        let r_first_facet = data.efc_R[start_row];

        // μ_reg = friction[0] · √(1/impratio)
        let mu_reg = contact.mu[0] * (1.0 / model.impratio).sqrt();
        let rpy = (2.0 * mu_reg * mu_reg * r_first_facet).max(MJ_MINVAL);

        for row_idx in start_row..start_row + n_facets {
            data.efc_R[row_idx] = rpy;
            data.efc_D[row_idx] = 1.0 / rpy;
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
#[allow(clippy::too_many_arguments)]
fn assemble_pyramidal_contact(
    model: &Model,
    data: &mut Data,
    nv: usize,
    row: &mut usize,
    qacc_smooth: &DVector<f64>,
    cj: &nalgebra::DMatrix<f64>,
    contact: &Contact,
    ci: usize,
    sr_normal: [f64; 2],
    si: [f64; 5],
    margin: f64,
    bw_contact: [f64; 2],
    pyramidal_ranges: &mut Vec<(usize, usize, usize)>,
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
        let mut vel = 0.0;
        for col in 0..nv {
            vel += data.efc_J[(*row, col)] * data.qvel[col];
        }
        finalize_constraint_row(
            model,
            data,
            nv,
            row,
            qacc_smooth,
            sr_normal,
            si,
            pos,
            margin,
            vel,
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
        let mut vel = 0.0;
        for col in 0..nv {
            vel += data.efc_J[(*row, col)] * data.qvel[col];
        }
        finalize_constraint_row(
            model,
            data,
            nv,
            row,
            qacc_smooth,
            sr_normal,
            si,
            pos,
            margin,
            vel,
            0.0,
            ConstraintType::ContactPyramidal,
            n_facets,
            ci,
            contact.mu,
            bw_py,
        );
    }

    pyramidal_ranges.push((start_row, n_facets, ci));
}
