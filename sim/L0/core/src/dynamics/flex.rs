//! Flex vertex world-position synchronization.
//!
//! Syncs flex vertex positions from body FK results. Each flex vertex has an
//! associated body (created by `process_flex_bodies`); its `xpos` — computed
//! by standard forward kinematics — IS the vertex world position.

use crate::types::{Data, Model};

/// Compute flex vertex world positions from body FK.
///
/// Each flex vertex has an associated body (created by process_flex_bodies).
/// The body's xpos (computed by standard FK) IS the vertex world position.
/// Pinned vertices (no DOFs) use xpos directly from their static body.
pub fn mj_flex(model: &Model, data: &mut Data) {
    for i in 0..model.nflexvert {
        let body_id = model.flexvert_bodyid[i];
        data.flexvert_xpos[i] = data.xpos[body_id];
    }
}

/// Pre-compute sparse edge Jacobian (`flexedge_J`), edge lengths
/// (`flexedge_length`), and edge velocities (`flexedge_velocity`).
///
/// Called after `mj_flex()` populates `flexvert_xpos`, before
/// `mj_fwd_passive()`. Length and J share the direction normalization
/// step (no double computation).
///
/// MuJoCo ref: `mj_flex()` in `engine_core_smooth.c` (J computation),
/// `mj_fwdVelocity()` in `engine_forward.c` (velocity = J * qvel).
#[allow(non_snake_case)]
pub fn mj_flex_edge(model: &Model, data: &mut Data) {
    use nalgebra::Vector3;

    for f in 0..model.nflex {
        // Per-flex skip conditions (evaluated once per flex, not per edge)
        let is_rigid = model.flex_rigid[f];
        let skip_jacobian = is_rigid
            || (model.flex_edgestiffness[f] == 0.0
                && model.flex_edgedamping[f] == 0.0
                && model.flex_bend_damping[f] == 0.0
                && model.flex_edge_solref[f] == [0.0, 0.0]);

        // Iterate over edges belonging to this flex
        let edge_start = model.flex_edgeadr[f];
        let edge_count = model.flex_edgenum[f];

        for e in edge_start..edge_start + edge_count {
            let [v0, v1] = model.flexedge_vert[e];
            let diff = data.flexvert_xpos[v1] - data.flexvert_xpos[v0];
            let dist = diff.norm();

            // Always compute length — even for rigid flex (MuJoCo does).
            data.flexedge_length[e] = dist;

            if dist < 1e-10 {
                data.flexedge_velocity[e] = 0.0;
                // J values remain zero (from reset)
                continue;
            }

            // Rigid flex: length computed above, skip J + velocity
            if is_rigid {
                continue;
            }

            let vec = diff / dist; // unit edge direction

            if skip_jacobian {
                // Still compute velocity inline (cheaper than J path)
                let dof0 = model.flexvert_dofadr[v0];
                let dof1 = model.flexvert_dofadr[v1];
                let vel0 = if dof0 == usize::MAX {
                    Vector3::zeros()
                } else {
                    Vector3::new(data.qvel[dof0], data.qvel[dof0 + 1], data.qvel[dof0 + 2])
                };
                let vel1 = if dof1 == usize::MAX {
                    Vector3::zeros()
                } else {
                    Vector3::new(data.qvel[dof1], data.qvel[dof1 + 1], data.qvel[dof1 + 2])
                };
                data.flexedge_velocity[e] = (vel1 - vel0).dot(&vec);
                continue;
            }

            // Compute J_edge = vec^T * jacdif
            // For free vertices: jacdif = [-I₃, +I₃]
            // J_edge = [-vec^T, +vec^T]
            let rowadr = model.flexedge_J_rowadr[e];
            let rownnz = model.flexedge_J_rownnz[e];
            let b0 = model.flexvert_bodyid[v0];
            let b1 = model.flexvert_bodyid[v1];
            let dn0 = model.body_dof_num[b0];
            let dn1 = model.body_dof_num[b1];

            // Body 0 contribution: -vec^T (for free vertex, 3 entries)
            for k in 0..dn0 {
                data.flexedge_J[rowadr + k] = -vec[k];
            }
            // Body 1 contribution: +vec^T (for free vertex, 3 entries)
            for k in 0..dn1 {
                data.flexedge_J[rowadr + dn0 + k] = vec[k];
            }

            // Velocity = J * qvel (sparse dot product)
            let mut vel = 0.0;
            for j in 0..rownnz {
                let col = model.flexedge_J_colind[rowadr + j];
                vel += data.flexedge_J[rowadr + j] * data.qvel[col];
            }
            data.flexedge_velocity[e] = vel;
        }
    }
}
