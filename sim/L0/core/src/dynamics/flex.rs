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

/// Pre-compute `flexedge_length` and `flexedge_velocity` Data fields.
///
/// Called after `mj_flex()` populates `flexvert_xpos`, before `mj_fwd_passive()`.
/// Replaces inline computation in all consumers (edge spring-damper, Newton penalty).
///
/// MuJoCo ref: `engine_passive.c` — `d->flexedge_length[e]`, `d->flexedge_velocity[e]`.
pub fn mj_flex_edge(model: &Model, data: &mut Data) {
    use nalgebra::Vector3;

    for e in 0..model.nflexedge {
        let [v0, v1] = model.flexedge_vert[e];
        let diff = data.flexvert_xpos[v1] - data.flexvert_xpos[v0];
        let dist = diff.norm();
        data.flexedge_length[e] = dist;

        if dist < 1e-10 {
            data.flexedge_velocity[e] = 0.0;
            continue;
        }

        let direction = diff / dist;

        // (§27F) Pinned vertices have dofadr=usize::MAX and zero velocity.
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
        data.flexedge_velocity[e] = (vel1 - vel0).dot(&direction);
    }
}
