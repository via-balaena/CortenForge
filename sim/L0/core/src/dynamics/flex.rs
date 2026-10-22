//! Flex vertex world positions, element boxes and hierarchies, and edge
//! lengths and Jacobians, as MuJoCo's `mj_flex` (`engine_core_smooth.c:
//! 548-713`).
//!
//! Each flex vertex has an associated body (created by `process_flex_bodies`);
//! its `xpos` — computed by standard forward kinematics — IS the vertex world
//! position.

use crate::constraint::impedance::MJ_MINVAL;
use crate::types::{DISABLE_MIDPHASE, Data, Model, disabled};

/// `mju_min` (`engine_util_misc.c:1286-1292`): the first unless it is above
/// the second (so a tie, `-0.0` with `0.0` too, is the first).
fn min(a: f64, b: f64) -> f64 {
    if a <= b { a } else { b }
}

/// `mju_max` (`:1296-1302`): the first unless it is below the second.
fn max(a: f64, b: f64) -> f64 {
    if a >= b { a } else { b }
}

/// The flex vertices' world positions, their bodies' (each flex vertex has
/// its own body, whose `xpos` is the vertex's; a pinned vertex's is static);
/// then every element's box (`flexelem_aabb`) and, unless the midphase is
/// disabled, each flex's hierarchy's boxes (`bvh_aabb_dyn`), as MuJoCo's
/// `mj_flex` (`engine_core_smooth.c:610-662`).
pub fn mj_flex(model: &Model, data: &mut Data) {
    for i in 0..model.nflexvert {
        let body_id = model.flexvert_bodyid[i];
        data.flexvert_xpos[i] = data.xpos[body_id];
    }

    // Each element's box over its vertices, its half-sizes grown by its
    // flex's radius; a rigid flex's too.
    for (e, aabb) in data.flexelem_aabb.iter_mut().enumerate() {
        let f = model.flexelem_flexid[e];
        let adr = model.flexelem_dataadr[e];
        let vertices = &model.flexelem_data[adr..adr + model.flexelem_datanum[e]];
        let first = data.flexvert_xpos[vertices[0]];
        let (mut lo, mut hi) = (first, first);
        for &v in &vertices[1..] {
            for j in 0..3 {
                let value = data.flexvert_xpos[v][j];
                lo[j] = min(lo[j], value);
                hi[j] = max(hi[j], value);
            }
        }
        let radius = model.flex_radius[f];
        *aabb = [
            0.5 * (hi[0] + lo[0]),
            0.5 * (hi[1] + lo[1]),
            0.5 * (hi[2] + lo[2]),
            0.5 * (hi[0] - lo[0]) + radius,
            0.5 * (hi[1] - lo[1]) + radius,
            0.5 * (hi[2] - lo[2]) + radius,
        ];
    }

    // Each hierarchy's leaves take their elements' boxes, then its inner
    // nodes their children's together, the last node first (a parent comes
    // before its children; `mj_updateDynamicBVH`, `:489-533`).
    if disabled(model, DISABLE_MIDPHASE) {
        return;
    }
    for (f, &bvhadr) in model.flex_bvhadr.iter().enumerate() {
        let Some(bvhadr) = bvhadr else {
            continue;
        };
        let dynamic = bvhadr - model.nbvhstatic;
        let num = model.flex_bvhnum[f];
        for i in 0..num {
            if let Some(e) = model.bvh_nodeid[bvhadr + i] {
                data.bvh_aabb_dyn[dynamic + i] = data.flexelem_aabb[model.flex_elemadr[f] + e];
            }
        }
        for i in (0..num).rev() {
            if model.bvh_nodeid[bvhadr + i].is_some() {
                continue;
            }
            let [Some(c1), Some(c2)] = model.bvh_child[bvhadr + i] else {
                continue;
            };
            let (a1, a2) = (
                data.bvh_aabb_dyn[dynamic + c1],
                data.bvh_aabb_dyn[dynamic + c2],
            );
            let mut aabb = [0.0; 6];
            for k in 0..3 {
                let lo = min(a1[k] - a1[k + 3], a2[k] - a2[k + 3]);
                let hi = max(a1[k] + a1[k + 3], a2[k] + a2[k + 3]);
                aabb[k] = 0.5 * (hi + lo);
                aabb[k + 3] = 0.5 * (hi - lo);
            }
            data.bvh_aabb_dyn[dynamic + i] = aabb;
        }
    }
}

/// The edges' lengths (`flexedge_length`) and sparse Jacobians
/// (`flexedge_J`), as MuJoCo's `mj_flex` (`engine_core_smooth.c:671-713`).
///
/// Called after `mj_flex()` populates `flexvert_xpos`. Every Jacobian is
/// cleared first; a rigid flex is then skipped, its lengths staying as they
/// were (0 from the reset), and an edge's Jacobian is made only when an edge
/// equality, an edge stiffness, an edge damping or the flex's damping needs
/// it. The velocities, `J·qvel`, are the velocity stage's
/// (`mj_fwd_velocity`), as MuJoCo's `mj_fwdVelocity`.
#[allow(non_snake_case)]
pub fn mj_flex_edge(model: &Model, data: &mut Data) {
    use nalgebra::Vector3;

    data.flexedge_J.fill(0.0);
    for f in 0..model.nflex {
        // A rigid flex's edges make no force: MuJoCo's `mj_flex` skips the
        // flex (`engine_core_smooth.c:677-680`; MuJoCo's interpolated flexes
        // too, which sim-core does not build).
        if model.flex_rigid[f] {
            continue;
        }
        // No built-in passive force needs the Jacobian (`:682-686`).
        // sim-core's bending damping reads the vertices, not the edges.
        let skip_jacobian = model.flex_edgeequality.get(f) != Some(&true)
            && model.flex_edgedamping[f] == 0.0
            && model.flex_edgestiffness[f] == 0.0
            && model.flex_damping[f] == 0.0;

        // Iterate over edges belonging to this flex
        let edge_start = model.flex_edgeadr[f];
        let edge_count = model.flex_edgenum[f];

        for e in edge_start..edge_start + edge_count {
            let [v0, v1] = model.flexedge_vert[e];
            let diff = data.flexvert_xpos[v1] - data.flexvert_xpos[v0];
            let dist = diff.norm();
            data.flexedge_length[e] = dist;

            // Unit edge direction at any length: the x axis below `mjMINVAL`,
            // as MuJoCo's `mju_normalize3` (`engine_util_blas.c:120-135`).
            let vec = if dist < MJ_MINVAL {
                Vector3::x()
            } else {
                diff / dist
            };

            if skip_jacobian {
                continue;
            }

            // Compute J_edge = vec^T * jacdif
            // For free vertices: jacdif = [-I₃, +I₃]
            // J_edge = [-vec^T, +vec^T]
            let rowadr = model.flexedge_J_rowadr[e];
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
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{max, min};

    /// `mju_min` and `mju_max` keep the first on a tie, the sign of a zero
    /// included (`engine_util_misc.c:1286-1302`).
    #[test]
    fn min_and_max_keep_the_first_on_a_tie() {
        assert_eq!(min(-0.0, 0.0).to_bits(), (-0.0_f64).to_bits());
        assert_eq!(min(0.0, -0.0).to_bits(), 0.0_f64.to_bits());
        assert_eq!(max(-0.0, 0.0).to_bits(), (-0.0_f64).to_bits());
        assert_eq!(max(0.0, -0.0).to_bits(), 0.0_f64.to_bits());
        assert_eq!(min(1.0, 2.0).to_bits(), 1.0_f64.to_bits());
        assert_eq!(max(1.0, 2.0).to_bits(), 2.0_f64.to_bits());
    }
}
