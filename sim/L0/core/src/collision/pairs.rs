//! Collision pair enumeration — flex-vertex vs rigid, flex self-collision,
//! and flex-flex cross-object collision dispatch.
//!
//! Extracted from the collision module to separate pair enumeration logic
//! from broad-phase dispatch and contact parameter mixing.

use crate::types::{Data, FlexSelfCollide, Model};
use crate::types::{ENABLE_OVERRIDE, enabled};

use super::flex_collide::{
    make_contact_flex_rigid, mj_collide_flex_internal, mj_collide_flex_pair,
    mj_collide_flex_self_bvh, mj_collide_flex_self_narrow, mj_collide_flex_self_sap,
    narrowphase_sphere_geom,
};

/// Detect contacts between flex vertices and rigid geoms.
///
/// Each flex vertex is treated as a sphere of radius `flexvert_radius[vi]`.
/// Uses brute-force broadphase (O(V*G)) for simplicity — SAP integration
/// deferred to when nflexvert is large enough to warrant it.
/// No-op when nflexvert == 0.
pub(super) fn mj_collision_flex(model: &Model, data: &mut Data) {
    if model.nflexvert == 0 {
        return;
    }

    for vi in 0..model.nflexvert {
        let vpos = data.flexvert_xpos[vi];
        let flex_id = model.flexvert_flexid[vi];
        let radius = model.flexvert_radius[vi];

        // Skip pinned vertices (infinite mass = immovable)
        if model.flexvert_invmass[vi] <= 0.0 {
            continue;
        }

        // Per-flex bitmask values (invariant across the inner geom loop)
        let fcontype = model.flex_contype[flex_id];
        let fconaffinity = model.flex_conaffinity[flex_id];

        for gi in 0..model.ngeom {
            // Proper contype/conaffinity bitmask filtering (MuJoCo filterBitmask protocol).
            // Collision proceeds iff: (flex_contype & geom_conaffinity) != 0
            //                      || (geom_contype & flex_conaffinity) != 0
            let gcontype = model.geom_contype[gi];
            let gconaffinity = model.geom_conaffinity[gi];
            if (fcontype & gconaffinity) == 0 && (gcontype & fconaffinity) == 0 {
                continue;
            }

            // Skip geoms belonging to the same body as this vertex
            let vertex_body = model.flexvert_bodyid[vi];
            let geom_body = model.geom_body[gi];
            if geom_body == vertex_body {
                continue;
            }
            // Skip geoms on the vertex's parent body (node body for node-flex)
            if geom_body < model.nbody && model.body_parent[vertex_body] == geom_body {
                continue;
            }

            // S10: When override active, use full o_margin for narrowphase detection.
            // Non-override path uses flex_margin + geom_margin, matching the
            // includemargin calculation in make_contact_flex_rigid.
            let effective_margin = if enabled(model, ENABLE_OVERRIDE) {
                model.o_margin
            } else {
                model.flex_margin[flex_id] + model.geom_margin[gi]
            };

            // Narrowphase: vertex sphere vs rigid geom
            let geom_pos = data.geom_xpos[gi];
            let geom_mat = data.geom_xmat[gi];

            if let Some((depth, normal, contact_pos)) = narrowphase_sphere_geom(
                vpos,
                radius + effective_margin,
                gi,
                model,
                geom_pos,
                geom_mat,
            ) {
                let contact = make_contact_flex_rigid(model, vi, gi, contact_pos, normal, depth);
                data.contacts.push(contact);
                data.ncon += 1;
            }
        }
    }
}

/// Dispatch flex self-collision: internal (adjacent) + self (non-adjacent).
///
/// Called after `mj_collision_flex()` in the collision pipeline. Iterates all
/// flex objects, applies three conjunctive gate conditions, dispatches to
/// internal and self-collision paths independently.
pub(super) fn mj_collision_flex_self(model: &Model, data: &mut Data) {
    if model.nflex == 0 {
        return;
    }

    for f in 0..model.nflex {
        // Gate condition 1: skip rigid flexes (all vertices invmass == 0)
        if model.flex_rigid[f] {
            continue;
        }

        // Gate condition 2: self-bitmask check
        // contype & conaffinity against ITSELF (not cross-object)
        if (model.flex_contype[f] & model.flex_conaffinity[f]) == 0 {
            continue;
        }

        // Path 1: internal collision (adjacent elements)
        if model.flex_internal[f] {
            mj_collide_flex_internal(model, data, f);
        }

        // Path 2: self-collision (non-adjacent elements)
        match model.flex_selfcollide[f] {
            FlexSelfCollide::None => {}
            FlexSelfCollide::Narrow => {
                mj_collide_flex_self_narrow(model, data, f);
            }
            FlexSelfCollide::Bvh => {
                mj_collide_flex_self_bvh(model, data, f);
            }
            FlexSelfCollide::Sap => {
                mj_collide_flex_self_sap(model, data, f);
            }
            FlexSelfCollide::Auto => {
                if model.flex_dim[f] == 3 {
                    mj_collide_flex_self_bvh(model, data, f);
                } else {
                    mj_collide_flex_self_sap(model, data, f);
                }
            }
        }
    }
}

/// Dispatch flex-flex cross-object collision.
///
/// Iterates all flex pair combinations (f1 < f2), applies bitmask filter,
/// dispatches to element-element narrowphase with BVH midphase. No
/// `flex_rigid` check — rigid flexes still participate in cross-object
/// collision (MuJoCo behavior, EGT-4 verified).
pub(super) fn mj_collide_flex_flex(model: &Model, data: &mut Data) {
    if model.nflex < 2 {
        return;
    }

    for f1 in 0..model.nflex {
        let ct1 = model.flex_contype[f1];
        let ca1 = model.flex_conaffinity[f1];

        for f2 in (f1 + 1)..model.nflex {
            // filterBitmask: (ct1 & ca2) != 0 || (ct2 & ca1) != 0
            let ct2 = model.flex_contype[f2];
            let ca2 = model.flex_conaffinity[f2];
            if (ct1 & ca2) == 0 && (ct2 & ca1) == 0 {
                continue;
            }

            mj_collide_flex_pair(model, data, f1, f2);
        }
    }
}
