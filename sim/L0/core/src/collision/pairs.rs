//! Collision pair enumeration — flex self-collision dispatch.
//!
//! Extracted from the collision module to separate pair enumeration logic
//! from broad-phase dispatch and contact parameter mixing.

use crate::types::{Data, FlexSelfCollide, Model};

use super::flex_collide::{
    mj_collide_flex_internal, mj_collide_flex_self_bvh, mj_collide_flex_self_narrow,
    mj_collide_flex_self_sap,
};

/// Dispatch flex self-collision: internal (adjacent) + self (non-adjacent).
///
/// Called after the body–flex and explicit pairs in the collision pipeline. Iterates all
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
