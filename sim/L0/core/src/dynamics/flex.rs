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
