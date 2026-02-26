//! State reset (§41 S8f).
//!
//! `mj_reset_data` restores `Data` to the model's initial state (`qpos0`).
//! Corresponds to MuJoCo's `mj_resetData()` in `engine_io.c`.

use crate::types::{Data, Model};

/// Reset simulation state to model defaults.
///
/// Delegates to [`Data::reset`] — see that method for the full field inventory.
pub fn mj_reset_data(model: &Model, data: &mut Data) {
    data.reset(model);
}
