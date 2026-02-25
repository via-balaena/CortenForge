//! State validation checks for forward dynamics pipeline.
//!
//! Validates qpos, qvel, and qacc for NaN/Inf/divergence before and after
//! pipeline stages. On detection, fires a warning and auto-resets to qpos0
//! (unless `DISABLE_AUTORESET` is set).
//!
//! Corresponds to MuJoCo's `mj_checkPos`, `mj_checkVel`, `mj_checkAcc`.

use crate::types::flags::{disabled, enabled};
use crate::types::validation::is_bad;
use crate::types::warning::{Warning, mj_warning};
use crate::types::{DISABLE_AUTORESET, Data, ENABLE_SLEEP, Model};

/// Validate position coordinates (NOT sleep-aware — scans all nq elements).
///
/// On bad value: fires `Warning::BadQpos`, auto-resets unless `DISABLE_AUTORESET`.
/// Position validation scans ALL `nq` elements because: (1) externally-set bad
/// qpos can appear on sleeping bodies, (2) sleep indexing is per-DOF (nv), not
/// per-position-element (nq).
#[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
pub fn mj_check_pos(model: &Model, data: &mut Data) {
    for i in 0..model.nq {
        if is_bad(data.qpos[i]) {
            mj_warning(data, Warning::BadQpos, i as i32);
            if !disabled(model, DISABLE_AUTORESET) {
                data.reset(model);
            }
            // Re-set warning after reset (reset zeroes all warnings).
            data.warnings[Warning::BadQpos as usize].count += 1;
            data.warnings[Warning::BadQpos as usize].last_info = i as i32;
            return;
        }
    }
}

/// Validate velocity coordinates (sleep-aware — only checks awake DOFs).
///
/// On bad value: fires `Warning::BadQvel`, auto-resets unless `DISABLE_AUTORESET`.
#[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
pub fn mj_check_vel(model: &Model, data: &mut Data) {
    let sleep_filter = enabled(model, ENABLE_SLEEP) && data.nv_awake < model.nv;
    let nv = if sleep_filter {
        data.nv_awake
    } else {
        model.nv
    };

    for j in 0..nv {
        let i = if sleep_filter {
            data.dof_awake_ind[j]
        } else {
            j
        };
        if is_bad(data.qvel[i]) {
            mj_warning(data, Warning::BadQvel, i as i32);
            if !disabled(model, DISABLE_AUTORESET) {
                data.reset(model);
            }
            data.warnings[Warning::BadQvel as usize].count += 1;
            data.warnings[Warning::BadQvel as usize].last_info = i as i32;
            return;
        }
    }
}

/// Validate acceleration (sleep-aware, re-runs forward after reset).
///
/// On bad value: fires `Warning::BadQacc`, auto-resets unless `DISABLE_AUTORESET`.
/// Unlike check_pos/check_vel, this re-runs `forward()` after reset because it
/// executes AFTER the forward pass — derived quantities need recomputation.
#[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
pub fn mj_check_acc(model: &Model, data: &mut Data) {
    let sleep_filter = enabled(model, ENABLE_SLEEP) && data.nv_awake < model.nv;
    let nv = if sleep_filter {
        data.nv_awake
    } else {
        model.nv
    };

    for j in 0..nv {
        let i = if sleep_filter {
            data.dof_awake_ind[j]
        } else {
            j
        };
        if is_bad(data.qacc[i]) {
            mj_warning(data, Warning::BadQacc, i as i32);
            if !disabled(model, DISABLE_AUTORESET) {
                data.reset(model);
            }
            data.warnings[Warning::BadQacc as usize].count += 1;
            data.warnings[Warning::BadQacc as usize].last_info = i as i32;
            // Unlike check_pos/check_vel, re-run forward after reset to
            // recompute derived quantities from the reset state.
            if !disabled(model, DISABLE_AUTORESET) {
                if let Err(e) = data.forward(model) {
                    tracing::error!(
                        "mj_forward() failed after auto-reset (model's \
                         qpos0 produces non-recoverable error): {e:?}"
                    );
                }
            }
            return;
        }
    }
}
