//! State validation checks for forward dynamics pipeline.
//!
//! Validates qpos, qvel, and qacc for NaN/Inf/divergence before and after
//! pipeline stages. On detection, fires a warning and auto-resets to qpos0
//! (unless `DISABLE_AUTORESET` is set).
//!
//! Corresponds to MuJoCo's `mj_checkPos`, `mj_checkVel`, `mj_checkAcc`.
//!
//! Before any of them, every `Result` entry point runs [`check_step_inputs`],
//! which refuses a timestep and a `Data` that MuJoCo does not check.

use crate::types::flags::{disabled, enabled};
use crate::types::validation::is_bad;
use crate::types::warning::{Warning, mj_warning};
use crate::types::{DISABLE_AUTORESET, Data, ENABLE_SLEEP, EqualityType, Model, StepError};

/// The check every `Result` entry point (`step`, `step1`, `step2`, `forward`,
/// `forward_skip`, `integrate`) runs before any work: the timestep, then the
/// shape of `data`.
///
/// # Errors
///
/// [`StepError::InvalidTimestep`] if `model.timestep` is not positive and
/// finite; [`check_data_shape`]'s error.
pub fn check_step_inputs(model: &Model, data: &Data) -> Result<(), StepError> {
    if model.timestep <= 0.0 || !model.timestep.is_finite() {
        return Err(StepError::InvalidTimestep);
    }
    check_data_shape(model, data)
}

/// The check the calls that run the position stage make before any work
/// (`step`, `step1`, `forward`, `forward_skip` from `MjStage::None`, the
/// transition finite differences, the init-sleep reset): MuJoCo 3.5.0's
/// position stage raises an error on an active tendon equality with sleep
/// enabled (`mj_wakeEquality`, `engine_sleep.c:398-400`). `step2`,
/// `integrate`, `forward_skip` past the position stage and the inverse
/// finite differences run, as MuJoCo's `mj_step2`, `mj_Euler`,
/// `mj_forwardSkip` and `mjd_inverseFD` do.
///
/// # Errors
///
/// [`StepError::TendonEqualityWithSleep`] naming the first such equality.
pub fn check_tendon_equality_sleep(model: &Model) -> Result<(), StepError> {
    if model.enableflags & ENABLE_SLEEP != 0
        && let Some(eq) = (0..model.neq)
            .find(|&eq| model.eq_active[eq] && model.eq_type[eq] == EqualityType::Tendon)
    {
        return Err(StepError::TendonEqualityWithSleep { eq });
    }
    Ok(())
}

/// Refuses a `data` made by a model of other dimensions, or one whose caller
/// resized one of the arrays below.
///
/// Compares the inputs (`qpos`, `qvel`, `act`, `ctrl`, `qfrc_applied`,
/// `xfrc_applied`, `mocap_pos`, `mocap_quat`), then derived arrays that cover
/// every dimension [`Model::make_data`] sizes from. A resized array outside
/// that list (`cvel` or `qacc_warmstart`, say) is not caught.
///
/// # Errors
///
/// [`StepError::DataShapeMismatch`] naming the first array, in that order,
/// whose length differs.
pub fn check_data_shape(model: &Model, data: &Data) -> Result<(), StepError> {
    let lengths = [
        ("qpos", data.qpos.len(), model.nq),
        ("qvel", data.qvel.len(), model.nv),
        ("act", data.act.len(), model.na),
        ("ctrl", data.ctrl.len(), model.nu),
        ("qfrc_applied", data.qfrc_applied.len(), model.nv),
        ("xfrc_applied", data.xfrc_applied.len(), model.nbody),
        ("mocap_pos", data.mocap_pos.len(), model.nmocap),
        ("mocap_quat", data.mocap_quat.len(), model.nmocap),
        ("xpos", data.xpos.len(), model.nbody),
        ("xanchor", data.xanchor.len(), model.njnt),
        ("geom_xpos", data.geom_xpos.len(), model.ngeom),
        ("site_xpos", data.site_xpos.len(), model.nsite),
        ("ten_length", data.ten_length.len(), model.ntendon),
        ("wrap_xpos", data.wrap_xpos.len(), 2 * model.nwrap),
        ("eq_violation", data.eq_violation.len(), 6 * model.neq),
        ("flexvert_xpos", data.flexvert_xpos.len(), model.nflexvert),
        (
            "flexedge_length",
            data.flexedge_length.len(),
            model.nflexedge,
        ),
        (
            "flexedge_J",
            data.flexedge_J.len(),
            model.flexedge_J_colind.len(),
        ),
        ("qLD_data", data.qLD_data.len(), model.qLD_nnz),
        ("sensordata", data.sensordata.len(), model.nsensordata),
        ("history", data.history.len(), model.nhistory),
        ("tree_asleep", data.tree_asleep.len(), model.ntree),
        ("plugin_state", data.plugin_state.len(), model.npluginstate),
        ("plugin_data", data.plugin_data.len(), model.nplugin),
    ];
    lengths
        .into_iter()
        .find(|&(_, actual, expected)| actual != expected)
        .map_or(Ok(()), |(field, actual, expected)| {
            Err(StepError::DataShapeMismatch {
                field,
                expected,
                actual,
            })
        })
}

/// Validate position coordinates (NOT sleep-aware — scans all nq elements).
///
/// On bad value: fires `Warning::BadQpos`, auto-resets unless `DISABLE_AUTORESET`.
/// Position validation scans ALL `nq` elements because: (1) externally-set bad
/// qpos can appear on sleeping bodies, (2) sleep indexing is per-DOF (nv), not
/// per-position-element (nq).
// `nq`/`nv` model dimensions are i32 in MuJoCo's spec but always non-negative and bounded by realistic model sizes.
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
// `nq`/`nv` model dimensions are i32 in MuJoCo's spec but always non-negative and bounded by realistic model sizes.
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
// `nq`/`nv` model dimensions are i32 in MuJoCo's spec but always non-negative and bounded by realistic model sizes.
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
            if !disabled(model, DISABLE_AUTORESET)
                && let Err(e) = data.forward(model)
            {
                log::error!(
                    "mj_forward() failed after auto-reset (model's \
                         qpos0 produces non-recoverable error): {e:?}"
                );
            }
            return;
        }
    }
}
