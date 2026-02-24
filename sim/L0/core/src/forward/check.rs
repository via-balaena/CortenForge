//! State validation checks for forward dynamics pipeline.
//!
//! Validates qpos, qvel, and qacc for NaN/Inf before and after pipeline stages.
//! Corresponds to MuJoCo's `mj_checkPos`, `mj_checkVel`, `mj_checkAcc`.

use crate::types::{Data, Model, StepError};

/// Validate position coordinates.
///
/// Returns `Err(StepError::InvalidPosition)` if any qpos element is NaN, Inf,
/// or exceeds 1e10 in magnitude (indicating numerical blow-up).
///
/// Unlike MuJoCo which silently resets to qpos0, this returns an error so
/// users can decide how to handle the situation.
pub fn mj_check_pos(model: &Model, data: &Data) -> Result<(), StepError> {
    for i in 0..model.nq {
        if !data.qpos[i].is_finite() || data.qpos[i].abs() > 1e10 {
            return Err(StepError::InvalidPosition);
        }
    }
    Ok(())
}

/// Validate velocity coordinates.
///
/// Returns `Err(StepError::InvalidVelocity)` if any qvel element is NaN, Inf,
/// or exceeds 1e10 in magnitude (indicating numerical blow-up).
///
/// Unlike MuJoCo which silently zeros velocity, this returns an error so
/// users can decide how to handle the situation.
pub fn mj_check_vel(model: &Model, data: &Data) -> Result<(), StepError> {
    for i in 0..model.nv {
        if !data.qvel[i].is_finite() || data.qvel[i].abs() > 1e10 {
            return Err(StepError::InvalidVelocity);
        }
    }
    Ok(())
}

/// Validate acceleration.
///
/// Returns `Err(StepError::InvalidAcceleration)` if any qacc element is NaN.
/// This typically indicates a singular mass matrix or other numerical issues.
pub fn mj_check_acc(model: &Model, data: &Data) -> Result<(), StepError> {
    for i in 0..model.nv {
        if !data.qacc[i].is_finite() {
            return Err(StepError::InvalidAcceleration);
        }
    }
    Ok(())
}
