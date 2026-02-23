//! Keyframe (named state snapshot) type.
//!
//! Extracted from `mujoco_pipeline.rs` — Phase 1, structural refactor.

use nalgebra::{DVector, UnitQuaternion, Vector3};

/// A named state snapshot for resetting simulation state.
///
/// All vectors are sized to match the model dimensions (nq, nv, na, nu,
/// nmocap). Unspecified fields in the MJCF `<key>` are filled with model
/// defaults at build time.
#[derive(Debug, Clone, PartialEq)]
pub struct Keyframe {
    /// Keyframe name (from MJCF `name` attribute). Empty string if unnamed.
    pub name: String,
    /// Simulation time. Default: 0.0.
    pub time: f64,
    /// Joint positions (length nq). Default: model.qpos0.
    pub qpos: DVector<f64>,
    /// Joint velocities (length nv). Default: zeros.
    pub qvel: DVector<f64>,
    /// Actuator activations (length na). Default: zeros.
    pub act: DVector<f64>,
    /// Control signals (length nu). Default: zeros.
    pub ctrl: DVector<f64>,
    /// Mocap body positions (length nmocap). Default: body_pos for each
    /// mocap body.
    pub mpos: Vec<Vector3<f64>>,
    /// Mocap body quaternions (length nmocap). Default: body_quat for each
    /// mocap body.
    pub mquat: Vec<UnitQuaternion<f64>>,
}
