//! cf-osim — OpenSim (`.osim`) reader for the musculoskeletal-builder arc.
//!
//! Status: **leg-region A1** — [`parse_leg_chain`] reads the gait2392 chain
//! (pelvis → femur → tibia, hip welded at neutral) into `cf_msk_lib`'s
//! source-agnostic [`Model`](cf_msk_lib::Model); [`oracle`] computes the
//! OpenSim-geometry ground-truth moment arms from that `Model`. Emission lives in
//! the sibling `cf-mjcf-emit` crate. See
//! `docs/msk_builder/03_phases/leg_region/recon.md`.
//!
//! Goal: does a converted gait2392 knee reproduce the OpenSim oracle's joint
//! center + moment-arm curves within tolerance? The oracle is validated against
//! real OpenSim 4.6 to ~0.3 mm RMSE (`tests/opensim_cross_check.rs`).
//!
//! This `lib.rs` also ships the **moment-arm extraction** helpers (read from a
//! loaded MJCF's tendon Jacobian) — the quantity the whole program is graded on.

pub mod oracle;
pub mod osim;
pub mod scorecard;
pub mod xml;

pub use osim::parse_leg_chain;

use sim_core::{MjJointType, Model};

/// One moment-arm sample at a single joint angle.
///
/// `moment_arm` follows the standard biomechanics / MuJoCo convention
/// `r = -∂L/∂θ`, extracted from the tendon Jacobian as `-ten_J[tendon][dof]`.
/// A positive moment arm means increasing the joint coordinate *shortens* the
/// tendon (the muscle acts to increase the coordinate).
#[derive(Debug, Clone, Copy)]
pub struct MomentArmSample {
    /// Joint coordinate (rad for a hinge, m for a slide).
    pub angle: f64,
    /// Tendon length at this configuration (m).
    pub length: f64,
    /// Moment arm of the tendon about the swept joint DOF (m): `-ten_J[t][dof]`.
    pub moment_arm: f64,
}

/// Resolve a joint name to its `jnt_id`. Spike-grade: panics if absent.
pub fn joint_id(model: &Model, name: &str) -> usize {
    model
        .jnt_name
        .iter()
        .position(|n| n.as_deref() == Some(name))
        // gait2392 always defines this joint; absence = a corrupt asset.
        .unwrap_or_else(|| panic!("joint '{name}' not found in model"))
}

/// Resolve a tendon name to its `tendon_id`. Spike-grade: panics if absent.
pub fn tendon_id(model: &Model, name: &str) -> usize {
    model
        .tendon_name
        .iter()
        .position(|n| n.as_deref() == Some(name))
        // gait2392 always defines this tendon; absence = a corrupt asset.
        .unwrap_or_else(|| panic!("tendon '{name}' not found in model"))
}

/// Sweep a 1-DOF (hinge or slide) joint through `angles`, returning the moment
/// arm of `tendon_idx` about that joint at each configuration.
///
/// Each sample runs a fresh forward pass (`Data::forward`) from a clean state
/// with only the swept coordinate set, so samples are independent and order
/// does not matter. The moment arm is read from `ten_J`, the analytic tendon
/// Jacobian the engine already validates against MuJoCo 3.4.0 + finite
/// difference (see `sim/L0/tests/integration/spatial_tendons.rs`).
pub fn moment_arm_sweep(
    model: &Model,
    tendon_idx: usize,
    jnt_id: usize,
    angles: &[f64],
) -> Vec<MomentArmSample> {
    assert!(
        matches!(
            model.jnt_type[jnt_id],
            MjJointType::Hinge | MjJointType::Slide
        ),
        "moment_arm_sweep expects a 1-DOF hinge/slide joint, got {:?}",
        model.jnt_type[jnt_id]
    );
    let qadr = model.jnt_qpos_adr[jnt_id];
    let dof = model.jnt_dof_adr[jnt_id];
    angles
        .iter()
        .map(|&angle| {
            let mut data = model.make_data();
            data.qpos[qadr] = angle;
            data.forward(model).expect("forward pass failed");
            MomentArmSample {
                angle,
                length: data.ten_length[tendon_idx],
                moment_arm: -data.ten_J[tendon_idx][dof],
            }
        })
        .collect()
}

/// Central-difference moment arm `r = -ΔL/Δθ` at one angle, computed purely
/// from tendon *length* (independent of `ten_J`). Used by the R4 micro-spike to
/// cross-check that the `ten_J`-derived moment arm is self-consistent with the
/// engine's own length curve — i.e. that the extraction + sign are correct.
pub fn moment_arm_finite_diff(
    model: &Model,
    tendon_idx: usize,
    jnt_id: usize,
    angle: f64,
    eps: f64,
) -> f64 {
    let qadr = model.jnt_qpos_adr[jnt_id];
    let len_at = |a: f64| {
        let mut data = model.make_data();
        data.qpos[qadr] = a;
        data.forward(model).expect("forward pass failed");
        data.ten_length[tendon_idx]
    };
    -(len_at(angle + eps) - len_at(angle - eps)) / (2.0 * eps)
}

/// Tendon length when the model is posed at knee angle `theta` by `set_qpos`,
/// which writes ALL coupled DOFs (the flexion angle plus any coupled slides /
/// patella DOFs). This drives the coupled-knee model directly rather than via
/// the constraint solver, so the pose is exact at every angle.
pub fn coupled_length(
    model: &Model,
    tendon_idx: usize,
    theta: f64,
    set_qpos: &dyn Fn(f64, &mut [f64]),
) -> f64 {
    let mut data = model.make_data();
    set_qpos(theta, data.qpos.as_mut_slice());
    data.forward(model).expect("forward pass failed");
    data.ten_length[tendon_idx]
}

/// Moment arm of `tendon_idx` about the knee coordinate for a coupled model:
/// the TOTAL derivative `-dL/dθ` along the coupled manifold (the coupled DOFs
/// move with `theta` via `set_qpos`), by central difference of [`coupled_length`].
/// This is the correct moment arm when extra DOFs are coupled to the flexion
/// angle — `-ten_J[knee]` alone would give only the partial derivative.
pub fn coupled_moment_arm(
    model: &Model,
    tendon_idx: usize,
    theta: f64,
    eps: f64,
    set_qpos: &dyn Fn(f64, &mut [f64]),
) -> f64 {
    -(coupled_length(model, tendon_idx, theta + eps, set_qpos)
        - coupled_length(model, tendon_idx, theta - eps, set_qpos))
        / (2.0 * eps)
}
