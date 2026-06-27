//! The frame-capturing friction-grip rollout (`coupled_trajectory_policy_gripped_capture`)
//! is the SAME physics as the scalar FD oracle (`coupled_trajectory_policy_gripped_x`).
//!
//! F1 of the R5 visual arc: a viewer needs the per-step deformed soft mesh + arm pose
//! to render the co-designed grip encounter, but those frames are transient (gone
//! after the rollout). The capturing method shares one loop body with the scalar
//! forward — the FD oracle for the design+policy-friction gradient (#406/#430) — so
//! the animated scene is provably the rollout the gradient differentiates, not a
//! re-derivation.
//!
//! Gates:
//! 1. the captured tip_x is BYTE-IDENTICAL to the scalar forward (the `None`/`Some`
//!    capture paths are the same physics);
//! 2. the frame count is `n_steps + 1` (a rest frame + one per step), each frame's
//!    soft mesh has the right length, and the soft body genuinely deforms.

#![allow(clippy::expect_used)]

use sim_coupling::{LinearFeedback, StaggeredCoupling};
use sim_mjcf::load_model;

const SPHERE_R: f64 = 0.08;
const PARAMS: [f64; 3] = [0.08, -0.02, 0.01];
const N_STEPS: usize = 4;

// The de-escalation grip: a hinge arm whose sphere tip sweeps tangentially into the
// soft block under sideways gravity (the centroid-posed friction grip).
fn fric_scene() -> &'static str {
    r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#
}

fn build(soft_mu: f64) -> StaggeredCoupling {
    let model = load_model(fric_scene()).expect("scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, soft_mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R)
    .with_friction(2.5, 0.1)
}

/// The captured rollout's terminal tip_x is byte-identical to the scalar forward —
/// the capture path adds no physics, so the picture is the oracle's rollout exactly.
#[test]
fn capture_tip_x_is_byte_identical_to_scalar_forward() {
    let scalar =
        build(3.0e3).coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, N_STEPS);
    let (captured, _frames) =
        build(3.0e3).coupled_trajectory_policy_gripped_capture(&LinearFeedback, &PARAMS, N_STEPS);
    assert_eq!(
        captured.x, scalar.x,
        "captured tip_x {} must byte-match the scalar oracle {}",
        captured.x, scalar.x
    );
    // The full COM vector matches too (not just the tracked .x component).
    assert_eq!(captured.y, scalar.y);
    assert_eq!(captured.z, scalar.z);
}

/// The capture yields `n_steps + 1` well-formed frames and the soft body deforms.
#[test]
fn capture_frames_are_well_formed_and_deform() {
    let mut c = build(3.0e3);
    let n_vertices = c.soft_positions().len() / 3;
    let (_tip, frames) =
        c.coupled_trajectory_policy_gripped_capture(&LinearFeedback, &PARAMS, N_STEPS);

    assert_eq!(
        frames.len(),
        N_STEPS + 1,
        "expected a rest frame + one per step ({} total), got {}",
        N_STEPS + 1,
        frames.len()
    );
    for (i, f) in frames.iter().enumerate() {
        assert_eq!(
            f.soft_positions.len(),
            3 * n_vertices,
            "frame {i}: soft_positions length {} != 3·n_vertices {}",
            f.soft_positions.len(),
            3 * n_vertices
        );
        assert!(
            f.soft_positions.iter().all(|p| p.is_finite())
                && f.arm_pivot.iter().all(|p| p.is_finite())
                && f.fist_center.iter().all(|p| p.is_finite()),
            "frame {i}: non-finite capture data"
        );
    }
    // The fist (centroid-posed sphere) sits over the 0.1 m block centre (x≈0.05, y≈0.05).
    let rest_fist = frames[0].fist_center;
    assert!(
        (rest_fist[0] - 0.05).abs() < 1e-6 && (rest_fist[1] - 0.05).abs() < 1e-6,
        "centroid-posed fist should sit over the block centre, got {rest_fist:?}"
    );
    // The soft body genuinely deforms between the rest frame and the last step.
    let max_disp = frames[N_STEPS]
        .soft_positions
        .iter()
        .zip(&frames[0].soft_positions)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0_f64, f64::max);
    assert!(
        max_disp > 1e-5,
        "the grip should deform the soft body (max vertex displacement {max_disp:e})"
    );
}
