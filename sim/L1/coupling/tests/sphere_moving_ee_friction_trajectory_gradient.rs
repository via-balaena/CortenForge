//! Moving-end-effector FRICTION wrench curvature — the coupled ARTICULATED friction-grip
//! trajectory gradient on a FINITE sphere whose centre TRACKS the arm tip (`with_contact_geom`),
//! so the contact centre moves laterally (x/y) as the gripped limb sweeps.
//!
//! The friction successor to `sphere_moving_ee_trajectory_gradient.rs` (moving-EE NORMAL) and the
//! moving-EE successor to `sphere_articulated_friction_trajectory_gradient.rs` (centroid-posed
//! friction): the same `coupled_trajectory_tangential_material_gradient_articulated` /
//! `..._friction_coeff_gradient_articulated` tapes against the full-grip re-rollout FD oracle
//! (`coupled_trajectory_gripped_articulated`), but with `with_contact_geom` so the sphere rides the
//! geom. The pose channel is the 3-vector centre (`PoseCentreVjp` + `WrenchPose::Centre` + the
//! grip soft node's 3-axis pose columns + the friction wrench's 3-vector `dforce_dpose`), each FD-
//! exact at the single step in `friction_sphere_tangent.rs`'s lateral leaf gates.
//!
//! This is the de-escalation capstone physics: a soft body friction-gripping a moving rigid limb.
//!
//! **What this gate validates (and what it does NOT).** It validates the moving-EE friction
//! COMPOSITION — the friction force / drift / matrix-carry tape on a sphere posed at the geom each
//! step — machine-exact vs the geom-posed full-grip re-rollout FD, plus engagement/stability and
//! that the full moving-EE friction tape constructs+runs (the pose nodes' shape asserts fire on a
//! wiring bug). It does NOT discriminate the moving-EE POSE channel: the tangential-drag objective
//! `tip_x` is pose-INSENSITIVE (verified — zeroing the ENTIRE pose-centre seam `J_geom` leaves
//! `∂tip_x/∂μ` machine-exact, whereas it breaks the NORMAL `∂tip_z/∂μ` gate at ~4e-3), because the
//! drag is set by the friction-force magnitude (via `x*`/drift), not by the contact re-posing.
//!
//! The moving-EE friction POSE channel (the grip-centre soft pose-residual + the friction wrench's
//! 3-vector `dforce_dpose`) is therefore proven at SINGLE STEP by the lateral leaf gates in
//! `sim_soft/tests/friction_sphere_tangent.rs` (`..._lateral_matches_fd`, machine-exact with the
//! clean roundoff-vs-truncation FD-sweep V) — the project's "single-step gates ARE the proof"
//! principle. The shared NORMAL pose seam is discriminatingly gated by
//! `sphere_moving_ee_trajectory_gradient.rs` (#428, the pose-sensitive `tip_z` objective).
//!
//! Tolerance: 5e-3 — the curved-friction END-TO-END floor (same convention as the centroid friction
//! gate; observed here ~2e-9, but the #415 `dE·H` tangent is NSD so a stiffer/deeper curved grip
//! can floor the forward solve at the ~2e-3 curved-contact limit, not a gradient error).
//!
//! Scope: contact-engaged, stable-active-set, EUCLIDEAN joints, flat normal, `rigid_damping = 0`.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// A long Y-hinge arm whose finite sphere end-effector presses DIRECTLY over the soft block (body at
// xy = block centroid), with a sideways gravity push so the tip sweeps tangentially in x — the
// contact centre then translates in x (the moving-EE lateral channel) AS the friction grip drags.
const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="1.5 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0.05 0.05 0.344">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.17" size="0.08" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const MU0: f64 = 3.0e3; // compliant block — a well-conditioned μ-lever for the FD gate
const FRIC_MU: f64 = 2.5;
const EPS_V: f64 = 0.1;
const SPHERE_R: f64 = 0.08;

fn build(soft_mu: f64) -> StaggeredCoupling {
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.05; // slight tilt → off-COM + the sideways push sweeps the tip in x
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, soft_mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R)
    .with_contact_geom(0)
    .with_friction(FRIC_MU, EPS_V)
}

/// The tip-posed sphere grips the block under the sideways push and the rollout stays stable.
#[test]
fn sphere_moving_ee_friction_engages_and_is_stable() {
    let x = build(MU0).coupled_trajectory_gripped_articulated(8).x;
    assert!(x.is_finite(), "tip x must stay finite");
    assert!(
        (0.0..0.1).contains(&x),
        "tip should remain over the block, got x={x}"
    );
}

/// The total `∂(tip x)_N/∂μ` (along λ = 4μ) from one moving-EE friction tape vs a central FD of the
/// full real grip sphere re-rollout (posed at the geom each step). The 3-vector centre channel
/// (lateral + height) is what makes this match — a height-only channel drops the lateral feedback.
fn tape_and_fd_material(n: usize) -> (f64, f64, f64) {
    let (tip_x, g_mu) =
        build(MU0).coupled_trajectory_tangential_material_gradient_articulated(n, 0);
    let (_t2, g_la) = build(MU0).coupled_trajectory_tangential_material_gradient_articulated(n, 1);
    let total = g_mu + 4.0 * g_la;
    let eps = MU0 * 1e-4;
    let xp = build(MU0 + eps).coupled_trajectory_gripped_articulated(n).x;
    let xm = build(MU0 - eps).coupled_trajectory_gripped_articulated(n).x;
    let fd = (xp - xm) / (2.0 * eps);
    (tip_x, total, fd)
}

#[test]
fn sphere_moving_ee_friction_material_gradient_matches_full_coupled_fd() {
    for n in [2usize, 6] {
        let (tip_x, total, fd) = tape_and_fd_material(n);
        assert!(tip_x.is_finite() && total.is_finite() && fd.is_finite());
        let rel = (total - fd).abs() / fd.abs().max(1e-9);
        eprintln!(
            "moving-EE friction material (n={n}): tape={total:.6e} FD={fd:.6e} rel={rel:.3e}"
        );
        assert!(
            total.abs() > 1e-9 && fd.abs() > 1e-9,
            "n={n}: gradient implausibly ~0 (total={total:e}, fd={fd:e}) — friction channel inert?"
        );
        assert!(
            rel < 5e-3,
            "n={n}: moving-EE friction ∂tip_x/∂μ {total} disagrees with FD {fd} (rel {rel:e})"
        );
    }
}

/// `∂(tip x)_N/∂μ_c` (the Coulomb coefficient) from one moving-EE friction-coeff tape vs the full
/// grip re-rollout FD (perturbing `cfg.friction_mu`).
fn build_fmu(fric_mu: f64) -> StaggeredCoupling {
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.05;
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, MU0, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R)
    .with_contact_geom(0)
    .with_friction(fric_mu, EPS_V)
}

fn tape_and_fd_coeff(n: usize) -> (f64, f64) {
    let (_tip_x, g) =
        build(MU0).coupled_trajectory_tangential_friction_coeff_gradient_articulated(n);
    let eps = FRIC_MU * 1e-4;
    let xp = build_fmu(FRIC_MU + eps)
        .coupled_trajectory_gripped_articulated(n)
        .x;
    let xm = build_fmu(FRIC_MU - eps)
        .coupled_trajectory_gripped_articulated(n)
        .x;
    let fd = (xp - xm) / (2.0 * eps);
    (g, fd)
}

#[test]
fn sphere_moving_ee_friction_coeff_gradient_matches_full_coupled_fd() {
    for n in [2usize, 6] {
        let (g, fd) = tape_and_fd_coeff(n);
        let rel = (g - fd).abs() / fd.abs().max(1e-9);
        eprintln!("moving-EE friction coeff (n={n}): tape={g:.6e} FD={fd:.6e} rel={rel:.3e}");
        assert!(
            g.abs() > 1e-9 && fd.abs() > 1e-9,
            "n={n}: ∂/∂μ_c implausibly ~0 (g={g:e}, fd={fd:e})"
        );
        assert!(
            rel < 5e-3,
            "n={n}: moving-EE friction ∂tip_x/∂μ_c {g} disagrees with FD {fd} (rel {rel:e})"
        );
    }
}
