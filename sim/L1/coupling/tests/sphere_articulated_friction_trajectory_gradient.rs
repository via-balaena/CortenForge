//! L1b articulated FRICTION wrench curvature — the coupled ARTICULATED friction-grip trajectory
//! gradient on a FINITE posed sphere collider (the off-COM FRICTION moment, curved collider).
//!
//! The articulated-friction successor to the `sphere-articulated·material[μ]` row of
//! `coupling_grad_harness.rs` (articulated NORMAL on the sphere) and the curved-collider successor to the plane
//! `friction_articulated_material_gradient.rs`: the same
//! `coupled_trajectory_tangential_material_gradient_articulated` tape against the same full-grip
//! re-rollout FD oracle (`coupled_trajectory_gripped_articulated`), but with `with_sphere_collider`.
//! A Y-hinge arm grips the soft block through a finite sphere end-effector under a sideways push;
//! the tangential grip reaction is routed as the full spatial wrench `[τ_fric; f_fric]` about the
//! body COM.
//!
//! Why it's genuinely new: a sphere's contact normal turns as a soft vertex slides over it
//! (`∂n̂/∂x = C`) and as the primitive translates with the tip height (`∂n̂/∂h = −C·pose_dir`), so
//! the per-vertex FRICTION force Jacobian (`friction_force_jacobians`) carries the curved term
//! `DN·C` (zero for the plane's constant normal) in both `dforce_dx` and `dforce_dheight`. That
//! per-vertex Jacobian is FD-exact on the sphere in the `per_vertex_force_jacobians_sphere_matches_fd`
//! lib unit test (rel ~5e-9, the rigorous machine-exact proof); this gate validates the multi-step
//! composition through the `FrictionWrenchTrajVjp` force + off-COM-moment fold.
//!
//! Tolerance: held at 5e-3 — the curved-friction END-TO-END floor. In this compliant scene the
//! forward grip solve happens to converge tightly (observed rel ~4e-8), but the #415 curved-normal
//! geometric-stiffness tangent `dE·H` is negative semidefinite, so a stiffer/deeper curved grip
//! can floor the friction equilibrium at the ~2e-3 curved-contact forward-solver limit (not a
//! gradient error; see `sphere_friction_trajectory_gradient.rs`). 5e-3 is robust to that floor yet
//! discriminating: empirically zeroing the curved `DN·C` term blows BOTH this gate (→ ~2e-2) and
//! the leaf gate (→ ~3.8e-2) past it. The machine-exact validation is the single-step leaf gate.
//!
//! Scope: contact-engaged, stable-active-set regime; the sphere is posed laterally over the block
//! centroid (the same laterally-fixed scope as the other sphere gates — lateral posing AT the arm
//! tip is a separate forward-fidelity follow-on). See `project-differentiable-finite-contact.md`.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// A Y-hinge arm pushed sideways by tilted gravity, started tilted so the tip arcs and presses the
// sphere south pole into the block top (z = 0.1) — a sustained, tangentially-swept friction grip.
const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const MU0: f64 = 3.0e3; // compliant block — a well-conditioned μ-lever for the FD gate
const FRIC_MU: f64 = 2.5;
const EPS_V: f64 = 0.1;
/// Sphere radius: large vs the 0.1 m block so the south-pole patch spans several top-face vertices
/// yet stays curved enough that the curved-normal `DN·C` is materially nonzero (matches the lib gate).
const SPHERE_R: f64 = 0.08;

fn build(soft_mu: f64) -> StaggeredCoupling {
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3; // tilt off vertical
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, soft_mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R)
    .with_friction(FRIC_MU, EPS_V)
}

/// The sphere end-effector engages the block under the sideways push and the rollout stays stable.
#[test]
fn sphere_articulated_friction_engages_and_is_stable() {
    let x = build(MU0).coupled_trajectory_gripped_articulated(8).x;
    assert!(x.is_finite(), "tip x must stay finite");
    // The tip sweeps tangentially under the sideways push but stays over the engaged block.
    assert!(x.abs() < 0.1, "tip should remain near the block, got x={x}");
}

/// The total `∂(tip x)_N/∂μ` (along λ = 4μ) from one tape vs a central FD of the full real grip
/// sphere re-rollout. The curved-normal `DN·C` friction-wrench term (zero for the plane) is what
/// holds this to the forward-solver floor — a flat `FrictionWrenchTrajVjp` misses it and disagrees
/// at the ~1e-2 curvature scale.
fn tape_and_fd(n: usize) -> (f64, f64, f64) {
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
fn sphere_articulated_friction_gradient_matches_full_coupled_fd() {
    for n in [2usize, 6] {
        let (tip_x, total, fd) = tape_and_fd(n);
        assert!(tip_x.is_finite() && total.is_finite() && fd.is_finite());
        let rel = (total - fd).abs() / fd.abs().max(1e-9);
        eprintln!(
            "sphere articulated friction (n={n}): tape={total:.6e} FD={fd:.6e} rel={rel:.3e}"
        );
        // Non-vacuity: a friction channel regressed to ~0 would make BOTH ~0 (mirrors the plane gate).
        assert!(
            total.abs() > 1e-9 && fd.abs() > 1e-9,
            "n={n}: gradient implausibly ~0 (total={total:e}, fd={fd:e}) — friction channel inert?"
        );
        // Curved-contact forward-solver floor (~2e-3); a dropped curved term → ~1e-2.
        assert!(
            rel < 5e-3,
            "n={n}: one-tape sphere articulated friction ∂tip_x/∂μ {total} disagrees with FD {fd} (rel {rel:e})"
        );
    }
}
