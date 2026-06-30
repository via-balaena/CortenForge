//! Keystone rotating-normal leaf (PR2) — the two invariants the consolidated grad
//! harness can't express, for a contact normal that tracks body orientation
//! (`n̂ = R(q)·(0,0,−1)`, enabled via `StaggeredCoupling::with_rotating_normal(true)`).
//!
//! The per-horizon FD-match of the rotating-normal trajectory gradient (ball joint and
//! free base, gentle θ ≈ 0.1 tilt) is now the `ball-rot·material[μ]` / `free-rot·material[μ]`
//! rows of `coupling_grad_harness.rs` — the gradient flowing through the normal's
//! `q`-dependence (the soft pose-adjoint's `δn̂ = ω×n̂` term + the wrench's `∂w/∂T`
//! normal-redirect + the `PoseTwistSeamVjp` seam), FD'd against the matched
//! rotating-normal oracle. What stays here:
//!
//! 1. **Materiality** — a cross-scene check the FD matrix can't hold: the rotating normal
//!    must *change* the gradient vs the flat normal on the SAME scene (the new channel is
//!    live, not a vacuous pass).
//! 2. **Single-step height probes** — a Jacobian/VJP-structure shape (Bucket B), not a
//!    scalar `(loss, grad)`: the explicit and total `∂force/∂height` match a central FD,
//!    AND the tilted normal gives the height-jacobian a horizontal component (the `height`
//!    arg stays load-bearing under a rotating normal, and the redirect is real).
//!
//! **Stability.** The rotating normal redirects ~`sin θ` of the contact force tangentially;
//! at large tilt it destabilizes the flat-tuned scenes (θ = 0.3 ball diverges), so these
//! use a GENTLE tilt (θ ≈ 0.1). See `docs/keystone/rotating_normal_recon.md`.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

const MU0: f64 = 3.0e4;

// A ball joint at a GENTLE tilt θ = 0.1 about Y (vs the flat-path gate's θ = 0.3):
// the off-COM tip presses the block, the rotating normal redirects the reaction.
const BALL_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="ball"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

fn build_ball(mu: f64, rotating: bool) -> StaggeredCoupling {
    let model = load_model(BALL_MJCF).expect("ball MJCF loads");
    let mut data = model.make_data();
    let half = 0.05_f64; // θ = 0.1 about Y
    data.qpos[0] = half.cos();
    data.qpos[2] = half.sin();
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_rotating_normal(rotating)
}

/// Materiality: the rotating normal genuinely changes the gradient — the new normal
/// channel is LIVE, not a vacuous pass. A gentle-tilt ball gradient with the rotating
/// normal differs measurably from the flat-normal gradient on the SAME scene.
#[test]
fn rotating_normal_changes_the_gradient() {
    let n = 4;
    let g_rot = {
        let (_t, gm) = build_ball(MU0, true).coupled_trajectory_material_gradient_articulated(n, 0);
        let (_t2, gl) =
            build_ball(MU0, true).coupled_trajectory_material_gradient_articulated(n, 1);
        gm + 4.0 * gl
    };
    let g_flat = {
        let (_t, gm) =
            build_ball(MU0, false).coupled_trajectory_material_gradient_articulated(n, 0);
        let (_t2, gl) =
            build_ball(MU0, false).coupled_trajectory_material_gradient_articulated(n, 1);
        gm + 4.0 * gl
    };
    let rel = (g_rot - g_flat).abs() / g_flat.abs().max(1e-30);
    println!("rot={g_rot:.6e} flat={g_flat:.6e} rel-diff={rel:.3e}");
    assert!(
        rel > 1e-3,
        "the rotating normal must change the gradient materially (rel diff {rel:.3e})"
    );
}

/// The single-step S3 height probes stay correct AND `height`-parameterized under a
/// rotating normal: `build_contact` honors the scalar `height` (translating the tilted
/// plane vertically), so the explicit `contact_force_height_jacobian` and the total
/// `contact_force_height_total_jacobian` match a central FD of their forward oracles
/// (`contact_force_at_height` / `resolved_contact_force`) on a tilted scene. This pins
/// the contract the rotating normal could otherwise have silently broken (a dead
/// `height` argument), and confirms the tilted normal redirects the force off-vertical.
#[test]
fn rotating_normal_height_probes_match_fd() {
    let c = build_ball(MU0, true);
    // The engaged plane height for the current pose (clearance = 0.005, as built).
    let h0 = c.data().xipos[1].z - 0.005;
    let eps = 1e-7;

    // Explicit (fixed-position) partial vs FD of contact_force_at_height.
    let analytic_expl = c.contact_force_height_jacobian(h0);
    let fd_expl =
        (c.contact_force_at_height(h0 + eps) - c.contact_force_at_height(h0 - eps)) / (2.0 * eps);
    let rel_expl = (analytic_expl - fd_expl).norm() / fd_expl.norm().max(1e-30);
    println!("explicit: a={analytic_expl:?} fd={fd_expl:?} rel={rel_expl:.3e}");
    assert!(
        fd_expl.norm() > 1e-6 && rel_expl < 1e-4,
        "rotating explicit ∂force/∂height must match FD (rel {rel_expl:.3e})"
    );
    // The tilted normal redirects the force off-vertical (flat would be purely +ẑ).
    assert!(
        analytic_expl.x.hypot(analytic_expl.y) > 1e-3 * analytic_expl.z.abs(),
        "the tilted normal must give the height-jacobian a horizontal component"
    );

    // Total (re-equilibrated) derivative vs FD of resolved_contact_force.
    let analytic_tot = c.contact_force_height_total_jacobian(h0);
    let fd_tot =
        (c.resolved_contact_force(h0 + eps) - c.resolved_contact_force(h0 - eps)) / (2.0 * eps);
    let rel_tot = (analytic_tot - fd_tot).norm() / fd_tot.norm().max(1e-30);
    println!("total: a={analytic_tot:?} fd={fd_tot:?} rel={rel_tot:.3e}");
    assert!(
        fd_tot.norm() > 1e-6 && rel_tot < 1e-3,
        "rotating total d force/d height must match resolve-FD (rel {rel_tot:.3e})"
    );
}
