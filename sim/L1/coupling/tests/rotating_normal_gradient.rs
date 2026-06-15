//! Keystone rotating-normal leaf (PR2) — the coupled ARTICULATED trajectory
//! gradient with a contact normal that tracks the body orientation
//! (`n̂ = R(q)·(0,0,−1)`).
//!
//! Enabled via `StaggeredCoupling::with_rotating_normal(true)`: a tilted body
//! presents a tilted contact face, and the coupled gradient flows through the
//! normal's `q`-dependence (the soft pose-adjoint's `δn̂ = ω×n̂` term + the wrench's
//! `∂w/∂T` normal-redirect feedback + the `PoseTwistSeamVjp` seam mapping the plane
//! twist through the body's spatial Jacobian `J_spatial = mj_jac_point(origin)`).
//!
//! **Matched pair.** Both the one-tape gradient AND the full-coupled FD oracle
//! (`coupled_trajectory_articulated_z`) build the contact through the SAME
//! `build_contact`, gated by the same flag — so with the flag on, the oracle uses
//! the rotating normal too. The gate is the tape gradient vs a central FD of that
//! rotating-normal oracle.
//!
//! **Stability.** The rotating normal redirects ~`sin θ` of the contact force
//! tangentially; at large tilt it destabilizes the flat-tuned scenes (θ = 0.3 ball
//! diverges), so these gates use a GENTLE tilt (θ ≈ 0.1) where both the flat and
//! rotating models are stable. Accuracy is FD-carry precision (~1e-6): a ball / free
//! joint has no single-hinge analytic `J_state`, so the carry uses the FD
//! `loaded_state_jacobian`. Gated at n = 1, 2, 4 (the well-conditioned, stably-
//! engaged regime). See `docs/keystone/rotating_normal_recon.md`.

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

// A free joint (floating base) with an off-COM mass — the full SE(3) successor.
// GENTLE engagement (started just above the block, θ = 0.04 tilt, a broad
// stabilizing base plate) so the unconstrained floating base stays stable under the
// rotating normal's tangential redirect: the flat-tuned deep-engagement free scene
// (the `articulated_trajectory_gradient` gate) LAUNCHES under a rotating normal by
// n = 2. The off-COM sphere still tips the body so the contact moment rotates it and
// `xipos.z` reads the orientation.
const FREE_OFFCOM_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="body" pos="0 0 0.123">
      <freejoint/>
      <geom type="box" pos="0 0 0" size="0.06 0.06 0.003" mass="0.2"/>
      <geom type="sphere" pos="0.02 0 -0.02" size="0.004" mass="0.1"/>
    </body>
  </worldbody>
</mujoco>"#;

fn build_free(mu: f64, rotating: bool) -> StaggeredCoupling {
    let model = load_model(FREE_OFFCOM_MJCF).expect("free off-COM MJCF loads");
    let mut data = model.make_data();
    let half = 0.02_f64; // θ = 0.04 tilt about Y
    data.qpos[3] = half.cos();
    data.qpos[5] = half.sin();
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_rotating_normal(rotating)
}

/// Tape gradient vs the rotating-normal full-coupled FD oracle, for a builder
/// `b(mu, rotating)`. Returns `(tape_total, fd, rel)` at horizon `n`.
fn tape_vs_fd(b: impl Fn(f64, bool) -> StaggeredCoupling, n: usize) -> (f64, f64, f64) {
    let (tip_z, gm) = b(MU0, true).coupled_trajectory_material_gradient_articulated(n, 0);
    let (_t, gl) = b(MU0, true).coupled_trajectory_material_gradient_articulated(n, 1);
    let total = gm + 4.0 * gl;
    // Fresh-output consistency: the tape forward must reproduce the real rollout.
    let z_oracle = b(MU0, true).coupled_trajectory_articulated_z(n);
    assert!(
        (tip_z - z_oracle).abs() < 1e-12,
        "n={n}: tape forward tip_z {tip_z} != rotating-normal rollout {z_oracle}"
    );
    let eps = MU0 * 1e-4;
    let zp = b(MU0 + eps, true).coupled_trajectory_articulated_z(n);
    let zm = b(MU0 - eps, true).coupled_trajectory_articulated_z(n);
    let fd = (zp - zm) / (2.0 * eps);
    let rel = (total - fd).abs() / fd.abs().max(1e-30);
    (total, fd, rel)
}

#[test]
fn ball_rotating_normal_gradient_matches_fd() {
    let z0 = build_ball(MU0, true).data().xipos[1].z;
    assert!(
        z0 > 0.10 && z0 < 0.115,
        "ball tip should start engaged near the block top, got {z0}"
    );
    for n in [1usize, 2, 4] {
        let (total, fd, rel) = tape_vs_fd(build_ball, n);
        println!("ball-rot n={n}: tape={total:.6e} FD={fd:.6e} rel={rel:.3e}");
        assert!(
            total.abs() > 1e-12 && rel < 1e-5,
            "ball rotating-normal gradient must match the rotating-normal full-coupled \
             FD (FD-carry precision) at n={n}, got rel {rel:.3e}"
        );
    }
}

#[test]
fn free_rotating_normal_gradient_matches_fd() {
    for n in [1usize, 2, 4] {
        let (total, fd, rel) = tape_vs_fd(build_free, n);
        println!("free-rot n={n}: tape={total:.6e} FD={fd:.6e} rel={rel:.3e}");
        assert!(
            total.abs() > 1e-12 && rel < 1e-5,
            "free-joint rotating-normal gradient must match the rotating-normal \
             full-coupled FD at n={n}, got rel {rel:.3e}"
        );
    }
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
