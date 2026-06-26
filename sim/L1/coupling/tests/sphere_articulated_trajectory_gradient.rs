//! L1b articulated NORMAL wrench curvature — the coupled ARTICULATED trajectory gradient on a
//! FINITE posed sphere collider (the off-COM contact moment, curved collider).
//!
//! The articulated successor to `sphere_trajectory_gradient.rs` (free-body NORMAL on the
//! sphere): the same `coupled_trajectory_material_gradient_articulated` tape against the same
//! full-coupled re-rollout FD oracle (`coupled_trajectory_articulated_z`) as the plane gate
//! `articulated_trajectory_gradient.rs`, but with `with_sphere_collider`. The Y-hinge arm
//! presses the soft block through a finite sphere end-effector; the reaction is routed as the
//! full spatial wrench `[τ; f]` about the body COM.
//!
//! Why it's genuinely new: a sphere's contact normal turns as a soft vertex slides over it
//! (`∂n̂/∂x = H`) and as the primitive translates with the tip height (`∂n̂/∂h = −H·ẑ`), so the
//! contact-wrench Jacobian carries the geometric-stiffness term `f_mag·H` (zero for the plane's
//! constant normal) in BOTH `∂w/∂x*` and `∂w/∂h`. The wrench node is FD-exact vs the curved
//! readout in the `sphere_contact_wrench_node_matches_readout_fd` lib unit test; this gate
//! validates the multi-step composition.
//!
//! Scope: contact-engaged, stable-active-set regime; the sphere is posed laterally over the
//! block centroid with its south pole at `tip_height = xipos.z − clearance` (the same
//! laterally-fixed scope as the free-body sphere gates — lateral posing AT the arm tip and the
//! articulated FRICTION wrench are the remaining L1b follow-ons). See
//! `project-differentiable-finite-contact.md`.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// The articulated plane scene + a finite sphere end-effector. Tilted (qpos = 0.3) so the tip
// arcs (off-COM moment) and the sphere south pole (xipos.z − clearance) presses into the block
// top (z = 0.1); gravity swings it inward → sustained engagement. No joint damping (bare-M⁻¹).
const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const MU0: f64 = 3.0e4;
/// Sphere radius: large vs the 0.1 m block so the south-pole patch spans several top-face
/// vertices yet stays curved enough that f_mag·H is materially nonzero (matches the lib gate).
const SPHERE_R: f64 = 0.08;

fn build(mu: f64) -> StaggeredCoupling {
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3; // tilt off vertical
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R)
}

/// The sphere end-effector engages the block and the rollout stays stable.
#[test]
fn sphere_articulated_trajectory_engages_and_is_stable() {
    let mut c = build(MU0);
    let zn = c.coupled_trajectory_articulated_z(8);
    assert!(zn.is_finite(), "tip height must stay finite");
    assert!(
        (0.10..0.118).contains(&zn),
        "tip should remain engaged near the block top, got zN={zn}"
    );
}

/// The total `∂tip_z_N/∂μ` (along the λ = 4μ tie) from one tape vs a central FD of the full
/// real coupled sphere re-rollout. The curved-normal `f_mag·H` wrench term (zero for the plane)
/// is what makes this match — a flat `ContactWrenchTrajVjp` misses it.
fn tape_and_fd(n: usize) -> (f64, f64) {
    let g_mu = build(MU0)
        .coupled_trajectory_material_gradient_articulated(n, 0)
        .1;
    let g_la = build(MU0)
        .coupled_trajectory_material_gradient_articulated(n, 1)
        .1;
    let total = g_mu + 4.0 * g_la;
    let eps = MU0 * 1e-4;
    let zp = build(MU0 + eps).coupled_trajectory_articulated_z(n);
    let zm = build(MU0 - eps).coupled_trajectory_articulated_z(n);
    let fd = (zp - zm) / (2.0 * eps);
    (total, fd)
}

#[test]
fn sphere_articulated_gradient_matches_full_coupled_fd() {
    for n in [2usize, 6] {
        let (total, fd) = tape_and_fd(n);
        let rel = (total - fd).abs() / fd.abs().max(1e-30);
        eprintln!("sphere articulated (n={n}): tape={total:.6e} FD={fd:.6e} rel={rel:.3e}");
        assert!(fd.abs() > 1e-11, "degenerate gate at n={n}: FD ≈ 0 ({fd})");
        // Machine-exact (~1e-9) like the plane articulated gate — the wrench node is FD-exact
        // per-term in the lib unit test, and the normal-only articulated solve converges tightly
        // (no curved-friction forward-solver floor). Gate at 1e-6 for cross-platform margin; a
        // dropped curved term blows it to the ~1e-2 curvature scale.
        assert!(
            rel < 1e-6,
            "n={n}: one-tape sphere articulated ∂tip_z/∂μ {total} disagrees with FD {fd} (rel {rel:e})"
        );
    }
}
