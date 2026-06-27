//! Moving-end-effector NORMAL wrench curvature — the coupled ARTICULATED trajectory gradient on
//! a FINITE posed sphere whose centre TRACKS the arm tip (`with_contact_geom`), so the contact
//! centre moves laterally (x/y) as well as vertically as the body swings.
//!
//! The lateral generalization of `sphere_articulated_trajectory_gradient.rs` (which poses the
//! sphere over the block CENTROID, threading only the scalar height channel `∂h/∂q = J_z`). Here
//! the sphere rides the geom (`centre = geom_xpos(q)`), so the pose channel is the 3-vector centre
//! (`∂centre/∂q = J_geom`, the `PoseCentreVjp` seam) and the wrench node's pose parent is
//! `WrenchPose::Centre` (FD-exact in the `sphere_contact_wrench_node_centre_matches_readout_fd`
//! lib unit test). This gate validates the multi-step composition against the full real coupled
//! re-rollout FD, which poses at the same geom each step (forward and adjoint share the posing).
//!
//! Why it's genuinely new vs the centroid gate: as the y-hinge arm swings, the contact sphere
//! centre moves in x (not just z), so the gradient must thread `∂centre_x/∂q`, `∂centre_y/∂q`
//! — the lateral pose-sensitivity (`soft_pose_sensitivity::sphere_pose_sensitivity_lateral...`)
//! and the lateral wrench feedback the scalar-height channel drops. A height-only gradient is
//! wrong here; this gate is the moving-EE foundation for R5 (a co-designed strike at the tip).
//!
//! Scope: contact-engaged, stable-active-set, normal-only (no friction). The arm tip is posed
//! directly OVER the block (unlike the centroid gates' deliberately off-block body at xy=(0,0)),
//! with a large gentle sphere (r = 0.08, low curvature ⇒ a definite forward tangent). See
//! `project-differentiable-finite-contact.md`.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// A Y-hinge arm whose long link carries a finite sphere end-effector DIRECTLY over the soft block
// (body at xy = block centroid (0.05, 0.05); the link reaches down so the sphere south pole presses
// ~4 mm into the block top at z = 0.1). Tilted (qpos = 0.15) so the tip arcs (off-COM moment) and,
// crucially, the contact centre translates in x as the arm swings — the lateral channel this gate
// exists to validate. No joint damping (bare-M⁻¹).
const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0.05 0.05 0.344">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.17" size="0.08" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const MU0: f64 = 3.0e4;
/// Sphere radius: large vs the 0.1 m block so the south-pole patch spans several top-face vertices
/// yet stays gently curved (low `H = 1/r` ⇒ a definite forward tangent, no LU fallback). Paired
/// with the geom's own `size` (the fist), as `with_contact_geom` expects.
const SPHERE_R: f64 = 0.08;

fn build(mu: f64) -> StaggeredCoupling {
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.15; // tilt → off-COM moment + lateral centre travel
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R)
    .with_contact_geom(0)
}

/// The tip-posed sphere engages the block and the rollout stays stable.
#[test]
fn sphere_moving_ee_trajectory_engages_and_is_stable() {
    let mut c = build(MU0);
    let zn = c.coupled_trajectory_articulated_z(8);
    assert!(zn.is_finite(), "tip height must stay finite");
    assert!(
        (0.10..0.40).contains(&zn),
        "arm reference height should remain finite/engaged, got zN={zn}"
    );
}

/// The total `∂tip_z_N/∂μ` (along λ = 4μ) from one moving-EE tape vs a central FD of the full real
/// coupled sphere re-rollout (which poses at the geom each step). The 3-vector centre channel
/// (lateral + height) is what makes this match — a height-only `PoseSeamVjp`/`WrenchPose::Height`
/// drops the lateral feedback as the arm swings.
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
fn sphere_moving_ee_gradient_matches_full_coupled_fd() {
    for n in [2usize, 6] {
        let (total, fd) = tape_and_fd(n);
        let rel = (total - fd).abs() / fd.abs().max(1e-30);
        eprintln!("sphere moving-EE (n={n}): tape={total:.6e} FD={fd:.6e} rel={rel:.3e}");
        assert!(fd.abs() > 1e-11, "degenerate gate at n={n}: FD ≈ 0 ({fd})");
        // Machine-exact (~1e-8) like the centroid articulated gate — the centre wrench node is
        // FD-exact per-term (lib unit test), the lateral pose-sensitivity is machine-exact (#427),
        // and the normal-only solve converges tightly (no curved-friction forward-solver floor).
        // Gate at 1e-6 for cross-platform margin; a dropped lateral channel blows it up.
        assert!(
            rel < 1e-6,
            "n={n}: moving-EE sphere ∂tip_z/∂μ {total} disagrees with FD {fd} (rel {rel:e})"
        );
    }
}

/// The moving-EE centre carry is threaded ONLY through the articulated MATERIAL (normal) gradient.
/// The sphere-capable articulated FRICTION gradients do NOT yet thread it, so a set contact geom
/// must FAIL LOUDLY (`require_no_moving_ee`) rather than silently pose at the block centroid and
/// disagree with a tip-posed forward (a silent contract violation is ship-blocking). The guard is
/// the gradient's first statement, so no friction config is needed to reach it.
#[test]
#[should_panic(expected = "moving end-effector")]
fn moving_ee_panics_on_articulated_friction_gradient() {
    let _ = build(MU0).coupled_trajectory_tangential_material_gradient_articulated(2, 0);
}

/// The friction-COEFFICIENT articulated gradient is the second sphere-capable friction path that
/// does not thread the centre — it too must reject a set contact geom (the same guard).
#[test]
#[should_panic(expected = "moving end-effector")]
fn moving_ee_panics_on_articulated_friction_coeff_gradient() {
    let _ = build(MU0).coupled_trajectory_tangential_friction_coeff_gradient_articulated(2);
}

/// Finding-1 regression: with a PLANE collider, `with_contact_geom` must be a NO-OP for the
/// gradient. `build_contact` ignores the centre override for a plane (the forward poses at the
/// COM height `xipos.z`), so the adjoint must take the scalar-height channel — NOT the
/// geom-Jacobian centre channel (which routes `∂h/∂q` through `geom_xpos ≠ xipos` → a silently
/// wrong gradient). The gradient with a set contact geom must be BYTE-IDENTICAL to without.
#[test]
fn plane_collider_ignores_contact_geom_in_gradient() {
    const PLANE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
    let build_plane = |geom: Option<usize>| -> StaggeredCoupling {
        let model = load_model(PLANE_MJCF).expect("plane MJCF loads");
        let mut data = model.make_data();
        data.qpos[0] = 0.3; // tilt → off-COM contact (geom_xpos ≠ xipos for the rotational DOF)
        data.forward(&model).expect("forward");
        let mut c: StaggeredCoupling = StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, MU0, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        );
        if let Some(g) = geom {
            c = c.with_contact_geom(g);
        }
        c
    };
    let (z0, g0) = build_plane(None).coupled_trajectory_material_gradient_articulated(4, 0);
    let (z1, g1) = build_plane(Some(0)).coupled_trajectory_material_gradient_articulated(4, 0);
    assert_eq!(
        z0, z1,
        "plane + contact_geom must not change the forward value"
    );
    assert_eq!(
        g0, g1,
        "plane + contact_geom must not change the gradient (with_contact_geom is a no-op for a plane)"
    );
}
