//! Moving-end-effector NORMAL wrench curvature — the two invariants the consolidated
//! grad harness can't express, for a FINITE posed sphere whose centre TRACKS the arm tip
//! (`with_contact_geom`), so the contact centre moves laterally (x/y) as well as vertically
//! as the body swings.
//!
//! The lateral generalization of `sphere_articulated_trajectory_gradient.rs` (which poses the
//! sphere over the block CENTROID, threading only the scalar height channel `∂h/∂q = J_z`). Here
//! the sphere rides the geom (`centre = geom_xpos(q)`), so the pose channel is the 3-vector centre
//! (`∂centre/∂q = J_geom`, the `PoseCentreVjp` seam) and the wrench node's pose parent is
//! `WrenchPose::Centre`. The per-horizon FD-match of that multi-step gradient against the full
//! real coupled re-rollout — and the engagement/stability of the tip-posed sphere — are now the
//! `sphere-moving-ee·material[μ]` row of `coupling_grad_harness.rs` (the oracle poses at the same
//! geom each step, so the FD is moving-EE too; a height-only carry blows the rel up). What stays
//! here are two checks that row structurally can't hold:
//!
//! 1. **Free-body contract guard** (`#[should_panic]`) — the moving-EE centre carry is threaded
//!    through the articulated NORMAL + FRICTION gradients, but the sphere-capable FREE-BODY
//!    gradients do NOT thread it, so a set contact geom must still FAIL LOUDLY
//!    (`require_no_moving_ee`) there rather than silently pose at the block centroid (a silent
//!    contract violation is ship-blocking).
//! 2. **Plane-collider no-op** (byte-identity) — Finding-1 regression: with a PLANE collider,
//!    `with_contact_geom` must be a NO-OP for the gradient, BYTE-IDENTICAL to without (the forward
//!    poses at the COM height `xipos.z`, so the adjoint must take the scalar-height channel, NOT
//!    the geom-Jacobian centre channel which routes `∂h/∂q` through `geom_xpos ≠ xipos`).
//!
//! Scope: contact-engaged, stable-active-set, normal-only (no friction). See
//! `project-differentiable-finite-contact.md`.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// A Y-hinge arm whose long link carries a finite sphere end-effector DIRECTLY over the soft block
// (body at xy = block centroid (0.05, 0.05); the link reaches down so the sphere south pole presses
// ~4 mm into the block top at z = 0.1). Tilted (qpos = 0.15) so the tip arcs (off-COM moment) and,
// crucially, the contact centre translates in x as the arm swings — the lateral channel.
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
/// yet stays gently curved (low `H = 1/r` ⇒ a definite forward tangent, no LU fallback).
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

/// The moving-EE centre carry is threaded through the articulated NORMAL + FRICTION gradients, but
/// the sphere-capable FREE-BODY gradients do NOT thread it, so a set contact geom must still FAIL
/// LOUDLY (`require_no_moving_ee`) there rather than silently pose at the block centroid (a silent
/// contract violation is ship-blocking). The guard is the gradient's first statement, so it panics
/// before any scope assert (the articulated `build()` coupling reaches it on the free-body path).
#[test]
#[should_panic(expected = "moving end-effector")]
fn moving_ee_panics_on_free_body_friction_gradient() {
    let _ = build(MU0).coupled_trajectory_tangential_material_gradient(2, 0);
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
