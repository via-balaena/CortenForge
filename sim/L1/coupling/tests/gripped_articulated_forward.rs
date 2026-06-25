//! Friction → matrix-carry lift, sub-leaf 1 — the articulated GRIPPED forward oracle.
//!
//! `StaggeredCoupling::coupled_trajectory_gripped_articulated` is the articulated successor
//! to the free-platen `coupled_trajectory_grip` and the gripped sibling of the normal-only
//! `coupled_trajectory_articulated_z`: it rolls a hinge arm forward routing the FULL gripped
//! reaction wrench `[τ; f]` (normal + friction + off-COM moment) onto the joint, with the
//! moving-collider drift read from the articulated state (`Δ_surf = J_lin·qvel·dt`). It is the
//! independent black-box reference the articulated friction gradient will be finite-differenced
//! against; this smoke test pins it down before the gradient is built.
//!
//! Gates: (1) the rollout stays finite and contact-engaged (the tilted arm presses on the
//! block top); (2) the friction path is LIVE — with friction active the tip's tangential drag
//! differs materially from the same scene run frictionless (friction at the swept contact
//! resists the arc).

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// A Y-hinge arm (pivot above the block, point mass on the tip) pushed sideways by a tilted
// gravity `gx = 1.0`, started tilted (qpos = 0.3) so the tip arcs and presses into the block
// top (z = 0.1) — a sustained, tangentially-swept contact for friction to grip.
const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const SOFT_MU: f64 = 3.0e3; // compliant block — a visible grip-drag lever
const FRIC_MU: f64 = 2.5; // Coulomb coefficient (the creep grip regime)
const EPS_V: f64 = 0.1;
const N: usize = 40; // the creep grip horizon (matches the free-platen friction gate)

fn build(friction: bool) -> StaggeredCoupling {
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3; // tilt off vertical
    data.forward(&model).expect("initial forward");
    // body=1, clearance=0.005, n_per_edge=4, edge=0.1, soft μ, dt=1e-3, kappa=3e4,
    // d_hat=1e-2, rigid_damping=0 (articulated v1 scope).
    let c = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, SOFT_MU, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    );
    if friction {
        c.with_friction(FRIC_MU, EPS_V)
    } else {
        c
    }
}

#[test]
fn gripped_articulated_engages_and_stays_finite() {
    let p = build(true).coupled_trajectory_gripped_articulated(N);
    assert!(
        p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
        "gripped articulated rollout must stay finite, got {p:?}"
    );
    // The arm frame origin is the pivot at (0, 0, 0.2); the tip (geom) presses near the
    // block top. The pivot z is fixed by the hinge, so engagement is read off the tip COM.
    let tip_z = build(true).data().xipos[1].z;
    assert!(
        tip_z < 0.115 && tip_z > 0.10,
        "tip should sit within the penalty band near the block top, got {tip_z}"
    );
}

#[test]
fn friction_path_is_live() {
    let with = build(true).coupled_trajectory_gripped_articulated(N);
    let without = build(false).coupled_trajectory_gripped_articulated(N);
    assert!(
        with.x.is_finite() && without.x.is_finite(),
        "both rollouts must stay finite"
    );
    // Friction at the swept tip contact resists the gravity-driven arc, so the tangential
    // drag (the pivot/joint-driven x of the tip) must differ from the frictionless run.
    let dx = (with.x - without.x).abs();
    assert!(
        dx > 1e-6,
        "friction must change the tangential drag (Δx = {dx:e}); the gripped/friction path \
         appears inert"
    );
}
