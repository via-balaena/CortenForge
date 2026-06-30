//! Free-body orientation gradient gate (Layer-2 keystone, quaternion position carry).
//!
//! [`StaggeredCoupling::coupled_trajectory_orientation_gradient`] differentiates a free body's
//! final orientation — a quaternion vector component `q_vec[axis]` — w.r.t. the soft block's
//! material parameter. Unlike `ω_N` (which exercises only the wrench MOMENT carry `τ→ω`), the
//! orientation depends on the multi-DOF carry's POSITION rows `G_pos = Δt·J_r·G_vel` (the SO(3)
//! right-Jacobian integrating `qvel'` into the quaternion), so this target is the one that gates
//! `G_pos` — the last untested rung of the free-joint wrench carry.
//!
//! The objective is smooth (a quaternion component, no angle-wrap) on a rollout kept below a half
//! turn. Its body-frame tangent VJP is closed-form (`∂q_vec/∂δ = ½(w·I + [v]×)`), so no SO(3)-log
//! enters the adjoint.
//!
//! The single-length FD-match + forward-consistency + engagement coverage (n = 16) is the
//! `freebody·orientation[μ]` row of `coupling_grad_harness.rs`, the channel-agnostic FD matrix.
//! What lives HERE is the one check that row does not subsume: the machine-exactness holds at
//! EVERY rollout length below a half turn (n = 4, 8, 16, 20) — an all-lengths sweep on the
//! off-centre tumble that pins `G_pos` across horizons.

#![allow(
    // A missing/malformed fixture (MJCF load, body index) surfaces as a test panic —
    // the canonical fixture idiom in this workspace's integration tests.
    clippy::expect_used
)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

const EDGE: f64 = 0.1;
const MASS: f64 = 0.2;
const HALF_X: f64 = 0.06;
const HALF_Z: f64 = 0.005;
const DT: f64 = 1.0e-3;
const PEN: f64 = 1.0e-4;
const MU0: f64 = 3.0e4;
const AXIS: usize = 1; // qy — an off-centre +x strike tumbles the body about body-y

/// An off-centre platen (COM at x = 0.07) so the contact moment tumbles the body, pre-penetrated
/// and released from rest. `rigid_damping = 0` (wrench-carry v1 scope), moment routed.
fn build(mu: f64) -> StaggeredCoupling {
    let com_z = EDGE + HALF_Z - PEN;
    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="{DT}"/>
  <worldbody>
    <body name="platen" pos="0.07 0.05 {com_z}">
      <freejoint/>
      <geom type="box" size="{HALF_X} {HALF_X} {HALF_Z}" mass="{MASS}"/>
    </body>
  </worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(model, data, 1, HALF_Z, 4, EDGE, mu, DT, 3.0e4, 1.0e-2, 0.0)
        .with_contact_moment(true)
}

/// Final quaternion component `qpos[3 + 1 + AXIS]` (single free joint at qpos adr 0: `[x,y,z, w,x,y,z]`)
/// after `n` steps of the REAL coupled (moment-on) dynamics.
fn final_qcomp(mu: f64, n: usize) -> f64 {
    let mut c = build(mu);
    for _ in 0..n {
        c.step();
    }
    c.data().qpos[4 + AXIS]
}

/// Machine-exact at every rollout length kept below a half turn (the smooth regime). The
/// single-length (n = 16) FD-match, forward-consistency, and engagement coverage is the
/// `freebody·orientation[μ]` row of `coupling_grad_harness.rs`; this sweep extends it across
/// horizons (n = 4, 8, 16, 20).
#[test]
fn orientation_gradient_machine_exact_at_all_lengths() {
    for &n in &[4_usize, 8, 16, 20] {
        let g = build(MU0)
            .coupled_trajectory_orientation_gradient(n, 0, AXIS)
            .1
            + 4.0
                * build(MU0)
                    .coupled_trajectory_orientation_gradient(n, 1, AXIS)
                    .1;
        let eps = MU0 * 5e-4;
        let fd = (final_qcomp(MU0 + eps, n) - final_qcomp(MU0 - eps, n)) / (2.0 * eps);
        let rel = (g - fd).abs() / fd.abs().max(1e-30);
        let abs = (g - fd).abs();
        eprintln!("n={n}: rel={rel:.3e} abs={abs:.3e}");
        assert!(
            rel < 1e-6 || abs < 1e-11,
            "dqy/dμ at n={n} should be machine-exact vs full-coupled FD, got rel {rel:e} abs {abs:e}"
        );
    }
}
