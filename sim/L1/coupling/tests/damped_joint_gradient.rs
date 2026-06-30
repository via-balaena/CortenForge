//! Keystone damped-joints leaf (PR1) — the one invariant the consolidated grad
//! harness can't express: joint `damping=` MATERIALLY changes the coupled gradient.
//!
//! Under the default Euler integrator, `eulerdamp` solves `(M + Δt·D)·qacc = F` then
//! `qvel += Δt·qacc`, so the contact wrench reaches `qvel'` through `M_impl = M + Δt·D`,
//! not the bare `M` the carry used pre-leaf. Bare `M` is materially wrong under damping
//! (≈28% on the hinge, ≈10× on the chain — the S0 spike).
//!
//! The per-horizon FD-match of the damped gradient (single hinge: analytic damped
//! `J_state`, ~1e-9; 2-link chain: FD `J_state`, ~1e-6) is now the
//! `damped-hinge·material[μ]` / `damped-2link·material[μ]` rows of
//! `coupling_grad_harness.rs`. What stays here is the cross-SCENE check those FD rows
//! structurally can't hold: the SAME scene with vs without joint damping gives a
//! measurably different `∂tip_z/∂μ`, so the `M_impl` channel is genuinely live (a
//! bare-`M` carry would silently mis-weight the wrench response).

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

const MU0: f64 = 3.0e4;

const DAMPED_HINGE: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0" damping="0.5"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const UNDAMPED_HINGE: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

fn build(mjcf: &str, mu: f64) -> StaggeredCoupling {
    let model = load_model(mjcf).expect("damped model loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

/// Materiality: damping genuinely changes the gradient — the same scene with vs without
/// joint damping gives a measurably different `∂tip_z/∂μ`, so the `M_impl` channel is
/// live (a bare-`M` carry would silently mis-weight the wrench response).
#[test]
fn damping_changes_the_gradient() {
    let n = 6;
    let grad = |mjcf: &str| {
        let (_t, gm) = build(mjcf, MU0).coupled_trajectory_material_gradient_articulated(n, 0);
        let (_t2, gl) = build(mjcf, MU0).coupled_trajectory_material_gradient_articulated(n, 1);
        gm + 4.0 * gl
    };
    let g_damped = grad(DAMPED_HINGE);
    let g_undamped = grad(UNDAMPED_HINGE);
    let rel = (g_damped - g_undamped).abs() / g_undamped.abs().max(1e-30);
    println!("damped={g_damped:.6e} undamped={g_undamped:.6e} rel-diff={rel:.3e}");
    assert!(
        rel > 1e-3,
        "joint damping must change the gradient materially (rel diff {rel:.3e})"
    );
}
