//! Keystone damped-joints leaf (PR1) — the coupled articulated trajectory gradient
//! through joints with MuJoCo `damping=` (the Euler `eulerdamp` velocity coupling).
//!
//! Under the default Euler integrator, `eulerdamp` solves `(M + Δt·D)·qacc = F` then
//! `qvel += Δt·qacc`, so the contact wrench reaches `qvel'` through `M_impl = M + Δt·D`,
//! not the bare `M` the carry used pre-leaf (`rigid_xfrc_column`; and the analytic
//! single-hinge `J_state`'s geometric-stiffness denominator). Bare `M` is materially
//! wrong under damping (≈28% on the hinge, ≈10× on the chain — the S0 spike).
//!
//! **Matched pair.** The FD oracle `coupled_trajectory_articulated_z` steps the REAL
//! engine, whose Euler `step` runs eulerdamp — so it is automatically damping-correct.
//! The gate is the one-tape gradient vs a central FD of that oracle, at the same
//! damping. The single HINGE uses the ANALYTIC damped `J_state` (the `M → M_impl`
//! correction reconciles `A`'s bare-`M` velocity rows with eulerdamp — machine-exact,
//! ~1e-9). The 2-link CHAIN still uses the FD `J_state` under damping (FD-carry precision
//! ~1e-6); the analytic multi-link damped carry is the documented follow-on.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

const MU0: f64 = 3.0e4;

// The single-hinge keystone scene + joint damping (the arm settles faster but stays
// engaged on the block top). Uses the analytic damped `J_state` (the `M → M_impl` fix).
const DAMPED_HINGE: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0" damping="0.5"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

// A 2-link chain with per-joint damping — off-diagonal M, so the M + Δt·D coupling is
// genuinely live across joints (the FD J_state path under damping).
const DAMPED_2LINK: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="upper" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0" damping="0.5"/>
      <geom type="sphere" pos="0 0 -0.025" size="0.004" mass="0.3"/>
      <body name="lower" pos="0 0 -0.05">
        <joint type="hinge" axis="0 1 0" damping="0.3"/>
        <geom type="sphere" pos="0 0 -0.04" size="0.004" mass="0.4"/>
      </body>
    </body>
  </worldbody>
</mujoco>"#;

fn build(mjcf: &str, mu: f64, body: usize, seed: &[(usize, f64)]) -> StaggeredCoupling {
    let model = load_model(mjcf).expect("damped model loads");
    let mut data = model.make_data();
    for &(i, q) in seed {
        data.qpos[i] = q;
    }
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, body, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

/// One-tape gradient vs central FD of the (damping-correct) full-coupled oracle.
fn tape_vs_fd(mjcf: &str, body: usize, seed: &[(usize, f64)], n: usize) -> (f64, f64, f64) {
    let b = || build(mjcf, MU0, body, seed);
    let (tip_z, gm) = b().coupled_trajectory_material_gradient_articulated(n, 0);
    let (_t, gl) = b().coupled_trajectory_material_gradient_articulated(n, 1);
    let total = gm + 4.0 * gl;
    let z_oracle = b().coupled_trajectory_articulated_z(n);
    assert!(
        (tip_z - z_oracle).abs() < 1e-12,
        "n={n}: tape forward tip_z {tip_z} != real rollout {z_oracle}"
    );
    let eps = MU0 * 1e-4;
    let zp = build(mjcf, MU0 + eps, body, seed).coupled_trajectory_articulated_z(n);
    let zm = build(mjcf, MU0 - eps, body, seed).coupled_trajectory_articulated_z(n);
    let fd = (zp - zm) / (2.0 * eps);
    (total, fd, (total - fd).abs() / fd.abs().max(1e-30))
}

/// The damped single hinge matches the full-coupled FD over long, sustained
/// engagement. It uses the ANALYTIC damped `J_state` (the `M → M_impl` correction), so the
/// per-step Jacobian is machine-exact and the gradient tightens to ~1e-9 (the FD oracle
/// caps the gate). The `M_impl` `G_vel` fix makes the wrench response correct too (bare `M`
/// would be ≈28% wrong, the S0 spike).
#[test]
fn damped_hinge_gradient_matches_fd() {
    let seed = [(0usize, 0.3)];
    for n in [2usize, 6, 10] {
        let (total, fd, rel) = tape_vs_fd(DAMPED_HINGE, 1, &seed, n);
        println!("damped-hinge n={n}: tape={total:.6e} FD={fd:.6e} rel={rel:.3e}");
        assert!(
            total.abs() > 1e-12 && rel < 1e-5,
            "damped hinge gradient must match full-coupled FD at n={n}, got rel {rel:.3e}"
        );
    }
}

/// The damped 2-link chain: FD `J_state` carry (FD-carry precision) — the off-diagonal
/// `M + Δt·D` coupling exercised across both joints.
#[test]
fn damped_2link_gradient_matches_fd() {
    let seed = [(0usize, 0.1), (1usize, -0.05)];
    for n in [2usize, 6, 10] {
        let (total, fd, rel) = tape_vs_fd(DAMPED_2LINK, 2, &seed, n);
        println!("damped-2link n={n}: tape={total:.6e} FD={fd:.6e} rel={rel:.3e}");
        assert!(
            total.abs() > 1e-12 && rel < 1e-5,
            "damped 2-link gradient must match full-coupled FD at n={n}, got rel {rel:.3e}"
        );
    }
}

/// Materiality: damping genuinely changes the gradient — the same scene with vs without
/// joint damping gives a measurably different `∂tip_z/∂μ`, so the `M_impl` channel is
/// live (a bare-`M` carry would silently mis-weight the wrench response).
#[test]
fn damping_changes_the_gradient() {
    const UNDAMPED_HINGE: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
    let n = 6;
    let grad = |mjcf: &str| {
        let (_t, gm) =
            build(mjcf, MU0, 1, &[(0, 0.3)]).coupled_trajectory_material_gradient_articulated(n, 0);
        let (_t2, gl) =
            build(mjcf, MU0, 1, &[(0, 0.3)]).coupled_trajectory_material_gradient_articulated(n, 1);
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
