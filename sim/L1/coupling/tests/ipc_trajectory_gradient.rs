//! IPC contact (PR3) — `StaggeredCoupling` runs on `IpcRigidContact`.
//!
//! The coupling is generic over the contact model (penalty default; IPC opt-in via
//! the type parameter). This gate validates that the **single-step** coupled
//! co-design gradient `∂vz'/∂μ` is machine-exact with IPC (the C²-barrier contact),
//! across engagement depths — confirming the generic refactor + IPC contact + the
//! per-pair-curvature gradient factors (PR2) compose correctly for one step.
//!
//! ## Open item (focused follow-up, see `docs/ipc/recon.md` §9)
//! The MULTI-step coupled trajectory gradient with IPC is much better than penalty
//! (penalty degraded 5–25% through marginal/bouncing contact; IPC reaches ~0.3% at
//! a well-chosen κ) but is **not yet machine-clean** in this marginally-engaged
//! falling-platen scene (the platen weight balances the barrier near `d̂`). Single
//! step is machine-exact and the forward rollout is exact, so the residual is a
//! multi-step-only, IPC-only effect under investigation. `diag_ipc_multi_step_residual`
//! (ignored) reproduces it for the debugging pass.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;
use sim_soft::IpcRigidContact;

const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.125">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const MU0: f64 = 3.0e4;
const LAMBDA0: f64 = 4.0 * MU0;

fn build_k(mu: f64, kappa: f64) -> StaggeredCoupling<IpcRigidContact> {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    // body=1, clearance, n_per_edge=4, edge=0.1, mu, dt=1e-3, kappa, d_hat, damping.
    StaggeredCoupling::<IpcRigidContact>::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, kappa, 1.0e-2, 8.0,
    )
}

/// The single-step coupled co-design gradient `∂vz'/∂p` is machine-exact with IPC
/// (the generic coupling + IPC contact + per-pair-curvature factors), validated at
/// an engaged height (`0 < sd < d̂`) against an independent re-solve FD per param.
#[test]
fn ipc_single_step_gradient_matches_fd() {
    let c = build_k(MU0, 3.0e4);
    // Top rest face is at z=0.1; plane at h ⇒ sd = h − 0.1 ∈ (0, d̂=1e-2) is engaged.
    for &h in &[0.103_f64, 0.105, 0.107] {
        // μ (param_idx 0), λ held.
        let gm = c.coupled_step_material_gradient(h, 0).1;
        let em = MU0 * 5e-4;
        let fd_m = (c.coupled_step_material_vz(h, 0, MU0 + em)
            - c.coupled_step_material_vz(h, 0, MU0 - em))
            / (2.0 * em);
        // λ (param_idx 1), μ held.
        let gl = c.coupled_step_material_gradient(h, 1).1;
        let el = LAMBDA0 * 5e-4;
        let fd_l = (c.coupled_step_material_vz(h, 1, LAMBDA0 + el)
            - c.coupled_step_material_vz(h, 1, LAMBDA0 - el))
            / (2.0 * el);

        let rel_m = (gm - fd_m).abs() / fd_m.abs().max(1e-30);
        let rel_l = (gl - fd_l).abs() / fd_l.abs().max(1e-30);
        eprintln!("h={h}: ∂vz'/∂μ rel={rel_m:.3e}  ∂vz'/∂λ rel={rel_l:.3e}");
        assert!(gm.abs() > 1e-9 && rel_m < 1e-6, "∂vz'/∂μ vs FD: {rel_m:e}");
        assert!(gl.abs() > 1e-9 && rel_l < 1e-6, "∂vz'/∂λ vs FD: {rel_l:e}");
    }
}

/// REPRO for the focused follow-up (recon §9): the multi-step IPC trajectory
/// gradient vs full-coupled FD across a κ sweep. Single-step is machine-exact and
/// the forward is exact, yet the composed multi-step gradient retains a residual
/// (best ~0.3% at κ≈3e3; 7% at κ=3e4) in this marginally-engaged scene. Run with
/// `--ignored` during the debugging pass.
#[test]
#[ignore]
fn diag_ipc_multi_step_residual() {
    let n = 90;
    for &kappa in &[3.0e4_f64, 3.0e3, 1.0e3, 3.0e2] {
        let total = build_k(MU0, kappa)
            .coupled_trajectory_material_gradient(n, 0)
            .1
            + 4.0
                * build_k(MU0, kappa)
                    .coupled_trajectory_material_gradient(n, 1)
                    .1;
        let eps = MU0 * 5e-4;
        let roll = |mu: f64| -> f64 {
            let mut c = build_k(mu, kappa);
            let mut z = 0.0;
            for _ in 0..n {
                z = c.step().rigid_z;
            }
            z
        };
        let fd = (roll(MU0 + eps) - roll(MU0 - eps)) / (2.0 * eps);
        eprintln!(
            "κ={kappa:>7.0}: total(tape)={total:.4e} FD={fd:.4e} rel={:.3e}",
            (total - fd).abs() / fd.abs().max(1e-30)
        );
    }
}
