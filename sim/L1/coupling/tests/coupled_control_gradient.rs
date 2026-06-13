//! Keystone control gradient — the policy half's substrate.
//!
//! `StaggeredCoupling::coupled_trajectory_control_gradient` rolls the coupled
//! soft↔rigid system forward applying a per-step vertical control force to the
//! platen, on ONE chassis tape, then a single `tape.backward(z_N)` gives the
//! gradient of the platen's final height w.r.t. EVERY control input
//! `[∂z_N/∂u_0 … ∂z_N/∂u_{N−1}]` — the gradient the policy/control half of the
//! co-design optimizer consumes.
//!
//! A control force adds to the same `xfrc_applied` the contact reaction uses, so
//! `∂vz'/∂u_k = +Δt/m` (the free-body factor, opposite sign to the contact
//! term). Each `u_k` is a tape leaf; the rest of the per-step tape is identical
//! to the material-gradient rollout, so the reverse pass captures BOTH paths each
//! control has on `z_N`: the direct rigid push AND the indirect coupled path
//! (the control moves the platen → changes contact penetration → soft
//! re-equilibration → reaction).
//!
//! Gate: the one-tape gradient matches a central FD of the FULL real coupled
//! re-rollout (a fresh coupling rolled at `u_k ± ε` via
//! `coupled_trajectory_control_z` — the real `step`-equivalent loop). That oracle
//! re-runs the real Newton solves + sim-core steps each perturbation, so it is
//! INDEPENDENT (not an affine identity), in the deeply-engaged regime the
//! keystone is scoped to.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// A platen started already in contact (plane at z−clearance = 0.103, the soft
// block's top face at z = 0.1) so the rollout is firmly engaged — the regime the
// gradient is scoped to. Identical to the material trajectory gate's fixture.
const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.108">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const MU0: f64 = 3.0e4;
const DAMPING: f64 = 60.0;

fn build() -> StaggeredCoupling {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    // body=1, clearance, n_per_edge=4, edge=0.1, mu, dt=1e-3, kappa, d_hat, damping.
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, MU0, 1.0e-3, 3.0e4, 1.0e-2, DAMPING,
    )
}

/// Final platen height after the real coupled rollout under `controls`.
fn final_z(controls: &[f64]) -> f64 {
    build().coupled_trajectory_control_z(controls)
}

/// The tape's forward rollout reproduces the real coupled dynamics exactly under
/// a control schedule (the nodes carry real `step` values; only the backward pass
/// is analytic).
#[test]
fn control_trajectory_forward_matches_real_rollout() {
    let controls: Vec<f64> = (0..12)
        .map(|k| if k % 2 == 0 { -1.5 } else { 1.0 })
        .collect();
    let (z_n, _grad) = build().coupled_trajectory_control_gradient(&controls);
    let z_ref = final_z(&controls);
    assert!(
        (z_n - z_ref).abs() < 1e-12,
        "tape forward z_N {z_n} != real rollout {z_ref}"
    );
}

/// One `tape.backward` over the coupled rollout gives EVERY `∂z_N/∂u_k`, each
/// matching the full-coupled FD oracle to machine precision.
#[test]
fn control_gradient_matches_full_fd() {
    // A varied, signed schedule (push-down / push-up around the platen weight
    // ~1.96 N) so every control input is genuinely exercised.
    let controls: Vec<f64> = (0..10)
        .map(|k| if k % 2 == 0 { -1.5 } else { 1.0 })
        .collect();

    let (_z, grad) = build().coupled_trajectory_control_gradient(&controls);
    assert_eq!(grad.len(), controls.len(), "one gradient per control input");

    for (k, &g) in grad.iter().enumerate() {
        let eps = 1e-2;
        let mut up = controls.clone();
        let mut dn = controls.clone();
        up[k] += eps;
        dn[k] -= eps;
        let fd = (final_z(&up) - final_z(&dn)) / (2.0 * eps);
        let rel = (g - fd).abs() / fd.abs().max(1e-30);
        let abs = (g - fd).abs();
        eprintln!("k={k}: tape={g:.6e} fd={fd:.6e} rel={rel:.3e} abs={abs:.3e}");
        // Machine-exact (~1e-11) since the control rides the C⁰ contact force and
        // the off-by-one-free rigid carry; gate at 1e-6 so a glue-Jacobian
        // regression — invisible to the real-step forward rollout — is caught.
        assert!(
            rel < 1e-6 || abs < 1e-12,
            "∂z_N/∂u_{k} tape {g} vs full-coupled FD {fd} (rel {rel:e} abs {abs:e})"
        );
    }
}

/// The final control input has zero effect on `z_N`: sim-core integrates the
/// platen height with the step's STARTING velocity (`z_{k+1} = z_k + Δt·vz_k`),
/// so the velocity bump a control delivers at the last step never integrates into
/// a height before the rollout ends — a physical invariant of the rigid carry
/// (the same off-by-one the `ZCarryVjp` doc records). The penultimate control,
/// by contrast, IS nonzero.
#[test]
fn last_control_has_no_effect_on_final_height() {
    let controls = vec![0.5_f64; 6];
    let (_z, grad) = build().coupled_trajectory_control_gradient(&controls);
    assert!(
        grad.last().expect("nonempty schedule").abs() < 1e-12,
        "last control should not move z_N, got {}",
        grad.last().expect("nonempty")
    );
    assert!(
        grad[grad.len() - 2].abs() > 1e-9,
        "penultimate control should move z_N, got {}",
        grad[grad.len() - 2]
    );
}
