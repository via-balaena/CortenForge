//! Keystone S5 (PR3) — the co-design gradient: `∂vz'/∂(material)` via ONE
//! `tape.backward` across both engines.
//!
//! `StaggeredCoupling::coupled_step_material_gradient` routes the soft block's
//! Neo-Hookean material parameter through the SAME S4 crossing the load
//! gradient uses — only the soft node differs (the reverse-mode
//! `MaterialStepVjp` in place of the load adjoint):
//! `p →[MaterialStepVjp] x* →[ContactForceVjp] fz →[neg] xfrc →[RigidStepVjp] vz'`.
//! `tape.backward(vz')` flows the cotangent back to `p`. A stiffer soft body
//! deforms less under the platen, changing the contact force hence the platen's
//! motion — the gradient the co-design optimizer consumes.
//!
//! The gate is INDEPENDENT: the oracle (`coupled_step_material_vz`) re-runs the
//! full nonlinear coupled step (re-solve the soft Newton problem with the
//! perturbed material → contact force → rigid step) and central-differences
//! `vz'`, touching none of the tape / VjpOp machinery.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

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
const LAMBDA0: f64 = 1.2e5; // = 4·μ

fn coupling() -> StaggeredCoupling {
    let model = load_model(PLATEN_MJCF).expect("MJCF");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, MU0, 1.0e-3, 3.0e4, 1.0e-2, 12.0,
    )
}

#[test]
fn material_cross_engine_backward_matches_full_coupled_fd() {
    let c = coupling();
    let h = 0.099; // deeply-engaged: 25 top vertices active, stable

    // ∂vz'/∂μ (param 0, λ held) and ∂vz'/∂λ (param 1, μ held).
    for (idx, p0, name) in [(0usize, MU0, "μ"), (1usize, LAMBDA0, "λ")] {
        let (vz, grad) = c.coupled_step_material_gradient(h, idx);
        let eps = p0 * 1e-6;
        let fd = (c.coupled_step_material_vz(h, idx, p0 + eps)
            - c.coupled_step_material_vz(h, idx, p0 - eps))
            / (2.0 * eps);
        let rel = (grad - fd).abs() / fd.abs();
        eprintln!(
            "S5 co-design ∂vz'/∂{name}: vz'={vz:.6e}  tape={grad:.6e}  full-coupled FD={fd:.6e}  rel={rel:.3e}"
        );
        assert!(vz.is_finite(), "vz' not finite");
        assert!(
            fd.abs() > 1e-12,
            "degenerate gate: ∂vz'/∂{name} ≈ 0 ({fd}) — material not coupling to vz'?"
        );
        assert!(
            rel < 1e-5,
            "tape ∂vz'/∂{name} disagrees with full-coupled FD (rel {rel:.3e}): {grad} vs {fd}"
        );
    }
}
