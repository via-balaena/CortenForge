//! Keystone S4 (PR1) — `RigidStepVjp`: the rigid engine as a chassis tape op.
//!
//! The `sim-core` rigid engine has no reverse-mode tape; its sensitivity is a
//! dense Jacobian. [`RigidStepVjp`] adapts the single scalar factor
//! `∂vz'/∂xfrc_z` into the chassis `Tape` so a `Tape::backward` flowing into a
//! rigid-step node continues back through the applied force — the rigid half of
//! the soft↔rigid crossing.
//!
//! Gates: (1) `rigid_vz_response`'s FD factor equals the closed-form free-body
//! semi-implicit-Euler `dt/m`; (2) a tiny tape `xfrc → RigidStepVjp → vz'`
//! recovers that factor via `backward` — the VjpOp wiring.

#![allow(clippy::expect_used)]

use sim_coupling::{RigidStepVjp, StaggeredCoupling};
use sim_mjcf::load_model;
use sim_ml_chassis::{Tape, Tensor};

const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.125">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const DT: f64 = 1.0e-3;
const MASS: f64 = 0.2;

fn coupling() -> StaggeredCoupling {
    let model = load_model(PLATEN_MJCF).expect("MJCF");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, DT, 3.0e4, 1.0e-2, 12.0,
    )
}

#[test]
fn rigid_step_vjp_matches_dt_over_m() {
    let c = coupling();
    let analytic_dt_m = DT / MASS; // 5e-3, semi-implicit-Euler free body (S2)

    // (1) the FD response factor equals dt/m (free body is exactly affine).
    let (vz, dvz_dfz) = c.rigid_vz_response(5.0);
    eprintln!(
        "rigid_vz_response: vz'={vz:.6e}  ∂vz'/∂fz(FD)={dvz_dfz:.6e}  dt/m={analytic_dt_m:.6e}"
    );
    assert!(
        (dvz_dfz - analytic_dt_m).abs() / analytic_dt_m < 1e-9,
        "FD factor {dvz_dfz} != dt/m {analytic_dt_m}"
    );

    // (2) the VjpOp recovers the factor through a tape backward: a 1-node tape
    // xfrc → RigidStepVjp → vz', seed ∂L/∂vz'=1, expect grad(xfrc)=∂vz'/∂fz.
    let mut tape = Tape::new();
    let xfrc = tape.param_tensor(Tensor::from_slice(&[5.0], &[1]));
    let vz_var = tape.push_custom(
        &[xfrc],
        Tensor::from_slice(&[vz], &[1]),
        Box::new(RigidStepVjp::new(dvz_dfz)),
    );
    tape.backward(vz_var);
    let grad = tape.grad_tensor(xfrc).as_slice()[0];
    assert!(
        (grad - dvz_dfz).abs() < 1e-15,
        "RigidStepVjp backward grad {grad} != factor {dvz_dfz}"
    );
}
