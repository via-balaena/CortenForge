//! With friction set, the coupling's normal-only gradients panic; without it they run.
//!
//! These methods build the soft step's VJP without `x_prev`, and sim-soft panics on that
//! when friction is set (`SolverConfig::friction_mu`). The friction-aware gradients are
//! the `coupled_trajectory_tangential_*` family, checked in `tests/coupling_grad_harness.rs`.
//! This file covers the free-platen methods; it checks only whether a call panics.

// let_underscore_must_use: each call is made only to see whether it panics, so its result
// is discarded on purpose.
#![allow(clippy::expect_used, clippy::let_underscore_must_use)]

use std::panic::{AssertUnwindSafe, catch_unwind};

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

const WITHOUT_X_PREV: &str = "friction-exact gradient requested without x_prev";

/// The free platen resting on the soft block (the `coupling_grad_harness.rs` grip scene).
fn platen(friction_mu: f64) -> StaggeredCoupling {
    const MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.115">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
    let model = load_model(MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 8.0,
    )
    .with_friction(friction_mu, 0.1)
}

/// Runs each method on a fresh platen and returns its name with its panic message.
fn outcomes(friction_mu: f64) -> Vec<(&'static str, Option<String>)> {
    let calls: [(&'static str, fn(&mut StaggeredCoupling)); 4] = [
        ("coupled_step_material_gradient", |c| {
            let _ = c.coupled_step_material_gradient(0.099, 0);
        }),
        ("coupled_trajectory_material_gradient", |c| {
            let _ = c.coupled_trajectory_material_gradient(2, 0);
        }),
        ("coupled_trajectory_peak_force_gradient", |c| {
            let _ = c.coupled_trajectory_peak_force_gradient(2, 0);
        }),
        ("coupled_trajectory_control_gradient", |c| {
            let _ = c.coupled_trajectory_control_gradient(&[0.0, 0.0]);
        }),
    ];
    calls
        .into_iter()
        .map(|(name, call)| {
            let mut coupling = platen(friction_mu);
            let panic = catch_unwind(AssertUnwindSafe(|| call(&mut coupling)))
                .err()
                .map(|e| {
                    e.downcast_ref::<String>()
                        .cloned()
                        .or_else(|| e.downcast_ref::<&str>().map(|s| (*s).to_string()))
                        .unwrap_or_default()
                });
            (name, panic)
        })
        .collect()
}

#[test]
fn normal_only_gradients_panic_with_friction() {
    for (name, panic) in outcomes(1.0) {
        let msg = panic.as_deref().unwrap_or_default();
        assert!(
            msg.contains(WITHOUT_X_PREV),
            "{name} should panic for want of x_prev, got {panic:?}"
        );
    }
}

/// The control: the same calls on the same scene run with friction off.
#[test]
fn normal_only_gradients_run_without_friction() {
    for (name, panic) in outcomes(0.0) {
        assert!(panic.is_none(), "{name} panicked: {panic:?}");
    }
}
