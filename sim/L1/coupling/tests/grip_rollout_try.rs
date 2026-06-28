//! PR1 gate — the fallible grip rollout surfaces the soft solver's fail-close as a typed
//! [`RolloutError`] instead of a panic.
//!
//! On a stiff-contact design/policy the co-design optimizer explores — an aggressive holding gain
//! pressing the finite end-effector deep into the coarse buffer — the per-step soft solve fails to
//! converge (a *genuinely infeasible* equilibrium: raising the Newton cap converts it to an Armijo
//! stall at a plateau residual; the deformation gradient never inverts). The panic-path gradient
//! methods fail-close with a panic (matching the soft solver's contract); the `try_`-prefixed
//! siblings return [`RolloutError`] so `cf_codesign::OptConfig::reject_infeasible` can skip the point
//! without a `catch_unwind`.
//!
//! Gates:
//! 1. an extreme holding policy makes `try_..._hold_gradient` (and the friction sibling) return
//!    `Err(RolloutError)` tagged with the failing rollout step — NOT a panic, NOT a bogus `Ok`;
//! 2. the panic-path sibling STILL panics on the same infeasible design (fail-closed contract intact);
//! 3. on a feasible design the `try_` `Ok` is byte-identical to the panic path (pure-refactor success
//!    path — same tape, same backward).

use sim_coupling::{LinearFeedback, StaggeredCoupling};
use sim_mjcf::load_model;
use sim_soft::SolverFailure;

// The moving-EE friction grip (the R5 scene): the contact sphere tracks the arm-tip geom, pivot
// above the block, a sideways gravity sweeping the tip tangentially into the buffer.
const MOVING_EE: &str = r#"<mujoco>
  <option gravity="1.5 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0.05 0.05 0.344">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.17" size="0.08" mass="0.2"/>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#;

// Mirrors `design_policy_hold_gradient.rs`'s `build` for the moving-EE case.
fn build(mu: f64) -> StaggeredCoupling {
    let model = load_model(MOVING_EE).expect("scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.05;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(0.08)
    .with_contact_geom(0)
    .with_friction(2.5, 0.1)
}

// An aggressive holding policy + soft buffer that tears the coarse mesh into a non-convergent
// contact problem (calibrated by a θ/μ sweep of this exact scene; the natural |θ| ≤ 10 grips all
// solve — only this extreme tears, so the feasible region is real, not contrived).
const INFEASIBLE_THETA: [f64; 3] = [50.0, 0.0, 50.0];
const INFEASIBLE_MU: f64 = 2.0e3;
const TEAR_N_STEPS: usize = 8;

/// Gate 1 (Hold): the infeasible design returns `Err(RolloutError)` naming the failing step.
#[test]
fn try_hold_gradient_returns_err_on_infeasible_design() {
    let err = build(INFEASIBLE_MU)
        .try_coupled_trajectory_design_policy_hold_gradient(
            &LinearFeedback,
            &INFEASIBLE_THETA,
            TEAR_N_STEPS,
            0.0,
        )
        .expect_err("aggressive holding policy must tear the buffer → Err, not Ok");
    assert!(
        err.step < TEAR_N_STEPS,
        "the failing step index must be within the rollout, got {} (n_steps {TEAR_N_STEPS})",
        err.step,
    );
    // Pin the mechanism: the grip's stiff-contact fail-close is the soft Newton iter-cap (NOT a
    // misfired precondition — those panic, which `expect_err` would surface as a test failure, not an
    // `Err` — and NOT an Armijo stall, which would still panic through the `try_` path today).
    assert!(
        matches!(err.failure, SolverFailure::NewtonIterCap { .. }),
        "expected a Newton iter-cap fail-close, got {:?}",
        err.failure,
    );
    let msg = err.to_string();
    assert!(
        msg.contains("coupled grip rollout failed at step"),
        "Display should name the failing step, got: {msg}",
    );
}

/// Gate 1 (friction/terminal-tip_x): the SAME infeasible design tears the shared tape → `Err`.
#[test]
fn try_friction_gradient_returns_err_on_infeasible_design() {
    let err = build(INFEASIBLE_MU)
        .try_coupled_trajectory_design_policy_friction_gradient(
            &LinearFeedback,
            &INFEASIBLE_THETA,
            TEAR_N_STEPS,
        )
        .expect_err("aggressive policy must tear the buffer → Err, not Ok");
    assert!(
        err.step < TEAR_N_STEPS,
        "failing step within rollout: {}",
        err.step
    );
    assert!(
        matches!(err.failure, SolverFailure::NewtonIterCap { .. }),
        "expected a Newton iter-cap fail-close, got {:?}",
        err.failure,
    );
}

/// Gate 2: the panic-path sibling STILL fail-closes (panics) on the same infeasible design — the
/// `try_` path widens the recoverable surface without weakening the panic-path contract.
#[test]
fn panic_path_hold_gradient_still_panics_on_infeasible() {
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let out = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        build(INFEASIBLE_MU).coupled_trajectory_design_policy_hold_gradient(
            &LinearFeedback,
            &INFEASIBLE_THETA,
            TEAR_N_STEPS,
            0.0,
        )
    }));
    std::panic::set_hook(prev);
    assert!(
        out.is_err(),
        "the panic-path method must still fail-closed (panic) on an infeasible design",
    );
}

/// Gate 3: on a feasible design the `try_` `Ok` is byte-identical to the panic path. Since the panic
/// path now delegates to `try_`, this guards the *delegation* (no divergence between the two
/// surfaces), not the numerics. The substantive guarantee — that swapping `replay_step` →
/// `try_replay_step` preserved the gradient values — rests on the pre-existing FD gradient gates
/// (`design_policy_hold_gradient.rs` / `design_policy_friction_gradient.rs`), which now route through
/// `try_replay_step` and still match their finite differences.
#[test]
fn try_hold_gradient_ok_matches_panic_path_when_feasible() {
    let params = [0.05_f64, -0.02, 0.01];
    let (c0, m0, t0) = build(3.0e3).coupled_trajectory_design_policy_hold_gradient(
        &LinearFeedback,
        &params,
        5,
        0.25,
    );
    let (c1, m1, t1) = build(3.0e3)
        .try_coupled_trajectory_design_policy_hold_gradient(&LinearFeedback, &params, 5, 0.25)
        .expect("feasible design must solve");
    assert_eq!(c0.to_bits(), c1.to_bits(), "cost byte-identical");
    assert_eq!(m0.to_bits(), m1.to_bits(), "μ-grad byte-identical");
    assert_eq!(t0.len(), t1.len(), "θ-grad length");
    for (k, (a, b)) in t0.iter().zip(&t1).enumerate() {
        assert_eq!(a.to_bits(), b.to_bits(), "θ-grad[{k}] byte-identical");
    }
}

// A SHARP fist (small sphere) deeply indenting the coarse buffer over-stretches a local tet past the
// material's validity domain — the OTHER stiff-contact fail-close mode (distinct from the Newton
// iter-cap above). Now a typed `SolverFailure::ValidityViolation` (the validity-as-Err completion),
// so the `try_` rollout returns it as `RolloutError` instead of panicking.
const VALIDITY_TEAR: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0.05 0.05 0.285">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.15" size="0.06" mass="0.5"/>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#;

fn build_validity_tear(mu: f64) -> StaggeredCoupling {
    let model = load_model(VALIDITY_TEAR).expect("scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.15; // the fist presses in
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(0.06)
    .with_contact_geom(0)
    .with_friction(2.5, 0.1)
}

/// Validity tear at the coupling layer: a sharp-fist grip that over-stretches a tet returns
/// `Err(RolloutError { failure: SolverFailure::ValidityViolation, .. })` — NOT a panic. The
/// structural sibling of `try_hold_gradient_returns_err_on_infeasible_design` (which pins the
/// `NewtonIterCap` failure mode), so both stiff-contact fail-close variants are gated at this layer.
#[test]
fn try_hold_gradient_returns_err_on_validity_tear() {
    let err = build_validity_tear(4.0e3)
        .try_coupled_trajectory_design_policy_hold_gradient(
            &LinearFeedback,
            &[0.0, 0.0, 0.0],
            6,
            0.0,
        )
        .expect_err("a tet-over-stretching grip must surface as Err, not panic");
    assert!(
        matches!(err.failure, SolverFailure::ValidityViolation { .. }),
        "expected a validity-domain fail-close, got {:?}",
        err.failure,
    );
}
