//! Actuator CONTROL gradient THROUGH the friction grip — the invariants the coupling gradient
//! harness can't express.
//!
//! The per-horizon FD cells of `StaggeredCoupling::coupled_trajectory_actuator_friction_gradient`
//! (the powered-exo substrate: a `<motor>`-driven limb gripping the soft buffer via friction,
//! `∂tip_x/∂u_k`) are folded into the `hinge·actuator-friction[motor]` and
//! `chain·actuator-friction[motor]` rows of `tests/coupling_grad_harness.rs`, which assert the same
//! per-control FD match against the same `coupled_trajectory_actuated_gripped_x(controls).x` oracle
//! (eps 1e-3), plus forward-consistency and a `Comp::Live` non-vacuity floor.
//!
//! What those single-horizon rows CAN'T fold, kept here:
//! - the **single-hinge multi-horizon machine-exactness** — the friction-loaded control gradient
//!   stays FD-limited (rel ~2e-11) as the rollout lengthens (n = 5, 20, 40), with the tip staying
//!   engaged near the block top (the z-band the tip-`x` loss can't express);
//! - the **friction-ON-vs-OFF materiality** — at the same control the gripped rollout differs from
//!   the frictionless one, so the friction path is genuinely live (a toggle the matrix, which only
//!   ever builds the friction scene, can't express).

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// A single-hinge arm with a motor actuator driving joint `j`, pushed sideways by tilted gravity
// (so the tip arcs into the block top z = 0.1 and sweeps tangentially — a controlled friction grip).
fn scene() -> &'static str {
    r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="j" gear="1"/>
  </actuator>
</mujoco>"#
}

const SOFT_MU: f64 = 3.0e3;
const FRIC_MU: f64 = 2.5;
const EPS_V: f64 = 0.1;

fn build(friction: bool) -> StaggeredCoupling {
    let model = load_model(scene()).expect("actuator scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");
    let c = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, SOFT_MU, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    );
    if friction {
        c.with_friction(FRIC_MU, EPS_V)
    } else {
        c
    }
}

/// The one-tape control gradient `∂tip_x/∂u_k` vs a per-control central FD of the independent
/// gripped-actuated rollout, plus a forward-match and an engagement check. Single hinge ⇒ analytic
/// friction-loaded carry (machine-exact).
fn gate_single(n: usize) {
    let controls = vec![0.03_f64; n];
    let (tip_x, grad) = build(true).coupled_trajectory_actuator_friction_gradient(&controls);
    let oracle = build(true).coupled_trajectory_actuated_gripped_x(&controls);
    assert!(
        (tip_x - oracle.x).abs() < 1e-12,
        "n={n}: tape forward tip_x {tip_x} != gripped-actuated rollout {}",
        oracle.x
    );
    assert!(
        (0.10..0.12).contains(&oracle.z),
        "n={n}: arm should stay engaged near the block top, got z={}",
        oracle.z
    );
    let eps = 1e-3;
    let mut max_rel = 0.0_f64;
    for k in 0..n {
        let mut cp = controls.clone();
        let mut cm = controls.clone();
        cp[k] += eps;
        cm[k] -= eps;
        let fd = (build(true).coupled_trajectory_actuated_gripped_x(&cp).x
            - build(true).coupled_trajectory_actuated_gripped_x(&cm).x)
            / (2.0 * eps);
        assert!(
            fd.abs() > 1e-9,
            "n={n}: ∂tip_x/∂u_{k} should be a nonzero, well-posed target (fd={fd:e})"
        );
        max_rel = max_rel.max((grad[k] - fd).abs() / fd.abs().max(1e-9));
    }
    assert!(
        max_rel < 1e-5,
        "n={n}: actuator-friction control gradient must match the gripped FD, worst rel {max_rel:.3e}"
    );
}

/// SINGLE-HINGE multi-horizon machine-exactness (the cross-horizon invariant + z-engagement the
/// single-length matrix rows can't express): the friction-loaded control gradient stays FD-limited
/// across the make and the sweep as the rollout lengthens.
#[test]
fn actuator_friction_gradient_machine_exact_at_all_lengths() {
    for n in [5usize, 20, 40] {
        gate_single(n);
    }
}

/// FRICTION-ON-vs-OFF materiality (the toggle the matrix — which only ever builds the friction
/// scene — can't express): at the same control the gripped rollout differs from the frictionless
/// one, so the friction path is genuinely live rather than a silent pass-through.
#[test]
fn friction_toggle_changes_the_drag() {
    let with = build(true)
        .coupled_trajectory_actuated_gripped_x(&[0.04_f64; 20])
        .x;
    let without = build(false)
        .coupled_trajectory_actuated_gripped_x(&[0.04_f64; 20])
        .x;
    assert!(
        (with - without).abs() > 1e-6,
        "friction must change the tangential drag (Δx = {:e}); the grip path looks inert",
        (with - without).abs()
    );
}
