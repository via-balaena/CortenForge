//! The actuator CONTROL gradient THROUGH the friction grip — the powered-exo substrate (a
//! controlled articulated limb actively gripping a soft buffer via friction). Extends the
//! normal-only `coupled_trajectory_actuator_gradient` (#322/#323) with the friction machinery
//! from #402/#403 (the gripped wrench + articulated drift), on the same tape as the actuator
//! control channel `s' = J_state·s + G·w + G_act·u`.
//!
//! Gates: a smoke for the gripped-actuated forward oracle `coupled_trajectory_actuated_gripped_x`
//! (control + friction both live), then the one-tape control gradient `d(tip_x)/du_k` vs a
//! per-control central FD of that oracle — MACHINE-EXACT at single-hinge (n = 5/20/40, rel ~2e-11,
//! the analytic friction-loaded carry) AND a 2-link chain (rel ~7e-10, the analytic
//! `chain_state_jacobian` under friction + actuator load). The motor control is a strong,
//! well-conditioned lever on tip_x, so the FD has no mu_c-style cancellation; `eps = 1e-3`.

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

#[test]
fn actuated_gripped_engages_and_stays_finite() {
    let controls = [0.05_f64; 20];
    let tip = build(true).coupled_trajectory_actuated_gripped_x(&controls);
    assert!(
        tip.x.is_finite() && (0.10..0.12).contains(&tip.z),
        "actuated grip rollout must stay finite and engaged, got {tip:?}"
    );
}

#[test]
fn control_and_friction_are_live() {
    // The motor control changes the tip's tangential drag (the actuator drives the gripped limb).
    let pos = build(true)
        .coupled_trajectory_actuated_gripped_x(&[0.08_f64; 20])
        .x;
    let neg = build(true)
        .coupled_trajectory_actuated_gripped_x(&[-0.08_f64; 20])
        .x;
    assert!(
        (pos - neg).abs() > 1e-4,
        "the motor control must move the tip (Δx = {:e}); the actuator path looks inert",
        (pos - neg).abs()
    );
    // Friction is live: at the same control, the gripped rollout differs from frictionless.
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

/// The one-tape control gradient `∂tip_x/∂u_k` vs a per-control central FD of the independent
/// gripped-actuated rollout. Single hinge ⇒ analytic friction-loaded carry (machine-exact).
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

#[test]
fn actuator_friction_gradient_matches_fd_n5() {
    gate_single(5);
}

#[test]
fn actuator_friction_gradient_matches_fd_n20() {
    gate_single(20);
}

#[test]
fn actuator_friction_gradient_matches_fd_n40() {
    gate_single(40);
}

// A 2-link hinge CHAIN (nv = 2) with the motor on the proximal joint — the gripped-actuated
// gradient on a genuine multi-DOF topology (the loaded `J_state` under friction + actuator force).
fn scene_2link() -> &'static str {
    r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="upper" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.025" size="0.004" mass="0.3"/>
      <body name="lower" pos="0 0 -0.05">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="sphere" pos="0 0 -0.04" size="0.004" mass="0.4"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="j" gear="1"/>
  </actuator>
</mujoco>"#
}

fn build_2link() -> StaggeredCoupling {
    let model = load_model(scene_2link()).expect("2-link actuator scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.1;
    data.qpos[1] = -0.05;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 2, 0.005, 4, 0.1, SOFT_MU, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_friction(FRIC_MU, EPS_V)
}

#[test]
fn actuator_friction_gradient_2link_matches_fd() {
    let z0 = build_2link().data().xipos[2].z;
    assert!(z0 > 0.10, "2-link tip must start engaged, got {z0}");
    for n in [3usize, 12] {
        let controls = vec![0.03_f64; n];
        let (_t, grad) = build_2link().coupled_trajectory_actuator_friction_gradient(&controls);
        let eps = 1e-3;
        let mut max_rel = 0.0_f64;
        for k in 0..n {
            let mut cp = controls.clone();
            let mut cm = controls.clone();
            cp[k] += eps;
            cm[k] -= eps;
            let fd = (build_2link().coupled_trajectory_actuated_gripped_x(&cp).x
                - build_2link().coupled_trajectory_actuated_gripped_x(&cm).x)
                / (2.0 * eps);
            assert!(
                fd.abs() > 1e-9,
                "2-link n={n}: ∂tip_x/∂u_{k} should be nonzero (fd={fd:e})"
            );
            max_rel = max_rel.max((grad[k] - fd).abs() / fd.abs().max(1e-9));
        }
        // The UNDAMPED 2-link chain routes through the analytic `chain_state_jacobian` carry
        // (machine-exact under the friction + actuator load), NOT the FD fallback — observed
        // worst rel ~7e-10.
        assert!(
            max_rel < 1e-5,
            "2-link n={n}: actuator-friction gradient vs gripped FD, worst rel {max_rel:.3e}"
        );
    }
}
