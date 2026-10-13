//! Activations against MuJoCo 3.5.0: a group-disabled actuator has its
//! `act_dot` and no force (`mj_fwdActuation`, `engine_forward.c:321-372`), and
//! a filter's time constant is clamped at mjMINVAL (`:352`;
//! `mj_nextActivation`, `engine_support.c:718`).
//!
//! The golden is `assets/golden/activation/activation.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_activation_reference.py`, which describes
//! its fixtures).

use serde_json::Value;
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/activation/activation.json"))
        .expect("activation golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn assert_close(what: &str, got: &[f64], want: &[f64]) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    for (a, b) in got.iter().zip(want) {
        assert!(
            (a - b).abs() <= 1e-12 * b.abs().max(1.0),
            "{what}: ours {a}, MuJoCo {b}"
        );
    }
}

/// Fixture `name` from a control of 1: `act_dot` and `actuator_force` after
/// a forward pass, and `act`, `act_dot`, `qpos` and `qvel` after each step,
/// within 1e-12 of MuJoCo's (relative above 1).
fn assert_matches_mujoco(name: &str) {
    let golden = golden();
    let case = golden["fixtures"]
        .as_array()
        .expect("fixtures")
        .iter()
        .find(|c| c["name"] == name)
        .unwrap_or_else(|| panic!("no fixture {name}"));
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward");
    let forward = &case["forward"];
    assert_close(
        &format!("{name}: forward act_dot"),
        data.act_dot.as_slice(),
        &floats(&forward["act_dot"]),
    );
    assert_close(
        &format!("{name}: forward actuator_force"),
        data.actuator_force.as_slice(),
        &floats(&forward["actuator_force"]),
    );
    for (k, step) in case["steps"].as_array().expect("steps").iter().enumerate() {
        data.step(&model).expect("step");
        for (field, got) in [
            ("act", data.act.as_slice()),
            ("act_dot", data.act_dot.as_slice()),
            ("qpos", data.qpos.as_slice()),
            ("qvel", data.qvel.as_slice()),
        ] {
            assert_close(
                &format!("{name}: {field} after step {k}"),
                got,
                &floats(&step[field]),
            );
        }
    }
}

/// A group-disabled filter has its `act_dot` (5.0) and no force; the advance
/// leaves its activation at 0. Under RK4 its stages advance the activation
/// with that `act_dot`, as MuJoCo's take it, so the step's `act_dot`, the
/// last stage's, is 4.95.
#[test]
fn group_disabled_actuator_has_its_act_dot() {
    assert_matches_mujoco("disabled_Euler");
    assert_matches_mujoco("disabled_RK4");
}

/// A filter's time constant has MuJoCo's floor, mjMINVAL, not 1e-10. One of
/// 1e-12 is used as it is: `act_dot` 1e12, and after one step a filter's
/// activation 2e9 and an exact filter's 1; under RK4, whose stages take no
/// exact filter, the exact filter's runs away (-3.3e26), MuJoCo's too. One of
/// 1e-16 is raised to 1e-15: `act_dot` 1e15, and under RK4 the exact filter's
/// activation -3.3e35 after a step (its gear 0).
#[test]
fn filter_time_constant_floor_is_mjminval() {
    assert_matches_mujoco("tiny_tau");
    assert_matches_mujoco("tiny_tau_exact");
    assert_matches_mujoco("tiny_tau_exact_RK4");
    assert_matches_mujoco("tau_floor");
    assert_matches_mujoco("tau_floor_exact");
    assert_matches_mujoco("tau_floor_exact_RK4");
}

/// Muscle activation dynamics blend the two time constants through the
/// sigmoid of `dctrl / width + 0.5` (`mju_muscleDynamics`,
/// `engine_util_misc.c:733-770`): a width of 0.3, 60 steps from rest, the
/// activation passing through the width.
#[test]
fn muscle_activation_smoothing_matches_mujoco_3_5_0() {
    assert_matches_mujoco("muscle_smooth");
}

/// A muscle's activation time constant has MuJoCo's floor, mjMINVAL, not
/// 1e-10: 5e-14 is used as it is (`act_dot` 2e13), 5e-17 is raised to 1e-15
/// (`act_dot` 1e15).
#[test]
fn muscle_time_constant_floor_is_mjminval() {
    assert_matches_mujoco("muscle_tiny_tau");
    assert_matches_mujoco("muscle_tau_floor");
}

/// With actuation disabled as a whole, MuJoCo's `mj_advance` leaves the
/// activations where they are, unclamped (`engine_forward.c:886-895`), and
/// `act_dot` keeps the value last written: a limited filter (actrange 0.5 1)
/// at activation 0 stays at 0 under Euler, RK4 and implicitfast. Under RK4
/// the stages advance it with that `act_dot` and the step restores it.
#[test]
fn actuation_disabled_leaves_activations_alone() {
    let golden = golden();
    for case in golden["actuation_off"].as_array().expect("actuation_off") {
        let name = case["name"].as_str().expect("name");
        let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.ctrl[0] = 1.0;
        data.forward(&model).expect("forward");
        model.disableflags |= sim_core::DISABLE_ACTUATION;
        for (k, step) in case["steps"].as_array().expect("steps").iter().enumerate() {
            data.step(&model).expect("step");
            for (field, got) in [
                ("act", data.act.as_slice()),
                ("act_dot", data.act_dot.as_slice()),
                ("qpos", data.qpos.as_slice()),
                ("qvel", data.qvel.as_slice()),
            ] {
                assert_close(
                    &format!("{name}: {field} after step {k}"),
                    got,
                    &floats(&step[field]),
                );
            }
        }
    }
}
