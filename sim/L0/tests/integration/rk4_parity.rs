//! RK4 against MuJoCo 3.5.0: the activations of the stages. MuJoCo's
//! `mj_RungeKutta` takes each stage's activation as the step's start plus `h`
//! times the weighted sum of the earlier stages' `act_dot`, whatever the
//! actuator's dynamics, and clamps nothing (`engine_forward.c:1086-1089`);
//! only the advance takes `mj_nextActivation`.
//!
//! The golden is `assets/golden/rk4/rk4.json`, from the unfused MuJoCo 3.5.0
//! oracle (`scripts/gen_rk4_reference.py`, which describes its fixtures).

use serde_json::Value;
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/rk4/rk4.json")).expect("rk4 golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

/// Step fixture `name` from MuJoCo's controls; `act`, `qpos` and `qvel`
/// after every step within 1e-12 of MuJoCo's.
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
    for (i, c) in floats(&case["ctrl"]).into_iter().enumerate() {
        data.ctrl[i] = c;
    }
    for (k, step) in case["steps"].as_array().expect("steps").iter().enumerate() {
        data.step(&model).expect("step");
        for (field, got) in [
            ("act", data.act.as_slice()),
            ("qpos", data.qpos.as_slice()),
            ("qvel", data.qvel.as_slice()),
        ] {
            let want = floats(&step[field]);
            assert_eq!(got.len(), want.len(), "{name}: {field} length");
            for (i, (a, b)) in got.iter().zip(&want).enumerate() {
                assert!(
                    (a - b).abs() <= 1e-12,
                    "{name}: {field}[{i}] after step {k}: ours {a}, MuJoCo {b}"
                );
            }
        }
    }
}

/// A `filterexact` actuator's stages advance its activation by `h` times the
/// weighted `act_dot`, as for any other actuator; only the advance takes the
/// exact filter: 200 steps of a hinge with `dynprm="0.03"`.
#[test]
fn rk4_filterexact_stages_take_h_times_act_dot() {
    assert_matches_mujoco("filterexact");
}

/// The stages take an activation past its range unclamped; the advance
/// clamps it: an integrator held at its limit of 0.1 by a control of 1, 200
/// steps.
#[test]
fn rk4_stages_do_not_clamp_the_activation() {
    assert_matches_mujoco("actrange");
}
