//! An actuator whose gain, bias or dynamics are a user's against MuJoCo
//! 3.5.0's, with its callback set and unset, and with two activations.
//!
//! The golden is `assets/golden/user_actuator/user_actuator.json`, from
//! `scripts/gen_user_actuator_reference.py` on the unfused oracle: two damped
//! hinges under implicitfast (one case under implicit) and one actuator on
//! the second, a user gain's or bias's own parameters written (MuJoCo's reads
//! none of them), 12 drawn states each, `act_dot` set before each pass
//! (MuJoCo zeroes an actuator's before its dynamics write it). sim-mjcf
//! refuses a user type and has no `actdim`, so each case loads a stand-in and
//! changes it to the user's (`patch`), checked against MuJoCo's compiled
//! types; the callbacks below are the script's, in the same operations, the
//! dynamics' reading its own `act_dot` (0 in MuJoCo's when it is called) and,
//! with two activations, giving the last one's, which MuJoCo's writes. After
//! a forward pass the actuator's force, `qfrc_actuator` and `act_dot` are
//! compared; after a step `qDeriv` on D's pattern, which `mjd_actuator_vel`
//! adds to (`engine_derivative.c:1071-1146`), and qpos, qvel and act; bit for
//! bit on the golden's platform (see `matches_golden`).

use serde_json::Value;
use sim_core::{ActuatorDynamics, BiasType, GainType, Model};

fn golden() -> Value {
    serde_json::from_str(include_str!(
        "../assets/golden/user_actuator/user_actuator.json"
    ))
    .expect("user actuator golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn usizes(v: &Value) -> Vec<usize> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| usize::try_from(x.as_u64().expect("index")).expect("fits"))
        .collect()
}

/// Bit for bit on macOS arm64, where the golden was made. The states turn
/// hinges through `sin` and `cos`, which another platform's libm may round
/// otherwise by an ulp; there each entry is within `1e-13 · max(1,
/// |MuJoCo's|)`, a NaN failing.
fn matches_golden(ours: &[f64], mj: &[f64]) -> bool {
    ours.len() == mj.len()
        && ours.iter().zip(mj).all(|(a, b)| {
            a.to_bits() == b.to_bits()
                || (!cfg!(all(target_os = "macos", target_arch = "aarch64"))
                    && (a - b).abs() <= 1e-13 * b.abs().max(1.0))
        })
}

/// The stand-in loaded and changed to the case's user types and callbacks.
fn patched(case: &Value) -> Model {
    let mut model = sim_mjcf::load_model(case["xml"].as_str().expect("xml")).expect("load");
    let patch = &case["patch"];
    match patch["gain"].as_str() {
        Some("callback") => {
            model.actuator_gaintype[0] = GainType::User;
            model.set_act_gain_callback(|_, d, i| {
                1.7 + 0.2 * d.actuator_length[i] - 0.3 * d.actuator_velocity[i]
            });
        }
        Some("unset") => model.actuator_gaintype[0] = GainType::User,
        _ => {}
    }
    match patch["bias"].as_str() {
        Some("callback") => {
            model.actuator_biastype[0] = BiasType::User;
            model.set_act_bias_callback(|_, d, i| {
                -0.4 * d.actuator_length[i] + 0.25 * d.actuator_velocity[i]
            });
        }
        Some("unset") => model.actuator_biastype[0] = BiasType::User,
        _ => {}
    }
    match patch["dyn"].as_str() {
        Some("callback") => {
            model.actuator_dyntype[0] = ActuatorDynamics::User;
            model.set_act_dyn_callback(|m, d, i| {
                let a = m.actuator_act_adr[i] + m.actuator_act_num[i] - 1;
                (d.ctrl[i] - d.act[a]) / 0.07 + d.act_dot[a]
            });
        }
        Some("unset") => model.actuator_dyntype[0] = ActuatorDynamics::User,
        _ => {}
    }
    if let Some(actnum) = patch["actnum"].as_u64() {
        model.actuator_act_num[0] = usize::try_from(actnum).expect("fits");
        model.na = model.actuator_act_num.iter().sum();
    }
    let gaintype = match model.actuator_gaintype[0] {
        GainType::Fixed => "mjGAIN_FIXED",
        GainType::Affine => "mjGAIN_AFFINE",
        GainType::User => "mjGAIN_USER",
        other => panic!("gain {other:?}"),
    };
    let biastype = match model.actuator_biastype[0] {
        BiasType::None => "mjBIAS_NONE",
        BiasType::Affine => "mjBIAS_AFFINE",
        BiasType::User => "mjBIAS_USER",
        other => panic!("bias {other:?}"),
    };
    let dyntype = match model.actuator_dyntype[0] {
        ActuatorDynamics::None => "mjDYN_NONE",
        ActuatorDynamics::User => "mjDYN_USER",
        other => panic!("dynamics {other:?}"),
    };
    assert_eq!(
        (gaintype, biastype, dyntype, model.actuator_act_num[0]),
        (
            case["gaintype"].as_str().expect("gaintype"),
            case["biastype"].as_str().expect("biastype"),
            case["dyntype"].as_str().expect("dyntype"),
            usize::try_from(case["actnum"].as_u64().expect("actnum")).expect("fits"),
        ),
        "{}: the patched model is not MuJoCo's",
        case["name"]
    );
    model
}

#[test]
fn user_actuators_act_as_mujoco() {
    let golden = golden();
    let mut failures = Vec::new();
    let mut compared = 0;
    for case in golden["cases"].as_array().expect("cases") {
        let name = case["name"].as_str().expect("name");
        let model = patched(case);
        let (rownnz, rowadr, colind) = (
            usizes(&case["D_rownnz"]),
            usizes(&case["D_rowadr"]),
            usizes(&case["D_colind"]),
        );
        for (k, state) in case["states"]
            .as_array()
            .expect("states")
            .iter()
            .enumerate()
        {
            let start = || {
                let mut data = model.make_data();
                for (field, key) in [
                    (data.qpos.as_mut_slice(), "qpos"),
                    (data.qvel.as_mut_slice(), "qvel"),
                    (data.ctrl.as_mut_slice(), "ctrl"),
                    (data.act.as_mut_slice(), "act"),
                    (data.act_dot.as_mut_slice(), "act_dot_before"),
                ] {
                    field.copy_from_slice(&floats(&state[key]));
                }
                data
            };
            let mut data = start();
            data.forward(&model).expect("forward");
            let mut compare = |what: &str, ours: &[f64], key: &str| {
                let mj = floats(&state[key]);
                if !matches_golden(ours, &mj) {
                    failures.push(format!("{name}, state {k}: {what} {ours:?}, MuJoCo {mj:?}"));
                }
            };
            compare(
                "actuator_force",
                data.actuator_force.as_slice(),
                "actuator_force",
            );
            compare(
                "qfrc_actuator",
                data.qfrc_actuator.as_slice(),
                "qfrc_actuator",
            );
            compare("act_dot", data.act_dot.as_slice(), "act_dot");
            let mut data = start();
            data.step(&model).expect("step");
            let qderiv: Vec<f64> = (0..model.nv)
                .flat_map(|r| (rowadr[r]..rowadr[r] + rownnz[r]).map(move |j| (r, j)))
                .map(|(r, j)| data.qDeriv[(r, colind[j])])
                .collect();
            compare("qDeriv after the step", &qderiv, "qDeriv");
            compare("qpos after the step", data.qpos.as_slice(), "qpos_after");
            compare("qvel after the step", data.qvel.as_slice(), "qvel_after");
            compare("act after the step", data.act.as_slice(), "act_after");
            compared += 1;
        }
    }
    assert_eq!(compared, 108, "9 cases of 12 states");
    assert!(
        failures.is_empty(),
        "{} differ:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
