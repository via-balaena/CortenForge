//! Joint transmissions on ball and free joints against MuJoCo 3.5.0.
//!
//! The golden is `assets/golden/joint_transmission/joint_transmission.json`,
//! from `scripts/gen_joint_transmission_reference.py` on the unfused oracle:
//! a motor, a position and a velocity actuator with a `joint` or
//! `jointinparent` transmission on a ball or a free joint, scalar and full
//! gears, rotations below and past π, a quaternion off unit norm, a rotation
//! where MuJoCo's form of the length rounds otherwise than an equal one, and
//! (forward only) a free joint whose velocity products sum differently in
//! another order; the model's actuator_acc0, the actuator's length,
//! velocity, moment and force, qfrc_actuator, qacc and its actuatorpos and
//! actuatorvel sensors after `forward`, and qpos and qvel over 10 steps under
//! Euler and implicitfast. The transmission's quantities are compared bit for
//! bit (on the golden's platform; see `matches_golden`); qacc and the
//! trajectories, which go through the mass matrix, to 1e-12. Under
//! implicitfast a velocity actuator's moment across a simple body's dofs (one
//! box on a ball or free joint here) puts cross terms in `qDeriv` that the
//! step leaves out, as MuJoCo's does: the mass matrix's pattern holds no
//! entry between those dofs.

use serde_json::Value;

fn golden() -> Value {
    serde_json::from_str(include_str!(
        "../assets/golden/joint_transmission/joint_transmission.json"
    ))
    .expect("joint transmission golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn label(case: &Value) -> String {
    format!(
        "{} {}",
        case["name"].as_str().expect("name"),
        case["kind"].as_str().expect("kind")
    )
}

fn start(case: &Value, data: &mut sim_core::Data) {
    data.qpos
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qpos"]));
    data.qvel
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qvel"]));
    data.ctrl
        .as_mut_slice()
        .copy_from_slice(&floats(&case["ctrl"]));
}

/// The case's quantities after `forward` against MuJoCo's: the
/// transmission's to [`matches_golden`], qacc to [`close`].
fn check_forward(case: &Value, failures: &mut Vec<String>) {
    let label = label(case);
    let model = sim_mjcf::load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    start(case, &mut data);
    data.forward(&model).expect("forward");
    let f = &case["forward"];
    let moment: Vec<f64> = data.actuator_moment[0].iter().copied().collect();
    let mj_moment = floats(&f["actuator_moment"]);
    for (name, ours, mj) in [
        (
            "acc0",
            model.actuator_acc0.clone(),
            floats(&f["actuator_acc0"]),
        ),
        (
            "length",
            data.actuator_length.as_slice().to_vec(),
            floats(&f["actuator_length"]),
        ),
        (
            "velocity",
            data.actuator_velocity.as_slice().to_vec(),
            floats(&f["actuator_velocity"]),
        ),
        ("moment", moment, mj_moment),
        (
            "force",
            data.actuator_force.as_slice().to_vec(),
            floats(&f["actuator_force"]),
        ),
        (
            "qfrc_actuator",
            data.qfrc_actuator.as_slice().to_vec(),
            floats(&f["qfrc_actuator"]),
        ),
        (
            "actuatorpos and actuatorvel sensors",
            data.sensordata.as_slice().to_vec(),
            floats(&f["sensordata"]),
        ),
    ] {
        if !matches_golden(&ours, &mj) {
            failures.push(format!("{label}: {name} {ours:?}, MuJoCo {mj:?}"));
        }
    }
    let (qacc, mj_qacc) = (data.qacc.as_slice().to_vec(), floats(&f["qacc"]));
    if !close(&qacc, &mj_qacc) {
        failures.push(format!("{label}: qacc {qacc:?}, MuJoCo {mj_qacc:?}"));
    }
}

#[test]
fn ball_and_free_joint_transmissions_match_mujoco() {
    let golden = golden();
    let mut failures = Vec::new();
    for case in golden["cases"].as_array().expect("cases") {
        check_forward(case, &mut failures);
        let xml = case["xml"].as_str().expect("xml");
        for integrator in ["Euler", "implicitfast"] {
            let model = sim_mjcf::load_model(&xml.replace(
                r#"integrator="Euler""#,
                &format!(r#"integrator="{integrator}""#),
            ))
            .expect("load");
            let mut data = model.make_data();
            start(case, &mut data);
            let mut agree = true;
            for state in case[integrator].as_array().expect("steps") {
                data.step(&model).expect("step");
                agree &= close(data.qpos.as_slice(), &floats(&state["qpos"]))
                    && close(data.qvel.as_slice(), &floats(&state["qvel"]));
            }
            if !agree {
                failures.push(format!("{}: {integrator} steps differ", label(case)));
            }
        }
    }
    for case in golden["forward_only"].as_array().expect("forward_only") {
        check_forward(case, &mut failures);
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Bit for bit on macOS arm64, where the golden was made. The values go
/// through `sin`, `cos` and `atan2`, which another platform's libm may round
/// otherwise by an ulp; there the comparison is to `1e-13 · max(1, |MuJoCo's|)`.
fn matches_golden(ours: &[f64], mj: &[f64]) -> bool {
    ours.len() == mj.len()
        && ours.iter().zip(mj).all(|(a, b)| {
            a.to_bits() == b.to_bits()
                || (!cfg!(all(target_os = "macos", target_arch = "aarch64"))
                    && (a - b).abs() <= 1e-13 * b.abs().max(1.0))
        })
}

/// Each entry within `1e-12 · max(1, |MuJoCo's|)`.
fn close(ours: &[f64], mj: &[f64]) -> bool {
    ours.len() == mj.len()
        && ours
            .iter()
            .zip(mj)
            .all(|(a, b)| (a - b).abs() <= 1e-12 * b.abs().max(1.0))
}
