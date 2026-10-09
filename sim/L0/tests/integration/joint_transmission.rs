//! Joint transmissions on ball and free joints against MuJoCo 3.5.0.
//!
//! The golden is `assets/golden/joint_transmission/joint_transmission.json`,
//! from `scripts/gen_joint_transmission_reference.py` on the unfused oracle:
//! a motor, a position and a velocity actuator with a `joint` or
//! `jointinparent` transmission on a ball or a free joint, scalar and full
//! gears, rotations below and past π, a quaternion off unit norm, a rotation
//! where MuJoCo's form of the length rounds otherwise than an equal one, and
//! (forward only) a free joint whose velocity products sum differently in
//! another order; the model's
//! actuator_acc0, the actuator's length, velocity, moment and force,
//! qfrc_actuator, qacc and its actuatorpos and actuatorvel sensors after
//! `forward`, and qpos and qvel over 10 steps
//! under Euler and implicitfast. The transmission's quantities are compared bit
//! for bit; qacc and the trajectories, which go through the mass matrix, to
//! 1e-12.
//!
//! One known difference: MuJoCo's implicitfast step gathers `qDeriv` onto the
//! mass matrix's sparsity pattern, which for a body MuJoCo calls simple (one
//! box on a ball or free joint here) holds no entry between the joint's dofs,
//! so it drops the cross terms a velocity actuator's moment puts in `qDeriv`;
//! ours keeps them (the spec book's gap chapter,
//! `docs/studies/a_double_dose_of_detail/src/41-what-planning-could-not-see.md`). Those cases must still differ, so the test
//! fails when that changes.

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
/// transmission's bit for bit, qacc to [`close`]. Returns the actuator's
/// moment row as MuJoCo has it.
fn check_forward(case: &Value, failures: &mut Vec<String>) -> Vec<f64> {
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
        ("moment", moment, mj_moment.clone()),
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
        if !same_bits(&ours, &mj) {
            failures.push(format!("{label}: {name} {ours:?}, MuJoCo {mj:?}"));
        }
    }
    let (qacc, mj_qacc) = (data.qacc.as_slice().to_vec(), floats(&f["qacc"]));
    if !close(&qacc, &mj_qacc) {
        failures.push(format!("{label}: qacc {qacc:?}, MuJoCo {mj_qacc:?}"));
    }
    mj_moment
}

#[test]
fn ball_and_free_joint_transmissions_match_mujoco() {
    let golden = golden();
    let mut failures = Vec::new();
    let mut known_still_differ = 0;
    for case in golden["cases"].as_array().expect("cases") {
        let mj_moment = check_forward(case, &mut failures);
        // A velocity actuator whose moment spans more than one dof: its
        // `qDeriv` cross terms, which MuJoCo's implicitfast drops.
        let known =
            case["kind"] == "velocity" && mj_moment.iter().filter(|m| **m != 0.0).count() > 1;
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
            match (integrator == "implicitfast" && known, agree) {
                (false, false) => {
                    failures.push(format!("{}: {integrator} steps differ", label(case)))
                }
                (true, true) => failures.push(format!(
                    "{}: implicitfast now agrees; the known difference is gone",
                    label(case)
                )),
                (true, false) => known_still_differ += 1,
                (false, true) => {}
            }
        }
    }
    for case in golden["forward_only"].as_array().expect("forward_only") {
        check_forward(case, &mut failures);
    }
    assert_eq!(known_still_differ, 9, "the known implicitfast cases");
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

fn same_bits(ours: &[f64], mj: &[f64]) -> bool {
    ours.len() == mj.len() && ours.iter().zip(mj).all(|(a, b)| a.to_bits() == b.to_bits())
}

/// Each entry within `1e-12 · max(1, |MuJoCo's|)`.
fn close(ours: &[f64], mj: &[f64]) -> bool {
    ours.len() == mj.len()
        && ours
            .iter()
            .zip(mj)
            .all(|(a, b)| (a - b).abs() <= 1e-12 * b.abs().max(1.0))
}
