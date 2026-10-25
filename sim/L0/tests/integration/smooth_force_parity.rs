//! The smooth force `qfrc_smooth` against MuJoCo 3.5.0's, which sums
//! `((qfrc_passive − qfrc_bias) + qfrc_applied) + qfrc_actuator`
//! (`mj_fwdAcceleration`, `engine_forward.c:583-585`).
//!
//! The golden is `assets/golden/smooth_force/smooth_force.json`, from
//! `scripts/gen_smooth_force_reference.py` on the unfused oracle: a chain of
//! three damped hinges and a damped slide under gravity, each with a velocity
//! actuator, at 8 drawn states with an applied force on every dof, so all
//! four terms are nonzero. The terms are compared first, so a difference
//! there is not read as one in the sum; then `qfrc_smooth` and
//! `qacc_smooth`, bit for bit on macOS arm64, where the golden was made;
//! elsewhere within `1e-13 · max(1, |MuJoCo's|)`, a NaN failing (the states
//! turn hinges through `sin` and `cos`, which another libm may round
//! otherwise).

use serde_json::Value;

fn golden() -> Value {
    serde_json::from_str(include_str!(
        "../assets/golden/smooth_force/smooth_force.json"
    ))
    .expect("smooth force golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn matches_golden(ours: &[f64], mj: &[f64]) -> bool {
    ours.len() == mj.len()
        && ours.iter().zip(mj).all(|(a, b)| {
            a.to_bits() == b.to_bits()
                || (!cfg!(all(target_os = "macos", target_arch = "aarch64"))
                    && (a - b).abs() <= 1e-13 * b.abs().max(1.0))
        })
}

#[test]
fn smooth_force_sums_as_mujoco() {
    let golden = golden();
    let model = sim_mjcf::load_model(golden["xml"].as_str().expect("xml")).expect("load");
    let mut failures = Vec::new();
    let states = golden["states"].as_array().expect("states");
    for (k, state) in states.iter().enumerate() {
        let mut data = model.make_data();
        data.qpos
            .as_mut_slice()
            .copy_from_slice(&floats(&state["qpos"]));
        data.qvel
            .as_mut_slice()
            .copy_from_slice(&floats(&state["qvel"]));
        data.ctrl
            .as_mut_slice()
            .copy_from_slice(&floats(&state["ctrl"]));
        data.qfrc_applied
            .as_mut_slice()
            .copy_from_slice(&floats(&state["qfrc_applied"]));
        data.forward(&model).expect("forward");
        for (what, ours) in [
            ("qfrc_passive", &data.qfrc_passive),
            ("qfrc_bias", &data.qfrc_bias),
            ("qfrc_actuator", &data.qfrc_actuator),
            ("qfrc_smooth", &data.qfrc_smooth),
            ("qacc_smooth", &data.qacc_smooth),
        ] {
            let mj = floats(&state[what]);
            if !matches_golden(ours.as_slice(), &mj) {
                failures.push(format!(
                    "state {k}: {what} {:?}, MuJoCo {mj:?}",
                    ours.as_slice()
                ));
            }
        }
    }
    assert_eq!(states.len(), 8, "8 states");
    assert!(
        failures.is_empty(),
        "{} differ:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
