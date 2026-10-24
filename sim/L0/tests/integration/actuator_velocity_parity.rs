//! Each actuator's velocity against MuJoCo 3.5.0's, which takes it as the
//! sparse dot of the actuator's moment row with `qvel`
//! (`mju_mulMatVecSparse` in `mj_fwdVelocity`, `engine_forward.c:240-246`)
//! over the entries `mj_transmission` stores (`engine_core_smooth.c:1250-1730`),
//! and zeroes it when actuation is disabled.
//!
//! The golden is `assets/golden/actuator_velocity/actuator_velocity.json`,
//! from `scripts/gen_actuator_velocity_reference.py` on the unfused oracle:
//! velocity actuators through a fixed tendon, a spatial tendon, two sites
//! (one with a reference site), a slider-crank and a hinge with a negative
//! gear, under the dense and the sparse Jacobian and with actuation
//! disabled, each at 12 drawn states, the last holding the hinge still. The
//! moment rows sim-core fills (`Data::actuator_moment`: not a hinge's or a
//! tendon's) are compared first, so a difference there is not read as one in
//! the velocity; then the velocities, the forces, `qfrc_actuator` and the
//! `actuatorvel` sensors, bit for bit on the golden's platform (see
//! `matches_golden`).

use serde_json::Value;

/// The actuators whose moment rows sim-core fills: the two site actuators,
/// the reference-site one and the slider-crank (`Data::actuator_moment`
/// leaves a tendon's and a hinge's row empty).
const MOMENT_ROWS: [usize; 4] = [2, 3, 4, 5];

fn golden() -> Value {
    serde_json::from_str(include_str!(
        "../assets/golden/actuator_velocity/actuator_velocity.json"
    ))
    .expect("actuator velocity golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
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

#[test]
fn actuator_velocity_as_mujoco() {
    let golden = golden();
    let mut failures = Vec::new();
    let mut compared = 0;
    for case in golden["cases"].as_array().expect("cases") {
        let name = case["name"].as_str().expect("name");
        let model = sim_mjcf::load_model(case["xml"].as_str().expect("xml")).expect("load");
        assert_eq!(
            model.is_sparse(),
            case["sparse"].as_bool().expect("sparse"),
            "{name}: the Jacobian's kind"
        );
        let nv = model.nv;
        for (k, state) in case["states"]
            .as_array()
            .expect("states")
            .iter()
            .enumerate()
        {
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
            data.forward(&model).expect("forward");
            let mj_moment = floats(&state["actuator_moment"]);
            for i in MOMENT_ROWS {
                let ours: Vec<f64> = data.actuator_moment[i].iter().copied().collect();
                let mj = &mj_moment[i * nv..(i + 1) * nv];
                if !matches_golden(&ours, mj) {
                    failures.push(format!(
                        "{name} state {k}: actuator {i} moment {ours:?}, MuJoCo {mj:?}"
                    ));
                }
            }
            for (what, ours, mj) in [
                (
                    "actuator_velocity",
                    data.actuator_velocity.clone(),
                    floats(&state["actuator_velocity"]),
                ),
                (
                    "actuator_force",
                    data.actuator_force.clone(),
                    floats(&state["actuator_force"]),
                ),
                (
                    "qfrc_actuator",
                    data.qfrc_actuator.as_slice().to_vec(),
                    floats(&state["qfrc_actuator"]),
                ),
                (
                    "actuatorvel sensors",
                    data.sensordata.as_slice().to_vec(),
                    floats(&state["sensordata"]),
                ),
            ] {
                if !matches_golden(&ours, &mj) {
                    failures.push(format!("{name} state {k}: {what} {ours:?}, MuJoCo {mj:?}"));
                }
            }
            compared += 1;
        }
    }
    assert_eq!(compared, 36, "3 cases of 12 states");
    assert!(
        failures.is_empty(),
        "{} differ:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
