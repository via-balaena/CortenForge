//! The velocity derivative `qDeriv` against MuJoCo 3.5.0's, which adds the
//! actuators' term, then the passive forces', then the bias force's
//! (`mjd_smooth_vel`, `engine_derivative.c:1792-1812`).
//!
//! The golden is `assets/golden/velocity_derivative/velocity_derivative.json`,
//! from `scripts/gen_velocity_derivative_reference.py` on the unfused oracle:
//! a chain of three damped hinges and a damped slide under a damped fixed
//! tendon and a damped spatial tendon, with velocity actuators on a hinge and
//! on both tendons, so each of the chain's entries of `qDeriv` sums an
//! actuator's term and more than one passive term; 8 drawn states under
//! implicit and under implicitfast, one step each; and an undamped slide whose
//! actuator's kv is 1e-31, whose term MuJoCo adds however small. `qDeriv` is compared on D's
//! pattern, bit for bit on the golden's platform (see `matches_golden`).

use serde_json::Value;

fn golden() -> Value {
    serde_json::from_str(include_str!(
        "../assets/golden/velocity_derivative/velocity_derivative.json"
    ))
    .expect("velocity derivative golden parses")
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

#[test]
fn velocity_derivative_sums_as_mujoco() {
    let golden = golden();
    let mut failures = Vec::new();
    let mut compared = 0;
    for case in golden["cases"].as_array().expect("cases") {
        let integrator = case["integrator"].as_str().expect("integrator");
        let model = sim_mjcf::load_model(case["xml"].as_str().expect("xml")).expect("load");
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
            data.step(&model).expect("step");
            let ours: Vec<f64> = (0..model.nv)
                .flat_map(|r| (rowadr[r]..rowadr[r] + rownnz[r]).map(move |j| (r, j)))
                .map(|(r, j)| data.qDeriv[(r, colind[j])])
                .collect();
            let mj = floats(&state["qDeriv"]);
            if !matches_golden(&ours, &mj) {
                failures.push(format!(
                    "{integrator} state {k}: qDeriv {ours:?}, MuJoCo {mj:?}"
                ));
            }
            compared += 1;
        }
    }
    assert_eq!(
        compared, 17,
        "2 integrators of 8 states, and the tiny actuator"
    );
    assert!(
        failures.is_empty(),
        "{} differ:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
