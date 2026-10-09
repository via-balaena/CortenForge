//! Quaternion integration against MuJoCo 3.5.0, bit for bit (on the golden's
//! platform; see `matches_golden`).
//!
//! The golden is `assets/golden/quat/quat.json`, from
//! `scripts/gen_quat_reference.py` on the unfused oracle: a box on a free and
//! on a ball joint spinning in zero gravity under Euler and RK4, slowly (an
//! angle per step far below 1e-10) and fast; and MuJoCo's `mj_integratePos`
//! on a ball joint for quaternions of unit and other norms, velocities below
//! `mjMINVAL`, and negative steps.

use serde_json::Value;
use sim_core::mj_integrate_pos_explicit;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/quat/quat.json"))
        .expect("quat golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
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

/// Each spin's qpos at every checkpoint is MuJoCo's, bit for bit (on the
/// golden's platform). The spin
/// is about a principal axis, so the gyroscopic force is 0 but for rounding,
/// which leaves the other two velocities below 1e-16 in MuJoCo's golden and
/// at other values that small here; the velocities are checked to 1e-15.
#[test]
fn spins_integrate_as_mujoco() {
    let golden = golden();
    let mut failures = Vec::new();
    for case in golden["spins"].as_array().expect("spins") {
        let label = format!(
            "{} {} at {} rad/s",
            case["root"].as_str().expect("root"),
            case["integrator"].as_str().expect("integrator"),
            case["speed"]
        );
        let model = sim_mjcf::load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qpos
            .as_mut_slice()
            .copy_from_slice(&floats(&case["start"]["qpos"]));
        data.qvel
            .as_mut_slice()
            .copy_from_slice(&floats(&case["start"]["qvel"]));
        let mut step = 0;
        for state in case["states"].as_array().expect("states") {
            let until = state["step"].as_u64().expect("step");
            while step < until {
                data.step(&model).expect("step");
                step += 1;
            }
            let (qpos, qvel) = (floats(&state["qpos"]), floats(&state["qvel"]));
            let qvel_off = data
                .qvel
                .iter()
                .zip(&qvel)
                .any(|(a, b)| (a - b).abs() > 1e-15);
            if !matches_golden(data.qpos.as_slice(), &qpos) || qvel_off {
                failures.push(format!(
                    "{label}, step {step}: qpos {:?} qvel {:?}, MuJoCo {qpos:?} {qvel:?}",
                    data.qpos.as_slice(),
                    data.qvel.as_slice()
                ));
                break;
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// `mj_integrate_pos_explicit` on a ball joint gives MuJoCo's
/// `mj_integratePos`, bit for bit (on the golden's platform).
#[test]
fn integrate_pos_explicit_matches_mujoco() {
    let model = sim_mjcf::load_model(
        r#"<mujoco>
          <worldbody>
            <body name="b" pos="0 0 1">
              <joint type="ball"/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body>
          </worldbody>
        </mujoco>"#,
    )
    .expect("load");
    let golden = golden();
    let mut failures = Vec::new();
    for case in golden["integrate_pos"].as_array().expect("integrate_pos") {
        let qpos = nalgebra::DVector::from_vec(floats(&case["qpos"]));
        let qvel = nalgebra::DVector::from_vec(floats(&case["qvel"]));
        let dt = case["dt"].as_f64().expect("dt");
        let mut out = nalgebra::DVector::zeros(4);
        mj_integrate_pos_explicit(&model, &mut out, &qpos, &qvel, dt);
        let want = floats(&case["result"]);
        if !matches_golden(out.as_slice(), &want) {
            failures.push(format!(
                "{}: {:?}, MuJoCo {want:?}",
                case["name"].as_str().expect("name"),
                out.as_slice()
            ));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
