//! Transition derivatives on quaternion joints against MuJoCo 3.5.0: the
//! finite differences difference the perturbed next states with each other
//! (`mjd_stepFD`, `engine_derivative_fd.c:295-531`), so a ball's or a free
//! joint's position rows are tangents at the next state
//! (`mj_differentiatePos`, `engine_support.c:608-638`), and the analytic
//! integration Jacobians are MuJoCo's `mjd_quatIntegrate`'s
//! (`engine_derivative.c:262-314`).
//!
//! The golden is `assets/golden/transition/transition.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_transition_reference.py`, which describes
//! its cases).

use nalgebra::{DVector, UnitQuaternion, Vector3};
use serde_json::Value;
use sim_core::{
    DerivativeConfig, mj_differentiate_pos, mjd_quat_integrate, mjd_transition_fd,
    mjd_transition_hybrid,
};
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/transition/transition.json"))
        .expect("transition golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn assert_close(what: &str, got: &[f64], want: &[f64], tol: f64) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    for (i, (a, b)) in got.iter().zip(want).enumerate() {
        assert!((a - b).abs() <= tol, "{what}[{i}]: {a}, expected {b}");
    }
}

fn assert_bits(what: &str, got: &[f64], want: &[f64]) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    for (i, (a, b)) in got.iter().zip(want).enumerate() {
        assert_eq!(a.to_bits(), b.to_bits(), "{what}[{i}]: {a}, MuJoCo {b}");
    }
}

/// A ball pendulum, a hinge with a ball pendulum below it and a free body,
/// spinning: the transition's A from sim-core's finite differences and from
/// its hybrid derivatives against MuJoCo's `mjd_transitionFD` (centred, eps
/// 1e-6), every row. Before, the position rows of a quaternion joint were
/// tangents at the step's start: 5.9e-4, 5.1e-4 and 9.0e-4 off, the finite
/// differences and the hybrid alike.
#[test]
fn ball_transition_matches_mujoco_fd() {
    let g = golden();
    let config = DerivativeConfig::default();
    for name in ["ball", "hinge_ball", "free"] {
        let case = &g["cases"]["transitions"][name];
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qpos.copy_from_slice(&floats(&case["qpos"]));
        data.qvel.copy_from_slice(&floats(&case["qvel"]));
        data.forward(&model).expect("forward");
        let want = floats(&case["A"]);
        for (method, a) in [
            (
                "FD",
                mjd_transition_fd(&model, &data, &config).expect("fd").A,
            ),
            (
                "hybrid",
                mjd_transition_hybrid(&model, &data, &config)
                    .expect("hybrid")
                    .A,
            ),
        ] {
            let nx = a.nrows();
            let rows: Vec<f64> = (0..nx)
                .flat_map(|r| (0..nx).map(move |c| (r, c)))
                .map(|(r, c)| a[(r, c)])
                .collect();
            assert_close(&format!("{name} {method} A"), &rows, &want, 1e-8);
        }
    }
}

/// `mj_differentiate_pos` gives MuJoCo's `mj_differentiatePos` bit for bit
/// on a hinge, a slide, a ball and a free joint: a 1e-12 rad rotation, a
/// rotation past π (taken the short way) and a negated quaternion, and
/// quaternions not of unit length, at dt 0.002, 2e-6 and 7e-4. Before, a rotation
/// below 2e-10 rad read 0, the angle was not wrapped, and the quaternions
/// were normalized.
#[test]
fn differentiate_pos_matches_mujoco() {
    let g = golden();
    let case = &g["cases"]["differentiate"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    for pair in case["cases"].as_array().expect("cases") {
        let mut qvel = DVector::zeros(model.nv);
        mj_differentiate_pos(
            &model,
            &mut qvel,
            &DVector::from_vec(floats(&pair["qpos1"])),
            &DVector::from_vec(floats(&pair["qpos2"])),
            pair["dt"].as_f64().expect("dt"),
        );
        assert_bits(
            &format!("{} at dt {}", pair["name"], pair["dt"]),
            qvel.as_slice(),
            &floats(&pair["qvel"]),
        );
    }
}

/// `mjd_quat_integrate` gives MuJoCo's `mjd_quatIntegrate` bit for bit, its
/// `Dquat` = exp(−[ω·h]×) and `h` times its `Dvel` (MuJoCo's is with respect
/// to the scaled velocity `ω·h`), above and below its Taylor threshold and at
/// 0. Before, the position block was J_l⁻¹(ω·h) and the velocity block `h·I`,
/// for tangents at the step's start.
#[test]
fn quat_integrate_jacobians_match_mujoco() {
    for case in golden()["cases"]["quat_integrate"]
        .as_array()
        .expect("cases")
    {
        let vel = floats(&case["vel"]);
        let h = case["scale"].as_f64().expect("scale");
        let (dquat, domega) = mjd_quat_integrate(
            &UnitQuaternion::identity(),
            &Vector3::new(vel[0], vel[1], vel[2]),
            h,
        );
        let rows = |m: &nalgebra::Matrix3<f64>| -> Vec<f64> {
            (0..3)
                .flat_map(|r| (0..3).map(move |c| (r, c)))
                .map(|(r, c)| m[(r, c)])
                .collect()
        };
        let what = format!("vel {vel:?}, scale {h}");
        assert_bits(
            &format!("{what} Dquat"),
            &rows(&dquat),
            &floats(&case["Dquat"]),
        );
        let want: Vec<f64> = floats(&case["Dvel"]).iter().map(|v| h * v).collect();
        assert_bits(&format!("{what} h·Dvel"), &rows(&domega), &want);
    }
}
