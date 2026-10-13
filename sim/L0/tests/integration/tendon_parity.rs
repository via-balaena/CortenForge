//! Tendons against MuJoCo 3.5.0: a tendon's spring length of `[-1, -1]` is
//! its length at `qpos_spring` (`setSpring`, `engine_setconst.c:1069-1084`),
//! and a tendon over a sphere whose two points lie on a line through its
//! centre wraps in MuJoCo's plane (`mju_wrap`, `engine_util_misc.c:302-340`).
//!
//! The golden is `assets/golden/tendon/tendon.json`, from the unfused MuJoCo
//! 3.5.0 oracle (`scripts/gen_tendon_reference.py`, which describes its
//! cases).

use serde_json::Value;
use sim_core::{Data, Model};
use sim_mjcf::load_model;

const TOL: f64 = 1e-12;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/tendon/tendon.json"))
        .expect("tendon golden parses")
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

fn load(case: &Value) -> Model {
    load_model(case["xml"].as_str().expect("xml")).expect("load")
}

fn lengthspring(model: &Model) -> Vec<f64> {
    model
        .tendon_lengthspring
        .iter()
        .flatten()
        .copied()
        .collect()
}

/// Steps `data` to each of the case's checkpoints and compares `qpos` and
/// `qvel` with MuJoCo's.
fn assert_trajectory(name: &str, model: &Model, data: &mut Data, traj: &Value) {
    let mut step = 0;
    for checkpoint in [1, 10, 100] {
        while step < checkpoint {
            data.step(model).expect("step");
            step += 1;
        }
        let want = &traj[checkpoint.to_string()];
        assert_close(
            &format!("{name} qpos at step {step}"),
            data.qpos.as_slice(),
            &floats(&want["qpos"]),
            TOL,
        );
        assert_close(
            &format!("{name} qvel at step {step}"),
            data.qvel.as_slice(),
            &floats(&want["qvel"]),
            TOL,
        );
    }
}

/// A spatial tendon with no `springlength` springs toward its length at
/// `qpos_spring`, where its hinges' and slide's `springref` differ from their
/// `ref` (A19 §2.6 #5); a given `springlength` stays, and a fixed tendon's
/// comes from `qpos_spring` as before. Before, the spatial tendon's came from
/// `qpos0`. The two in-tree examples whose hinges have a `ref`: the
/// spatial-path one, and the tendon-limits one (A20 §1A.3.7: 0.7071067811865476).
#[test]
fn spatial_lengthspring_at_springref() {
    let g = golden();
    let case = &g["cases"]["spring_length"];
    let model = load(case);
    assert_close(
        "qpos_spring",
        &model.qpos_spring,
        &floats(&case["qpos_spring"]),
        0.0,
    );
    assert_close(
        "tendon_length0",
        &model.tendon_length0,
        &floats(&case["tendon_length0"]),
        TOL,
    );
    assert_close(
        "tendon_lengthspring",
        &lengthspring(&model),
        &floats(&case["tendon_lengthspring"]),
        TOL,
    );
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert_close(
        "qfrc_spring at qpos0",
        data.qfrc_spring.as_slice(),
        &floats(&case["qfrc_spring"]),
        TOL,
    );
    data.qvel
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qvel"]));
    assert_trajectory("spring_length", &model, &mut data, &case["traj"]);

    for name in ["spatial_path", "tendon_limits"] {
        let case = &g["cases"][name];
        assert_close(
            &format!("{name} tendon_lengthspring"),
            &lengthspring(&load(case)),
            &floats(&case["tendon_lengthspring"]),
            TOL,
        );
    }
}

/// Runs `mj_forward` from the case's velocity and compares each tendon's
/// length, velocity, Jacobian and wrap points with MuJoCo's, then the
/// trajectory where the case has one.
fn assert_wrap(name: &str, case: &Value) {
    let model = load(case);
    let mut data = model.make_data();
    data.qvel
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qvel"]));
    data.forward(&model).expect("forward");
    let tendons = case["forward"].as_array().expect("array");
    assert_eq!(tendons.len(), model.ntendon, "{name}: tendons");
    for (t, want) in tendons.iter().enumerate() {
        let what = |field: &str| format!("{name} tendon {t} {field}");
        assert_close(
            &what("ten_length"),
            &[data.ten_length[t]],
            &[want["ten_length"].as_f64().expect("number")],
            TOL,
        );
        assert_close(
            &what("ten_velocity"),
            &[data.ten_velocity[t]],
            &[want["ten_velocity"].as_f64().expect("number")],
            TOL,
        );
        assert_close(
            &what("ten_J"),
            data.ten_J[t].as_slice(),
            &floats(&want["ten_J"]),
            TOL,
        );
        let (adr, num) = (data.ten_wrapadr[t], data.ten_wrapnum[t]);
        let obj: Vec<i64> = data.wrap_obj[adr..adr + num]
            .iter()
            .map(|&o| i64::from(o))
            .collect();
        let want_obj: Vec<i64> = want["wrap_obj"]
            .as_array()
            .expect("array")
            .iter()
            .map(|x| x.as_i64().expect("integer"))
            .collect();
        assert_eq!(obj, want_obj, "{}", what("wrap_obj"));
        let points: Vec<f64> = data.wrap_xpos[adr..adr + num]
            .iter()
            .flat_map(|p| [p.x, p.y, p.z])
            .collect();
        assert_close(
            &what("wrap_xpos"),
            &points,
            &floats(&want["wrap_xpos"]),
            TOL,
        );
    }
    if !case["traj"].is_null() {
        assert_trajectory(name, &model, &mut data, &case["traj"]);
    }
}

/// A tendon over a sphere whose two points lie on a line through its centre
/// wraps in MuJoCo's plane: its normal is the first point's direction crossed
/// with a vector of ones holding 0 at y if that direction's y exceeds both
/// its other components in magnitude, at z if its z does, else at x, and is
/// the x axis where that cross product is 0 (a line along (0, 1, 1)). Before,
/// the plane held the axis least aligned with that direction: the sphere-wrap
/// example's tendon velocity read 0 at the start, against MuJoCo's
/// 0.007071067811865478 (A20 §1A.3.9, U11).
#[test]
fn collinear_sphere_wrap_takes_mujocos_plane() {
    let g = golden();
    for name in [
        "sphere_wrap",
        "sphere_wrap_tie",
        "sphere_wrap_ties",
        "collinear_slides",
    ] {
        assert_wrap(name, &g["cases"][name]);
    }
}

/// The same example with one point 1e-12 off that line: MuJoCo takes the
/// plane through the three points, as |p0 × p1| = 2e-13 is above mjMINVAL.
/// Before, a cut-off of 1e-10 |p0| |p1| took it for a line.
#[test]
fn near_collinear_sphere_wrap_keeps_its_plane() {
    assert_wrap("sphere_wrap_near", &golden()["cases"]["sphere_wrap_near"]);
}
