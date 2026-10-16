//! Tendons against MuJoCo 3.5.0: a tendon's spring length of `[-1, -1]` is
//! its length at `qpos_spring` (`setSpring`, `engine_setconst.c:1069-1084`),
//! a tendon over a sphere whose two points lie on a line through its centre
//! wraps in MuJoCo's plane (`mju_wrap`, `engine_util_misc.c:302-340`), and a
//! wrap over a sphere or a cylinder, with or without a sidesite, inside or
//! around, gives MuJoCo's length, Jacobian and wrap points bit for bit.
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

/// Compares each tendon's length, velocity, Jacobian and wrap points after
/// `data`'s `mj_forward` with MuJoCo's `want` within `tol`.
fn assert_tendons(name: &str, model: &Model, data: &Data, want: &Value, tol: f64) {
    let tendons = want.as_array().expect("array");
    assert_eq!(tendons.len(), model.ntendon, "{name}: tendons");
    for (t, want) in tendons.iter().enumerate() {
        let what = |field: &str| format!("{name} tendon {t} {field}");
        assert_close(
            &what("ten_length"),
            &[data.ten_length[t]],
            &[want["ten_length"].as_f64().expect("number")],
            tol,
        );
        assert_close(
            &what("ten_velocity"),
            &[data.ten_velocity[t]],
            &[want["ten_velocity"].as_f64().expect("number")],
            tol,
        );
        assert_close(
            &what("ten_J"),
            data.ten_J[t].as_slice(),
            &floats(&want["ten_J"]),
            tol,
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
            tol,
        );
    }
}

/// Runs `mj_forward` from the case's velocity and compares each tendon's
/// length, velocity, Jacobian and wrap points with MuJoCo's within `tol`
/// ([`assert_tendons`]), then the trajectory where the case has one.
fn assert_wrap(name: &str, case: &Value, tol: f64) {
    let model = load(case);
    let mut data = model.make_data();
    data.qvel
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qvel"]));
    data.forward(&model).expect("forward");
    assert_tendons(name, &model, &data, &case["forward"], tol);
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
/// 0.007071067811865478 (A20 §1A.3.9, U11). Bit for bit, but the tie case
/// within TOL: its hinge's axis "1 -1 0" loads 1 ulp off MuJoCo's (ledger
/// L146), which puts one `ten_J` entry 1 ulp off; with MuJoCo's axis written
/// in, it is bit for bit too (measured).
#[test]
fn collinear_sphere_wrap_takes_mujocos_plane() {
    let g = golden();
    for (name, tol) in [
        ("sphere_wrap", 0.0),
        ("sphere_wrap_tie", TOL),
        ("sphere_wrap_ties", 0.0),
        ("collinear_slides", 0.0),
    ] {
        assert_wrap(name, &g["cases"][name], tol);
    }
}

/// The same example with one point 1e-12 off that line: MuJoCo takes the
/// plane through the three points, as |p0 × p1| = 2e-13 is above mjMINVAL.
/// Before, a cut-off of 1e-10 |p0| |p1| took it for a line. Bit for bit.
#[test]
fn near_collinear_sphere_wrap_keeps_its_plane() {
    assert_wrap(
        "sphere_wrap_near",
        &golden()["cases"]["sphere_wrap_near"],
        0.0,
    );
}

/// A sphere and a cylinder on a moving body, each wrapped by a tendon with no
/// sidesite, one with a sidesite outside it and one with a sidesite inside it
/// (an inside wrap), at 17 configurations where each tendon wraps and where
/// it does not: each tendon's length, velocity, Jacobian and wrap points
/// are MuJoCo's bit for bit (`mju_wrap`, `wrap_circle`, `wrap_inside`,
/// `engine_util_misc.c:36-412`), and so with `jacobian="sparse"`, whose
/// velocity MuJoCo sums over each tendon's chain. Before, a sphere's tangent
/// points came from 3D formulae and a cylinder's from another 2D one: of 200
/// such configurations, the wrap points of 4 to 169 per tendon differed (by
/// up to 2.8e-16), the lengths of 3 to 79 (by up to 3.1e-15).
#[test]
fn wraps_as_mujoco_bit_for_bit() {
    let g = golden();
    for name in ["wraps", "wraps_sparse"] {
        assert_configs(name, &g["cases"][name]);
    }
}

/// Runs `mj_forward` at each of the case's configurations and compares each
/// tendon's state with MuJoCo's bit for bit ([`assert_tendons`]).
fn assert_configs(name: &str, case: &Value) {
    let model = load(case);
    for (k, config) in case["configs"]
        .as_array()
        .expect("array")
        .iter()
        .enumerate()
    {
        let mut data = model.make_data();
        data.qpos
            .as_mut_slice()
            .copy_from_slice(&floats(&config["qpos"]));
        data.qvel
            .as_mut_slice()
            .copy_from_slice(&floats(&config["qvel"]));
        data.forward(&model).expect("forward");
        assert_tendons(
            &format!("{name} configuration {k}"),
            &model,
            &data,
            &config["forward"],
            0.0,
        );
    }
}

/// Tendons between sites a and 2a on six rays from the centre of a sphere
/// and of a cylinder, each with a sidesite inside it: the ends point one way
/// (cos G within mjMINVAL of 1), so each wraps inside at the ends'
/// midpoint's direction scaled to the radius, as MuJoCo's `wrap_inside`
/// falls back (`engine_util_misc.c:195-210`), bit for bit.
#[test]
fn inside_wrap_falls_back_as_mujoco() {
    assert_wrap(
        "inside_fallback",
        &golden()["cases"]["inside_fallback"],
        0.0,
    );
}

/// A fixed tendon over six hinges with coefficients that are not round, at
/// ten velocities: its velocity is MuJoCo's bit for bit, `mju_dot` over
/// every dof at the dense size, `mju_dotSparse` over its chain with
/// `jacobian="sparse"` (`engine_forward.c:232-238`). MuJoCo's two differ in
/// 3 of the 10. Before, both were nalgebra's dot.
#[test]
fn fixed_tendon_velocity_as_mujoco() {
    let g = golden();
    for name in ["fixed_velocity", "fixed_velocity_sparse"] {
        let case = &g["cases"][name];
        let model = load(case);
        for (k, config) in case["configs"]
            .as_array()
            .expect("array")
            .iter()
            .enumerate()
        {
            let mut data = model.make_data();
            data.qvel
                .as_mut_slice()
                .copy_from_slice(&floats(&config["qvel"]));
            data.forward(&model).expect("forward");
            assert_close(
                &format!("{name} configuration {k} ten_J"),
                data.ten_J[0].as_slice(),
                &floats(&config["ten_J"]),
                0.0,
            );
            assert_close(
                &format!("{name} configuration {k} ten_velocity"),
                &[data.ten_velocity[0]],
                &floats(&config["ten_velocity"]),
                0.0,
            );
        }
    }
}

/// A fixed tendon naming joint a twice (coefficients 1, then 2) and joint b
/// once, with stiffness, damping and a motor: MuJoCo's dense Jacobian row
/// assigns each joint's coefficient, so a's is 2, its sparse row adds them,
/// 3 (`engine_core_smooth.c:961-976`), and the forces go through that row;
/// the length adds every entry at both sizes. The motor's acc0 within TOL
/// (its moment is the row); the length, row, velocity, passive and actuator
/// forces after `mj_forward` bit for bit, and the trajectory. Before, the
/// row added them at both sizes, the forces took every coefficient, and the
/// moment for acc0 the last.
#[test]
fn fixed_tendon_naming_a_joint_twice_as_mujoco() {
    let g = golden();
    for name in ["fixed_repeated", "fixed_repeated_sparse"] {
        let case = &g["cases"][name];
        let model = load(case);
        assert_close(
            &format!("{name} actuator_acc0"),
            &model.actuator_acc0,
            &floats(&case["actuator_acc0"]),
            TOL,
        );
        let mut data = model.make_data();
        data.qpos
            .as_mut_slice()
            .copy_from_slice(&floats(&case["qpos"]));
        data.qvel
            .as_mut_slice()
            .copy_from_slice(&floats(&case["qvel"]));
        data.ctrl
            .as_mut_slice()
            .copy_from_slice(&floats(&case["ctrl"]));
        data.forward(&model).expect("forward");
        let want = &case["forward"];
        let what = |field: &str| format!("{name} {field}");
        assert_close(
            &what("ten_length"),
            &[data.ten_length[0]],
            &[want["ten_length"].as_f64().expect("number")],
            0.0,
        );
        assert_close(
            &what("ten_J"),
            data.ten_J[0].as_slice(),
            &floats(&want["ten_J"]),
            0.0,
        );
        assert_close(
            &what("ten_velocity"),
            &[data.ten_velocity[0]],
            &[want["ten_velocity"].as_f64().expect("number")],
            0.0,
        );
        assert_close(
            &what("qfrc_passive"),
            data.qfrc_passive.as_slice(),
            &floats(&want["qfrc_passive"]),
            0.0,
        );
        assert_close(
            &what("qfrc_actuator"),
            data.qfrc_actuator.as_slice(),
            &floats(&want["qfrc_actuator"]),
            0.0,
        );
        assert_trajectory(name, &model, &mut data, &case["traj"]);
    }
}
