//! Forward kinematics against MuJoCo 3.5.0's `mj_kinematics`
//! (`engine_core_smooth.c:40-232`): a hinge or a slide moves by
//! `qpos − qpos0`, a hinge or a ball turns about its anchor, each joint's
//! `xanchor` and `xaxis` are taken in the frame before it turns, and the
//! arithmetic is MuJoCo's. With them, the sizes behind `dof_length`, and the
//! ball and free joint springs and the ball-quaternion sensor, which
//! normalize a quaternion as MuJoCo does.
//!
//! The golden is `assets/golden/kinematics/kinematics.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_kinematics_reference.py`, which
//! describes its cases).

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use serde_json::Value;
use sim_core::{Data, DerivativeConfig, Model, mjd_transition_hybrid};
use sim_mjcf::load_model;

const TOL: f64 = 1e-12;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/kinematics/kinematics.json"))
        .expect("kinematics golden parses")
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

fn vec3s(v: &[Vector3<f64>]) -> Vec<f64> {
    v.iter().flat_map(|x| [x.x, x.y, x.z]).collect()
}

fn quats(v: &[UnitQuaternion<f64>]) -> Vec<f64> {
    v.iter().flat_map(|q| [q.w, q.i, q.j, q.k]).collect()
}

fn to_vec3s(v: &[f64]) -> Vec<Vector3<f64>> {
    v.chunks(3)
        .map(|c| Vector3::new(c[0], c[1], c[2]))
        .collect()
}

fn to_quats(v: &[f64]) -> Vec<UnitQuaternion<f64>> {
    v.chunks(4)
        .map(|c| UnitQuaternion::new_unchecked(Quaternion::new(c[0], c[1], c[2], c[3])))
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

/// A pendulum whose hinge has `ref="30"` is at its MJCF orientation at
/// `qpos0`, and swings as MuJoCo's. Before, the kinematics turned it by
/// `qpos` rather than `qpos − qpos0`: 30° off at `qpos0`.
#[test]
fn hinge_ref_fk_matches_mujoco() {
    let case = &golden()["cases"]["hinge_ref"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    assert_close(
        "qpos0",
        model.qpos0.as_slice(),
        &floats(&case["qpos0"]),
        0.0,
    );
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert_close(
        "xquat at qpos0",
        &quats(&data.xquat),
        &floats(&case["xquat"]),
        0.0,
    );
    assert_close(
        "xpos at qpos0",
        &vec3s(&data.xpos),
        &floats(&case["xpos"]),
        0.0,
    );
    assert_trajectory("hinge_ref", &model, &mut data, &case["traj"]);
}

/// A slide with `ref="0.3"` leaves its body at the MJCF position at `qpos0`,
/// and its anchor includes the joint's `pos` (ledger L36b, A14's X5).
#[test]
fn slide_ref_fk() {
    let case = &golden()["cases"]["slide_ref"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert_close(
        "xpos at qpos0",
        &vec3s(&data.xpos),
        &floats(&case["xpos"]),
        0.0,
    );
    assert_close(
        "xanchor at qpos0",
        &vec3s(&data.xanchor),
        &floats(&case["xanchor"]),
        1e-15,
    );
    assert_trajectory("slide_ref", &model, &mut data, &case["traj"]);
}

/// A ball joint whose `pos` is off its body's origin turns the body about
/// that anchor, alone and below a hinge, as MuJoCo's: the pose, the
/// trajectory, and the transition's A against MuJoCo's finite differences
/// (centred, eps 1e-6). Before, the ball turned the body about its origin;
/// and A's position rows were 6.0e-4 off, the quaternion's tangent taken at
/// the step's start (spec book P31, ledger L115).
#[test]
fn offset_ball_matches_mujoco() {
    for name in ["offset_ball", "chain_offball"] {
        let case = &golden()["cases"][name];
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qpos.copy_from_slice(&floats(&case["qpos"]));
        data.qvel.copy_from_slice(&floats(&case["qvel"]));
        data.forward(&model).expect("forward");
        assert_close(
            &format!("{name} xpos"),
            &vec3s(&data.xpos),
            &floats(&case["xpos"]),
            1e-15,
        );
        assert_close(
            &format!("{name} xanchor"),
            &vec3s(&data.xanchor),
            &floats(&case["xanchor"]),
            1e-15,
        );

        let config = DerivativeConfig {
            eps: 1e-6,
            centered: true,
            ..DerivativeConfig::default()
        };
        let a = mjd_transition_hybrid(&model, &data, &config)
            .expect("transition")
            .A;
        let want = floats(&case["A"]);
        let nx = a.nrows();
        for r in 0..nx {
            let row: Vec<f64> = (0..nx).map(|c| a[(r, c)]).collect();
            assert_close(
                &format!("{name} A row {r}"),
                &row,
                &want[r * nx..(r + 1) * nx],
                1e-8,
            );
        }

        assert_trajectory(name, &model, &mut data, &case["traj"]);
    }
}

/// Every pose `mj_kinematics` writes equals MuJoCo's bit for bit, on hinge,
/// slide, ball, free and mocap bodies, turned geoms and sites, at a state
/// whose quaternions are not of unit length: a ball's of length 1.3, a free
/// joint's within mjMINVAL of 1 (which `mju_normalize4` leaves as it is), a
/// mocap body's of length 0.9 and a body's `body_quat` of length 1.1. The
/// model's frame fields are MuJoCo's (that `body_quat` among them), so
/// the comparison is of the kinematics alone. It includes A14's X1, X3 and X5:
/// a turned ball's `xaxis`, a free joint's (its `jnt_axis`, unturned) and a
/// slide's `xanchor` with its `pos` (ledger L36b).
#[test]
fn kinematics_match_mujoco_bit_for_bit() {
    let case = &golden()["cases"]["frames"];
    let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let fields = &case["model"];
    model.body_pos = to_vec3s(&floats(&fields["body_pos"]));
    model.body_quat = to_quats(&floats(&fields["body_quat"]));
    model.body_ipos = to_vec3s(&floats(&fields["body_ipos"]));
    model.body_iquat = to_quats(&floats(&fields["body_iquat"]));
    model.jnt_pos = to_vec3s(&floats(&fields["jnt_pos"]));
    model.jnt_axis = to_vec3s(&floats(&fields["jnt_axis"]));
    model.qpos0.copy_from_slice(&floats(&fields["qpos0"]));
    model.geom_pos = to_vec3s(&floats(&fields["geom_pos"]));
    model.geom_quat = to_quats(&floats(&fields["geom_quat"]));
    model.site_pos = to_vec3s(&floats(&fields["site_pos"]));
    model.site_quat = to_quats(&floats(&fields["site_quat"]));
    // The copied frames follow the frames, as MuJoCo's compiler derives them.
    model.compute_simple_bodies();
    let mut data = model.make_data();
    data.qpos.copy_from_slice(&floats(&case["qpos"]));
    data.mocap_pos = to_vec3s(&floats(&case["mocap_pos"]));
    data.mocap_quat = to_quats(&floats(&case["mocap_quat"]));
    data.forward(&model).expect("forward");

    let mats = |v: &[nalgebra::Matrix3<f64>]| -> Vec<f64> {
        v.iter()
            .flat_map(|m| (0..9).map(move |i| m[(i / 3, i % 3)]))
            .collect()
    };
    let poses = &case["poses"];
    for (what, got) in [
        ("xpos", vec3s(&data.xpos)),
        ("xquat", quats(&data.xquat)),
        ("xmat", mats(&data.xmat)),
        ("xipos", vec3s(&data.xipos)),
        ("ximat", mats(&data.ximat)),
        ("xanchor", vec3s(&data.xanchor)),
        ("xaxis", vec3s(&data.xaxis)),
        ("geom_xpos", vec3s(&data.geom_xpos)),
        ("geom_xmat", mats(&data.geom_xmat)),
        ("site_xpos", vec3s(&data.site_xpos)),
        ("site_xmat", mats(&data.site_xmat)),
    ] {
        assert_close(what, &got, &floats(&poses[what]), 0.0);
    }
}

/// `dof_length` on the three models of ledger L79, sized from the kinematics
/// at `qpos0`: a hinge whose child's slide has its anchor off the origin, a
/// ball off its body's origin, and a slide with `ref` before a hinge.
/// Before, they read [0.2, 1], 0.05 and [1, 0.3].
#[test]
fn dof_length_matches_mujoco_3_5_0() {
    let golden = golden();
    let cases = golden["cases"]["dof_length"].as_object().expect("cases");
    assert_eq!(cases.len(), 3);
    for (name, case) in cases {
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        assert_close(
            &format!("{name} dof_length"),
            &model.dof_length,
            &floats(&case["dof_length"]),
            1e-15,
        );
    }
}

/// The ball and free joint springs turn `qpos`'s quaternion, normalized as
/// `mju_normalize4` does, toward `qpos_spring`'s as it is
/// (`engine_passive.c:155-161`), and the ball-quaternion sensor normalizes as
/// `mju_normalize4` (`engine_sensor.c:572-575`): bit for bit, with
/// `qpos_spring` set to quaternions not of unit length and `qpos`'s within
/// mjMINVAL of unit length (the ball's) or of length 1.25 (the free joint's).
#[test]
fn quat_springs_and_ballquat_match_mujoco_bit_for_bit() {
    let case = &golden()["cases"]["quat_springs"];
    let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    model.qpos_spring = floats(&case["qpos_spring"]);
    for (i, state) in case["states"]
        .as_array()
        .expect("states")
        .iter()
        .enumerate()
    {
        let mut data = model.make_data();
        data.qpos.copy_from_slice(&floats(&state["qpos"]));
        data.forward(&model).expect("forward");
        assert_close(
            &format!("state {i} qfrc_spring"),
            data.qfrc_spring.as_slice(),
            &floats(&state["qfrc_spring"]),
            0.0,
        );
        assert_close(
            &format!("state {i} sensordata"),
            data.sensordata.as_slice(),
            &floats(&state["sensordata"]),
            0.0,
        );
    }
}
