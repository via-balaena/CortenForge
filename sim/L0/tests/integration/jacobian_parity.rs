//! The Jacobians against MuJoCo 3.5.0: `mj_jac` built from MuJoCo's `cdof`
//! about the subtree centre of mass (`engine_core_util.c:172-220`), and
//! `mj_applyFT`'s transposes (`engine_support.c:446-501`).
//!
//! The golden is `assets/golden/jacobian/jacobian.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_jacobian_reference.py`, which describes
//! its cases).

use nalgebra::{DVector, Quaternion, UnitQuaternion, Vector3};
use serde_json::Value;
use sim_core::{Data, Model, mj_apply_ft, mj_jac};
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/jacobian/jacobian.json"))
        .expect("jacobian golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn vec3(v: &Value) -> Vector3<f64> {
    Vector3::from_column_slice(&floats(v))
}

fn assert_bits(what: &str, got: &[f64], want: &[f64]) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    let differ: Vec<String> = got
        .iter()
        .zip(want)
        .enumerate()
        .filter(|(_, (a, b))| a.to_bits() != b.to_bits())
        .map(|(i, (a, b))| format!("[{i}] {a:e}, MuJoCo {b:e}"))
        .collect();
    assert!(
        differ.is_empty(),
        "{what}: {} differ: {}",
        differ.len(),
        differ.join("; ")
    );
}

fn vec3s(v: &Value) -> Vec<Vector3<f64>> {
    floats(v)
        .chunks(3)
        .map(Vector3::from_column_slice)
        .collect()
}

/// The case's model with MuJoCo's frame and mass fields written in (the
/// loader rounds a normalized axis or quaternion otherwise), at its state.
fn at_state(case: &Value) -> (Model, Data) {
    let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let f = &case["model"];
    let quats = |v: &Value| -> Vec<UnitQuaternion<f64>> {
        floats(v)
            .chunks(4)
            .map(|q| UnitQuaternion::new_unchecked(Quaternion::new(q[0], q[1], q[2], q[3])))
            .collect()
    };
    model.body_pos = vec3s(&f["body_pos"]);
    model.body_quat = quats(&f["body_quat"]);
    model.body_ipos = vec3s(&f["body_ipos"]);
    model.body_iquat = quats(&f["body_iquat"]);
    model.body_mass = floats(&f["body_mass"]);
    model.body_inertia = vec3s(&f["body_inertia"]);
    model.jnt_pos = vec3s(&f["jnt_pos"]);
    model.jnt_axis = vec3s(&f["jnt_axis"]);
    model.qpos0.copy_from_slice(&floats(&f["qpos0"]));
    model.recompute_derived().expect("recompute_derived");
    let mut data = model.make_data();
    data.qpos.copy_from_slice(&floats(&case["qpos"]));
    data.forward(&model).expect("forward");
    (model, data)
}

fn row_major(m: &nalgebra::DMatrix<f64>) -> Vec<f64> {
    (0..m.nrows())
        .flat_map(|r| (0..m.ncols()).map(move |c| m[(r, c)]))
        .collect()
}

/// `mj_jac` at a point near each body's centre of mass: a hinge and a ball
/// off their bodies' origins, a slide and a hinge on one body, a jointless
/// body welded to it, a free body; and `subtree_com`, which it reads.
#[test]
fn jacobians_match_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["jac"];
    let (model, data) = at_state(case);
    let com: Vec<f64> = data
        .subtree_com
        .iter()
        .flat_map(|c| [c.x, c.y, c.z])
        .collect();
    assert_bits("subtree_com", &com, &floats(&case["subtree_com"]));
    for b in case["bodies"].as_array().expect("bodies") {
        let body = usize::try_from(b["body"].as_u64().expect("body")).expect("fits");
        let (jacp, jacr) = mj_jac(&model, &data, body, &vec3(&b["point"]));
        assert_bits(
            &format!("body {body} jacp"),
            &row_major(&jacp),
            &floats(&b["jacp"]),
        );
        assert_bits(
            &format!("body {body} jacr"),
            &row_major(&jacr),
            &floats(&b["jacr"]),
        );
    }
}

/// `mj_applyFT` of a force and a torque at that point, added to a vector that
/// is not 0: the force's term first, then the torque's.
#[test]
fn apply_ft_matches_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["apply_ft"];
    let (model, data) = at_state(case);
    for b in case["bodies"].as_array().expect("bodies") {
        let body = usize::try_from(b["body"].as_u64().expect("body")).expect("fits");
        let mut qfrc = DVector::from_vec(floats(&b["qfrc0"]));
        mj_apply_ft(
            &model,
            &data,
            &vec3(&b["force"]),
            &vec3(&b["torque"]),
            &vec3(&b["point"]),
            body,
            &mut qfrc,
        );
        assert_bits(
            &format!("body {body} qfrc"),
            qfrc.as_slice(),
            &floats(&b["qfrc"]),
        );
    }
}

/// Gravity compensation projects each body's weight through `mj_applyFT`.
#[test]
fn gravity_compensation_matches_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["gravcomp"];
    let (_, data) = at_state(case);
    assert_bits(
        "qfrc_gravcomp",
        data.qfrc_gravcomp.as_slice(),
        &floats(&case["qfrc_gravcomp"]),
    );
}
