//! The Jacobians against MuJoCo 3.5.0: `mj_jac` built from MuJoCo's `cdof`
//! about the subtree centre of mass (`engine_core_util.c:172-220`),
//! `mj_applyFT`'s transposes (`engine_support.c:446-501`), and a contact's
//! rows (`mj_contactJacobian`, `engine_core_constraint.c:907-1087`).
//!
//! The golden is `assets/golden/jacobian/jacobian.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_jacobian_reference.py`, which describes
//! its cases).

use nalgebra::{DVector, Quaternion, UnitQuaternion, Vector3};
use serde_json::Value;
use sim_core::{
    Contact, Data, Model, compute_contact_jacobian, mj_apply_ft, mj_energy_vel, mj_jac, mj_mul_m,
};
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
    at_state_of(
        load_model(case["xml"].as_str().expect("xml")).expect("load"),
        case,
    )
}

fn at_state_of(mut model: Model, case: &Value) -> (Model, Data) {
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

/// A contact's Jacobian rows in its frame, for MuJoCo's own contacts (their
/// geoms, position, frame and dimension written into a sim-core contact):
/// a free box on a plane (condim 3), a hinged capsule (6) and a sphere on a
/// ball joint below it (4), each row bit for bit with MuJoCo's `efc_J`
/// (an elliptic cone, whose rows are the Jacobian's).
#[test]
fn contact_jacobians_match_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["contacts"];
    let (model, data) = at_state(case);
    for (i, c) in case["contacts"]
        .as_array()
        .expect("contacts")
        .iter()
        .enumerate()
    {
        let geom: Vec<usize> = c["geom"]
            .as_array()
            .expect("geom")
            .iter()
            .map(|g| usize::try_from(g.as_u64().expect("geom")).expect("fits"))
            .collect();
        let frame = floats(&c["frame"]);
        let dim = usize::try_from(c["dim"].as_u64().expect("dim")).expect("fits");
        let contact = Contact {
            pos: vec3(&c["pos"]),
            normal: Vector3::from_column_slice(&frame[0..3]),
            depth: 0.0,
            geom1: geom[0],
            geom2: geom[1],
            friction: 1.0,
            dim,
            includemargin: 0.0,
            mu: [1.0, 1.0, 0.005, 0.0001, 0.0001],
            solref: [0.02, 1.0],
            solreffriction: [0.0, 0.0],
            solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
            frame: [
                Vector3::from_column_slice(&frame[3..6]),
                Vector3::from_column_slice(&frame[6..9]),
            ],
            flex_vertex: None,
            flex_vertex2: None,
        };
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_bits(
            &format!("contact {i} (dim {dim})"),
            &row_major(&j),
            &floats(&c["efc_J"]),
        );
    }
}

/// Connect and weld rows: a connect between a box on a ball joint (below a
/// hinged arm) and a free sphere, a weld between the sphere and the arm, a
/// weld of the box to the world, at a moved state, `efc_J` and `efc_pos`
/// bit for bit with MuJoCo's. MuJoCo's `eq_data` (its compiler's anchors
/// and `relpose`) is written in, as the frames are.
#[test]
fn equality_rows_match_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["equalities"];
    let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let eq_data = floats(&case["eq_data"]);
    for (eq, chunk) in model.eq_data.iter_mut().zip(eq_data.chunks(11)) {
        eq.copy_from_slice(chunk);
    }
    let (_, data) = at_state_of(model, case);
    assert_eq!(data.efc_J.nrows(), 15, "the equality rows alone");
    assert_bits("efc_J", &row_major(&data.efc_J), &floats(&case["efc_J"]));
    assert_bits("efc_pos", &data.efc_pos, &floats(&case["efc_pos"]));
}

/// Spatial tendons through sites on a hinged arm, a box on a ball joint
/// below it and a free sphere, one with a pulley: `ten_J` bit for bit with
/// MuJoCo's (each segment between different bodies adds its Jacobians'
/// difference projected on its normalized direction, scaled by 1/divisor).
#[test]
fn spatial_tendon_jacobians_match_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["tendons"];
    let (_, data) = at_state(case);
    let sites: Vec<f64> = data
        .site_xpos
        .iter()
        .flat_map(|p| [p.x, p.y, p.z])
        .collect();
    assert_bits("site_xpos", &sites, &floats(&case["site_xpos"]));
    assert_bits("ten_length", &data.ten_length, &floats(&case["ten_length"]));
    let ten_j: Vec<f64> = data
        .ten_J
        .iter()
        .flat_map(|row| row.iter().copied())
        .collect();
    assert_bits("ten_J", &ten_j, &floats(&case["ten_J"]));
}

/// MuJoCo's mass matrix of a chain of seven hinges written into `qM`, times
/// `qvel` (`mj_mulM`) and the kinetic energy from it (`mj_energyVel`), bit
/// for bit, at velocities that make the order of each sum show (the
/// generator checks that). sim-core's own mass matrix rounds otherwise, as
/// the mass factor golden shows.
#[test]
fn mass_product_and_kinetic_energy_match_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["energy"];
    let (model, mut data) = at_state(case);
    data.qvel.copy_from_slice(&floats(&case["qvel"]));
    let m = floats(&case["M"]);
    for i in 0..model.nv {
        let row = model.M_rowadr[i]..model.M_rowadr[i] + model.M_rownnz[i];
        for (&j, &v) in model.M_colind[row.clone()].iter().zip(&m[row]) {
            data.qM[(i, j)] = v;
            data.qM[(j, i)] = v;
        }
    }
    let mqvel = mj_mul_m(&model, &data, data.qvel.as_slice());
    assert_bits("mj_mulM", mqvel.as_slice(), &floats(&case["mul_m"]));
    mj_energy_vel(&model, &mut data);
    assert_bits(
        "energy_kinetic",
        &[data.energy_kinetic],
        &[case["energy_kinetic"].as_f64().expect("energy")],
    );
}
