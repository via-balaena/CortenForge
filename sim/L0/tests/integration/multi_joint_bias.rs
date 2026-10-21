//! Bodies carrying two or more joints against MuJoCo 3.5.0.
//!
//! MuJoCo's `mj_comVel` steps a body's velocity over its joints in order and
//! takes each joint's velocity product with the velocity the earlier joints
//! leave (`engine_core_smooth.c:2285-2336`; a ball's three dofs share the
//! velocity before it, `:2315-2317`). The golden is
//! `assets/golden/multijoint/multijoint.json`, from the unfused MuJoCo 3.5.0
//! oracle (`scripts/gen_multijoint_reference.py`, which describes its cases).

use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use serde_json::Value;
use sim_core::{Data, DerivativeConfig, Model, mjd_transition};
use sim_mjcf::load_model;

const TOL: f64 = 1e-12;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/multijoint/multijoint.json"))
        .expect("multijoint golden parses")
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

fn data_at(model: &Model, case: &Value) -> Data {
    let mut data = model.make_data();
    data.qpos.copy_from_slice(&floats(&case["qpos"]));
    data.qvel.copy_from_slice(&floats(&case["qvel"]));
    data
}

/// `R(q)·diag(d)·Rᵀ`, which does not depend on the principal axes' order.
fn tensor(q: &UnitQuaternion<f64>, d: &Vector3<f64>) -> Matrix3<f64> {
    let r = q.to_rotation_matrix();
    r.matrix() * Matrix3::from_diagonal(d) * r.matrix().transpose()
}

fn quat(v: &[f64]) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(v[0], v[1], v[2], v[3]))
}

/// The fields the dynamics read agree, so a difference below is not the fixture's.
fn assert_same_fixture(name: &str, model: &Model, golden: &Value) {
    let g = |k: &str| floats(&golden[k]);
    let mass: Vec<f64> = model.body_mass.clone();
    assert_close(&format!("{name} body_mass"), &mass, &g("body_mass"), TOL);
    let (gi, gq) = (g("body_inertia"), g("body_iquat"));
    for b in 0..model.nbody {
        let ours = tensor(&model.body_iquat[b], &model.body_inertia[b]);
        let theirs = tensor(
            &quat(&gq[4 * b..4 * b + 4]),
            &Vector3::new(gi[3 * b], gi[3 * b + 1], gi[3 * b + 2]),
        );
        assert!(
            (ours - theirs).amax() <= TOL,
            "{name} body {b} inertia tensor: ours {ours}, MuJoCo {theirs}"
        );
    }
    let flat3 = |v: &[Vector3<f64>]| v.iter().flat_map(|x| [x.x, x.y, x.z]).collect::<Vec<_>>();
    let flatq = |v: &[UnitQuaternion<f64>]| {
        v.iter()
            .flat_map(|q| [q.w, q.i, q.j, q.k])
            .collect::<Vec<_>>()
    };
    assert_close(
        &format!("{name} body_ipos"),
        &flat3(&model.body_ipos),
        &g("body_ipos"),
        TOL,
    );
    assert_close(
        &format!("{name} body_pos"),
        &flat3(&model.body_pos),
        &g("body_pos"),
        TOL,
    );
    assert_close(
        &format!("{name} body_quat"),
        &flatq(&model.body_quat),
        &g("body_quat"),
        TOL,
    );
    assert_close(
        &format!("{name} jnt_pos"),
        &flat3(&model.jnt_pos),
        &g("jnt_pos"),
        TOL,
    );
    assert_close(
        &format!("{name} jnt_axis"),
        &flat3(&model.jnt_axis),
        &g("jnt_axis"),
        TOL,
    );
    assert_close(
        &format!("{name} qpos0"),
        model.qpos0.as_slice(),
        &g("qpos0"),
        TOL,
    );
}

/// Each case at its state: the bias force, the acceleration, the mass matrix,
/// and each body's acceleration and internal force, about the subtree centre
/// of mass of the body's root, as MuJoCo's.
#[test]
fn multi_joint_bodies_match_mujoco_3_5_0_at_a_state() {
    let golden = golden();
    for (name, case) in golden["cases"].as_object().expect("cases") {
        let model = load_model(case["xml"]["Euler"].as_str().expect("xml")).expect("load");
        assert_same_fixture(name, &model, &case["model"]);
        let mut data = data_at(&model, case);
        data.forward(&model).expect("forward");
        let f = &case["forward"];
        assert_close(
            &format!("{name} qfrc_bias"),
            data.qfrc_bias.as_slice(),
            &floats(&f["qfrc_bias"]),
            TOL,
        );
        assert_close(
            &format!("{name} qacc"),
            data.qacc.as_slice(),
            &floats(&f["qacc"]),
            TOL,
        );
        let qm: Vec<f64> = data.qM.transpose().iter().copied().collect();
        assert_close(&format!("{name} qM"), &qm, &floats(&f["qM"]), TOL);
        data.inverse(&model).expect("inverse");
        for b in 0..model.nbody {
            assert_close(
                &format!("{name} cacc[{b}]"),
                data.cacc[b].as_slice(),
                &floats(&f["cacc"][b]),
                TOL,
            );
            assert_close(
                &format!("{name} cfrc_int[{b}]"),
                data.cfrc_int[b].as_slice(),
                &floats(&f["cfrc_int"][b]),
                TOL,
            );
        }
    }
}

/// Each case stepped from its state under Euler, RK4, implicit and
/// implicitfast; qpos and qvel at steps 1, 10, 100 and 500.
#[test]
fn multi_joint_bodies_step_as_mujoco_3_5_0() {
    let golden = golden();
    for (name, case) in golden["cases"].as_object().expect("cases") {
        for (integrator, checkpoints) in case["traj"].as_object().expect("traj") {
            let model = load_model(case["xml"][integrator].as_str().expect("xml")).expect("load");
            let mut data = data_at(&model, case);
            for step in 1..=500 {
                data.step(&model).expect("step");
                if let Some(c) = checkpoints.get(step.to_string()) {
                    let what = format!("{name} {integrator} step {step}");
                    assert_close(
                        &format!("{what} qpos"),
                        data.qpos.as_slice(),
                        &floats(&c["qpos"]),
                        TRAJ_TOL,
                    );
                    assert_close(
                        &format!("{what} qvel"),
                        data.qvel.as_slice(),
                        &floats(&c["qvel"]),
                        TRAJ_TOL,
                    );
                }
            }
        }
    }
}

const TRAJ_TOL: f64 = 1e-12;

/// The same joints on one body and split over bodies chained through 1e-9 kg
/// intermediates accelerate alike, in sim-core and in MuJoCo. The
/// intermediates' inertia moves the acceleration by up to 4e-7 of it in
/// MuJoCo 3.5.0 on these cases, so the two agree relatively, to 1e-5.
#[test]
fn a_multi_joint_body_accelerates_as_its_joints_split_over_bodies() {
    let golden = golden();
    let mut checked = 0;
    for (name, case) in golden["cases"].as_object().expect("cases") {
        let Some(split) = case.get("split") else {
            continue;
        };
        let one = load_model(case["xml"]["Euler"].as_str().expect("xml")).expect("load");
        let mut data = data_at(&one, case);
        data.forward(&one).expect("forward");
        let apart = load_model(split["xml"].as_str().expect("xml")).expect("load split");
        let mut split_data = data_at(&apart, case);
        split_data.forward(&apart).expect("forward split");
        for (i, (a, b)) in split_data.qacc.iter().zip(data.qacc.iter()).enumerate() {
            assert!(
                (a - b).abs() <= 1e-5 * b.abs().max(1.0),
                "{name} qacc[{i}]: split {a}, one body {b}"
            );
        }
        assert_close(
            &format!("{name} split vs MuJoCo"),
            split_data.qacc.as_slice(),
            &floats(&split["qacc"]),
            TOL,
        );
        checked += 1;
    }
    assert_eq!(checked, 3);
}

/// The analytic transition derivative agrees with finite differences, A and B
/// entry by entry, on every case: B's column is a motor's at a site on the
/// multi-joint body, at a control of 0.
#[test]
fn multi_joint_bodies_have_the_finite_difference_transition_derivative() {
    let golden = golden();
    for (name, case) in golden["cases"].as_object().expect("cases") {
        // `mjd_transition` refuses RK4.
        for integrator in ["Euler", "implicit", "implicitfast"] {
            let model = load_model(case["xml"][integrator].as_str().expect("xml")).expect("load");
            let mut data = data_at(&model, case);
            data.forward(&model).expect("forward");
            let derivative = |use_analytical| {
                let config = DerivativeConfig {
                    use_analytical,
                    ..Default::default()
                };
                mjd_transition(&model, &data, &config).expect("derivative")
            };
            let (fd, analytic) = (derivative(false), derivative(true));
            assert_eq!(fd.B.ncols(), 1, "{name}: B has the motor's column");
            let a = (&fd.A - &analytic.A).amax();
            let b = (&fd.B - &analytic.B).amax();
            assert!(
                a <= FD_TOL && b <= FD_TOL,
                "{name} {integrator}: A {a:e}, B {b:e}"
            );
        }
    }
}

const FD_TOL: f64 = 1e-8;
