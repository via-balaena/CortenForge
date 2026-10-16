//! Constraint impedance against MuJoCo 3.5.0.
//!
//! MuJoCo takes one impedance for all rows of a connect (3) or a weld (6),
//! from the norm of their violation (`getposdim`,
//! `engine_core_constraint.c:1365-1398`), and a row's impedance from
//! `pos - margin` (`getimpedance`, `:1413-1461`), and replaces a mixed
//! `solref` or `solreffriction` (`getsolparam`, `:1291-1362`). The golden is
//! `assets/golden/impedance/impedance.json`, from the unfused MuJoCo 3.5.0
//! oracle (`scripts/gen_impedance_reference.py`, which describes its cases).

use serde_json::Value;
use sim_core::{ConstraintType, Data, Model};
use sim_mjcf::load_model;

const TOL: f64 = 1e-12;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/impedance/impedance.json"))
        .expect("impedance golden parses")
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

/// qpos and qvel at the golden's checkpoints, stepping `model` from `data`;
/// returns the data at the last checkpoint.
fn assert_trajectory(what: &str, model: &Model, mut data: Data, traj: &Value, tol: f64) -> Data {
    let checkpoints = traj.as_object().expect("checkpoints");
    let last = checkpoints
        .keys()
        .map(|k| k.parse::<usize>().expect("step"))
        .max();
    for step in 1..=last.expect("a checkpoint") {
        data.step(model).expect("step");
        if let Some(c) = checkpoints.get(&step.to_string()) {
            assert_close(
                &format!("{what} step {step} qpos"),
                data.qpos.as_slice(),
                &floats(&c["qpos"]),
                tol,
            );
            assert_close(
                &format!("{what} step {step} qvel"),
                data.qvel.as_slice(),
                &floats(&c["qvel"]),
                tol,
            );
        }
    }
    data
}

/// A hinged capsule and a free box joined by a connect, the hinge turned
/// 0.002 rad: each of the connect's three rows takes the impedance of the
/// norm of their violation, as MuJoCo's, and the acceleration follows.
#[test]
fn connect_impedance_is_shared_across_rows() {
    let case = &golden()["cases"]["connect_rows"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos[0] = case["hinge"].as_f64().expect("hinge");
    data.forward(&model).expect("forward");
    let rows: Vec<usize> = (0..data.efc_type.len())
        .filter(|&i| data.efc_type[i] == ConstraintType::Equality)
        .collect();
    let pos: Vec<f64> = rows.iter().map(|&i| data.efc_pos[i]).collect();
    assert_close("connect efc_pos", &pos, &floats(&case["efc_pos"]), TOL);
    let imp: Vec<f64> = rows.iter().map(|&i| data.efc_imp[i]).collect();
    assert_close("connect efc_imp", &imp, &floats(&case["efc_imp"]), 1e-15);
    assert_close("qacc", data.qacc.as_slice(), &floats(&case["qacc"]), TOL);
}

/// A box on a hinge welded to the world, started 0.008 rad from its weld pose,
/// under Euler, implicit and implicitfast.
#[test]
fn weld_matches_mujoco_3_5_0() {
    let case = &golden()["cases"]["weld_hinge"];
    for (integrator, traj) in case["traj"].as_object().expect("traj") {
        let model = load_model(case["xml"][integrator].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qpos[0] += case["hinge"].as_f64().expect("hinge");
        assert_trajectory(&format!("weld {integrator}"), &model, data, traj, TOL);
    }
}

/// The composite cable of `implicit_integration.rs`'s
/// `test_implicitfast_connect_ball_chain_stability`, as MuJoCo writes it out
/// (plain bodies), its last segment connected to a fixed body.
#[test]
fn a_connected_cable_matches_mujoco_3_5_0() {
    let case = &golden()["cases"]["cable"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let data = model.make_data();
    assert_trajectory("cable", &model, data, &case["traj"], CABLE_TOL);
}

const CABLE_TOL: f64 = 1e-12;

/// A hinge limited to ±0.5 with margin 0.1, at 0.42 rad: its upper limit
/// row is 0.08 from the limit, 0.02 inside the margin, and takes the
/// impedance at 0.02.
#[test]
fn a_limit_takes_its_impedance_inside_the_margin() {
    let case = &golden()["cases"]["limit_margin"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos[0] = case["hinge"].as_f64().expect("hinge");
    data.forward(&model).expect("forward");
    let rows: Vec<usize> = (0..data.efc_type.len())
        .filter(|&i| data.efc_type[i] == ConstraintType::LimitJoint)
        .collect();
    assert_eq!(rows.len(), 1, "one limit row");
    let i = rows[0];
    let want = |k: &str| case[k].as_f64().expect(k);
    assert_close("limit efc_pos", &[data.efc_pos[i]], &[want("efc_pos")], TOL);
    assert_close(
        "limit efc_margin",
        &[data.efc_margin[i]],
        &[want("efc_margin")],
        TOL,
    );
    assert_close(
        "limit efc_imp",
        &[data.efc_imp[i]],
        &[want("efc_imp")],
        1e-15,
    );
}

/// Census doc d81df1fa, a sphere sliding on a plane under an elliptic cone
/// whose `<pair>` gives `solreffriction="0.1 0.0"`: one value positive, the
/// other not. MuJoCo replaces it by (0, 0), so the friction rows take the
/// pair's `solref` (`getsolparam`, `engine_core_constraint.c:1345-1349`;
/// `mj_makeImpedance`, `:1489-1491`).
///
/// The second case gives the pair `solref="0.05 1"`, not the default, and
/// `solreffriction="-0.1 0.5"`, mixed the other way: there a mixed
/// solreffriction replaced by the default solref, not by (0, 0), would show.
#[test]
fn mixed_solreffriction_uses_solref() {
    for name in ["mixed_solreffriction", "mixed_solreffriction_own_solref"] {
        let case = &golden()["cases"][name];
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qvel.copy_from_slice(&floats(&case["qvel"]));
        assert_trajectory(name, &model, data, &case["traj"], TOL);
    }
}

/// Two hinged arms falling onto their limits, whose `solreflimit` is mixed
/// each way ("0.05 -2", "-500 2"): MuJoCo replaces it by the default
/// (0.02, 1) (`getsolparam`, `engine_core_constraint.c:1334-1338`), which the
/// limit rows then hold.
#[test]
fn mixed_solref_uses_default() {
    let case = &golden()["cases"]["mixed_solref_limit"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let end = assert_trajectory(
        "mixed solref limit",
        &model,
        model.make_data(),
        &case["traj"],
        TOL,
    );
    assert_eq!(
        end.efc_type,
        [ConstraintType::LimitJoint; 2],
        "two limit rows"
    );
    assert_eq!(end.efc_solref, [[0.02, 1.0]; 2], "the limit rows' solref");
}
