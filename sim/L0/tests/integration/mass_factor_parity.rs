//! The mass matrix's factor and solve against MuJoCo 3.5.0: `L'·D·L` on the
//! mass matrix's own pattern, a simple dof's row its diagonal only
//! (`mj_factorM`, `engine_core_smooth.c:1961-1986`), solved as `mj_solveM`
//! solves (`:2131-2224`), and Euler's damped factor in `qH`
//! (`mj_EulerSkip`, `engine_forward.c:943-1009`).
//!
//! The golden is `assets/golden/mass_factor/mass_factor.json`, from the
//! unfused MuJoCo 3.5.0 oracle (`scripts/gen_mass_factor_reference.py`, which
//! describes its cases).

use serde_json::Value;
use sim_core::{Data, Model, mj_factor_m, mj_solve_m};
use sim_mjcf::load_model;

const TOL: f64 = 1e-12;

fn golden() -> Value {
    serde_json::from_str(include_str!(
        "../assets/golden/mass_factor/mass_factor.json"
    ))
    .expect("mass factor golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
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

fn assert_close(what: &str, got: &[f64], want: &[f64]) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    for (i, (a, b)) in got.iter().zip(want).enumerate() {
        assert!((a - b).abs() <= TOL, "{what}[{i}]: {a:e}, MuJoCo {b:e}");
    }
}

fn start(case: &Value) -> (Model, Data) {
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    if let Some(qpos) = case.get("qpos") {
        data.qpos.copy_from_slice(&floats(qpos));
    }
    data.qvel.copy_from_slice(&floats(&case["qvel"]));
    (model, data)
}

fn rows(model: &Model, i: usize) -> std::ops::Range<usize> {
    model.M_rowadr[i]..model.M_rowadr[i] + model.M_rownnz[i]
}

/// `qM` on the mass matrix's pattern, as MuJoCo's `d.M` stores it.
fn m_on_pattern(model: &Model, data: &Data) -> Vec<f64> {
    (0..model.nv)
        .flat_map(|i| rows(model, i).map(move |adr| data.qM[(i, model.M_colind[adr])]))
        .collect()
}

/// Writes MuJoCo's `d.M`, given on the pattern, into `qM`, both triangles.
fn set_m(model: &Model, data: &mut Data, m: &[f64]) {
    assert_eq!(m.len(), model.M_colind.len(), "M on the pattern: length");
    for i in 0..model.nv {
        for adr in rows(model, i) {
            let j = model.M_colind[adr];
            data.qM[(i, j)] = m[adr];
            data.qM[(j, i)] = m[adr];
        }
    }
}

/// MuJoCo's mass matrix of a hinge-hinge-ball chain with a hinge off its
/// root, factored and solved here: its `qLD`, `qLDiagInv` and `mj_solveM`
/// bit for bit, of two vectors, the second one whose solve rounds otherwise
/// when a row's sum is taken left to right. (sim-core's own mass matrix of
/// the chain rounds otherwise than MuJoCo's, so the test factors MuJoCo's.)
#[test]
fn mass_matrix_factor_and_solve_match_mujoco_3_5_0() {
    let case = &golden()["cases"]["chain"];
    let (model, mut data) = start(case);
    data.forward(&model).expect("forward");
    set_m(&model, &mut data, &floats(&case["M"]));
    mj_factor_m(&model, &mut data);
    assert_bits("chain qLD", &data.qLD_data, &floats(&case["qLD"]));
    assert_bits(
        "chain qLDiagInv",
        &data.qLD_diag_inv,
        &floats(&case["qLDiagInv"]),
    );
    for (y, want) in [("y", "x"), ("y2", "x2")] {
        let mut x = floats(&case[y]);
        mj_solve_m(&model, &data, &mut x);
        assert_bits(&format!("chain mj_solveM of {y}"), &x, &floats(&case[want]));
    }
}

/// With a tree asleep, the factor is made over the awake rows: they become
/// MuJoCo's factor of its mass matrix, and a sleeping row keeps what it
/// held (here a sentinel).
#[test]
fn mass_factor_leaves_the_sleeping_rows_as_mujoco_3_5_0() {
    const SENTINEL: f64 = 7.0;
    let case = &golden()["cases"]["sleeping"];
    let (model, mut data) = start(case);
    for _ in 0..case["steps"].as_u64().expect("steps") {
        data.step(&model).expect("step");
    }
    let awake: Vec<usize> = case["awake"]
        .as_array()
        .expect("awake")
        .iter()
        .map(|v| usize::try_from(v.as_u64().expect("dof")).expect("fits"))
        .collect();
    assert_eq!(
        &data.dof_awake_ind[..data.nv_awake],
        awake.as_slice(),
        "awake dofs"
    );

    set_m(&model, &mut data, &floats(&case["M"]));
    let (mut want_qld, mut want_inv) = (floats(&case["qLD"]), floats(&case["qLDiagInv"]));
    for i in 0..model.nv {
        let fill = if awake.contains(&i) {
            f64::NAN
        } else {
            want_qld[rows(&model, i)].fill(SENTINEL);
            want_inv[i] = SENTINEL;
            SENTINEL
        };
        data.qLD_data[rows(&model, i)].fill(fill);
        data.qLD_diag_inv[i] = fill;
    }
    mj_factor_m(&model, &mut data);
    assert_bits("sleeping qLD", &data.qLD_data, &want_qld);
    assert_bits("sleeping qLDiagInv", &data.qLD_diag_inv, &want_inv);
}

/// Euler with dof damping factors `M + h·diag(damping)` into `qH` and
/// steps with it, as `mj_EulerSkip` does.
#[test]
fn euler_damping_factor_matches_mujoco_3_5_0() {
    let case = &golden()["cases"]["eulerdamp"];
    let traj = &case["traj"];
    let (model, mut data) = start(case);
    for step in 1..=100 {
        data.step(&model).expect("step");
        if step == 1 {
            assert_close("eulerdamp qH", &data.qH, &floats(&traj["first"]["qH"]));
            assert_close(
                "eulerdamp qHDiagInv",
                &data.qHDiagInv,
                &floats(&traj["first"]["qHDiagInv"]),
            );
        }
        if let Some(at) = traj.get(step.to_string()) {
            assert_close(
                &format!("eulerdamp qpos at {step}"),
                data.qpos.as_slice(),
                &floats(&at["qpos"]),
            );
            assert_close(
                &format!("eulerdamp qvel at {step}"),
                data.qvel.as_slice(),
                &floats(&at["qvel"]),
            );
        }
    }
}

/// With a tree asleep from the start, Euler's damped factor is made over
/// the awake rows: the sleeping rows of `qH`, never filled, stay 0.
#[test]
fn euler_damping_factor_leaves_the_sleeping_rows_as_mujoco_3_5_0() {
    let case = &golden()["cases"]["eulerdamp_sleeping"];
    let (model, mut data) = start(case);
    data.step(&model).expect("step");
    assert_close("eulerdamp_sleeping qH", &data.qH, &floats(&case["qH"]));
    assert_close(
        "eulerdamp_sleeping qHDiagInv",
        &data.qHDiagInv,
        &floats(&case["qHDiagInv"]),
    );
}

/// A simple body with two slides along one axis: MuJoCo keeps its block of
/// the mass matrix diagonal, so each slide accelerates under its own mass
/// and armature (ledger L118). The whole `qM` is MuJoCo's (`mj_fullM`), the
/// two slides uncoupled, so what reads it densely reads MuJoCo's: with an
/// applied force, `inverse` after `forward` gives MuJoCo's `qfrc_inverse`
/// bit for bit, the applied force to rounding.
#[test]
fn two_slides_on_one_axis_step_as_mujoco_3_5_0() {
    let case = &golden()["cases"]["two_slides"];
    let want = &case["forward"];
    let (model, mut data) = start(case);
    data.forward(&model).expect("forward");
    assert_bits(
        "two_slides M",
        &m_on_pattern(&model, &data),
        &floats(&want["M"]),
    );
    assert_bits("two_slides qLD", &data.qLD_data, &floats(&want["qLD"]));
    assert_bits(
        "two_slides qLDiagInv",
        &data.qLD_diag_inv,
        &floats(&want["qLDiagInv"]),
    );
    assert_bits(
        "two_slides qacc",
        data.qacc.as_slice(),
        &floats(&want["qacc"]),
    );
    assert_bits(
        "two_slides qM",
        data.qM.transpose().as_slice(),
        &floats(&want["qM"]),
    );

    let inverse = &case["inverse"];
    let (model, mut data) = start(case);
    data.qfrc_applied
        .copy_from_slice(&floats(&inverse["qfrc_applied"]));
    data.forward(&model).expect("forward");
    data.inverse(&model);
    assert_bits(
        "two_slides qfrc_inverse",
        data.qfrc_inverse.as_slice(),
        &floats(&inverse["qfrc_inverse"]),
    );

    let (model, mut data) = start(case);
    for step in 1..=10 {
        data.step(&model).expect("step");
        if let Some(at) = case["traj"].get(step.to_string()) {
            assert_close(
                &format!("two_slides qvel at {step}"),
                data.qvel.as_slice(),
                &floats(&at["qvel"]),
            );
        }
    }
}
