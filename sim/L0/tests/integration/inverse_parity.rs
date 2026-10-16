//! Inverse dynamics against MuJoCo 3.5.0: `mj_inverse` takes the constraint
//! force from `qacc` (`mj_invConstraint`, `engine_inverse.c:157-180`), each
//! row's state and force as `mj_constraintUpdate` makes them at
//! `jar = J·qacc − aref`; `mjd_inverseFD` differentiates that; and
//! `mj_compareFwdInv` (`:262-302`) reports the change and puts the forward's
//! force back.
//!
//! The golden is `assets/golden/inverse/inverse.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_inverse_reference.py`, which describes
//! its cases).

use serde_json::Value;
use sim_core::{ConstraintState, Data, DerivativeConfig, Model, mjd_inverse_fd};
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/inverse/inverse.json"))
        .expect("inverse golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn bits(v: &[f64]) -> Vec<u64> {
    v.iter().map(|x| x.to_bits()).collect()
}

/// Each row's state as MuJoCo numbers it (`mjtConstraintState`).
fn mujoco_states(states: &[ConstraintState]) -> Vec<i64> {
    states
        .iter()
        .map(|s| match s {
            ConstraintState::Satisfied => 0,
            ConstraintState::Quadratic => 1,
            ConstraintState::LinearNeg => 2,
            ConstraintState::LinearPos => 3,
            ConstraintState::Cone => 4,
        })
        .collect()
}

/// The case's model and its state after `forward`, from MuJoCo's qpos,
/// qvel, warmstart and time (where the case gives them).
fn at_state(case: &Value) -> (Model, Data) {
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qpos"]));
    data.qvel
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qvel"]));
    if !case["qacc_warmstart"].is_null() {
        data.qacc_warmstart
            .as_mut_slice()
            .copy_from_slice(&floats(&case["qacc_warmstart"]));
        data.time = case["time"].as_f64().expect("time");
    }
    if let Some(xfrc) = case["xfrc_applied"].as_array() {
        let body = usize::try_from(xfrc[0].as_u64().expect("body")).expect("body");
        let w = floats(&xfrc[1]);
        data.xfrc_applied[body] = sim_core::BodyWrench::new(
            nalgebra::Vector3::new(w[0], w[1], w[2]),
            nalgebra::Vector3::new(w[3], w[4], w[5]),
        );
    }
    data.forward(&model).expect("forward");
    (model, data)
}

/// A hinge held past its limit carrying a second hinge, and a box resting on
/// a plane under the pyramidal and the elliptic cone: at MuJoCo's forward
/// `qacc` and at it shifted, `inverse()` gives MuJoCo's constraint force,
/// row states and `qfrc_constraint` bit for bit, and `qfrc_inverse` within
/// 1e-12: the forward's `qfrc_bias` differs from MuJoCo's in 1 entry of the
/// hinges' and 3 of the box's (ledger L154). Before, `inverse()` kept the
/// forward solve's force: at the shifted `qacc` the limit's force was the
/// forward's 136.48 where MuJoCo's is remade from `qacc`.
#[test]
fn inverse_takes_the_constraint_force_from_qacc() {
    let g = golden();
    for name in ["limit", "contact", "elliptic"] {
        let case = &g["cases"][name];
        let (model, mut data) = at_state(case);
        assert_eq!(
            data.efc_type.len(),
            usize::try_from(case["nefc"].as_u64().expect("nefc")).expect("nefc"),
            "{name}: rows"
        );
        for at in ["at_forward_qacc", "at_shifted_qacc"] {
            let want = &case[at];
            data.qacc
                .as_mut_slice()
                .copy_from_slice(&floats(&want["qacc"]));
            data.inverse(&model);
            let what = |field: &str| format!("{name} {at}: {field}");
            assert_eq!(
                bits(data.efc_force.as_slice()),
                bits(&floats(&want["efc_force"])),
                "{} {:?}",
                what("efc_force"),
                data.efc_force.as_slice()
            );
            let states = mujoco_states(&data.efc_state);
            let want_states: Vec<i64> = want["efc_state"]
                .as_array()
                .expect("efc_state")
                .iter()
                .map(|x| x.as_i64().expect("int"))
                .collect();
            assert_eq!(states, want_states, "{}", what("efc_state"));
            assert_eq!(
                bits(data.qfrc_constraint.as_slice()),
                bits(&floats(&want["qfrc_constraint"])),
                "{} {:?}",
                what("qfrc_constraint"),
                data.qfrc_constraint.as_slice()
            );
            for (k, (a, b)) in data
                .qfrc_inverse
                .iter()
                .zip(floats(&want["qfrc_inverse"]))
                .enumerate()
            {
                assert!(
                    (a - b).abs() <= 1e-12,
                    "{}[{k}]: {a}, MuJoCo {b}",
                    what("qfrc_inverse")
                );
            }
        }
    }
}

/// `mjd_inverse_fd` (forward differences, eps 1e-6) against MuJoCo's
/// `mjd_inverseFD` on the same cases, at MuJoCo's forward `qacc`, within
/// 1e-9 relative to the largest entry of each matrix. MuJoCo writes them transposed. Before, with the
/// forward solve's constraint force kept, the limit's DfDa was 20 times
/// smaller (the constraint's `J'·D·J` missing), and DfDv and DfDq with it.
#[test]
fn inverse_fd_as_mujoco() {
    let g = golden();
    for name in ["limit", "contact", "elliptic"] {
        let case = &g["cases"][name];
        let (model, mut data) = at_state(case);
        data.qacc
            .as_mut_slice()
            .copy_from_slice(&floats(&case["forward_qacc"]));
        let config = DerivativeConfig {
            eps: 1e-6,
            centered: false,
            ..Default::default()
        };
        let ours = mjd_inverse_fd(&model, &data, &config).expect("mjd_inverse_fd");
        let nv = model.nv;
        for (field, m) in [
            ("DfDq", &ours.DfDq),
            ("DfDv", &ours.DfDv),
            ("DfDa", &ours.DfDa),
        ] {
            let want = floats(&case[field]);
            let scale = want.iter().fold(0.0_f64, |a, x| a.max(x.abs()));
            for i in 0..nv {
                for j in 0..nv {
                    let (a, b) = (m[(j, i)], want[i * nv + j]);
                    assert!(
                        (a - b).abs() <= 1e-9 * scale,
                        "{name} {field}[{i}][{j}]: {a}, MuJoCo {b}"
                    );
                }
            }
        }
    }
}

/// The limit and contact cases with the fwdinv flag (the contact one with a
/// wrench on the box), and a damped three-hinge chain with no rows: `solver_fwdinv`
/// within 1e-12 of MuJoCo's (exactly 0 with no rows, which MuJoCo returns
/// before any inverse), and `efc_state` the inverse's, as MuJoCo's
/// `mj_compareFwdInv` leaves it; the forward's `efc_force` and
/// `qfrc_constraint` are put back, bit for bit what the same forward gives
/// with the flag off. Before, `solver_fwdinv[0]` was always 0 and the
/// second statistic compared another sum, with no early return: on the
/// chain it read 5.8e-16 where MuJoCo returns 0.
#[test]
fn fwdinv_reports_and_puts_back_as_mujoco() {
    let g = golden();
    for name in ["limit_fwdinv", "contact_fwdinv", "free_fwdinv"] {
        let case = &g["cases"][name];
        let (model, data) = at_state(case);
        // With no rows MuJoCo returns both as 0 before any inverse.
        let tol = if data.efc_type.is_empty() { 0.0 } else { 1e-12 };
        for (k, (a, b)) in data
            .solver_fwdinv
            .iter()
            .zip(floats(&case["solver_fwdinv"]))
            .enumerate()
        {
            assert!(
                (a - b).abs() <= tol,
                "{name}: solver_fwdinv[{k}] {a}, MuJoCo {b}"
            );
        }
        let states = mujoco_states(&data.efc_state);
        let want_states: Vec<i64> = case["efc_state"]
            .as_array()
            .expect("efc_state")
            .iter()
            .map(|x| x.as_i64().expect("int"))
            .collect();
        assert_eq!(states, want_states, "{name}: efc_state");

        let mut off = model.clone();
        off.enableflags &= !sim_core::ENABLE_FWDINV;
        let mut plain = off.make_data();
        plain.qpos.copy_from(&data.qpos);
        plain.qvel.copy_from(&data.qvel);
        if !case["qacc_warmstart"].is_null() {
            plain
                .qacc_warmstart
                .as_mut_slice()
                .copy_from_slice(&floats(&case["qacc_warmstart"]));
            plain.time = case["time"].as_f64().expect("time");
        }
        plain.xfrc_applied.clone_from(&data.xfrc_applied);
        plain.forward(&off).expect("forward");
        assert_eq!(
            bits(data.efc_force.as_slice()),
            bits(plain.efc_force.as_slice()),
            "{name}: efc_force put back"
        );
        assert_eq!(
            bits(data.qfrc_constraint.as_slice()),
            bits(plain.qfrc_constraint.as_slice()),
            "{name}: qfrc_constraint put back"
        );
    }
}
