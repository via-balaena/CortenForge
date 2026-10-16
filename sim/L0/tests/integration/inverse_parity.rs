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
    set_state(case, &mut data);
    data.forward(&model).expect("forward");
    (model, data)
}

/// Writes the case's qpos, qvel, warmstart and time (where the case gives
/// them) and its wrench (where it gives one) into `data`.
fn set_state(case: &Value, data: &mut Data) {
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

/// The fwdinv flag's comparison runs in a step, after its forward pass and
/// `mj_check_acc`, as MuJoCo's `mj_step` (`engine_forward.c:1431-1434`),
/// and not in a forward pass alone. The cases: the limit and contact cases
/// (the contact one with a wrench on the box); a damped three-hinge chain
/// with no rows; the box under PGS, where the inverse's constraint force is
/// not the forward's; the limit case with friction loss and the elliptic
/// box with noslip, where its row states are not the forward's; and the
/// PGS box under RK4. After
/// `forward()`, `solver_fwdinv` is MuJoCo's after `mj_forward`, still 0,
/// and `efc_state` a forward's with the flag off; after `step()`,
/// `solver_fwdinv` is MuJoCo's within 1e-12
/// of the larger of 1 and its size (exactly 0 with no rows, which MuJoCo
/// returns before any inverse) and `efc_state` MuJoCo's, the inverse's
/// (under RK4 the later stages remake the rows, so it is a flag-off step's,
/// as in MuJoCo; this PGS's states are not MuJoCo's); and the step's
/// `qpos`, `qvel`, `efc_force` and `qfrc_constraint` are bit for bit a
/// step's with the flag off (the forward's force put back). `step1()` then
/// `step2()` gives the same `solver_fwdinv` and `efc_state` as `step()`
/// (MuJoCo's `mj_step2` compares too, `:1500-1503`; under RK4 it takes
/// Euler's step, so that case is left out).
#[test]
fn fwdinv_compares_in_the_step_as_mujoco() {
    let g = golden();
    let states = |v: &Value| -> Vec<i64> {
        v.as_array()
            .expect("efc_state")
            .iter()
            .map(|x| x.as_i64().expect("int"))
            .collect()
    };
    for name in [
        "limit_fwdinv",
        "contact_fwdinv",
        "free_fwdinv",
        "pgs_fwdinv",
        "floss_fwdinv",
        "noslip_fwdinv",
        "rk4_fwdinv",
    ] {
        let case = &g["cases"][name];
        let (model, forwarded) = at_state(case);
        let mut off = model.clone();
        off.enableflags &= !sim_core::ENABLE_FWDINV;
        let mut plain = off.make_data();
        set_state(case, &mut plain);
        plain.forward(&off).expect("forward");
        assert_eq!(
            forwarded.solver_fwdinv.to_vec(),
            floats(&case["after_forward"]["solver_fwdinv"]),
            "{name}: solver_fwdinv after forward()"
        );
        assert_eq!(
            forwarded.efc_state, plain.efc_state,
            "{name}: efc_state after forward(), as with the flag off"
        );

        let mut stepped = model.make_data();
        set_state(case, &mut stepped);
        stepped.step(&model).expect("step");
        let want = floats(&case["after_step"]["solver_fwdinv"]);
        for (k, (&a, &b)) in stepped.solver_fwdinv.iter().zip(&want).enumerate() {
            let tol = if forwarded.efc_type.is_empty() {
                0.0
            } else {
                1e-12 * b.abs().max(1.0)
            };
            assert!(
                (a - b).abs() <= tol,
                "{name}: solver_fwdinv[{k}] {a}, MuJoCo {b}"
            );
        }
        let mut plain = off.make_data();
        set_state(case, &mut plain);
        plain.step(&off).expect("step");
        if model.integrator == sim_core::Integrator::RungeKutta4 {
            assert_eq!(
                stepped.efc_state, plain.efc_state,
                "{name}: efc_state after step(), as with the flag off"
            );
        } else {
            assert_eq!(
                mujoco_states(&stepped.efc_state),
                states(&case["after_step"]["efc_state"]),
                "{name}: efc_state after step()"
            );
            let mut split = model.make_data();
            set_state(case, &mut split);
            split.step1(&model).expect("step1");
            split.step2(&model).expect("step2");
            assert_eq!(
                bits(&split.solver_fwdinv),
                bits(&stepped.solver_fwdinv),
                "{name}: solver_fwdinv after step2()"
            );
            assert_eq!(
                split.efc_state, stepped.efc_state,
                "{name}: efc_state after step2()"
            );
        }
        for (field, a, b) in [
            ("qpos", &stepped.qpos, &plain.qpos),
            ("qvel", &stepped.qvel, &plain.qvel),
            ("efc_force", &stepped.efc_force, &plain.efc_force),
            (
                "qfrc_constraint",
                &stepped.qfrc_constraint,
                &plain.qfrc_constraint,
            ),
        ] {
            assert_eq!(
                bits(a.as_slice()),
                bits(b.as_slice()),
                "{name}: {field} as with the flag off"
            );
        }
    }
}
