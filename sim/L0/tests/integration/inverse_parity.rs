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
use sim_core::{
    ConstraintState, Data, DerivativeConfig, Model, StepError, mjd_inverse_fd, mjd_transition_fd,
};
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
/// row states and `qfrc_constraint` bit for bit, `qfrc_inverse` within
/// 1e-12 (the forward's `qfrc_bias` differs from MuJoCo's in 1 entry of the
/// hinges' and 3 of the box's, ledger L154), and the body accumulators'
/// `cfrc_ext` within 1e-12·max(1, |MuJoCo's|), made from the inverse's force
/// (MuJoCo's `mj_rnePostConstraint` after `mj_inverse`). Before, `inverse()` kept the
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
            data.inverse(&model).expect("inverse");
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
            // The body accumulators come from the inverse's force.
            for b in 0..model.nbody {
                for (k, (x, y)) in data.cfrc_ext[b]
                    .iter()
                    .zip(floats(&want["cfrc_ext"][b]))
                    .enumerate()
                {
                    assert!(
                        (x - y).abs() <= 1e-12 * y.abs().max(1.0),
                        "{}[{b}][{k}]: {x}, MuJoCo {y}",
                        what("cfrc_ext")
                    );
                }
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
/// box with noslip, where its row states are not the forward's; the PGS box
/// under RK4; and a free box resting on a plane whose centre of mass is off
/// its body's origin, with a force on it, which the comparison's applied
/// forces take at the centre of mass (`xipos`), as MuJoCo's
/// `mj_xfrcAccumulate`. After
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
        "offset_fwdinv",
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

/// `values` against MuJoCo's `want` within `1e-12·max(1, |MuJoCo's|)`.
fn assert_rel(what: &str, values: &[f64], want: &[f64]) {
    assert_eq!(values.len(), want.len(), "{what}: length");
    for (k, (a, b)) in values.iter().zip(want).enumerate() {
        assert!(
            (a - b).abs() <= 1e-12 * b.abs().max(1.0),
            "{what}[{k}]: {a}, MuJoCo {b}"
        );
    }
}

/// `inverse()` runs MuJoCo's own position and velocity stages
/// (`mj_invPosition`, `mj_invVelocity`, `engine_inverse.c:38-72`) with
/// their sensors and energy, then the acceleration sensors: after a forward
/// pass at a state with no constraint row, `qpos` and `qvel` moved past the
/// arm's limit and `qacc` set, it makes the limit's row and gives MuJoCo's
/// `efc_force` and `qfrc_constraint` bit for bit, and its `qfrc_inverse`,
/// sensors (joint position, a site's position, joint velocity, a
/// velocimeter, an accelerometer, a force, a torque) and energy within
/// 1e-12·max(1, |MuJoCo's|). Before, it read the forward pass's stages: no
/// row, and the forward's sensors and energy.
#[test]
fn inverse_runs_its_own_stages_as_mujoco() {
    let case = &golden()["cases"]["stages"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos
        .as_mut_slice()
        .copy_from_slice(&floats(&case["forward_qpos"]));
    data.qvel
        .as_mut_slice()
        .copy_from_slice(&floats(&case["forward_qvel"]));
    data.forward(&model).expect("forward");
    assert!(data.efc_type.is_empty(), "rows at the forward's state");
    for (field, target) in [
        ("qpos", &mut data.qpos),
        ("qvel", &mut data.qvel),
        ("qacc", &mut data.qacc),
    ] {
        target.as_mut_slice().copy_from_slice(&floats(&case[field]));
    }
    data.inverse(&model).expect("inverse");
    assert_eq!(
        data.efc_type.len(),
        usize::try_from(case["nefc"].as_u64().expect("nefc")).expect("nefc"),
        "rows at the moved state"
    );
    assert_eq!(
        bits(data.efc_force.as_slice()),
        bits(&floats(&case["efc_force"])),
        "efc_force {:?}",
        data.efc_force.as_slice()
    );
    assert_eq!(
        bits(data.qfrc_constraint.as_slice()),
        bits(&floats(&case["qfrc_constraint"])),
        "qfrc_constraint {:?}",
        data.qfrc_constraint.as_slice()
    );
    assert_rel(
        "qfrc_inverse",
        data.qfrc_inverse.as_slice(),
        &floats(&case["qfrc_inverse"]),
    );
    assert_rel(
        "sensordata",
        data.sensordata.as_slice(),
        &floats(&case["sensordata"]),
    );
    assert_rel(
        "energy",
        &[data.energy_potential, data.energy_kinetic],
        &floats(&case["energy"]),
    );
    assert_eq!(
        data.nisland,
        usize::try_from(case["nisland"].as_u64().expect("nisland")).expect("nisland"),
        "nisland: the inverse makes no islands"
    );
}

/// With sleep enabled, the inverse's position stage wakes no tree a contact
/// touches (MuJoCo's `mj_invPosition` runs no `mj_wakeCollision`), and a
/// contact with a sleeping geom is refused, as MuJoCo's `mj_inverse` stops
/// ("mj_nc: contact … involves sleeping geom …", the golden): a box starting
/// asleep on a plane and a ball moved onto it give
/// `StepError::ContactWithSleepingGeom` naming the box's geom. A forward
/// pass at the same state wakes the box, as MuJoCo's, and the inverse then
/// gives MuJoCo's `qfrc_inverse` within 1e-9 (the forward's applied forces,
/// 0). Before, the inverse read the forward's stages and made no rows.
#[test]
fn inverse_refuses_a_contact_with_a_sleeping_tree_as_mujoco() {
    let case = &golden()["cases"]["wake"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let box_geom = usize::try_from(case["box_geom"].as_u64().expect("geom")).expect("geom");
    let at_state = || {
        let mut data = model.make_data();
        assert!(!data.tree_awake[0], "the box is awake after the reset");
        data.qpos
            .as_mut_slice()
            .copy_from_slice(&floats(&case["qpos"]));
        data
    };
    let mujoco = case["inverse_error"].as_str().expect("error");
    let mut data = at_state();
    match data.inverse(&model) {
        Err(StepError::ContactWithSleepingGeom { contact, geom }) => {
            assert_eq!(geom, box_geom, "MuJoCo's: {mujoco}");
            assert!(
                mujoco.ends_with(&format!("contact {contact} involves sleeping geom {geom}")),
                "MuJoCo's: {mujoco}"
            );
            assert_eq!(
                StepError::ContactWithSleepingGeom { contact, geom }.to_string(),
                format!("contact {contact} involves sleeping geom {geom}")
            );
        }
        other => panic!("inverse: {other:?}, MuJoCo's: {mujoco}"),
    }
    let mut data = at_state();
    data.forward(&model).expect("forward");
    assert!(
        data.tree_awake.iter().all(|&a| a),
        "the forward wakes the box"
    );
    data.inverse(&model).expect("inverse");
    for (k, (a, b)) in data
        .qfrc_inverse
        .iter()
        .zip(floats(&case["qfrc_inverse"]))
        .enumerate()
    {
        assert!((a - b).abs() <= 1e-9, "qfrc_inverse[{k}]: {a}, MuJoCo {b}");
    }
}

/// A contact MuJoCo excludes (`dist ≥ margin − gap`) is not refused: with
/// margin and gap 0.02 on the ball's geom and the ball 0.01 from the box
/// asleep, MuJoCo lists the contact and excludes it, its `mj_inverse` runs
/// and the box sleeps on (the golden); so here. (Its rows are P43's: until
/// then sim-core gives every listed contact rows.)
#[test]
fn inverse_does_not_refuse_an_excluded_contact_with_a_sleeping_tree() {
    let case = &golden()["cases"]["wake_gap"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qpos"]));
    data.inverse(&model).expect("inverse");
    assert!(data.ncon > 0, "no contact listed");
    assert!(!data.tree_awake[0], "the box woke");
}

/// The energy the inverse computes is not taken as the drift baseline:
/// `Data::energy_initial` is a forward pass's (registry
/// `D-ENERGY-INITIAL`). On the stages case, `inverse()` straight after
/// `make_data` leaves it 0, and the forward pass after it takes its own
/// total.
#[test]
fn inverse_takes_no_drift_baseline() {
    let case = &golden()["cases"]["stages"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qpos"]));
    data.inverse(&model).expect("inverse");
    assert!(
        data.energy_potential != 0.0,
        "the inverse computed no energy"
    );
    assert_eq!(data.energy_initial, 0.0, "taken by the inverse");
    data.forward(&model).expect("forward");
    assert_eq!(data.energy_initial, data.total_energy(), "the forward's");
}

/// At a sleeping dof, which the forward pass does not solve for, `inverse()`
/// gives the forces applied there, `qfrc_applied + qfrc_actuator +
/// Jᵀ·xfrc_applied` (0 here), call after call, within 1e-12, whatever
/// acceleration the dof keeps. MuJoCo's `mj_rne` writes the awake dofs only
/// (`engine_core_smooth.c:2481-2484`) and its sum adds to `qfrc_inverse` at
/// every dof (`engine_inverse.c:238-241`), so a sleeping dof's grows call
/// by call. Two states (each MuJoCo's: `qacc`
/// and `qfrc_bias` after the forward pass within 1e-12):
/// - every tree asleep after 3,000 steps from rest (a box resting on a plane
///   and a damped hanging arm): MuJoCo's value grows, −19.62, −39.24,
///   −58.86 on the box's vertical dof;
/// - a box asleep from the reset (`qacc` 0) beside a sphere resting on the
///   plane: MuJoCo's stays 0, as ours; the sum at the box's kept bias force
///   would read its weight, 19.62 (before, this inverse did). A dof force
///   and a wrench applied to it wake it in the inverse's kinematics, as
///   MuJoCo's `mj_wake`, and it takes MuJoCo's awake sum; from the position
///   stage (`inverse_skip`, which runs no kinematics) it sleeps on and its
///   dofs read them. With fwdinv
///   on, the sleeping box then adds nothing to `solver_fwdinv[1]`: over five
///   steps, the sphere's contact making rows from the second, both
///   statistics are MuJoCo's within 1e-12 (MuJoCo's at most 2.2e-14).
///
/// Registry `D-INVERSE-SLEEP`.
#[test]
fn inverse_at_a_sleeping_state_gives_the_applied_forces() {
    let case = &golden()["cases"]["sleep"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    for _ in 0..case["steps"].as_u64().expect("steps") {
        data.step(&model).expect("step");
    }
    assert!(data.tree_awake.iter().all(|&a| !a), "a tree is awake");
    data.forward(&model).expect("forward");
    for (field, values) in [("qacc", &data.qacc), ("qfrc_bias", &data.qfrc_bias)] {
        for (k, (a, b)) in values.iter().zip(floats(&case[field])).enumerate() {
            assert!((a - b).abs() <= 1e-12, "{field}[{k}]: {a}, MuJoCo {b}");
        }
    }
    let applied = &data.qfrc_applied + &data.qfrc_actuator;
    let mujoco: Vec<Vec<f64>> = case["qfrc_inverse"]
        .as_array()
        .expect("calls")
        .iter()
        .map(floats)
        .collect();
    assert_ne!(mujoco[0], mujoco[2], "MuJoCo's value does not grow");
    for (call, theirs) in mujoco.iter().enumerate() {
        data.inverse(&model).expect("inverse");
        for (k, (a, b)) in data.qfrc_inverse.iter().zip(applied.iter()).enumerate() {
            assert!(
                (a - b).abs() <= 1e-12,
                "call {call}: qfrc_inverse[{k}] {a}, applied {b} (MuJoCo's {})",
                theirs[k]
            );
        }
    }

    let case = &golden()["cases"]["sleep_init"];
    let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let box_tree = model.body_treeid[1];
    let mut data = model.make_data();
    assert!(
        !data.tree_awake[box_tree],
        "the box is awake after the reset"
    );
    data.forward(&model).expect("forward");
    for (field, values) in [("qacc", &data.qacc), ("qfrc_bias", &data.qfrc_bias)] {
        for (k, (a, b)) in values.iter().zip(floats(&case[field])).enumerate() {
            assert!(
                (a - b).abs() <= 1e-12,
                "init: {field}[{k}]: {a}, MuJoCo {b}"
            );
        }
    }
    let applied = &data.qfrc_applied + &data.qfrc_actuator;
    let box_dofs: Vec<usize> = case["box_dofs"]
        .as_array()
        .expect("box_dofs")
        .iter()
        .map(|k| usize::try_from(k.as_u64().expect("dof")).expect("dof"))
        .collect();
    for (call, theirs) in case["qfrc_inverse"]
        .as_array()
        .expect("calls")
        .iter()
        .enumerate()
    {
        data.inverse(&model).expect("inverse");
        for (k, (a, b)) in data.qfrc_inverse.iter().zip(floats(theirs)).enumerate() {
            assert!(
                (a - b).abs() <= 1e-12,
                "init call {call}: qfrc_inverse[{k}] {a}, MuJoCo {b}"
            );
        }
        for &k in &box_dofs {
            assert!(
                (data.qfrc_inverse[k] - applied[k]).abs() <= 1e-12,
                "init call {call}: the box's qfrc_inverse[{k}] {}, applied {}",
                data.qfrc_inverse[k],
                applied[k]
            );
        }
    }
    // Forces applied to the sleeping box after the forward pass: the
    // inverse's kinematics wake it, as MuJoCo's `mj_wake` wakes a tree that
    // cannot sleep, and its dofs take the awake sum, MuJoCo's within 1e-12.
    // From the position stage (`inverse_skip`, no kinematics, no wake) the
    // box sleeps on and its dofs read the applied forces: the dof force and
    // the wrench through its centre on its translational dofs.
    let applied_case = &case["applied"];
    let apply = |data: &mut Data| {
        let dof =
            usize::try_from(applied_case["qfrc_applied_dof"].as_u64().expect("dof")).expect("dof");
        data.qfrc_applied[dof] = applied_case["qfrc_applied"].as_f64().expect("force");
        let w = floats(&applied_case["xfrc_applied"]);
        data.xfrc_applied[1] = sim_core::BodyWrench::new(
            nalgebra::Vector3::new(w[0], w[1], w[2]),
            nalgebra::Vector3::new(w[3], w[4], w[5]),
        );
    };
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    apply(&mut data);
    data.inverse(&model).expect("inverse");
    assert!(
        data.tree_awake[box_tree],
        "the applied forces left the box asleep"
    );
    for (k, (a, b)) in data
        .qfrc_inverse
        .iter()
        .zip(floats(&applied_case["qfrc_inverse"]))
        .enumerate()
    {
        assert!(
            (a - b).abs() <= 1e-12,
            "applied: qfrc_inverse[{k}] {a}, MuJoCo {b}"
        );
    }
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    apply(&mut data);
    data.inverse_skip(&model, sim_core::MjStage::Pos, false)
        .expect("inverse_skip");
    assert!(!data.tree_awake[box_tree], "inverse_skip woke the box");
    let want = [1.0, -2.0, 8.0, 0.0, 0.0, 0.0];
    for (&k, want) in box_dofs.iter().zip(want) {
        assert!(
            (data.qfrc_inverse[k] - want).abs() <= 1e-12,
            "applied, skipped: the box's qfrc_inverse[{k}] {}, applied {want}",
            data.qfrc_inverse[k]
        );
    }

    model.enableflags |= sim_core::ENABLE_FWDINV;
    let mut data = model.make_data();
    for (k, theirs) in case["steps"].as_array().expect("steps").iter().enumerate() {
        data.step(&model).expect("step");
        assert!(!data.tree_awake[box_tree], "step {k}: the box woke");
        assert_eq!(
            data.efc_type.len(),
            usize::try_from(theirs["nefc"].as_u64().expect("nefc")).expect("nefc"),
            "step {k}: nefc"
        );
        for (i, (a, b)) in data
            .solver_fwdinv
            .iter()
            .zip(floats(&theirs["solver_fwdinv"]))
            .enumerate()
        {
            assert!(
                (a - b).abs() <= 1e-12,
                "step {k}: solver_fwdinv[{i}] {a}, MuJoCo {b}"
            );
        }
    }
}

/// `mjd_inverse_fd` at the state where every tree sleeps (above): each point
/// starts from the centre's whole state, its sleep state included, and a
/// sleeping dof's value is the forces applied there, so DfDa is 0 (no
/// `qacc` nudge wakes a tree), and the hinge arm's qpos column, whose nudge
/// wakes the arm alone, is 0 at the box's dofs. MuJoCo's `mjd_inverseFD` adds each point
/// to the sleeping dofs' `qfrc_inverse` (the golden's DfDa reads −1.962e7
/// at the box's vertical dof for the first input) and carries the box's
/// wake from its own columns into the arm's (−7.16e8 there). Registry
/// `D-INVERSE-SLEEP`.
#[test]
fn inverse_fd_at_a_sleeping_state_takes_each_point_from_the_centre() {
    let case = &golden()["cases"]["sleep"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    for _ in 0..case["steps"].as_u64().expect("steps") {
        data.step(&model).expect("step");
    }
    data.forward(&model).expect("forward");
    assert!(data.tree_awake.iter().all(|&a| !a), "a tree is awake");
    let config = DerivativeConfig {
        eps: 1e-6,
        centered: false,
        ..Default::default()
    };
    let fd = mjd_inverse_fd(&model, &data, &config).expect("mjd_inverse_fd");
    let nv = model.nv;
    assert!(fd.DfDa.iter().all(|&a| a == 0.0), "DfDa {}", fd.DfDa);
    let arm = nv - 1;
    for j in 0..arm {
        assert_eq!(fd.DfDq[(j, arm)], 0.0, "DfDq[{j}] of the arm's input");
    }
    let (mujoco_a, mujoco_q) = (floats(&case["DfDa"]), floats(&case["DfDq"]));
    assert!(mujoco_a[2].abs() > 1e6, "MuJoCo's DfDa {}", mujoco_a[2]);
    assert!(
        mujoco_q[arm * nv + 2].abs() > 1e6,
        "MuJoCo's DfDq {}",
        mujoco_q[arm * nv + 2]
    );
}

/// With `ENABLE_INVDISCRETE` the inverse takes `qacc` as the integrator's
/// discrete-time acceleration and turns it into the continuous-time one
/// first, as MuJoCo's `mj_discreteAcc` (`engine_inverse.c:76-153`): a chain
/// of three damped hinges on turned axes, one past its limit, a velocity
/// actuator and a damped fixed tendon, at the forward's `qacc` shifted,
/// under Euler, Euler with eulerdamp disabled (where the flag changes
/// nothing), Euler with the damper flag disabled (which `mj_discreteAcc`
/// does not read: its dofs' damping still counts), implicit and
/// implicitfast. `qfrc_inverse` and `efc_force` are
/// MuJoCo's within 1e-12·max(1, |MuJoCo's|) with the flag and without it,
/// and `qacc` is put back bit for bit. Before, the flag was not read
/// (ledger L155): the flagged `qfrc_inverse` was the unflagged one.
#[test]
fn inverse_takes_the_discrete_acceleration_as_mujoco() {
    let g = golden();
    for (name, case) in g["cases"]["discrete"].as_object().expect("cases") {
        if name == "rk4" {
            continue;
        }
        let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
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
        let qacc = floats(&case["qacc"]);
        for run in ["plain", "flagged"] {
            if run == "flagged" {
                model.enableflags |= sim_core::ENABLE_INVDISCRETE;
            }
            data.qacc.as_mut_slice().copy_from_slice(&qacc);
            data.inverse(&model).expect("inverse");
            assert_eq!(
                bits(data.qacc.as_slice()),
                bits(&qacc),
                "{name} {run}: qacc put back"
            );
            let want = &case[run];
            assert_rel(
                &format!("{name} {run} qfrc_inverse"),
                data.qfrc_inverse.as_slice(),
                &floats(&want["qfrc_inverse"]),
            );
            assert_rel(
                &format!("{name} {run} efc_force"),
                data.efc_force.as_slice(),
                &floats(&want["efc_force"]),
            );
        }
    }
}

/// The discrete inverse under RK4 is refused, as MuJoCo's `mj_discreteAcc`
/// stops ("discrete inverse dynamics is not supported by RK4 integrator",
/// both from `mj_inverse` and, with fwdinv on, from `mj_step`'s comparison:
/// the golden): `inverse()` returns
/// `StepError::UnsupportedInverseDiscrete`, and with fwdinv on `step()` and
/// `step2()` refuse it when the comparison reaches the inverse, at a state
/// whose forward pass makes a constraint row, before the advance. From rest,
/// where the chain's limit makes no row, MuJoCo's comparison returns first
/// and its step runs (`engine_inverse.c:267-270`): ours runs that step, to
/// MuJoCo's `qpos` within 1e-12·max(1, |MuJoCo's|), and refuses the next one,
/// where MuJoCo's stops. ImplicitSpringDamper, which MuJoCo does not have,
/// is refused the same way. Without fwdinv a step does not compare, and
/// runs.
#[test]
fn the_discrete_inverse_refuses_rk4_as_mujoco() {
    let g = golden();
    let case = &g["cases"]["discrete"]["rk4"];
    let errors = &case["errors"];
    for what in ["inverse", "step"] {
        assert!(
            errors[what]
                .as_str()
                .expect("error")
                .ends_with("discrete inverse dynamics is not supported by RK4 integrator"),
            "MuJoCo's {what}: {}",
            errors[what]
        );
    }
    let from_rest = &case["from_rest"];
    let mujoco_ran = from_rest["qpos_after"].as_array().expect("qpos_after");
    assert_eq!(
        from_rest["stopped_at"].as_u64(),
        Some(1),
        "MuJoCo's first step from rest runs and its second stops"
    );
    let row_state = &g["cases"]["discrete"]["euler"];
    let rk4 = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut isd = rk4.clone();
    isd.integrator = sim_core::Integrator::ImplicitSpringDamper;
    for base in [rk4, isd] {
        let integrator = base.integrator;
        let refused = Err(StepError::UnsupportedInverseDiscrete { integrator });
        let mut model = base.clone();
        model.enableflags |= sim_core::ENABLE_INVDISCRETE;
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        assert_eq!(data.inverse(&model), refused, "{integrator:?}: inverse");

        model.enableflags |= sim_core::ENABLE_FWDINV;
        let mut data = model.make_data();
        data.step(&model)
            .unwrap_or_else(|e| panic!("{integrator:?}: the step from rest, with no row: {e}"));
        if integrator == sim_core::Integrator::RungeKutta4 {
            assert_rel(
                "rk4 qpos after the step from rest",
                data.qpos.as_slice(),
                &floats(&mujoco_ran[0]),
            );
            let (time, qpos) = (data.time, data.qpos.clone());
            assert_eq!(
                data.step(&model),
                refused,
                "rk4: the second step, as MuJoCo's"
            );
            assert_eq!(
                (data.time.to_bits(), bits(data.qpos.as_slice())),
                (time.to_bits(), bits(qpos.as_slice())),
                "rk4: refused before the advance"
            );
        }

        set_state(row_state, &mut data);
        let (time, qpos) = (data.time, data.qpos.clone());
        assert_eq!(data.step(&model), refused, "{integrator:?}: step at a row");
        assert_eq!(
            data.step2(&model),
            refused,
            "{integrator:?}: step2 at a row"
        );
        assert_eq!(
            (data.time.to_bits(), bits(data.qpos.as_slice())),
            (time.to_bits(), bits(qpos.as_slice())),
            "{integrator:?}: refused before the advance"
        );
        model.enableflags &= !sim_core::ENABLE_FWDINV;
        data.step(&model)
            .unwrap_or_else(|e| panic!("{integrator:?}: a step without fwdinv: {e}"));
        assert_eq!(
            StepError::UnsupportedInverseDiscrete { integrator }.to_string(),
            format!("discrete inverse dynamics is not supported by {integrator:?} integrator")
        );
    }
}

/// `mjd_inverse_fd` refuses the discrete inverse where `inverse()` does,
/// before any work: under ImplicitSpringDamper with `ENABLE_INVDISCRETE`
/// (before, its points took the continuous-time inverse, the flag dropped);
/// under RK4 its own refusal of the integrator comes first, as MuJoCo's
/// `mjd_inverseFD` checks the integrator before its inverse passes reach
/// `mj_discreteAcc`. Without the flag it runs.
#[test]
fn the_inverse_finite_differences_refuse_the_discrete_inverse_as_inverse_does() {
    let case = &golden()["cases"]["discrete"]["rk4"];
    let rk4 = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut isd = rk4.clone();
    isd.integrator = sim_core::Integrator::ImplicitSpringDamper;
    let config = DerivativeConfig {
        eps: 1e-6,
        centered: false,
        ..Default::default()
    };
    for (base, want) in [
        (
            rk4,
            StepError::UnsupportedIntegrator {
                integrator: sim_core::Integrator::RungeKutta4,
            },
        ),
        (
            isd,
            StepError::UnsupportedInverseDiscrete {
                integrator: sim_core::Integrator::ImplicitSpringDamper,
            },
        ),
    ] {
        let mut model = base.clone();
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        if model.integrator == sim_core::Integrator::ImplicitSpringDamper {
            mjd_inverse_fd(&model, &data, &config).expect("without the flag");
        }
        model.enableflags |= sim_core::ENABLE_INVDISCRETE;
        match mjd_inverse_fd(&model, &data, &config) {
            Err(e) => assert_eq!(e, want, "{:?}", model.integrator),
            Ok(_) => panic!("{:?}: mjd_inverse_fd ran with the flag", model.integrator),
        }
    }
}

/// With fwdinv and invdiscrete under implicit and implicitfast, the
/// transition finite differences are those without the flags: the
/// comparison's discrete inverse writes `M − h·qDeriv` unfactored into the
/// integrator's matrix, and the comparison puts the forward pass's factor
/// back before the control columns' steps solve with it (they skip the
/// factor, as MuJoCo's `mj_stepSkip` from the velocity stage). A and B
/// (pure finite differences, eps 1e-6, forward) are bit for bit those with
/// no flag, on the discrete chain at its row state. MuJoCo's comparison
/// leaves the matrix written: its B moves by more than 1 with the two flags
/// (the golden: about 8e3 under implicit, 7e3 under implicitfast). Registry
/// `D-FWDINV-FACTOR`.
#[test]
fn the_fwdinv_comparison_leaves_the_transition_derivatives_as_they_are() {
    let g = golden();
    for name in ["implicit", "implicitfast"] {
        let case = &g["cases"]["discrete"][name];
        let mujoco = &case["transition_B"];
        let moved = floats(&mujoco["both"])
            .iter()
            .zip(floats(&mujoco["none"]))
            .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs()));
        assert!(moved > 1.0, "{name}: MuJoCo's B moved by {moved}");
        let config = DerivativeConfig {
            eps: 1e-6,
            centered: false,
            use_analytical: false,
            ..Default::default()
        };
        let mut matrices = Vec::new();
        for flags in [0, sim_core::ENABLE_FWDINV | sim_core::ENABLE_INVDISCRETE] {
            let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
            model.enableflags |= flags;
            let mut data = model.make_data();
            set_state(case, &mut data);
            data.ctrl
                .as_mut_slice()
                .copy_from_slice(&floats(&case["ctrl"]));
            data.forward(&model).expect("forward");
            let t = mjd_transition_fd(&model, &data, &config).expect("mjd_transition_fd");
            matrices.push((bits(t.A.as_slice()), bits(t.B.as_slice()), t.B.clone()));
        }
        assert_eq!(matrices[0].0, matrices[1].0, "{name}: A with both flags");
        assert_eq!(
            matrices[0].1, matrices[1].1,
            "{name}: B with both flags {}, without {}",
            matrices[1].2, matrices[0].2
        );
    }
}
