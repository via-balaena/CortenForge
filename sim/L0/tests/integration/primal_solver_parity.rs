//! Newton and CG against MuJoCo 3.5.0's primal solver (`mj_solPrimal`,
//! `engine_solver.c:1811-1968`, from `warmstart`, `engine_forward.c:611-687`):
//! with islands disabled, every row at once; with islands (the `islands_*`
//! cases), each island apart (`engine_forward.c:762-797`).
//!
//! The golden is `assets/golden/primal_solver/primal_solver.json`, from the
//! unfused MuJoCo 3.5.0 oracle (`scripts/gen_primal_solver_reference.py`, which
//! describes its cases). Given MuJoCo's own inputs the solver's outputs are
//! compared bit for bit: it uses `+ − × ÷` and `sqrt` only. Through the whole
//! pipeline they are compared at a tolerance: the stages before the solver
//! round otherwise than MuJoCo's in places, and use libm.

use serde_json::Value;
use sim_core::{ConstraintState, Data, Model, SolverStat};
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!(
        "../assets/golden/primal_solver/primal_solver.json"
    ))
    .expect("primal solver golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn usizes(v: &Value) -> Vec<usize> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| usize::try_from(x.as_u64().expect("count")).expect("fits"))
        .collect()
}

fn count(v: &Value) -> usize {
    usize::try_from(v.as_u64().expect("count")).expect("fits")
}

/// The case's model and its data at the case's start (qpos0, the case's qvel).
fn start(case: &Value) -> (Model, Data) {
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qvel.copy_from_slice(&floats(&case["qvel"]));
    (model, data)
}

fn assert_close(what: &str, got: &[f64], want: &[f64], tol: f64) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    for (i, (a, b)) in got.iter().zip(want).enumerate() {
        assert!((a - b).abs() <= tol, "{what}[{i}]: {a:e}, MuJoCo {b:e}");
    }
}

fn assert_bits(what: &str, got: &[f64], want: &[f64]) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    for (i, (a, b)) in got.iter().zip(want).enumerate() {
        assert_eq!(a.to_bits(), b.to_bits(), "{what}[{i}]: {a:e}, MuJoCo {b:e}");
    }
}

/// The counts of each iteration's statistics, and the iteration count (with
/// islands, MuJoCo's per island, summed).
fn assert_stat_counts(what: &str, got: &[SolverStat], want: &Value) {
    let niter = match want["niter"].as_array() {
        Some(per_island) => per_island.iter().map(count).sum(),
        None => count(&want["niter"]),
    };
    assert_eq!(got.len(), niter, "{what}: iterations");
    let field = |name: &str, f: fn(&SolverStat) -> usize| {
        let ours: Vec<usize> = got.iter().map(f).collect();
        assert_eq!(ours, usizes(&want[name]), "{what}: {name}");
    };
    field("nactive", |s| s.nactive);
    field("nchange", |s| s.nchange);
    field("neval", |s| s.neval);
    field("nupdate", |s| s.nupdate);
}

/// Steps the case to its last checkpoint: each step's iteration count is
/// MuJoCo's, and qpos and qvel at each checkpoint up to `through` are within
/// `tol` of MuJoCo's.
fn assert_trajectory(what: &str, case: &Value, through: usize, tol: f64) {
    step_and_compare(what, case, through, tol, usize::MAX);
}

/// [`assert_trajectory`], the iteration counts compared through step
/// `counts_through`; stepping stops after the later of the two.
fn step_and_compare(what: &str, case: &Value, through: usize, tol: f64, counts_through: usize) {
    let (model, mut data) = start(case);
    let niter = usizes(&case["niter"]);
    let last = through.max(counts_through);
    for (k, &want) in niter.iter().enumerate().take(last) {
        let step = k + 1;
        data.step(&model).expect("step");
        if step <= counts_through {
            assert_eq!(data.solver_niter, want, "{what}: iterations at step {step}");
        }
        if step <= through
            && let Some(at) = case["traj"].get(step.to_string())
        {
            assert_close(
                &format!("{what} qpos at {step}"),
                data.qpos.as_slice(),
                &floats(&at["qpos"]),
                tol,
            );
            assert_close(
                &format!("{what} qvel at {step}"),
                data.qvel.as_slice(),
                &floats(&at["qvel"]),
                tol,
            );
        }
    }
}

/// Given MuJoCo's inputs (the mass matrix and its factor, the unconstrained
/// acceleration and force, the rows' Jacobian, reference acceleration, D and
/// R), the solver gives MuJoCo's acceleration, forces and statistics bit for
/// bit, Newton and CG, on equality, friction-loss, limit and contact rows,
/// pyramidal and elliptic (condim 3 and 6).
#[test]
fn primal_solver_matches_mujoco_3_5_0_bit_for_bit_given_its_inputs() {
    let g = golden();
    for (name, case) in g["cases"]["same_input"].as_object().expect("cases") {
        solve_given_mujocos_inputs(name, case);
    }
}

/// Hands the solver the case's MuJoCo inputs after a forward pass and
/// compares its statistics, acceleration and forces with MuJoCo's, bit for
/// bit; returns the data.
fn solve_given_mujocos_inputs(name: &str, case: &Value) -> Data {
    let (model, mut data) = start(case);
    data.forward(&model).expect("forward");
    assert_eq!(
        model.M_rowadr,
        usizes(&case["M_rowadr"]),
        "{name}: M_rowadr"
    );
    assert_eq!(
        model.M_rownnz,
        usizes(&case["M_rownnz"]),
        "{name}: M_rownnz"
    );
    assert_eq!(
        model.M_colind,
        usizes(&case["M_colind"]),
        "{name}: M_colind"
    );
    let nefc = data.efc_type.len();
    assert_eq!(nefc, floats(&case["efc_D"]).len(), "{name}: nefc");

    let m = floats(&case["M"]);
    for i in 0..model.nv {
        let row = model.M_rowadr[i]..model.M_rowadr[i] + model.M_rownnz[i];
        for (&j, &value) in model.M_colind[row.clone()].iter().zip(&m[row]) {
            data.qM[(i, j)] = value;
            data.qM[(j, i)] = value;
        }
    }
    data.qLD_data.copy_from_slice(&floats(&case["qLD"]));
    data.qLD_diag_inv
        .copy_from_slice(&floats(&case["qLDiagInv"]));
    data.qacc_smooth
        .copy_from_slice(&floats(&case["qacc_smooth"]));
    data.qfrc_smooth
        .copy_from_slice(&floats(&case["qfrc_smooth"]));
    let j = floats(&case["efc_J"]);
    for r in 0..nefc {
        for c in 0..model.nv {
            data.efc_J[(r, c)] = j[r * model.nv + c];
        }
    }
    data.efc_aref.copy_from_slice(&floats(&case["efc_aref"]));
    data.efc_D.copy_from_slice(&floats(&case["efc_D"]));
    data.efc_R.copy_from_slice(&floats(&case["efc_R"]));
    data.qacc_warmstart.fill(0.0);

    let newton = case["newton"].as_bool().expect("newton");
    sim_core::constraint::mj_sol_primal_rows(&model, &mut data, newton).expect("solve");

    let stats = &case["stats"];
    assert_stat_counts(name, &data.solver_stat, stats);
    let field = |f: fn(&SolverStat) -> f64| data.solver_stat.iter().map(f).collect::<Vec<_>>();
    assert_bits(
        &format!("{name} improvement"),
        &field(|s| s.improvement),
        &floats(&stats["improvement"]),
    );
    assert_bits(
        &format!("{name} gradient"),
        &field(|s| s.gradient),
        &floats(&stats["gradient"]),
    );
    assert_bits(
        &format!("{name} lineslope"),
        &field(|s| s.lineslope),
        &floats(&stats["lineslope"]),
    );
    assert_bits(
        &format!("{name} qacc"),
        data.qacc.as_slice(),
        &floats(&case["qacc"]),
    );
    assert_bits(
        &format!("{name} efc_force"),
        data.efc_force.as_slice(),
        &floats(&case["efc_force"]),
    );
    data
}

/// Census doc 406a1991, a box on a tilted plane, at the census start: three
/// Newton iterations with MuJoCo's statistics, and MuJoCo's acceleration.
/// The parent's Newton re-added a row it meant to remove from the Hessian's
/// factor, took 100 iterations and then PGS.
#[test]
fn newton_matches_mujoco_3_5_0_iterates() {
    let case = &golden()["cases"]["newton_box"];
    let (model, mut data) = start(case);
    assert_eq!(
        model.stat_meaninertia.to_bits(),
        case["meaninertia"].as_f64().expect("meaninertia").to_bits(),
        "meaninertia"
    );
    data.forward(&model).expect("forward");
    let stats = &case["stats"];
    assert_stat_counts("newton_box", &data.solver_stat, stats);
    for (k, (s, want)) in data
        .solver_stat
        .iter()
        .zip(floats(&stats["improvement"]))
        .enumerate()
    {
        assert!(
            (s.improvement - want).abs() <= 1e-9 * want.abs(),
            "improvement at {k}: {:e}, MuJoCo {want:e}",
            s.improvement
        );
    }
    assert_close(
        "newton_box qacc",
        data.qacc.as_slice(),
        &floats(&case["qacc"]),
        1e-12,
    );
    assert_close(
        "newton_box efc_force",
        data.efc_force.as_slice(),
        &floats(&case["efc_force"]),
        1e-10,
    );
}

/// The same box with one iteration allowed: Newton keeps its first iterate,
/// as MuJoCo's does; it does not hand the rows to PGS.
#[test]
fn newton_has_no_pgs_fallback() {
    let g = golden();
    let case = &g["cases"]["newton_box"];
    let one = &case["one_iteration"];
    let model = load_model(one["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qvel.copy_from_slice(&floats(&case["qvel"]));
    data.forward(&model).expect("forward");
    assert_eq!(data.solver_niter, count(&one["niter"]), "iterations");
    assert_close(
        "first iterate",
        data.qacc.as_slice(),
        &floats(&one["qacc"]),
        1e-12,
    );
}

/// A hinge arm and a free box joined by a connect, under CG: 500 steps, each
/// with MuJoCo's iteration count, the state within 1e-11 of MuJoCo's. (With
/// MuJoCo's inputs the solver is bit for bit; with ours, 1e-17 apart in
/// `efc_R` and `qacc_smooth`, the first forces are 2.5e-11 apart.)
#[test]
fn cg_matches_mujoco_3_5_0() {
    let case = &golden()["cases"]["cg_connect"];
    assert_trajectory("cg_connect", case, 500, 1e-11);
}

/// Census doc 443824ba, a sphere on a plane under CG with an elliptic cone:
/// 100 steps, each with MuJoCo's iteration count, the state within 1e-12.
/// The parent's CG ran PGS when it stopped short of its tolerance and made
/// the acceleration again from the forces.
#[test]
fn cg_keeps_its_iterate() {
    let case = &golden()["cases"]["cg_sphere"];
    assert_trajectory("cg_sphere", case, 100, 1e-12);
}

/// Census doc 28695e44, two stacked boxes: a step without rows reports no
/// iterations and no statistics, as MuJoCo clears its count
/// (`mj_fwdConstraint`, `engine_forward.c:747-752`); the parent kept the
/// previous step's.
#[test]
fn solver_niter_cleared_without_rows() {
    let case = &golden()["cases"]["no_rows"];
    let (model, mut data) = start(case);
    let (nefc, niter) = (usizes(&case["nefc"]), usizes(&case["niter"]));
    assert!(
        nefc.windows(2).any(|w| w[0] > 0 && w[1] == 0),
        "the fixture has a step without rows after one with rows"
    );
    for (k, (&want_nefc, &want_niter)) in nefc.iter().zip(&niter).enumerate() {
        data.step(&model).expect("step");
        assert_eq!(data.efc_type.len(), want_nefc, "nefc at step {}", k + 1);
        assert_eq!(
            data.solver_niter,
            want_niter,
            "iterations at step {}",
            k + 1
        );
        if want_nefc == 0 {
            assert!(data.solver_stat.is_empty(), "statistics at step {}", k + 1);
        }
    }
}

/// Ledger L81's matched fixture: a tilted box sliding and spinning on a plane,
/// every bottom corner below it, one step under CG and Newton held to 1 and 3
/// iterations: the box's velocity is MuJoCo's. The parent's unconverged solves
/// went to PGS (CG 1: 0.9145 where MuJoCo gives 0.9357).
#[test]
fn few_iteration_steps_match_mujoco_3_5_0() {
    let g = golden();
    for (name, case) in g["cases"]["few_iterations"].as_object().expect("cases") {
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qvel.copy_from_slice(&floats(&case["qvel0"]));
        data.step(&model).expect("step");
        assert_eq!(data.ncon, count(&case["ncon"]), "{name}: contacts");
        assert_eq!(
            data.solver_niter,
            count(&case["niter"]),
            "{name}: iterations"
        );
        assert_close(name, data.qvel.as_slice(), &floats(&case["qvel"]), 1e-15);
    }
}

/// Ledger L71's fixture: L81's box beside a damped arm that starts asleep,
/// held to few iterations. Euler's damped solve takes only the awake dofs'
/// damping, as MuJoCo's (`mj_EulerSkip`, `engine_forward.c:948-962`), so it
/// does not run, and the box keeps the solver's acceleration. Read every
/// dof's damping and the damped solve runs, from the forces, which the
/// solver's iterate does not satisfy exactly: the box's x velocity goes from
/// MuJoCo's 0.936 to −0.053 at CG 1 iteration (measured).
#[test]
fn euler_damping_reads_the_awake_dofs_as_mujoco_3_5_0() {
    let g = golden();
    for (name, case) in g["cases"]["eulerdamp_asleep"].as_object().expect("cases") {
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qvel.copy_from_slice(&floats(&case["qvel0"]));
        data.step(&model).expect("step");
        assert_eq!(
            data.solver_niter,
            count(&case["niter"]),
            "{name}: iterations"
        );
        assert_close(name, data.qvel.as_slice(), &floats(&case["qvel"]), 1e-15);
    }
}

/// An arm on a limited hinge with friction loss, a ball connected to its tip,
/// a sphere sliding and spinning on a plane: every kind of row. Newton and
/// CG, pyramidal and elliptic (condim 3): the first forward's iteration,
/// evaluation and update counts and acceleration are MuJoCo's, each step's
/// iteration count is MuJoCo's through step 140 and the state through step
/// 50. (Past step 140 the elliptic CG run grows as MuJoCo's own run does under
/// a 1-ulp change of a starting velocity: both reach 1e-6 by step 200.) At
/// condim 6 the first forward: the torsional and rolling rows' R differ from
/// MuJoCo's after it (book P38).
#[test]
fn every_row_kind_steps_as_mujoco_3_5_0() {
    let g = golden();
    for (name, case) in g["cases"]["mixed_rows"].as_object().expect("cases") {
        let (model, mut data) = start(case);
        data.forward(&model).expect("forward");
        let first = &case["forward"];
        assert_eq!(data.efc_type.len(), count(&first["nefc"]), "{name}: nefc");
        assert_stat_counts(name, &data.solver_stat, &first["stats"]);
        assert_close(
            &format!("{name} qacc"),
            data.qacc.as_slice(),
            &floats(&first["qacc"]),
            1e-12,
        );
        if !name.ends_with("_6") {
            step_and_compare(name, case, 50, 1e-12, 140);
        }
    }
}

/// Ten spheres sliding and spinning on a plane under Newton with an elliptic
/// cone: 60 dofs, MuJoCo's sparse size. Its Hessian factor, with the cones'
/// contributions, is made anew at each iteration where MuJoCo updates its
/// own (the same matrix, rounded otherwise): 50 steps, each with MuJoCo's
/// iteration count, the state within 1e-12. Then two of the spheres in
/// contact with each other, so an elliptic contact couples two trees and its
/// cone term pairs columns of different rows: the first forward's iteration,
/// evaluation and update counts are MuJoCo's, and 100 steps keep its
/// iteration count and its state within 1e-12.
#[test]
fn sparse_newton_steps_as_mujoco_3_5_0() {
    let g = golden();
    let case = &g["cases"]["sparse_spheres"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    assert_eq!(model.nv, 60, "MuJoCo's sparse size");
    assert_trajectory("sparse_spheres", case, 50, 1e-12);
    for name in ["sparse_pair", "sparse_stack"] {
        let case = &g["cases"][name];
        let (model, mut data) = start(case);
        assert_eq!(model.nv, 60, "{name}: MuJoCo's sparse size");
        data.forward(&model).expect("forward");
        let first = &case["forward"];
        assert_eq!(data.efc_type.len(), count(&first["nefc"]), "{name}: nefc");
        assert_stat_counts(name, &data.solver_stat, &first["stats"]);
        assert_trajectory(name, case, 100, 1e-12);
    }
}

/// A pendulum swinging free beside a sphere sliding and spinning on a plane,
/// noslip on: MuJoCo makes the islands (the sphere's; the pendulum is in
/// none) but solves every row at once, from a warmstart that starts the
/// pendulum's dof at its unconstrained acceleration (`warmstart`,
/// `engine_forward.c:670-676`). Newton and CG, 100 steps within 1e-12 of
/// MuJoCo's. Started at its warmstart value instead, CG's pendulum dof takes
/// part in the line search: 2.5e-7 off (measured). Each step's iteration
/// count, noslip's sweeps included, is MuJoCo's.
#[test]
fn warmstart_starts_the_dofs_outside_the_islands_unconstrained() {
    let g = golden();
    for (solver, case) in g["cases"]["warmstart_islands"].as_object().expect("cases") {
        assert_trajectory(solver, case, 100, 1e-12);
    }
}

/// MuJoCo's `mjtConstraintState` value of a row's state.
fn mj_state(state: ConstraintState) -> usize {
    match state {
        ConstraintState::Satisfied => 0,
        ConstraintState::Quadratic => 1,
        ConstraintState::LinearNeg => 2,
        ConstraintState::LinearPos => 3,
        ConstraintState::Cone => 4,
    }
}

/// Noslip counts its sweeps in `solver_niter`, records their statistics after
/// the solver's and leaves each row's state as MuJoCo's `dualState` gives it
/// (ledger L129): an equality, a friction-loss row and a contact, pyramidal
/// and elliptic, noslip on, Newton and CG; and a box resting on a plane,
/// elliptic, whose contacts' friction blocks couple their two tangents, so a
/// sweep's cost change is MuJoCo's `costChange` over the whole block. The
/// first forward's iteration, active-row and change counts and the rows'
/// states are MuJoCo's, each statistic's improvement within 1e-12 of the
/// case's largest (2.8e-16 measured), and each step's iteration count
/// through the steps recorded (200, or 50 for the one-iteration cases). The
/// elliptic contacts hold inside their cones: sliding, noslip leaves a
/// contact on the cone's edge, where the zone turns on the last bit, and the
/// forces reaching noslip already differ from MuJoCo's by a few ulps
/// (measured).
#[test]
fn noslip_counts_and_records_its_sweeps_as_mujoco_3_5_0() {
    let g = golden();
    let mut off = Vec::new();
    for (name, case) in g["cases"]["noslip_rows"].as_object().expect("cases") {
        let (model, mut data) = start(case);
        data.forward(&model).expect("forward");
        let first = &case["forward"];
        let stats = &first["stats"];
        let field = |f: fn(&SolverStat) -> usize| -> Vec<usize> {
            data.solver_stat.iter().map(f).collect()
        };
        let checks = [
            (
                "nefc",
                vec![data.efc_type.len()],
                vec![count(&first["nefc"])],
            ),
            (
                "niter",
                vec![data.solver_niter],
                vec![count(&stats["niter"])],
            ),
            ("nactive", field(|s| s.nactive), usizes(&stats["nactive"])),
            ("nchange", field(|s| s.nchange), usizes(&stats["nchange"])),
            ("neval", field(|s| s.neval), usizes(&stats["neval"])),
            ("nupdate", field(|s| s.nupdate), usizes(&stats["nupdate"])),
            (
                "efc_state",
                data.efc_state.iter().map(|&s| mj_state(s)).collect(),
                usizes(&first["efc_state"]),
            ),
        ];
        for (what, ours, mujoco) in checks {
            if ours != mujoco {
                off.push(format!("{name} {what}: {ours:?}, MuJoCo {mujoco:?}"));
            }
        }
        let want = floats(&stats["improvement"]);
        let scale = want.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
        for (k, (s, w)) in data.solver_stat.iter().zip(&want).enumerate() {
            if (s.improvement - w).abs() > 1e-12 * scale {
                off.push(format!(
                    "{name} improvement {k}: {:e}, MuJoCo {w:e}",
                    s.improvement
                ));
            }
        }
        let (model, mut data) = start(case);
        for (k, want) in usizes(&case["niter"]).into_iter().enumerate() {
            data.step(&model).expect("step");
            if data.solver_niter != want {
                let got = data.solver_niter;
                off.push(format!(
                    "{name} step {}: {got} iterations, MuJoCo {want}",
                    k + 1
                ));
                break;
            }
        }
    }
    assert!(off.is_empty(), "{off:#?}");
}

/// `<option jacobian>` picks the solver's path as MuJoCo's `mj_isSparse`
/// (ledger L130): ten spheres at 60 dofs under `jacobian="dense"` take the
/// dense Hessian, and their first step is MuJoCo's bit for bit (by the dof
/// count alone, the sparse path, 5.6e-17 off: measured); 50 steps then keep
/// MuJoCo's iteration counts within 1e-12. mixed_rows at 13 dofs under
/// `jacobian="sparse"` steps 200 with MuJoCo's iteration counts within 1e-12
/// (2.3e-15 measured, either path).
#[test]
fn the_jacobian_option_picks_the_solvers_path() {
    let g = golden();
    let cases = &g["cases"]["jacobian_option"];
    let dense = &cases["dense_large"];
    let (model, mut data) = start(dense);
    assert_eq!(model.nv, 60, "the automatic sparse size");
    assert!(!model.is_sparse(), "jacobian dense");
    data.step(&model).expect("step");
    let first = &dense["traj"]["1"];
    assert_bits(
        "dense_large qpos at 1",
        data.qpos.as_slice(),
        &floats(&first["qpos"]),
    );
    assert_bits(
        "dense_large qvel at 1",
        data.qvel.as_slice(),
        &floats(&first["qvel"]),
    );
    assert_trajectory("dense_large", dense, 50, 1e-12);
    let sparse = &cases["sparse_small"];
    let (model, _) = start(sparse);
    assert!(model.is_sparse(), "jacobian sparse at {} dofs", model.nv);
    assert_trajectory("sparse_small", sparse, 200, 1e-12);
}

/// [`step_and_compare`] with each step's island count MuJoCo's as well.
fn step_islands_and_compare(
    what: &str,
    case: &Value,
    through: usize,
    tol: f64,
    counts_through: usize,
) {
    let (model, mut data) = start(case);
    let nisland = usizes(&case["nisland"]);
    for (k, &want) in nisland.iter().enumerate().take(counts_through) {
        data.step(&model).expect("step");
        assert_eq!(data.nisland, want, "{what}: islands at step {}", k + 1);
    }
    step_and_compare(what, case, through, tol, counts_through);
}

/// mixed_rows' arm and ball, and its puck, with islands enabled (MuJoCo's
/// default): two islands, each solved apart over its own dofs and rows,
/// scaled by its own inertia (`mj_fwdConstraint`, `engine_forward.c:762-797`;
/// book P35). Newton and CG, pyramidal and elliptic (condim 3 and 6): the
/// first forward's island count, each island's iteration, active-row,
/// change, evaluation and update counts in island order and the
/// acceleration are MuJoCo's; each step's island count and iteration count
/// are MuJoCo's, and the state within 1e-12 through step 50.
#[test]
fn islands_iterate_as_mujoco() {
    let g = golden();
    for (name, case) in g["cases"]["islands_mixed"].as_object().expect("cases") {
        let (model, mut data) = start(case);
        data.forward(&model).expect("forward");
        let first = &case["forward"];
        assert_eq!(data.nisland, count(&first["nisland"]), "{name}: islands");
        assert_eq!(data.efc_type.len(), count(&first["nefc"]), "{name}: nefc");
        assert_stat_counts(name, &data.solver_stat, &first["stats"]);
        assert_close(
            &format!("{name} qacc"),
            data.qacc.as_slice(),
            &floats(&first["qacc"]),
            1e-12,
        );
        if !name.ends_with("_6") {
            step_islands_and_compare(name, case, 50, 1e-12, 200);
        }
    }
}

/// [`primal_solver_matches_mujoco_3_5_0_bit_for_bit_given_its_inputs`] with
/// islands: each mixed_rows variant's two islands solved apart from MuJoCo's
/// inputs give MuJoCo's statistics, in island order, its acceleration and
/// its forces bit for bit.
#[test]
fn islands_match_mujoco_3_5_0_bit_for_bit_given_their_inputs() {
    let g = golden();
    for (name, case) in g["cases"]["same_input_islands"].as_object().expect("cases") {
        let data = solve_given_mujocos_inputs(name, case);
        assert_eq!(data.nisland, count(&case["nisland"]), "{name}: islands");
    }
}

/// sparse_spheres, sparse_pair and sparse_stack with islands enabled: at
/// MuJoCo's sparse size each island's Hessian is factored over its own
/// dofs. The first forward's counts are MuJoCo's, and each step's island
/// and iteration counts and the state within 1e-12 through the case.
#[test]
fn sparse_islands_step_as_mujoco() {
    let g = golden();
    for (name, case) in g["cases"]["islands_sparse"].as_object().expect("cases") {
        let (model, mut data) = start(case);
        assert!(model.is_sparse(), "{name}: MuJoCo's sparse size");
        data.forward(&model).expect("forward");
        let first = &case["forward"];
        assert_eq!(data.nisland, count(&first["nisland"]), "{name}: islands");
        assert_stat_counts(name, &data.solver_stat, &first["stats"]);
        let steps = usizes(&case["niter"]).len();
        step_islands_and_compare(name, case, steps, 1e-12, steps);
    }
}

/// 22 spheres on vertical slides falling onto a plane, each its own island:
/// MuJoCo counts the iterations and keeps the statistics of its first 20
/// islands (`mjNISLAND`, `engine_solver.c:1946-1951`) and solves all 22. The
/// first forward's counts and each step's are MuJoCo's (20), and the state
/// of every slider within 1e-12 through 50 steps.
#[test]
fn islands_past_the_twentieth_are_solved_but_not_counted() {
    let g = golden();
    for (solver, case) in g["cases"]["islands_many"].as_object().expect("cases") {
        let (model, mut data) = start(case);
        data.forward(&model).expect("forward");
        let first = &case["forward"];
        assert_eq!(data.nisland, 22, "{solver}: islands");
        assert_eq!(count(&first["nisland"]), 22, "{solver}: MuJoCo's islands");
        assert_stat_counts(solver, &data.solver_stat, &first["stats"]);
        step_islands_and_compare(solver, case, 50, 1e-12, 50);
    }
}

/// Not matched (registry `D-ISLAND-EFC-STATE`, wrong-value): MuJoCo copies
/// each island's forces back but not its rows' states
/// (`engine_forward.c:794-797`), so with islands its `efc_state` is the
/// warmstart's, made at `qacc_smooth` (`:631-660`). An arm whose hinge
/// friction loss (5) holds it against gravity, one island, Newton and CG:
/// MuJoCo's force is −1.32435, inside the friction band, and its state with
/// islands is LinearPos, the state of a row at −5; without islands it is
/// Quadratic. Ours is Quadratic either way, with MuJoCo's force.
#[test]
fn island_rows_keep_their_solves_states() {
    let g = golden();
    for (solver, cases) in g["cases"]["islands_friction_hold"]
        .as_object()
        .expect("cases")
    {
        for (tag, case) in cases.as_object().expect("cases") {
            let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
            let mut data = model.make_data();
            data.forward(&model).expect("forward");
            let what = format!("{solver} {tag}");
            assert_eq!(data.nisland, count(&case["nisland"]), "{what}: islands");
            assert_close(
                &format!("{what} efc_force"),
                data.efc_force.as_slice(),
                &floats(&case["efc_force"]),
                1e-12,
            );
            let floss = model.dof_frictionloss[0];
            assert!(data.efc_force[0].abs() < floss, "{what}: inside the band");
            assert_eq!(data.efc_state, [ConstraintState::Quadratic], "{what}");
            let mujoco = usizes(&case["efc_state"]);
            let want = if tag == "islands" {
                ConstraintState::LinearPos
            } else {
                ConstraintState::Quadratic
            };
            assert_eq!(mujoco, [mj_state(want)], "{what}: MuJoCo's state");
        }
    }
}

/// At jacobian="sparse", a hinge with friction loss beside a second hinge
/// under a fixed tendon of coef 0 with friction loss and a range its length
/// exceeds: the tendon's two rows are all zero. A row in no island takes the
/// force and state at its residual `efc_b`, with the warmstart and without:
/// every row's force and the acceleration are MuJoCo's, and every row's
/// state is MuJoCo's `efc_state` (with the warmstart) or its island solve's
/// (without, where MuJoCo writes no `efc_state`). Not matched (ledger L143):
/// MuJoCo's partition reads a row's dof chain (`treeNext`,
/// `engine_island.c:148-162`), so the tendon's rows make the second hinge an
/// island (2 islands; with the warmstart 2 iterations a step, without it the
/// first island's 1 and the second's 0 or 1); sim-core's reads the nonzero
/// entries, so they are in none (1 island, 1 iteration).
#[test]
fn a_row_in_no_island_takes_its_force_at_its_residual() {
    let g = golden();
    for (solver, case) in g["cases"]["islands_zero_row"].as_object().expect("cases") {
        let (model, mut data) = start(case);
        assert!(model.is_sparse(), "{solver}: the sparse size");
        data.forward(&model).expect("forward");
        assert_eq!(data.efc_type.len(), count(&case["nefc"]), "{solver}: nefc");
        assert_eq!(
            (data.nisland, count(&case["nisland"])),
            (1, 2),
            "{solver}: islands, ours and MuJoCo's"
        );
        assert_bits(
            &format!("{solver} efc_force"),
            data.efc_force.as_slice(),
            &floats(&case["efc_force"]),
        );
        let states: Vec<usize> = data.efc_state.iter().map(|&s| mj_state(s)).collect();
        let want = case.get("efc_state").unwrap_or(&case["solve_state"]);
        assert_eq!(states, usizes(want), "{solver}: efc_state");
        assert_close(
            &format!("{solver} qacc"),
            data.qacc.as_slice(),
            &floats(&case["qacc"]),
            1e-12,
        );
        for (k, &want) in usizes(&case["niter"]).iter().enumerate() {
            data.step(&model).expect("step");
            match case.get("island_niter") {
                // Without the warmstart, MuJoCo's second island (the tendon's
                // rows) takes 0 or 1 iterations a step: compare its first.
                Some(per_island) => assert_eq!(
                    data.solver_niter,
                    count(&per_island[k][0]),
                    "{solver}: iterations at step {}, MuJoCo's first island's",
                    k + 1
                ),
                None => assert_eq!(
                    (data.solver_niter, want),
                    (1, 2),
                    "{solver}: iterations at step {}, ours and MuJoCo's",
                    k + 1
                ),
            }
        }
    }
}
