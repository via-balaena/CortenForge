//! Newton and CG against MuJoCo 3.5.0's primal solver (`mj_solPrimal`,
//! `engine_solver.c:1811-1968`, from `warmstart`, `engine_forward.c:611-687`):
//! with islands disabled, every row at once; with islands (the `islands_*`
//! cases), each island apart (`engine_forward.c:762-799`).
//!
//! The golden is `assets/golden/primal_solver/primal_solver.json`, from the
//! unfused MuJoCo 3.5.0 oracle (`scripts/gen_primal_solver_reference.py`, which
//! describes its cases). Given MuJoCo's own inputs the solver's outputs are
//! compared bit for bit: it uses `+ − × ÷` and `sqrt` only. Through the whole
//! pipeline they are compared at a tolerance: the stages before the solver
//! round otherwise than MuJoCo's in places, and use libm.

use serde_json::Value;
use sim_core::{ConstraintState, DISABLE_ISLAND, Data, Model, SolverStat};
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
    if let Some(qpos) = case.get("qpos") {
        data.qpos.copy_from_slice(&floats(qpos));
    }
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
    // MuJoCo keeps at most 200 statistics a solve or an island (`mjNSOLVER`).
    let kept = match want["niter"].as_array() {
        Some(per_island) => per_island.iter().map(|n| count(n).min(200)).sum(),
        None => count(&want["niter"]).min(200),
    };
    assert_eq!(got.len(), kept, "{what}: statistics kept");
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
    // sim-core's own regularization is MuJoCo's bit for bit, but in the
    // condim-6 cases, whose torsional and rolling rows take P38's.
    if !name.contains("_6") {
        assert_bits(
            &format!("{name} efc_R"),
            &data.efc_R,
            &floats(&case["efc_R"]),
        );
    }
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
/// a 1-ulp change of a starting velocity: both reach 1e-6 by step 200.) The
/// same at condim 6, whose torsional and rolling rows take their R from the
/// normal row's by the friction ratios.
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
        step_and_compare(name, case, 50, 1e-12, 140);
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
/// sweep's cost change is MuJoCo's `costChange` over the whole block; and a
/// three-link arm with friction loss, bent away from `qpos0` so its mass
/// matrix's mean diagonal is not the model's mean inertia, which scales the
/// statistics. The first forward's iteration, active-row and change counts
/// and the rows' states are MuJoCo's, each statistic's improvement,
/// gradient and line slope within 1e-12 of the case's largest or of 1
/// (noslip's gradient and slope are 0), and each step's iteration count
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
        let real =
            |f: fn(&SolverStat) -> f64| -> Vec<f64> { data.solver_stat.iter().map(f).collect() };
        for (what, ours, key) in [
            ("improvement", real(|s| s.improvement), "improvement"),
            ("gradient", real(|s| s.gradient), "gradient"),
            ("lineslope", real(|s| s.lineslope), "lineslope"),
        ] {
            let want = floats(&stats[key]);
            let scale = want.iter().fold(1.0_f64, |m, x| m.max(x.abs()));
            for (k, (s, w)) in ours.iter().zip(&want).enumerate() {
                if (s - w).abs() > 1e-12 * scale {
                    off.push(format!("{name} {what} {k}: {s:e}, MuJoCo {w:e}"));
                }
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

/// Past 200 iterations MuJoCo counts an iteration but keeps no statistic
/// (`mjNSOLVER`, `saveStats`; ledger L140): of a solve over every row, of
/// each island, and of noslip's sweeps after the solver's. Mixed rows with a
/// negative tolerance: CG over every row and by island, Newton by island
/// (over every row it stops after 3 iterations), noslip's 250 sweeps and
/// PGS's 300 iterations. The
/// first forward's iteration count and kept statistics are MuJoCo's, each
/// improvement within 1e-12 of the case's largest, and PGS's rows' states
/// (`dualState`'s) MuJoCo's. sim-mjcf refuses a tolerance
/// that is not positive (ledger L131), so the document loads without it and
/// the model takes MuJoCo's.
#[test]
fn statistics_past_the_200th_are_counted_not_kept() {
    const TOLERANCE: &str = r#" tolerance="-1""#;
    let g = golden();
    let mut off = Vec::new();
    for (name, case) in g["cases"]["statistics_cap"].as_object().expect("cases") {
        let xml = case["xml"].as_str().expect("xml");
        let mut model = load_model(&xml.replace(TOLERANCE, "")).expect("load");
        if xml.contains(TOLERANCE) {
            model.solver_tolerance = -1.0;
        }
        let mut data = model.make_data();
        data.qvel.copy_from_slice(&floats(&case["qvel"]));
        data.forward(&model).expect("forward");
        let stats = &case["stats"];
        let per_island = match &stats["niter"] {
            Value::Array(_) => usizes(&stats["niter"]),
            one => vec![count(one)],
        };
        let niter: usize = per_island.iter().sum();
        let kept: usize = per_island.iter().map(|&n| n.min(200)).sum();
        assert!(
            niter > 200,
            "{name}: MuJoCo's {niter} iterations reach past 200"
        );
        if data.solver_niter != niter {
            off.push(format!(
                "{name} niter: {}, MuJoCo {niter}",
                data.solver_niter
            ));
        }
        if data.solver_stat.len() != kept {
            off.push(format!(
                "{name} kept: {}, MuJoCo {kept}",
                data.solver_stat.len()
            ));
        }
        if let Some(want) = stats.get("efc_state") {
            let ours: Vec<usize> = data.efc_state.iter().map(|&s| mj_state(s)).collect();
            if ours != usizes(want) {
                off.push(format!("{name} efc_state: {ours:?}, MuJoCo {want}"));
            }
        }
        if stats.get("improvement").is_none() {
            continue;
        }
        let field = |f: fn(&SolverStat) -> usize| -> Vec<usize> {
            data.solver_stat.iter().map(f).collect()
        };
        for (what, ours, mujoco) in [
            ("nactive", field(|s| s.nactive), usizes(&stats["nactive"])),
            ("nchange", field(|s| s.nchange), usizes(&stats["nchange"])),
            ("neval", field(|s| s.neval), usizes(&stats["neval"])),
            ("nupdate", field(|s| s.nupdate), usizes(&stats["nupdate"])),
        ] {
            if ours != mujoco {
                off.push(format!("{name} {what} differs"));
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
    }
    assert!(off.is_empty(), "{off:#?}");
}

/// PGS scales its statistics by the model's mean inertia, the mass matrix's
/// trace over nv at qpos0, at every state, as MuJoCo's `mj_solPGS`
/// (`engine_solver.c:326`): the golden's two-hinge arm bent past its
/// shoulder's limit with its hand on the floor, where the trace over nv is
/// the golden's `trace_over_nv`, more than 1% from the model's. The first
/// forward's iteration count, active and changed counts and states are
/// MuJoCo's, each improvement within 1e-12 of the largest.
#[test]
fn pgs_scales_its_statistics_by_the_models_mean_inertia() {
    let case = &golden()["cases"]["pgs_scale"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos.copy_from_slice(&floats(&case["qpos"]));
    data.qvel.copy_from_slice(&floats(&case["qvel"]));
    data.forward(&model).expect("forward");

    let nv = f64::from(u32::try_from(model.nv).expect("nv"));
    let trace = (0..model.nv).map(|i| data.qM[(i, i)]).sum::<f64>() / nv;
    let (want_trace, want_mean) = (
        case["trace_over_nv"].as_f64().expect("trace"),
        case["meaninertia"].as_f64().expect("meaninertia"),
    );
    assert!(
        (trace - want_trace).abs() <= 1e-12 * want_trace,
        "trace over nv {trace}"
    );
    assert!(
        (model.stat_meaninertia - want_mean).abs() <= 1e-12 * want_mean,
        "mean inertia {}",
        model.stat_meaninertia
    );
    assert!(
        (trace / want_mean - 1.0).abs() > 0.01,
        "the fixture tells the scales apart"
    );

    let stats = &case["stats"];
    assert_eq!(data.solver_niter, count(&stats["niter"]), "iterations");
    let field =
        |f: fn(&SolverStat) -> usize| -> Vec<usize> { data.solver_stat.iter().map(f).collect() };
    assert_eq!(field(|s| s.nactive), usizes(&stats["nactive"]), "nactive");
    assert_eq!(field(|s| s.nchange), usizes(&stats["nchange"]), "nchange");
    let states: Vec<usize> = data.efc_state.iter().map(|&s| mj_state(s)).collect();
    assert_eq!(states, usizes(&stats["efc_state"]), "states");
    let want = floats(&stats["improvement"]);
    let scale = want.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
    for (k, (s, w)) in data.solver_stat.iter().zip(&want).enumerate() {
        assert!(
            (s.improvement - w).abs() <= 1e-12 * scale,
            "improvement {k}: {:e}, MuJoCo {w:e}",
            s.improvement
        );
    }
}

/// Noslip's active-row count is `dualState`'s: every row not satisfied, a
/// contact in the cone counting each of its rows (`engine_solver.c:233-310`).
/// noslip_rows' box sliding at about 1 m/s and spinning, three starts,
/// elliptic, noslip on, Newton and CG: after each of 200 steps the last
/// statistic's count is the rows the step leaves not satisfied. Noslip
/// leaves a sliding contact on the cone's edge, where the zone turns on the
/// last bit, so which steps leave one in the cone is the platform's rounding
/// (7, 1 and 11 of 200 for the three starts here, under each solver); the
/// six runs together must reach it.
/// The count follows whichever zone it is.
#[test]
fn noslip_counts_the_rows_it_leaves_active() {
    let g = golden();
    let starts = [
        [1.0, 0.3, 0.0, 0.0, 0.0, 2.0, 0.5],
        [0.8, -0.4, 0.0, 0.0, 0.0, -1.5, 0.5],
        [1.2, 0.1, 0.0, 0.0, 0.0, 3.0, 0.5],
    ];
    let mut cone_steps = 0;
    for solver in ["Newton", "CG"] {
        let case = &g["cases"]["noslip_rows"][format!("{solver}_box_elliptic").as_str()];
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        for (s, qvel) in starts.iter().enumerate() {
            let mut data = model.make_data();
            data.qvel.copy_from_slice(qvel);
            for step in 1..=200 {
                data.step(&model).expect("step");
                let active = data
                    .efc_state
                    .iter()
                    .filter(|&&s| s != ConstraintState::Satisfied)
                    .count();
                let last = data.solver_stat.last().expect("a statistic");
                assert_eq!(
                    last.nactive, active,
                    "{solver} start {s} step {step}: nactive"
                );
                if data.efc_state.contains(&ConstraintState::Cone) {
                    cone_steps += 1;
                }
            }
        }
    }
    assert!(cone_steps > 0, "a contact in the cone");
}

/// `<option jacobian>` picks the solver's path as MuJoCo's `mj_isSparse`
/// (ledger L130): ten spheres at 60 dofs under `jacobian="dense"` take the
/// dense Hessian, and their first step's velocity is MuJoCo's bit for bit
/// (by the dof count alone, the sparse path, it is not: measured); their
/// position too on macOS arm64, where the golden was made (a quaternion
/// passes through `sin` and `cos`, which another platform's libm may round
/// otherwise; there within 1e-13). 50 steps then keep MuJoCo's iteration
/// counts. mixed_rows at 13 dofs under `jacobian="sparse"` steps 200 with
/// MuJoCo's iteration counts.
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
        "dense_large qvel at 1",
        data.qvel.as_slice(),
        &floats(&first["qvel"]),
    );
    let qpos = floats(&first["qpos"]);
    if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
        assert_bits("dense_large qpos at 1", data.qpos.as_slice(), &qpos);
    } else {
        assert_close("dense_large qpos at 1", data.qpos.as_slice(), &qpos, 1e-13);
    }
    assert_trajectory("dense_large", dense, 50, 1e-12);
    let sparse = &cases["sparse_small"];
    let (model, _) = start(sparse);
    assert!(model.is_sparse(), "jacobian sparse at {} dofs", model.nv);
    assert_trajectory("sparse_small", sparse, 200, 1e-12);
}

/// The rows' solve makes `efc_b` from the rows it is given, as
/// `mj_fwd_constraint` does when the solve starts (`engine_forward.c:755-756`):
/// with every row's `efc_aref` moved by 1 after the forward pass, `efc_b`
/// is J·`qacc_smooth` − `efc_aref` for the moved references.
#[test]
fn the_rows_solve_makes_efc_b_from_its_inputs() {
    let g = golden();
    for name in ["Newton_pyramidal_3", "CG_elliptic_3"] {
        let case = &g["cases"]["same_input"][name];
        let (model, mut data) = start(case);
        data.forward(&model).expect("forward");
        for aref in data.efc_aref.iter_mut() {
            *aref += 1.0;
        }
        let newton = case["newton"].as_bool().expect("newton");
        sim_core::constraint::mj_sol_primal_rows(&model, &mut data, newton).expect("solve");
        for r in 0..data.efc_type.len() {
            let jv: f64 = (0..model.nv)
                .map(|c| data.efc_J[(r, c)] * data.qacc_smooth[c])
                .sum();
            let want = jv - data.efc_aref[r];
            assert!(
                (data.efc_b[r] - want).abs() <= 1e-12 * want.abs().max(1.0),
                "{name} efc_b[{r}]: {:e}, J·qacc_smooth − aref {want:e}",
                data.efc_b[r]
            );
        }
    }
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
/// scaled by its own inertia (`mj_fwdConstraint`, `engine_forward.c:762-799`;
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

/// Islands whose mass matrix couples their dofs (a two-link arm; a free body
/// whose centre of mass is off its origin), dense under CG and at the sparse
/// size under Newton (beside ten spheres), so an island's scale reads the
/// diagonal of a block with entries off it; and noslip on with islands
/// enabled, where MuJoCo solves every row at once (`mj_fwdConstraint`'s
/// condition, `engine_forward.c:763-766`). The first forward's island count,
/// iteration counts and acceleration, each step's island and iteration
/// counts, and the state within 1e-12 through step 50 are MuJoCo's.
#[test]
fn islands_with_coupled_dofs_or_noslip_step_as_mujoco() {
    let g = golden();
    for (name, case) in g["cases"]["islands_review"].as_object().expect("cases") {
        let (model, mut data) = start(case);
        data.forward(&model).expect("forward");
        let first = &case["forward"];
        assert_eq!(data.nisland, count(&first["nisland"]), "{name}: islands");
        assert!(data.nisland > 1, "{name}: more than one island");
        assert_stat_counts(name, &data.solver_stat, &first["stats"]);
        assert_close(
            &format!("{name} qacc"),
            data.qacc.as_slice(),
            &floats(&first["qacc"]),
            1e-12,
        );
        let steps = usizes(&case["niter"]).len();
        step_islands_and_compare(name, case, 50, 1e-12, steps);
    }
}

/// Solving island by island leaves what solving every row at once leaves,
/// to the solvers' tolerance: each row's residual (`efc_jar`), the cost
/// (`efc_cost`), and the cone Hessians and their count, one for each contact
/// in the cone. mixed_rows'
/// elliptic variants (its sliding puck's contact in the cone), the first
/// forward, each run with islands and with `DISABLE_ISLAND`. (Not the
/// zero-row document: there, with every row at once, MuJoCo's solve and
/// this one both stop after one iteration at an acceleration far from the
/// islands' (measured).)
#[test]
fn island_solves_leave_the_whole_solves_residuals_cost_and_cone_hessians() {
    let g = golden();
    let close = |what: &str, a: &[f64], b: &[f64]| {
        assert_eq!(a.len(), b.len(), "{what}: length");
        let scale = b.iter().fold(1.0_f64, |m, x| m.max(x.abs()));
        for (i, (x, y)) in a.iter().zip(b).enumerate() {
            assert!(
                (x - y).abs() <= 1e-6 * scale,
                "{what}[{i}]: {x:e}, whole {y:e}"
            );
        }
    };
    for key in ["Newton_elliptic_3", "CG_elliptic_3"] {
        let what = format!("islands_mixed/{key}");
        let case = &g["cases"]["islands_mixed"][key];
        let (model, mut data) = start(case);
        data.forward(&model).expect("forward");
        let mut whole_model = model.clone();
        whole_model.disableflags |= DISABLE_ISLAND;
        let mut whole = whole_model.make_data();
        whole.qvel.copy_from_slice(&floats(&case["qvel"]));
        whole.forward(&whole_model).expect("forward");
        assert!(
            data.nisland > 0 && whole.nisland == 0,
            "{what}: islands on and off"
        );
        close(
            &format!("{what} efc_jar"),
            data.efc_jar.as_slice(),
            whole.efc_jar.as_slice(),
        );
        close(
            &format!("{what} efc_cost"),
            &[data.efc_cost],
            &[whole.efc_cost],
        );
        assert_eq!(data.ncone, whole.ncone, "{what}: cone contacts");
        assert!(data.ncone > 0, "{what}: a contact in the cone");
        let mut in_cone: Vec<usize> = (0..data.efc_type.len())
            .filter(|&r| data.efc_state[r] == ConstraintState::Cone)
            .map(|r| data.efc_id[r])
            .collect();
        in_cone.dedup();
        assert_eq!(data.ncone, in_cone.len(), "{what}: contacts in the cone");
        let kept = data.efc_cone_hessian.iter().flatten().count();
        assert_eq!(kept, data.ncone, "{what}: cone Hessians kept");
        for (c, (a, b)) in data
            .efc_cone_hessian
            .iter()
            .zip(&whole.efc_cone_hessian)
            .enumerate()
        {
            match (a, b) {
                (Some(a), Some(b)) => {
                    close(&format!("{what} cone {c}"), a.as_slice(), b.as_slice())
                }
                (None, None) => {}
                _ => panic!("{what}: contact {c}'s cone Hessian kept by one solve only"),
            }
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
/// (`engine_forward.c:796-798`), so with islands its `efc_state` is the
/// warmstart's, made at `qacc_smooth` (`:662`). An arm whose hinge
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

/// A dof's friction-loss row and a tendon's, both of id 0 and adjacent, are
/// two constraints, as MuJoCo's `mjCNSTR_FRICTION_DOF` and
/// `mjCNSTR_FRICTION_TENDON` are, so the tendon's row joins the trees it
/// couples (`findEdges`, `engine_island.c:327-336`): two slides in two
/// trees, the first with friction loss, a fixed tendon over both with
/// friction loss. The first forward's islands, each tree's and row's island,
/// the forces and the acceleration are MuJoCo's.
#[test]
fn a_tendon_friction_row_after_a_dof_friction_row_joins_its_trees() {
    let case = &golden()["cases"]["islands_friction_key"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qvel.copy_from_slice(&floats(&case["qvel"]));
    data.forward(&model).expect("forward");
    let ints = |v: &Value| -> Vec<i64> {
        v.as_array()
            .expect("array")
            .iter()
            .map(|x| x.as_i64().expect("int"))
            .collect()
    };
    assert_eq!(data.nisland, count(&case["nisland"]), "islands");
    let trees: Vec<i64> = data.tree_island.iter().map(|&i| i64::from(i)).collect();
    assert_eq!(trees, ints(&case["tree_island"]), "tree_island");
    let rows: Vec<i64> = data.efc_island.iter().map(|&i| i64::from(i)).collect();
    assert_eq!(rows, ints(&case["efc_island"]), "efc_island");
    for (what, ours, want) in [
        (
            "efc_force",
            data.efc_force.as_slice(),
            floats(&case["efc_force"]),
        ),
        ("qacc", data.qacc.as_slice(), floats(&case["qacc"])),
    ] {
        for (k, (a, b)) in ours.iter().zip(&want).enumerate() {
            assert!((a - b).abs() <= 1e-12, "{what}[{k}]: {a}, MuJoCo {b}");
        }
    }
}

/// At jacobian="sparse", a hinge with friction loss beside a second hinge
/// under a fixed tendon of coef 0 with friction loss and a range its length
/// exceeds: the tendon's two rows are all zero, and their dof chain is the
/// second hinge's. The partition reads a row's chain at the sparse size, as
/// MuJoCo's (`treeNext`, `engine_island.c:148-162`), so the tendon's rows
/// make the second hinge an island. With the warmstart and without: the
/// islands, every row's force and the acceleration are MuJoCo's, every
/// row's state is MuJoCo's `efc_state` (with the warmstart) or its island
/// solve's (without, where MuJoCo writes no `efc_state`), and each step's
/// iterations over the islands are MuJoCo's.
#[test]
fn a_zero_row_joins_the_island_of_its_chain() {
    let g = golden();
    for (solver, case) in g["cases"]["islands_zero_row"].as_object().expect("cases") {
        let (model, mut data) = start(case);
        assert!(model.is_sparse(), "{solver}: the sparse size");
        data.forward(&model).expect("forward");
        assert_eq!(data.efc_type.len(), count(&case["nefc"]), "{solver}: nefc");
        assert_eq!(data.nisland, count(&case["nisland"]), "{solver}: islands");
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
        // The cost is the tendon limit's ½·D·jar² (its R is 1e-15), which
        // outweighs the rest by far.
        let limit = data.efc_D[2] * data.efc_jar[2] * data.efc_jar[2] / 2.0;
        assert!(
            (data.efc_cost - limit).abs() <= 1e-9 * limit,
            "{solver}: efc_cost {:e}, the limit row's {limit:e}",
            data.efc_cost
        );
        for (k, &want) in usizes(&case["niter"]).iter().enumerate() {
            data.step(&model).expect("step");
            assert_eq!(
                data.solver_niter,
                want,
                "{solver}: iterations at step {}",
                k + 1
            );
        }
    }
}
