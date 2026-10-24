//! The dual solvers against MuJoCo 3.5.0's: the rows' inverse inertia
//! `efc_AR` (`mj_projectConstraint`, `engine_core_constraint.c:2140-2371`),
//! PGS from its warmstart (`mj_solPGS`, `engine_solver.c:316-531`;
//! `warmstart`, `engine_forward.c:611-687`) and noslip (`mj_solNoSlip`,
//! `engine_solver.c:537-747`).
//!
//! The golden is `assets/golden/dual_solver/dual_solver.json`, from the
//! unfused MuJoCo 3.5.0 oracle (`scripts/gen_dual_solver_reference.py`,
//! which describes its cases). Given MuJoCo's own inputs (the mass matrix
//! and its factor, the rows' Jacobian, reference accelerations,
//! regularization, each contact's regularized cone μ, and the warmstart),
//! `efc_AR` and every output of the solve are compared bit for bit: they use
//! `+ − × ÷` and `sqrt` only.

use serde_json::Value;
use sim_core::{ConstraintState, ConstraintType, Data, Integrator, MjStage, Model, SolverStat};
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!(
        "../assets/golden/dual_solver/dual_solver.json"
    ))
    .expect("dual solver golden parses")
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

/// Where `got` differs from `want` in any bit, each as a line in `off`.
fn bits(off: &mut Vec<String>, what: &str, got: &[f64], want: &[f64]) {
    if got.len() != want.len() {
        off.push(format!(
            "{what}: {} values, MuJoCo {}",
            got.len(),
            want.len()
        ));
        return;
    }
    for (i, (a, b)) in got.iter().zip(want).enumerate() {
        if a.to_bits() != b.to_bits() {
            off.push(format!("{what}[{i}]: {a:e}, MuJoCo {b:e}"));
        }
    }
}

/// MuJoCo's `mjtConstraint` codes, in order, as this crate's row kinds.
const MJ_ROW_KINDS: [ConstraintType; 8] = [
    ConstraintType::Equality,
    ConstraintType::FrictionDof,
    ConstraintType::FrictionTendon,
    ConstraintType::LimitJoint,
    ConstraintType::LimitTendon,
    ConstraintType::ContactFrictionless,
    ConstraintType::ContactPyramidal,
    ConstraintType::ContactElliptic,
];

/// The case's model and data after a forward pass from its state, with
/// MuJoCo's inputs to the constraint solve put in its place: every one the
/// dual solvers read, the rows' kinds and ids checked to be MuJoCo's.
fn given_mujocos_inputs(name: &str, case: &Value) -> (Model, Data) {
    let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos.copy_from_slice(&floats(&case["qpos"]));
    data.qvel.copy_from_slice(&floats(&case["qvel"]));
    data.forward(&model).expect("forward");
    let nefc = count(&case["nefc"]);
    assert_eq!(data.efc_type.len(), nefc, "{name}: nefc");
    assert_eq!(
        (data.ne, data.nf),
        (count(&case["ne"]), count(&case["nf"])),
        "{name}: ne, nf"
    );
    assert_eq!(
        model.M_rowadr,
        usizes(&case["M_rowadr"]),
        "{name}: M_rowadr"
    );
    assert_eq!(
        model.M_colind,
        usizes(&case["M_colind"]),
        "{name}: M_colind"
    );

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
    data.qacc_warmstart
        .copy_from_slice(&floats(&case["qacc_warmstart"]));
    let j = floats(&case["efc_J"]);
    if case.get("efc_J_colind").is_some() {
        assert!(model.is_sparse(), "{name}: the sparse size");
        assert_eq!(
            data.efc_J_rownnz,
            usizes(&case["efc_J_rownnz"]),
            "{name}: rownnz"
        );
        assert_eq!(
            data.efc_J_colind,
            usizes(&case["efc_J_colind"]),
            "{name}: colind"
        );
        data.efc_J.fill(0.0);
        for r in 0..nefc {
            let span = data.efc_J_rowadr[r]..data.efc_J_rowadr[r] + data.efc_J_rownnz[r];
            for (&c, &value) in data.efc_J_colind[span.clone()].iter().zip(&j[span]) {
                data.efc_J[(r, c)] = value;
            }
        }
    } else {
        for r in 0..nefc {
            for c in 0..model.nv {
                data.efc_J[(r, c)] = j[r * model.nv + c];
            }
        }
    }
    data.efc_aref.copy_from_slice(&floats(&case["efc_aref"]));
    data.efc_R.copy_from_slice(&floats(&case["efc_R"]));
    data.efc_D.copy_from_slice(&floats(&case["efc_D"]));
    let mu = floats(&case["contact_mu"]);
    assert_eq!(data.contacts.len(), mu.len(), "{name}: ncon");
    for (c, &m) in data.contacts.iter_mut().zip(&mu) {
        c.regularized_mu = m;
    }
    let kinds: Vec<ConstraintType> = usizes(&case["efc_type"])
        .into_iter()
        .map(|t| MJ_ROW_KINDS[t])
        .collect();
    assert_eq!(data.efc_type, kinds, "{name}: the rows' kinds");
    assert_eq!(
        data.efc_id,
        usizes(&case["efc_id"]),
        "{name}: the rows' ids"
    );
    data.efc_floss
        .copy_from_slice(&floats(&case["efc_frictionloss"]));
    let friction = case["contact_friction"].as_array().expect("contacts");
    for (c, &dim) in data.contacts.iter_mut().zip(&usizes(&case["contact_dim"])) {
        c.dim = dim;
    }
    for i in 0..nefc {
        if matches!(
            data.efc_type[i],
            ConstraintType::ContactFrictionless
                | ConstraintType::ContactPyramidal
                | ConstraintType::ContactElliptic
        ) {
            data.efc_mu[i].copy_from_slice(&floats(&friction[data.efc_id[i]]));
        }
    }
    model.stat_meaninertia = case["stat_meaninertia"].as_f64().expect("meaninertia");
    (model, data)
}

/// `efc_AR` as the golden keeps it: the dense matrix, or at the sparse size
/// each row's columns and values, row after row.
fn assert_ar(off: &mut Vec<String>, name: &str, data: &Data, case: &Value) {
    match case.get("efc_AR_rownnz") {
        None => bits(
            off,
            &format!("{name} efc_AR"),
            &data.efc_AR,
            &floats(&case["efc_AR"]),
        ),
        Some(rownnz) => {
            if data.efc_AR_rownnz != usizes(rownnz)
                || data.efc_AR_rowadr != usizes(&case["efc_AR_rowadr"])
            {
                off.push(format!("{name}: efc_AR's rows' lengths or starts"));
                return;
            }
            let (mut cols, mut vals) = (Vec::new(), Vec::new());
            for (&adr, &nnz) in data.efc_AR_rowadr.iter().zip(&data.efc_AR_rownnz) {
                cols.extend_from_slice(&data.efc_AR_colind[adr..adr + nnz]);
                vals.extend_from_slice(&data.efc_AR[adr..adr + nnz]);
            }
            if cols != usizes(&case["efc_AR_colind"]) {
                off.push(format!("{name}: efc_AR's columns"));
            }
            bits(
                off,
                &format!("{name} efc_AR"),
                &vals,
                &floats(&case["efc_AR"]),
            );
            if data.efc_AR.len() != count(&case["nA"]) {
                off.push(format!(
                    "{name}: nA {}, MuJoCo {}",
                    data.efc_AR.len(),
                    count(&case["nA"])
                ));
            }
        }
    }
}

/// The rows' inverse inertia `efc_AR`, given MuJoCo's factor of the mass
/// matrix, Jacobian and regularization, for every case: dense, and at the
/// sparse size its rows' columns, lengths and starts as MuJoCo lays them out.
#[test]
fn efc_ar_is_mujocos_given_its_inputs() {
    let g = golden();
    let mut off = Vec::new();
    for group in ["pgs", "noslip"] {
        for (name, case) in g["cases"][group].as_object().expect("cases") {
            let (model, mut data) = given_mujocos_inputs(name, case);
            sim_core::constraint::mj_project_constraint(&model, &mut data);
            assert_ar(&mut off, name, &data, case);
        }
    }
    assert!(off.is_empty(), "{} differ:\n{}", off.len(), off.join("\n"));
}

/// MuJoCo's `mjtConstraintState` code of a row's state.
fn mj_state(state: ConstraintState) -> usize {
    match state {
        ConstraintState::Satisfied => 0,
        ConstraintState::Quadratic => 1,
        ConstraintState::LinearNeg => 2,
        ConstraintState::LinearPos => 3,
        ConstraintState::Cone => 4,
    }
}

/// The solve's outputs against the case's: forces, states, acceleration,
/// constraint force, iteration count, statistics and `solver_nnz`.
fn assert_solve(off: &mut Vec<String>, name: &str, data: &Data, case: &Value) {
    bits(
        off,
        &format!("{name} efc_force"),
        data.efc_force.as_slice(),
        &floats(&case["efc_force"]),
    );
    let states: Vec<usize> = data.efc_state.iter().map(|&s| mj_state(s)).collect();
    if states != usizes(&case["efc_state"]) {
        off.push(format!(
            "{name}: efc_state {states:?}, MuJoCo {:?}",
            usizes(&case["efc_state"])
        ));
    }
    bits(
        off,
        &format!("{name} qacc"),
        data.qacc.as_slice(),
        &floats(&case["qacc"]),
    );
    bits(
        off,
        &format!("{name} qfrc_constraint"),
        data.qfrc_constraint.as_slice(),
        &floats(&case["qfrc_constraint"]),
    );
    let stats = &case["stats"];
    if data.solver_niter != count(&stats["niter"]) {
        off.push(format!(
            "{name}: niter {}, MuJoCo {}",
            data.solver_niter,
            count(&stats["niter"])
        ));
    }
    let field = |f: fn(&SolverStat) -> f64| data.solver_stat.iter().map(f).collect::<Vec<_>>();
    bits(
        off,
        &format!("{name} improvement"),
        &field(|s| s.improvement),
        &floats(&stats["improvement"]),
    );
    let counts = |f: fn(&SolverStat) -> usize| data.solver_stat.iter().map(f).collect::<Vec<_>>();
    if counts(|s| s.nactive) != usizes(&stats["nactive"])
        || counts(|s| s.nchange) != usizes(&stats["nchange"])
    {
        off.push(format!("{name}: nactive or nchange"));
    }
    if data.solver_nnz != count(&case["solver_nnz"]) {
        off.push(format!(
            "{name}: solver_nnz {}, MuJoCo {}",
            data.solver_nnz,
            count(&case["solver_nnz"])
        ));
    }
}

/// PGS from its warmstart, given MuJoCo's inputs: the warmstart's forces kept
/// when their dual cost is not positive (the `*_warm` cases at a cost of
/// exactly 0 among them), zero otherwise (`*_zeroed`, most `*_warm`) and with
/// the warmstart disabled (`*_nowarm`); then every sweep as `mj_solPGS`'s, and
/// `dualFinish`: forces, states, acceleration, constraint force, iteration
/// count, each iteration's improvement and counts, and `solver_nnz`, bit for
/// bit, dense and at the sparse size.
#[test]
fn pgs_matches_mujoco_3_5_0_bit_for_bit_given_its_inputs() {
    let g = golden();
    let mut off = Vec::new();
    for (name, case) in g["cases"]["pgs"].as_object().expect("cases") {
        let (model, mut data) = given_mujocos_inputs(name, case);
        sim_core::constraint::mj_sol_pgs_rows(&model, &mut data);
        bits(
            &mut off,
            &format!("{name} efc_b"),
            data.efc_b.as_slice(),
            &floats(&case["efc_b"]),
        );
        assert_solve(&mut off, name, &data, case);
    }
    assert!(off.is_empty(), "{} differ:\n{}", off.len(), off.join("\n"));
}

/// The primal solver's `solver_nnz` after a forward pass, as MuJoCo's counts
/// it (`engine_solver.c:1953-1964`): Newton's sparse Hessian (its factor's
/// entries, twice under an elliptic cone, and the Hessian's count, which
/// takes every entry of M), Newton's dense `nv²`, CG's 0; with islands, the
/// first 20 islands' counts summed (ledger L221).
#[test]
fn primal_solver_nnz_is_mujocos() {
    let g = golden();
    let mut off = Vec::new();
    for (name, case) in g["cases"]["nnz"].as_object().expect("cases") {
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qpos.copy_from_slice(&floats(&case["qpos"]));
        data.qvel.copy_from_slice(&floats(&case["qvel"]));
        data.forward(&model).expect("forward");
        assert_eq!(data.nisland, count(&case["nisland"]), "{name}: islands");
        if data.solver_nnz != count(&case["solver_nnz"]) {
            off.push(format!(
                "{name}: solver_nnz {}, MuJoCo {} ({:?} by island)",
                data.solver_nnz,
                count(&case["solver_nnz"]),
                usizes(&case["solver_nnz_per_island"])
            ));
        }
    }
    assert!(off.is_empty(), "{} differ:\n{}", off.len(), off.join("\n"));
}

/// Noslip after PGS, Newton and CG, given MuJoCo's inputs: the main solver as
/// the forward pass runs it, then noslip's sweeps over the friction rows on
/// `efc_AR` less R, the normal forces fixed, ended by `dualFinish`
/// (`mj_solNoSlip`): forces, states, acceleration, constraint force, the
/// iteration count with noslip's sweeps, every statistic's improvement and
/// counts, and `solver_nnz` (the main solver's), bit for bit, dense and at
/// the sparse size.
#[test]
fn noslip_matches_mujoco_3_5_0_bit_for_bit_given_its_inputs() {
    let g = golden();
    let mut off = Vec::new();
    for (name, case) in g["cases"]["noslip"].as_object().expect("cases") {
        let (model, mut data) = given_mujocos_inputs(name, case);
        sim_core::constraint::mj_sol_noslip_rows(&model, &mut data).expect("solve");
        assert_solve(&mut off, name, &data, case);
    }
    assert!(off.is_empty(), "{} differ:\n{}", off.len(), off.join("\n"));
}

/// A forward pass that skips the position stage after an inverse one, whose
/// position stage makes no `efc_AR` (`mj_invPosition`), makes it before its
/// dual solve: under PGS, and under Newton with noslip, a sphere resting on a
/// plane gets the acceleration a full forward pass gets, bit for bit.
#[test]
fn a_forward_from_the_velocity_stage_after_an_inverse_makes_efc_ar() {
    for option in [
        r#"solver="PGS""#,
        r#"solver="Newton" noslip_iterations="5""#,
    ] {
        let xml = format!(
            r#"<mujoco><option timestep="0.002" {option}/><worldbody>
  <geom type="plane" size="5 5 0.1"/>
  <body pos="0 0 0.09"><freejoint/><geom type="sphere" size="0.1"/></body>
</worldbody></mujoco>"#
        );
        let model = load_model(&xml).expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        assert!(data.ncon > 0, "{option}: in contact");
        let full: Vec<u64> = data.qacc.iter().map(|x| x.to_bits()).collect();
        data.inverse(&model).expect("inverse");
        data.forward_skip(&model, MjStage::Pos, false)
            .expect("forward from the velocity stage");
        let skipped: Vec<u64> = data.qacc.iter().map(|x| x.to_bits()).collect();
        assert_eq!(skipped, full, "{option}: qacc");
    }
}

/// Under the implicit spring-damper integrator (this crate's; MuJoCo has
/// none) the constraint stage makes `efc_AR` over `M_impl`. With the model
/// then under Euler, a forward pass that skips the position stage remakes it
/// over `M` before its dual solve: a damped, sprung sphere on a plane,
/// pushed sideways, moving sideways and at rest sideways, gets under PGS and
/// under Newton with noslip the acceleration a full Euler pass gets, bit for
/// bit, and so does a clone of the `Data` the implicit pass left (ledger
/// L263).
#[test]
fn a_forward_from_the_velocity_stage_after_the_implicit_spring_damper_remakes_efc_ar() {
    for option in [
        r#"solver="PGS""#,
        r#"solver="Newton" noslip_iterations="5""#,
    ] {
        let xml = format!(
            r#"<mujoco><option timestep="0.002" integrator="implicitspringdamper" {option}/><worldbody>
  <geom type="plane" size="5 5 0.1"/>
  <body pos="0 0 0.09">
    <joint type="slide" axis="0 0 1" damping="3" stiffness="20"/>
    <joint type="slide" axis="1 0 0" damping="2"/>
    <geom type="sphere" size="0.1"/>
  </body>
</worldbody></mujoco>"#
        );
        for qvel in [[-0.3, 0.4], [-0.3, 0.0]] {
            let mut model = load_model(&xml).expect("load");
            let mut data = model.make_data();
            data.qvel.as_mut_slice().copy_from_slice(&qvel);
            data.qfrc_applied[1] = 0.5;
            data.forward(&model).expect("forward");
            assert!(data.ncon > 0, "{option} {qvel:?}: in contact");
            let mut copy = data.clone();
            model.integrator = Integrator::Euler;
            data.forward_skip(&model, MjStage::Pos, false)
                .expect("forward from the velocity stage");
            let skipped: Vec<u64> = data.qacc.iter().map(|x| x.to_bits()).collect();
            copy.forward_skip(&model, MjStage::Pos, false)
                .expect("the clone's forward from the velocity stage");
            let copied: Vec<u64> = copy.qacc.iter().map(|x| x.to_bits()).collect();
            data.forward(&model).expect("forward under Euler");
            let full: Vec<u64> = data.qacc.iter().map(|x| x.to_bits()).collect();
            assert_eq!(skipped, full, "{option} {qvel:?}: qacc");
            assert_eq!(copied, full, "{option} {qvel:?}: the clone's qacc");
        }
    }
}

/// The whole forward pass under PGS, PGS then noslip, Newton then noslip, and
/// CG then noslip, from MuJoCo's state after three steps (qpos, qvel, the
/// warmstart): its
/// acceleration, forces and constraint force as MuJoCo's, bit for bit on
/// macOS arm64, where the golden was made, elsewhere within
/// `1e-13 · max(1, |MuJoCo's|)` (a NaN never), and its iteration count. On
/// every platform the pass keeps the dual solve's own acceleration
/// (`dualFinish`'s), bit for bit: the same rows solved again from the same
/// warmstart give it, where one remade from the forces would round otherwise.
/// CG's scenes carry each body's inertial as MuJoCo compiles it: as written,
/// sim-mjcf compiles the arm capsule's inertial frame otherwise (ledgers L195,
/// L261), which moves the mass matrix by an ulp at six of CG's states.
#[test]
fn a_forward_pass_with_a_dual_solver_matches_mujoco_3_5_0() {
    let g = golden();
    let exact = cfg!(all(target_os = "macos", target_arch = "aarch64"));
    let mut off = Vec::new();
    for (name, case) in g["cases"]["forward"].as_object().expect("cases") {
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qpos.copy_from_slice(&floats(&case["qpos"]));
        data.qvel.copy_from_slice(&floats(&case["qvel"]));
        data.qacc_warmstart
            .copy_from_slice(&floats(&case["qacc_warmstart"]));
        data.forward(&model).expect("forward");
        assert_eq!(data.efc_type.len(), count(&case["nefc"]), "{name}: nefc");
        if data.solver_niter != count(&case["solver_niter"]) {
            off.push(format!(
                "{name}: solver_niter {}, MuJoCo {}",
                data.solver_niter,
                count(&case["solver_niter"])
            ));
        }
        for (field, ours) in [
            ("qacc", data.qacc.as_slice()),
            ("efc_force", data.efc_force.as_slice()),
            ("qfrc_constraint", data.qfrc_constraint.as_slice()),
        ] {
            let want = floats(&case[field]);
            let what = format!("{name} {field}");
            if exact {
                bits(&mut off, &what, ours, &want);
            } else if ours.len() != want.len()
                || ours.iter().zip(&want).any(|(a, b)| {
                    // Past the bound or NaN: a NaN is never within it.
                    let d = (a - b).abs();
                    d.is_nan() || d > 1e-13 * b.abs().max(1.0)
                })
            {
                off.push(format!("{what}: past 1e-13 of MuJoCo's"));
            }
        }
        let mut again = data.clone();
        if model.noslip_iterations > 0 {
            sim_core::constraint::mj_sol_noslip_rows(&model, &mut again).expect("solve");
        } else {
            sim_core::constraint::mj_sol_pgs_rows(&model, &mut again);
        }
        let what = format!("{name} qacc against the solve's own");
        bits(&mut off, &what, data.qacc.as_slice(), again.qacc.as_slice());
    }
    assert!(off.is_empty(), "{} differ:\n{}", off.len(), off.join("\n"));
}
