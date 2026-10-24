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
use sim_core::{Data, Model};
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

/// The case's model and data after a forward pass from its state, with
/// MuJoCo's inputs to the constraint solve put in its place.
fn given_mujocos_inputs(name: &str, case: &Value) -> (Model, Data) {
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
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
