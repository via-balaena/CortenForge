//! Phase 13 Session 0 — Diagnostic test for golden flag divergence.
//!
//! Loads the flag_golden_test.xml model, runs one forward pass, and dumps
//! per-constraint-row data for comparison against MuJoCo reference.

use sim_core::types::{ConstraintType, Data, Model};
use sim_mjcf::load_model;

const CTRL_VALUE: f64 = 2.0;

fn golden_path(filename: &str) -> std::path::PathBuf {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    std::path::Path::new(manifest_dir)
        .join("assets")
        .join("golden")
        .join("flags")
        .join(filename)
}

/// Parse a NumPy .npy v1.0 file into shape + f64 data.
fn parse_npy(path: &std::path::Path) -> Option<(Vec<usize>, Vec<f64>)> {
    let bytes = std::fs::read(path).ok()?;
    if bytes.len() < 10 || &bytes[0..6] != b"\x93NUMPY" {
        panic!("invalid .npy magic");
    }
    let major = bytes[6];
    let minor = bytes[7];
    assert!(major == 1 && minor == 0, "only .npy v1.0 supported");

    let header_len = u16::from_le_bytes([bytes[8], bytes[9]]) as usize;
    let header_end = 10 + header_len;
    let header = std::str::from_utf8(&bytes[10..header_end]).expect("header not utf-8");

    assert!(
        header.contains("'<f8'") || header.contains("'float64'"),
        "only float64 dtype supported"
    );
    assert!(
        header.contains("'fortran_order': False"),
        "only C-contiguous arrays supported"
    );

    let shape_start = header.find("'shape': (").expect("no shape") + 10;
    let shape_end = header[shape_start..].find(')').expect("no shape end") + shape_start;
    let shape_str = &header[shape_start..shape_end];
    let shape: Vec<usize> = shape_str
        .split(',')
        .filter(|s| !s.trim().is_empty())
        .map(|s| s.trim().parse().expect("bad shape dim"))
        .collect();

    let data_bytes = &bytes[header_end..];
    let n_elements: usize = shape.iter().product();
    assert_eq!(data_bytes.len(), n_elements * 8);

    let data: Vec<f64> = data_bytes
        .chunks_exact(8)
        .map(|chunk| f64::from_le_bytes(chunk.try_into().unwrap()))
        .collect();

    Some((shape, data))
}

fn load_npy_1d(subdir: &str, name: &str) -> Option<Vec<f64>> {
    let path = golden_path("diagnostic").join(subdir).join(name);
    parse_npy(&path).map(|(_, data)| data)
}

fn load_npy_2d(subdir: &str, name: &str) -> Option<(usize, usize, Vec<f64>)> {
    let path = golden_path("diagnostic").join(subdir).join(name);
    parse_npy(&path).map(|(shape, data)| (shape[0], shape[1], data))
}

fn type_name(ct: &ConstraintType) -> &'static str {
    match ct {
        ConstraintType::Equality => "Equality",
        ConstraintType::FrictionLoss => "FrictionLoss",
        ConstraintType::LimitJoint => "LimitJoint",
        ConstraintType::LimitTendon => "LimitTendon",
        ConstraintType::ContactFrictionless => "ContactFrictionless",
        ConstraintType::ContactPyramidal => "ContactPyramidal",
        ConstraintType::ContactElliptic => "ContactElliptic",
        ConstraintType::FlexEdge => "FlexEdge",
    }
}

fn dump_cortenforge_data(model: &Model, data: &Data) {
    let nefc = data.efc_type.len();
    let nv = model.nv;

    println!("\n=== CortenForge Constraint Data ===");
    println!("  nefc={nefc}  ne={}  nf={}  nv={nv}", data.ne, data.nf);
    println!("  ncon={}", data.contacts.len());
    println!("  solver_type={:?}", model.solver_type);

    for i in 0..nefc {
        let ctype = &data.efc_type[i];
        println!("\n  Row {i}: type={} ({ctype:?})", type_name(ctype));
        println!("    efc_id         = {}", data.efc_id[i]);
        println!("    efc_pos        = {:.15e}", data.efc_pos[i]);
        println!("    efc_margin     = {:.15e}", data.efc_margin[i]);

        // Jacobian nonzeros
        let mut nonzero = Vec::new();
        for col in 0..nv {
            let v = data.efc_J[(i, col)];
            if v.abs() > 1e-20 {
                nonzero.push((col, v));
            }
        }
        println!("    efc_J          = {nonzero:?}");
        println!("    efc_diagApprox = {:.15e}", data.efc_diagApprox[i]);
        println!("    efc_R          = {:.15e}", data.efc_R[i]);
        println!("    efc_D          = {:.15e}", data.efc_D[i]);
        println!("    efc_aref       = {:.15e}", data.efc_aref[i]);
        println!("    efc_b          = {:.15e}", data.efc_b[i]);
        println!("    efc_force      = {:.15e}", data.efc_force[i]);
        println!("    efc_state      = {:?}", data.efc_state[i]);
    }

    print!("\n  qacc = [");
    for i in 0..nv {
        if i > 0 {
            print!(", ");
        }
        print!("{:.15e}", data.qacc[i]);
    }
    println!("]");

    print!("  qacc_smooth = [");
    for i in 0..nv {
        if i > 0 {
            print!(", ");
        }
        print!("{:.15e}", data.qacc_smooth[i]);
    }
    println!("]");

    print!("  qfrc_constraint = [");
    for i in 0..nv {
        if i > 0 {
            print!(", ");
        }
        print!("{:.15e}", data.qfrc_constraint[i]);
    }
    println!("]");

    print!("  qfrc_smooth = [");
    for i in 0..nv {
        if i > 0 {
            print!(", ");
        }
        print!("{:.15e}", data.qfrc_smooth[i]);
    }
    println!("]");

    // Mass matrix diagonal
    print!("  qM_diag = [");
    for i in 0..nv {
        if i > 0 {
            print!(", ");
        }
        print!("{:.15e}", data.qM[(i, i)]);
    }
    println!("]");

    // invweight0
    print!("  dof_invweight0 = [");
    for i in 0..nv {
        if i > 0 {
            print!(", ");
        }
        print!("{:.15e}", model.dof_invweight0[i]);
    }
    println!("]");
}

/// T1 (Spec A, AC1): Flag model tendon_invweight0 conformance.
/// MuJoCo 3.4.0 reference: tendon_invweight0[0] ≈ 5388.92
#[test]
fn phase13_tendon_invweight0_flag_model() {
    let model_path = golden_path("flag_golden_test.xml");
    let xml = std::fs::read_to_string(&model_path).expect("read model");
    let model = load_model(&xml).expect("load model");

    // AC1: flag model tendon_invweight0[0] ≈ 5388.92 (MuJoCo 3.4.0 reference)
    assert!(
        (model.tendon_invweight0[0] - 5388.92).abs() < 1e-2,
        "tendon_invweight0[0] = {}, expected ≈ 5388.92",
        model.tendon_invweight0[0]
    );
}

fn compare_field(label: &str, cf_val: f64, mj_val: f64, row: usize) -> bool {
    let diff = (cf_val - mj_val).abs();
    let ok = diff < 1e-6;
    if !ok {
        println!(
            "  ** DIVERGENCE row {row} {label}: CF={cf_val:.15e}  MJ={mj_val:.15e}  diff={diff:.6e}"
        );
    }
    ok
}

#[test]
#[allow(clippy::needless_range_loop)]
fn phase13_diagnostic_constraint_comparison() {
    let model_path = golden_path("flag_golden_test.xml");
    let xml = std::fs::read_to_string(&model_path).expect("read model");
    let model = load_model(&xml).expect("load model");

    let mut data = model.make_data();
    data.ctrl[0] = CTRL_VALUE;

    // Run forward() to populate constraint data without stepping.
    // forward() does FK + CRBA + passive + collision + constraint assembly + solve.
    data.forward(&model).expect("forward failed");

    let nefc = data.efc_type.len();
    let nv = model.nv;

    // Dump CortenForge data
    dump_cortenforge_data(&model, &data);

    // Load MuJoCo reference data
    let subdir = "step0_pre";
    let mj_diag = load_npy_1d(subdir, "efc_diagApprox.npy");
    let mj_r = load_npy_1d(subdir, "efc_R.npy");
    let mj_d = load_npy_1d(subdir, "efc_D.npy");
    let mj_aref = load_npy_1d(subdir, "efc_aref.npy");
    let mj_b = load_npy_1d(subdir, "efc_b.npy");
    let mj_force = load_npy_1d(subdir, "efc_force.npy");
    let mj_pos = load_npy_1d(subdir, "efc_pos.npy");
    let mj_qacc = load_npy_1d(subdir, "qacc.npy");
    let mj_qacc_smooth = load_npy_1d(subdir, "qacc_smooth.npy");
    let mj_qfrc_constraint = load_npy_1d(subdir, "qfrc_constraint.npy");
    let mj_qfrc_smooth = load_npy_1d(subdir, "qfrc_smooth.npy");
    let mj_j = load_npy_2d(subdir, "efc_J.npy");
    let mj_dof_invweight0 = load_npy_1d(subdir, "dof_invweight0.npy");
    let mj_qm = load_npy_2d(subdir, "qM_full.npy");

    if mj_diag.is_none() {
        println!("SKIP: MuJoCo reference data not found. Run dump_constraint_diagnostic.py first.");
        return;
    }

    let mj_diag = mj_diag.unwrap();
    let mj_r = mj_r.unwrap();
    let mj_d = mj_d.unwrap();
    let mj_aref = mj_aref.unwrap();
    let mj_b = mj_b.unwrap();
    let mj_force = mj_force.unwrap();
    let mj_pos = mj_pos.unwrap();
    let mj_qacc = mj_qacc.unwrap();
    let mj_qacc_smooth = mj_qacc_smooth.unwrap();
    let mj_qfrc_constraint = mj_qfrc_constraint.unwrap();
    let mj_qfrc_smooth = mj_qfrc_smooth.unwrap();
    let mj_dof_invweight0 = mj_dof_invweight0.unwrap();

    println!("\n\n{}", "=".repeat(80));
    println!("=== COMPARISON: CortenForge vs MuJoCo ===");
    println!("{}", "=".repeat(80));

    // Compare row count
    println!("\nRow count: CF={nefc}  MJ={}", mj_diag.len());
    let mj_nefc = mj_diag.len();

    // Compare dof_invweight0
    println!("\n--- dof_invweight0 ---");
    for i in 0..nv.min(mj_dof_invweight0.len()) {
        compare_field(
            "dof_invweight0",
            model.dof_invweight0[i],
            mj_dof_invweight0[i],
            i,
        );
    }

    // Compare mass matrix diagonal
    if let Some((_, _, ref mj_qm_data)) = mj_qm {
        println!("\n--- Mass matrix diagonal ---");
        for i in 0..nv {
            let mj_val = mj_qm_data[i * nv + i];
            compare_field("qM_diag", data.qM[(i, i)], mj_val, i);
        }
    }

    // Compare qacc_smooth
    println!("\n--- qacc_smooth ---");
    for i in 0..nv.min(mj_qacc_smooth.len()) {
        compare_field("qacc_smooth", data.qacc_smooth[i], mj_qacc_smooth[i], i);
    }

    // Compare qfrc_smooth
    println!("\n--- qfrc_smooth ---");
    for i in 0..nv.min(mj_qfrc_smooth.len()) {
        compare_field("qfrc_smooth", data.qfrc_smooth[i], mj_qfrc_smooth[i], i);
    }

    // Per-row comparison
    let compare_rows = nefc.min(mj_nefc);
    println!("\n--- Per-row constraint data (comparing {compare_rows} rows) ---");
    for i in 0..compare_rows {
        println!("\n  Row {i} (CF type={:?}):", data.efc_type[i]);

        // Jacobian comparison
        if let Some((_, ncols, ref mj_j_data)) = mj_j {
            for col in 0..nv.min(ncols) {
                let cf_val = data.efc_J[(i, col)];
                let mj_val = mj_j_data[i * ncols + col];
                if cf_val.abs() > 1e-20 || mj_val.abs() > 1e-20 {
                    compare_field(&format!("efc_J[{i},{col}]"), cf_val, mj_val, i);
                }
            }
        }

        compare_field("efc_pos", data.efc_pos[i], mj_pos[i], i);
        compare_field("efc_diagApprox", data.efc_diagApprox[i], mj_diag[i], i);
        compare_field("efc_R", data.efc_R[i], mj_r[i], i);
        compare_field("efc_D", data.efc_D[i], mj_d[i], i);
        compare_field("efc_aref", data.efc_aref[i], mj_aref[i], i);
        compare_field("efc_b", data.efc_b[i], mj_b[i], i);
        compare_field("efc_force", data.efc_force[i], mj_force[i], i);
    }

    // Compare qacc
    println!("\n--- qacc (final) ---");
    for i in 0..nv.min(mj_qacc.len()) {
        compare_field("qacc", data.qacc[i], mj_qacc[i], i);
    }

    // Compare qfrc_constraint
    println!("\n--- qfrc_constraint ---");
    for i in 0..nv.min(mj_qfrc_constraint.len()) {
        compare_field(
            "qfrc_constraint",
            data.qfrc_constraint[i],
            mj_qfrc_constraint[i],
            i,
        );
    }

    // Summary: which row diverges first?
    println!("\n\n=== DIVERGENCE SUMMARY ===");
    let mut first_diverge_row: Option<usize> = None;
    let mut first_diverge_field: Option<String> = None;
    let mut first_diverge_diff: f64 = 0.0;

    for i in 0..compare_rows {
        let fields = [
            ("efc_diagApprox", data.efc_diagApprox[i], mj_diag[i]),
            ("efc_R", data.efc_R[i], mj_r[i]),
            ("efc_D", data.efc_D[i], mj_d[i]),
            ("efc_aref", data.efc_aref[i], mj_aref[i]),
            ("efc_b", data.efc_b[i], mj_b[i]),
            ("efc_force", data.efc_force[i], mj_force[i]),
        ];

        for (name, cf, mj) in &fields {
            let diff = (cf - mj).abs();
            if diff > 1e-6 && first_diverge_row.is_none() {
                first_diverge_row = Some(i);
                first_diverge_field = Some(name.to_string());
                first_diverge_diff = diff;
            }
        }
    }

    if let Some(row) = first_diverge_row {
        println!(
            "  First divergence: row {row} ({:?}) field={} diff={:.6e}",
            data.efc_type[row],
            first_diverge_field.as_ref().unwrap(),
            first_diverge_diff
        );
    } else if nefc != mj_nefc {
        println!("  Row count mismatch: CF={nefc}  MJ={mj_nefc}");
    } else {
        println!("  No per-row divergence found! All constraint fields match within 1e-6.");
        println!("  Divergence must be in solver computation (Newton vs MuJoCo Newton).");
    }

    // qacc divergence detail
    println!("\n  qacc divergence per DOF:");
    for i in 0..nv.min(mj_qacc.len()) {
        let diff = (data.qacc[i] - mj_qacc[i]).abs();
        println!(
            "    dof {i}: CF={:.15e}  MJ={:.15e}  diff={:.6e}",
            data.qacc[i], mj_qacc[i], diff
        );
    }
}
