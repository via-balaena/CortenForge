//! Phase 13 Spec B — PGS solver conformance tests.
//!
//! Tests PGS early termination (DT-128), solver_stat population (S4),
//! Newton regression (AC6), and PGS MuJoCo conformance (AC9).

use sim_mjcf::load_model;

// ============================================================================
// .npy parser (for T7 MuJoCo conformance reference data)
// ============================================================================

/// Parse a NumPy .npy v1.0 file into shape + f64 data.
fn parse_npy_f64(path: &std::path::Path) -> Option<(Vec<usize>, Vec<f64>)> {
    let bytes = std::fs::read(path).ok()?;
    if bytes.len() < 10 || &bytes[0..6] != b"\x93NUMPY" {
        return None;
    }
    let header_len = u16::from_le_bytes([bytes[8], bytes[9]]) as usize;
    let header_end = 10 + header_len;
    let header = std::str::from_utf8(&bytes[10..header_end]).ok()?;

    let shape_start = header.find("'shape': (")? + 10;
    let shape_end = header[shape_start..].find(')')? + shape_start;
    let shape: Vec<usize> = header[shape_start..shape_end]
        .split(',')
        .filter(|s| !s.trim().is_empty())
        .map(|s| s.trim().parse().ok())
        .collect::<Option<Vec<_>>>()?;

    let data_bytes = &bytes[header_end..];
    let n_elements: usize = shape.iter().product();
    if n_elements == 0 {
        return Some((shape, vec![]));
    }
    if data_bytes.len() < n_elements * 8 {
        return None;
    }
    let data: Vec<f64> = data_bytes[..n_elements * 8]
        .chunks_exact(8)
        .map(|c| f64::from_le_bytes(c.try_into().unwrap()))
        .collect();
    Some((shape, data))
}

/// Parse a scalar int64 .npy file.
fn parse_npy_i64(path: &std::path::Path) -> Option<i64> {
    let bytes = std::fs::read(path).ok()?;
    if bytes.len() < 10 || &bytes[0..6] != b"\x93NUMPY" {
        return None;
    }
    let header_len = u16::from_le_bytes([bytes[8], bytes[9]]) as usize;
    let header_end = 10 + header_len;
    let data_bytes = &bytes[header_end..];
    if data_bytes.len() < 8 {
        return None;
    }
    Some(i64::from_le_bytes(data_bytes[..8].try_into().unwrap()))
}

/// Helper: load model from MJCF string.
fn model_from_mjcf(mjcf: &str) -> (sim_core::Model, sim_core::Data) {
    let model = load_model(mjcf).expect("MJCF should load");
    let data = model.make_data();
    (model, data)
}

/// Minimal PGS model: 1 body, 1 hinge, 1 equality constraint (joint coupling).
/// solver = PGS, iterations = 100, tolerance = 1e-8.
fn pgs_test_model() -> (sim_core::Model, sim_core::Data) {
    model_from_mjcf(
        r#"
        <mujoco model="pgs_test">
            <option gravity="0 0 -9.81" timestep="0.002" solver="PGS"
                    iterations="100" tolerance="1e-8"/>
            <worldbody>
                <body name="arm1" pos="0 0 0">
                    <joint name="hinge1" type="hinge" axis="0 0 1"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                    <body name="arm2" pos="0 0.6 0">
                        <joint name="hinge2" type="hinge" axis="0 0 1"/>
                        <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <equality>
                <joint joint1="hinge1" joint2="hinge2" polycoef="0 0.5 0 0 0"/>
            </equality>
        </mujoco>
    "#,
    )
}

// ============================================================================
// T1: PGS early termination → AC1
// ============================================================================

#[test]
fn test_pgs_early_termination() {
    let (model, mut data) = pgs_test_model();
    data.step(&model).expect("step should succeed");

    // PGS should converge before max_iters for this simple model
    assert!(
        data.solver_niter < 100,
        "PGS should converge early, got solver_niter = {}",
        data.solver_niter
    );
    assert!(data.solver_niter > 0, "PGS should run at least 1 iteration");
}

// ============================================================================
// T2: PGS force correctness → AC2
// ============================================================================

#[test]
fn test_pgs_force_correctness_early_vs_full() {
    // Run with early termination (tolerance = 1e-8)
    let (model_early, mut data_early) = pgs_test_model();
    data_early.step(&model_early).expect("step should succeed");
    let forces_early = data_early.efc_force.clone();
    let niter_early = data_early.solver_niter;

    // Run without early termination (tolerance = 0.0, iterations = 100)
    let (mut model_full, mut data_full) = pgs_test_model();
    model_full.solver_tolerance = 0.0;
    data_full.step(&model_full).expect("step should succeed");
    let forces_full = data_full.efc_force.clone();

    assert_eq!(forces_early.len(), forces_full.len());
    assert!(niter_early < 100, "early should converge before 100");

    // Forces should match within tolerance after convergence
    for i in 0..forces_early.len() {
        let diff = (forces_early[i] - forces_full[i]).abs();
        assert!(
            diff < 1e-6,
            "efc_force[{}] diverged: early={}, full={}, diff={}",
            i,
            forces_early[i],
            forces_full[i],
            diff
        );
    }
}

// ============================================================================
// T3: PGS solver_stat → AC3
// ============================================================================

#[test]
fn test_pgs_solver_stat_populated() {
    let (model, mut data) = pgs_test_model();
    data.step(&model).expect("step should succeed");

    assert_eq!(
        data.solver_stat.len(),
        data.solver_niter,
        "solver_stat.len() should equal solver_niter"
    );

    for (i, stat) in data.solver_stat.iter().enumerate() {
        assert!(
            stat.improvement >= 0.0,
            "improvement[{}] = {} should be >= 0",
            i,
            stat.improvement
        );
        assert_eq!(stat.gradient, 0.0, "PGS gradient should be 0");
        assert_eq!(stat.lineslope, 0.0, "PGS lineslope should be 0");
        assert_eq!(stat.nline, 0, "PGS nline should be 0");
    }

    // Last entry's improvement should be below tolerance (convergence criterion)
    if let Some(last) = data.solver_stat.last() {
        assert!(
            last.improvement < model.solver_tolerance,
            "last improvement {} should be < tolerance {}",
            last.improvement,
            model.solver_tolerance
        );
    }
}

// ============================================================================
// T4: PGS max_iters edge case → AC4
// ============================================================================

#[test]
fn test_pgs_max_iters_honored() {
    let (mut model, mut data) = pgs_test_model();
    model.solver_tolerance = 0.0; // Disable early termination
    model.solver_iterations = 5;

    data.step(&model).expect("step should succeed");
    assert_eq!(
        data.solver_niter, 5,
        "should run exactly max_iters when tolerance = 0"
    );
    assert_eq!(data.solver_stat.len(), 5);
}

#[test]
fn test_pgs_zero_iterations() {
    let (mut model, mut data) = pgs_test_model();
    model.solver_iterations = 0;

    data.step(&model).expect("step should succeed");
    assert_eq!(
        data.solver_niter, 0,
        "zero iterations means solver_niter = 0"
    );
    assert!(data.solver_stat.is_empty());
}

// ============================================================================
// T5: Newton regression → AC6
// ============================================================================

#[test]
fn test_newton_unaffected_by_pgs_changes() {
    // Load the golden flag test model (Newton solver, default)
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let model_path = std::path::Path::new(manifest_dir)
        .join("assets")
        .join("golden")
        .join("flags")
        .join("flag_golden_test.xml");
    let mjcf = std::fs::read_to_string(&model_path).expect("flag_golden_test.xml should exist");

    let model = load_model(&mjcf).expect("model should load");
    let mut data = model.make_data();

    // Set ctrl
    if !data.ctrl.is_empty() {
        data.ctrl[0] = 2.0;
    }

    data.step(&model).expect("step should succeed");

    // Verify solver is Newton (not PGS)
    assert!(
        matches!(model.solver_type, sim_core::SolverType::Newton),
        "flag model should use Newton solver"
    );

    // Key qacc values from Spec A Session 4 (post tendon_invweight0 fix)
    // DOF 0 (hinge1) ≈ 90.766, DOF 4 (free tz) ≈ 2.0095
    // These should be stable across Spec B (PGS-only changes)
    assert!(
        data.qacc.len() >= 5,
        "flag model should have at least 5 DOFs"
    );

    // Verify gravity DOF is correct (this is the most stable reference)
    let qacc_tz = data.qacc[4];
    assert!(
        (qacc_tz - 2.0095).abs() < 0.01,
        "DOF 4 (free tz) = {}, expected ~2.0095",
        qacc_tz
    );
}

// ============================================================================
// T7: PGS MuJoCo conformance → AC9
// ============================================================================

#[test]
fn test_pgs_mujoco_conformance() {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let pgs_dir = std::path::Path::new(manifest_dir)
        .join("assets")
        .join("golden")
        .join("pgs_conformance");

    // Load reference data
    let (_, ref_force) =
        parse_npy_f64(&pgs_dir.join("pgs_efc_force.npy")).expect("pgs_efc_force.npy should exist");
    let (_, ref_qacc) =
        parse_npy_f64(&pgs_dir.join("pgs_qacc.npy")).expect("pgs_qacc.npy should exist");
    let ref_niter = parse_npy_i64(&pgs_dir.join("pgs_solver_niter.npy"))
        .expect("pgs_solver_niter.npy should exist");

    // Load and run model
    let model_path = pgs_dir.join("pgs_test_model.xml");
    let xml = std::fs::read_to_string(&model_path).expect("model XML should exist");
    let model = load_model(&xml).expect("model should load");
    let mut data = model.make_data();

    assert!(
        matches!(model.solver_type, sim_core::SolverType::PGS),
        "model should use PGS solver"
    );

    data.step(&model).expect("step should succeed");

    // AC9: solver_niter matches MuJoCo exactly
    assert_eq!(
        data.solver_niter, ref_niter as usize,
        "solver_niter: CF={}, MJ={}",
        data.solver_niter, ref_niter
    );

    // AC9: efc_force within 1e-10 of MuJoCo
    assert_eq!(data.efc_force.len(), ref_force.len());
    for (i, &mj_f) in ref_force.iter().enumerate() {
        let diff = (data.efc_force[i] - mj_f).abs();
        assert!(
            diff < 1e-10,
            "efc_force[{}]: CF={}, MJ={}, diff={}",
            i,
            data.efc_force[i],
            mj_f,
            diff
        );
    }

    // AC9: qacc within 1e-10 of MuJoCo
    assert_eq!(data.qacc.len(), ref_qacc.len());
    for (i, &mj_q) in ref_qacc.iter().enumerate() {
        let diff = (data.qacc[i] - mj_q).abs();
        assert!(
            diff < 1e-10,
            "qacc[{}]: CF={}, MJ={}, diff={}",
            i,
            data.qacc[i],
            mj_q,
            diff
        );
    }
}
