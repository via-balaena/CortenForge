//! Shared utilities for MuJoCo conformance tests.
//!
//! NOTE: `parse_npy()` is intentionally duplicated from `integration/golden_flags.rs`.
//! The `mujoco_conformance` and `integration` test binaries are separate `[[test]]`
//! targets and cannot share code via module imports. ~50 lines of duplication is
//! the correct approach.

/// Parse a NumPy .npy v1.0 file into shape + f64 data.
///
/// Supports only v1.0 format, little-endian float64 (`<f8`), C-contiguous.
/// Returns `None` if the file doesn't exist or can't be parsed.
#[allow(dead_code)]
pub fn parse_npy(path: &std::path::Path) -> Option<(Vec<usize>, Vec<f64>)> {
    let bytes = std::fs::read(path).ok()?;

    // Magic: \x93NUMPY
    if bytes.len() < 10 || &bytes[0..6] != b"\x93NUMPY" {
        panic!("invalid .npy magic");
    }
    let major = bytes[6];
    let minor = bytes[7];
    assert!(major == 1 && minor == 0, "only .npy v1.0 supported");

    let header_len = u16::from_le_bytes([bytes[8], bytes[9]]) as usize;
    let header_end = 10 + header_len;
    let header = std::str::from_utf8(&bytes[10..header_end]).expect("header not utf-8");

    // Parse dtype — must be '<f8' (little-endian float64)
    assert!(
        header.contains("'<f8'") || header.contains("'float64'"),
        "only float64 dtype supported, got: {header}"
    );

    // Parse Fortran order — must be False
    assert!(
        header.contains("'fortran_order': False"),
        "only C-contiguous arrays supported"
    );

    // Parse shape: 'shape': (10, 1) or 'shape': (10,)
    let shape_start = header.find("'shape': (").expect("no shape in header") + 10;
    let shape_end = header[shape_start..].find(')').expect("no shape end") + shape_start;
    let shape_str = &header[shape_start..shape_end];
    let shape: Vec<usize> = shape_str
        .split(',')
        .filter(|s| !s.trim().is_empty())
        .map(|s| s.trim().parse().expect("bad shape dim"))
        .collect();

    // Parse data
    let data_bytes = &bytes[header_end..];
    let n_elements: usize = shape.iter().product();
    assert_eq!(
        data_bytes.len(),
        n_elements * 8,
        "data size mismatch: {} bytes for {} elements",
        data_bytes.len(),
        n_elements
    );

    let data: Vec<f64> = data_bytes
        .chunks_exact(8)
        .map(|chunk| f64::from_le_bytes(chunk.try_into().unwrap()))
        .collect();

    Some((shape, data))
}

// ── Tolerance constants for conformance tests ──
// Starting points from Phase 12 Umbrella spec. Rubric/spec sessions refine these.

#[allow(dead_code)]
pub const TOL_FK: f64 = 1e-12;
#[allow(dead_code)]
pub const TOL_CRBA: f64 = 1e-12;
#[allow(dead_code)]
pub const TOL_RNE: f64 = 1e-10;
#[allow(dead_code)]
pub const TOL_PASSIVE: f64 = 1e-10;
#[allow(dead_code)]
pub const TOL_COLLISION_DEPTH: f64 = 1e-6;
#[allow(dead_code)]
pub const TOL_CONSTRAINT: f64 = 1e-4;
#[allow(dead_code)]
pub const TOL_ACTUATION: f64 = 1e-10;
#[allow(dead_code)]
pub const TOL_SENSOR: f64 = 1e-8;
#[allow(dead_code)]
pub const TOL_TENDON: f64 = 1e-10;
#[allow(dead_code)]
pub const TOL_INTEGRATION: f64 = 1e-8;
#[allow(dead_code)]
pub const TOL_FLAG_GOLDEN: f64 = 1e-8;
#[allow(dead_code)]
pub const TOL_CONSTRAINT_JAC: f64 = 1e-8;

/// Parse a NumPy .npy v1.0 file with int32 (`<i4`) dtype.
/// Used for contact geom pair reference data.
#[allow(dead_code)]
pub fn parse_npy_i32(path: &std::path::Path) -> Option<(Vec<usize>, Vec<i32>)> {
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
        header.contains("'<i4'") || header.contains("'int32'"),
        "expected int32 dtype, got: {header}"
    );
    assert!(
        header.contains("'fortran_order': False"),
        "only C-contiguous arrays supported"
    );
    let shape_start = header.find("'shape': (").expect("no shape in header") + 10;
    let shape_end = header[shape_start..].find(')').expect("no shape end") + shape_start;
    let shape_str = &header[shape_start..shape_end];
    let shape: Vec<usize> = shape_str
        .split(',')
        .filter(|s| !s.trim().is_empty())
        .map(|s| s.trim().parse().expect("bad shape dim"))
        .collect();
    let data_bytes = &bytes[header_end..];
    let n_elements: usize = shape.iter().product();
    assert_eq!(data_bytes.len(), n_elements * 4, "data size mismatch");
    let data: Vec<i32> = data_bytes
        .chunks_exact(4)
        .map(|chunk| i32::from_le_bytes(chunk.try_into().unwrap()))
        .collect();
    Some((shape, data))
}

/// Resolve path to a conformance reference data file.
#[allow(dead_code)]
pub fn reference_path(filename: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("assets/golden/conformance/reference")
        .join(filename)
}

/// Resolve path to a conformance model file and load it.
#[allow(dead_code)]
pub fn load_conformance_model(name: &str) -> (sim_core::Model, sim_core::Data) {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("assets/golden/conformance/models")
        .join(format!("{name}.xml"));
    let xml = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()));
    let model =
        sim_mjcf::load_model(&xml).unwrap_or_else(|e| panic!("failed to load model {name}: {e}"));
    let data = model.make_data();
    (model, data)
}

/// Load reference .npy float64 data, panicking with a diagnostic message on failure.
#[allow(dead_code)]
pub fn load_reference_f64(model: &str, stage: &str, field: &str) -> (Vec<usize>, Vec<f64>) {
    let filename = format!("{model}_{stage}_{field}.npy");
    let path = reference_path(&filename);
    parse_npy(&path).unwrap_or_else(|| panic!("failed to load reference: {}", path.display()))
}

/// Load reference .npy int32 data.
#[allow(dead_code)]
pub fn load_reference_i32(model: &str, stage: &str, field: &str) -> (Vec<usize>, Vec<i32>) {
    let filename = format!("{model}_{stage}_{field}.npy");
    let path = reference_path(&filename);
    parse_npy_i32(&path).unwrap_or_else(|| panic!("failed to load reference: {}", path.display()))
}

/// Compare f64 arrays element-wise with diagnostic output on failure.
///
/// On mismatch, prints: `[{model}] {stage}.{field}[{index}]: expected {expected},
/// got {actual}, diff {diff}, tol {tol}`
#[allow(dead_code)]
pub fn assert_array_eq(
    model_name: &str,
    stage: &str,
    field: &str,
    expected: &[f64],
    actual: &[f64],
    tol: f64,
) {
    assert_eq!(
        expected.len(),
        actual.len(),
        "[{model_name}] {stage}.{field}: length mismatch: \
         expected {}, got {}",
        expected.len(),
        actual.len()
    );
    for (i, (e, a)) in expected.iter().zip(actual.iter()).enumerate() {
        let diff = (e - a).abs();
        assert!(
            diff <= tol,
            "[{model_name}] {stage}.{field}[{i}]: \
             expected {e:.15e}, got {a:.15e}, \
             diff {diff:.3e}, tol {tol:.3e}"
        );
    }
}

/// Compare quaternions with sign ambiguity handling (q ≡ -q).
///
/// Uses L∞ norm: `min(max|q_cf - q_ref|, max|q_cf + q_ref|) < tol`.
#[allow(dead_code)]
pub fn assert_quat_eq(
    model_name: &str,
    body_idx: usize,
    q_ref: [f64; 4], // [w, x, y, z] from MuJoCo reference
    q_cf: &nalgebra::UnitQuaternion<f64>,
    tol: f64,
) {
    let cf = [q_cf.w, q_cf.i, q_cf.j, q_cf.k];
    let dist_pos = (0..4)
        .map(|k| (cf[k] - q_ref[k]).abs())
        .fold(0.0f64, f64::max);
    let dist_neg = (0..4)
        .map(|k| (cf[k] + q_ref[k]).abs())
        .fold(0.0f64, f64::max);
    let dist = dist_pos.min(dist_neg);
    assert!(
        dist <= tol,
        "[{model_name}] fk.xquat[body {body_idx}]: \
         expected [{:.15e}, {:.15e}, {:.15e}, {:.15e}], \
         got [{:.15e}, {:.15e}, {:.15e}, {:.15e}], \
         min L∞ dist {dist:.3e}, tol {tol:.3e}",
        q_ref[0],
        q_ref[1],
        q_ref[2],
        q_ref[3],
        cf[0],
        cf[1],
        cf[2],
        cf[3]
    );
}
