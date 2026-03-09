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
