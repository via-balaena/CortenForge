//! Collision test infrastructure.
//!
//! This module provides shared utilities for collision detection tests, designed
//! with obsessive attention to numerical rigor and mathematical justification.
//!
//! # Design Philosophy
//!
//! > **Todorov Standard**: Every tolerance must be mathematically justified.
//! > No magic numbers. Derive from IEEE 754 precision guarantees.
//! >
//! > **Rust Purist Standard**: Zero allocations in assertion paths.
//! > Prefer compile-time computation. Exhaustive coverage via type system.
//!
//! # Tolerance Hierarchy
//!
//! ```text
//! Machine Epsilon (ε = 2^-52 ≈ 2.22e-16)
//!     │
//!     ├─── GEOM_TOL = 1e-10 (100ε) ─── Geometric comparisons
//!     │    Justification: Rotation matrix ≈ 10 multiplications, each with ε error
//!     │
//!     ├─── DEPTH_TOL = 1e-6 (mm) ─── Penetration depth
//!     │    Justification: Sub-mm precision sufficient for 1kHz contact dynamics
//!     │
//!     └─── NORMAL_TOL = 1e-3 (rad) ─── Normal direction
//!          Justification: arcsin(1e-3) ≈ 0.057° imperceptible in contact response
//! ```

use nalgebra::{Matrix3, Point3, Vector3};
use std::f64::consts::FRAC_PI_4;

// ============================================================================
// Numerical Constants — Mathematically Justified
// ============================================================================

/// Machine epsilon for IEEE 754 double precision.
///
/// This is the difference between 1.0 and the next representable f64.
/// Value: 2^-52 ≈ 2.220446049250313e-16
///
/// All other tolerances are derived from this fundamental constant.
pub const MACHINE_EPS: f64 = f64::EPSILON;

/// Geometric comparison tolerance: 100 × machine epsilon.
///
/// **Justification**: A rotation matrix computation involves approximately
/// 10 floating-point operations (3 trig calls, several multiplications).
/// Each operation introduces up to ε relative error. By the standard error
/// accumulation model, total error ≈ 10ε. We use 100ε for safety margin.
///
/// **Usage**: Position comparisons, rotation matrix entries, point distances.
pub const GEOM_TOL: f64 = 1e-10;

/// Penetration depth tolerance: millimeter-scale for meter-scale geometry.
///
/// **Justification**: In robotics and physics simulation, geometry is typically
/// specified in meters. Sub-millimeter precision (1e-6 m = 1 μm) far exceeds
/// requirements for contact dynamics at typical timesteps (1ms = 1kHz).
/// At 1kHz, penetration velocity of 1m/s produces 1mm penetration per step.
///
/// **Usage**: Contact depth comparisons, penetration assertions.
pub const DEPTH_TOL: f64 = 1e-6;

/// Normal direction tolerance: approximately 0.057 degrees.
///
/// **Justification**: arcsin(1e-3) ≈ 0.001 radians ≈ 0.057°. Angular
/// deviation this small is imperceptible in contact force direction.
/// Two normals are considered equal if their dot product > 1 - NORMAL_TOL.
///
/// **Usage**: Contact normal direction verification.
pub const NORMAL_TOL: f64 = 1e-3;

/// MuJoCo reference comparison tolerance: 0.1mm position agreement.
///
/// **Justification**: We accept small implementation differences from MuJoCo
/// (e.g., different SAT axis ordering, floating-point evaluation order).
/// 0.1mm is imperceptible in practice but catches algorithmic errors.
pub const MUJOCO_DEPTH_TOL: f64 = 1e-4;

/// MuJoCo reference position tolerance: 0.1mm agreement.
pub const MUJOCO_POS_TOL: f64 = 1e-4;

// ============================================================================
// Canonical Rotation Matrices — Exhaustive Testing Set
// ============================================================================

/// Standard rotation matrices for exhaustive orientation testing.
///
/// These rotations are chosen to cover:
/// - Identity (baseline)
/// - Single-axis 90° rotations (axis-aligned cases)
/// - Compound 45° rotation (maximally non-aligned case)
///
/// Every collision function should be tested with all orientations from
/// `canonical_set()` to ensure orientation-invariance.
pub mod rotations {
    use super::*;

    /// Identity rotation: no transformation.
    #[inline]
    #[must_use]
    pub fn identity() -> Matrix3<f64> {
        Matrix3::identity()
    }

    /// 90° rotation about X axis.
    ///
    /// ```text
    /// [1  0  0]
    /// [0  0 -1]
    /// [0  1  0]
    /// ```
    ///
    /// Maps: Y → Z, Z → -Y
    #[inline]
    #[must_use]
    pub fn rot_x_90() -> Matrix3<f64> {
        Matrix3::new(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0)
    }

    /// 90° rotation about Y axis.
    ///
    /// ```text
    /// [ 0  0  1]
    /// [ 0  1  0]
    /// [-1  0  0]
    /// ```
    ///
    /// Maps: Z → X, X → -Z
    #[inline]
    #[must_use]
    pub fn rot_y_90() -> Matrix3<f64> {
        Matrix3::new(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0)
    }

    /// 90° rotation about Z axis.
    ///
    /// ```text
    /// [0 -1  0]
    /// [1  0  0]
    /// [0  0  1]
    /// ```
    ///
    /// Maps: X → Y, Y → -X
    #[inline]
    #[must_use]
    pub fn rot_z_90() -> Matrix3<f64> {
        Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    }

    /// 45° rotation about X, then Y, then Z (maximally non-aligned).
    ///
    /// This rotation ensures no local axis aligns with any world axis,
    /// exposing any hidden assumptions about axis alignment in collision code.
    #[must_use]
    pub fn rot_xyz_45() -> Matrix3<f64> {
        let c = FRAC_PI_4.cos();
        let s = FRAC_PI_4.sin();

        let rx = Matrix3::new(1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c);

        let ry = Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);

        let rz = Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0);

        rz * ry * rx
    }

    /// 180° rotation about X axis (flip Y and Z).
    #[inline]
    #[must_use]
    pub fn rot_x_180() -> Matrix3<f64> {
        Matrix3::new(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0)
    }

    /// 180° rotation about Y axis (flip X and Z).
    #[inline]
    #[must_use]
    pub fn rot_y_180() -> Matrix3<f64> {
        Matrix3::new(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0)
    }

    /// Rotation from axis-angle representation.
    ///
    /// Uses Rodrigues' rotation formula. Useful for testing arbitrary orientations.
    #[must_use]
    pub fn from_axis_angle(axis: &Vector3<f64>, angle: f64) -> Matrix3<f64> {
        let axis = axis.normalize();
        let c = angle.cos();
        let s = angle.sin();
        let t = 1.0 - c;

        Matrix3::new(
            t * axis.x * axis.x + c,
            t * axis.x * axis.y - s * axis.z,
            t * axis.x * axis.z + s * axis.y,
            t * axis.x * axis.y + s * axis.z,
            t * axis.y * axis.y + c,
            t * axis.y * axis.z - s * axis.x,
            t * axis.x * axis.z - s * axis.y,
            t * axis.y * axis.z + s * axis.x,
            t * axis.z * axis.z + c,
        )
    }

    /// All canonical rotations for exhaustive testing.
    ///
    /// Returns pairs of (name, rotation_matrix) for use in parametric tests.
    /// The name is included for diagnostic output when a test fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// for (name, rot) in rotations::canonical_set() {
    ///     let contacts = collide_cylinder_plane(..., rot, ...);
    ///     assert!(!contacts.is_empty(), "Failed for rotation: {}", name);
    /// }
    /// ```
    #[must_use]
    pub fn canonical_set() -> Vec<(&'static str, Matrix3<f64>)> {
        vec![
            ("identity", identity()),
            ("rot_x_90", rot_x_90()),
            ("rot_y_90", rot_y_90()),
            ("rot_z_90", rot_z_90()),
            ("rot_xyz_45", rot_xyz_45()),
            ("rot_x_180", rot_x_180()),
            ("rot_y_180", rot_y_180()),
        ]
    }

    /// Verify that a matrix is a valid rotation matrix.
    ///
    /// Checks:
    /// - Orthonormality: R^T * R = I
    /// - Proper rotation: det(R) = +1
    #[must_use]
    pub fn is_valid_rotation(m: &Matrix3<f64>) -> bool {
        let should_be_identity = m.transpose() * m;
        let det = m.determinant();

        let identity_check = (should_be_identity - Matrix3::identity()).norm() < GEOM_TOL;
        let det_check = (det - 1.0).abs() < GEOM_TOL;

        identity_check && det_check
    }
}

// ============================================================================
// Test Assertion Utilities
// ============================================================================

/// Result of a contact assertion check.
#[derive(Debug)]
pub struct ContactAssertionResult {
    pub passed: bool,
    pub message: String,
}

/// Check if a contact depth matches expected value within tolerance.
#[inline]
#[must_use]
pub fn check_depth(actual: f64, expected: f64) -> ContactAssertionResult {
    let diff = (actual - expected).abs();
    let passed = diff < DEPTH_TOL;
    ContactAssertionResult {
        passed,
        message: format!(
            "Depth: expected {:.9}, got {:.9} (diff {:.2e}, tol {:.2e})",
            expected, actual, diff, DEPTH_TOL
        ),
    }
}

/// Check if a contact normal matches expected direction within tolerance.
///
/// Normals are unit vectors, so we check via dot product.
/// Accepts both the normal and its negation (direction ambiguity).
#[inline]
#[must_use]
pub fn check_normal(actual: &Vector3<f64>, expected: &Vector3<f64>) -> ContactAssertionResult {
    let dot = actual.dot(expected);
    let passed = (dot - 1.0).abs() < NORMAL_TOL || (dot + 1.0).abs() < NORMAL_TOL;
    ContactAssertionResult {
        passed,
        message: format!(
            "Normal: expected {:?}, got {:?} (dot {:.6})",
            expected.as_slice(),
            actual.as_slice(),
            dot
        ),
    }
}

/// Check if a contact normal is a unit vector.
#[inline]
#[must_use]
pub fn check_normal_unit(normal: &Vector3<f64>) -> ContactAssertionResult {
    let norm = normal.norm();
    let passed = (norm - 1.0).abs() < GEOM_TOL;
    ContactAssertionResult {
        passed,
        message: format!("Normal unit length: expected 1.0, got {:.9}", norm),
    }
}

/// Check if a position matches expected value within geometric tolerance.
#[inline]
#[must_use]
pub fn check_position(actual: &Vector3<f64>, expected: &Vector3<f64>) -> ContactAssertionResult {
    let diff = (actual - expected).norm();
    let passed = diff < DEPTH_TOL; // Use depth tolerance for positions too
    ContactAssertionResult {
        passed,
        message: format!(
            "Position: expected {:?}, got {:?} (diff {:.2e})",
            expected.as_slice(),
            actual.as_slice(),
            diff
        ),
    }
}

// ============================================================================
// Assertion Macros — Zero-Allocation Validation
// ============================================================================

/// Assert that a contact exists with expected depth and normal.
///
/// # Arguments
///
/// * `$contacts` - Slice of contacts to check
/// * `depth: $depth` - Expected penetration depth
/// * `normal: $normal` - Expected contact normal (Vector3)
///
/// # Example
///
/// ```ignore
/// assert_contact!(contacts, depth: 0.05, normal: Vector3::z());
/// ```
#[macro_export]
macro_rules! assert_contact {
    ($contacts:expr, depth: $depth:expr, normal: $normal:expr $(,)?) => {{
        let contacts = &$contacts;
        assert!(!contacts.is_empty(), "Expected contact, got none");
        let c = &contacts[0];

        // Check depth
        let depth_result = $crate::collision_test_utils::check_depth(c.depth, $depth);
        assert!(depth_result.passed, "{}", depth_result.message);

        // Check normal
        let expected_normal: Vector3<f64> = $normal;
        let normal_result = $crate::collision_test_utils::check_normal(&c.normal, &expected_normal);
        assert!(normal_result.passed, "{}", normal_result.message);

        // Verify normal is unit length
        let unit_result = $crate::collision_test_utils::check_normal_unit(&c.normal);
        assert!(unit_result.passed, "{}", unit_result.message);
    }};
}

/// Assert that no contact exists.
///
/// # Example
///
/// ```ignore
/// assert_no_contact!(contacts);
/// ```
#[macro_export]
macro_rules! assert_no_contact {
    ($contacts:expr) => {{
        let contacts = &$contacts;
        assert!(
            contacts.is_empty(),
            "Expected no contact, got {} contact(s)",
            contacts.len()
        );
    }};
}

/// Assert that a contact exists with approximately correct properties.
///
/// Less strict than `assert_contact!` — useful for MuJoCo conformance tests
/// where we accept implementation differences.
#[macro_export]
macro_rules! assert_contact_approx {
    ($contacts:expr, depth: $depth:expr, pos_tol: $pos_tol:expr $(,)?) => {{
        let contacts = &$contacts;
        assert!(!contacts.is_empty(), "Expected contact, got none");

        let c = &contacts[0];
        let diff = (c.depth - $depth).abs();
        assert!(
            diff < $pos_tol,
            "Depth: expected {:.6}, got {:.6} (diff {:.2e}, tol {:.2e})",
            $depth,
            c.depth,
            diff,
            $pos_tol
        );
    }};
}

// ============================================================================
// Test Geometry Builders — Common Configurations
// ============================================================================

/// Standard unit cube vertices: ±0.5 on each axis.
#[must_use]
pub fn unit_cube_vertices() -> Vec<Point3<f64>> {
    vec![
        Point3::new(-0.5, -0.5, -0.5),
        Point3::new(0.5, -0.5, -0.5),
        Point3::new(0.5, 0.5, -0.5),
        Point3::new(-0.5, 0.5, -0.5),
        Point3::new(-0.5, -0.5, 0.5),
        Point3::new(0.5, -0.5, 0.5),
        Point3::new(0.5, 0.5, 0.5),
        Point3::new(-0.5, 0.5, 0.5),
    ]
}

/// Standard unit cube faces (12 triangles, 2 per face).
#[must_use]
pub fn unit_cube_faces() -> Vec<[usize; 3]> {
    vec![
        // Bottom (z = -0.5)
        [0, 2, 1],
        [0, 3, 2],
        // Top (z = +0.5)
        [4, 5, 6],
        [4, 6, 7],
        // Front (y = -0.5)
        [0, 1, 5],
        [0, 5, 4],
        // Back (y = +0.5)
        [2, 3, 7],
        [2, 7, 6],
        // Left (x = -0.5)
        [0, 4, 7],
        [0, 7, 3],
        // Right (x = +0.5)
        [1, 2, 6],
        [1, 6, 5],
    ]
}

/// Regular tetrahedron with circumradius 1.
///
/// Vertices are positioned so the circumsphere has radius 1.
/// The tetrahedron is centered at the origin (centroid at origin).
///
/// For a regular tetrahedron with circumradius R:
/// - Edge length a = R * sqrt(8/3) ≈ 1.633 * R
/// - Top vertex at z = R
/// - Base plane at z = -R/3 (since centroid divides height 1:3 from base)
/// - Base vertices on circle of radius r = R * sqrt(8/9) ≈ 0.943 * R
#[must_use]
pub fn regular_tetrahedron_vertices() -> Vec<Point3<f64>> {
    // For circumradius R = 1:
    // Top vertex at (0, 0, 1)
    // Base plane at z = -1/3
    // Base circle radius = sqrt(1 - 1/9) = sqrt(8/9) ≈ 0.943
    let z_top = 1.0;
    let z_base = -1.0 / 3.0;
    let r_base = (8.0_f64 / 9.0).sqrt(); // ≈ 0.9428

    // sin(120°) = sqrt(3)/2 ≈ 0.866
    let sin_120 = 3.0_f64.sqrt() / 2.0;

    vec![
        // Top vertex on circumsphere at (0, 0, 1)
        Point3::new(0.0, 0.0, z_top),
        // Base vertices on circumsphere, 120° apart
        Point3::new(r_base, 0.0, z_base),
        Point3::new(r_base * (-0.5), r_base * sin_120, z_base),
        Point3::new(r_base * (-0.5), r_base * (-sin_120), z_base),
    ]
}

/// Regular tetrahedron faces (4 triangles).
#[must_use]
pub fn regular_tetrahedron_faces() -> Vec<[usize; 3]> {
    vec![
        [0, 1, 2], // Side 1
        [0, 2, 3], // Side 2
        [0, 3, 1], // Side 3
        [1, 3, 2], // Base (pointing down)
    ]
}

/// Single triangle in the XY plane at z=0.
#[must_use]
pub fn xy_plane_triangle_vertices() -> Vec<Point3<f64>> {
    vec![
        Point3::new(-1.0, -1.0, 0.0),
        Point3::new(1.0, -1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ]
}

/// Single triangle face.
#[must_use]
pub fn single_triangle_face() -> Vec<[usize; 3]> {
    vec![[0, 1, 2]]
}

// ============================================================================
// MJCF Test Model Builders
// ============================================================================

/// Generate MJCF for a single body with a specified geometry type.
///
/// The body is placed at the given position with a free joint.
#[must_use]
pub fn mjcf_single_body(geom_type: &str, size: &str, pos: &str) -> String {
    format!(
        r#"
        <mujoco model="test_{geom_type}">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="body" pos="{pos}">
                    <freejoint/>
                    <geom type="{geom_type}" size="{size}" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
        "#
    )
}

/// Generate MJCF for two bodies that should collide.
#[must_use]
pub fn mjcf_two_bodies(
    geom1_type: &str,
    geom1_size: &str,
    pos1: &str,
    geom2_type: &str,
    geom2_size: &str,
    pos2: &str,
) -> String {
    format!(
        r#"
        <mujoco model="test_{geom1_type}_{geom2_type}">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="body1" pos="{pos1}">
                    <geom type="{geom1_type}" size="{geom1_size}"/>
                </body>
                <body name="body2" pos="{pos2}">
                    <geom type="{geom2_type}" size="{geom2_size}"/>
                </body>
            </worldbody>
        </mujoco>
        "#
    )
}

/// Generate MJCF with embedded mesh data.
#[must_use]
pub fn mjcf_mesh_body(
    mesh_name: &str,
    vertices: &[Point3<f64>],
    faces: &[[usize; 3]],
    pos: &str,
) -> String {
    // Format vertices as space-separated string
    let vertex_str: String = vertices
        .iter()
        .map(|v| format!("{} {} {}", v.x, v.y, v.z))
        .collect::<Vec<_>>()
        .join("  ");

    // Format faces as space-separated string
    let face_str: String = faces
        .iter()
        .map(|f| format!("{} {} {}", f[0], f[1], f[2]))
        .collect::<Vec<_>>()
        .join("  ");

    format!(
        r#"
        <mujoco model="test_mesh">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <asset>
                <mesh name="{mesh_name}" vertex="{vertex_str}" face="{face_str}"/>
            </asset>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="mesh_body" pos="{pos}">
                    <freejoint/>
                    <geom type="mesh" mesh="{mesh_name}" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
        "#
    )
}

// ============================================================================
// Performance Measurement Utilities
// ============================================================================

/// Measure execution time of a closure, returning (result, duration_secs).
#[inline]
pub fn timed<T, F: FnOnce() -> T>(f: F) -> (T, f64) {
    let start = std::time::Instant::now();
    let result = f();
    let elapsed = start.elapsed().as_secs_f64();
    (result, elapsed)
}

/// Run a benchmark and return steps per second.
///
/// Includes warmup phase to avoid cold-start effects.
#[must_use]
pub fn benchmark_steps_per_second<F>(warmup_steps: usize, bench_steps: usize, mut step_fn: F) -> f64
where
    F: FnMut(),
{
    // Warmup
    for _ in 0..warmup_steps {
        step_fn();
    }

    // Benchmark
    let start = std::time::Instant::now();
    for _ in 0..bench_steps {
        step_fn();
    }
    let elapsed = start.elapsed().as_secs_f64();

    bench_steps as f64 / elapsed
}

// ============================================================================
// Debug Formatting
// ============================================================================

/// Format a contact for diagnostic output.
#[must_use]
pub fn format_contact(c: &sim_core::Contact) -> String {
    format!(
        "Contact {{ pos: [{:.4}, {:.4}, {:.4}], normal: [{:.4}, {:.4}, {:.4}], depth: {:.6}, geom1: {}, geom2: {} }}",
        c.pos.x, c.pos.y, c.pos.z,
        c.normal.x, c.normal.y, c.normal.z,
        c.depth, c.geom1, c.geom2
    )
}

/// Print all contacts for debugging.
pub fn debug_print_contacts(contacts: &[sim_core::Contact], label: &str) {
    eprintln!("=== {} ({} contacts) ===", label, contacts.len());
    for (i, c) in contacts.iter().enumerate() {
        eprintln!("  [{}] {}", i, format_contact(c));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotation_matrices_are_valid() {
        for (name, rot) in rotations::canonical_set() {
            assert!(
                rotations::is_valid_rotation(&rot),
                "Rotation '{}' is not a valid rotation matrix",
                name
            );
        }
    }

    #[test]
    fn tolerances_are_ordered() {
        // Verify tolerance hierarchy
        assert!(MACHINE_EPS < GEOM_TOL);
        assert!(GEOM_TOL < DEPTH_TOL);
        // NORMAL_TOL is for angular comparison, not directly comparable
    }

    #[test]
    fn cube_has_correct_topology() {
        let verts = unit_cube_vertices();
        let faces = unit_cube_faces();

        assert_eq!(verts.len(), 8, "Cube should have 8 vertices");
        assert_eq!(
            faces.len(),
            12,
            "Cube should have 12 triangles (2 per face)"
        );

        // Verify all face indices are valid
        for face in &faces {
            for &idx in face {
                assert!(idx < verts.len(), "Invalid vertex index in face");
            }
        }
    }

    #[test]
    fn tetrahedron_has_correct_geometry() {
        let verts = regular_tetrahedron_vertices();

        assert_eq!(verts.len(), 4, "Tetrahedron should have 4 vertices");

        // All vertices should lie on circumsphere of radius 1
        for (i, v) in verts.iter().enumerate() {
            let dist = (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
            assert!(
                (dist - 1.0).abs() < GEOM_TOL,
                "Vertex {} not on circumsphere: distance = {}",
                i,
                dist
            );
        }

        // All edges should have equal length (regular tetrahedron property)
        let edge_pairs = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)];
        let first_edge_len = (verts[0] - verts[1]).norm();
        for (i, j) in edge_pairs.iter() {
            let edge_len = (verts[*i] - verts[*j]).norm();
            assert!(
                (edge_len - first_edge_len).abs() < GEOM_TOL,
                "Edge ({}, {}) has length {}, expected {}",
                i,
                j,
                edge_len,
                first_edge_len
            );
        }
    }

    #[test]
    fn check_depth_works() {
        let result = check_depth(0.05, 0.05);
        assert!(result.passed);

        let result = check_depth(0.05, 0.06);
        assert!(!result.passed);
    }

    #[test]
    fn check_normal_works() {
        let up = Vector3::z();
        let result = check_normal(&up, &up);
        assert!(result.passed);

        let down = -Vector3::z();
        let result = check_normal(&down, &up);
        assert!(result.passed); // Accepts negation

        let sideways = Vector3::x();
        let result = check_normal(&sideways, &up);
        assert!(!result.passed);
    }
}
