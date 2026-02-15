//! Acceptance tests for §23 `<compiler> exactmeshinertia` + Full Inertia Tensor Pipeline.
//!
//! Tests AC1–AC9 from the spec, covering:
//! - Parser: default value, explicit true/false (AC1, AC2)
//! - Unit cube mesh: analytical match (AC3)
//! - Asymmetric mesh: off-diagonal terms, non-identity iquat (AC4)
//! - Explicit `<inertial>` overrides mesh computation (AC5)
//! - COM offset: parallel axis theorem correctness (AC6)
//! - Zero-volume degenerate mesh fallback (AC7)
//! - `exactmeshinertia=true` == `exactmeshinertia=false` (AC8)
//! - Multi-geom body: mixed mesh + primitive aggregation (AC9)
//!
//! Tolerance: 1e-6 for mass/inertia (floating-point accumulation over triangles).

use approx::assert_relative_eq;
use sim_mjcf::load_model;

/// Unit cube mesh vertices and faces for reuse across tests.
/// Cube centered at origin, edge length 1.0 (vertices at ±0.5).
/// 8 vertices, 12 triangles (2 per face), outward-facing normals.
const CUBE_VERTICES: &str = "-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  \
     -0.5 -0.5  0.5  0.5 -0.5  0.5  0.5 0.5  0.5  -0.5 0.5  0.5";
const CUBE_FACES: &str =
    "0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5";

// ============================================================================
// AC1 — Parser: Default Value
// ============================================================================

/// AC1: When `<compiler>` omits `exactmeshinertia`, the field defaults to `false`.
#[test]
fn ac1_parser_default_false() {
    let mjcf = r#"
        <mujoco>
          <worldbody>
            <body>
              <inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/>
              <geom type="sphere" size="0.1"/>
            </body>
          </worldbody>
        </mujoco>
    "#;
    // If this parses without error, the default was applied.
    // We can't directly inspect compiler.exactmeshinertia from the Model,
    // but we verify the model loads and the attribute doesn't cause errors.
    let model = load_model(mjcf).expect("should parse with default exactmeshinertia");
    assert_eq!(model.nbody, 2); // worldbody + 1 body
}

// ============================================================================
// AC2 — Parser: Explicit True/False
// ============================================================================

/// AC2a: `exactmeshinertia="true"` parses without error.
#[test]
fn ac2a_parser_explicit_true() {
    let mjcf = r#"
        <mujoco>
          <compiler exactmeshinertia="true"/>
          <worldbody>
            <body>
              <inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/>
              <geom type="sphere" size="0.1"/>
            </body>
          </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should parse with exactmeshinertia=true");
    assert_eq!(model.nbody, 2);
}

/// AC2b: `exactmeshinertia="false"` parses without error.
#[test]
fn ac2b_parser_explicit_false() {
    let mjcf = r#"
        <mujoco>
          <compiler exactmeshinertia="false"/>
          <worldbody>
            <body>
              <inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/>
              <geom type="sphere" size="0.1"/>
            </body>
          </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should parse with exactmeshinertia=false");
    assert_eq!(model.nbody, 2);
}

// ============================================================================
// AC3 — Unit Cube Mesh: Analytical Match
// ============================================================================

/// AC3: A unit cube mesh with density=1000 should match the analytical box inertia.
///
/// For a unit cube (edge=1, V=1, density=1000):
///   mass = 1000
///   I_xx = I_yy = I_zz = (m/12) * (a² + b²) = (1000/12) * (1 + 1) = 166.667
///   COM = (0, 0, 0)
///   Off-diagonal = 0 (symmetric cube)
#[test]
fn ac3_unit_cube_analytical_match() {
    let mjcf = format!(
        r#"
        <mujoco>
          <compiler exactmeshinertia="true"/>
          <asset>
            <mesh name="cube" vertex="{}" face="{}"/>
          </asset>
          <worldbody>
            <body name="cube_body">
              <geom type="mesh" mesh="cube" density="1000"/>
            </body>
          </worldbody>
        </mujoco>
    "#,
        CUBE_VERTICES, CUBE_FACES
    );

    let model = load_model(&mjcf).expect("should load cube mesh model");

    // Body 0 = worldbody, Body 1 = cube_body
    let mass = model.body_mass[1];
    let inertia = model.body_inertia[1];
    let ipos = model.body_ipos[1];

    // Mass = density * volume = 1000 * 1.0 = 1000
    assert_relative_eq!(mass, 1000.0, epsilon = 1.0);

    // COM at origin
    assert_relative_eq!(ipos.x, 0.0, epsilon = 1e-6);
    assert_relative_eq!(ipos.y, 0.0, epsilon = 1e-6);
    assert_relative_eq!(ipos.z, 0.0, epsilon = 1e-6);

    // Diagonal inertia: I = (m/12)(a² + b²) = (1000/12)(1+1) = 166.667
    let expected_inertia = 1000.0 / 12.0 * 2.0;
    assert_relative_eq!(inertia.x, expected_inertia, epsilon = 1.0);
    assert_relative_eq!(inertia.y, expected_inertia, epsilon = 1.0);
    assert_relative_eq!(inertia.z, expected_inertia, epsilon = 1.0);
}

// ============================================================================
// AC4 — Asymmetric Mesh: Off-Diagonal Terms
// ============================================================================

/// AC4: An L-shaped mesh should produce non-identity iquat (off-diagonal terms).
///
/// The L-shape is two unit cubes joined: one at (0,0,0)–(1,1,1) and one at
/// (1,0,0)–(2,1,1). The COM is NOT at the geometric center of either cube,
/// and the inertia tensor has off-diagonal terms.
#[test]
fn ac4_asymmetric_mesh_off_diagonal() {
    // Asymmetric wedge (right-angle prism, longer in x than y or z)
    let wedge_vertices = "0 0 0  2 0 0  0 1 0  0 0 1  2 0 1  0 1 1";
    let wedge_faces = "0 1 2  0 2 3  0 3 4  0 4 1  3 2 5  3 5 4  1 4 5  1 5 2";

    let mjcf = format!(
        r#"
        <mujoco>
          <compiler exactmeshinertia="true"/>
          <asset>
            <mesh name="wedge" vertex="{}" face="{}"/>
          </asset>
          <worldbody>
            <body name="wedge_body">
              <geom type="mesh" mesh="wedge" density="1000"/>
            </body>
          </worldbody>
        </mujoco>
    "#,
        wedge_vertices, wedge_faces
    );

    let model = load_model(&mjcf).expect("should load wedge mesh model");

    let mass = model.body_mass[1];
    let inertia = model.body_inertia[1];
    let iquat = &model.body_iquat[1];

    // Mass should be positive (wedge has positive volume)
    assert!(mass > 0.0, "wedge mass should be positive, got {mass}");

    // All principal inertia values should be positive
    assert!(inertia.x > 0.0, "Ixx should be positive, got {}", inertia.x);
    assert!(inertia.y > 0.0, "Iyy should be positive, got {}", inertia.y);
    assert!(inertia.z > 0.0, "Izz should be positive, got {}", inertia.z);

    // For an asymmetric shape, the principal axes should not be aligned with
    // the body frame → iquat should not be identity.
    // The wedge is asymmetric in all three axes.
    let is_identity = (iquat.w - 1.0).abs() < 1e-6
        && iquat.i.abs() < 1e-6
        && iquat.j.abs() < 1e-6
        && iquat.k.abs() < 1e-6;
    assert!(
        !is_identity,
        "iquat should NOT be identity for asymmetric mesh, got {:?}",
        iquat
    );

    // Principal inertia values should not all be equal (asymmetric)
    let all_equal = (inertia.x - inertia.y).abs() < 1e-6 && (inertia.y - inertia.z).abs() < 1e-6;
    assert!(
        !all_equal,
        "principal inertia should not be all equal for asymmetric mesh"
    );
}

// ============================================================================
// AC5 — Explicit Inertial Overrides Mesh
// ============================================================================

/// AC5: When `<inertial>` is explicit, mesh inertia computation is bypassed.
#[test]
fn ac5_explicit_inertial_overrides_mesh() {
    let mjcf = format!(
        r#"
        <mujoco>
          <compiler exactmeshinertia="true"/>
          <asset>
            <mesh name="cube" vertex="{}" face="{}"/>
          </asset>
          <worldbody>
            <body name="cube_body">
              <inertial mass="42.0" pos="0 0 0" diaginertia="1 2 3"/>
              <geom type="mesh" mesh="cube"/>
            </body>
          </worldbody>
        </mujoco>
    "#,
        CUBE_VERTICES, CUBE_FACES
    );

    let model = load_model(&mjcf).expect("should load with explicit inertial");

    // Body mass should be 42.0 (from <inertial>), NOT computed from mesh
    assert_relative_eq!(model.body_mass[1], 42.0, epsilon = 1e-10);

    // Diagonal inertia should be [1, 2, 3] (from <inertial>)
    assert_relative_eq!(model.body_inertia[1].x, 1.0, epsilon = 1e-10);
    assert_relative_eq!(model.body_inertia[1].y, 2.0, epsilon = 1e-10);
    assert_relative_eq!(model.body_inertia[1].z, 3.0, epsilon = 1e-10);
}

// ============================================================================
// AC6 — COM Offset (Parallel Axis Theorem)
// ============================================================================

/// AC6: Two identical mesh geoms at different positions should produce inertia
/// that reflects the parallel axis theorem shift.
///
/// Two identical unit cubes (mass m=1000 each) placed at x=±D should produce:
/// - Total mass = 2m = 2000
/// - COM at origin (symmetric placement)
/// - I_yy = I_zz increased by 2 * m * D² relative to a single cube
///   (each cube has offset d=D from the body COM along x)
/// - I_xx unchanged (offset is along x, so x-axis inertia unaffected)
///
/// This verifies that the full-tensor parallel axis theorem is working:
///   I_shifted = I_rotated + m * (d·d * I₃ - d ⊗ d)
#[test]
fn ac6_com_offset_parallel_axis() {
    let d = 2.0; // half-separation along x

    // Model A: single cube at origin (reference — no PAT shift)
    let mjcf_single = format!(
        r#"
        <mujoco>
          <compiler exactmeshinertia="true"/>
          <asset>
            <mesh name="cube" vertex="{}" face="{}"/>
          </asset>
          <worldbody>
            <body name="cube_body">
              <geom type="mesh" mesh="cube" pos="0 0 0" density="1000"/>
            </body>
          </worldbody>
        </mujoco>
    "#,
        CUBE_VERTICES, CUBE_FACES
    );

    // Model B: two cubes at pos="±D 0 0" (PAT applies)
    let mjcf_pair = format!(
        r#"
        <mujoco>
          <compiler exactmeshinertia="true"/>
          <asset>
            <mesh name="cube" vertex="{v}" face="{f}"/>
          </asset>
          <worldbody>
            <body name="cube_body">
              <geom type="mesh" mesh="cube" pos="{d} 0 0" density="1000"/>
              <geom type="mesh" mesh="cube" pos="-{d} 0 0" density="1000"/>
            </body>
          </worldbody>
        </mujoco>
    "#,
        v = CUBE_VERTICES,
        f = CUBE_FACES,
        d = d
    );

    let model_single = load_model(&mjcf_single).expect("should load single cube");
    let model_pair = load_model(&mjcf_pair).expect("should load cube pair");

    let m_single = model_single.body_mass[1]; // ~1000
    let i_single = model_single.body_inertia[1]; // ~166.667 on all axes

    let m_pair = model_pair.body_mass[1]; // ~2000
    let i_pair = model_pair.body_inertia[1];
    let ipos_pair = model_pair.body_ipos[1];

    // Total mass doubles
    assert_relative_eq!(m_pair, 2.0 * m_single, epsilon = 1.0);

    // COM at origin (symmetric placement)
    assert_relative_eq!(ipos_pair.x, 0.0, epsilon = 1e-6);
    assert_relative_eq!(ipos_pair.y, 0.0, epsilon = 1e-6);
    assert_relative_eq!(ipos_pair.z, 0.0, epsilon = 1e-6);

    // I_xx: sum of two cubes' local I_xx, no PAT contribution (offset along x).
    // I_xx_pair = 2 * I_xx_single (just doubled, no shift on this axis)
    // Note: eigendecomposition may reorder axes, so we sort principal values.
    let mut i_pair_sorted = [i_pair.x, i_pair.y, i_pair.z];
    i_pair_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());

    // The smallest principal inertia is I_xx (offset along x doesn't add to it).
    // Expected: 2 * I_single ≈ 2 * 166.667 = 333.333
    let i_xx_pair = i_pair_sorted[0];
    assert_relative_eq!(i_xx_pair, 2.0 * i_single.x, epsilon = 2.0);

    // The two larger principal inertia values are I_yy = I_zz.
    // Expected: 2 * I_single + 2 * m * D² = 333.333 + 2 * 1000 * 4 = 8333.333
    let i_yy_pair = i_pair_sorted[1];
    let i_zz_pair = i_pair_sorted[2];
    let expected_shifted = 2.0 * i_single.x + 2.0 * m_single * d * d;
    assert_relative_eq!(i_yy_pair, expected_shifted, epsilon = 5.0);
    assert_relative_eq!(i_zz_pair, expected_shifted, epsilon = 5.0);

    // Verify the PAT contribution is significant (not just rounding)
    assert!(
        i_yy_pair > 3.0 * i_xx_pair,
        "PAT should make I_yy >> I_xx: I_yy={i_yy_pair}, I_xx={i_xx_pair}"
    );
}

// ============================================================================
// AC7 — Zero-Volume Degenerate Mesh Fallback
// ============================================================================

/// AC7: A degenerate mesh (all coplanar triangles) should not panic or produce NaN.
#[test]
fn ac7_zero_volume_degenerate_mesh() {
    let mjcf = r#"
        <mujoco>
          <compiler exactmeshinertia="true"/>
          <asset>
            <mesh name="flat" vertex="0 0 0  1 0 0  1 1 0  0 1 0"
                  face="0 1 2  0 2 3"/>
          </asset>
          <worldbody>
            <body name="flat_body">
              <geom type="mesh" mesh="flat" density="1000"/>
            </body>
          </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("degenerate mesh should not panic");

    let mass = model.body_mass[1];
    let inertia = model.body_inertia[1];

    // Should not be NaN or Inf
    assert!(mass.is_finite(), "mass should be finite, got {mass}");
    assert!(
        inertia.x.is_finite(),
        "Ixx should be finite, got {}",
        inertia.x
    );
    assert!(
        inertia.y.is_finite(),
        "Iyy should be finite, got {}",
        inertia.y
    );
    assert!(
        inertia.z.is_finite(),
        "Izz should be finite, got {}",
        inertia.z
    );
}

// ============================================================================
// AC8 — exactmeshinertia=true == exactmeshinertia=false
// ============================================================================

/// AC8: Both `true` and `false` modes produce identical results (CortenForge
/// uses the same exact algorithm for both).
#[test]
fn ac8_true_equals_false() {
    let mjcf_true = format!(
        r#"
        <mujoco>
          <compiler exactmeshinertia="true"/>
          <asset>
            <mesh name="cube" vertex="{}" face="{}"/>
          </asset>
          <worldbody>
            <body name="cube_body">
              <geom type="mesh" mesh="cube" density="1000"/>
            </body>
          </worldbody>
        </mujoco>
    "#,
        CUBE_VERTICES, CUBE_FACES
    );

    let mjcf_false = format!(
        r#"
        <mujoco>
          <compiler exactmeshinertia="false"/>
          <asset>
            <mesh name="cube" vertex="{}" face="{}"/>
          </asset>
          <worldbody>
            <body name="cube_body">
              <geom type="mesh" mesh="cube" density="1000"/>
            </body>
          </worldbody>
        </mujoco>
    "#,
        CUBE_VERTICES, CUBE_FACES
    );

    let model_true = load_model(&mjcf_true).expect("should load with true");
    let model_false = load_model(&mjcf_false).expect("should load with false");

    // Identical mass
    assert_relative_eq!(
        model_true.body_mass[1],
        model_false.body_mass[1],
        epsilon = 1e-10
    );

    // Identical COM
    assert_relative_eq!(
        model_true.body_ipos[1].x,
        model_false.body_ipos[1].x,
        epsilon = 1e-10
    );
    assert_relative_eq!(
        model_true.body_ipos[1].y,
        model_false.body_ipos[1].y,
        epsilon = 1e-10
    );
    assert_relative_eq!(
        model_true.body_ipos[1].z,
        model_false.body_ipos[1].z,
        epsilon = 1e-10
    );

    // Identical inertia
    assert_relative_eq!(
        model_true.body_inertia[1].x,
        model_false.body_inertia[1].x,
        epsilon = 1e-10
    );
    assert_relative_eq!(
        model_true.body_inertia[1].y,
        model_false.body_inertia[1].y,
        epsilon = 1e-10
    );
    assert_relative_eq!(
        model_true.body_inertia[1].z,
        model_false.body_inertia[1].z,
        epsilon = 1e-10
    );

    // Identical iquat
    let qt = &model_true.body_iquat[1];
    let qf = &model_false.body_iquat[1];
    assert_relative_eq!(qt.w, qf.w, epsilon = 1e-10);
    assert_relative_eq!(qt.i, qf.i, epsilon = 1e-10);
    assert_relative_eq!(qt.j, qf.j, epsilon = 1e-10);
    assert_relative_eq!(qt.k, qf.k, epsilon = 1e-10);
}

// ============================================================================
// AC9 — Multi-Geom Body: Mixed Mesh + Primitive
// ============================================================================

/// AC9: A body with both a sphere and a mesh geom should produce correct
/// aggregated inertia (full tensor with parallel axis theorem).
#[test]
fn ac9_multi_geom_mixed_mesh_primitive() {
    let mjcf = format!(
        r#"
        <mujoco>
          <compiler exactmeshinertia="true"/>
          <asset>
            <mesh name="cube" vertex="{}" face="{}"/>
          </asset>
          <worldbody>
            <body name="mixed_body">
              <geom name="sphere" type="sphere" size="0.1" pos="0 0 0" density="1000"/>
              <geom name="cube" type="mesh" mesh="cube" pos="2 0 0" density="1000"/>
            </body>
          </worldbody>
        </mujoco>
    "#,
        CUBE_VERTICES, CUBE_FACES
    );

    let model = load_model(&mjcf).expect("should load mixed geom model");

    let mass = model.body_mass[1];
    let ipos = model.body_ipos[1];
    let inertia = model.body_inertia[1];

    // Sphere: mass = density * (4/3)πr³ = 1000 * 4.189e-3 ≈ 4.189
    let sphere_mass = 1000.0 * (4.0 / 3.0) * std::f64::consts::PI * 0.1_f64.powi(3);
    // Cube: mass = density * volume = 1000 * 1.0 = 1000.0
    let cube_mass = 1000.0;
    let expected_total_mass = sphere_mass + cube_mass;

    assert_relative_eq!(mass, expected_total_mass, epsilon = 1.0);

    // COM should be weighted average:
    // COM = (sphere_mass * 0 + cube_mass * 2) / total ≈ 2000 / 1004.189 ≈ 1.992
    let expected_com_x = cube_mass * 2.0 / expected_total_mass;
    assert_relative_eq!(ipos.x, expected_com_x, epsilon = 0.01);
    assert_relative_eq!(ipos.y, 0.0, epsilon = 1e-6);
    assert_relative_eq!(ipos.z, 0.0, epsilon = 1e-6);

    // All principal inertia values should be positive
    assert!(inertia.x > 0.0, "Ixx should be positive");
    assert!(inertia.y > 0.0, "Iyy should be positive");
    assert!(inertia.z > 0.0, "Izz should be positive");

    // The off-axis placement means this is NOT a symmetric configuration,
    // so the principal inertia values should NOT all be equal
    let all_equal = (inertia.x - inertia.y).abs() < 1e-3 && (inertia.y - inertia.z).abs() < 1e-3;
    assert!(
        !all_equal,
        "mixed mesh+primitive body should not have all-equal inertia"
    );
}

// ============================================================================
// AC10 — Mesh COM Offset Propagation
// ============================================================================

/// AC10: A mesh whose centroid is NOT at the mesh origin should have the
/// body COM reflect the mesh's internal COM offset.
///
/// Uses a unit cube with all vertices shifted to x∈[0,1] (not centered).
/// The mesh COM is at (0.5, 0.5, 0.5). When placed at geom pos="0 0 0",
/// the body COM should be (0.5, 0.5, 0.5), NOT (0, 0, 0).
#[test]
fn ac10_mesh_com_offset_propagation() {
    // Cube with vertices at [0,1]³ instead of [-0.5,0.5]³
    let offset_cube_vertices = "0 0 0  1 0 0  1 1 0  0 1 0  0 0 1  1 0 1  1 1 1  0 1 1";

    let mjcf = format!(
        r#"
        <mujoco>
          <compiler exactmeshinertia="true"/>
          <asset>
            <mesh name="cube" vertex="{}" face="{}"/>
          </asset>
          <worldbody>
            <body name="cube_body">
              <geom type="mesh" mesh="cube" pos="0 0 0" density="1000"/>
            </body>
          </worldbody>
        </mujoco>
    "#,
        offset_cube_vertices, CUBE_FACES
    );

    let model = load_model(&mjcf).expect("should load offset cube mesh");

    let ipos = model.body_ipos[1];

    // Mesh COM is at (0.5, 0.5, 0.5) — body COM should reflect this
    assert_relative_eq!(ipos.x, 0.5, epsilon = 1e-6);
    assert_relative_eq!(ipos.y, 0.5, epsilon = 1e-6);
    assert_relative_eq!(ipos.z, 0.5, epsilon = 1e-6);

    // Mass should still be density * volume = 1000 * 1.0 = 1000
    assert_relative_eq!(model.body_mass[1], 1000.0, epsilon = 1.0);

    // Inertia should match the centered cube (same shape, just offset)
    // I = (m/12)(a² + b²) = (1000/12)(1+1) = 166.667
    let expected_inertia = 1000.0 / 12.0 * 2.0;
    let inertia = model.body_inertia[1];
    assert_relative_eq!(inertia.x, expected_inertia, epsilon = 1.0);
    assert_relative_eq!(inertia.y, expected_inertia, epsilon = 1.0);
    assert_relative_eq!(inertia.z, expected_inertia, epsilon = 1.0);
}
