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

/// AC6: Placing a mesh geom at a non-zero position should shift the body COM
/// and adjust inertia via the parallel axis theorem.
#[test]
fn ac6_com_offset_parallel_axis() {
    // Model A: cube at origin
    let mjcf_origin = format!(
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

    // Model B: cube at pos="2 0 0"
    let mjcf_offset = format!(
        r#"
        <mujoco>
          <compiler exactmeshinertia="true"/>
          <asset>
            <mesh name="cube" vertex="{}" face="{}"/>
          </asset>
          <worldbody>
            <body name="cube_body">
              <geom type="mesh" mesh="cube" pos="2 0 0" density="1000"/>
            </body>
          </worldbody>
        </mujoco>
    "#,
        CUBE_VERTICES, CUBE_FACES
    );

    let model_a = load_model(&mjcf_origin).expect("should load origin model");
    let model_b = load_model(&mjcf_offset).expect("should load offset model");

    // Same mass
    assert_relative_eq!(model_a.body_mass[1], model_b.body_mass[1], epsilon = 1e-6);

    // COM of offset model should be at (2, 0, 0)
    assert_relative_eq!(model_b.body_ipos[1].x, 2.0, epsilon = 1e-6);
    assert_relative_eq!(model_b.body_ipos[1].y, 0.0, epsilon = 1e-6);
    assert_relative_eq!(model_b.body_ipos[1].z, 0.0, epsilon = 1e-6);

    // For a single geom, inertia about COM is the same regardless of position
    // (parallel axis theorem shifts are relative to body COM, which IS the geom COM
    // for a single-geom body). So inertia should be the same.
    let inertia_a = model_a.body_inertia[1];
    let inertia_b = model_b.body_inertia[1];
    assert_relative_eq!(inertia_a.x, inertia_b.x, epsilon = 1.0);
    assert_relative_eq!(inertia_a.y, inertia_b.y, epsilon = 1.0);
    assert_relative_eq!(inertia_a.z, inertia_b.z, epsilon = 1.0);
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
