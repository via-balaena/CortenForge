//! Acceptance tests for §43 Mesh Inertia Modes (Spec B).
//!
//! Tests T1–T19 from the spec, covering:
//! - Default mode is Convex (T1)
//! - Shell mesh inertia: unit cube (T2), explicit mass override (T3)
//! - Primitive shell formulas: sphere (T4), box (T5), cylinder (T6),
//!   capsule (T7), ellipsoid (T8)
//! - Convex mode on non-convex mesh (T9)
//! - Legacy mode (T10)
//! - shellinertia rejected on mesh geom (T11)
//! - Default class inheritance for mesh inertia (T12)
//! - Ellipsoid solid volume fix (T13)
//! - exactmeshinertia deprecation (T14)
//! - Existing exact tests regression (T15 — covered by exactmeshinertia.rs)
//! - Degenerate mesh + shell mode (T16)
//! - Mixed shell/solid body (T17)
//! - Exact mode rejects misoriented mesh (T18)
//! - Default class inheritance for shellinertia (T19)

use approx::assert_relative_eq;
use sim_mjcf::load_model;

/// Unit cube mesh vertices and faces for reuse across tests.
/// Cube centered at origin, edge length 1.0 (vertices at ±0.5).
const CUBE_VERTICES: &str = "-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  \
     -0.5 -0.5  0.5  0.5 -0.5  0.5  0.5 0.5  0.5  -0.5 0.5  0.5";
const CUBE_FACES: &str =
    "0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5";

/// L-shape mesh for non-convex inertia tests.
/// An actual L-shape: bottom bar (0,0,0)→(3,1,1) + left column (0,1,0)→(1,3,1).
/// Exact volume = 3 + 2 = 5. Convex hull = bounding box (0,0,0)→(3,3,1), volume 9.
const L_SHAPE_VERTICES: &str = "\
    0 0 0  3 0 0  3 1 0  1 1 0  1 3 0  0 3 0  \
    0 0 1  3 0 1  3 1 1  1 1 1  1 3 1  0 3 1";
const L_SHAPE_FACES: &str = "\
    0 2 1  0 3 2  0 4 3  0 5 4  \
    6 7 8  6 8 9  6 9 10  6 10 11  \
    0 1 7  0 7 6  1 2 8  1 8 7  \
    2 3 9  2 9 8  3 4 10  3 10 9  \
    4 5 11  4 11 10  5 0 6  5 6 11";

/// Reversed winding cube for misorientation tests.
const REVERSED_CUBE_FACES: &str =
    "0 1 2  0 2 3  4 6 5  4 7 6  0 5 1  0 4 5  2 7 3  2 6 7  0 7 4  0 3 7  1 6 2  1 5 6";

// ============================================================================
// T1: Default mode is Convex → AC1
// ============================================================================

#[test]
fn t1_default_mode_is_convex() {
    let mjcf = format!(
        r#"<mujoco>
  <asset><mesh name="cube" vertex="{CUBE_VERTICES}" face="{CUBE_FACES}"/></asset>
  <worldbody><body>
    <geom type="mesh" mesh="cube" density="1000"/>
  </body></worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).unwrap();
    // Convex == Exact for convex meshes: mass = density × volume = 1000 × 1.0
    assert_relative_eq!(model.body_mass[1], 1000.0, epsilon = 1e-6);
    assert_relative_eq!(model.body_inertia[1].x, 166.667, epsilon = 0.01);
    assert_relative_eq!(model.body_inertia[1].y, 166.667, epsilon = 0.01);
    assert_relative_eq!(model.body_inertia[1].z, 166.667, epsilon = 0.01);
}

// ============================================================================
// T2: Shell mesh inertia — unit cube → AC2, AC16
// ============================================================================

#[test]
fn t2_shell_mesh_inertia_unit_cube() {
    let mjcf = format!(
        r#"<mujoco>
  <asset><mesh name="cube" inertia="shell" vertex="{CUBE_VERTICES}" face="{CUBE_FACES}"/></asset>
  <worldbody><body>
    <geom type="mesh" mesh="cube" density="1000"/>
  </body></worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).unwrap();
    // Shell: mass = density × surface_area = 1000 × 6.0 = 6000
    assert_relative_eq!(model.body_mass[1], 6000.0, epsilon = 1e-6);
    assert_relative_eq!(model.body_inertia[1].x, 1666.667, epsilon = 1.0);
    assert_relative_eq!(model.body_inertia[1].y, 1666.667, epsilon = 1.0);
    assert_relative_eq!(model.body_inertia[1].z, 1666.667, epsilon = 1.0);
}

// ============================================================================
// T3: Shell mesh inertia — explicit mass override → AC3
// ============================================================================

#[test]
fn t3_shell_mesh_explicit_mass_override() {
    let mjcf = format!(
        r#"<mujoco>
  <asset><mesh name="cube" inertia="shell" vertex="{CUBE_VERTICES}" face="{CUBE_FACES}"/></asset>
  <worldbody><body>
    <geom type="mesh" mesh="cube" mass="5.0"/>
  </body></worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).unwrap();
    assert_relative_eq!(model.body_mass[1], 5.0, epsilon = 1e-6);
    // Shell inertia scaled by mass/area = 5.0/6.0
    assert_relative_eq!(model.body_inertia[1].x, 1.389, epsilon = 0.01);
    assert_relative_eq!(model.body_inertia[1].y, 1.389, epsilon = 0.01);
    assert_relative_eq!(model.body_inertia[1].z, 1.389, epsilon = 0.01);
}

// ============================================================================
// T4: Sphere shell → AC4
// ============================================================================

#[test]
fn t4_sphere_shell_inertia() {
    let mjcf = r#"<mujoco>
  <worldbody><body>
    <geom type="sphere" size="1" density="1000" shellinertia="true"/>
  </body></worldbody>
</mujoco>"#;
    let model = load_model(mjcf).unwrap();
    // SA = 4πr² = 12566.37, mass = 1000 × 4π = 12566.37
    // I = 2/3 × m × r² = 8377.58
    assert_relative_eq!(model.body_mass[1], 12566.37, epsilon = 0.1);
    assert_relative_eq!(model.body_inertia[1].x, 8377.58, epsilon = 0.1);
    assert_relative_eq!(model.body_inertia[1].y, 8377.58, epsilon = 0.1);
    assert_relative_eq!(model.body_inertia[1].z, 8377.58, epsilon = 0.1);
}

// ============================================================================
// T5: Box shell → AC5
// ============================================================================

#[test]
fn t5_box_shell_inertia() {
    let mjcf = r#"<mujoco>
  <worldbody><body>
    <geom type="box" size="1 2 3" density="1000" shellinertia="true"/>
  </body></worldbody>
</mujoco>"#;
    let model = load_model(mjcf).unwrap();
    // SA = 8(ab + ac + bc) = 8(2+3+6) = 88, mass = 88000
    assert_relative_eq!(model.body_mass[1], 88000.0, epsilon = 1e-6);
    assert_relative_eq!(model.body_inertia[1].x, 541333.33, epsilon = 1.0);
    assert_relative_eq!(model.body_inertia[1].y, 421333.33, epsilon = 1.0);
    assert_relative_eq!(model.body_inertia[1].z, 242666.67, epsilon = 1.0);
}

// ============================================================================
// T6: Cylinder shell → AC6
// ============================================================================

#[test]
fn t6_cylinder_shell_inertia() {
    let mjcf = r#"<mujoco>
  <worldbody><body>
    <geom type="cylinder" size="1 2" density="1000" shellinertia="true"/>
  </body></worldbody>
</mujoco>"#;
    let model = load_model(mjcf).unwrap();
    // SA = 2πrh + 2πr² = 2π(4) + 2π(1) = 10π ≈ 31415.93
    assert_relative_eq!(model.body_mass[1], 31415.93, epsilon = 0.1);
    assert_relative_eq!(model.body_inertia[1].x, 72780.23, epsilon = 1.0);
    assert_relative_eq!(model.body_inertia[1].y, 72780.23, epsilon = 1.0);
    assert_relative_eq!(model.body_inertia[1].z, 28274.33, epsilon = 1.0);
}

// ============================================================================
// T7: Capsule shell → AC7
// ============================================================================

#[test]
fn t7_capsule_shell_inertia() {
    let mjcf = r#"<mujoco>
  <worldbody><body>
    <geom type="capsule" size="1 2" density="1000" shellinertia="true"/>
  </body></worldbody>
</mujoco>"#;
    let model = load_model(mjcf).unwrap();
    // SA = 2πrh + 4πr² = 2π(4) + 4π = 12π ≈ 37699.11
    assert_relative_eq!(model.body_mass[1], 37699.11, epsilon = 0.1);
    assert_relative_eq!(model.body_inertia[1].x, 129852.50, epsilon = 2.0);
    assert_relative_eq!(model.body_inertia[1].y, 129852.50, epsilon = 2.0);
    assert_relative_eq!(model.body_inertia[1].z, 33510.32, epsilon = 1.0);
}

// ============================================================================
// T8: Ellipsoid shell → AC8
// ============================================================================

#[test]
fn t8_ellipsoid_shell_inertia() {
    let mjcf = r#"<mujoco>
  <worldbody><body>
    <geom type="ellipsoid" size="1 2 3" density="1000" shellinertia="true"/>
  </body></worldbody>
</mujoco>"#;
    let model = load_model(mjcf).unwrap();
    // Thomsen SA ≈ 48971.93, tolerance 2% (Thomsen approximation ~1.05% off)
    let tol_mass = model.body_mass[1] * 0.02;
    assert_relative_eq!(model.body_mass[1], 48971.93, epsilon = tol_mass);
    let tol_ix = model.body_inertia[1].x * 0.02;
    let tol_iy = model.body_inertia[1].y * 0.02;
    let tol_iz = model.body_inertia[1].z * 0.02;
    assert_relative_eq!(model.body_inertia[1].x, 180751.03, epsilon = tol_ix);
    assert_relative_eq!(model.body_inertia[1].y, 140683.07, epsilon = tol_iy);
    assert_relative_eq!(model.body_inertia[1].z, 81026.34, epsilon = tol_iz);
}

// ============================================================================
// T9: Convex mode on non-convex mesh → AC9
// ============================================================================

#[test]
fn t9_convex_mode_non_convex_mesh() {
    let mjcf = format!(
        r#"<mujoco>
  <asset><mesh name="lshape" inertia="convex" vertex="{L_SHAPE_VERTICES}" face="{L_SHAPE_FACES}"/></asset>
  <worldbody><body>
    <geom type="mesh" mesh="lshape" density="1000"/>
  </body></worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).unwrap();
    // Convex hull of L-shape is a pentagonal prism, volume = 7.0
    // mass = density × hull_volume = 1000 × 7.0 = 7000
    assert_relative_eq!(model.body_mass[1], 7000.0, epsilon = 1.0);
    assert_relative_eq!(model.body_inertia[1].x, 6666.667, epsilon = 1.0);
    assert_relative_eq!(model.body_inertia[1].y, 3539.683, epsilon = 1.0);
    assert_relative_eq!(model.body_inertia[1].z, 9039.683, epsilon = 1.0);
}

// ============================================================================
// T10: Legacy mode → AC10
// ============================================================================

#[test]
fn t10_legacy_mode() {
    let mjcf = format!(
        r#"<mujoco>
  <asset><mesh name="lshape" inertia="legacy" vertex="{L_SHAPE_VERTICES}" face="{L_SHAPE_FACES}"/></asset>
  <worldbody><body>
    <geom type="mesh" mesh="lshape" density="1000"/>
  </body></worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).unwrap();
    // L-shape: exact volume = 5, legacy uses |det|/6 which equals signed det/6
    // for this L-shape (centroid inside solid), so legacy == exact: mass = 5000
    assert_relative_eq!(model.body_mass[1], 5000.0, epsilon = 1.0);
    assert_relative_eq!(model.body_inertia[1].x, 5833.333, epsilon = 1.0);
    assert_relative_eq!(model.body_inertia[1].y, 2233.333, epsilon = 1.0);
    assert_relative_eq!(model.body_inertia[1].z, 7233.333, epsilon = 1.0);
}

// ============================================================================
// T11: shellinertia rejected on mesh geom → AC11
// ============================================================================

#[test]
fn t11_shellinertia_rejected_on_mesh_geom() {
    let mjcf = format!(
        r#"<mujoco>
  <asset><mesh name="cube" vertex="{CUBE_VERTICES}" face="{CUBE_FACES}"/></asset>
  <worldbody><body>
    <geom type="mesh" mesh="cube" shellinertia="true"/>
  </body></worldbody>
</mujoco>"#
    );
    let result = load_model(&mjcf);
    assert!(result.is_err());
    assert!(result.unwrap_err().to_string().contains("for mesh geoms"));
}

// ============================================================================
// T12: Default class inheritance → AC12
// ============================================================================

#[test]
fn t12_default_class_inheritance_mesh_inertia() {
    let mjcf = format!(
        r#"<mujoco>
  <default><mesh inertia="shell"/></default>
  <asset><mesh name="cube" vertex="{CUBE_VERTICES}" face="{CUBE_FACES}"/></asset>
  <worldbody><body>
    <geom type="mesh" mesh="cube" density="1000"/>
  </body></worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).unwrap();
    // Shell inherited: mass = density × SA = 1000 × 6.0 = 6000
    assert_relative_eq!(model.body_mass[1], 6000.0, epsilon = 1e-6);
}

// ============================================================================
// T13: Ellipsoid solid volume fix → AC13
// ============================================================================

#[test]
fn t13_ellipsoid_solid_volume_fix() {
    let mjcf = r#"<mujoco>
  <worldbody><body>
    <geom type="ellipsoid" size="1 2 3" density="1000"/>
  </body></worldbody>
</mujoco>"#;
    let model = load_model(mjcf).unwrap();
    // V = 4/3 × π × 1 × 2 × 3 = 8π ≈ 25132.74
    let expected = 1000.0 * (4.0 / 3.0) * std::f64::consts::PI * 1.0 * 2.0 * 3.0;
    assert_relative_eq!(model.body_mass[1], expected, epsilon = 0.1);
}

// ============================================================================
// T14: exactmeshinertia deprecation → AC14
// ============================================================================

#[test]
fn t14_exactmeshinertia_deprecation() {
    let mjcf = format!(
        r#"<mujoco>
  <compiler exactmeshinertia="true"/>
  <asset><mesh name="cube" vertex="{CUBE_VERTICES}" face="{CUBE_FACES}"/></asset>
  <worldbody><body>
    <geom type="mesh" mesh="cube" density="1000"/>
  </body></worldbody>
</mujoco>"#
    );
    // Should parse without error (warning emitted)
    assert!(load_model(&mjcf).is_ok());
}

// ============================================================================
// T16: Degenerate mesh shell mode (edge case)
// ============================================================================

#[test]
fn t16_degenerate_mesh_shell() {
    // A flat (zero-volume) mesh: all vertices in the z=0 plane
    let mjcf = r#"<mujoco>
  <asset><mesh name="flat" inertia="shell"
    vertex="0 0 0  1 0 0  1 1 0  0 1 0"
    face="0 1 2  0 2 3"/></asset>
  <worldbody><body>
    <geom type="mesh" mesh="flat" density="1000"/>
  </body></worldbody>
</mujoco>"#;
    let model = load_model(mjcf).unwrap();
    // Should not crash. Area ≈ 1.0, mass ≈ 1000.0
    assert!(model.body_mass[1] > 0.0);
}

// ============================================================================
// T17: Mixed shell/solid body (edge case)
// ============================================================================

#[test]
fn t17_mixed_shell_solid_body() {
    let mjcf = r#"<mujoco>
  <worldbody><body>
    <geom type="sphere" size="1" density="1000"/>
    <geom type="sphere" size="1" density="1000" shellinertia="true" pos="3 0 0"/>
  </body></worldbody>
</mujoco>"#;
    let model = load_model(mjcf).unwrap();
    // Solid sphere mass = 4/3 π r³ × 1000 ≈ 4188.79
    // Shell sphere mass = 4π r² × 1000 ≈ 12566.37
    let solid_mass = (4.0 / 3.0) * std::f64::consts::PI * 1000.0;
    let shell_mass = 4.0 * std::f64::consts::PI * 1000.0;
    assert_relative_eq!(model.body_mass[1], solid_mass + shell_mass, epsilon = 0.1);
}

// ============================================================================
// T18: Exact mode rejects misoriented mesh → AC17
// ============================================================================

#[test]
fn t18_exact_mode_rejects_misoriented_mesh() {
    let mjcf = format!(
        r#"<mujoco>
  <asset><mesh name="rev" inertia="exact" vertex="{CUBE_VERTICES}" face="{REVERSED_CUBE_FACES}"/></asset>
  <worldbody><body>
    <geom type="mesh" mesh="rev" density="1000"/>
  </body></worldbody>
</mujoco>"#
    );
    let result = load_model(&mjcf);
    // Reversed winding produces negative volume. Exact mode must reject.
    // MuJoCo ref: "mesh volume is negative (misoriented triangles)"
    assert!(result.is_err(), "exact mode must reject misoriented mesh");
    assert!(
        result
            .unwrap_err()
            .to_string()
            .contains("mesh volume is negative"),
        "error must mention 'mesh volume is negative'"
    );
}

// ============================================================================
// T19: Default class inheritance for shellinertia → AC18
// ============================================================================

#[test]
fn t19_default_class_inheritance_shellinertia() {
    let mjcf = r#"<mujoco>
  <default><geom shellinertia="true"/></default>
  <worldbody><body>
    <geom type="sphere" size="1" density="1000"/>
  </body></worldbody>
</mujoco>"#;
    let model = load_model(mjcf).unwrap();
    // Shell mass = ρ × 4πr² = 12566.37 (not solid 4188.79)
    assert_relative_eq!(model.body_mass[1], 12566.37, epsilon = 0.1);
}
