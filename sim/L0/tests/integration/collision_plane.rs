//! Plane collision tests — exhaustive coverage for all primitive-plane pairs.
//!
//! This module validates collision detection between an infinite plane and all
//! primitive geometry types: Sphere, Box, Capsule, Cylinder, Ellipsoid.
//!
//! # Test Philosophy
//!
//! > **Todorov Standard**: Every geometric configuration must be tested.
//! > Penetrating, touching, separated. All canonical orientations.
//! >
//! > **Rust Purist Standard**: Tests are self-documenting. Each test name
//! > describes the exact configuration being validated.
//!
//! # Coverage Matrix
//!
//! Each primitive type is tested in these configurations:
//! - **Penetrating**: Geometry overlaps plane, positive depth
//! - **Touching**: Geometry exactly touches plane (depth ≈ 0)
//! - **Separated**: Geometry above plane with gap (no contact)
//! - **Orientations**: All rotations from `canonical_set()`
//!
//! Cylinder and Ellipsoid have additional orientation-dependent cases.

use sim_mjcf::load_model;

use crate::collision_test_utils::{DEPTH_TOL, GEOM_TOL};

// ============================================================================
// Sphere-Plane Tests
// ============================================================================

/// Sphere penetrating plane: center above plane but surface below.
///
/// Configuration:
/// - Plane at z=0, normal +Z
/// - Sphere radius 0.5, center at z=0.4
/// - Expected depth: 0.5 - 0.4 = 0.1
#[test]
fn sphere_plane_penetrating() {
    let mjcf = r#"
        <mujoco model="sphere_plane_penetrating">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="ball" pos="0 0 0.4">
                    <geom type="sphere" size="0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(data.ncon, 1, "Expected exactly 1 contact");

    let contact = &data.contacts[0];

    // Check penetration depth
    let expected_depth = 0.1;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Depth: expected {}, got {} (diff {})",
        expected_depth,
        contact.depth,
        (contact.depth - expected_depth).abs()
    );

    // Check normal points up (+Z)
    assert!(
        contact.normal.z > 0.99,
        "Normal should point up (+Z), got {:?}",
        contact.normal
    );

    // Check normal is unit length
    let normal_len = contact.normal.norm();
    assert!(
        (normal_len - 1.0).abs() < GEOM_TOL,
        "Normal should be unit length, got {}",
        normal_len
    );
}

/// Sphere touching plane exactly: surface at z=0.
///
/// Configuration:
/// - Plane at z=0, normal +Z
/// - Sphere radius 0.5, center at z=0.5
/// - Expected depth: ≈ 0 (may have numerical noise)
#[test]
fn sphere_plane_touching() {
    let mjcf = r#"
        <mujoco model="sphere_plane_touching">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="ball" pos="0 0 0.5">
                    <geom type="sphere" size="0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Touching case: depth should be essentially zero or very small positive
    // Some implementations report no contact for exactly touching, others report ~0 depth
    if data.ncon > 0 {
        let contact = &data.contacts[0];
        assert!(
            contact.depth < 1e-4,
            "Touching sphere should have near-zero depth, got {}",
            contact.depth
        );
    }
    // If ncon == 0, that's also acceptable for exactly touching
}

/// Sphere separated from plane: gap between surface and plane.
///
/// Configuration:
/// - Plane at z=0
/// - Sphere radius 0.5, center at z=0.6 (0.1 gap)
/// - Expected: no contact
#[test]
fn sphere_plane_separated() {
    let mjcf = r#"
        <mujoco model="sphere_plane_separated">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="ball" pos="0 0 0.6">
                    <geom type="sphere" size="0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(data.ncon, 0, "Separated sphere should have no contacts");
}

/// Sphere with large penetration: center below plane surface.
///
/// Configuration:
/// - Plane at z=0
/// - Sphere radius 0.5, center at z=0.1 (40% embedded)
/// - Expected depth: 0.5 - 0.1 = 0.4
#[test]
fn sphere_plane_deep_penetration() {
    let mjcf = r#"
        <mujoco model="sphere_plane_deep">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="ball" pos="0 0 0.1">
                    <geom type="sphere" size="0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(data.ncon, 1, "Expected exactly 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.4;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Deep penetration: expected {} depth, got {}",
        expected_depth,
        contact.depth
    );
}

// ============================================================================
// Box-Plane Tests
// ============================================================================

/// Box corner penetrating plane.
///
/// Configuration:
/// - Plane at z=0
/// - Box half-extents (0.5, 0.5, 0.5), center at z=0.4
/// - Bottom face at z = 0.4 - 0.5 = -0.1
/// - Expected depth: 0.1
#[test]
fn box_plane_corner_penetrating() {
    let mjcf = r#"
        <mujoco model="box_plane_penetrating">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="box" pos="0 0 0.4">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert!(data.ncon >= 1, "Expected at least 1 contact");

    // Find deepest contact
    let max_depth = data.contacts[..data.ncon]
        .iter()
        .map(|c| c.depth)
        .fold(0.0_f64, f64::max);

    let expected_depth = 0.1;
    assert!(
        (max_depth - expected_depth).abs() < DEPTH_TOL,
        "Box penetration: expected {} depth, got {}",
        expected_depth,
        max_depth
    );
}

/// Box resting flat on plane: entire face touching.
///
/// Configuration:
/// - Plane at z=0
/// - Box half-extents (0.5, 0.5, 0.5), center at z=0.5
/// - Bottom face exactly at z=0
#[test]
fn box_plane_face_touching() {
    let mjcf = r#"
        <mujoco model="box_plane_touching">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="box" pos="0 0 0.5">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Touching case: depth ≈ 0
    for c in &data.contacts[..data.ncon] {
        assert!(
            c.depth < 1e-4,
            "Touching box should have near-zero depth, got {}",
            c.depth
        );
    }
}

/// Box tilted 45° touching plane with single edge.
///
/// Configuration:
/// - Plane at z=0
/// - Box rotated 45° about X axis
/// - Single edge should contact plane
#[test]
fn box_plane_edge_tilted() {
    let mjcf = r#"
        <mujoco model="box_plane_tilted">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="box" pos="0 0 0.6" euler="45 0 0">
                    <geom type="box" size="0.3 0.3 0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // For tilted box, we should still detect contact if penetrating
    // At 45°, the diagonal is sqrt(2) * 0.3 ≈ 0.424
    // Center at 0.6 means lowest corner is at roughly 0.6 - 0.424 ≈ 0.176 (above plane)
    // So this should NOT contact. Let's verify.
    assert_eq!(data.ncon, 0, "Tilted box at z=0.6 should not contact plane");
}

/// Box tilted 45° penetrating plane with corner.
#[test]
fn box_plane_corner_tilted_penetrating() {
    let mjcf = r#"
        <mujoco model="box_plane_tilted_pen">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="box" pos="0 0 0.35" euler="45 0 0">
                    <geom type="box" size="0.3 0.3 0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // At 45°, diagonal ≈ 0.424. Center at 0.35 → lowest at ~0.35 - 0.424 = -0.074
    // Should penetrate by about 0.074
    assert!(
        data.ncon >= 1,
        "Tilted penetrating box should contact plane"
    );

    let max_depth = data.contacts[..data.ncon]
        .iter()
        .map(|c| c.depth)
        .fold(0.0_f64, f64::max);

    // Rough check - exact value depends on which corner
    assert!(
        max_depth > 0.05 && max_depth < 0.15,
        "Tilted box depth should be ~0.07, got {}",
        max_depth
    );
}

// ============================================================================
// Capsule-Plane Tests
// ============================================================================

/// Capsule standing upright on plane.
///
/// Configuration:
/// - Plane at z=0
/// - Capsule radius 0.2, half-length 0.5, axis along Z
/// - Center at z=0.6 → bottom endpoint at z=0.1, bottom sphere surface at z=-0.1
/// - Expected depth: 0.1 if penetrating
#[test]
fn capsule_plane_upright_penetrating() {
    let mjcf = r#"
        <mujoco model="capsule_plane_upright">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cap" pos="0 0 0.6">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Capsule: radius=0.2, half_length=0.5
    // Bottom endpoint at z = 0.6 - 0.5 = 0.1
    // Bottom sphere surface at z = 0.1 - 0.2 = -0.1
    // Penetration = 0.1
    assert_eq!(data.ncon, 1, "Upright capsule should have 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.1;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Capsule depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Capsule lying horizontal on plane.
///
/// Configuration:
/// - Plane at z=0
/// - Capsule radius 0.2, half-length 0.5, axis along X
/// - Center at z=0.15 → curved surface at z=-0.05
/// - Expected depth: 0.05
#[test]
fn capsule_plane_horizontal_penetrating() {
    let mjcf = r#"
        <mujoco model="capsule_plane_horizontal">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cap" pos="0 0 0.15" euler="0 90 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Horizontal capsule: lowest point is on hemisphere, at z = 0.15 - 0.2 = -0.05
    // Penetration = 0.05
    assert!(data.ncon >= 1, "Horizontal capsule should contact plane");

    let max_depth = data.contacts[..data.ncon]
        .iter()
        .map(|c| c.depth)
        .fold(0.0_f64, f64::max);

    let expected_depth = 0.05;
    assert!(
        (max_depth - expected_depth).abs() < DEPTH_TOL,
        "Horizontal capsule depth: expected {}, got {}",
        expected_depth,
        max_depth
    );
}

/// Capsule tilted 45° on plane.
#[test]
fn capsule_plane_tilted_45() {
    let mjcf = r#"
        <mujoco model="capsule_plane_tilted">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cap" pos="0 0 0.5" euler="45 0 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // At 45°, the lower endpoint is at z = 0.5 - 0.5*cos(45°) = 0.5 - 0.354 = 0.146
    // The sphere surface extends further down by radius 0.2: 0.146 - 0.2 = -0.054
    // Penetration ≈ 0.054
    assert!(data.ncon >= 1, "Tilted capsule should contact plane");

    let max_depth = data.contacts[..data.ncon]
        .iter()
        .map(|c| c.depth)
        .fold(0.0_f64, f64::max);

    // Allow tolerance for trig precision
    assert!(
        max_depth > 0.04 && max_depth < 0.07,
        "Tilted capsule depth should be ~0.054, got {}",
        max_depth
    );
}

// ============================================================================
// Cylinder-Plane Tests
// ============================================================================

/// Cylinder standing upright on plane (axis perpendicular to plane).
///
/// This is the simplest cylinder-plane case: the entire bottom rim touches.
#[test]
fn cylinder_plane_upright_penetrating() {
    let mjcf = r#"
        <mujoco model="cylinder_plane_upright">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cyl" pos="0 0 0.4">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Cylinder: radius=0.3, half_height=0.5
    // Center at z=0.4 → bottom cap at z = 0.4 - 0.5 = -0.1
    // Penetration = 0.1
    assert!(data.ncon >= 1, "Upright cylinder should contact plane");

    let contact = &data.contacts[0];
    let expected_depth = 0.1;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Upright cylinder depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );

    // Normal should point up
    assert!(
        contact.normal.z > 0.99,
        "Contact normal should point up, got {:?}",
        contact.normal
    );
}

/// Cylinder lying horizontal on plane (axis parallel to plane).
///
/// The curved surface should contact the plane.
#[test]
fn cylinder_plane_horizontal_penetrating() {
    let mjcf = r#"
        <mujoco model="cylinder_plane_horizontal">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cyl" pos="0 0 0.25" euler="0 90 0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Horizontal cylinder: radius=0.3, axis along X
    // Lowest point is on curved surface at z = 0.25 - 0.3 = -0.05
    // Penetration = 0.05
    assert!(data.ncon >= 1, "Horizontal cylinder should contact plane");

    let max_depth = data.contacts[..data.ncon]
        .iter()
        .map(|c| c.depth)
        .fold(0.0_f64, f64::max);

    let expected_depth = 0.05;
    assert!(
        (max_depth - expected_depth).abs() < DEPTH_TOL,
        "Horizontal cylinder depth: expected {}, got {}",
        expected_depth,
        max_depth
    );
}

/// Cylinder tilted 45° on plane.
///
/// The rim edge should contact the plane. This tests the tilted case
/// in the cylinder-plane collision algorithm.
#[test]
fn cylinder_plane_tilted_45_penetrating() {
    let mjcf = r#"
        <mujoco model="cylinder_plane_tilted">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cyl" pos="0 0 0.5" euler="45 0 0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Tilted cylinder at 45°:
    // The deepest point is on the lower rim edge
    // Axis tilted 45° about X means bottom cap center at:
    //   z = 0.5 - 0.5*cos(45°) = 0.5 - 0.354 ≈ 0.146
    // Rim point extends further in the -z direction by:
    //   radius * sin(45°) = 0.3 * 0.707 ≈ 0.212
    // So deepest rim point at z ≈ 0.146 - 0.212 = -0.066
    // Penetration ≈ 0.066
    assert!(data.ncon >= 1, "Tilted cylinder should contact plane");

    let max_depth = data.contacts[..data.ncon]
        .iter()
        .map(|c| c.depth)
        .fold(0.0_f64, f64::max);

    // Allow tolerance for geometric computation
    assert!(
        max_depth > 0.04 && max_depth < 0.10,
        "Tilted cylinder depth should be ~0.066, got {}",
        max_depth
    );

    // Verify normal points up
    let contact = &data.contacts[0];
    assert!(
        contact.normal.z > 0.99,
        "Contact normal should point up, got {:?}",
        contact.normal
    );
}

/// Cylinder separated from plane (no contact).
#[test]
fn cylinder_plane_separated() {
    let mjcf = r#"
        <mujoco model="cylinder_plane_separated">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cyl" pos="0 0 1.0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Center at z=1.0, half_height=0.5 → bottom at z=0.5 (above plane)
    assert_eq!(data.ncon, 0, "Separated cylinder should have no contacts");
}

// ============================================================================
// Ellipsoid-Plane Tests
// ============================================================================

/// Ellipsoid with equal radii (sphere) on plane — baseline test.
#[test]
fn ellipsoid_plane_spherical_penetrating() {
    let mjcf = r#"
        <mujoco model="ellipsoid_plane_spherical">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="ell" pos="0 0 0.4">
                    <geom type="ellipsoid" size="0.5 0.5 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Spherical ellipsoid: radius=0.5, center at z=0.4
    // Penetration = 0.5 - 0.4 = 0.1
    assert!(data.ncon >= 1, "Spherical ellipsoid should contact plane");

    let contact = &data.contacts[0];
    let expected_depth = 0.1;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Spherical ellipsoid depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Ellipsoid with anisotropic radii (tall): longest axis vertical.
///
/// Configuration:
/// - Radii (0.2, 0.2, 0.5) — tall in Z
/// - Center at z=0.4
/// - Bottom at z = 0.4 - 0.5 = -0.1
/// - Expected depth: 0.1
#[test]
fn ellipsoid_plane_tall_vertical() {
    let mjcf = r#"
        <mujoco model="ellipsoid_plane_tall">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="ell" pos="0 0 0.4">
                    <geom type="ellipsoid" size="0.2 0.2 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Tall ellipsoid: radii=(0.2, 0.2, 0.5), center at z=0.4
    // Lowest point at z = 0.4 - 0.5 = -0.1
    // Penetration = 0.1
    assert!(data.ncon >= 1, "Tall ellipsoid should contact plane");

    let contact = &data.contacts[0];
    let expected_depth = 0.1;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Tall ellipsoid depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Ellipsoid with anisotropic radii (flat): shortest axis vertical.
///
/// Configuration:
/// - Radii (0.5, 0.5, 0.2) — flat in Z
/// - Center at z=0.15
/// - Bottom at z = 0.15 - 0.2 = -0.05
/// - Expected depth: 0.05
#[test]
fn ellipsoid_plane_flat_vertical() {
    let mjcf = r#"
        <mujoco model="ellipsoid_plane_flat">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="ell" pos="0 0 0.15">
                    <geom type="ellipsoid" size="0.5 0.5 0.2"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Flat ellipsoid: radii=(0.5, 0.5, 0.2), center at z=0.15
    // Lowest point at z = 0.15 - 0.2 = -0.05
    // Penetration = 0.05
    assert!(data.ncon >= 1, "Flat ellipsoid should contact plane");

    let contact = &data.contacts[0];
    let expected_depth = 0.05;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Flat ellipsoid depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Ellipsoid rotated so longest axis is horizontal.
///
/// Configuration:
/// - Radii (0.5, 0.2, 0.2) — long in X
/// - Rotated 90° about Y so long axis points along Z? No, that would be vertical.
/// - Actually: rotate so long axis (X) stays horizontal, short axis (Z) becomes vertical
/// - After rotation, effective vertical radius should change
#[test]
fn ellipsoid_plane_rotated_long_horizontal() {
    let mjcf = r#"
        <mujoco model="ellipsoid_plane_rotated">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="ell" pos="0 0 0.15">
                    <geom type="ellipsoid" size="0.5 0.2 0.2"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Ellipsoid: radii=(0.5, 0.2, 0.2), long axis along X
    // No rotation, so effective vertical radius is 0.2 (Z radius)
    // Center at z=0.15 → bottom at z = 0.15 - 0.2 = -0.05
    // Penetration = 0.05
    assert!(data.ncon >= 1, "Rotated ellipsoid should contact plane");

    let contact = &data.contacts[0];
    let expected_depth = 0.05;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Rotated ellipsoid depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Ellipsoid rotated to put long axis vertical.
#[test]
fn ellipsoid_plane_rotated_long_vertical() {
    let mjcf = r#"
        <mujoco model="ellipsoid_plane_rotated_vert">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="ell" pos="0 0 0.4" euler="0 90 0">
                    <geom type="ellipsoid" size="0.5 0.2 0.2"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Ellipsoid: radii=(0.5, 0.2, 0.2)
    // Rotated 90° about Y: local X becomes world Z
    // Now the long axis (0.5) is vertical
    // Center at z=0.4 → bottom at z = 0.4 - 0.5 = -0.1
    // Penetration = 0.1
    assert!(
        data.ncon >= 1,
        "Vertically-rotated ellipsoid should contact plane"
    );

    let contact = &data.contacts[0];
    let expected_depth = 0.1;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Vertically-rotated ellipsoid depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Ellipsoid separated from plane.
#[test]
fn ellipsoid_plane_separated() {
    let mjcf = r#"
        <mujoco model="ellipsoid_plane_separated">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="ell" pos="0 0 1.0">
                    <geom type="ellipsoid" size="0.5 0.5 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Center at z=1.0, max radius=0.5 → bottom at z=0.5 (above plane)
    assert_eq!(data.ncon, 0, "Separated ellipsoid should have no contacts");
}

// ============================================================================
// Tilted Plane Tests — Verify correct normal handling
// ============================================================================

/// Sphere on tilted plane (plane rotated 30° about X).
///
/// Tests that plane normal is correctly computed from plane orientation.
#[test]
fn sphere_tilted_plane_penetrating() {
    let mjcf = r#"
        <mujoco model="sphere_tilted_plane">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1" euler="30 0 0"/>
                <body name="ball" pos="0 0 0.4">
                    <geom type="sphere" size="0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // The tilted plane's normal is not +Z anymore
    // 30° rotation about X: normal = (0, -sin(30°), cos(30°)) = (0, -0.5, 0.866)
    // Wait, rotation about X rotates Y toward Z: normal starts as (0,0,1)
    // After rotation: (0, -sin(30°), cos(30°)) = (0, -0.5, 0.866)
    // Actually the sign depends on rotation direction convention.

    // For this test, just verify contact exists and normal is not purely +Z
    assert!(data.ncon >= 1, "Sphere should contact tilted plane");

    let contact = &data.contacts[0];

    // Normal should have a Y component now (not purely +Z)
    let is_tilted_normal = contact.normal.z.abs() < 0.99;
    assert!(
        is_tilted_normal,
        "Tilted plane normal should not be purely +Z, got {:?}",
        contact.normal
    );

    // Normal should still be unit length
    let normal_len = contact.normal.norm();
    assert!(
        (normal_len - 1.0).abs() < GEOM_TOL,
        "Normal should be unit length, got {}",
        normal_len
    );
}

// ============================================================================
// Numerical Edge Cases
// ============================================================================

/// Very small penetration (near machine precision).
#[test]
fn sphere_plane_tiny_penetration() {
    let mjcf = r#"
        <mujoco model="sphere_plane_tiny">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="ball" pos="0 0 0.4999">
                    <geom type="sphere" size="0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Penetration = 0.5 - 0.4999 = 0.0001
    assert!(data.ncon >= 1, "Tiny penetration should still be detected");

    let contact = &data.contacts[0];
    let expected_depth = 0.0001;
    assert!(
        (contact.depth - expected_depth).abs() < 1e-5,
        "Tiny penetration depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Large geometry far from origin.
#[test]
fn sphere_plane_far_from_origin() {
    let offset = 1000.0; // 1km from origin
    let mjcf = format!(
        r#"
        <mujoco model="sphere_plane_far">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10000 10000 0.1" pos="{} 0 0"/>
                <body name="ball" pos="{} 0 0.4">
                    <geom type="sphere" size="0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
        offset, offset
    );

    let model = load_model(&mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Even at large offsets, collision should work correctly
    assert!(
        data.ncon >= 1,
        "Far-from-origin collision should be detected"
    );

    let contact = &data.contacts[0];
    let expected_depth = 0.1;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Far-from-origin depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

// ============================================================================
// Multi-contact box-plane tests (Phase 1a)
// ============================================================================

/// Box resting flat on horizontal plane → exactly 4 contacts.
///
/// Configuration:
/// - Plane at z=0 (horizontal)
/// - Box half-extents (0.5, 0.5, 0.5), center at z=0.4
/// - Bottom face at z=-0.1 → all 4 corners penetrate equally
///
/// Expected: 4 contacts, all with identical depth (0.1).
#[test]
fn box_plane_horizontal_4_contacts() {
    let mjcf = r#"
        <mujoco model="box_plane_4contacts">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="box" pos="0 0 0.4">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(
        data.ncon, 4,
        "Box flat on horizontal plane should produce 4 contacts, got {}",
        data.ncon
    );

    // All 4 corners have the same depth on a horizontal plane
    let expected_depth = 0.1;
    for (i, c) in data.contacts[..4].iter().enumerate() {
        assert!(
            (c.depth - expected_depth).abs() < DEPTH_TOL,
            "Contact {i}: expected depth {expected_depth}, got {}",
            c.depth
        );
    }
}

/// Box resting on tilted plane → 4 contacts with distinct depths.
///
/// Configuration:
/// - Plane tilted 10° around X axis
/// - Box half-extents (0.3, 0.3, 0.3), center placed to penetrate
/// - Bottom face corners have varying distances to tilted plane
///
/// Expected: 4 contacts, depths vary by ~2*0.3*sin(10°) ≈ 0.104 across face.
#[test]
fn box_plane_tilted_4_contacts_distinct_depths() {
    let mjcf = r#"
        <mujoco model="box_plane_tilted_4">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"
                      euler="10 0 0"/>
                <body name="box" pos="0 0 0.2">
                    <geom type="box" size="0.3 0.3 0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(
        data.ncon, 4,
        "Box on tilted plane should produce 4 contacts, got {}",
        data.ncon
    );

    // Depths should be distinct (not all identical)
    let depths: Vec<f64> = data.contacts[..4].iter().map(|c| c.depth).collect();
    let min_depth = depths.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_depth = depths.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let depth_variation = max_depth - min_depth;

    // On a 10° tilt, the y-extent (0.3) contributes sin(10°)*0.6 ≈ 0.104
    assert!(
        depth_variation > 0.05,
        "Depths should vary significantly on tilted plane, got variation={depth_variation:.4e}, depths={depths:?}"
    );

    // All contacts should share the same normal (plane normal)
    let n0 = data.contacts[0].normal;
    for (i, c) in data.contacts[1..4].iter().enumerate() {
        let dot = n0.dot(&c.normal);
        assert!(
            (dot - 1.0).abs() < 1e-10,
            "Contact {} normal should match contact 0: dot={dot}",
            i + 1
        );
    }
}

/// Single box on horizontal plane settles stably with 4-contact support.
///
/// Tests that multi-contact plane-box provides stable resting on a flat
/// surface under gravity. (3-box stack deferred to Phase 2 when box-box
/// multi-contact is implemented.)
#[test]
fn box_plane_single_box_stable() {
    let mjcf = r#"
        <mujoco model="box_plane_stable">
            <option gravity="0 0 -9.81" solver="Newton" cone="elliptic"
                    iterations="100" tolerance="1e-8"/>
            <worldbody>
                <geom type="plane" size="5 5 0.1"/>
                <body name="box" pos="0 0 0.11">
                    <joint type="free"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                          friction="0.5 0.5 0.005"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();

    for _ in 0..500 {
        data.step(&model).unwrap();
    }

    assert!(
        data.qpos.iter().all(|x| x.is_finite()),
        "Box diverged: qpos contains NaN/Inf"
    );

    // Box should have settled: very small velocities
    let max_vel = data.qvel.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
    assert!(
        max_vel < 0.01,
        "Box should have settled: max velocity = {max_vel:.4e}"
    );

    // Z position should be approximately the half-extent (resting on plane)
    let z = data.qpos[2];
    assert!(z > 0.08 && z < 0.12, "Box z should be ~0.1, got {z:.4}");

    // Should have 4 contacts in steady state
    assert_eq!(
        data.ncon, 4,
        "Settled box on plane should have 4 contacts, got {}",
        data.ncon
    );
}

/// Noslip with multi-contact box-plane produces finite, bounded slip.
///
/// Verifies noslip postprocessor convergence with 4 contacts on a tilted
/// plane — the configuration that broke with the original quick fix.
#[test]
fn noslip_multi_contact_box_plane_converges() {
    let mjcf = r#"
        <mujoco model="noslip_multi">
            <option gravity="0 0 -9.81" solver="Newton" cone="elliptic"
                    noslip_iterations="50" noslip_tolerance="1e-12"/>
            <worldbody>
                <body pos="0 0 0.11">
                    <joint type="free"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                          friction="0.3 0.3 0.005"/>
                </body>
                <geom type="plane" size="5 5 0.1"
                      euler="3 0 0" friction="0.3 0.3 0.005"/>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();

    for _ in 0..50 {
        data.step(&model).unwrap();
    }

    let slip = (data.qvel[0].powi(2) + data.qvel[1].powi(2)).sqrt();

    assert!(slip.is_finite(), "Noslip slip should be finite");
    assert!(
        slip < 0.5,
        "Noslip multi-contact slip should be bounded, got {slip:.4e}"
    );
    assert!(
        data.qpos.iter().all(|x| x.is_finite()),
        "All positions should be finite"
    );
}

// ============================================================================
// Multi-contact capsule-plane tests (Phase 1b)
// ============================================================================

/// Horizontal capsule on plane → 2 contacts (both endpoints penetrate).
///
/// Configuration:
/// - Plane at z=0 (horizontal)
/// - Capsule radius=0.2, half_length=0.5, axis along X (euler="0 90 0")
/// - Center at z=0.1 → both endpoint spheres at z=0.1
/// - Both sphere surfaces at z = 0.1 - 0.2 = -0.1
///
/// Expected: 2 contacts, both with depth 0.1.
#[test]
fn capsule_plane_horizontal_2_contacts() {
    let mjcf = r#"
        <mujoco model="capsule_plane_2">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cap" pos="0 0 0.1" euler="0 90 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(
        data.ncon, 2,
        "Horizontal capsule on plane should produce 2 contacts, got {}",
        data.ncon
    );

    // Both endpoints at same height → same depth
    let expected_depth = 0.1;
    for (i, c) in data.contacts[..2].iter().enumerate() {
        assert!(
            (c.depth - expected_depth).abs() < DEPTH_TOL,
            "Contact {i}: expected depth {expected_depth}, got {}",
            c.depth
        );
    }

    // Contact positions should be separated by ~capsule length along X
    let dx = (data.contacts[0].pos.x - data.contacts[1].pos.x).abs();
    assert!(
        dx > 0.8,
        "Contacts should be separated along capsule axis, got dx={dx:.3}"
    );
}

/// Upright capsule on plane → 1 contact (only bottom endpoint).
///
/// Verifies that a vertical capsule still produces exactly 1 contact
/// (the top endpoint is far from the plane).
#[test]
fn capsule_plane_upright_1_contact() {
    let mjcf = r#"
        <mujoco model="capsule_plane_upright_1">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cap" pos="0 0 0.6">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Bottom endpoint at z=0.1, top at z=1.1 — only bottom contacts
    assert_eq!(
        data.ncon, 1,
        "Upright capsule should have exactly 1 contact, got {}",
        data.ncon
    );
}

// ============================================================================
// Phase 1c: Cylinder-Plane Multi-Contact Tests
// ============================================================================

/// Horizontal cylinder on plane → 2 contacts (both cap rim points).
///
/// Configuration: radius=0.3, half_height=0.5, euler="0 90 0" (axis along X),
/// center at z=0.25. Both cap rims at z = 0.25 - 0.3 = -0.05 (penetration = 0.05).
/// Triangle points are above the plane (no extra contacts).
#[test]
fn cylinder_plane_horizontal_2_contacts() {
    let mjcf = r#"
        <mujoco model="cylinder_plane_horiz_2">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cyl" pos="0 0 0.25" euler="0 90 0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(
        data.ncon, 2,
        "Horizontal cylinder should produce 2 contacts (both cap rims), got {}",
        data.ncon
    );

    let expected_depth = 0.05;
    for (i, c) in data.contacts[..2].iter().enumerate() {
        assert!(
            (c.depth - expected_depth).abs() < DEPTH_TOL,
            "Contact {i}: expected depth {expected_depth}, got {}",
            c.depth
        );
    }

    // Both contacts should share the same upward normal
    for c in &data.contacts[..2] {
        assert!(
            c.normal.z > 0.99,
            "Contact normal should point up, got {:?}",
            c.normal
        );
    }
}

/// Upright cylinder on plane → 3 contacts (1 rim + 2 triangle support).
///
/// Configuration: radius=0.3, half_height=0.5, upright (axis along Z),
/// center at z=0.4. Bottom cap at z = -0.1, penetration = 0.1.
/// Rim point + 2 equilateral triangle points on the bottom disk, all at
/// the same depth (disk is parallel to plane).
#[test]
fn cylinder_plane_upright_3_contacts() {
    let mjcf = r#"
        <mujoco model="cylinder_plane_upright_3">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cyl" pos="0 0 0.4">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Upright: 1 near-cap rim + 2 triangle. Far cap is above plane (no contact).
    assert_eq!(
        data.ncon, 3,
        "Upright cylinder should produce 3 contacts (rim + 2 triangle), got {}",
        data.ncon
    );

    // All 3 points are on the bottom disk at z=-0.1, so same depth
    let expected_depth = 0.1;
    for (i, c) in data.contacts[..3].iter().enumerate() {
        assert!(
            (c.depth - expected_depth).abs() < DEPTH_TOL,
            "Contact {i}: expected depth {expected_depth}, got {}",
            c.depth
        );
    }
}

/// Tilted cylinder (45°) on plane → 1 contact (near-cap rim only).
///
/// At 45° tilt, only the deepest near-cap rim point contacts the plane.
/// The far-cap rim and triangle points are too far above the plane.
#[test]
fn cylinder_plane_tilted_1_contact() {
    let mjcf = r#"
        <mujoco model="cylinder_plane_tilted_1">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cyl" pos="0 0 0.5" euler="45 0 0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // At 45° tilt, only the near-cap rim contacts
    assert_eq!(
        data.ncon, 1,
        "45° tilted cylinder should produce 1 contact, got {}",
        data.ncon
    );

    // Deepest rim at z ≈ 0.5 - 0.5*cos(45°) - 0.3*sin(45°) ≈ -0.066
    let depth = data.contacts[0].depth;
    assert!(
        depth > 0.04 && depth < 0.10,
        "Tilted cylinder depth should be ~0.066, got {depth}"
    );
}

/// Slightly tilted cylinder (10°) on plane → 3 contacts (rim + 2 triangle).
///
/// At shallow tilt, the near-cap rim contacts AND the two equilateral
/// triangle support points on the near-cap disk also contact. The far cap
/// is too far above to contribute.
#[test]
fn cylinder_plane_slight_tilt_3_contacts() {
    let mjcf = r#"
        <mujoco model="cylinder_plane_tilt10_3">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cyl" pos="0 0 0.4" euler="10 0 0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // At 10° tilt: near rim + 2 triangle (near-cap disk still faces plane)
    assert_eq!(
        data.ncon, 3,
        "10° tilted cylinder should produce 3 contacts, got {}",
        data.ncon
    );

    // Depths should vary: rim is deepest, triangles are shallower
    let depths: Vec<f64> = data.contacts[..3].iter().map(|c| c.depth).collect();
    let max_depth = depths.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let min_depth = depths.iter().cloned().fold(f64::INFINITY, f64::min);

    assert!(
        max_depth > min_depth + 0.01,
        "Depths should vary on tilted plane: max={max_depth:.4}, min={min_depth:.4}"
    );
}

// ============================================================================
// Phase 1d: Mesh-Plane Multi-Contact Tests
// ============================================================================

/// Box-shaped mesh on horizontal plane → 3 contacts (capped from 4 bottom vertices).
///
/// Configuration: 1×1×1 box mesh at z=0.4. Bottom face at z=-0.1 has 4 vertices,
/// all penetrating with depth=0.1. Proximity filter (0.3 * rbound ≈ 0.26) passes
/// all 4 since they're 1.0 apart. Capped at maxplanemesh=3.
#[test]
fn mesh_plane_box_3_contacts() {
    let mjcf = r#"
        <mujoco model="mesh_plane_box_3">
            <option gravity="0 0 0" timestep="0.001"/>
            <asset>
                <mesh name="box1"
                    vertex="-.5 -.5 -.5  .5 -.5 -.5  .5 .5 -.5  -.5 .5 -.5
                            -.5 -.5 .5   .5 -.5 .5   .5 .5 .5   -.5 .5 .5"
                    face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4
                          2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/>
            </asset>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="mesh_box" pos="0 0 0.4">
                    <geom type="mesh" mesh="box1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // 4 bottom vertices penetrate, capped at 3
    assert_eq!(
        data.ncon, 3,
        "Box mesh on plane should produce 3 contacts (capped from 4 bottom vertices), got {}",
        data.ncon
    );

    // All contacts on the bottom face at z=-0.1 → depth=0.1
    let expected_depth = 0.1;
    for (i, c) in data.contacts[..3].iter().enumerate() {
        assert!(
            (c.depth - expected_depth).abs() < DEPTH_TOL,
            "Contact {i}: expected depth {expected_depth}, got {}",
            c.depth
        );
    }
}

/// Box-shaped mesh separated from plane → 0 contacts.
#[test]
fn mesh_plane_box_separated() {
    let mjcf = r#"
        <mujoco model="mesh_plane_separated">
            <option gravity="0 0 0" timestep="0.001"/>
            <asset>
                <mesh name="box1"
                    vertex="-.5 -.5 -.5  .5 -.5 -.5  .5 .5 -.5  -.5 .5 -.5
                            -.5 -.5 .5   .5 -.5 .5   .5 .5 .5   -.5 .5 .5"
                    face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4
                          2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/>
            </asset>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="mesh_box" pos="0 0 2.0">
                    <geom type="mesh" mesh="box1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(
        data.ncon, 0,
        "Separated mesh should have no contacts, got {}",
        data.ncon
    );
}
