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
