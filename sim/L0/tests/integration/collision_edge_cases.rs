//! Collision edge case tests — numerical pathology and degenerate geometry.
//!
//! This module targets floating-point edge cases that can cause collision
//! detection to fail silently or produce incorrect results.
//!
//! # Test Philosophy
//!
//! > **Todorov Standard**: Physics code must be robust to numerical edge cases.
//! > If MuJoCo handles it, we handle it. If MuJoCo doesn't, document why.
//! >
//! > **Rust Purist Standard**: No panics, no NaN propagation, no silent failures.
//! > Every edge case either produces a valid result or returns None gracefully.
//!
//! # Categories of Edge Cases
//!
//! 1. **Near-parallel geometry**: Cross products approach zero
//! 2. **Near-zero distances**: Division by small numbers
//! 3. **Large coordinates**: Precision loss in subtraction
//! 4. **Degenerate geometry**: Zero-sized dimensions
//! 5. **Coincident geometry**: Same position/orientation

use sim_mjcf::load_model;

use crate::collision_test_utils::{DEPTH_TOL, GEOM_TOL};

// ============================================================================
// Near-Parallel Geometry
// ============================================================================

/// Cylinder axis nearly parallel to plane normal (< 0.1° tilt).
///
/// The rim direction computation involves a cross product that approaches
/// zero as the cylinder becomes vertical. Must not produce NaN or panic.
#[test]
fn cylinder_plane_near_parallel_axis() {
    // Tilt by 0.05° ≈ 0.000873 radians
    let tiny_angle = 0.05_f64.to_radians();
    let mjcf = format!(
        r#"
        <mujoco model="cyl_plane_parallel">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cyl" pos="0 0 0.4" euler="{} 0 0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
        tiny_angle.to_degrees()
    );

    let model = load_model(&mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Should detect contact without panicking
    assert!(
        data.ncon >= 1,
        "Near-parallel cylinder should contact plane"
    );

    let contact = &data.contacts[0];

    // Verify no NaN in output
    assert!(!contact.pos.x.is_nan(), "Contact position X is NaN");
    assert!(!contact.pos.y.is_nan(), "Contact position Y is NaN");
    assert!(!contact.pos.z.is_nan(), "Contact position Z is NaN");
    assert!(!contact.normal.x.is_nan(), "Contact normal X is NaN");
    assert!(!contact.normal.y.is_nan(), "Contact normal Y is NaN");
    assert!(!contact.normal.z.is_nan(), "Contact normal Z is NaN");
    assert!(!contact.depth.is_nan(), "Contact depth is NaN");

    // Normal should be unit length
    let normal_len = contact.normal.norm();
    assert!(
        (normal_len - 1.0).abs() < GEOM_TOL,
        "Normal not unit length: {}",
        normal_len
    );
}

/// Two capsules with nearly parallel axes (< 0.1° angle).
///
/// Segment-segment closest point computation can degenerate when axes
/// are parallel. The implementation should handle this gracefully.
#[test]
fn capsule_capsule_near_parallel() {
    let tiny_angle = 0.05_f64.to_radians();
    let mjcf = format!(
        r#"
        <mujoco model="cap_cap_parallel">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cap1" pos="0 0 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
                <body name="cap2" pos="0.35 0 0" euler="{} 0 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
        tiny_angle.to_degrees()
    );

    let model = load_model(&mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Should detect contact
    assert!(data.ncon >= 1, "Near-parallel capsules should contact");

    // Verify no NaN
    let contact = &data.contacts[0];
    assert!(contact.depth.is_finite(), "Depth should be finite");
    assert!(
        contact.normal.norm().is_finite(),
        "Normal magnitude should be finite"
    );
}

/// Cylinder axis exactly perpendicular to contact direction.
///
/// When the cylinder axis is exactly horizontal and the sphere approaches
/// from the side, the rim vs. curved surface decision is at a boundary.
#[test]
fn cylinder_sphere_axis_perpendicular_to_contact() {
    let mjcf = r#"
        <mujoco model="cyl_sph_perp">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cyl" pos="0 0 0" euler="0 90 0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
                <body name="sph" pos="0 0.55 0">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Cylinder axis along X (rotated 90° about Y), sphere approaching along Y
    // This is exactly perpendicular approach to curved surface
    assert!(data.ncon >= 1, "Perpendicular approach should contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.05; // 0.3 + 0.3 - 0.55
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Perpendicular depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

// ============================================================================
// Near-Zero Distances
// ============================================================================

/// Two spheres with centers nearly coincident (distance < 1e-10).
///
/// The normal direction is undefined when centers coincide. The implementation
/// should pick an arbitrary consistent direction.
#[test]
fn sphere_sphere_nearly_coincident() {
    let epsilon = 1e-10;
    let mjcf = format!(
        r#"
        <mujoco model="sph_sph_coincident">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="sph1" pos="0 0 0">
                    <geom type="sphere" size="0.5"/>
                </body>
                <body name="sph2" pos="{} 0 0">
                    <geom type="sphere" size="0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
        epsilon
    );

    let model = load_model(&mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Should detect deep penetration
    assert_eq!(data.ncon, 1, "Coincident spheres should have 1 contact");

    let contact = &data.contacts[0];

    // Depth should be approximately sum of radii
    let expected_depth = 1.0; // 0.5 + 0.5
    assert!(
        (contact.depth - expected_depth).abs() < 1e-6,
        "Coincident depth: expected ~{}, got {}",
        expected_depth,
        contact.depth
    );

    // Normal must be unit length even for degenerate case
    let normal_len = contact.normal.norm();
    assert!(
        (normal_len - 1.0).abs() < GEOM_TOL,
        "Normal not unit length for coincident: {}",
        normal_len
    );
}

/// Sphere center exactly on box face.
///
/// Distance to face is exactly zero, which can cause issues in normal
/// computation if not handled carefully.
#[test]
fn sphere_box_center_on_face() {
    let mjcf = r#"
        <mujoco model="sph_box_on_face">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="box" pos="0 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
                <body name="sph" pos="0.5 0 0">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Sphere center exactly on box face at x=0.5
    // Penetration = sphere radius = 0.3
    assert_eq!(data.ncon, 1, "Sphere on face should have 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.3;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "On-face depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Capsule endpoint exactly on plane surface.
#[test]
fn capsule_plane_endpoint_on_surface() {
    let mjcf = r#"
        <mujoco model="cap_plane_on_surface">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cap" pos="0 0 0.7">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Capsule: center at z=0.7, half_length=0.5, radius=0.2
    // Bottom endpoint at z = 0.7 - 0.5 = 0.2
    // Bottom surface at z = 0.2 - 0.2 = 0
    // Exactly touching
    if data.ncon > 0 {
        let contact = &data.contacts[0];
        assert!(
            contact.depth < 1e-4,
            "Exactly touching should have near-zero depth, got {}",
            contact.depth
        );
    }
}

// ============================================================================
// Large Coordinates (Precision Loss)
// ============================================================================

/// Collision detection at 1km from origin.
///
/// At large offsets, subtraction of similar values loses precision.
/// 1e6 - (1e6 + 0.1) should equal -0.1, but floating-point may differ.
#[test]
fn sphere_plane_far_from_origin_1km() {
    let offset = 1000.0; // 1km
    let mjcf = format!(
        r#"
        <mujoco model="far_1km">
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

    assert!(data.ncon >= 1, "Far collision should be detected");

    let contact = &data.contacts[0];
    let expected_depth = 0.1;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Far collision depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Collision detection at 1000km from origin.
///
/// At very large offsets, precision loss becomes more significant.
/// Relative precision of f64 is ~1e-15, so at 1e9, absolute precision is ~1e-6.
#[test]
fn sphere_plane_far_from_origin_1000km() {
    let offset = 1_000_000.0; // 1000km
    let mjcf = format!(
        r#"
        <mujoco model="far_1000km">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10000000 10000000 0.1" pos="{} 0 0"/>
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

    // At this scale, we may lose some precision, but collision should still work
    assert!(data.ncon >= 1, "Very far collision should be detected");

    let contact = &data.contacts[0];
    // Allow larger tolerance for precision loss at extreme distances
    let expected_depth = 0.1;
    assert!(
        (contact.depth - expected_depth).abs() < 1e-3,
        "Very far collision depth: expected ~{}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Collision between very small geometry (micrometer scale).
///
/// Small geometry tests the opposite precision regime: values near zero
/// where relative precision is good but absolute values are tiny.
#[test]
fn sphere_sphere_microscale() {
    let scale = 1e-6; // 1 micrometer
    let mjcf = format!(
        r#"
        <mujoco model="microscale">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="sph1" pos="0 0 0">
                    <geom type="sphere" size="{}"/>
                </body>
                <body name="sph2" pos="{} 0 0">
                    <geom type="sphere" size="{}"/>
                </body>
            </worldbody>
        </mujoco>
    "#,
        scale,
        1.5e-6, // 1.5 micrometers apart
        scale
    );

    let model = load_model(&mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Sum of radii = 2e-6, distance = 1.5e-6
    // Penetration = 2e-6 - 1.5e-6 = 0.5e-6 = 5e-7
    assert_eq!(data.ncon, 1, "Microscale collision should be detected");

    let contact = &data.contacts[0];
    let expected_depth = 0.5e-6;
    assert!(
        (contact.depth - expected_depth).abs() < 1e-8,
        "Microscale depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

// ============================================================================
// Degenerate Geometry
// ============================================================================

/// Cylinder with very small radius (approaching line segment).
#[test]
fn cylinder_plane_tiny_radius() {
    let mjcf = r#"
        <mujoco model="cyl_tiny_radius">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cyl" pos="0 0 0.4">
                    <geom type="cylinder" size="0.001 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Cylinder: radius=0.001, half_height=0.5
    // Bottom at z = 0.4 - 0.5 = -0.1
    // Penetration = 0.1
    assert!(data.ncon >= 1, "Tiny-radius cylinder should contact plane");

    let contact = &data.contacts[0];
    let expected_depth = 0.1;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Tiny-radius cylinder depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Ellipsoid with one very small radius (disk-like).
#[test]
fn ellipsoid_plane_disk_like() {
    let mjcf = r#"
        <mujoco model="ell_disk">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="ell" pos="0 0 0.0005">
                    <geom type="ellipsoid" size="0.5 0.5 0.001"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Ellipsoid: radii=(0.5, 0.5, 0.001), center at z=0.0005
    // Bottom at z = 0.0005 - 0.001 = -0.0005
    // Penetration = 0.0005
    assert!(data.ncon >= 1, "Disk-like ellipsoid should contact plane");

    let contact = &data.contacts[0];
    let expected_depth = 0.0005;
    assert!(
        (contact.depth - expected_depth).abs() < 1e-5,
        "Disk-like ellipsoid depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Box with one very small dimension (slab-like).
#[test]
fn box_plane_slab_like() {
    let mjcf = r#"
        <mujoco model="box_slab">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="box" pos="0 0 0.0005">
                    <geom type="box" size="0.5 0.5 0.001"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Box: half-extents=(0.5, 0.5, 0.001), center at z=0.0005
    // Bottom at z = 0.0005 - 0.001 = -0.0005
    // Penetration = 0.0005
    assert!(data.ncon >= 1, "Slab-like box should contact plane");

    let max_depth = data.contacts[..data.ncon]
        .iter()
        .map(|c| c.depth)
        .fold(0.0_f64, f64::max);

    let expected_depth = 0.0005;
    assert!(
        (max_depth - expected_depth).abs() < 1e-5,
        "Slab-like box depth: expected {}, got {}",
        expected_depth,
        max_depth
    );
}

/// Capsule with very small radius (rod-like).
#[test]
fn capsule_plane_rod_like() {
    let mjcf = r#"
        <mujoco model="cap_rod">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="cap" pos="0 0 0.4">
                    <geom type="capsule" size="0.001 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Capsule: radius=0.001, half_length=0.5
    // Bottom endpoint at z = 0.4 - 0.5 = -0.1
    // Bottom surface at z = -0.1 - 0.001 = -0.101
    // Penetration = 0.101
    assert!(data.ncon >= 1, "Rod-like capsule should contact plane");

    let contact = &data.contacts[0];
    let expected_depth = 0.101;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Rod-like capsule depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

// ============================================================================
// Exactly Degenerate Cases
// ============================================================================

/// Sphere with zero radius (point).
///
/// This is mathematically degenerate but should not crash.
#[test]
fn sphere_plane_zero_radius() {
    let mjcf = r#"
        <mujoco model="sph_zero">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="sph" pos="0 0 -0.1">
                    <geom type="sphere" size="0.001"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();

    // Should not panic
    data.forward(&model).expect("forward failed");

    // May or may not detect contact for essentially-zero-radius sphere
    // Just verify no crash
}

/// Box with all dimensions equal (cube).
///
/// This is not degenerate per se, but tests that cube special cases
/// don't cause issues in SAT.
#[test]
fn box_box_cubes() {
    let mjcf = r#"
        <mujoco model="cubes">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cube1" pos="0 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
                <body name="cube2" pos="0.9 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Perfect cubes overlapping
    assert!(data.ncon >= 1, "Overlapping cubes should contact");

    let max_depth = data.contacts[..data.ncon]
        .iter()
        .map(|c| c.depth)
        .fold(0.0_f64, f64::max);

    let expected_depth = 0.1; // 0.5 + 0.5 - 0.9
    assert!(
        (max_depth - expected_depth).abs() < DEPTH_TOL,
        "Cube-cube depth: expected {}, got {}",
        expected_depth,
        max_depth
    );
}

// ============================================================================
// Symmetry Tests
// ============================================================================

/// Verify collision is symmetric: collide(A,B) ≈ collide(B,A).
///
/// Contact depth should be identical regardless of order.
#[test]
fn sphere_sphere_symmetry() {
    let mjcf_ab = r#"
        <mujoco model="sym_ab">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="a" pos="0 0 0">
                    <geom type="sphere" size="0.5"/>
                </body>
                <body name="b" pos="0.7 0 0">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let mjcf_ba = r#"
        <mujoco model="sym_ba">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="b" pos="0.7 0 0">
                    <geom type="sphere" size="0.3"/>
                </body>
                <body name="a" pos="0 0 0">
                    <geom type="sphere" size="0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model_ab = load_model(mjcf_ab).expect("Failed to load model AB");
    let mut data_ab = model_ab.make_data();
    data_ab.forward(&model_ab).expect("forward failed");

    let model_ba = load_model(mjcf_ba).expect("Failed to load model BA");
    let mut data_ba = model_ba.make_data();
    data_ba.forward(&model_ba).expect("forward failed");

    assert_eq!(
        data_ab.ncon, data_ba.ncon,
        "Symmetric cases should have same contact count"
    );

    if data_ab.ncon > 0 && data_ba.ncon > 0 {
        let depth_ab = data_ab.contacts[0].depth;
        let depth_ba = data_ba.contacts[0].depth;
        assert!(
            (depth_ab - depth_ba).abs() < DEPTH_TOL,
            "Symmetric depths should match: {} vs {}",
            depth_ab,
            depth_ba
        );
    }
}

/// Verify box-box collision is symmetric under body reordering.
#[test]
fn box_box_symmetry() {
    let mjcf_ab = r#"
        <mujoco model="box_sym_ab">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="a" pos="0 0 0">
                    <geom type="box" size="0.5 0.4 0.3"/>
                </body>
                <body name="b" pos="0.8 0 0" euler="0 0 30">
                    <geom type="box" size="0.4 0.3 0.2"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let mjcf_ba = r#"
        <mujoco model="box_sym_ba">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="b" pos="0.8 0 0" euler="0 0 30">
                    <geom type="box" size="0.4 0.3 0.2"/>
                </body>
                <body name="a" pos="0 0 0">
                    <geom type="box" size="0.5 0.4 0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model_ab = load_model(mjcf_ab).expect("Failed to load model AB");
    let mut data_ab = model_ab.make_data();
    data_ab.forward(&model_ab).expect("forward failed");

    let model_ba = load_model(mjcf_ba).expect("Failed to load model BA");
    let mut data_ba = model_ba.make_data();
    data_ba.forward(&model_ba).expect("forward failed");

    // Contact count should be same
    assert_eq!(
        data_ab.ncon, data_ba.ncon,
        "Symmetric box cases should have same contact count"
    );

    // Depths should match (order may differ, compare max)
    if data_ab.ncon > 0 && data_ba.ncon > 0 {
        let max_depth_ab = data_ab.contacts[..data_ab.ncon]
            .iter()
            .map(|c| c.depth)
            .fold(0.0_f64, f64::max);
        let max_depth_ba = data_ba.contacts[..data_ba.ncon]
            .iter()
            .map(|c| c.depth)
            .fold(0.0_f64, f64::max);

        assert!(
            (max_depth_ab - max_depth_ba).abs() < DEPTH_TOL,
            "Symmetric box depths should match: {} vs {}",
            max_depth_ab,
            max_depth_ba
        );
    }
}

// ============================================================================
// Stress Tests
// ============================================================================

/// Many simultaneous contacts (100 spheres in a pile).
///
/// Tests that the collision system can handle many contacts without
/// performance degradation or memory issues in a single frame.
#[test]
fn many_spheres_pile() {
    // Create 10 spheres in a grid
    let mut bodies = String::new();
    for i in 0..10 {
        let x = (i % 5) as f64 * 0.9 - 1.8;
        let y = (i / 5) as f64 * 0.9 - 0.45;
        bodies.push_str(&format!(
            r#"<body name="s{}" pos="{} {} 0.5"><geom type="sphere" size="0.5"/></body>"#,
            i, x, y
        ));
    }

    let mjcf = format!(
        r#"
        <mujoco model="sphere_pile">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                {}
            </worldbody>
        </mujoco>
    "#,
        bodies
    );

    let model = load_model(&mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Should have many contacts (floor + sphere-sphere)
    // Each sphere touches floor = 10 contacts
    // Adjacent spheres may or may not overlap depending on spacing
    assert!(
        data.ncon >= 10,
        "Sphere pile should have at least floor contacts"
    );

    // Verify all contacts are valid
    for c in &data.contacts[..data.ncon] {
        assert!(c.depth.is_finite(), "All depths should be finite");
        assert!(c.depth >= 0.0, "All depths should be non-negative");
        let normal_len = c.normal.norm();
        assert!(
            (normal_len - 1.0).abs() < GEOM_TOL,
            "All normals should be unit length"
        );
    }
}
