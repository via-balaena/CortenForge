//! Primitive-primitive collision tests — analytical collision pairs.
//!
//! This module validates collision detection between primitive geometry pairs
//! that have analytical (closed-form) solutions, avoiding GJK/EPA.
//!
//! # Test Philosophy
//!
//! > **Todorov Standard**: Analytical solutions are preferred over iterative ones.
//! > Every analytical collision function must match its mathematical derivation.
//! >
//! > **Rust Purist Standard**: Tests cover all branches in collision code.
//! > Side, cap, rim, edge — every geometric configuration.
//!
//! # Analytical Collision Pairs
//!
//! | Pair | Algorithm | Notes |
//! |------|-----------|-------|
//! | Sphere-Sphere | Distance check | Trivial |
//! | Sphere-Capsule | Point-segment distance | Endpoint cases |
//! | Sphere-Box | Closest point on box | Corner/edge/face |
//! | Capsule-Capsule | Segment-segment distance | Parallel case |
//! | Capsule-Box | Segment-box distance | Complex |
//! | Box-Box | SAT (15 axes) | Edge-edge cases |
//! | Cylinder-Sphere | Side/cap/rim | Analytical |
//! | Cylinder-Capsule | Axis-axis distance | Analytical |

use sim_core::{GeomType, SdfGrid, ShapeSphere};
use sim_mjcf::load_model;
use std::sync::Arc;

use crate::collision_test_utils::{DEPTH_TOL, GEOM_TOL};

// ============================================================================
// Sphere-Sphere Tests
// ============================================================================

/// Two spheres overlapping along X axis.
///
/// Configuration:
/// - Sphere 1: radius 0.5, center at origin
/// - Sphere 2: radius 0.3, center at (0.7, 0, 0)
/// - Distance = 0.7, sum of radii = 0.8
/// - Penetration = 0.8 - 0.7 = 0.1
#[test]
fn sphere_sphere_overlap_x_axis() {
    let mjcf = r#"
        <mujoco model="sphere_sphere_x">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="s1" pos="0 0 0">
                    <geom type="sphere" size="0.5"/>
                </body>
                <body name="s2" pos="0.7 0 0">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(data.ncon, 1, "Expected exactly 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.1;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Sphere-sphere depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );

    // Normal should point along X (from sphere 1 to sphere 2)
    assert!(
        contact.normal.x.abs() > 0.99,
        "Normal should be along X axis, got {:?}",
        contact.normal
    );
}

/// Two spheres overlapping along diagonal.
#[test]
fn sphere_sphere_overlap_diagonal() {
    let mjcf = r#"
        <mujoco model="sphere_sphere_diag">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="s1" pos="0 0 0">
                    <geom type="sphere" size="0.5"/>
                </body>
                <body name="s2" pos="0.4 0.4 0.4">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Distance = sqrt(0.16 + 0.16 + 0.16) = sqrt(0.48) ≈ 0.693
    // Sum of radii = 0.8
    // Penetration = 0.8 - 0.693 ≈ 0.107
    assert_eq!(data.ncon, 1, "Expected exactly 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.8 - (0.48_f64).sqrt();
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Diagonal sphere-sphere depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Two spheres just touching (zero penetration).
#[test]
fn sphere_sphere_touching() {
    let mjcf = r#"
        <mujoco model="sphere_sphere_touch">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="s1" pos="0 0 0">
                    <geom type="sphere" size="0.5"/>
                </body>
                <body name="s2" pos="0.8 0 0">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Distance = sum of radii = 0.8, so exactly touching
    // May or may not report contact depending on implementation
    if data.ncon > 0 {
        let contact = &data.contacts[0];
        assert!(
            contact.depth < 1e-4,
            "Touching spheres should have near-zero depth, got {}",
            contact.depth
        );
    }
}

/// Two spheres separated (no contact).
#[test]
fn sphere_sphere_separated() {
    let mjcf = r#"
        <mujoco model="sphere_sphere_sep">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="s1" pos="0 0 0">
                    <geom type="sphere" size="0.5"/>
                </body>
                <body name="s2" pos="1.0 0 0">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(data.ncon, 0, "Separated spheres should have no contact");
}

/// Two identical spheres at same location (fully overlapping).
#[test]
fn sphere_sphere_coincident() {
    let mjcf = r#"
        <mujoco model="sphere_sphere_coincident">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="s1" pos="0 0 0">
                    <geom type="sphere" size="0.5"/>
                </body>
                <body name="s2" pos="0 0 0">
                    <geom type="sphere" size="0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Coincident spheres: penetration = sum of radii = 1.0
    // Normal direction is undefined (any direction valid)
    assert_eq!(data.ncon, 1, "Coincident spheres should have 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 1.0;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Coincident spheres depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );

    // Normal must still be unit length
    let normal_len = contact.normal.norm();
    assert!(
        (normal_len - 1.0).abs() < GEOM_TOL,
        "Normal should be unit length, got {}",
        normal_len
    );
}

// ============================================================================
// Sphere-Capsule Tests
// ============================================================================

/// Sphere touching capsule side (midpoint contact).
#[test]
fn sphere_capsule_side_contact() {
    let mjcf = r#"
        <mujoco model="sphere_capsule_side">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cap" pos="0 0 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
                <body name="sph" pos="0.45 0 0">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Capsule: radius=0.2, half_length=0.5, axis along Z
    // Sphere: radius=0.3, center at (0.45, 0, 0)
    // Distance from capsule axis to sphere center = 0.45
    // Penetration = 0.2 + 0.3 - 0.45 = 0.05
    assert_eq!(data.ncon, 1, "Expected 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.05;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Sphere-capsule side depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );

    // Normal should point along X
    assert!(
        contact.normal.x.abs() > 0.99,
        "Normal should be along X, got {:?}",
        contact.normal
    );
}

/// Sphere touching capsule endpoint (hemisphere contact).
#[test]
fn sphere_capsule_endpoint_contact() {
    let mjcf = r#"
        <mujoco model="sphere_capsule_endpoint">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cap" pos="0 0 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
                <body name="sph" pos="0 0 0.95">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Capsule endpoint at z=0.5, endpoint sphere surface at z=0.7
    // Sphere center at z=0.95, sphere surface at z=0.65
    // Distance from endpoint center (0,0,0.5) to sphere center (0,0,0.95) = 0.45
    // Penetration = 0.2 + 0.3 - 0.45 = 0.05
    assert_eq!(data.ncon, 1, "Expected 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.05;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Sphere-capsule endpoint depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );

    // Normal should point along +Z
    assert!(
        contact.normal.z > 0.99,
        "Normal should point +Z, got {:?}",
        contact.normal
    );
}

/// Sphere separated from capsule.
#[test]
fn sphere_capsule_separated() {
    let mjcf = r#"
        <mujoco model="sphere_capsule_sep">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cap" pos="0 0 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
                <body name="sph" pos="1.0 0 0">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(
        data.ncon, 0,
        "Separated sphere-capsule should have no contact"
    );
}

// ============================================================================
// Capsule-Capsule Tests
// ============================================================================

/// Two parallel capsules side by side.
#[test]
fn capsule_capsule_parallel_side() {
    let mjcf = r#"
        <mujoco model="capsule_capsule_parallel">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cap1" pos="0 0 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
                <body name="cap2" pos="0.35 0 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Both capsules have radius 0.2, parallel axes along Z
    // Distance between axes = 0.35
    // Penetration = 0.2 + 0.2 - 0.35 = 0.05
    assert_eq!(data.ncon, 1, "Expected 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.05;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Parallel capsule depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Two perpendicular capsules (T-bone configuration).
#[test]
fn capsule_capsule_perpendicular() {
    let mjcf = r#"
        <mujoco model="capsule_capsule_perp">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cap1" pos="0 0 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
                <body name="cap2" pos="0.35 0 0" euler="0 90 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Capsule 1: axis along Z, center at origin, radius 0.2, half-length 0.5
    // Capsule 2: axis along X (rotated 90° about Y), center at (0.35, 0, 0)
    //
    // With euler properly applied, the capsule axes actually INTERSECT at the
    // origin (one spans Z from -0.5 to +0.5, the other spans X from -0.15 to 0.85,
    // and they cross at (0,0,0)). The distance between segments is 0.
    //
    // For this degenerate case, the collision code picks +Z as the arbitrary
    // normal. Penetration = sum_radii = 0.4.
    assert_eq!(
        data.ncon, 1,
        "Intersecting capsule axes should have 1 contact"
    );

    let contact = &data.contacts[0];
    let expected_depth = 0.4; // sum_radii when segments intersect
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Perpendicular capsule depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Two capsules touching at endpoints.
#[test]
fn capsule_capsule_endpoint_endpoint() {
    let mjcf = r#"
        <mujoco model="capsule_capsule_ends">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cap1" pos="0 0 0">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
                <body name="cap2" pos="0 0 1.35">
                    <geom type="capsule" size="0.2 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Capsule 1: top endpoint at z=0.5
    // Capsule 2: bottom endpoint at z=1.35-0.5=0.85
    // Distance between endpoints = 0.85 - 0.5 = 0.35
    // Distance between endpoint centers = 0.35
    // But wait, for capsules the "surface" includes hemisphere radius
    // Capsule 1 top surface at z = 0.5 + 0.2 = 0.7
    // Capsule 2 bottom surface at z = 0.85 - 0.2 = 0.65
    // Gap = 0.7 - 0.65 = 0.05... that's penetrating!
    // Actually: endpoint-to-endpoint distance = 1.35 - 0 = 1.35 (center to center)
    // Capsule 1 top endpoint center at 0.5, capsule 2 bottom endpoint center at 0.85
    // Distance = 0.35, penetration = 0.4 - 0.35 = 0.05
    assert_eq!(data.ncon, 1, "Expected 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.05;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Endpoint-endpoint capsule depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

// ============================================================================
// Sphere-Box Tests
// ============================================================================

/// Sphere touching box face.
#[test]
fn sphere_box_face_contact() {
    let mjcf = r#"
        <mujoco model="sphere_box_face">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="box" pos="0 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
                <body name="sph" pos="0.75 0 0">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Box face at x=0.5
    // Sphere center at x=0.75, radius=0.3, surface at x=0.45
    // Penetration = 0.5 - 0.45 = 0.05
    assert_eq!(data.ncon, 1, "Expected 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.05;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Sphere-box face depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );

    // Normal should point along +X
    assert!(
        contact.normal.x > 0.99,
        "Normal should point +X, got {:?}",
        contact.normal
    );
}

/// Sphere touching box edge.
#[test]
fn sphere_box_edge_contact() {
    let mjcf = r#"
        <mujoco model="sphere_box_edge">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="box" pos="0 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
                <body name="sph" pos="0.6 0.6 0">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Box corner at (0.5, 0.5, z), sphere center at (0.6, 0.6, 0)
    // Distance to edge (at y=0.5, x=0.5) = sqrt((0.1)² + (0.1)²) = sqrt(0.02) ≈ 0.141
    // Penetration = 0.3 - 0.141 ≈ 0.159
    assert_eq!(data.ncon, 1, "Expected 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.3 - (0.02_f64).sqrt();
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Sphere-box edge depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Sphere touching box corner.
#[test]
fn sphere_box_corner_contact() {
    let mjcf = r#"
        <mujoco model="sphere_box_corner">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="box" pos="0 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
                <body name="sph" pos="0.6 0.6 0.6">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Box corner at (0.5, 0.5, 0.5), sphere center at (0.6, 0.6, 0.6)
    // Distance to corner = sqrt(3 × 0.1²) = sqrt(0.03) ≈ 0.173
    // Penetration = 0.3 - 0.173 ≈ 0.127
    assert_eq!(data.ncon, 1, "Expected 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.3 - (0.03_f64).sqrt();
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Sphere-box corner depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

// ============================================================================
// Box-Box Tests
// ============================================================================

/// Two boxes overlapping along X axis.
#[test]
fn box_box_face_face() {
    let mjcf = r#"
        <mujoco model="box_box_face">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="box1" pos="0 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
                <body name="box2" pos="0.9 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Box 1 right face at x=0.5
    // Box 2 left face at x=0.9-0.5=0.4
    // Penetration = 0.5 - 0.4 = 0.1
    assert!(data.ncon >= 1, "Expected at least 1 contact");

    let max_depth = data.contacts[..data.ncon]
        .iter()
        .map(|c| c.depth)
        .fold(0.0_f64, f64::max);

    let expected_depth = 0.1;
    assert!(
        (max_depth - expected_depth).abs() < DEPTH_TOL,
        "Box-box face depth: expected {}, got {}",
        expected_depth,
        max_depth
    );
}

/// Two boxes with edge-edge contact (one rotated 45°).
#[test]
fn box_box_edge_edge() {
    let mjcf = r#"
        <mujoco model="box_box_edge">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="box1" pos="0 0 0">
                    <geom type="box" size="0.3 0.3 0.3"/>
                </body>
                <body name="box2" pos="0.5 0 0" euler="0 0 45">
                    <geom type="box" size="0.3 0.3 0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Box 2 rotated 45° about Z, its corner extends further toward box 1
    // This creates an edge-edge or vertex-edge contact
    // The exact depth depends on SAT implementation
    assert!(data.ncon >= 1, "Expected at least 1 contact");

    // Just verify contact is detected and has positive depth
    let contact = &data.contacts[0];
    assert!(
        contact.depth > 0.0,
        "Box-box edge contact should have positive depth"
    );
}

/// Two boxes separated.
#[test]
fn box_box_separated() {
    let mjcf = r#"
        <mujoco model="box_box_sep">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="box1" pos="0 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
                <body name="box2" pos="1.5 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    assert_eq!(data.ncon, 0, "Separated boxes should have no contact");
}

// ============================================================================
// Phase 2: Box-Box Multi-Contact Tests
// ============================================================================

/// Aligned boxes face-face → 4 contacts (clipped incident face has 4 corners).
///
/// Configuration: two 1×1×1 boxes along X, overlapping by 0.1.
/// Reference face is box1's +X face. Incident face is box2's -X face (1×1).
/// Clipped polygon = full incident face (fits inside reference face).
/// All 4 vertices at same depth = 0.1.
#[test]
fn box_box_face_face_4_contacts() {
    let mjcf = r#"
        <mujoco model="box_box_ff_4">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="box1" pos="0 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
                <body name="box2" pos="0.9 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Two aligned cubes: incident face (1×1) fits inside reference face (1×1)
    // → 4 contacts, all at depth 0.1
    assert_eq!(
        data.ncon, 4,
        "Aligned box-box face-face should produce 4 contacts, got {}",
        data.ncon
    );

    let expected_depth = 0.1;
    for (i, c) in data.contacts[..4].iter().enumerate() {
        assert!(
            (c.depth - expected_depth).abs() < DEPTH_TOL,
            "Contact {i}: expected depth {expected_depth}, got {}",
            c.depth
        );
    }
}

/// Box stacked on box (gravity, free joint) stays stable with multi-contact.
#[test]
fn box_box_stacked_stable() {
    let mjcf = r#"
        <mujoco model="box_stack_stable">
            <option gravity="0 0 -9.81" solver="Newton" cone="elliptic"
                    iterations="100" tolerance="1e-8"/>
            <worldbody>
                <geom type="plane" size="5 5 0.1"/>
                <body name="box1" pos="0 0 0.11">
                    <joint type="free"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"
                          friction="0.5 0.5 0.005"/>
                </body>
                <body name="box2" pos="0 0 0.31">
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

    // Both boxes should settle near their starting z positions
    let box1_z = data.qpos[2]; // free joint qpos[2] = z
    let box2_z = data.qpos[9]; // second free body qpos[7+2] = z

    assert!(
        box1_z > 0.05 && box1_z < 0.20,
        "Box1 should settle near z=0.1, got {box1_z:.4}"
    );
    assert!(
        box2_z > 0.20 && box2_z < 0.40,
        "Box2 should settle near z=0.3, got {box2_z:.4}"
    );
}

/// Edge-edge contact (box rotated 45°) → 1 contact.
#[test]
fn box_box_edge_edge_1_contact() {
    let mjcf = r#"
        <mujoco model="box_box_ee_1">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="box1" pos="0 0 0">
                    <geom type="box" size="0.3 0.3 0.3"/>
                </body>
                <body name="box2" pos="0.5 0 0" euler="0 0 45">
                    <geom type="box" size="0.3 0.3 0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Rotated 45° should give edge-edge (1 contact) or face-edge (few contacts)
    assert!(
        data.ncon >= 1,
        "Edge-edge box contact should produce at least 1 contact, got {}",
        data.ncon
    );

    // Contact depth should be positive
    assert!(
        data.contacts[0].depth > 0.0,
        "Edge contact depth should be positive"
    );
}

// ============================================================================
// Cylinder-Sphere Tests
// ============================================================================

/// Sphere touching cylinder side (radial contact).
#[test]
fn cylinder_sphere_side_contact() {
    let mjcf = r#"
        <mujoco model="cyl_sphere_side">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cyl" pos="0 0 0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
                <body name="sph" pos="0.55 0 0">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Cylinder radius=0.3, axis along Z
    // Sphere center at (0.55, 0, 0), radius=0.3
    // Distance from axis = 0.55
    // Penetration = 0.3 + 0.3 - 0.55 = 0.05
    assert_eq!(data.ncon, 1, "Expected 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.05;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Cylinder-sphere side depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );

    // Normal should point along +X
    assert!(
        contact.normal.x > 0.99,
        "Normal should point +X, got {:?}",
        contact.normal
    );
}

/// Sphere touching cylinder cap.
#[test]
fn cylinder_sphere_cap_contact() {
    let mjcf = r#"
        <mujoco model="cyl_sphere_cap">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cyl" pos="0 0 0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
                <body name="sph" pos="0 0 0.75">
                    <geom type="sphere" size="0.3"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Cylinder cap at z=0.5
    // Sphere center at z=0.75, radius=0.3, surface at z=0.45
    // Penetration = 0.5 - 0.45 = 0.05
    assert_eq!(data.ncon, 1, "Expected 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.05;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Cylinder-sphere cap depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );

    // Normal should point along +Z
    assert!(
        contact.normal.z > 0.99,
        "Normal should point +Z, got {:?}",
        contact.normal
    );
}

/// Sphere touching cylinder rim (edge of cap).
#[test]
fn cylinder_sphere_rim_contact() {
    let mjcf = r#"
        <mujoco model="cyl_sphere_rim">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cyl" pos="0 0 0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
                <body name="sph" pos="0.4 0 0.6">
                    <geom type="sphere" size="0.2"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Rim point at (0.3, 0, 0.5)
    // Sphere center at (0.4, 0, 0.6)
    // Distance = sqrt((0.1)² + (0.1)²) = sqrt(0.02) ≈ 0.141
    // Penetration = 0.2 - 0.141 ≈ 0.059
    assert_eq!(data.ncon, 1, "Expected 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.2 - (0.02_f64).sqrt();
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Cylinder-sphere rim depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

// ============================================================================
// Cylinder-Capsule Tests
// ============================================================================

/// Capsule parallel to cylinder, side contact.
#[test]
fn cylinder_capsule_parallel_side() {
    let mjcf = r#"
        <mujoco model="cyl_cap_parallel">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cyl" pos="0 0 0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
                <body name="cap" pos="0.55 0 0">
                    <geom type="capsule" size="0.2 0.4"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Both axes along Z
    // Distance between axes = 0.55
    // Penetration = 0.3 + 0.2 - 0.55 = -0.05 (no contact!)
    // Wait, that's negative. Let me recalculate...
    // Cylinder radius = 0.3, capsule radius = 0.2
    // Distance = 0.55, sum of radii = 0.5
    // So they're separated by 0.05
    assert_eq!(data.ncon, 0, "These should NOT contact");
}

/// Capsule parallel to cylinder, overlapping.
#[test]
fn cylinder_capsule_parallel_overlap() {
    let mjcf = r#"
        <mujoco model="cyl_cap_parallel_overlap">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cyl" pos="0 0 0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
                <body name="cap" pos="0.45 0 0">
                    <geom type="capsule" size="0.2 0.4"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Distance = 0.45, sum of radii = 0.5
    // Penetration = 0.5 - 0.45 = 0.05
    assert_eq!(data.ncon, 1, "Expected 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.05;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Cylinder-capsule parallel depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

/// Capsule perpendicular to cylinder.
#[test]
fn cylinder_capsule_perpendicular() {
    let mjcf = r#"
        <mujoco model="cyl_cap_perp">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="cyl" pos="0 0 0">
                    <geom type="cylinder" size="0.3 0.5"/>
                </body>
                <body name="cap" pos="0.45 0 0" euler="0 90 0">
                    <geom type="capsule" size="0.2 0.4"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Cylinder at origin, axis Z, radius 0.3, half-length 0.5
    // Capsule at (0.45, 0, 0), axis X (after 90° Y rotation), radius 0.2, half-length 0.4
    //
    // Capsule spine: (0.05, 0, 0) to (0.85, 0, 0)
    // Cylinder axis: Z line at (0, 0, z)
    //
    // The closest approach is from (0, 0, 0) on cylinder axis to (0.05, 0, 0) on capsule.
    // Distance = 0.05
    // Penetration = 0.3 + 0.2 - 0.05 = 0.45
    assert_eq!(data.ncon, 1, "Expected 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.45;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Cylinder-capsule perpendicular depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

// ============================================================================
// Capsule-Box Tests
// ============================================================================

/// Capsule touching box face.
#[test]
fn capsule_box_face_contact() {
    let mjcf = r#"
        <mujoco model="cap_box_face">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="box" pos="0 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
                <body name="cap" pos="0.75 0 0">
                    <geom type="capsule" size="0.2 0.4"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Box face at x=0.5
    // Capsule center at x=0.75, radius=0.2, surface at x=0.55
    // Penetration = 0.55 - 0.5 = 0.05... wait, that's the wrong direction
    // Capsule surface CLOSEST to box is at x = 0.75 - 0.2 = 0.55
    // Box face is at x = 0.5
    // So they're separated by 0.05
    assert_eq!(data.ncon, 0, "These should NOT contact");
}

/// Capsule touching box face (overlapping).
#[test]
fn capsule_box_face_overlap() {
    let mjcf = r#"
        <mujoco model="cap_box_face_overlap">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="box" pos="0 0 0">
                    <geom type="box" size="0.5 0.5 0.5"/>
                </body>
                <body name="cap" pos="0.65 0 0">
                    <geom type="capsule" size="0.2 0.4"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Capsule surface at x = 0.65 - 0.2 = 0.45
    // Box face at x = 0.5
    // Penetration = 0.5 - 0.45 = 0.05
    assert_eq!(data.ncon, 1, "Expected 1 contact");

    let contact = &data.contacts[0];
    let expected_depth = 0.05;
    assert!(
        (contact.depth - expected_depth).abs() < DEPTH_TOL,
        "Capsule-box face depth: expected {}, got {}",
        expected_depth,
        contact.depth
    );
}

// ============================================================================
// Multi-Contact Scenarios
// ============================================================================

/// Stack of three spheres — tests contact constraint stability.
#[test]
fn sphere_stack_contacts() {
    let mjcf = r#"
        <mujoco model="sphere_stack">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="ball1" pos="0 0 0.5">
                    <geom type="sphere" size="0.5"/>
                </body>
                <body name="ball2" pos="0 0 1.5">
                    <geom type="sphere" size="0.5"/>
                </body>
                <body name="ball3" pos="0 0 2.5">
                    <geom type="sphere" size="0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Ball 1 touches floor: penetration = 0
    // Ball 2 touches ball 1: penetration = 0
    // Ball 3 touches ball 2: penetration = 0
    // All are exactly touching (no penetration)
    // Implementation may or may not report zero-depth contacts
    // Just verify no deep penetration is reported
    for c in &data.contacts[..data.ncon] {
        assert!(
            c.depth < 1e-3,
            "Stacked spheres should have near-zero depth, got {}",
            c.depth
        );
    }
}

/// Two free-body spheres stacking under gravity — tests dynamic contact stability.
/// KNOWN ISSUE: analytical spheres also fail to stack. The constraint solver
/// doesn't maintain sphere-on-sphere separation with free joints.
/// This is NOT an SDF-specific problem — it affects all geom types.
#[test]
#[ignore] // Known issue: free-body stacking doesn't work yet
fn sphere_stack_dynamic() {
    let mjcf = r#"
        <mujoco model="dynamic_stack">
            <option gravity="0 0 -9810" timestep="0.002"/>
            <worldbody>
                <geom name="floor" type="plane" size="40 40 0.1" solref="0.005 1"/>
                <body name="lower" pos="0 0 5.5">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom type="sphere" size="5" solref="0.005 1"/>
                </body>
                <body name="upper" pos="0 0 15.5">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom type="sphere" size="5" solref="0.005 1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Run 2500 steps (5 seconds of sim)
    for _ in 0..2500 {
        data.step(&model).expect("step failed");
    }

    let z_lo = data.qpos[2];
    let z_up = data.qpos[9];

    eprintln!(
        "dynamic_stack: z_lo={:.3} z_up={:.3} ncon={}",
        z_lo, z_up, data.ncon
    );

    // Lower sphere should rest near z=5 (radius on ground)
    assert!(
        (z_lo - 5.0).abs() < 1.0,
        "lower sphere should rest at z≈5, got {z_lo:.3}"
    );
    // Upper sphere should rest near z=15 (on top of lower)
    assert!(
        (z_up - 15.0).abs() < 2.0,
        "upper sphere should rest at z≈15, got {z_up:.3}"
    );
    // Gap should be ~10 (2 radii)
    let gap = z_up - z_lo;
    assert!((gap - 10.0).abs() < 2.0, "gap should be ~10, got {gap:.3}");
}

/// Same stacking test but at meter-scale (MuJoCo standard units).
/// Tests whether the stacking failure is mm-scale-specific.
#[test]
#[ignore] // Diagnostic test for scale investigation
fn sphere_stack_dynamic_meter_scale() {
    let mjcf = r#"
        <mujoco model="dynamic_stack_m">
            <option gravity="0 0 -9.81" timestep="0.002"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="lower" pos="0 0 0.55">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom type="sphere" size="0.5"/>
                </body>
                <body name="upper" pos="0 0 1.55">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom type="sphere" size="0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    for _ in 0..2500 {
        data.step(&model).expect("step failed");
    }

    let z_lo = data.qpos[2];
    let z_up = data.qpos[9];

    eprintln!(
        "meter_scale_stack: z_lo={:.4} z_up={:.4} ncon={}",
        z_lo, z_up, data.ncon
    );
    eprintln!("  gap={:.4} (expected 1.0)", z_up - z_lo);
}

/// MM-scale with DEFAULT solref (not tightened). Tests whether solref=0.005
/// specifically causes the failure.
#[test]
#[ignore]
fn sphere_stack_dynamic_mm_default_solref() {
    let mjcf = r#"
        <mujoco model="dynamic_stack_mm_default">
            <option gravity="0 0 -9810" timestep="0.002"/>
            <worldbody>
                <geom name="floor" type="plane" size="40 40 0.1"/>
                <body name="lower" pos="0 0 5.5">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom type="sphere" size="5"/>
                </body>
                <body name="upper" pos="0 0 15.5">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom type="sphere" size="5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    for _ in 0..2500 {
        data.step(&model).expect("step failed");
    }

    let z_lo = data.qpos[2];
    let z_up = data.qpos[9];

    eprintln!(
        "mm_default_solref: z_lo={:.3} z_up={:.3} gap={:.3} ncon={}",
        z_lo,
        z_up,
        z_up - z_lo,
        data.ncon
    );
}

/// MM-scale with smaller timestep. Tests whether the issue is Nyquist-related.
#[test]
#[ignore]
fn sphere_stack_dynamic_mm_small_timestep() {
    let mjcf = r#"
        <mujoco model="dynamic_stack_mm_small_dt">
            <option gravity="0 0 -9810" timestep="0.0001"/>
            <worldbody>
                <geom name="floor" type="plane" size="40 40 0.1"/>
                <body name="lower" pos="0 0 5.5">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom type="sphere" size="5"/>
                </body>
                <body name="upper" pos="0 0 15.5">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom type="sphere" size="5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // 50000 steps at 0.0001s = 5 seconds
    for _ in 0..50000 {
        data.step(&model).expect("step failed");
    }

    let z_lo = data.qpos[2];
    let z_up = data.qpos[9];
    eprintln!(
        "mm_small_dt: z_lo={:.3} z_up={:.3} gap={:.3} ncon={}",
        z_lo,
        z_up,
        z_up - z_lo,
        data.ncon
    );
}

/// MM-scale with 0.0005s timestep — compromise between speed and stability.
#[test]
#[ignore]
fn sphere_stack_dynamic_mm_mid_timestep() {
    let mjcf = r#"
        <mujoco model="dynamic_stack_mm_mid_dt">
            <option gravity="0 0 -9810" timestep="0.0005"/>
            <worldbody>
                <geom name="floor" type="plane" size="40 40 0.1"/>
                <body name="lower" pos="0 0 5.5">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom type="sphere" size="5"/>
                </body>
                <body name="upper" pos="0 0 15.5">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom type="sphere" size="5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    for _ in 0..10000 {
        data.step(&model).expect("step failed");
    }

    let z_lo = data.qpos[2];
    let z_up = data.qpos[9];
    eprintln!(
        "mm_mid_dt: z_lo={:.3} z_up={:.3} gap={:.3} ncon={}",
        z_lo,
        z_up,
        z_up - z_lo,
        data.ncon
    );
}

/// SDF-SDF stacking: two SDF spheres should stack vertically.
///
/// This is the critical test for the SDF-SDF stacking blocker.
/// Loads an MJCF model for bodies/joints, then swaps sphere geoms
/// for SDF geoms to test the SDF contact pipeline end-to-end.
#[test]
fn sphere_stack_dynamic_sdf() {
    use nalgebra::Point3;

    let mjcf = r#"
        <mujoco model="dynamic_stack_sdf">
            <option gravity="0 0 -9810" timestep="0.0005"/>
            <worldbody>
                <geom name="floor" type="plane" size="40 40 0.1"/>
                <body name="lower" pos="0 0 5.5">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom name="lo_sphere" type="sphere" size="5"/>
                </body>
                <body name="upper" pos="0 0 15.5">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom name="up_sphere" type="sphere" size="5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let mut model = load_model(mjcf).expect("Failed to load model");

    // Create SDF sphere (radius=5mm, resolution=20, padding=2mm)
    let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 20, 2.0));

    // Replace sphere geoms (indices 1, 2 — index 0 is the floor plane) with SDF
    model
        .shape_data
        .push(Arc::new(ShapeSphere::new(grid.clone(), 5.0)));
    model.shape_data.push(Arc::new(ShapeSphere::new(grid, 5.0)));
    model.nshape = 2;

    for geom_id in 1..=2 {
        model.geom_type[geom_id] = GeomType::Sdf;
        model.geom_shape[geom_id] = Some(geom_id - 1);
    }

    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Simulate 1 second (2000 steps at 0.0005s) — enough for impact + settling
    for step in 0..2000 {
        data.step(&model).expect("step failed");
        // Print diagnostics at key moments
        if step == 1999 {
            eprintln!(
                "  sdf_stack: z_lo={:.3} z_up={:.3} gap={:.3} ncon={}",
                data.qpos[2],
                data.qpos[9],
                data.qpos[9] - data.qpos[2],
                data.ncon,
            );
        }
    }

    let z_lo = data.qpos[2];
    let z_up = data.qpos[9];
    let gap = z_up - z_lo;

    eprintln!(
        "sdf_stack: z_lo={:.3} z_up={:.3} gap={:.3} ncon={}",
        z_lo, z_up, gap, data.ncon
    );

    // Lower sphere should rest near z=5 (radius on ground)
    assert!(
        (z_lo - 5.0).abs() < 1.0,
        "lower SDF sphere should rest at z≈5, got {z_lo:.3}"
    );
    // Upper sphere should rest near z=15 (on top of lower)
    assert!(
        (z_up - 15.0).abs() < 2.0,
        "upper SDF sphere should rest at z≈15, got {z_up:.3}"
    );
    // Gap should be ~10 (2 radii)
    assert!((gap - 10.0).abs() < 2.0, "gap should be ~10, got {gap:.3}");
}
