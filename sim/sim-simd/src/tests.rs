//! Tests for SIMD operations.

#![allow(clippy::unwrap_used, clippy::float_cmp)]

use approx::assert_relative_eq;
use nalgebra::Vector3;

use crate::*;

// =============================================================================
// Vec3x4 Tests
// =============================================================================

#[test]
fn test_vec3x4_from_vectors() {
    let vectors = [
        Vector3::new(1.0, 2.0, 3.0),
        Vector3::new(4.0, 5.0, 6.0),
        Vector3::new(7.0, 8.0, 9.0),
        Vector3::new(10.0, 11.0, 12.0),
    ];
    let batch = Vec3x4::from_vectors(vectors);

    for i in 0..4 {
        assert_eq!(batch.get(i), vectors[i]);
    }
}

#[test]
fn test_vec3x4_splat() {
    let v = Vector3::new(1.0, 2.0, 3.0);
    let batch = Vec3x4::splat(v);

    for i in 0..4 {
        assert_eq!(batch.get(i), v);
    }
}

#[test]
fn test_vec3x4_dot() {
    let vectors = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
    ];
    let batch = Vec3x4::from_vectors(vectors);
    let direction = Vector3::new(1.0, 2.0, 3.0);

    let dots = batch.dot(&direction);

    assert_eq!(dots[0], 1.0);
    assert_eq!(dots[1], 2.0);
    assert_eq!(dots[2], 3.0);
    assert_eq!(dots[3], 6.0);
}

#[test]
fn test_vec3x4_dot_simd() {
    let vectors = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
    ];
    let batch = Vec3x4::from_vectors(vectors);
    let direction = Vector3::new(1.0, 2.0, 3.0);

    let dots_normal = batch.dot(&direction);
    let dots_simd = batch.dot_simd(&direction);

    for i in 0..4 {
        assert_relative_eq!(dots_normal[i], dots_simd[i], epsilon = 1e-10);
    }
}

#[test]
fn test_vec3x4_norm_squared() {
    let vectors = [
        Vector3::new(3.0, 4.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 2.0, 2.0),
    ];
    let batch = Vec3x4::from_vectors(vectors);

    let norms_sq = batch.norm_squared();

    assert_eq!(norms_sq[0], 25.0); // 9 + 16
    assert_eq!(norms_sq[1], 1.0);
    assert_eq!(norms_sq[2], 0.0);
    assert_eq!(norms_sq[3], 9.0); // 1 + 4 + 4
}

#[test]
fn test_vec3x4_add_sub() {
    let a = Vec3x4::from_vectors([
        Vector3::new(1.0, 2.0, 3.0),
        Vector3::new(4.0, 5.0, 6.0),
        Vector3::new(7.0, 8.0, 9.0),
        Vector3::new(10.0, 11.0, 12.0),
    ]);
    let b = Vec3x4::from_vectors([
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(2.0, 2.0, 2.0),
        Vector3::new(3.0, 3.0, 3.0),
        Vector3::new(4.0, 4.0, 4.0),
    ]);

    let sum = a + b;
    let diff = a - b;

    assert_eq!(sum.get(0), Vector3::new(2.0, 3.0, 4.0));
    assert_eq!(diff.get(0), Vector3::new(0.0, 1.0, 2.0));
}

#[test]
fn test_vec3x4_cross() {
    let vectors = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
    ];
    let batch = Vec3x4::from_vectors(vectors);
    let direction = Vector3::new(0.0, 0.0, 1.0);

    let crosses = batch.cross(&direction);

    // (1,0,0) × (0,0,1) = (0,-1,0)
    assert_eq!(crosses.get(0), Vector3::new(0.0, -1.0, 0.0));
    // (0,1,0) × (0,0,1) = (1,0,0)
    assert_eq!(crosses.get(1), Vector3::new(1.0, 0.0, 0.0));
    // (0,0,1) × (0,0,1) = (0,0,0)
    assert_eq!(crosses.get(2), Vector3::zeros());
}

#[test]
fn test_vec3x4_argmax_dot() {
    let vectors = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(0.5, 0.0, 0.0),
        Vector3::new(1.5, 0.0, 0.0),
    ];
    let batch = Vec3x4::from_vectors(vectors);
    let direction = Vector3::new(1.0, 0.0, 0.0);

    let (idx, val) = batch.argmax_dot(&direction);

    assert_eq!(idx, 1);
    assert_eq!(val, 2.0);
}

#[test]
fn test_vec3x4_normalize() {
    let vectors = [
        Vector3::new(3.0, 4.0, 0.0),
        Vector3::new(0.0, 0.0, 5.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0), // Zero vector
    ];
    let batch = Vec3x4::from_vectors(vectors);

    let normalized = batch.normalize();

    // (3,4,0) normalized = (0.6, 0.8, 0)
    assert_relative_eq!(normalized.get(0).norm(), 1.0, epsilon = 1e-10);
    assert_relative_eq!(normalized.get(1).norm(), 1.0, epsilon = 1e-10);
    assert_relative_eq!(normalized.get(2).norm(), 1.0, epsilon = 1e-10);
    // Zero vector should stay zero (or some safe value)
}

#[test]
fn test_vec3x4_clamp_to_box() {
    let vectors = [
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(0.0, 3.0, 0.0),
        Vector3::new(0.0, 0.0, -2.0),
        Vector3::new(0.5, 0.5, 0.5),
    ];
    let batch = Vec3x4::from_vectors(vectors);
    let half_extents = Vector3::new(1.0, 1.0, 1.0);

    let clamped = batch.clamp_to_box(&half_extents);

    assert_eq!(clamped.get(0), Vector3::new(1.0, 0.0, 0.0));
    assert_eq!(clamped.get(1), Vector3::new(0.0, 1.0, 0.0));
    assert_eq!(clamped.get(2), Vector3::new(0.0, 0.0, -1.0));
    assert_eq!(clamped.get(3), Vector3::new(0.5, 0.5, 0.5));
}

// =============================================================================
// Vec3x8 Tests
// =============================================================================

#[test]
fn test_vec3x8_splat() {
    let v = Vector3::new(1.0, 2.0, 3.0);
    let batch = Vec3x8::splat(v);

    for i in 0..8 {
        assert_eq!(batch.get(i), v);
    }
}

#[test]
fn test_vec3x8_dot() {
    let batch = Vec3x8::splat(Vector3::new(1.0, 2.0, 3.0));
    let direction = Vector3::new(1.0, 1.0, 1.0);

    let dots = batch.dot(&direction);

    for dot in dots {
        assert_eq!(dot, 6.0);
    }
}

// =============================================================================
// Batch Operations Tests
// =============================================================================

#[test]
fn test_batch_dot_product_4() {
    let vectors = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
    ];
    let direction = Vector3::new(2.0, 3.0, 4.0);

    let dots = batch_dot_product_4(&vectors, &direction);

    assert_eq!(dots[0], 2.0);
    assert_eq!(dots[1], 3.0);
    assert_eq!(dots[2], 4.0);
    assert_eq!(dots[3], 9.0);
}

#[test]
fn test_find_max_dot() {
    let vectors = vec![
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
        Vector3::new(0.0, 0.0, 3.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(-1.0, -1.0, -1.0),
        Vector3::new(0.5, 0.5, 0.5),
        Vector3::new(2.0, 2.0, 2.0), // Maximum
        Vector3::new(1.5, 1.5, 1.5),
    ];
    let direction = Vector3::new(1.0, 1.0, 1.0);

    let (idx, val) = find_max_dot(&vectors, &direction);

    assert_eq!(idx, 7);
    assert_eq!(val, 6.0); // 2+2+2
}

#[test]
fn test_find_max_dot_small() {
    let vectors = vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 2.0, 0.0)];
    let direction = Vector3::new(0.0, 1.0, 0.0);

    let (idx, val) = find_max_dot(&vectors, &direction);

    assert_eq!(idx, 1);
    assert_eq!(val, 2.0);
}

#[test]
fn test_batch_aabb_overlap() {
    let a_mins = Vec3x4::from_vectors([
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 10.0, 10.0),
        Vector3::new(0.0, 0.0, 0.0),
    ]);
    let a_maxs = Vec3x4::from_vectors([
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(11.0, 11.0, 11.0),
        Vector3::new(1.0, 1.0, 1.0),
    ]);
    let b_mins = Vec3x4::from_vectors([
        Vector3::new(0.5, 0.5, 0.5), // Overlaps
        Vector3::new(2.0, 0.0, 0.0), // Doesn't overlap
        Vector3::new(0.0, 0.0, 0.0), // Doesn't overlap
        Vector3::new(0.5, 0.5, 2.0), // Doesn't overlap (z)
    ]);
    let b_maxs = Vec3x4::from_vectors([
        Vector3::new(1.5, 1.5, 1.5),
        Vector3::new(3.0, 1.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(1.5, 1.5, 3.0),
    ]);

    let overlaps = batch_aabb_overlap_4(&a_mins, &a_maxs, &b_mins, &b_maxs);

    assert!(overlaps[0]); // Overlaps
    assert!(!overlaps[1]); // Doesn't overlap
    assert!(!overlaps[2]); // Doesn't overlap
    assert!(!overlaps[3]); // Doesn't overlap
}

#[test]
fn test_batch_normal_force() {
    let penetrations = [0.01, 0.02, 0.0, 0.005];
    let approach_velocities = [0.0, 0.1, 0.0, -0.1];
    let stiffness = 100_000.0;
    let stiffness_power = 1.0;
    let damping = 1000.0;

    let forces = batch_normal_force_4(
        &penetrations,
        &approach_velocities,
        stiffness,
        stiffness_power,
        damping,
    );

    // F = k * d + c * v
    assert_relative_eq!(forces[0], 1000.0, epsilon = 1.0); // 100000 * 0.01
    assert_relative_eq!(forces[1], 2100.0, epsilon = 1.0); // 100000 * 0.02 + 1000 * 0.1
    assert_eq!(forces[2], 0.0); // No penetration
    assert_relative_eq!(forces[3], 400.0, epsilon = 1.0); // 100000 * 0.005 - 1000 * 0.1 (clamped)
}

#[test]
fn test_batch_friction_force() {
    let tangent_velocities = Vec3x4::from_vectors([
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),   // Zero velocity
        Vector3::new(0.001, 0.0, 0.0), // Very slow (regularized)
    ]);
    let normal_magnitudes = [100.0, 100.0, 100.0, 100.0];
    let friction_coefficient = 0.5;
    let regularization_velocity = 0.01;

    let friction = batch_friction_force_4(
        &tangent_velocities,
        &normal_magnitudes,
        friction_coefficient,
        regularization_velocity,
    );

    // Friction should oppose motion
    assert!(friction.xs[0] < 0.0);
    assert!(friction.ys[1] < 0.0);
    assert_eq!(friction.get(2), Vector3::zeros()); // Zero velocity = zero friction
    // Regularized friction should be smaller
    assert!(friction.xs[3].abs() < friction.xs[0].abs());
}

#[test]
fn test_batch_integration() {
    let mut positions = Vec3x4::from_vectors([
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    ]);
    let velocities = Vec3x4::splat(Vector3::new(1.0, 2.0, 3.0));
    let dt = 0.1;

    batch_integrate_position_4(&mut positions, &velocities, dt);

    let p0 = positions.get(0);
    let p1 = positions.get(1);
    assert_relative_eq!(p0.x, 0.1, epsilon = 1e-10);
    assert_relative_eq!(p0.y, 0.2, epsilon = 1e-10);
    assert_relative_eq!(p0.z, 0.3, epsilon = 1e-10);
    assert_relative_eq!(p1.x, 1.1, epsilon = 1e-10);
    assert_relative_eq!(p1.y, 0.2, epsilon = 1e-10);
    assert_relative_eq!(p1.z, 0.3, epsilon = 1e-10);
}

// =============================================================================
// Property-Based Tests
// =============================================================================

use proptest::prelude::*;

proptest! {
    #[test]
    fn test_vec3x4_dot_matches_scalar(
        v0 in prop::array::uniform3(-100.0..100.0f64),
        v1 in prop::array::uniform3(-100.0..100.0f64),
        v2 in prop::array::uniform3(-100.0..100.0f64),
        v3 in prop::array::uniform3(-100.0..100.0f64),
        dir in prop::array::uniform3(-100.0..100.0f64),
    ) {
        let vectors = [
            Vector3::new(v0[0], v0[1], v0[2]),
            Vector3::new(v1[0], v1[1], v1[2]),
            Vector3::new(v2[0], v2[1], v2[2]),
            Vector3::new(v3[0], v3[1], v3[2]),
        ];
        let direction = Vector3::new(dir[0], dir[1], dir[2]);

        let batch = Vec3x4::from_vectors(vectors);
        let batch_dots = batch.dot(&direction);

        for i in 0..4 {
            let scalar_dot = vectors[i].dot(&direction);
            prop_assert!((batch_dots[i] - scalar_dot).abs() < 1e-10);
        }
    }

    #[test]
    fn test_vec3x4_norm_squared_non_negative(
        v0 in prop::array::uniform3(-100.0..100.0f64),
        v1 in prop::array::uniform3(-100.0..100.0f64),
        v2 in prop::array::uniform3(-100.0..100.0f64),
        v3 in prop::array::uniform3(-100.0..100.0f64),
    ) {
        let vectors = [
            Vector3::new(v0[0], v0[1], v0[2]),
            Vector3::new(v1[0], v1[1], v1[2]),
            Vector3::new(v2[0], v2[1], v2[2]),
            Vector3::new(v3[0], v3[1], v3[2]),
        ];

        let batch = Vec3x4::from_vectors(vectors);
        let norms_sq = batch.norm_squared();

        for norm_sq in norms_sq {
            prop_assert!(norm_sq >= 0.0);
        }
    }

    #[test]
    fn test_vec3x4_cross_orthogonal(
        v0 in prop::array::uniform3(-100.0..100.0f64),
        dir in prop::array::uniform3(-100.0..100.0f64),
    ) {
        let v = Vector3::new(v0[0], v0[1], v0[2]);
        let d = Vector3::new(dir[0], dir[1], dir[2]);

        if v.norm() > 1e-10 && d.norm() > 1e-10 {
            let batch = Vec3x4::splat(v);
            let crosses = batch.cross(&d);
            let cross = crosses.get(0);

            // Cross product is orthogonal to both inputs
            let dot_v = cross.dot(&v);
            let dot_d = cross.dot(&d);

            prop_assert!(dot_v.abs() < 1e-6);
            prop_assert!(dot_d.abs() < 1e-6);
        }
    }
}
