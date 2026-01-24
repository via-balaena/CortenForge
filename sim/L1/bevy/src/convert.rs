//! Type conversions between sim-core and Bevy types.
//!
//! This module is THE ONLY place that knows both sim-core and Bevy types.
//! When Bevy upgrades change type layouts, only this file needs updating.
//!
//! ## Coordinate System Conversion
//!
//! sim-core uses a **Z-up** coordinate system (common in physics/robotics):
//! - X: forward
//! - Y: left
//! - Z: up
//!
//! Bevy uses a **Y-up** coordinate system (common in graphics):
//! - X: right
//! - Y: up
//! - Z: forward (out of screen)
//!
//! The conversion swaps Y and Z axes: `(x, y, z)_physics -> (x, z, y)_bevy`

#![allow(clippy::cast_possible_truncation)] // f64 -> f32 is intentional for Bevy

use bevy::math::{Quat, Vec3};
use nalgebra::{Point3, UnitQuaternion, Vector3};

/// Rotation that converts from Z-up to Y-up coordinate system.
/// This is a -90 degree rotation around the X-axis.
fn z_up_to_y_up_rotation() -> UnitQuaternion<f64> {
    // Rotate -90 degrees around X to convert Z-up to Y-up
    UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(Vector3::x()),
        -std::f64::consts::FRAC_PI_2,
    )
}

/// Convert a nalgebra Point3 (Z-up) to Bevy Vec3 (Y-up).
///
/// Swaps Y and Z: `(x, y, z) -> (x, z, y)`
#[inline]
#[must_use]
pub fn vec3_from_point(p: &Point3<f64>) -> Vec3 {
    // Z-up to Y-up: swap Y and Z
    Vec3::new(p.x as f32, p.z as f32, p.y as f32)
}

/// Convert a nalgebra Vector3 (Z-up) to Bevy Vec3 (Y-up).
///
/// Swaps Y and Z: `(x, y, z) -> (x, z, y)`
#[inline]
#[must_use]
pub fn vec3_from_vector(v: &Vector3<f64>) -> Vec3 {
    // Z-up to Y-up: swap Y and Z
    Vec3::new(v.x as f32, v.z as f32, v.y as f32)
}

/// Convert a nalgebra `UnitQuaternion` (Z-up) to Bevy `Quat` (Y-up).
///
/// Applies coordinate system rotation to convert from Z-up to Y-up.
#[inline]
#[must_use]
pub fn quat_from_unit_quaternion(q: &UnitQuaternion<f64>) -> Quat {
    // Convert the quaternion to Y-up coordinate system
    let coord_rotation = z_up_to_y_up_rotation();
    let converted = coord_rotation * q * coord_rotation.inverse();
    let q = converted.quaternion();
    Quat::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32)
}

/// Convert a Bevy Vec3 (Y-up) to nalgebra Point3 (Z-up).
///
/// Swaps Y and Z: `(x, y, z) -> (x, z, y)`
#[inline]
#[must_use]
pub fn point_from_vec3(v: Vec3) -> Point3<f64> {
    // Y-up to Z-up: swap Y and Z
    Point3::new(f64::from(v.x), f64::from(v.z), f64::from(v.y))
}

/// Convert a Bevy Vec3 (Y-up) to nalgebra Vector3 (Z-up).
///
/// Swaps Y and Z: `(x, y, z) -> (x, z, y)`
#[inline]
#[must_use]
pub fn vector_from_vec3(v: Vec3) -> Vector3<f64> {
    // Y-up to Z-up: swap Y and Z
    Vector3::new(f64::from(v.x), f64::from(v.z), f64::from(v.y))
}

/// Convert a Bevy `Quat` (Y-up) to nalgebra `UnitQuaternion` (Z-up).
///
/// Applies coordinate system rotation to convert from Y-up to Z-up.
#[inline]
#[must_use]
pub fn unit_quaternion_from_quat(q: Quat) -> UnitQuaternion<f64> {
    let bevy_quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        f64::from(q.w),
        f64::from(q.x),
        f64::from(q.y),
        f64::from(q.z),
    ));
    // Convert from Y-up to Z-up
    let coord_rotation = z_up_to_y_up_rotation();
    coord_rotation.inverse() * bevy_quat * coord_rotation
}

// ============================================================================
// Bulk conversion functions for mesh data
// ============================================================================

/// Convert a slice of nalgebra Point3 (Z-up) to Bevy vertex positions (Y-up).
///
/// This is the canonical way to convert physics mesh vertices to Bevy format.
/// All mesh generation should use this function rather than manual swapping.
#[must_use]
pub fn vertex_positions_from_points(points: &[Point3<f64>]) -> Vec<[f32; 3]> {
    points
        .iter()
        .map(|p| {
            // Z-up to Y-up: swap Y and Z
            [p.x as f32, p.z as f32, p.y as f32]
        })
        .collect()
}

/// Convert box/ellipsoid dimensions from physics (Z-up) to Bevy (Y-up).
///
/// For axis-aligned shapes like boxes and ellipsoids, the dimensions need
/// Y and Z swapped to match the coordinate system change.
///
/// Example: A box with half-extents `(1, 2, 3)` in physics (width=1, depth=2, height=3)
/// becomes `(1, 3, 2)` in Bevy (width=1, height=3, depth=2).
#[inline]
#[must_use]
pub fn dimensions_to_bevy(v: &Vector3<f64>) -> (f32, f32, f32) {
    // Z-up to Y-up: swap Y and Z
    (v.x as f32, v.z as f32, v.y as f32)
}

/// Convert a normal vector from physics (Z-up) to Bevy (Y-up) as an f32 array.
///
/// Used for mesh normals which need the same coordinate swap as positions.
#[inline]
#[must_use]
pub fn normal_to_bevy(n: &Vector3<f64>) -> [f32; 3] {
    // Z-up to Y-up: swap Y and Z
    [n.x as f32, n.z as f32, n.y as f32]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_4;

    #[test]
    fn point_roundtrip() {
        let original = Point3::new(1.0, 2.0, 3.0);
        let bevy = vec3_from_point(&original);
        let back = point_from_vec3(bevy);

        assert!((original.x - back.x).abs() < 1e-6);
        assert!((original.y - back.y).abs() < 1e-6);
        assert!((original.z - back.z).abs() < 1e-6);
    }

    #[test]
    fn quaternion_roundtrip() {
        let original = UnitQuaternion::from_euler_angles(FRAC_PI_4, 0.0, 0.0);
        let bevy = quat_from_unit_quaternion(&original);
        let back = unit_quaternion_from_quat(bevy);

        let diff = original.rotation_to(&back).angle();
        assert!(diff < 1e-5, "Quaternion roundtrip error: {diff}");
    }

    #[test]
    fn vertex_positions_converts_correctly() {
        let points = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];
        let bevy_positions = vertex_positions_from_points(&points);

        // Physics (1, 2, 3) -> Bevy (1, 3, 2)
        let p0 = bevy_positions[0];
        assert!((p0[0] - 1.0).abs() < 1e-6);
        assert!((p0[1] - 3.0).abs() < 1e-6);
        assert!((p0[2] - 2.0).abs() < 1e-6);

        // Physics (4, 5, 6) -> Bevy (4, 6, 5)
        let p1 = bevy_positions[1];
        assert!((p1[0] - 4.0).abs() < 1e-6);
        assert!((p1[1] - 6.0).abs() < 1e-6);
        assert!((p1[2] - 5.0).abs() < 1e-6);
    }

    #[test]
    fn dimensions_converts_correctly() {
        let dims = Vector3::new(1.0, 2.0, 3.0);
        let (x, y, z) = dimensions_to_bevy(&dims);

        // Physics (1, 2, 3) -> Bevy (1, 3, 2)
        assert!((x - 1.0).abs() < 1e-6);
        assert!((y - 3.0).abs() < 1e-6);
        assert!((z - 2.0).abs() < 1e-6);
    }

    #[test]
    fn normal_converts_correctly() {
        let normal = Vector3::new(0.0, 0.0, 1.0); // Z-up normal
        let bevy_normal = normal_to_bevy(&normal);

        // Physics Z-up (0, 0, 1) -> Bevy Y-up (0, 1, 0)
        assert!(bevy_normal[0].abs() < 1e-6);
        assert!((bevy_normal[1] - 1.0).abs() < 1e-6);
        assert!(bevy_normal[2].abs() < 1e-6);
    }

    #[test]
    fn identity_quaternion_stays_identity() {
        let identity = UnitQuaternion::identity();
        let bevy = quat_from_unit_quaternion(&identity);

        // Identity should remain identity (no rotation in either system)
        assert!((bevy.x).abs() < 1e-5);
        assert!((bevy.y).abs() < 1e-5);
        assert!((bevy.z).abs() < 1e-5);
        assert!((bevy.w - 1.0).abs() < 1e-5);
    }

    #[test]
    fn rotation_around_physics_z_becomes_rotation_around_bevy_y() {
        // 90 degrees around Z (physics up) should become 90 degrees around Y (Bevy up)
        let physics_rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::z()),
            std::f64::consts::FRAC_PI_2,
        );
        let bevy_rot = quat_from_unit_quaternion(&physics_rot);

        // Expected: rotation around Y axis by 90 degrees
        let expected = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);

        // Compare quaternions (allowing for sign flip)
        let dot = bevy_rot.dot(expected).abs();
        assert!(
            dot > 0.999,
            "Physics Z rotation should map to Bevy Y rotation: got {bevy_rot:?}, expected {expected:?}"
        );
    }

    #[test]
    fn rotation_around_physics_x_stays_around_x() {
        // Rotation around X should stay around X (X axis is the same in both systems)
        let physics_rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::x()),
            std::f64::consts::FRAC_PI_2,
        );
        let bevy_rot = quat_from_unit_quaternion(&physics_rot);

        // Expected: rotation around X axis by 90 degrees
        let expected = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);

        let dot = bevy_rot.dot(expected).abs();
        assert!(
            dot > 0.999,
            "Physics X rotation should stay as Bevy X rotation: got {bevy_rot:?}, expected {expected:?}"
        );
    }

    #[test]
    fn rotation_180_degrees_roundtrips() {
        // 180 degree rotation is a common edge case
        let original = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::new(1.0, 1.0, 1.0)),
            std::f64::consts::PI,
        );
        let bevy = quat_from_unit_quaternion(&original);
        let back = unit_quaternion_from_quat(bevy);

        let diff = original.rotation_to(&back).angle();
        assert!(diff < 1e-4, "180 degree rotation roundtrip error: {diff}");
    }

    #[test]
    fn axis_vectors_convert_correctly() {
        // Physics +X should stay +X in Bevy
        let x_axis = Vector3::new(1.0, 0.0, 0.0);
        let bevy_x = vec3_from_vector(&x_axis);
        assert!((bevy_x.x - 1.0).abs() < 1e-6);
        assert!(bevy_x.y.abs() < 1e-6);
        assert!(bevy_x.z.abs() < 1e-6);

        // Physics +Y (left) should become Bevy +Z (forward)
        let y_axis = Vector3::new(0.0, 1.0, 0.0);
        let bevy_y = vec3_from_vector(&y_axis);
        assert!(bevy_y.x.abs() < 1e-6);
        assert!(bevy_y.y.abs() < 1e-6);
        assert!((bevy_y.z - 1.0).abs() < 1e-6);

        // Physics +Z (up) should become Bevy +Y (up)
        let z_axis = Vector3::new(0.0, 0.0, 1.0);
        let bevy_z = vec3_from_vector(&z_axis);
        assert!(bevy_z.x.abs() < 1e-6);
        assert!((bevy_z.y - 1.0).abs() < 1e-6);
        assert!(bevy_z.z.abs() < 1e-6);
    }
}
