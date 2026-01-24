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
}
