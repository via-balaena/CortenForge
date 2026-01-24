//! Type conversions between sim-core and Bevy types.
//!
//! This module is THE ONLY place that knows both sim-core and Bevy types.
//! When Bevy upgrades change type layouts, only this file needs updating.

#![allow(clippy::cast_possible_truncation)] // f64 -> f32 is intentional for Bevy

use bevy::math::{Quat, Vec3};
use nalgebra::{Point3, UnitQuaternion, Vector3};

/// Convert a nalgebra Point3 to Bevy Vec3.
#[inline]
#[must_use]
pub fn vec3_from_point(p: &Point3<f64>) -> Vec3 {
    Vec3::new(p.x as f32, p.y as f32, p.z as f32)
}

/// Convert a nalgebra Vector3 to Bevy Vec3.
#[inline]
#[must_use]
pub fn vec3_from_vector(v: &Vector3<f64>) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

/// Convert a nalgebra `UnitQuaternion` to Bevy `Quat`.
#[inline]
#[must_use]
pub fn quat_from_unit_quaternion(q: &UnitQuaternion<f64>) -> Quat {
    let q = q.quaternion();
    Quat::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32)
}

/// Convert a Bevy Vec3 to nalgebra Point3.
#[inline]
#[must_use]
pub fn point_from_vec3(v: Vec3) -> Point3<f64> {
    Point3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z))
}

/// Convert a Bevy Vec3 to nalgebra Vector3.
#[inline]
#[must_use]
pub fn vector_from_vec3(v: Vec3) -> Vector3<f64> {
    Vector3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z))
}

/// Convert a Bevy `Quat` to nalgebra `UnitQuaternion`.
#[inline]
#[must_use]
pub fn unit_quaternion_from_quat(q: Quat) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        f64::from(q.w),
        f64::from(q.x),
        f64::from(q.y),
        f64::from(q.z),
    ))
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
