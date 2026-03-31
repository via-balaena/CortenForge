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
use bevy::transform::components::Transform;
use nalgebra::{Point3, UnitQuaternion, Vector3};

/// Convert a rotation matrix from Z-up to Y-up via axis swap.
///
/// The Y/Z swap `(x,y,z) → (x,z,y)` is a *reflection* (det = -1), not a
/// rotation. A rotation conjugation `C·R·C⁻¹` approximates this but
/// introduces a sign error on one axis: it maps directions as
/// `(x,y,z) → (x,z,-y)` instead of `(x,y,z) → (x,z,y)`.
///
/// The correct transformation is `R_bevy = S·R_physics·S` where S swaps
/// rows 1↔2 and columns 1↔2. This preserves `det(R) = +1` (it's still a
/// proper rotation) and is exactly consistent with the position swap
/// `vec3_from_vector`.
fn rotation_z_up_to_y_up(m: &nalgebra::Matrix3<f64>) -> nalgebra::Matrix3<f64> {
    // S·R·S where S swaps indices 1 and 2:
    //   new[i][j] = old[σ(i)][σ(j)]  where σ = {0→0, 1→2, 2→1}
    nalgebra::Matrix3::new(
        m[(0, 0)],
        m[(0, 2)],
        m[(0, 1)],
        m[(2, 0)],
        m[(2, 2)],
        m[(2, 1)],
        m[(1, 0)],
        m[(1, 2)],
        m[(1, 1)],
    )
}

/// Convert a rotation matrix from Y-up to Z-up via axis swap.
///
/// Inverse of [`rotation_z_up_to_y_up`]. Since the swap matrix S is its
/// own inverse (S² = I), this is the same operation: `S·R·S`.
fn rotation_y_up_to_z_up(m: &nalgebra::Matrix3<f64>) -> nalgebra::Matrix3<f64> {
    rotation_z_up_to_y_up(m) // S is self-inverse
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
/// Uses the matrix swap `R_bevy = S·R·S` which is exactly consistent with
/// the position swap `(x,y,z) → (x,z,y)` used by [`vec3_from_vector`].
#[inline]
#[must_use]
pub fn quat_from_unit_quaternion(q: &UnitQuaternion<f64>) -> Quat {
    let r = q.to_rotation_matrix();
    quat_from_physics_matrix(r.matrix())
}

/// Convert a physics rotation matrix (Z-up, `Matrix3<f64>`) directly to
/// Bevy `Quat` (Y-up).
///
/// This is the preferred entry point when you already have a rotation matrix
/// (e.g. `data.geom_xmat`). It applies the Y/Z swap and extracts a single
/// quaternion — avoiding the double-extraction instability that occurs if
/// you first convert the matrix to a quaternion and then call
/// [`quat_from_unit_quaternion`].
#[inline]
#[must_use]
pub fn quat_from_physics_matrix(m: &nalgebra::Matrix3<f64>) -> Quat {
    let bevy_mat = rotation_z_up_to_y_up(m);
    let bevy_rot = nalgebra::Rotation3::from_matrix_unchecked(bevy_mat);
    let bevy_q = UnitQuaternion::from_rotation_matrix(&bevy_rot);
    Quat::from_xyzw(
        bevy_q.i as f32,
        bevy_q.j as f32,
        bevy_q.k as f32,
        bevy_q.w as f32,
    )
}

/// Transform a body-local offset to Bevy world-space (Y-up).
///
/// Computes `world = body_pos + body_rot * local_offset` in physics space,
/// then applies the Z-up → Y-up swap. Uses the body's world position and
/// rotation matrix from `Data::xpos` / `Data::xmat`.
#[inline]
#[must_use]
pub fn body_local_to_bevy(
    body_pos: &Vector3<f64>,
    body_mat: &nalgebra::Matrix3<f64>,
    local: &[f64; 3],
) -> Vec3 {
    let wx = body_pos.x
        + body_mat[(0, 0)] * local[0]
        + body_mat[(0, 1)] * local[1]
        + body_mat[(0, 2)] * local[2];
    let wy = body_pos.y
        + body_mat[(1, 0)] * local[0]
        + body_mat[(1, 1)] * local[1]
        + body_mat[(1, 2)] * local[2];
    let wz = body_pos.z
        + body_mat[(2, 0)] * local[0]
        + body_mat[(2, 1)] * local[1]
        + body_mat[(2, 2)] * local[2];
    // Z-up → Y-up swap
    Vec3::new(wx as f32, wz as f32, wy as f32)
}

/// Create a Bevy [`Transform`] from a physics-space position (`Point3<f64>`, Z-up).
///
/// This is the recommended way to position any entity using physics coordinates.
/// Handles the Z-up → Y-up conversion automatically.
#[inline]
#[must_use]
pub fn transform_from_physics(position: &Point3<f64>) -> Transform {
    Transform::from_translation(vec3_from_point(position))
}

/// Create a Bevy [`Transform`] from a physics-space position and orientation.
///
/// Converts both translation (`Vector3<f64>`) and rotation (`UnitQuaternion<f64>`)
/// from physics Z-up to Bevy Y-up. Use this when positioning entities that have
/// both position and orientation from simulation data (e.g., geom transforms).
#[inline]
#[must_use]
pub fn transform_from_physics_pose(
    position: &Vector3<f64>,
    orientation: &UnitQuaternion<f64>,
) -> Transform {
    Transform {
        translation: vec3_from_vector(position),
        rotation: quat_from_unit_quaternion(orientation),
        scale: Vec3::ONE,
    }
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
/// Inverse of [`quat_from_unit_quaternion`]. Uses the same matrix swap
/// (S is its own inverse).
#[inline]
#[must_use]
pub fn unit_quaternion_from_quat(q: Quat) -> UnitQuaternion<f64> {
    let bevy_q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        f64::from(q.w),
        f64::from(q.x),
        f64::from(q.y),
        f64::from(q.z),
    ));
    let bevy_rot = bevy_q.to_rotation_matrix();
    let physics_mat = rotation_y_up_to_z_up(bevy_rot.matrix());
    let physics_rot = nalgebra::Rotation3::from_matrix_unchecked(physics_mat);
    UnitQuaternion::from_rotation_matrix(&physics_rot)
}

// ============================================================================
// Lightweight f32 helpers for example / visual code
// ============================================================================

/// Convert raw physics coordinates (Z-up) to a Bevy `Vec3` (Y-up).
///
/// This is the **recommended entry point for example code** that works with
/// hardcoded positions from MJCF models. It performs the same Y↔Z swap as
/// [`vec3_from_point`] but accepts raw `f32` values instead of requiring
/// nalgebra types.
///
/// # Example
///
/// ```ignore
/// // MJCF: pos="0 0 2.0" (Z-up) → Bevy Vec3(0, 2, 0) (Y-up)
/// let mount = physics_pos(0.0, 0.0, 2.0);
/// assert_eq!(mount, Vec3::new(0.0, 2.0, 0.0));
/// ```
#[inline]
#[must_use]
pub fn physics_pos(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3::new(x, z, y)
}

/// Convert a Bevy `Vec3` (Y-up) back to raw physics coordinates (Z-up).
///
/// Inverse of [`physics_pos`]. Returns `(x, y, z)` in physics (Z-up) order.
#[inline]
#[must_use]
pub fn bevy_to_physics(v: Vec3) -> (f32, f32, f32) {
    (v.x, v.z, v.y)
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
    fn rotation_around_physics_z_becomes_negative_rotation_around_bevy_y() {
        // +90° around Z (physics up) → -90° around Y (Bevy up).
        // The sign flips because the Y/Z swap is a reflection (det=-1),
        // which reverses the handedness of the swapped plane.
        let physics_rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::z()),
            std::f64::consts::FRAC_PI_2,
        );
        let bevy_rot = quat_from_unit_quaternion(&physics_rot);

        let expected = Quat::from_rotation_y(-std::f32::consts::FRAC_PI_2);

        let dot = bevy_rot.dot(expected).abs();
        assert!(
            dot > 0.999,
            "Physics +90° Z → Bevy -90° Y: got {bevy_rot:?}, expected {expected:?}"
        );
    }

    #[test]
    fn rotation_around_physics_x_reverses_sense() {
        // +90° around X in physics → -90° around X in Bevy.
        // X axis is the same in both systems, but the rotation sense
        // reverses because the Y/Z swap flips the handedness of the YZ plane.
        let physics_rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::x()),
            std::f64::consts::FRAC_PI_2,
        );
        let bevy_rot = quat_from_unit_quaternion(&physics_rot);

        let expected = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);

        let dot = bevy_rot.dot(expected).abs();
        assert!(
            dot > 0.999,
            "Physics +90° X → Bevy -90° X: got {bevy_rot:?}, expected {expected:?}"
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

    #[test]
    fn rotation_and_position_are_consistent() {
        // THE critical property: if a physics rotation maps direction d
        // to some world direction, and a position p is at that direction,
        // then the Bevy rotation applied to the Bevy mesh axis must give
        // the same Bevy direction as the Bevy-converted position.
        //
        // This is the bug that was caught by the ball-joint example:
        // the old conjugation approach broke this for diagonal rotations.

        // Physics: capsule along +Z, rotated 30° about (1, 1, 0)
        let axis = nalgebra::Unit::new_normalize(Vector3::new(1.0, 1.0, 0.0));
        let physics_rot = UnitQuaternion::from_axis_angle(&axis, std::f64::consts::FRAC_PI_6);

        // The capsule default axis in physics is +Z
        let capsule_tip_physics = physics_rot * Vector3::new(0.0, 0.0, 1.0);

        // Convert position to Bevy
        let tip_bevy = vec3_from_vector(&capsule_tip_physics);

        // Convert rotation to Bevy, apply to Bevy mesh axis (+Y)
        let bevy_quat = quat_from_unit_quaternion(&physics_rot);
        let mesh_axis = Vec3::new(0.0, 1.0, 0.0); // Bevy capsule mesh axis
        let rotated_axis = bevy_quat * mesh_axis;

        // These must match: position-converted direction == rotation-converted direction
        let err = (rotated_axis - tip_bevy.normalize()).length();
        assert!(
            err < 1e-4,
            "Position/rotation inconsistency: position says {tip_bevy:?}, rotation says {rotated_axis:?}, error={err}"
        );
    }
}
