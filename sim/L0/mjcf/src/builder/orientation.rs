//! Quaternion and orientation helpers for MJCF element processing.
//!
//! Converts between MJCF's (w, x, y, z) quaternion convention and nalgebra's
//! internal representation, and resolves the five alternative orientation
//! specifications (euler, axisangle, xyaxes, zaxis, quat) into a single
//! `UnitQuaternion`.

use nalgebra::{Matrix3, UnitQuaternion, Vector3, Vector4};

use crate::types::{AngleUnit, MjcfCompiler};

/// Convert an MJCF quaternion `[w, x, y, z]` to a `UnitQuaternion`.
pub fn quat_from_wxyz(q: nalgebra::Vector4<f64>) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q[0], q[1], q[2], q[3]))
}

/// Convert a `UnitQuaternion` back to MJCF `Vector4<f64>` (w, x, y, z order).
pub fn quat_to_wxyz(q: &UnitQuaternion<f64>) -> Vector4<f64> {
    let qi = q.into_inner();
    Vector4::new(qi.w, qi.i, qi.j, qi.k)
}

/// Convert euler angles (radians) to quaternion using a configurable rotation sequence.
///
/// Mirrors MuJoCo's `mju_euler2Quat` exactly:
/// - Each character in `seq` selects the rotation axis (x/y/z).
/// - **Lowercase** = intrinsic (body-fixed): **post-multiply** `q = q * R_i`.
/// - **Uppercase** = extrinsic (space-fixed): **pre-multiply** `q = R_i * q`.
///
/// Examples:
/// - `"xyz"` (default) → intrinsic XYZ: `q = Rx * Ry * Rz`
/// - `"XYZ"` → extrinsic XYZ: `q = Rz * Ry * Rx`
/// - `"XYz"` → extrinsic X, extrinsic Y, intrinsic z: `q = Ry * Rx * Rz`
pub fn euler_seq_to_quat(euler_rad: Vector3<f64>, seq: &str) -> UnitQuaternion<f64> {
    let mut q = UnitQuaternion::identity();
    for (i, ch) in seq.chars().enumerate() {
        let angle = euler_rad[i];
        let axis = match ch.to_ascii_lowercase() {
            'x' => Vector3::x_axis(),
            'y' => Vector3::y_axis(),
            // 'z' or validated-at-parse-time fallback
            _ => Vector3::z_axis(),
        };
        let r = UnitQuaternion::from_axis_angle(&axis, angle);
        if ch.is_ascii_lowercase() {
            // Intrinsic: post-multiply
            q *= r;
        } else {
            // Extrinsic: pre-multiply
            q = r * q;
        }
    }
    q
}

/// Unified orientation resolution.
///
/// Priority when multiple alternatives are present (MuJoCo errors on
/// multiple; we silently use the highest-priority one for robustness):
/// `euler` > `axisangle` > `xyaxes` > `zaxis` > `quat`.
pub fn resolve_orientation(
    quat: Vector4<f64>,
    euler: Option<Vector3<f64>>,
    axisangle: Option<Vector4<f64>>,
    xyaxes: Option<[f64; 6]>,
    zaxis: Option<Vector3<f64>>,
    compiler: &MjcfCompiler,
) -> UnitQuaternion<f64> {
    if let Some(euler) = euler {
        let euler_rad = if compiler.angle == AngleUnit::Degree {
            euler * (std::f64::consts::PI / 180.0)
        } else {
            euler
        };
        euler_seq_to_quat(euler_rad, &compiler.eulerseq)
    } else if let Some(aa) = axisangle {
        let axis = Vector3::new(aa.x, aa.y, aa.z);
        let angle = if compiler.angle == AngleUnit::Degree {
            aa.w * (std::f64::consts::PI / 180.0)
        } else {
            aa.w
        };
        let norm = axis.norm();
        if norm > 1e-10 {
            UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle)
        } else {
            UnitQuaternion::identity()
        }
    } else if let Some(xy) = xyaxes {
        // Gram-Schmidt orthogonalization (matches MuJoCo's ResolveOrientation).
        let mut x = Vector3::new(xy[0], xy[1], xy[2]);
        let x_norm = x.norm();
        if x_norm < 1e-10 {
            return UnitQuaternion::identity();
        }
        x /= x_norm;

        let mut y = Vector3::new(xy[3], xy[4], xy[5]);
        // Orthogonalize y against x
        y -= x * x.dot(&y);
        let y_norm = y.norm();
        if y_norm < 1e-10 {
            return UnitQuaternion::identity();
        }
        y /= y_norm;

        let z = x.cross(&y);
        // z is already unit-length since x, y are orthonormal

        let rot = Matrix3::from_columns(&[x, y, z]);
        let rotation = nalgebra::Rotation3::from_matrix_unchecked(rot);
        UnitQuaternion::from_rotation_matrix(&rotation)
    } else if let Some(zdir) = zaxis {
        // Minimal rotation from (0,0,1) to given direction.
        // Matches MuJoCo's mjuu_z2quat.
        let zn = zdir.norm();
        if zn < 1e-10 {
            return UnitQuaternion::identity();
        }
        let zdir = zdir / zn;

        let default_z = Vector3::z();
        let mut axis = default_z.cross(&zdir);
        let s = axis.norm();

        if s < 1e-10 {
            // Parallel or anti-parallel
            axis = Vector3::x();
        } else {
            axis /= s;
        }

        let angle = s.atan2(zdir.z);
        let half = angle / 2.0;
        let w = half.cos();
        let xyz = axis * half.sin();
        UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, xyz.x, xyz.y, xyz.z))
    } else {
        quat_from_wxyz(quat)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn test_euler_seq_to_quat_intrinsic_xyz() {
        // Default MuJoCo sequence: intrinsic xyz → Rx * Ry * Rz
        let euler = Vector3::new(0.1, 0.2, 0.3);
        let q = euler_seq_to_quat(euler, "xyz");

        let rx = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1);
        let ry = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let rz = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        let expected = rx * ry * rz;

        assert!(
            (q.into_inner() - expected.into_inner()).norm() < 1e-12,
            "intrinsic xyz failed: got {q:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_euler_seq_to_quat_extrinsic_xyz() {
        // Extrinsic XYZ → pre-multiply: Rz * Ry * Rx (= intrinsic zyx)
        let euler = Vector3::new(0.1, 0.2, 0.3);
        let q = euler_seq_to_quat(euler, "XYZ");

        let rx = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1);
        let ry = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let rz = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        let expected = rz * ry * rx;

        assert!(
            (q.into_inner() - expected.into_inner()).norm() < 1e-12,
            "extrinsic XYZ failed: got {q:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_euler_seq_to_quat_mixed_case() {
        // Mixed: XYz → extrinsic X, extrinsic Y, intrinsic z
        // q = identity
        // ch='X' (extrinsic): q = Rx * q = Rx
        // ch='Y' (extrinsic): q = Ry * q = Ry * Rx
        // ch='z' (intrinsic): q = q * Rz = Ry * Rx * Rz
        let euler = Vector3::new(0.1, 0.2, 0.3);
        let q = euler_seq_to_quat(euler, "XYz");

        let rx = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1);
        let ry = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let rz = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        let expected = ry * rx * rz;

        assert!(
            (q.into_inner() - expected.into_inner()).norm() < 1e-12,
            "mixed XYz failed: got {q:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_euler_seq_zyx_matches_different_order() {
        // Intrinsic zyx should match nalgebra's from_euler_angles(roll, pitch, yaw)
        // which is extrinsic XYZ
        let euler = Vector3::new(0.3, 0.2, 0.1); // z, y, x angles
        let q = euler_seq_to_quat(euler, "zyx");

        // Intrinsic zyx: Rz * Ry * Rx
        let rz = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        let ry = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let rx = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1);
        let expected = rz * ry * rx;

        assert!(
            (q.into_inner() - expected.into_inner()).norm() < 1e-12,
            "intrinsic zyx failed"
        );
    }
}
