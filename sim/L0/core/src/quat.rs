//! Quaternion arithmetic as MuJoCo 3.5.0's, operation for operation, so a
//! step's rotations round as MuJoCo's do. Quaternions are `[w, x, y, z]`.

use crate::constraint::impedance::MJ_MINVAL;
use nalgebra::DVector;

/// Scales `vec` to unit length and returns its length before (MuJoCo
/// `mju_normalize3`, `engine_util_blas.c:120-135`). Below [`MJ_MINVAL`] it
/// becomes the x axis.
pub fn normalize3(vec: &mut [f64; 3]) -> f64 {
    let norm = (vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]).sqrt();
    if norm < MJ_MINVAL {
        *vec = [1.0, 0.0, 0.0];
    } else {
        let inv = 1.0 / norm;
        vec[0] *= inv;
        vec[1] *= inv;
        vec[2] *= inv;
    }
    norm
}

/// Scales `quat` to unit length and returns its length before (MuJoCo
/// `mju_normalize4`, `engine_util_blas.c:255-272`). Below [`MJ_MINVAL`] it
/// becomes the identity; a length within [`MJ_MINVAL`] of 1 is left as it is.
pub fn normalize4(quat: &mut [f64; 4]) -> f64 {
    let norm =
        (quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]).sqrt();
    if norm < MJ_MINVAL {
        *quat = [1.0, 0.0, 0.0, 0.0];
    } else if (norm - 1.0).abs() > MJ_MINVAL {
        let inv = 1.0 / norm;
        quat[0] *= inv;
        quat[1] *= inv;
        quat[2] *= inv;
        quat[3] *= inv;
    }
    norm
}

/// The quaternion at `qpos[adr..adr + 4]` (a ball joint's, or a free joint's
/// rotation), normalized by [`normalize4`].
pub fn qpos_quat(qpos: &DVector<f64>, adr: usize) -> [f64; 4] {
    let mut quat = [qpos[adr], qpos[adr + 1], qpos[adr + 2], qpos[adr + 3]];
    normalize4(&mut quat);
    quat
}

/// The rotation by `angle` about the unit `axis` (MuJoCo
/// `mji_axisAngle2Quat`, `engine_inline.h:275-292`): the identity at an angle
/// of exactly 0.
pub fn axis_angle_to_quat(axis: &[f64; 3], angle: f64) -> [f64; 4] {
    if angle == 0.0 {
        [1.0, 0.0, 0.0, 0.0]
    } else {
        let s = (angle * 0.5).sin();
        [(angle * 0.5).cos(), axis[0] * s, axis[1] * s, axis[2] * s]
    }
}

/// The product `a * b` (MuJoCo `mju_mulQuat`, `engine_util_spatial.c:66-77`).
pub fn mul_quat(a: &[f64; 4], b: &[f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// Turns `quat` by the angular velocity `vel` over `scale` (MuJoCo
/// `mju_quatIntegrate`, `engine_util_spatial.c:234-243`): about `vel`'s
/// direction (the x axis below [`MJ_MINVAL`]) by `|vel|·scale`, of either
/// sign, after normalizing `quat` as [`normalize4`] does.
pub fn quat_integrate(quat: &mut [f64; 4], vel: [f64; 3], scale: f64) {
    let mut axis = vel;
    let angle = scale * normalize3(&mut axis);
    let rotation = axis_angle_to_quat(&axis, angle);
    normalize4(quat);
    *quat = mul_quat(quat, &rotation);
}

/// The conjugate of `quat` (MuJoCo `mju_negQuat`, `engine_util_spatial.c:57-62`).
pub fn neg_quat(quat: &[f64; 4]) -> [f64; 4] {
    [quat[0], -quat[1], -quat[2], -quat[3]]
}

/// `vec` rotated by the unit `quat` (MuJoCo `mji_rotVecQuat`,
/// `engine_inline.h:222-240`): `vec` itself for the identity.
// MuJoCo tests for the identity exactly, so the comparison is too.
#[allow(clippy::float_cmp)]
pub fn rot_vec_quat(vec: &[f64; 3], quat: &[f64; 4]) -> [f64; 3] {
    if quat[0] == 1.0 && quat[1] == 0.0 && quat[2] == 0.0 && quat[3] == 0.0 {
        return *vec;
    }
    let tmp = [
        quat[0] * vec[0] + quat[2] * vec[2] - quat[3] * vec[1],
        quat[0] * vec[1] + quat[3] * vec[0] - quat[1] * vec[2],
        quat[0] * vec[2] + quat[1] * vec[1] - quat[2] * vec[0],
    ];
    [
        vec[0] + 2.0 * (quat[2] * tmp[2] - quat[3] * tmp[1]),
        vec[1] + 2.0 * (quat[3] * tmp[0] - quat[1] * tmp[2]),
        vec[2] + 2.0 * (quat[1] * tmp[1] - quat[2] * tmp[0]),
    ]
}

/// The angular velocity that turns the identity into the unit `quat` over
/// `dt` (MuJoCo `mji_quat2Vel`, `engine_inline.h:297-309`): the rotation's
/// axis times its angle, the angle taken in (-π, π], divided by `dt`.
pub fn quat_to_vel(quat: &[f64; 4], dt: f64) -> [f64; 3] {
    let mut axis = [quat[1], quat[2], quat[3]];
    let sin_half = normalize3(&mut axis);
    let mut speed = 2.0 * sin_half.atan2(quat[0]);
    if speed > std::f64::consts::PI {
        speed -= 2.0 * std::f64::consts::PI;
    }
    speed /= dt;
    [axis[0] * speed, axis[1] * speed, axis[2] * speed]
}
