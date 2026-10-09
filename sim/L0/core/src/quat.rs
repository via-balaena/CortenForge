//! Quaternion arithmetic as MuJoCo 3.5.0's, operation for operation, so a
//! step's rotations round as MuJoCo's do. Quaternions are `[w, x, y, z]`.

use crate::constraint::impedance::MJ_MINVAL;

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
