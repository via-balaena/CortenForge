// The obstacle's pose: a rotation (unit quaternion) and a translation, body
// to world, `x_world = q x_body q* + t`. Lowering samples the insertion path
// evenly in time, so a pose at any time is an interpolation between two
// neighbouring samples, found without a search.

/// A rigid pose, body to world. `#[repr(C)]` with seven scalars and no
/// `vec3` (plan §13d rule 3).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    /// Quaternion scalar part.
    pub qw: R,
    /// Quaternion vector part, x.
    pub qx: R,
    /// Quaternion vector part, y.
    pub qy: R,
    /// Quaternion vector part, z.
    pub qz: R,
    /// Translation, x.
    pub tx: R,
    /// Translation, y.
    pub ty: R,
    /// Translation, z.
    pub tz: R,
}

/// Two neighbouring samples and how far between them a time falls.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SampleSpan {
    /// The earlier sample's index.
    pub lower: u32,
    /// The later sample's index (equal to `lower` at the last sample).
    pub upper: u32,
    /// The weight of `upper`, in `[0, 1]`.
    pub fraction: R,
}

/// Below this `sin θ`, interpolation is linear rather than spherical.
///
/// θ is the angle between two quaternions as 4-vectors, half the rotation
/// between them. The threshold guards the division by `sin θ` when the rotations are
/// equal.
pub const SLERP_THRESHOLD: R = 1e-6;

/// Rotate `v` by the pose's rotation (no translation):
/// `v + w t + u × t`, with `u` the vector part and `t = 2 u × v`.
#[must_use]
pub const fn pose_rotate(pose: Pose, v: [R; 3]) -> [R; 3] {
    let u = [pose.qx, pose.qy, pose.qz];
    let t = vec3_scale(vec3_cross(u, v), 2.0);
    vec3_add(vec3_add(v, vec3_scale(t, pose.qw)), vec3_cross(u, t))
}

/// Rotate `v` by the inverse of the pose's rotation.
#[must_use]
pub const fn pose_unrotate(pose: Pose, v: [R; 3]) -> [R; 3] {
    let u = [-pose.qx, -pose.qy, -pose.qz];
    let t = vec3_scale(vec3_cross(u, v), 2.0);
    vec3_add(vec3_add(v, vec3_scale(t, pose.qw)), vec3_cross(u, t))
}

/// A body-frame point in the world frame.
#[must_use]
pub const fn pose_to_world(pose: Pose, point: [R; 3]) -> [R; 3] {
    vec3_add(pose_rotate(pose, point), [pose.tx, pose.ty, pose.tz])
}

/// A world-frame point in the body frame.
#[must_use]
pub const fn pose_to_body(pose: Pose, point: [R; 3]) -> [R; 3] {
    pose_unrotate(pose, vec3_sub(point, [pose.tx, pose.ty, pose.tz]))
}

/// The samples around `time`, for `count` samples taken every `interval`
/// from `start`. Times outside the samples clamp to the first or last.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::cast_lossless
)]
#[must_use]
pub fn pose_sample_span(time: R, start: R, interval: R, count: u32) -> SampleSpan {
    let last = if count > 0 { count - 1 } else { 0 };
    let coordinate = ((time - start) / interval).clamp(0.0, last as R);
    let lower = coordinate.floor() as u32;
    let upper = (lower + 1).min(last);
    SampleSpan {
        lower,
        upper,
        fraction: coordinate - lower as R,
    }
}

/// The pose a fraction `s` of the way from `a` to `b`: spherical linear
/// interpolation of the rotation along the shorter arc, linear
/// interpolation of the translation.
#[must_use]
pub fn pose_interpolate(a: Pose, b: Pose, s: R) -> Pose {
    let dot = a.qw * b.qw + a.qx * b.qx + a.qy * b.qy + a.qz * b.qz;
    let sign = if dot < 0.0 { -1.0 } else { 1.0 };
    let cos_theta = (dot * sign).min(1.0);
    let theta = cos_theta.acos();
    let sin_theta = theta.sin();
    let spherical = sin_theta > SLERP_THRESHOLD;
    let guarded_sin = if spherical { sin_theta } else { 1.0 };
    let weight_a = if spherical {
        ((1.0 - s) * theta).sin() / guarded_sin
    } else {
        1.0 - s
    };
    let weight_b = sign
        * if spherical {
            (s * theta).sin() / guarded_sin
        } else {
            s
        };
    let qw = weight_a * a.qw + weight_b * b.qw;
    let qx = weight_a * a.qx + weight_b * b.qx;
    let qy = weight_a * a.qy + weight_b * b.qy;
    let qz = weight_a * a.qz + weight_b * b.qz;
    let norm = (qw * qw + qx * qx + qy * qy + qz * qz).sqrt();
    Pose {
        qw: qw / norm,
        qx: qx / norm,
        qy: qy / norm,
        qz: qz / norm,
        tx: a.tx + s * (b.tx - a.tx),
        ty: a.ty + s * (b.ty - a.ty),
        tz: a.tz + s * (b.tz - a.tz),
    }
}
