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
/// (positive) from `start`. Times outside the samples clamp to the first or
/// last.
// The coordinate is clamped to [0, count − 1] before its floor becomes a
// `u32`, and `count as R` is exact below 2^24 samples at f32.
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

/// The pose a fraction `s` of the way from `a` to `b`: the rotation
/// interpolated linearly along the shorter arc and renormalized, the
/// translation linearly.
///
/// Spherical interpolation would need `acos` and `sin`, which WGSL computes
/// only to its own accuracy. For samples close together, as lowering takes
/// them, the renormalized rotation stays close to the spherical one, and
/// the gap shrinks with the cube of the step
/// (`tests/motion.rs`, `interpolation_stays_close_to_spherical`).
#[must_use]
pub fn pose_interpolate(a: Pose, b: Pose, s: R) -> Pose {
    let dot = a.qw * b.qw + a.qx * b.qx + a.qy * b.qy + a.qz * b.qz;
    // `q` and `−q` are the same rotation; take the one nearer `a`.
    let sign: R = if dot < 0.0 { -1.0 } else { 1.0 };
    let weight_a = 1.0 - s;
    let weight_b = sign * s;
    let qw = weight_a * a.qw + weight_b * b.qw;
    let qx = weight_a * a.qx + weight_b * b.qx;
    let qy = weight_a * a.qy + weight_b * b.qy;
    let qz = weight_a * a.qz + weight_b * b.qz;
    // At least √½ for unit inputs on the shorter arc, so never zero.
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
