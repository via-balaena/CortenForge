// common.wgsl — Shared quaternion and matrix utilities.
//
// Quaternion layout: vec4<f32> = (x, y, z, w)
// Matches nalgebra's memory layout. All functions use this convention.
//
// Matrix layout: WGSL mat3x3<f32> is column-major.
// mat[0] = column 0 = (m00, m10, m20)
// mat[1] = column 1 = (m01, m11, m21)
// mat[2] = column 2 = (m02, m12, m22)

// ── Quaternion operations ─────────────────────────────────────────────

/// Hamilton product: a × b. Layout: (x, y, z, w).
fn quat_mul(a: vec4<f32>, b: vec4<f32>) -> vec4<f32> {
    return vec4<f32>(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,  // x
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,  // y
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,  // z
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,  // w
    );
}

/// Conjugate: (-x, -y, -z, w).
fn quat_conjugate(q: vec4<f32>) -> vec4<f32> {
    return vec4<f32>(-q.x, -q.y, -q.z, q.w);
}

/// Normalize quaternion. Returns identity if near-zero.
fn quat_normalize(q: vec4<f32>) -> vec4<f32> {
    let len_sq = dot(q, q);
    if (len_sq < 1e-14) {
        return vec4<f32>(0.0, 0.0, 0.0, 1.0);
    }
    return q * inverseSqrt(len_sq);
}

/// Rotate vector v by unit quaternion q: q * (0,v) * q*.
/// Optimized Rodrigues form — avoids two full quaternion multiplies.
fn quat_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let u = q.xyz;  // vector part
    let s = q.w;    // scalar part
    return 2.0 * dot(u, v) * u
         + (s * s - dot(u, u)) * v
         + 2.0 * s * cross(u, v);
}

/// Quaternion from axis-angle. Axis must be unit length.
fn quat_from_axis_angle(axis: vec3<f32>, angle: f32) -> vec4<f32> {
    let half = angle * 0.5;
    let s = sin(half);
    let c = cos(half);
    return vec4<f32>(axis * s, c);
}

/// Identity quaternion: (0, 0, 0, 1).
fn quat_identity() -> vec4<f32> {
    return vec4<f32>(0.0, 0.0, 0.0, 1.0);
}

// ── Matrix operations ─────────────────────────────────────────────────

/// Convert unit quaternion to 3×3 rotation matrix (column-major).
fn quat_to_mat3(q: vec4<f32>) -> mat3x3<f32> {
    let x = q.x;
    let y = q.y;
    let z = q.z;
    let w = q.w;

    let x2 = x + x;
    let y2 = y + y;
    let z2 = z + z;

    let xx = x * x2;
    let xy = x * y2;
    let xz = x * z2;
    let yy = y * y2;
    let yz = y * z2;
    let zz = z * z2;
    let wx = w * x2;
    let wy = w * y2;
    let wz = w * z2;

    // Column-major: mat[col][row]
    return mat3x3<f32>(
        vec3<f32>(1.0 - (yy + zz), xy + wz, xz - wy),        // column 0
        vec3<f32>(xy - wz, 1.0 - (xx + zz), yz + wx),          // column 1
        vec3<f32>(xz + wy, yz - wx, 1.0 - (xx + yy)),          // column 2
    );
}
