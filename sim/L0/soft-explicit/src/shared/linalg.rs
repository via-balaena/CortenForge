// Small fixed-size linear algebra: 3-vectors are `[R; 3]`, and 3×3 matrices
// are `[R; 9]` in row-major order, `m[3 * row + column]`. Written out without
// loops (plan §13d rule 1).

/// `a + b`.
#[must_use]
pub const fn vec3_add(a: [R; 3], b: [R; 3]) -> [R; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// `a − b`.
#[must_use]
pub const fn vec3_sub(a: [R; 3], b: [R; 3]) -> [R; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// `s · v`.
#[must_use]
pub const fn vec3_scale(v: [R; 3], s: R) -> [R; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

/// `a · b`.
#[must_use]
pub const fn vec3_dot(a: [R; 3], b: [R; 3]) -> R {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// `a × b`.
#[must_use]
pub const fn vec3_cross(a: [R; 3], b: [R; 3]) -> [R; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// `|v|`.
#[must_use]
pub fn vec3_length(v: [R; 3]) -> R {
    vec3_dot(v, v).sqrt()
}

/// `if condition { a } else { b }`, component by component. WGSL's `select`
/// takes scalars and vectors, not arrays, so an array choice is made per
/// component.
#[must_use]
pub const fn vec3_select(condition: bool, a: [R; 3], b: [R; 3]) -> [R; 3] {
    [
        if condition { a[0] } else { b[0] },
        if condition { a[1] } else { b[1] },
        if condition { a[2] } else { b[2] },
    ]
}

/// `a + b`.
#[must_use]
pub const fn mat3_add(a: [R; 9], b: [R; 9]) -> [R; 9] {
    [
        a[0] + b[0],
        a[1] + b[1],
        a[2] + b[2],
        a[3] + b[3],
        a[4] + b[4],
        a[5] + b[5],
        a[6] + b[6],
        a[7] + b[7],
        a[8] + b[8],
    ]
}

/// `a − b`.
#[must_use]
pub const fn mat3_sub(a: [R; 9], b: [R; 9]) -> [R; 9] {
    [
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2],
        a[3] - b[3],
        a[4] - b[4],
        a[5] - b[5],
        a[6] - b[6],
        a[7] - b[7],
        a[8] - b[8],
    ]
}

/// `s · m`.
#[must_use]
pub const fn mat3_scale(m: [R; 9], s: R) -> [R; 9] {
    [
        m[0] * s,
        m[1] * s,
        m[2] * s,
        m[3] * s,
        m[4] * s,
        m[5] * s,
        m[6] * s,
        m[7] * s,
        m[8] * s,
    ]
}

/// `a · b`.
#[must_use]
pub const fn mat3_mul(a: [R; 9], b: [R; 9]) -> [R; 9] {
    [
        a[0] * b[0] + a[1] * b[3] + a[2] * b[6],
        a[0] * b[1] + a[1] * b[4] + a[2] * b[7],
        a[0] * b[2] + a[1] * b[5] + a[2] * b[8],
        a[3] * b[0] + a[4] * b[3] + a[5] * b[6],
        a[3] * b[1] + a[4] * b[4] + a[5] * b[7],
        a[3] * b[2] + a[4] * b[5] + a[5] * b[8],
        a[6] * b[0] + a[7] * b[3] + a[8] * b[6],
        a[6] * b[1] + a[7] * b[4] + a[8] * b[7],
        a[6] * b[2] + a[7] * b[5] + a[8] * b[8],
    ]
}

/// `mᵀ`.
#[must_use]
pub const fn mat3_transpose(m: [R; 9]) -> [R; 9] {
    [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]]
}

/// `det m`.
#[must_use]
pub const fn mat3_det(m: [R; 9]) -> R {
    m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
        + m[2] * (m[3] * m[7] - m[4] * m[6])
}

/// The cofactor matrix, `cof m = (det m) · m⁻ᵀ`. Polynomial in `m`, so it is
/// finite even where `m` is singular.
#[must_use]
pub const fn mat3_cofactor(m: [R; 9]) -> [R; 9] {
    [
        m[4] * m[8] - m[5] * m[7],
        m[5] * m[6] - m[3] * m[8],
        m[3] * m[7] - m[4] * m[6],
        m[2] * m[7] - m[1] * m[8],
        m[0] * m[8] - m[2] * m[6],
        m[1] * m[6] - m[0] * m[7],
        m[1] * m[5] - m[2] * m[4],
        m[2] * m[3] - m[0] * m[5],
        m[0] * m[4] - m[1] * m[3],
    ]
}

/// `‖m‖²`, the sum of squared entries. For a deformation gradient it is the
/// first invariant `I₁ = tr(FᵀF)`.
#[must_use]
pub const fn mat3_frobenius_squared(m: [R; 9]) -> R {
    m[0] * m[0]
        + m[1] * m[1]
        + m[2] * m[2]
        + m[3] * m[3]
        + m[4] * m[4]
        + m[5] * m[5]
        + m[6] * m[6]
        + m[7] * m[7]
        + m[8] * m[8]
}
