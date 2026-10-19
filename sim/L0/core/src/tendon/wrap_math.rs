//! A spatial tendon's wrap over a sphere or a cylinder, as MuJoCo 3.5.0's
//! `mju_wrap` (`engine_util_misc.c:281-417`) and the 2D wraps it calls,
//! operation for operation, so the wrap points and lengths round as
//! MuJoCo's. Pure geometry: no `Model`/`Data`. Used by
//! `tendon/spatial.rs`'s `mj_fwd_tendon_spatial`.

use crate::constraint::impedance::MJ_MINVAL;
use crate::linalg::mju_dot;
use crate::quat::{mul_mat_vec3, normalize3};
use nalgebra::{Matrix3, Vector3};
use std::f64::consts::PI;

/// The geom a tendon wraps over (MuJoCo's `mjWRAP_SPHERE`, `mjWRAP_CYLINDER`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WrapShape {
    /// A sphere about the geom's centre.
    Sphere,
    /// An infinite cylinder along the geom's z axis.
    Cylinder,
}

/// A wrap's two points on the geom, in the world frame, and the length of
/// the path between them over the geom (`mju_wrap`'s `wpnt` and its return
/// value). An inside wrap's two points are one.
#[derive(Clone, Copy, Debug)]
pub struct Wrap {
    /// Where the path meets the geom, then where it leaves it.
    pub points: [Vector3<f64>; 2],
    /// The path's length over the geom: 0 for an inside wrap.
    pub length: f64,
}

/// The wrap of the segment `x0`–`x1` over a geom at `xpos` with rotation
/// `xmat` and `radius`, `side` its sidesite's position (all in the world
/// frame); `None` where the segment does not wrap (MuJoCo `mju_wrap`,
/// `engine_util_misc.c:281-417`). The points go to the geom's frame; a
/// sphere's 2D frame is the plane through its centre and the two points
/// (MuJoCo's plane where they lie on one line through it), a cylinder's its
/// x and y axes. A sidesite inside the geom (`‖s‖ < radius`, 3D) wraps
/// inside ([`wrap_inside`]), any other wraps around ([`wrap_circle`]),
/// steered by the sidesite's direction in that frame scaled to `radius`. A
/// cylinder's points take heights along the path's 2D length and its length
/// the height between them.
// MuJoCo's names: x0, x1, xpos, xmat, p, d, s, sd, res. A cylinder's lengths
// are MuJoCo's square roots of sums of squares, which `hypot` would round
// otherwise.
#[allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::imprecise_flops
)]
pub fn mju_wrap(
    x0: &Vector3<f64>,
    x1: &Vector3<f64>,
    xpos: &Vector3<f64>,
    xmat: &Matrix3<f64>,
    radius: f64,
    shape: WrapShape,
    side: Option<&Vector3<f64>>,
) -> Option<Wrap> {
    // The points in the geom's frame.
    let p = [
        mul_mat_t_vec3(xmat, &sub3(x0, xpos)),
        mul_mat_t_vec3(xmat, &sub3(x1, xpos)),
    ];
    if norm3(&p[0]) < MJ_MINVAL || norm3(&p[1]) < MJ_MINVAL {
        return None;
    }

    let axis = match shape {
        WrapShape::Sphere => sphere_plane(&p[0], &p[1]),
        WrapShape::Cylinder => [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    };
    let d = [
        dot3(&p[0], &axis[0]),
        dot3(&p[0], &axis[1]),
        dot3(&p[1], &axis[0]),
        dot3(&p[1], &axis[1]),
    ];

    let (pnt, mut wlen) = match side {
        Some(side) => {
            let s = mul_mat_t_vec3(xmat, &sub3(side, xpos));
            let mut sd = [dot3(&s, &axis[0]), dot3(&s, &axis[1])];
            normalize(&mut sd);
            let sd = [sd[0] * radius, sd[1] * radius];
            if norm3(&s) < radius {
                let pnt = wrap_inside(&d, radius)?;
                ([pnt[0], pnt[1], pnt[0], pnt[1]], 0.0)
            } else {
                wrap_circle(&d, Some(&sd), radius)?
            }
        }
        None => wrap_circle(&d, None, radius)?,
    };

    // The points back in 3D, in the geom's frame.
    let mut res = [[0.0; 3]; 2];
    for (i, r) in res.iter_mut().enumerate() {
        let a = scl3(&axis[0], pnt[2 * i]);
        let b = scl3(&axis[1], pnt[2 * i + 1]);
        *r = [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
    }

    if shape == WrapShape::Cylinder {
        let l0 = ((p[0][0] - res[0][0]) * (p[0][0] - res[0][0])
            + (p[0][1] - res[0][1]) * (p[0][1] - res[0][1]))
            .sqrt();
        let l1 = ((p[1][0] - res[1][0]) * (p[1][0] - res[1][0])
            + (p[1][1] - res[1][1]) * (p[1][1] - res[1][1]))
            .sqrt();
        res[0][2] = p[0][2] + (p[1][2] - p[0][2]) * l0 / (l0 + wlen + l1);
        res[1][2] = p[0][2] + (p[1][2] - p[0][2]) * (l0 + wlen) / (l0 + wlen + l1);
        let height = (res[1][2] - res[0][2]).abs();
        wlen = (wlen * wlen + height * height).sqrt();
    }

    let world = |r: &[f64; 3]| {
        let v = mul_mat_vec3(xmat, r);
        Vector3::new(v[0] + xpos.x, v[1] + xpos.y, v[2] + xpos.z)
    };
    Some(Wrap {
        points: [world(&res[0]), world(&res[1])],
        length: wlen,
    })
}

/// A sphere wrap's 2D frame (`mju_wrap`, `engine_util_misc.c:302-340`): the
/// first axis is `p0`'s direction, the second the plane's normal `p0 × p1`
/// crossed with it. When `p0`, the centre and `p1` lie on one line
/// (`‖p0 × p1‖ < mjMINVAL`), the normal is the first axis crossed with `a`,
/// `a` being 1 in every component but one, which is 0: y's if the first
/// axis's |y| exceeds both its other components, z's if its |z| does, x's
/// otherwise. Each is normalized as `mju_normalize3` (below mjMINVAL, the x
/// axis).
fn sphere_plane(p0: &[f64; 3], p1: &[f64; 3]) -> [[f64; 3]; 2] {
    let mut axis0 = *p0;
    normalize3(&mut axis0);
    let mut normal = cross(p0, p1);
    if normalize3(&mut normal) < MJ_MINVAL {
        let a = axis0.map(f64::abs);
        let i = if a[1] > a[0] && a[1] > a[2] {
            1
        } else if a[2] > a[0] && a[2] > a[1] {
            2
        } else {
            0
        };
        let mut second = [1.0; 3];
        second[i] = 0.0;
        normal = cross(&axis0, &second);
        normalize3(&mut normal);
    }
    let mut axis1 = cross(&normal, &axis0);
    normalize3(&mut axis1);
    [axis0, axis1]
}

/// The wrap of the 2D segment `end[0..2]`–`end[2..4]` around a circle of
/// `radius` at the origin: its two tangent points and the arc between them,
/// or `None` (MuJoCo `wrap_circle`, `engine_util_misc.c:78-152`). No wrap
/// where an end is inside the circle, the ends are within mjMINVAL (squared)
/// of each other, or the segment's nearest point to the origin is outside
/// the circle (and, with a `side`, on its side). Of the two tangent pairs,
/// the one whose midpoint's direction is nearer `side`'s (without one: the
/// shorter chord) wins, a pair whose lines cross scoring −10000; the winner
/// does not wrap if its lines cross.
// MuJoCo's names: sqlen0, sqlen1, sqrad, sol, good.
#[allow(clippy::similar_names)]
fn wrap_circle(end: &[f64; 4], side: Option<&[f64; 2]>, radius: f64) -> Option<([f64; 4], f64)> {
    let sqlen0 = end[0] * end[0] + end[1] * end[1];
    let sqlen1 = end[2] * end[2] + end[3] * end[3];
    let sqrad = radius * radius;
    if sqlen0 < sqrad || sqlen1 < sqrad || radius < MJ_MINVAL {
        return None;
    }

    let dif = [end[2] - end[0], end[3] - end[1]];
    let dd = dif[0] * dif[0] + dif[1] * dif[1];
    if dd < MJ_MINVAL {
        return None;
    }

    // The segment's nearest point to the origin, a·dif + end[0..2].
    let a = (-(dif[0] * end[0] + dif[1] * end[1]) / dd).clamp(0.0, 1.0);
    let near = [a * dif[0] + end[0], a * dif[1] + end[1]];
    if near[0] * near[0] + near[1] * near[1] > sqrad
        && side.is_none_or(|s| mju_dot(s, &near) >= 0.0)
    {
        return None;
    }

    let sqrt0 = (sqlen0 - sqrad).sqrt();
    let sqrt1 = (sqlen1 - sqrad).sqrt();
    let mut sol = [[0.0; 4]; 2];
    let mut good = [0.0; 2];
    for i in 0..2 {
        let sgn = if i == 0 { 1.0 } else { -1.0 };
        sol[i] = [
            (end[0] * sqrad + sgn * radius * end[1] * sqrt0) / sqlen0,
            (end[1] * sqrad - sgn * radius * end[0] * sqrt0) / sqlen0,
            (end[2] * sqrad - sgn * radius * end[3] * sqrt1) / sqlen1,
            (end[3] * sqrad + sgn * radius * end[2] * sqrt1) / sqlen1,
        ];
        good[i] = if let Some(s) = side {
            let mut mid = [sol[i][0] + sol[i][2], sol[i][1] + sol[i][3]];
            normalize(&mut mid);
            mju_dot(&mid, s)
        } else {
            let chord = [sol[i][0] - sol[i][2], sol[i][1] - sol[i][3]];
            -mju_dot(&chord, &chord)
        };
        if is_intersect(&end[0..2], &sol[i][0..2], &end[2..4], &sol[i][2..4]) {
            good[i] = -10000.0;
        }
    }

    // MuJoCo's `good[0] > good[1] ? 0 : 1`: a NaN takes the second pair.
    #[allow(clippy::bool_to_int_with_if)]
    let i = if good[0] > good[1] { 0 } else { 1 };
    let pnt = sol[i];
    if is_intersect(&end[0..2], &pnt[0..2], &end[2..4], &pnt[2..4]) {
        return None;
    }
    Some((pnt, length_circle(&pnt[0..2], &pnt[2..4], i, radius)))
}

/// The arc of a circle of `radius` from `p0` to `p1`, both on it (MuJoCo
/// `length_circle`, `engine_util_misc.c:55-71`): the angle between their
/// directions, taken the long way round where their cross product's sign
/// and the tangent pair `ind` ask for it. The angle's cosine is clamped to
/// [−1, 1]; MuJoCo's `acos` is not, and gives NaN past it (registry
/// `D-WRAP-ACOS`). Where the cosine is 1 or rounds past it the two
/// directions are one and the cross product's sign is rounding: the arc is
/// 0, not the long way round, where MuJoCo's at exactly 1 can be a full
/// circle (ledger L185). One ulp below 1 ours goes the long way round where
/// MuJoCo's does (`D-WRAP-ACOS`).
fn length_circle(p0: &[f64], p1: &[f64], ind: usize, radius: f64) -> f64 {
    let mut p0n = [p0[0], p0[1]];
    let mut p1n = [p1[0], p1[1]];
    normalize(&mut p0n);
    normalize(&mut p1n);
    let cosine = mju_dot(&p0n, &p1n);
    let mut angle = cosine.clamp(-1.0, 1.0).acos();
    let cross = p0[1] * p1[0] - p0[0] * p1[1];
    if cosine < 1.0 && ((cross > 0.0 && ind == 1) || (cross < 0.0 && ind == 0)) {
        angle = 2.0 * PI - angle;
    }
    radius * angle
}

/// Whether the 2D segments `p1`–`p2` and `p3`–`p4` meet, ends included
/// (MuJoCo `is_intersect`, `engine_util_misc.c:36-51`); parallel ones
/// (|det| < mjMINVAL) do not.
fn is_intersect(p1: &[f64], p2: &[f64], p3: &[f64], p4: &[f64]) -> bool {
    let det = (p4[1] - p3[1]) * (p2[0] - p1[0]) - (p4[0] - p3[0]) * (p2[1] - p1[1]);
    if det.abs() < MJ_MINVAL {
        return false;
    }
    let a = ((p4[0] - p3[0]) * (p1[1] - p3[1]) - (p4[1] - p3[1]) * (p1[0] - p3[0])) / det;
    let b = ((p2[0] - p1[0]) * (p1[1] - p3[1]) - (p2[1] - p1[1]) * (p1[0] - p3[0])) / det;
    (0.0..=1.0).contains(&a) && (0.0..=1.0).contains(&b)
}

/// The one point where the 2D segment `end[0..2]`–`end[2..4]` touches a
/// circle of `radius` at the origin when it wraps inside it, or `None`
/// (MuJoCo `wrap_inside`, `engine_util_misc.c:158-271`). No wrap where an
/// end is on or inside the circle, the circle or an end's distance is below
/// mjMINVAL, the segment crosses the circle, or the ends point opposite ways
/// (cos G < −1 + mjMINVAL). Newton's method solves
/// `asin(A z) + asin(B z) − 2 asin(z) + G = 0` (`A`, `B` the radius over the
/// ends' distances, `G` the angle between them) from z = 1 − 1e-7, at most 20
/// iterations to |f| ≤ 1e-6; the point is the first end's direction
/// rotated by asin(z) − asin(A z) where end0 × end1 > 0, else the second's
/// by asin(z) − asin(B z) (`engine_util_misc.c:257-263`), at `radius`. Where the ends nearly
/// align, the start is past the root, or Newton fails, the point is the ends'
/// midpoint's direction at `radius`.
// MuJoCo's names: A, B, G, z, f, df.
#[allow(clippy::many_single_char_names)]
fn wrap_inside(end: &[f64; 4], radius: f64) -> Option<[f64; 2]> {
    const MAXITER: usize = 20;
    const ZINIT: f64 = 1.0 - 1e-7;
    const TOLERANCE: f64 = 1e-6;

    let len0 = mju_dot(&end[0..2], &end[0..2]).sqrt();
    let len1 = mju_dot(&end[2..4], &end[2..4]).sqrt();
    let dif = [end[2] - end[0], end[3] - end[1]];
    let dd = dif[0] * dif[0] + dif[1] * dif[1];
    if len0 <= radius
        || len1 <= radius
        || radius < MJ_MINVAL
        || len0 < MJ_MINVAL
        || len1 < MJ_MINVAL
    {
        return None;
    }

    if dd > MJ_MINVAL {
        let a = -(dif[0] * end[0] + dif[1] * end[1]) / dd;
        if a > 0.0 && a < 1.0 {
            let near = [end[0] + a * dif[0], end[1] + a * dif[1]];
            if mju_dot(&near, &near).sqrt() <= radius {
                return None;
            }
        }
    }

    // The fallback: the ends' midpoint's direction, at radius.
    let mut mid = [0.5 * (end[0] + end[2]), 0.5 * (end[1] + end[3])];
    normalize(&mut mid);
    let fallback = [mid[0] * radius, mid[1] * radius];

    let a = radius / len0;
    let b = radius / len1;
    let cos_g = (len0 * len0 + len1 * len1 - dd) / (2.0 * len0 * len1);
    if cos_g < -1.0 + MJ_MINVAL {
        return None;
    } else if cos_g > 1.0 - MJ_MINVAL {
        return Some(fallback);
    }
    let g = cos_g.acos();

    let fun = |z: f64| (a * z).asin() + (b * z).asin() - 2.0 * z.asin() + g;
    let mut z = ZINIT;
    let mut f = fun(z);
    if f > 0.0 {
        return Some(fallback);
    }

    let mut iter = 0;
    while iter < MAXITER && f.abs() > TOLERANCE {
        let df = a / max(MJ_MINVAL, (1.0 - z * z * a * a).sqrt())
            + b / max(MJ_MINVAL, (1.0 - z * z * b * b).sqrt())
            - 2.0 / max(MJ_MINVAL, (1.0 - z * z).sqrt());
        if df > -MJ_MINVAL {
            return Some(fallback);
        }
        let z1 = z - f / df;
        if z1 > z {
            return Some(fallback);
        }
        z = z1;
        f = fun(z);
        if f > TOLERANCE {
            return Some(fallback);
        }
        iter += 1;
    }
    if iter >= MAXITER {
        return Some(fallback);
    }

    let (mut vec, ang) = if end[0] * end[3] - end[1] * end[2] > 0.0 {
        ([end[0], end[1]], z.asin() - (a * z).asin())
    } else {
        ([end[2], end[3]], z.asin() - (b * z).asin())
    };
    normalize(&mut vec);
    Some([
        radius * (ang.cos() * vec[0] - ang.sin() * vec[1]),
        radius * (ang.sin() * vec[0] + ang.cos() * vec[1]),
    ])
}

/// `vec1 − vec2` (MuJoCo `mju_sub3`).
fn sub3(vec1: &Vector3<f64>, vec2: &Vector3<f64>) -> [f64; 3] {
    [vec1.x - vec2.x, vec1.y - vec2.y, vec1.z - vec2.z]
}

/// `matᵀ · vec`, each component summed left to right (MuJoCo
/// `mju_mulMatTVec3`, `engine_util_blas.c:171-180`).
fn mul_mat_t_vec3(mat: &Matrix3<f64>, vec: &[f64; 3]) -> [f64; 3] {
    let col = |c: usize| mat[(0, c)] * vec[0] + mat[(1, c)] * vec[1] + mat[(2, c)] * vec[2];
    [col(0), col(1), col(2)]
}

/// `‖vec‖` (MuJoCo `mju_norm3`).
fn norm3(vec: &[f64; 3]) -> f64 {
    (vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]).sqrt()
}

/// `vec1 · vec2` (MuJoCo `mju_dot3`).
fn dot3(vec1: &[f64; 3], vec2: &[f64; 3]) -> f64 {
    vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2]
}

/// `vec · scl` (MuJoCo `mju_scl3`).
fn scl3(vec: &[f64; 3], scl: f64) -> [f64; 3] {
    [vec[0] * scl, vec[1] * scl, vec[2] * scl]
}

/// `a × b` (MuJoCo `mju_cross`, `engine_util_spatial.c:358-367`).
fn cross(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Scales the 2D `vec` to unit length (MuJoCo `mju_normalize`,
/// `engine_util_blas.c:680-694`): below mjMINVAL it becomes (1, 0).
fn normalize(vec: &mut [f64; 2]) {
    let norm = mju_dot(&vec[..], &vec[..]).sqrt();
    if norm < MJ_MINVAL {
        *vec = [1.0, 0.0];
    } else {
        let inv = 1.0 / norm;
        vec[0] *= inv;
        vec[1] *= inv;
    }
}

/// `a` if `a ≥ b`, else `b` (MuJoCo `mju_max`, `engine_util_misc.c:1296-1302`).
fn max(a: f64, b: f64) -> f64 {
    if a >= b { a } else { b }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod wrap_inside_tests {
    use super::*;
    use approx::assert_relative_eq;

    fn norm2(v: [f64; 2]) -> f64 {
        v[0].hypot(v[1])
    }

    /// T5. Newton convergence and tangency verification.
    ///
    /// Reference geometry: end0 = (2, 0), end1 = (1.5, 1.5√3), radius = 1.
    /// |end0| = 2, |end1| = 3, G = π/3 exactly.
    #[test]
    fn t5_newton_convergence_and_tangency() {
        let end = [2.0, 0.0, 1.5, 1.5 * 3.0_f64.sqrt()];
        let radius = 1.0;

        let t = wrap_inside(&end, radius).expect("wrap_inside should wrap this geometry");

        // Tangent point lies on the circle.
        assert_relative_eq!(norm2(t), radius, epsilon = 1e-10);

        // Tangency (stationarity) condition:
        // τ · (û₀ + û₁) = 0, where τ is the circle tangent at t.
        let unit = |v: [f64; 2]| {
            let n = norm2(v);
            [v[0] / n, v[1] / n]
        };
        let tau = unit([-t[1], t[0]]); // tangent = perpendicular to radial
        let u0 = unit([t[0] - end[0], t[1] - end[1]]);
        let u1 = unit([t[0] - end[2], t[1] - end[3]]);
        let stationarity = (tau[0] * (u0[0] + u1[0]) + tau[1] * (u0[1] + u1[1])).abs();
        assert!(
            stationarity < 1e-8,
            "tangency condition violated: |τ·(û₀+û₁)| = {stationarity:.2e}"
        );
    }

    /// T11. Degenerate: segment crosses circle.
    ///
    /// Endpoints on opposite sides of the circle — the straight line
    /// intersects the circle. wrap_inside must not wrap.
    #[test]
    fn t11_segment_crosses_circle() {
        let end = [2.0, 0.0, -2.0, 0.1]; // opposite side, segment crosses circle
        assert!(
            wrap_inside(&end, 1.0).is_none(),
            "expected no wrap when segment crosses circle"
        );
    }

    /// T12. Degenerate: endpoint exactly at circle surface.
    ///
    /// When |endpoint| = radius, the ≤ check rejects it.
    #[test]
    fn t12_endpoint_at_surface() {
        let end = [1.0, 0.0, 2.0, 1.0]; // end0 exactly at radius
        assert!(
            wrap_inside(&end, 1.0).is_none(),
            "expected no wrap when endpoint is at circle surface"
        );
    }

    /// T13. Fallback behavior: near-degenerate geometry.
    ///
    /// Endpoints barely outside the circle (A ≈ 1, B ≈ 1) and close together
    /// on the same angular side so the connecting segment clears the circle.
    /// This triggers the cosG > 1-ε fallback path, the ends' midpoint's
    /// direction on the circle.
    #[test]
    fn t13_fallback_on_circle() {
        let r = 1.0;
        // Both endpoints barely outside, nearly coincident (small angular separation).
        // Segment at x ≈ 1+1e-8 clears the circle. cosG ≈ 1 → fallback.
        let d = r + 1e-8;
        let end = [d, 1e-6, d, -1e-6];

        let t = wrap_inside(&end, r).expect("near-degenerate should fall back, not refuse");

        // Fallback point must lie on the circle surface.
        assert_relative_eq!(norm2(t), r, epsilon = 1e-10);
    }
}
