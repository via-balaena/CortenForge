//! Batched evaluation of field nodes — 4 points in one tree walk.
//!
//! Carries `[f64; 4]` through the expression tree, eliminating 3/4 of the
//! recursive match/dispatch overhead and enabling LLVM auto-vectorization
//! (AVX2 / NEON) of element-wise arithmetic.
//!
//! No explicit SIMD intrinsics — relies entirely on compiler auto-vectorization
//! of `[f64; 4]` element-wise operations.

use nalgebra::{Point3, Vector3};

use crate::field_node::FieldNode;

/// 4-wide batch of f64 values for auto-vectorization.
type F4 = [f64; 4];

/// 4-wide batch of 3D points.
type P4 = [Point3<f64>; 4];

// ── Element-wise arithmetic on F4 ─────────────────────────────────────────

const fn f4_min(a: F4, b: F4) -> F4 {
    [
        a[0].min(b[0]),
        a[1].min(b[1]),
        a[2].min(b[2]),
        a[3].min(b[3]),
    ]
}

const fn f4_max(a: F4, b: F4) -> F4 {
    [
        a[0].max(b[0]),
        a[1].max(b[1]),
        a[2].max(b[2]),
        a[3].max(b[3]),
    ]
}

const fn f4_neg(a: F4) -> F4 {
    [-a[0], -a[1], -a[2], -a[3]]
}

const fn f4_abs(a: F4) -> F4 {
    [a[0].abs(), a[1].abs(), a[2].abs(), a[3].abs()]
}

const fn f4_sub_scalar(a: F4, s: f64) -> F4 {
    [a[0] - s, a[1] - s, a[2] - s, a[3] - s]
}

const fn f4_splat(v: f64) -> F4 {
    [v, v, v, v]
}

// ── Smooth boolean helpers (4-wide) ───────────────────────────────────────

fn f4_smooth_union(a: F4, b: F4, k: f64) -> F4 {
    let mut out = [0.0; 4];
    for i in 0..4 {
        let h = (0.5 + 0.5 * (b[i] - a[i]) / k).clamp(0.0, 1.0);
        let one_minus_h = 1.0 - h;
        let mix = a[i].mul_add(h, b[i] * one_minus_h);
        out[i] = (k * h).mul_add(-one_minus_h, mix);
    }
    out
}

// ── Batch point transform helpers ─────────────────────────────────────────

fn p4_translate(ps: &P4, offset: &Vector3<f64>) -> P4 {
    [
        Point3::new(ps[0].x - offset.x, ps[0].y - offset.y, ps[0].z - offset.z),
        Point3::new(ps[1].x - offset.x, ps[1].y - offset.y, ps[1].z - offset.z),
        Point3::new(ps[2].x - offset.x, ps[2].y - offset.y, ps[2].z - offset.z),
        Point3::new(ps[3].x - offset.x, ps[3].y - offset.y, ps[3].z - offset.z),
    ]
}

impl FieldNode {
    /// Evaluate the scalar field at 4 points simultaneously.
    ///
    /// Walks the expression tree once, carrying 4 values through each operation.
    /// This eliminates 3/4 of the dispatch overhead and enables auto-vectorization.
    ///
    /// Results are identical to calling `evaluate` 4 times (within floating-point
    /// associativity — operations are in the same order).
    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub(crate) fn evaluate_batch(&self, ps: &P4) -> F4 {
        match self {
            // ── Primitives ────────────────────────────────────────────
            Self::Sphere { radius } => {
                let r = *radius;
                [
                    ps[0].coords.norm() - r,
                    ps[1].coords.norm() - r,
                    ps[2].coords.norm() - r,
                    ps[3].coords.norm() - r,
                ]
            }
            Self::Cuboid { half_extents } => {
                let h = half_extents;
                let mut out = [0.0; 4];
                for i in 0..4 {
                    let qx = ps[i].x.abs() - h.x;
                    let qy = ps[i].y.abs() - h.y;
                    let qz = ps[i].z.abs() - h.z;
                    let outside = Vector3::new(qx.max(0.0), qy.max(0.0), qz.max(0.0)).norm();
                    let inside = qx.max(qy.max(qz)).min(0.0);
                    out[i] = outside + inside;
                }
                out
            }
            Self::Cylinder {
                radius,
                half_height,
            } => {
                let r = *radius;
                let hh = *half_height;
                let mut out = [0.0; 4];
                for i in 0..4 {
                    let d_r = ps[i].x.hypot(ps[i].y) - r;
                    let d_a = ps[i].z.abs() - hh;
                    let outside = d_r.max(0.0).hypot(d_a.max(0.0));
                    let inside = d_r.max(d_a).min(0.0);
                    out[i] = outside + inside;
                }
                out
            }
            Self::Capsule {
                radius,
                half_height,
            } => {
                let r = *radius;
                let hh = *half_height;
                let mut out = [0.0; 4];
                for i in 0..4 {
                    let cz = ps[i].z.clamp(-hh, hh);
                    let closest = Point3::new(0.0, 0.0, cz);
                    out[i] = nalgebra::distance(&ps[i], &closest) - r;
                }
                out
            }
            Self::Ellipsoid { radii } => {
                let mut out = [0.0; 4];
                for i in 0..4 {
                    let scaled =
                        Vector3::new(ps[i].x / radii.x, ps[i].y / radii.y, ps[i].z / radii.z);
                    let sn = scaled.norm();
                    if sn < 1e-12 {
                        out[i] = -radii.x.min(radii.y.min(radii.z));
                        continue;
                    }
                    let grad = Vector3::new(
                        ps[i].x / (radii.x * radii.x),
                        ps[i].y / (radii.y * radii.y),
                        ps[i].z / (radii.z * radii.z),
                    );
                    let gn = grad.norm();
                    if gn < 1e-12 {
                        out[i] = sn - 1.0;
                    } else {
                        out[i] = sn * (sn - 1.0) / gn;
                    }
                }
                out
            }
            Self::Torus { major, minor } => {
                let maj = *major;
                let min = *minor;
                let mut out = [0.0; 4];
                for i in 0..4 {
                    let q_xy = ps[i].x.hypot(ps[i].y) - maj;
                    out[i] = q_xy.hypot(ps[i].z) - min;
                }
                out
            }
            Self::Cone { .. } => {
                let mut out = [0.0; 4];
                for i in 0..4 {
                    out[i] = self.evaluate(&ps[i]);
                }
                out
            }
            Self::Plane { normal, offset } => {
                let n = normal;
                let o = *offset;
                [
                    n.dot(&ps[0].coords) - o,
                    n.dot(&ps[1].coords) - o,
                    n.dot(&ps[2].coords) - o,
                    n.dot(&ps[3].coords) - o,
                ]
            }
            Self::Superellipsoid { radii, n1, n2 } => {
                let e2 = 2.0 / n2;
                let e1 = 2.0 / n1;
                let n2_over_n1 = n2 / n1;
                let n1_over_2 = n1 / 2.0;
                let mut out = [0.0; 4];
                for i in 0..4 {
                    let xy =
                        (ps[i].x / radii.x).abs().powf(e2) + (ps[i].y / radii.y).abs().powf(e2);
                    let f =
                        (xy.powf(n2_over_n1) + (ps[i].z / radii.z).abs().powf(e1)).powf(n1_over_2);
                    out[i] = f - 1.0;
                }
                out
            }
            // Complex primitives: fall back to scalar
            Self::LogSpiral { .. }
            | Self::Helix { .. }
            | Self::Pipe { .. }
            | Self::PipeSpline { .. }
            | Self::Loft { .. } => [
                self.evaluate(&ps[0]),
                self.evaluate(&ps[1]),
                self.evaluate(&ps[2]),
                self.evaluate(&ps[3]),
            ],
            Self::Gyroid { scale, thickness } => {
                let s = *scale;
                let th = *thickness;
                let mut out = [0.0; 4];
                for i in 0..4 {
                    let sx = (s * ps[i].x).sin();
                    let sy = (s * ps[i].y).sin();
                    let sz = (s * ps[i].z).sin();
                    let cx = (s * ps[i].x).cos();
                    let cy = (s * ps[i].y).cos();
                    let cz = (s * ps[i].z).cos();
                    let g = sz.mul_add(cx, sx.mul_add(cy, sy * cz));
                    out[i] = g.abs() - th;
                }
                out
            }
            Self::SchwarzP { scale, thickness } => {
                let s = *scale;
                let th = *thickness;
                let mut out = [0.0; 4];
                for i in 0..4 {
                    let g = (s * ps[i].x).cos() + (s * ps[i].y).cos() + (s * ps[i].z).cos();
                    out[i] = g.abs() - th;
                }
                out
            }

            // ── Booleans ──────────────────────────────────────────────
            Self::Union(a, b) => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                f4_min(va, vb)
            }
            Self::Subtract(a, b) => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                f4_max(va, f4_neg(vb))
            }
            Self::Intersect(a, b) => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                f4_max(va, vb)
            }
            Self::SmoothUnion(a, b, k) => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                f4_smooth_union(va, vb, *k)
            }
            Self::SmoothSubtract(a, b, k) => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                f4_neg(f4_smooth_union(f4_neg(va), vb, *k))
            }
            Self::SmoothIntersect(a, b, k) => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                f4_neg(f4_smooth_union(f4_neg(va), f4_neg(vb), *k))
            }
            Self::SmoothUnionAll(children, k) => {
                if children.is_empty() {
                    return f4_splat(f64::INFINITY);
                }
                if children.len() == 1 {
                    return children[0].evaluate_batch(ps);
                }
                let k_val = *k;
                // Evaluate all children batch-wise
                let child_vals: Vec<F4> = children.iter().map(|c| c.evaluate_batch(ps)).collect();
                let mut out = [0.0; 4];
                for lane in 0..4 {
                    let vals: Vec<f64> = child_vals.iter().map(|cv| cv[lane]).collect();
                    let m = vals.iter().copied().fold(f64::INFINITY, f64::min);
                    let sum: f64 = vals.iter().map(|&v| (-(v - m) / k_val).exp()).sum();
                    out[lane] = k_val.mul_add(-sum.ln(), m);
                }
                out
            }
            Self::SmoothUnionVariable {
                a, b, radius_fn, ..
            } => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                let mut out = [0.0; 4];
                for i in 0..4 {
                    let k = (radius_fn.0)(ps[i]).max(1e-15);
                    let h = (0.5 + 0.5 * (vb[i] - va[i]) / k).clamp(0.0, 1.0);
                    let one_minus_h = 1.0 - h;
                    let mix = va[i].mul_add(h, vb[i] * one_minus_h);
                    out[i] = (k * h).mul_add(-one_minus_h, mix);
                }
                out
            }

            // ── Transforms ────────────────────────────────────────────
            Self::Translate(child, offset) => child.evaluate_batch(&p4_translate(ps, offset)),
            Self::Rotate(child, q) => {
                let local = [
                    q.inverse_transform_point(&ps[0]),
                    q.inverse_transform_point(&ps[1]),
                    q.inverse_transform_point(&ps[2]),
                    q.inverse_transform_point(&ps[3]),
                ];
                child.evaluate_batch(&local)
            }
            Self::ScaleUniform(child, s) => {
                let inv_s = 1.0 / *s;
                let local = [
                    Point3::new(ps[0].x * inv_s, ps[0].y * inv_s, ps[0].z * inv_s),
                    Point3::new(ps[1].x * inv_s, ps[1].y * inv_s, ps[1].z * inv_s),
                    Point3::new(ps[2].x * inv_s, ps[2].y * inv_s, ps[2].z * inv_s),
                    Point3::new(ps[3].x * inv_s, ps[3].y * inv_s, ps[3].z * inv_s),
                ];
                let vals = child.evaluate_batch(&local);
                [vals[0] * *s, vals[1] * *s, vals[2] * *s, vals[3] * *s]
            }
            Self::Mirror(child, normal) => {
                let mut local = *ps;
                for i in 0..4 {
                    let d = ps[i].coords.dot(normal).min(0.0);
                    let two_d = 2.0 * d;
                    local[i] = Point3::new(
                        two_d.mul_add(-normal.x, ps[i].x),
                        two_d.mul_add(-normal.y, ps[i].y),
                        two_d.mul_add(-normal.z, ps[i].z),
                    );
                }
                child.evaluate_batch(&local)
            }

            // ── Domain operations ─────────────────────────────────────
            Self::Shell(child, thickness) => {
                let vals = child.evaluate_batch(ps);
                f4_sub_scalar(f4_abs(vals), *thickness)
            }
            Self::Round(child, radius) => {
                let vals = child.evaluate_batch(ps);
                f4_sub_scalar(vals, *radius)
            }
            Self::Offset(child, distance) => {
                let vals = child.evaluate_batch(ps);
                f4_sub_scalar(vals, *distance)
            }
            Self::Elongate(child, half) => {
                let local = [
                    Point3::new(
                        ps[0].x - ps[0].x.clamp(-half.x, half.x),
                        ps[0].y - ps[0].y.clamp(-half.y, half.y),
                        ps[0].z - ps[0].z.clamp(-half.z, half.z),
                    ),
                    Point3::new(
                        ps[1].x - ps[1].x.clamp(-half.x, half.x),
                        ps[1].y - ps[1].y.clamp(-half.y, half.y),
                        ps[1].z - ps[1].z.clamp(-half.z, half.z),
                    ),
                    Point3::new(
                        ps[2].x - ps[2].x.clamp(-half.x, half.x),
                        ps[2].y - ps[2].y.clamp(-half.y, half.y),
                        ps[2].z - ps[2].z.clamp(-half.z, half.z),
                    ),
                    Point3::new(
                        ps[3].x - ps[3].x.clamp(-half.x, half.x),
                        ps[3].y - ps[3].y.clamp(-half.y, half.y),
                        ps[3].z - ps[3].z.clamp(-half.z, half.z),
                    ),
                ];
                child.evaluate_batch(&local)
            }
            Self::Twist(child, rate) => {
                let r = *rate;
                let mut local = [Point3::origin(); 4];
                for i in 0..4 {
                    let angle = r * ps[i].z;
                    let (s, c) = angle.sin_cos();
                    local[i] = Point3::new(
                        c.mul_add(ps[i].x, -(s * ps[i].y)),
                        s.mul_add(ps[i].x, c * ps[i].y),
                        ps[i].z,
                    );
                }
                child.evaluate_batch(&local)
            }
            Self::Bend(child, rate) => {
                let r = *rate;
                let mut local = [Point3::origin(); 4];
                for i in 0..4 {
                    let angle = r * ps[i].z;
                    let (s, c) = angle.sin_cos();
                    local[i] = Point3::new(
                        c.mul_add(ps[i].x, -(s * ps[i].z)),
                        ps[i].y,
                        s.mul_add(ps[i].x, c * ps[i].z),
                    );
                }
                child.evaluate_batch(&local)
            }
            Self::Repeat(child, spacing) => {
                let mut local = [Point3::origin(); 4];
                for i in 0..4 {
                    local[i] = Point3::new(
                        fold_repeat(ps[i].x, spacing.x),
                        fold_repeat(ps[i].y, spacing.y),
                        fold_repeat(ps[i].z, spacing.z),
                    );
                }
                child.evaluate_batch(&local)
            }
            Self::RepeatBounded {
                child,
                spacing,
                count,
            } => {
                let mut local = [Point3::origin(); 4];
                for i in 0..4 {
                    local[i] = Point3::new(
                        fold_repeat_bounded(ps[i].x, spacing.x, count[0]),
                        fold_repeat_bounded(ps[i].y, spacing.y, count[1]),
                        fold_repeat_bounded(ps[i].z, spacing.z, count[2]),
                    );
                }
                child.evaluate_batch(&local)
            }

            // ── User escape hatch ─────────────────────────────────────
            Self::UserFn { eval, .. } => [
                (eval.0)(ps[0]),
                (eval.0)(ps[1]),
                (eval.0)(ps[2]),
                (eval.0)(ps[3]),
            ],
        }
    }

    /// Compute the analytic gradient at 4 points simultaneously.
    ///
    /// Same tree-walk-once strategy as [`evaluate_batch`](Self::evaluate_batch).
    /// For complex variants (`UserFn`, path-based primitives), falls back to 4x
    /// scalar gradient calls.
    #[must_use]
    #[allow(dead_code, clippy::too_many_lines)]
    pub(crate) fn gradient_batch(&self, ps: &P4) -> [Vector3<f64>; 4] {
        match self {
            // ── Primitives ────────────────────────────────────────────
            Self::Sphere { .. } => {
                let mut out = [Vector3::zeros(); 4];
                for i in 0..4 {
                    let n = ps[i].coords.norm();
                    if n > 1e-15 {
                        out[i] = ps[i].coords / n;
                    }
                }
                out
            }
            Self::Cuboid { half_extents } => {
                let h = half_extents;
                let mut out = [Vector3::zeros(); 4];
                for i in 0..4 {
                    out[i] = grad_cuboid_inline(h, &ps[i]);
                }
                out
            }
            Self::Plane { normal, .. } => [*normal, *normal, *normal, *normal],
            // For remaining primitives, fall back to scalar
            Self::Cylinder { .. }
            | Self::Capsule { .. }
            | Self::Ellipsoid { .. }
            | Self::Torus { .. }
            | Self::Cone { .. }
            | Self::Superellipsoid { .. }
            | Self::LogSpiral { .. }
            | Self::Gyroid { .. }
            | Self::SchwarzP { .. }
            | Self::Helix { .. }
            | Self::Pipe { .. }
            | Self::PipeSpline { .. }
            | Self::Loft { .. }
            | Self::UserFn { .. } => [
                self.gradient(&ps[0]),
                self.gradient(&ps[1]),
                self.gradient(&ps[2]),
                self.gradient(&ps[3]),
            ],

            // ── Booleans ──────────────────────────────────────────────
            Self::Union(a, b) => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                let ga = a.gradient_batch(ps);
                let gb = b.gradient_batch(ps);
                let mut out = [Vector3::zeros(); 4];
                for i in 0..4 {
                    out[i] = if va[i] <= vb[i] { ga[i] } else { gb[i] };
                }
                out
            }
            Self::Subtract(a, b) => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                let ga = a.gradient_batch(ps);
                let gb = b.gradient_batch(ps);
                let mut out = [Vector3::zeros(); 4];
                for i in 0..4 {
                    let neg_vb = -vb[i];
                    out[i] = if va[i] >= neg_vb { ga[i] } else { -gb[i] };
                }
                out
            }
            Self::Intersect(a, b) => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                let ga = a.gradient_batch(ps);
                let gb = b.gradient_batch(ps);
                let mut out = [Vector3::zeros(); 4];
                for i in 0..4 {
                    out[i] = if va[i] >= vb[i] { ga[i] } else { gb[i] };
                }
                out
            }
            Self::SmoothUnion(a, b, k) => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                let ga = a.gradient_batch(ps);
                let gb = b.gradient_batch(ps);
                let k_val = *k;
                let mut out = [Vector3::zeros(); 4];
                for i in 0..4 {
                    let h = (0.5 + 0.5 * (vb[i] - va[i]) / k_val).clamp(0.0, 1.0);
                    out[i] = ga[i] * h + gb[i] * (1.0 - h);
                }
                out
            }
            Self::SmoothSubtract(a, b, k) => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                let ga = a.gradient_batch(ps);
                let gb = b.gradient_batch(ps);
                let k_val = *k;
                let mut out = [Vector3::zeros(); 4];
                for i in 0..4 {
                    let h = (0.5 + 0.5 * (vb[i] + va[i]) / k_val).clamp(0.0, 1.0);
                    out[i] = ga[i] * h - gb[i] * (1.0 - h);
                }
                out
            }
            Self::SmoothIntersect(a, b, k) => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                let ga = a.gradient_batch(ps);
                let gb = b.gradient_batch(ps);
                let k_val = *k;
                let mut out = [Vector3::zeros(); 4];
                for i in 0..4 {
                    let h = (0.5 + 0.5 * (va[i] - vb[i]) / k_val).clamp(0.0, 1.0);
                    out[i] = ga[i] * h + gb[i] * (1.0 - h);
                }
                out
            }
            Self::SmoothUnionAll(children, k) => {
                if children.is_empty() {
                    return [Vector3::zeros(); 4];
                }
                if children.len() == 1 {
                    return children[0].gradient_batch(ps);
                }
                let k_val = *k;
                let child_vals: Vec<F4> = children.iter().map(|c| c.evaluate_batch(ps)).collect();
                let child_grads: Vec<[Vector3<f64>; 4]> =
                    children.iter().map(|c| c.gradient_batch(ps)).collect();
                let mut out = [Vector3::zeros(); 4];
                for lane in 0..4 {
                    let vals: Vec<f64> = child_vals.iter().map(|cv| cv[lane]).collect();
                    let m = vals.iter().copied().fold(f64::INFINITY, f64::min);
                    let weights: Vec<f64> =
                        vals.iter().map(|&v| (-(v - m) / k_val).exp()).collect();
                    let sum: f64 = weights.iter().sum();
                    let mut grad = Vector3::zeros();
                    for (j, &w) in weights.iter().enumerate() {
                        grad += child_grads[j][lane] * (w / sum);
                    }
                    out[lane] = grad;
                }
                out
            }
            Self::SmoothUnionVariable {
                a, b, radius_fn, ..
            } => {
                let va = a.evaluate_batch(ps);
                let vb = b.evaluate_batch(ps);
                let ga = a.gradient_batch(ps);
                let gb = b.gradient_batch(ps);
                let mut out = [Vector3::zeros(); 4];
                for i in 0..4 {
                    let k = (radius_fn.0)(ps[i]).max(1e-15);
                    let h = (0.5 + 0.5 * (vb[i] - va[i]) / k).clamp(0.0, 1.0);
                    out[i] = ga[i] * h + gb[i] * (1.0 - h);
                }
                out
            }

            // ── Transforms ────────────────────────────────────────────
            Self::Translate(child, offset) => child.gradient_batch(&p4_translate(ps, offset)),
            Self::Rotate(child, rot) => {
                let local = [
                    rot.inverse_transform_point(&ps[0]),
                    rot.inverse_transform_point(&ps[1]),
                    rot.inverse_transform_point(&ps[2]),
                    rot.inverse_transform_point(&ps[3]),
                ];
                let gc = child.gradient_batch(&local);
                [
                    rot.transform_vector(&gc[0]),
                    rot.transform_vector(&gc[1]),
                    rot.transform_vector(&gc[2]),
                    rot.transform_vector(&gc[3]),
                ]
            }
            Self::ScaleUniform(child, s) => {
                let inv_s = 1.0 / *s;
                let local = [
                    Point3::new(ps[0].x * inv_s, ps[0].y * inv_s, ps[0].z * inv_s),
                    Point3::new(ps[1].x * inv_s, ps[1].y * inv_s, ps[1].z * inv_s),
                    Point3::new(ps[2].x * inv_s, ps[2].y * inv_s, ps[2].z * inv_s),
                    Point3::new(ps[3].x * inv_s, ps[3].y * inv_s, ps[3].z * inv_s),
                ];
                child.gradient_batch(&local)
            }
            Self::Mirror(child, normal) => {
                let mut local = *ps;
                let mut reflected = [false; 4];
                for i in 0..4 {
                    let d = ps[i].coords.dot(normal);
                    if d < 0.0 {
                        reflected[i] = true;
                        let two_d = 2.0 * d;
                        local[i] = Point3::new(
                            two_d.mul_add(-normal.x, ps[i].x),
                            two_d.mul_add(-normal.y, ps[i].y),
                            two_d.mul_add(-normal.z, ps[i].z),
                        );
                    }
                }
                let gc = child.gradient_batch(&local);
                let mut out = gc;
                for i in 0..4 {
                    if reflected[i] {
                        let dot = gc[i].dot(normal);
                        out[i] = gc[i] - *normal * (2.0 * dot);
                    }
                }
                out
            }

            // ── Domain operations ─────────────────────────────────────
            Self::Shell(child, _) => {
                let vals = child.evaluate_batch(ps);
                let grads = child.gradient_batch(ps);
                let mut out = grads;
                for i in 0..4 {
                    if vals[i] < 0.0 {
                        out[i] = -grads[i];
                    }
                }
                out
            }
            Self::Round(child, _) | Self::Offset(child, _) => child.gradient_batch(ps),
            Self::Elongate(child, half) => {
                let mut local = [Point3::origin(); 4];
                for i in 0..4 {
                    local[i] = Point3::new(
                        ps[i].x - ps[i].x.clamp(-half.x, half.x),
                        ps[i].y - ps[i].y.clamp(-half.y, half.y),
                        ps[i].z - ps[i].z.clamp(-half.z, half.z),
                    );
                }
                let gc = child.gradient_batch(&local);
                let mut out = [Vector3::zeros(); 4];
                for i in 0..4 {
                    out[i] = Vector3::new(
                        if ps[i].x.abs() > half.x { gc[i].x } else { 0.0 },
                        if ps[i].y.abs() > half.y { gc[i].y } else { 0.0 },
                        if ps[i].z.abs() > half.z { gc[i].z } else { 0.0 },
                    );
                }
                out
            }
            Self::Twist(child, rate) => {
                let r = *rate;
                let mut local = [Point3::origin(); 4];
                for i in 0..4 {
                    let angle = r * ps[i].z;
                    let (s, c) = angle.sin_cos();
                    local[i] = Point3::new(
                        c.mul_add(ps[i].x, -(s * ps[i].y)),
                        s.mul_add(ps[i].x, c * ps[i].y),
                        ps[i].z,
                    );
                }
                let gc = child.gradient_batch(&local);
                let mut out = [Vector3::zeros(); 4];
                for i in 0..4 {
                    let angle = r * ps[i].z;
                    let (s, c) = angle.sin_cos();
                    let qx = local[i].x;
                    let qy = local[i].y;
                    out[i] = Vector3::new(
                        c.mul_add(gc[i].x, s * gc[i].y),
                        (-s).mul_add(gc[i].x, c * gc[i].y),
                        r.mul_add(qx.mul_add(gc[i].y, -(qy * gc[i].x)), gc[i].z),
                    );
                }
                out
            }
            Self::Bend(child, rate) => {
                let r = *rate;
                let mut local = [Point3::origin(); 4];
                for i in 0..4 {
                    let angle = r * ps[i].z;
                    let (s, c) = angle.sin_cos();
                    local[i] = Point3::new(
                        c.mul_add(ps[i].x, -(s * ps[i].z)),
                        ps[i].y,
                        s.mul_add(ps[i].x, c * ps[i].z),
                    );
                }
                let gc = child.gradient_batch(&local);
                let mut out = [Vector3::zeros(); 4];
                for i in 0..4 {
                    let angle = r * ps[i].z;
                    let (s, c) = angle.sin_cos();
                    let qx = local[i].x;
                    let qz = local[i].z;
                    out[i] = Vector3::new(
                        c.mul_add(gc[i].x, s * gc[i].z),
                        gc[i].y,
                        (-r.mul_add(qz, s)).mul_add(gc[i].x, r.mul_add(qx, c) * gc[i].z),
                    );
                }
                out
            }
            Self::Repeat(child, spacing) => {
                let mut local = [Point3::origin(); 4];
                for i in 0..4 {
                    local[i] = Point3::new(
                        fold_repeat(ps[i].x, spacing.x),
                        fold_repeat(ps[i].y, spacing.y),
                        fold_repeat(ps[i].z, spacing.z),
                    );
                }
                child.gradient_batch(&local)
            }
            Self::RepeatBounded {
                child,
                spacing,
                count,
            } => {
                let mut local = [Point3::origin(); 4];
                for i in 0..4 {
                    local[i] = Point3::new(
                        fold_repeat_bounded(ps[i].x, spacing.x, count[0]),
                        fold_repeat_bounded(ps[i].y, spacing.y, count[1]),
                        fold_repeat_bounded(ps[i].z, spacing.z, count[2]),
                    );
                }
                child.gradient_batch(&local)
            }
        }
    }
}

// ── Inline helpers ────────────────────────────────────────────────────────

fn grad_cuboid_inline(half: &Vector3<f64>, p: &Point3<f64>) -> Vector3<f64> {
    let q = Vector3::new(p.x.abs() - half.x, p.y.abs() - half.y, p.z.abs() - half.z);
    let sx = p.x.signum();
    let sy = p.y.signum();
    let sz = p.z.signum();
    if q.x > 0.0 || q.y > 0.0 || q.z > 0.0 {
        let w = Vector3::new(q.x.max(0.0), q.y.max(0.0), q.z.max(0.0));
        let w_norm = w.norm();
        if w_norm < 1e-15 {
            return Vector3::zeros();
        }
        Vector3::new(
            if q.x > 0.0 { w.x / w_norm * sx } else { 0.0 },
            if q.y > 0.0 { w.y / w_norm * sy } else { 0.0 },
            if q.z > 0.0 { w.z / w_norm * sz } else { 0.0 },
        )
    } else if q.x >= q.y && q.x >= q.z {
        Vector3::new(sx, 0.0, 0.0)
    } else if q.y >= q.x && q.y >= q.z {
        Vector3::new(0.0, sy, 0.0)
    } else {
        Vector3::new(0.0, 0.0, sz)
    }
}

/// Fold coordinate into fundamental domain for infinite repetition.
fn fold_repeat(coord: f64, spacing: f64) -> f64 {
    spacing.mul_add(-(coord / spacing).round(), coord)
}

/// Fold coordinate for bounded repetition.
fn fold_repeat_bounded(coord: f64, spacing: f64, count: u32) -> f64 {
    if count <= 1 {
        return coord;
    }
    let n = f64::from(count);
    let half = (n - 1.0) * spacing * 0.5;
    let id = ((coord + half) / spacing).round().clamp(0.0, n - 1.0);
    coord - id.mul_add(spacing, -half)
}

// ── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    use approx::assert_relative_eq;
    use nalgebra::UnitQuaternion;

    /// Helper: check `evaluate_batch` matches scalar evaluate for 4 test points.
    fn check_eval_batch(node: &FieldNode, label: &str) {
        let test_points: P4 = [
            Point3::new(1.0, 0.3, -0.5),
            Point3::new(-2.1, 1.7, 0.8),
            Point3::new(0.0, 0.0, 3.0),
            Point3::new(0.5, -1.2, -2.0),
        ];

        let batch = node.evaluate_batch(&test_points);
        for (i, p) in test_points.iter().enumerate() {
            let scalar = node.evaluate(p);
            assert_relative_eq!(batch[i], scalar, epsilon = 1e-12, max_relative = 1e-10,);
        }

        // Second set of points (near origin / farther out)
        let test_points2: P4 = [
            Point3::new(0.01, -0.01, 0.01),
            Point3::new(5.0, 5.0, 5.0),
            Point3::new(-3.0, 0.0, 0.0),
            Point3::new(0.0, -4.0, 2.0),
        ];
        let batch2 = node.evaluate_batch(&test_points2);
        for (i, p) in test_points2.iter().enumerate() {
            let scalar = node.evaluate(p);
            assert_relative_eq!(batch2[i], scalar, epsilon = 1e-12, max_relative = 1e-10,);
        }
        let _ = label;
    }

    /// Helper: check `gradient_batch` matches scalar gradient for 4 test points.
    fn check_grad_batch(node: &FieldNode, label: &str) {
        let test_points: P4 = [
            Point3::new(1.0, 0.3, -0.5),
            Point3::new(-2.1, 1.7, 0.8),
            Point3::new(0.0, 0.0, 3.0),
            Point3::new(0.5, -1.2, -2.0),
        ];

        let batch = node.gradient_batch(&test_points);
        for (i, p) in test_points.iter().enumerate() {
            let scalar = node.gradient(p);
            assert_relative_eq!(batch[i].x, scalar.x, epsilon = 1e-12, max_relative = 1e-10,);
            assert_relative_eq!(batch[i].y, scalar.y, epsilon = 1e-12, max_relative = 1e-10,);
            assert_relative_eq!(batch[i].z, scalar.z, epsilon = 1e-12, max_relative = 1e-10,);
        }
        let _ = label;
    }

    // ── Primitives ────────────────────────────────────────────────────

    #[test]
    fn batch_eval_sphere() {
        check_eval_batch(&FieldNode::Sphere { radius: 3.0 }, "sphere");
    }

    #[test]
    fn batch_eval_cuboid() {
        check_eval_batch(
            &FieldNode::Cuboid {
                half_extents: Vector3::new(2.0, 1.5, 3.0),
            },
            "cuboid",
        );
    }

    #[test]
    fn batch_eval_cylinder() {
        check_eval_batch(
            &FieldNode::Cylinder {
                radius: 2.0,
                half_height: 3.0,
            },
            "cylinder",
        );
    }

    #[test]
    fn batch_eval_capsule() {
        check_eval_batch(
            &FieldNode::Capsule {
                radius: 1.5,
                half_height: 2.0,
            },
            "capsule",
        );
    }

    #[test]
    fn batch_eval_ellipsoid() {
        check_eval_batch(
            &FieldNode::Ellipsoid {
                radii: Vector3::new(3.0, 2.0, 1.0),
            },
            "ellipsoid",
        );
    }

    #[test]
    fn batch_eval_torus() {
        check_eval_batch(
            &FieldNode::Torus {
                major: 3.0,
                minor: 1.0,
            },
            "torus",
        );
    }

    #[test]
    fn batch_eval_cone() {
        check_eval_batch(
            &FieldNode::Cone {
                radius: 2.0,
                height: 4.0,
            },
            "cone",
        );
    }

    #[test]
    fn batch_eval_plane() {
        check_eval_batch(
            &FieldNode::Plane {
                normal: Vector3::new(0.0, 0.0, 1.0),
                offset: 1.0,
            },
            "plane",
        );
    }

    #[test]
    fn batch_eval_superellipsoid() {
        check_eval_batch(
            &FieldNode::Superellipsoid {
                radii: Vector3::new(2.0, 1.5, 3.0),
                n1: 1.5,
                n2: 2.5,
            },
            "superellipsoid",
        );
    }

    #[test]
    fn batch_eval_log_spiral() {
        check_eval_batch(
            &FieldNode::LogSpiral {
                a: 0.5,
                b: 0.2,
                thickness: 0.3,
                turns: 2.0,
            },
            "log_spiral",
        );
    }

    #[test]
    fn batch_eval_gyroid() {
        check_eval_batch(
            &FieldNode::Gyroid {
                scale: 1.0,
                thickness: 0.3,
            },
            "gyroid",
        );
    }

    #[test]
    fn batch_eval_schwarz_p() {
        check_eval_batch(
            &FieldNode::SchwarzP {
                scale: 1.0,
                thickness: 0.3,
            },
            "schwarz_p",
        );
    }

    #[test]
    fn batch_eval_helix() {
        check_eval_batch(
            &FieldNode::Helix {
                radius: 2.0,
                pitch: 1.0,
                thickness: 0.3,
                turns: 3.0,
            },
            "helix",
        );
    }

    #[test]
    fn batch_eval_pipe() {
        check_eval_batch(
            &FieldNode::Pipe {
                vertices: vec![
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(3.0, 0.0, 0.0),
                    Point3::new(3.0, 3.0, 0.0),
                ],
                radius: 0.5,
            },
            "pipe",
        );
    }

    #[test]
    fn batch_eval_pipe_spline() {
        check_eval_batch(
            &FieldNode::PipeSpline {
                control_points: vec![
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(2.0, 0.0, 0.0),
                    Point3::new(4.0, 2.0, 0.0),
                    Point3::new(6.0, 2.0, 0.0),
                ],
                radius: 0.5,
            },
            "pipe_spline",
        );
    }

    #[test]
    fn batch_eval_loft() {
        check_eval_batch(
            &FieldNode::Loft {
                stations: vec![[0.0, 1.0], [2.0, 2.0], [4.0, 1.5]],
            },
            "loft",
        );
    }

    // ── Booleans ──────────────────────────────────────────────────────

    #[test]
    fn batch_eval_union() {
        let node = FieldNode::Union(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.5, 1.5, 1.5),
            }),
        );
        check_eval_batch(&node, "union");
    }

    #[test]
    fn batch_eval_subtract() {
        let node = FieldNode::Subtract(
            Box::new(FieldNode::Sphere { radius: 3.0 }),
            Box::new(FieldNode::Cylinder {
                radius: 1.0,
                half_height: 4.0,
            }),
        );
        check_eval_batch(&node, "subtract");
    }

    #[test]
    fn batch_eval_intersect() {
        let node = FieldNode::Intersect(
            Box::new(FieldNode::Sphere { radius: 3.0 }),
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(2.0, 2.0, 2.0),
            }),
        );
        check_eval_batch(&node, "intersect");
    }

    #[test]
    fn batch_eval_smooth_union() {
        let node = FieldNode::SmoothUnion(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Box::new(FieldNode::Translate(
                Box::new(FieldNode::Sphere { radius: 2.0 }),
                Vector3::new(3.0, 0.0, 0.0),
            )),
            0.5,
        );
        check_eval_batch(&node, "smooth_union");
    }

    #[test]
    fn batch_eval_smooth_subtract() {
        let node = FieldNode::SmoothSubtract(
            Box::new(FieldNode::Sphere { radius: 3.0 }),
            Box::new(FieldNode::Sphere { radius: 1.5 }),
            0.3,
        );
        check_eval_batch(&node, "smooth_subtract");
    }

    #[test]
    fn batch_eval_smooth_intersect() {
        let node = FieldNode::SmoothIntersect(
            Box::new(FieldNode::Sphere { radius: 3.0 }),
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(2.0, 2.0, 2.0),
            }),
            0.3,
        );
        check_eval_batch(&node, "smooth_intersect");
    }

    #[test]
    fn batch_eval_smooth_union_all() {
        let node = FieldNode::SmoothUnionAll(
            vec![
                FieldNode::Sphere { radius: 1.5 },
                FieldNode::Translate(
                    Box::new(FieldNode::Sphere { radius: 1.5 }),
                    Vector3::new(2.0, 0.0, 0.0),
                ),
                FieldNode::Translate(
                    Box::new(FieldNode::Sphere { radius: 1.5 }),
                    Vector3::new(0.0, 2.0, 0.0),
                ),
            ],
            0.5,
        );
        check_eval_batch(&node, "smooth_union_all");
    }

    // ── Transforms ────────────────────────────────────────────────────

    #[test]
    fn batch_eval_translate() {
        let node = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(1.0, 2.0, 3.0),
        );
        check_eval_batch(&node, "translate");
    }

    #[test]
    fn batch_eval_rotate() {
        let node = FieldNode::Rotate(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(2.0, 1.0, 3.0),
            }),
            UnitQuaternion::from_euler_angles(0.3, 0.5, 0.7),
        );
        check_eval_batch(&node, "rotate");
    }

    #[test]
    fn batch_eval_scale_uniform() {
        let node = FieldNode::ScaleUniform(Box::new(FieldNode::Sphere { radius: 1.0 }), 2.5);
        check_eval_batch(&node, "scale_uniform");
    }

    #[test]
    fn batch_eval_mirror() {
        let node = FieldNode::Mirror(
            Box::new(FieldNode::Translate(
                Box::new(FieldNode::Sphere { radius: 1.0 }),
                Vector3::new(2.0, 0.0, 0.0),
            )),
            Vector3::new(1.0, 0.0, 0.0),
        );
        check_eval_batch(&node, "mirror");
    }

    // ── Domain operations ─────────────────────────────────────────────

    #[test]
    fn batch_eval_shell() {
        let node = FieldNode::Shell(Box::new(FieldNode::Sphere { radius: 3.0 }), 0.2);
        check_eval_batch(&node, "shell");
    }

    #[test]
    fn batch_eval_round() {
        let node = FieldNode::Round(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(2.0, 2.0, 2.0),
            }),
            0.3,
        );
        check_eval_batch(&node, "round");
    }

    #[test]
    fn batch_eval_offset() {
        let node = FieldNode::Offset(Box::new(FieldNode::Sphere { radius: 2.0 }), 0.5);
        check_eval_batch(&node, "offset");
    }

    #[test]
    fn batch_eval_elongate() {
        let node = FieldNode::Elongate(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
            Vector3::new(2.0, 0.5, 1.0),
        );
        check_eval_batch(&node, "elongate");
    }

    #[test]
    fn batch_eval_twist() {
        let node = FieldNode::Twist(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 1.0, 3.0),
            }),
            0.5,
        );
        check_eval_batch(&node, "twist");
    }

    #[test]
    fn batch_eval_bend() {
        let node = FieldNode::Bend(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 1.0, 3.0),
            }),
            0.3,
        );
        check_eval_batch(&node, "bend");
    }

    #[test]
    fn batch_eval_repeat() {
        let node = FieldNode::Repeat(
            Box::new(FieldNode::Sphere { radius: 0.5 }),
            Vector3::new(2.0, 2.0, 2.0),
        );
        check_eval_batch(&node, "repeat");
    }

    #[test]
    fn batch_eval_repeat_bounded() {
        let node = FieldNode::RepeatBounded {
            child: Box::new(FieldNode::Sphere { radius: 0.5 }),
            spacing: Vector3::new(2.0, 2.0, 2.0),
            count: [3, 3, 3],
        };
        check_eval_batch(&node, "repeat_bounded");
    }

    #[test]
    fn batch_eval_user_fn() {
        let node = FieldNode::UserFn {
            eval: crate::field_node::UserEvalFn(Arc::new(|p: Point3<f64>| p.coords.norm() - 2.0)),
            interval: None,
            bounds: cf_geometry::Aabb::new(
                Point3::new(-3.0, -3.0, -3.0),
                Point3::new(3.0, 3.0, 3.0),
            ),
        };
        check_eval_batch(&node, "user_fn");
    }

    #[test]
    fn batch_eval_smooth_union_variable() {
        let node = FieldNode::SmoothUnionVariable {
            a: Box::new(FieldNode::Sphere { radius: 2.0 }),
            b: Box::new(FieldNode::Translate(
                Box::new(FieldNode::Sphere { radius: 2.0 }),
                Vector3::new(3.0, 0.0, 0.0),
            )),
            radius_fn: crate::field_node::UserEvalFn(Arc::new(|_: Point3<f64>| 0.5)),
            max_k: 0.5,
        };
        check_eval_batch(&node, "smooth_union_variable");
    }

    // ── Gradient batch tests ──────────────────────────────────────────

    #[test]
    fn batch_grad_sphere() {
        check_grad_batch(&FieldNode::Sphere { radius: 3.0 }, "sphere");
    }

    #[test]
    fn batch_grad_cuboid() {
        check_grad_batch(
            &FieldNode::Cuboid {
                half_extents: Vector3::new(2.0, 1.5, 3.0),
            },
            "cuboid",
        );
    }

    #[test]
    fn batch_grad_union() {
        let node = FieldNode::Union(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.5, 1.5, 1.5),
            }),
        );
        check_grad_batch(&node, "union");
    }

    #[test]
    fn batch_grad_smooth_union() {
        let node = FieldNode::SmoothUnion(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Box::new(FieldNode::Translate(
                Box::new(FieldNode::Sphere { radius: 2.0 }),
                Vector3::new(3.0, 0.0, 0.0),
            )),
            0.5,
        );
        check_grad_batch(&node, "smooth_union");
    }

    #[test]
    fn batch_grad_translate() {
        let node = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(1.0, 2.0, 3.0),
        );
        check_grad_batch(&node, "translate");
    }

    #[test]
    fn batch_grad_rotate() {
        let node = FieldNode::Rotate(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(2.0, 1.0, 3.0),
            }),
            UnitQuaternion::from_euler_angles(0.3, 0.5, 0.7),
        );
        check_grad_batch(&node, "rotate");
    }

    #[test]
    fn batch_grad_shell() {
        let node = FieldNode::Shell(Box::new(FieldNode::Sphere { radius: 3.0 }), 0.2);
        check_grad_batch(&node, "shell");
    }

    #[test]
    fn batch_grad_twist() {
        let node = FieldNode::Twist(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 1.0, 3.0),
            }),
            0.5,
        );
        check_grad_batch(&node, "twist");
    }

    #[test]
    fn batch_grad_bend() {
        let node = FieldNode::Bend(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 1.0, 3.0),
            }),
            0.3,
        );
        check_grad_batch(&node, "bend");
    }

    #[test]
    fn batch_grad_mirror() {
        let node = FieldNode::Mirror(
            Box::new(FieldNode::Translate(
                Box::new(FieldNode::Sphere { radius: 1.0 }),
                Vector3::new(2.0, 0.0, 0.0),
            )),
            Vector3::new(1.0, 0.0, 0.0),
        );
        check_grad_batch(&node, "mirror");
    }

    #[test]
    fn batch_grad_subtract() {
        let node = FieldNode::Subtract(
            Box::new(FieldNode::Sphere { radius: 3.0 }),
            Box::new(FieldNode::Cylinder {
                radius: 1.0,
                half_height: 4.0,
            }),
        );
        check_grad_batch(&node, "subtract");
    }

    #[test]
    fn batch_grad_elongate() {
        let node = FieldNode::Elongate(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
            Vector3::new(2.0, 0.5, 1.0),
        );
        check_grad_batch(&node, "elongate");
    }

    #[test]
    fn batch_grad_repeat_bounded() {
        let node = FieldNode::RepeatBounded {
            child: Box::new(FieldNode::Sphere { radius: 0.5 }),
            spacing: Vector3::new(2.0, 2.0, 2.0),
            count: [3, 3, 3],
        };
        check_grad_batch(&node, "repeat_bounded");
    }

    #[test]
    fn batch_grad_user_fn() {
        let node = FieldNode::UserFn {
            eval: crate::field_node::UserEvalFn(Arc::new(|p: Point3<f64>| p.coords.norm() - 2.0)),
            interval: None,
            bounds: cf_geometry::Aabb::new(
                Point3::new(-3.0, -3.0, -3.0),
                Point3::new(3.0, 3.0, 3.0),
            ),
        };
        check_grad_batch(&node, "user_fn");
    }

    // ── Composed tree (deep recursion) ────────────────────────────────

    #[test]
    fn batch_eval_composed_tree() {
        // SmoothUnion(Translate(Sphere), Rotate(Cuboid)) — tests deep recursion
        let node = FieldNode::SmoothUnion(
            Box::new(FieldNode::Translate(
                Box::new(FieldNode::Sphere { radius: 1.5 }),
                Vector3::new(1.0, 0.0, 0.0),
            )),
            Box::new(FieldNode::Rotate(
                Box::new(FieldNode::Cuboid {
                    half_extents: Vector3::new(1.0, 1.0, 2.0),
                }),
                UnitQuaternion::from_euler_angles(0.0, 0.0, 0.7),
            )),
            0.3,
        );
        check_eval_batch(&node, "composed_tree");
        check_grad_batch(&node, "composed_tree");
    }

    #[test]
    fn batch_eval_nested_booleans() {
        // Subtract(Union(Sphere, Cuboid), Cylinder) — multi-level booleans
        let node = FieldNode::Subtract(
            Box::new(FieldNode::Union(
                Box::new(FieldNode::Sphere { radius: 3.0 }),
                Box::new(FieldNode::Cuboid {
                    half_extents: Vector3::new(2.0, 2.0, 2.0),
                }),
            )),
            Box::new(FieldNode::Cylinder {
                radius: 1.0,
                half_height: 5.0,
            }),
        );
        check_eval_batch(&node, "nested_booleans");
        check_grad_batch(&node, "nested_booleans");
    }
}
