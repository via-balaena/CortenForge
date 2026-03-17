//! Interval evaluation of field nodes.
//!
//! Given an axis-aligned bounding box, compute conservative (min, max) bounds
//! on the field value over the entire box. This enables octree pruning during
//! meshing: if the interval is entirely positive, the box is fully outside; if
//! entirely negative, fully inside.
//!
//! **Correctness invariant**: for any point `p` inside the `Aabb`,
//! `interval.0 <= evaluate(p) <= interval.1`. The bounds may be loose but
//! must never be violated.

use cf_geometry::Aabb;
use nalgebra::{Point3, Vector3};

use crate::field_node::FieldNode;

impl FieldNode {
    /// Compute conservative (min, max) bounds of the field over an `Aabb`.
    ///
    /// The returned interval `(lo, hi)` satisfies:
    /// `lo <= self.evaluate(p) <= hi` for all `p` in the box.
    pub(crate) fn evaluate_interval(&self, aabb: &Aabb) -> (f64, f64) {
        match self {
            // Primitives
            Self::Sphere { radius } => interval_sphere(*radius, aabb),
            Self::Cuboid { half_extents } => interval_cuboid(half_extents, aabb),
            Self::Cylinder {
                radius,
                half_height,
            } => interval_cylinder(*radius, *half_height, aabb),
            Self::Capsule {
                radius,
                half_height,
            } => interval_capsule(*radius, *half_height, aabb),
            Self::Ellipsoid { radii } => interval_ellipsoid(radii, aabb),
            Self::Torus { major, minor } => interval_torus(*major, *minor, aabb),
            Self::Cone { radius, height } => interval_cone(*radius, *height, aabb),
            Self::Plane { normal, offset } => interval_plane(normal, *offset, aabb),
            Self::Superellipsoid { radii, n1, n2 } => {
                // For the raw implicit function f-1, the gradient magnitude is
                // bounded by ≈ sqrt(3) * max(1/rx, 1/ry, 1/rz) for n1,n2 ≥ 1.
                // For n < 1 (star shapes), gradients can be steeper, so we scale
                // by max(n1,n2)/min(n1,n2) as a safety factor.
                let min_r = radii.x.min(radii.y).min(radii.z);
                let exponent_factor = (n1.max(*n2) / n1.min(*n2)).max(1.0);
                let lipschitz = 2.0 * 3.0_f64.sqrt() / min_r * exponent_factor;
                lipschitz_bound(self, aabb, lipschitz)
            }
            Self::Gyroid { scale, .. } => {
                // |∇g| ≤ 2·scale·√3 for the raw trig field; shell (abs) preserves this.
                let lipschitz = 2.0 * scale * 3.0_f64.sqrt();
                lipschitz_bound(self, aabb, lipschitz)
            }
            Self::SchwarzP { scale, .. } => {
                // |∇g| ≤ scale·√3 for cos(sx)+cos(sy)+cos(sz); shell preserves this.
                let lipschitz = scale * 3.0_f64.sqrt();
                lipschitz_bound(self, aabb, lipschitz)
            }
            Self::LogSpiral { .. }
            | Self::Helix { .. }
            | Self::Pipe { .. }
            | Self::PipeSpline { .. } => lipschitz_bound(self, aabb, 1.0),
            Self::Loft { stations } => {
                // Lipschitz ≈ sqrt(1 + max_taper_slope²). For gentle tapers, ≈ 1.
                // Conservative estimate: piecewise station differences × 2 safety
                // factor for cubic interpolation overshoot.
                let max_slope = stations
                    .windows(2)
                    .map(|w| {
                        let dz = w[1][0] - w[0][0];
                        if dz.abs() < 1e-15 {
                            0.0
                        } else {
                            ((w[1][1] - w[0][1]) / dz).abs()
                        }
                    })
                    .fold(0.0_f64, f64::max)
                    * 2.0;
                let lipschitz = (1.0 + max_slope * max_slope).sqrt();
                lipschitz_bound(self, aabb, lipschitz)
            }

            // Booleans
            Self::Union(a, b) => interval_union(a, b, aabb),
            Self::Subtract(a, b) => interval_subtract(a, b, aabb),
            Self::Intersect(a, b) => interval_intersect(a, b, aabb),
            Self::SmoothUnion(a, b, k) => interval_smooth_union(a, b, *k, aabb),
            Self::SmoothSubtract(a, b, k) => interval_smooth_subtract(a, b, *k, aabb),
            Self::SmoothIntersect(a, b, k) => interval_smooth_intersect(a, b, *k, aabb),
            Self::SmoothUnionAll(children, k) => interval_smooth_union_all(children, *k, aabb),

            // Transforms
            Self::Translate(child, offset) => interval_translate(child, offset, aabb),
            Self::Rotate(child, q) => interval_rotate(child, q, aabb),
            Self::ScaleUniform(child, s) => interval_scale_uniform(child, *s, aabb),
            Self::Mirror(child, normal) => interval_mirror(child, normal, aabb),

            // Domain operations
            Self::Shell(child, thickness) => interval_shell(child, *thickness, aabb),
            Self::Round(child, radius) => interval_round(child, *radius, aabb),
            Self::Offset(child, distance) => interval_offset(child, *distance, aabb),
            Self::Elongate(child, half) => interval_elongate(child, half, aabb),
            Self::Twist(_, rate) => {
                // Lipschitz ≈ sqrt(1 + (rate · r_xy_max)²) where r_xy_max is
                // the maximum distance from Z axis within the AABB.
                let x_max = aabb.min.x.abs().max(aabb.max.x.abs());
                let y_max = aabb.min.y.abs().max(aabb.max.y.abs());
                let r_xy_max = x_max.hypot(y_max);
                let kr = rate * r_xy_max;
                let lipschitz = kr.mul_add(kr, 1.0).sqrt();
                lipschitz_bound(self, aabb, lipschitz)
            }
            Self::Bend(_, rate) => {
                // Same distortion analysis as Twist but in the XZ plane.
                let x_max = aabb.min.x.abs().max(aabb.max.x.abs());
                let z_max = aabb.min.z.abs().max(aabb.max.z.abs());
                let r_xz_max = x_max.hypot(z_max);
                let kr = rate * r_xz_max;
                let lipschitz = kr.mul_add(kr, 1.0).sqrt();
                lipschitz_bound(self, aabb, lipschitz)
            }
            Self::Repeat(_, _) | Self::RepeatBounded { .. } => {
                // The repeat fold is distance-preserving (Lipschitz = 1).
                lipschitz_bound(self, aabb, 1.0)
            }

            // User function
            Self::UserFn { interval, .. } => interval
                .as_ref()
                .map_or((f64::NEG_INFINITY, f64::INFINITY), |f| (f.0)(aabb)),
        }
    }

    /// Compute the Lipschitz distortion factor for this expression tree.
    ///
    /// Returns the factor by which domain distortion operations (Twist, Bend,
    /// Loft taper) amplify the field gradient relative to an exact SDF.
    /// For undistorted fields, returns 1.0.
    ///
    /// The mesher divides cell size by this factor to ensure thin features
    /// in high-distortion regions are not silently lost.
    ///
    /// Only accounts for geometric distortion — not primitive-level non-SDF
    /// effects (which the mesher handles via interval arithmetic).
    #[allow(clippy::too_many_lines)]
    pub(crate) fn lipschitz_factor(&self) -> f64 {
        match self {
            // ── Loft: distortion from taper ──────────────────────────
            Self::Loft { stations } => {
                let max_slope = stations
                    .windows(2)
                    .map(|w| {
                        let dz = w[1][0] - w[0][0];
                        if dz.abs() < 1e-15 {
                            0.0
                        } else {
                            ((w[1][1] - w[0][1]) / dz).abs()
                        }
                    })
                    .fold(0.0_f64, f64::max)
                    * 2.0; // Safety factor for cubic interpolation overshoot
                (1.0 + max_slope * max_slope).sqrt()
            }

            // ── Booleans: max of children ────────────────────────────
            Self::Union(a, b)
            | Self::Subtract(a, b)
            | Self::Intersect(a, b)
            | Self::SmoothUnion(a, b, _)
            | Self::SmoothSubtract(a, b, _)
            | Self::SmoothIntersect(a, b, _) => a.lipschitz_factor().max(b.lipschitz_factor()),

            Self::SmoothUnionAll(children, _) => children
                .iter()
                .map(Self::lipschitz_factor)
                .fold(1.0_f64, f64::max),

            // ── Transforms, domain ops, repeat (preserve child L) ────
            Self::Translate(child, _)
            | Self::Rotate(child, _)
            | Self::Mirror(child, _)
            | Self::ScaleUniform(child, _)
            | Self::Shell(child, _)
            | Self::Round(child, _)
            | Self::Offset(child, _)
            | Self::Elongate(child, _)
            | Self::Repeat(child, _)
            | Self::RepeatBounded { child, .. } => child.lipschitz_factor(),

            // ── Distortion ops (multiply by distortion factor) ───────
            Self::Twist(child, rate) => {
                // Lipschitz: √(1 + (rate · r_xy_max)²)
                // r_xy_max from child's bounding box.
                let r_xy_max = child.bounds().map_or(10.0, |bb| {
                    let x = bb.min.x.abs().max(bb.max.x.abs());
                    let y = bb.min.y.abs().max(bb.max.y.abs());
                    x.hypot(y)
                });
                let kr = rate * r_xy_max;
                let twist_l = kr.mul_add(kr, 1.0).sqrt();
                child.lipschitz_factor() * twist_l
            }
            Self::Bend(child, rate) => {
                // Same analysis as Twist but in XZ plane.
                let r_xz_max = child.bounds().map_or(10.0, |bb| {
                    let x = bb.min.x.abs().max(bb.max.x.abs());
                    let z = bb.min.z.abs().max(bb.max.z.abs());
                    x.hypot(z)
                });
                let kr = rate * r_xz_max;
                let bend_l = kr.mul_add(kr, 1.0).sqrt();
                child.lipschitz_factor() * bend_l
            }

            // ── Leaf primitives + user function (no distortion) ──────
            Self::Sphere { .. }
            | Self::Cuboid { .. }
            | Self::Cylinder { .. }
            | Self::Capsule { .. }
            | Self::Ellipsoid { .. }
            | Self::Torus { .. }
            | Self::Cone { .. }
            | Self::Plane { .. }
            | Self::Superellipsoid { .. }
            | Self::LogSpiral { .. }
            | Self::Gyroid { .. }
            | Self::SchwarzP { .. }
            | Self::Helix { .. }
            | Self::Pipe { .. }
            | Self::PipeSpline { .. }
            | Self::UserFn { .. } => 1.0,
        }
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────

/// Interval of `|p|` over an AABB — the range of distances from origin.
fn norm_interval(aabb: &Aabb) -> (f64, f64) {
    // Closest point to origin clamped to box
    let closest = Point3::new(
        0.0_f64.clamp(aabb.min.x, aabb.max.x),
        0.0_f64.clamp(aabb.min.y, aabb.max.y),
        0.0_f64.clamp(aabb.min.z, aabb.max.z),
    );
    let min_dist = closest.coords.norm();

    // Farthest point from origin is at a corner
    let max_dist = aabb
        .corners()
        .iter()
        .map(|c| c.coords.norm())
        .fold(0.0_f64, f64::max);

    (min_dist, max_dist)
}

/// Interval of `x.abs()` given `x` in `[lo, hi]`.
fn abs_interval(lo: f64, hi: f64) -> (f64, f64) {
    if lo >= 0.0 {
        (lo, hi)
    } else if hi <= 0.0 {
        (-hi, -lo)
    } else {
        (0.0, lo.abs().max(hi.abs()))
    }
}

/// Interval of the 2D norm `sqrt(x^2 + y^2)` given axis-aligned ranges.
fn norm2d_interval(x_range: (f64, f64), y_range: (f64, f64)) -> (f64, f64) {
    let x_abs = abs_interval(x_range.0, x_range.1);
    let y_abs = abs_interval(y_range.0, y_range.1);
    let min_norm = x_abs.0.hypot(y_abs.0);
    let max_norm = x_abs.1.hypot(y_abs.1);
    (min_norm, max_norm)
}

/// Interval of `x^2` given `x` in `[lo, hi]`.
fn sq_interval(lo: f64, hi: f64) -> (f64, f64) {
    if lo >= 0.0 {
        (lo * lo, hi * hi)
    } else if hi <= 0.0 {
        (hi * hi, lo * lo)
    } else {
        (0.0, lo.abs().max(hi.abs()).powi(2))
    }
}

// ── Primitive interval implementations ───────────────────────────────────

/// Sphere: `|p| - r`. Tight bounds via norm interval.
fn interval_sphere(radius: f64, aabb: &Aabb) -> (f64, f64) {
    let (min_norm, max_norm) = norm_interval(aabb);
    (min_norm - radius, max_norm - radius)
}

/// Cuboid: proper interval arithmetic on the exact SDF formula.
///
/// `SDF = length(max(q, 0)) + min(max(q.x, q.y, q.z), 0)`
/// where `q = |p| - half_extents` (component-wise).
fn interval_cuboid(half_extents: &Vector3<f64>, aabb: &Aabb) -> (f64, f64) {
    // q = |p| - half_extents per axis
    let x_abs = abs_interval(aabb.min.x, aabb.max.x);
    let qx = (x_abs.0 - half_extents.x, x_abs.1 - half_extents.x);
    let y_abs = abs_interval(aabb.min.y, aabb.max.y);
    let qy = (y_abs.0 - half_extents.y, y_abs.1 - half_extents.y);
    let z_abs = abs_interval(aabb.min.z, aabb.max.z);
    let qz = (z_abs.0 - half_extents.z, z_abs.1 - half_extents.z);

    // outside = length(max(q, 0))
    let ox = (qx.0.max(0.0), qx.1.max(0.0));
    let oy = (qy.0.max(0.0), qy.1.max(0.0));
    let oz = (qz.0.max(0.0), qz.1.max(0.0));
    let outside_lo = ox.0.hypot(oy.0).hypot(oz.0);
    let outside_hi = ox.1.hypot(oy.1).hypot(oz.1);

    // inside = min(max(q.x, q.y, q.z), 0)
    let max3_lo = qx.0.max(qy.0).max(qz.0);
    let max3_hi = qx.1.max(qy.1).max(qz.1);
    let inside_lo = max3_lo.min(0.0);
    let inside_hi = max3_hi.min(0.0);

    // SDF = outside + inside (conservative: they're correlated but this is safe)
    (outside_lo + inside_lo, outside_hi + inside_hi)
}

/// Z-aligned cylinder: proper interval arithmetic on the exact SDF formula.
///
/// `SDF = length(max(d_r, 0), max(d_z, 0)) + min(max(d_r, d_z), 0)`
/// where `d_r = sqrt(x²+y²) - radius`, `d_z = |z| - half_height`.
fn interval_cylinder(radius: f64, half_height: f64, aabb: &Aabb) -> (f64, f64) {
    // d_r = sqrt(x² + y²) - radius
    let (r_lo, r_hi) = norm2d_interval((aabb.min.x, aabb.max.x), (aabb.min.y, aabb.max.y));
    let dr = (r_lo - radius, r_hi - radius);

    // d_z = |z| - half_height
    let z_abs = abs_interval(aabb.min.z, aabb.max.z);
    let dz = (z_abs.0 - half_height, z_abs.1 - half_height);

    // outside = length(max(d_r, 0), max(d_z, 0))
    let outside_r = (dr.0.max(0.0), dr.1.max(0.0));
    let outside_z = (dz.0.max(0.0), dz.1.max(0.0));
    let outside_lo = outside_r.0.hypot(outside_z.0);
    let outside_hi = outside_r.1.hypot(outside_z.1);

    // inside = min(max(d_r, d_z), 0)
    let max2_lo = dr.0.max(dz.0);
    let max2_hi = dr.1.max(dz.1);
    let inside_lo = max2_lo.min(0.0);
    let inside_hi = max2_hi.min(0.0);

    (outside_lo + inside_lo, outside_hi + inside_hi)
}

/// Capsule: distance to line segment `[0,0,-h]..[0,0,h]` minus radius.
fn interval_capsule(radius: f64, half_height: f64, aabb: &Aabb) -> (f64, f64) {
    // Capsule SDF = distance_to_segment(p, axis) - radius.
    // Closest axis point: (0, 0, clamp(p.z, -h, h)).
    // Distance = sqrt(x² + y² + (z - clamp(z,-h,h))²) - radius.
    let (r_lo, r_hi) = norm2d_interval((aabb.min.x, aabb.max.x), (aabb.min.y, aabb.max.y));

    // Axial residual after clamping: z - clamp(z, -h, h)
    let z_lo = aabb.min.z;
    let z_hi = aabb.max.z;
    let residual_lo = (z_lo - half_height)
        .max(0.0)
        .min((z_lo + half_height).min(0.0));
    let residual_hi = (z_hi - half_height)
        .max(0.0)
        .max((z_hi + half_height).min(0.0));
    let res_abs = abs_interval(residual_lo, residual_hi);

    let (dist_lo, dist_hi) = norm2d_interval((r_lo, r_hi), res_abs);
    (dist_lo - radius, dist_hi - radius)
}

/// Ellipsoid: Lipschitz-corrected sampling.
///
/// The ellipsoid SDF approximation is not an exact SDF, so we use sampling
/// with a Lipschitz correction. The Lipschitz constant for the IQ ellipsoid
/// approximation is bounded by `max_radius / min_radius`.
fn interval_ellipsoid(radii: &Vector3<f64>, aabb: &Aabb) -> (f64, f64) {
    let node = FieldNode::Ellipsoid { radii: *radii };
    let max_r = radii.x.max(radii.y).max(radii.z);
    let min_r = radii.x.min(radii.y).min(radii.z);
    // Conservative Lipschitz bound for the IQ ellipsoid approximation
    let lipschitz = (max_r / min_r).max(1.0);
    lipschitz_bound(&node, aabb, lipschitz)
}

/// Torus: proper interval arithmetic on the exact SDF formula.
///
/// `SDF = sqrt(q_xy² + z²) - minor`
/// where `q_xy = sqrt(x² + y²) - major`.
fn interval_torus(major: f64, minor: f64, aabb: &Aabb) -> (f64, f64) {
    // q_xy = sqrt(x² + y²) - major
    let (r_lo, r_hi) = norm2d_interval((aabb.min.x, aabb.max.x), (aabb.min.y, aabb.max.y));
    let qxy = (r_lo - major, r_hi - major);

    // SDF = sqrt(q_xy² + z²) - minor
    let qxy_sq = sq_interval(qxy.0, qxy.1);
    let z_sq = sq_interval(aabb.min.z, aabb.max.z);
    let sum_lo = qxy_sq.0 + z_sq.0;
    let sum_hi = qxy_sq.1 + z_sq.1;

    (sum_lo.sqrt() - minor, sum_hi.sqrt() - minor)
}

/// Cone: Lipschitz-corrected sampling (L=1 for exact SDF).
fn interval_cone(radius: f64, height: f64, aabb: &Aabb) -> (f64, f64) {
    let node = FieldNode::Cone { radius, height };
    lipschitz_bound(&node, aabb, 1.0)
}

/// Plane: exact interval via dot product bounds.
fn interval_plane(normal: &Vector3<f64>, offset: f64, aabb: &Aabb) -> (f64, f64) {
    // dot(normal, p) - offset is linear: extremes are at corners.
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for corner in &aabb.corners() {
        let v = normal.dot(&corner.coords) - offset;
        if v < lo {
            lo = v;
        }
        if v > hi {
            hi = v;
        }
    }
    (lo, hi)
}

/// Lipschitz-corrected sampling bound.
///
/// Evaluates the field at strategic sample points within the AABB, then
/// expands the interval by `lipschitz * max_dist_to_nearest_sample` to
/// guarantee conservative bounds.
///
/// For a function with Lipschitz constant L, the value at any point p
/// satisfies: `|f(p) - f(s)| <= L * |p - s|` for the nearest sample s.
fn lipschitz_bound(node: &FieldNode, aabb: &Aabb, lipschitz: f64) -> (f64, f64) {
    let center = aabb.center();
    let half = aabb.half_extents();

    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;

    // Sample at 3x3x3 = 27 evenly spaced points (corners + face centers +
    // edge midpoints + center). This gives sub-cells of half-extent h/2.
    for ix in 0..=2_u8 {
        for iy in 0..=2_u8 {
            for iz in 0..=2_u8 {
                let off = Vector3::new(
                    (f64::from(ix) - 1.0) * half.x,
                    (f64::from(iy) - 1.0) * half.y,
                    (f64::from(iz) - 1.0) * half.z,
                );
                let p = Point3::from(center.coords + off);
                let v = node.evaluate(&p);
                if v < lo {
                    lo = v;
                }
                if v > hi {
                    hi = v;
                }
            }
        }
    }

    // Max distance from any point in the AABB to its nearest sample:
    // each sub-cell has half-extents (hx/2, hy/2, hz/2), so the
    // half-diagonal is the correction bound.
    let hx2 = half.x / 2.0;
    let hy2 = half.y / 2.0;
    let hz2 = half.z / 2.0;
    let correction = lipschitz * hx2.hypot(hy2).hypot(hz2);

    (lo - correction, hi + correction)
}

// ── Boolean interval implementations ────────────────────────────────────

fn interval_union(a: &FieldNode, b: &FieldNode, aabb: &Aabb) -> (f64, f64) {
    let (a_lo, a_hi) = a.evaluate_interval(aabb);
    let (b_lo, b_hi) = b.evaluate_interval(aabb);
    (a_lo.min(b_lo), a_hi.min(b_hi))
}

fn interval_subtract(a: &FieldNode, b: &FieldNode, aabb: &Aabb) -> (f64, f64) {
    // max(a, -b): negate b's interval, then take max
    let (a_lo, a_hi) = a.evaluate_interval(aabb);
    let (b_lo, b_hi) = b.evaluate_interval(aabb);
    (a_lo.max(-b_hi), a_hi.max(-b_lo))
}

fn interval_intersect(a: &FieldNode, b: &FieldNode, aabb: &Aabb) -> (f64, f64) {
    let (a_lo, a_hi) = a.evaluate_interval(aabb);
    let (b_lo, b_hi) = b.evaluate_interval(aabb);
    (a_lo.max(b_lo), a_hi.max(b_hi))
}

fn interval_smooth_union(a: &FieldNode, b: &FieldNode, k: f64, aabb: &Aabb) -> (f64, f64) {
    let (a_lo, a_hi) = a.evaluate_interval(aabb);
    let (b_lo, b_hi) = b.evaluate_interval(aabb);
    // smooth_union ≤ min(a, b); max correction is k/4
    (a_lo.min(b_lo) - k / 4.0, a_hi.min(b_hi))
}

fn interval_smooth_subtract(a: &FieldNode, b: &FieldNode, k: f64, aabb: &Aabb) -> (f64, f64) {
    // -smooth_union(-a, b, k)
    // -a ∈ [-a_hi, -a_lo], b ∈ [b_lo, b_hi]
    // smooth_union(-a, b) ∈ [min(-a_hi, b_lo) - k/4, min(-a_lo, b_hi)]
    // negate: [max(a_lo, -b_hi), max(a_hi, -b_lo) + k/4]
    let (a_lo, a_hi) = a.evaluate_interval(aabb);
    let (b_lo, b_hi) = b.evaluate_interval(aabb);
    (a_lo.max(-b_hi), a_hi.max(-b_lo) + k / 4.0)
}

fn interval_smooth_intersect(a: &FieldNode, b: &FieldNode, k: f64, aabb: &Aabb) -> (f64, f64) {
    // -smooth_union(-a, -b, k)
    // negate: [max(a_lo, b_lo), max(a_hi, b_hi) + k/4]
    let (a_lo, a_hi) = a.evaluate_interval(aabb);
    let (b_lo, b_hi) = b.evaluate_interval(aabb);
    (a_lo.max(b_lo), a_hi.max(b_hi) + k / 4.0)
}

#[allow(clippy::cast_precision_loss)] // n-ary child count is never > 2^52
fn interval_smooth_union_all(children: &[FieldNode], k: f64, aabb: &Aabb) -> (f64, f64) {
    if children.is_empty() {
        return (f64::INFINITY, f64::INFINITY);
    }
    let intervals: Vec<(f64, f64)> = children.iter().map(|c| c.evaluate_interval(aabb)).collect();
    let min_lo = intervals.iter().map(|i| i.0).fold(f64::INFINITY, f64::min);
    let min_hi = intervals.iter().map(|i| i.1).fold(f64::INFINITY, f64::min);
    let n = children.len() as f64;
    // smooth_union_all ≤ min(all); max correction is k*ln(n)
    (k.mul_add(-n.ln(), min_lo), min_hi)
}

// ── Transform interval implementations ──────────────────────────────────

fn interval_translate(child: &FieldNode, offset: &Vector3<f64>, aabb: &Aabb) -> (f64, f64) {
    // Shift AABB into child's local space
    let shifted = Aabb::new(
        Point3::new(
            aabb.min.x - offset.x,
            aabb.min.y - offset.y,
            aabb.min.z - offset.z,
        ),
        Point3::new(
            aabb.max.x - offset.x,
            aabb.max.y - offset.y,
            aabb.max.z - offset.z,
        ),
    );
    child.evaluate_interval(&shifted)
}

fn interval_rotate(
    child: &FieldNode,
    q: &nalgebra::UnitQuaternion<f64>,
    aabb: &Aabb,
) -> (f64, f64) {
    let inv = q.inverse();
    let corners = aabb.corners();
    let rotated: Vec<Point3<f64>> = corners.iter().map(|c| inv.transform_point(c)).collect();
    let rotated_aabb = Aabb::from_points(rotated.iter());
    child.evaluate_interval(&rotated_aabb)
}

fn interval_scale_uniform(child: &FieldNode, s: f64, aabb: &Aabb) -> (f64, f64) {
    let inv_s = 1.0 / s;
    let scaled = Aabb::new(
        Point3::new(aabb.min.x * inv_s, aabb.min.y * inv_s, aabb.min.z * inv_s),
        Point3::new(aabb.max.x * inv_s, aabb.max.y * inv_s, aabb.max.z * inv_s),
    );
    let (lo, hi) = child.evaluate_interval(&scaled);
    (lo * s, hi * s)
}

// ── Domain operation interval implementations ────────────────────────────

fn interval_shell(child: &FieldNode, thickness: f64, aabb: &Aabb) -> (f64, f64) {
    let (lo, hi) = child.evaluate_interval(aabb);
    let (abs_lo, abs_hi) = abs_interval(lo, hi);
    (abs_lo - thickness, abs_hi - thickness)
}

fn interval_round(child: &FieldNode, radius: f64, aabb: &Aabb) -> (f64, f64) {
    let (lo, hi) = child.evaluate_interval(aabb);
    (lo - radius, hi - radius)
}

fn interval_offset(child: &FieldNode, distance: f64, aabb: &Aabb) -> (f64, f64) {
    let (lo, hi) = child.evaluate_interval(aabb);
    (lo - distance, hi - distance)
}

fn interval_elongate(child: &FieldNode, half: &Vector3<f64>, aabb: &Aabb) -> (f64, f64) {
    // g(p) = p - clamp(p, -h, h) is monotonically non-decreasing per axis,
    // so the range of g over [lo, hi] is [g(lo), g(hi)].
    let q_min = Point3::new(
        aabb.min.x - aabb.min.x.clamp(-half.x, half.x),
        aabb.min.y - aabb.min.y.clamp(-half.y, half.y),
        aabb.min.z - aabb.min.z.clamp(-half.z, half.z),
    );
    let q_max = Point3::new(
        aabb.max.x - aabb.max.x.clamp(-half.x, half.x),
        aabb.max.y - aabb.max.y.clamp(-half.y, half.y),
        aabb.max.z - aabb.max.z.clamp(-half.z, half.z),
    );
    let q_aabb = Aabb::new(q_min, q_max);
    child.evaluate_interval(&q_aabb)
}

fn interval_mirror(child: &FieldNode, normal: &Vector3<f64>, aabb: &Aabb) -> (f64, f64) {
    // Mirror maps p → p' where dot(p', n) = |dot(p, n)|.
    // When the AABB straddles the mirror plane, we include corner projections
    // onto the plane to cover the near-plane gap.
    let corners = aabb.corners();
    let mut has_positive = false;
    let mut has_negative = false;
    let mut points: Vec<Point3<f64>> = Vec::with_capacity(16);

    for c in &corners {
        let d = c.coords.dot(normal);
        if d >= 0.0 {
            has_positive = true;
        }
        if d < 0.0 {
            has_negative = true;
        }
        let two_d_neg = 2.0 * d.min(0.0);
        points.push(Point3::new(
            two_d_neg.mul_add(-normal.x, c.x),
            two_d_neg.mul_add(-normal.y, c.y),
            two_d_neg.mul_add(-normal.z, c.z),
        ));
    }

    if has_positive && has_negative {
        // AABB straddles the mirror plane — include projections of
        // corners onto the plane to cover the near-plane gap.
        for c in &corners {
            let d = c.coords.dot(normal);
            points.push(Point3::new(
                d.mul_add(-normal.x, c.x),
                d.mul_add(-normal.y, c.y),
                d.mul_add(-normal.z, c.z),
            ));
        }
    }

    let child_aabb = Aabb::from_points(points.iter());
    child.evaluate_interval(&child_aabb)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    /// Helper: verify that interval bounds contain the point evaluation
    /// for a grid of test points inside the AABB.
    fn verify_interval_contains_points(node: &FieldNode, aabb: &Aabb, samples_per_axis: u32) {
        let (lo, hi) = node.evaluate_interval(aabb);
        let size = aabb.max - aabb.min;

        let n = f64::from(samples_per_axis);
        for ix in 0..=samples_per_axis {
            for iy in 0..=samples_per_axis {
                for iz in 0..=samples_per_axis {
                    let tx = f64::from(ix) / n;
                    let ty = f64::from(iy) / n;
                    let tz = f64::from(iz) / n;
                    let p = Point3::new(
                        tx.mul_add(size.x, aabb.min.x),
                        ty.mul_add(size.y, aabb.min.y),
                        tz.mul_add(size.z, aabb.min.z),
                    );
                    let v = node.evaluate(&p);
                    assert!(
                        v >= lo - 1e-10 && v <= hi + 1e-10,
                        "Interval violation at {p:?}: value={v}, interval=({lo}, {hi})"
                    );
                }
            }
        }
    }

    /// Standard test AABBs covering various positions relative to origin.
    fn test_aabbs() -> Vec<Aabb> {
        vec![
            // Box containing origin
            Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0)),
            // Box fully outside
            Aabb::new(Point3::new(5.0, 5.0, 5.0), Point3::new(6.0, 6.0, 6.0)),
            // Box straddling surface
            Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(3.0, 3.0, 3.0)),
            // Small box near origin
            Aabb::new(Point3::new(-0.1, -0.1, -0.1), Point3::new(0.1, 0.1, 0.1)),
            // Box offset in one axis
            Aabb::new(Point3::new(2.0, -0.5, -0.5), Point3::new(4.0, 0.5, 0.5)),
        ]
    }

    #[test]
    fn sphere_interval_contains_points() {
        let node = FieldNode::Sphere { radius: 3.0 };
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn sphere_interval_tight_outside() {
        let node = FieldNode::Sphere { radius: 1.0 };
        // Box fully outside the sphere
        let aabb = Aabb::new(Point3::new(3.0, 0.0, 0.0), Point3::new(4.0, 1.0, 1.0));
        let (lo, _hi) = node.evaluate_interval(&aabb);
        assert!(
            lo > 0.0,
            "Sphere interval should be positive for fully-outside box"
        );
    }

    #[test]
    fn sphere_interval_tight_inside() {
        let node = FieldNode::Sphere { radius: 10.0 };
        // Small box fully inside the sphere
        let aabb = Aabb::new(Point3::new(-0.1, -0.1, -0.1), Point3::new(0.1, 0.1, 0.1));
        let (_lo, hi) = node.evaluate_interval(&aabb);
        assert!(
            hi < 0.0,
            "Sphere interval should be negative for fully-inside box"
        );
    }

    #[test]
    fn cuboid_interval_contains_points() {
        let node = FieldNode::Cuboid {
            half_extents: Vector3::new(2.0, 3.0, 1.0),
        };
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn cylinder_interval_contains_points() {
        let node = FieldNode::Cylinder {
            radius: 2.0,
            half_height: 3.0,
        };
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn capsule_interval_contains_points() {
        let node = FieldNode::Capsule {
            radius: 1.5,
            half_height: 2.0,
        };
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn ellipsoid_interval_contains_points() {
        let node = FieldNode::Ellipsoid {
            radii: Vector3::new(2.0, 3.0, 4.0),
        };
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn torus_interval_contains_points() {
        let node = FieldNode::Torus {
            major: 5.0,
            minor: 1.0,
        };
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn cone_interval_contains_points() {
        let node = FieldNode::Cone {
            radius: 2.0,
            height: 4.0,
        };
        let aabbs = vec![
            Aabb::new(Point3::new(-1.0, -1.0, -3.0), Point3::new(1.0, 1.0, -1.0)),
            Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0)),
            Aabb::new(Point3::new(3.0, 0.0, -2.0), Point3::new(5.0, 1.0, 0.0)),
            Aabb::new(Point3::new(-0.5, -0.5, -4.5), Point3::new(0.5, 0.5, -3.5)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn plane_interval_contains_points() {
        let node = FieldNode::Plane {
            normal: Vector3::new(0.0, 0.0, 1.0),
            offset: 2.0,
        };
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn plane_interval_exact_for_axis_aligned() {
        // For an axis-aligned plane, the interval should be exact.
        let node = FieldNode::Plane {
            normal: Vector3::new(0.0, 0.0, 1.0),
            offset: 0.0,
        };
        let aabb = Aabb::new(Point3::new(-1.0, -1.0, 2.0), Point3::new(1.0, 1.0, 5.0));
        let (lo, hi) = node.evaluate_interval(&aabb);
        assert!((lo - 2.0).abs() < 1e-10);
        assert!((hi - 5.0).abs() < 1e-10);
    }

    // ── Boolean interval tests ──────────────────────────────────────

    #[test]
    fn union_interval_contains_points() {
        let a = FieldNode::Sphere { radius: 2.0 };
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let u = FieldNode::Union(Box::new(a), Box::new(b));
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&u, aabb, 5);
        }
    }

    #[test]
    fn subtract_interval_contains_points() {
        let a = FieldNode::Sphere { radius: 5.0 };
        let b = FieldNode::Sphere { radius: 2.0 };
        let sub = FieldNode::Subtract(Box::new(a), Box::new(b));
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&sub, aabb, 5);
        }
    }

    #[test]
    fn intersect_interval_contains_points() {
        let a = FieldNode::Sphere { radius: 5.0 };
        let b = FieldNode::Cuboid {
            half_extents: Vector3::new(3.0, 3.0, 3.0),
        };
        let inter = FieldNode::Intersect(Box::new(a), Box::new(b));
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&inter, aabb, 5);
        }
    }

    #[test]
    fn smooth_union_interval_contains_points() {
        let a = FieldNode::Sphere { radius: 2.0 };
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let su = FieldNode::SmoothUnion(Box::new(a), Box::new(b), 1.0);
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&su, aabb, 5);
        }
    }

    #[test]
    fn smooth_subtract_interval_contains_points() {
        let a = FieldNode::Sphere { radius: 5.0 };
        let b = FieldNode::Sphere { radius: 2.0 };
        let ss = FieldNode::SmoothSubtract(Box::new(a), Box::new(b), 1.0);
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&ss, aabb, 5);
        }
    }

    #[test]
    fn smooth_intersect_interval_contains_points() {
        let a = FieldNode::Sphere { radius: 5.0 };
        let b = FieldNode::Cuboid {
            half_extents: Vector3::new(3.0, 3.0, 3.0),
        };
        let si = FieldNode::SmoothIntersect(Box::new(a), Box::new(b), 1.0);
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&si, aabb, 5);
        }
    }

    #[test]
    fn smooth_union_all_interval_contains_points() {
        let a = FieldNode::Sphere { radius: 2.0 };
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let c = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(0.0, 3.0, 0.0),
        );
        let sua = FieldNode::SmoothUnionAll(vec![a, b, c], 1.0);
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&sua, aabb, 5);
        }
    }

    // ── Transform interval tests ────────────────────────────────────

    #[test]
    fn translate_interval_contains_points() {
        let node = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 3.0 }),
            Vector3::new(5.0, 0.0, 0.0),
        );
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn rotate_interval_contains_points() {
        let rot = nalgebra::UnitQuaternion::from_axis_angle(
            &Vector3::z_axis(),
            std::f64::consts::FRAC_PI_4,
        );
        let node = FieldNode::Rotate(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 2.0, 3.0),
            }),
            rot,
        );
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn scale_uniform_interval_contains_points() {
        let node = FieldNode::ScaleUniform(Box::new(FieldNode::Sphere { radius: 1.0 }), 3.0);
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn mirror_interval_contains_points() {
        let node = FieldNode::Mirror(
            Box::new(FieldNode::Translate(
                Box::new(FieldNode::Sphere { radius: 1.0 }),
                Vector3::new(3.0, 0.0, 0.0),
            )),
            Vector3::new(1.0, 0.0, 0.0),
        );
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    // ── Composed tree interval tests ────────────────────────────────

    #[test]
    fn union_translated_spheres_interval() {
        let a = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(-3.0, 0.0, 0.0),
        );
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let u = FieldNode::Union(Box::new(a), Box::new(b));
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&u, aabb, 5);
        }
    }

    #[test]
    fn subtract_then_scale_interval() {
        let big = FieldNode::Sphere { radius: 5.0 };
        let small = FieldNode::Sphere { radius: 2.0 };
        let sub = FieldNode::Subtract(Box::new(big), Box::new(small));
        let scaled = FieldNode::ScaleUniform(Box::new(sub), 2.0);
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&scaled, aabb, 5);
        }
    }

    // ── Domain operation interval tests ────────────────────────────────

    #[test]
    fn shell_interval_contains_points() {
        let node = FieldNode::Shell(Box::new(FieldNode::Sphere { radius: 5.0 }), 1.0);
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn shell_interval_prunes_interior() {
        // Small box deep inside the sphere — shell should be fully outside (positive)
        let node = FieldNode::Shell(Box::new(FieldNode::Sphere { radius: 10.0 }), 1.0);
        let aabb = Aabb::new(Point3::new(-0.1, -0.1, -0.1), Point3::new(0.1, 0.1, 0.1));
        let (lo, _hi) = node.evaluate_interval(&aabb);
        assert!(
            lo > 0.0,
            "Shell interval should be positive for box deep inside sphere"
        );
    }

    #[test]
    fn round_interval_contains_points() {
        let node = FieldNode::Round(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(2.0, 2.0, 2.0),
            }),
            0.5,
        );
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn offset_interval_contains_points() {
        let node = FieldNode::Offset(Box::new(FieldNode::Sphere { radius: 3.0 }), 1.0);
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn offset_shrink_interval_contains_points() {
        let node = FieldNode::Offset(Box::new(FieldNode::Sphere { radius: 3.0 }), -1.0);
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn elongate_interval_contains_points() {
        let node = FieldNode::Elongate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn elongate_3axis_interval_contains_points() {
        let node = FieldNode::Elongate(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
            Vector3::new(1.0, 2.0, 3.0),
        );
        let aabbs = vec![
            Aabb::new(Point3::new(-3.0, -3.0, -5.0), Point3::new(3.0, 3.0, 5.0)),
            Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)),
            Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5)),
            Aabb::new(Point3::new(2.0, 3.0, 4.0), Point3::new(3.0, 4.0, 5.0)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    // ── UserFn interval tests ──────────────────────────────────────────

    #[test]
    fn user_fn_with_interval_contains_points() {
        use crate::field_node::{UserEvalFn, UserIntervalFn};
        use std::sync::Arc;

        // Custom sphere r=4 with a user-provided interval function
        let node = FieldNode::UserFn {
            eval: UserEvalFn(Arc::new(|p: Point3<f64>| p.coords.norm() - 4.0)),
            interval: Some(UserIntervalFn(Arc::new(|aabb: &Aabb| {
                // Reuse norm_interval logic inline
                let closest = Point3::new(
                    0.0_f64.clamp(aabb.min.x, aabb.max.x),
                    0.0_f64.clamp(aabb.min.y, aabb.max.y),
                    0.0_f64.clamp(aabb.min.z, aabb.max.z),
                );
                let min_dist = closest.coords.norm();
                let max_dist = aabb
                    .corners()
                    .iter()
                    .map(|c| c.coords.norm())
                    .fold(0.0_f64, f64::max);
                (min_dist - 4.0, max_dist - 4.0)
            }))),
            bounds: Aabb::new(Point3::new(-5.0, -5.0, -5.0), Point3::new(5.0, 5.0, 5.0)),
        };
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn user_fn_without_interval_returns_full_range() {
        use crate::field_node::UserEvalFn;
        use std::sync::Arc;

        let node = FieldNode::UserFn {
            eval: UserEvalFn(Arc::new(|p: Point3<f64>| p.coords.norm() - 4.0)),
            interval: None,
            bounds: Aabb::new(Point3::new(-5.0, -5.0, -5.0), Point3::new(5.0, 5.0, 5.0)),
        };
        let aabb = Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0));
        let (lo, hi) = node.evaluate_interval(&aabb);
        assert!(lo == f64::NEG_INFINITY);
        assert!(hi == f64::INFINITY);
    }

    // ── Pipe interval tests ──────────────────────────────────────────

    #[test]
    fn pipe_interval_contains_points() {
        let node = FieldNode::Pipe {
            vertices: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(5.0, 0.0, 0.0),
                Point3::new(5.0, 5.0, 0.0),
            ],
            radius: 1.0,
        };
        let aabbs = vec![
            Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0)),
            Aabb::new(Point3::new(2.0, -1.0, -1.0), Point3::new(4.0, 1.0, 1.0)),
            Aabb::new(Point3::new(4.0, 2.0, -1.0), Point3::new(6.0, 4.0, 1.0)),
            Aabb::new(Point3::new(8.0, 8.0, 0.0), Point3::new(10.0, 10.0, 1.0)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn pipe_spline_interval_contains_points() {
        let node = FieldNode::PipeSpline {
            control_points: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(5.0, 0.0, 0.0),
                Point3::new(5.0, 5.0, 0.0),
            ],
            radius: 1.0,
        };
        let aabbs = vec![
            Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0)),
            Aabb::new(Point3::new(2.0, -1.0, -1.0), Point3::new(4.0, 1.0, 1.0)),
            Aabb::new(Point3::new(4.0, 2.0, -1.0), Point3::new(6.0, 4.0, 1.0)),
            Aabb::new(Point3::new(8.0, 8.0, 0.0), Point3::new(10.0, 10.0, 1.0)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    // ── Bio-inspired primitive interval tests ──────────────────────────

    #[test]
    fn superellipsoid_interval_contains_points() {
        let node = FieldNode::Superellipsoid {
            radii: Vector3::new(2.0, 3.0, 4.0),
            n1: 2.0,
            n2: 2.0,
        };
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn superellipsoid_octahedron_interval_contains_points() {
        let node = FieldNode::Superellipsoid {
            radii: Vector3::new(2.0, 2.0, 2.0),
            n1: 1.0,
            n2: 1.0,
        };
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn log_spiral_interval_contains_points() {
        let node = FieldNode::LogSpiral {
            a: 2.0,
            b: 0.15,
            thickness: 0.5,
            turns: 1.5,
        };
        let aabbs = vec![
            Aabb::new(Point3::new(1.0, -1.0, -0.5), Point3::new(3.0, 1.0, 0.5)),
            Aabb::new(Point3::new(-3.0, -3.0, -0.5), Point3::new(3.0, 3.0, 0.5)),
            Aabb::new(Point3::new(5.0, 5.0, -0.5), Point3::new(6.0, 6.0, 0.5)),
            Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn gyroid_interval_contains_points() {
        let node = FieldNode::Gyroid {
            scale: 1.0,
            thickness: 0.5,
        };
        let aabbs = vec![
            Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0)),
            Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0)),
            Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5)),
            Aabb::new(Point3::new(3.0, 3.0, 3.0), Point3::new(4.0, 4.0, 4.0)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn schwarz_p_interval_contains_points() {
        let node = FieldNode::SchwarzP {
            scale: 1.0,
            thickness: 0.5,
        };
        let aabbs = vec![
            Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0)),
            Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0)),
            Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5)),
            Aabb::new(Point3::new(3.0, 3.0, 3.0), Point3::new(4.0, 4.0, 4.0)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn helix_interval_contains_points() {
        let node = FieldNode::Helix {
            radius: 3.0,
            pitch: 2.0,
            thickness: 0.5,
            turns: 2.0,
        };
        let aabbs = vec![
            Aabb::new(Point3::new(2.0, -1.0, -0.5), Point3::new(4.0, 1.0, 0.5)),
            Aabb::new(Point3::new(-4.0, -4.0, 0.0), Point3::new(4.0, 4.0, 4.0)),
            Aabb::new(Point3::new(-1.0, -1.0, 1.0), Point3::new(1.0, 1.0, 3.0)),
            Aabb::new(Point3::new(5.0, 5.0, 0.0), Point3::new(6.0, 6.0, 1.0)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    // ── Loft interval tests ──────────────────────────────────────────

    #[test]
    fn loft_constant_radius_interval_contains_points() {
        let node = FieldNode::Loft {
            stations: vec![[-5.0, 2.0], [5.0, 2.0]],
        };
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn loft_tapered_interval_contains_points() {
        let node = FieldNode::Loft {
            stations: vec![[-3.0, 3.0], [3.0, 1.0]],
        };
        let aabbs = vec![
            Aabb::new(Point3::new(-1.0, -1.0, -3.0), Point3::new(1.0, 1.0, 3.0)),
            Aabb::new(Point3::new(-3.0, -3.0, -3.0), Point3::new(3.0, 3.0, 3.0)),
            Aabb::new(Point3::new(0.0, 0.0, -1.0), Point3::new(2.0, 2.0, 1.0)),
            Aabb::new(Point3::new(5.0, 5.0, 0.0), Point3::new(6.0, 6.0, 1.0)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn loft_multi_station_interval_contains_points() {
        let node = FieldNode::Loft {
            stations: vec![[-4.0, 1.0], [-1.0, 3.0], [1.0, 2.0], [4.0, 1.0]],
        };
        let aabbs = vec![
            Aabb::new(Point3::new(-3.0, -3.0, -4.0), Point3::new(3.0, 3.0, 4.0)),
            Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0)),
            Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0)),
            Aabb::new(Point3::new(5.0, 5.0, 0.0), Point3::new(6.0, 6.0, 1.0)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    // ── Twist interval tests ────────────────────────────────────────

    #[test]
    fn twist_interval_contains_points() {
        let node = FieldNode::Twist(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 2.0, 3.0),
            }),
            0.5,
        );
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn twist_high_rate_interval_contains_points() {
        let node = FieldNode::Twist(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 1.0, 5.0),
            }),
            2.0,
        );
        let aabbs = vec![
            Aabb::new(Point3::new(-2.0, -2.0, -5.0), Point3::new(2.0, 2.0, 5.0)),
            Aabb::new(Point3::new(-0.5, -0.5, 0.0), Point3::new(0.5, 0.5, 1.0)),
            Aabb::new(Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 1.0, 1.0)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    // ── Bend interval tests ─────────────────────────────────────────

    #[test]
    fn bend_interval_contains_points() {
        let node = FieldNode::Bend(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 1.0, 5.0),
            }),
            0.2,
        );
        for aabb in &test_aabbs() {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn bend_moderate_rate_interval_contains_points() {
        let node = FieldNode::Bend(
            Box::new(FieldNode::Cylinder {
                radius: 1.0,
                half_height: 4.0,
            }),
            0.3,
        );
        let aabbs = vec![
            Aabb::new(Point3::new(-2.0, -2.0, -5.0), Point3::new(2.0, 2.0, 5.0)),
            Aabb::new(Point3::new(-0.5, -0.5, 0.0), Point3::new(0.5, 0.5, 1.0)),
            Aabb::new(Point3::new(3.0, 0.0, 0.0), Point3::new(4.0, 1.0, 1.0)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    // ── Repeat interval tests ────────────────────────────────────────

    #[test]
    fn repeat_interval_contains_points() {
        let node = FieldNode::Repeat(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
            Vector3::new(5.0, 5.0, 5.0),
        );
        let aabbs = vec![
            // Inside one cell
            Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0)),
            // Offset copy
            Aabb::new(Point3::new(4.0, -1.0, -1.0), Point3::new(6.0, 1.0, 1.0)),
            // Spanning multiple cells
            Aabb::new(Point3::new(-3.0, -1.0, -1.0), Point3::new(8.0, 1.0, 1.0)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    #[test]
    fn repeat_bounded_interval_contains_points() {
        let node = FieldNode::RepeatBounded {
            child: Box::new(FieldNode::Sphere { radius: 1.0 }),
            spacing: Vector3::new(5.0, 5.0, 5.0),
            count: [3, 1, 1],
        };
        let aabbs = vec![
            // Center copy
            Aabb::new(Point3::new(-2.0, -2.0, -2.0), Point3::new(2.0, 2.0, 2.0)),
            // Near right copy
            Aabb::new(Point3::new(3.0, -2.0, -2.0), Point3::new(7.0, 2.0, 2.0)),
            // Beyond array (clamped region)
            Aabb::new(Point3::new(8.0, -1.0, -1.0), Point3::new(12.0, 1.0, 1.0)),
        ];
        for aabb in &aabbs {
            verify_interval_contains_points(&node, aabb, 5);
        }
    }

    // ── Lipschitz factor tests ───────────────────────────────────────

    #[test]
    fn lipschitz_factor_exact_sdf_primitives_are_1() {
        let cases: Vec<FieldNode> = vec![
            FieldNode::Sphere { radius: 5.0 },
            FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 2.0, 3.0),
            },
            FieldNode::Cylinder {
                radius: 2.0,
                half_height: 3.0,
            },
            FieldNode::Capsule {
                radius: 1.0,
                half_height: 2.0,
            },
            FieldNode::Torus {
                major: 5.0,
                minor: 1.0,
            },
            FieldNode::Cone {
                radius: 2.0,
                height: 3.0,
            },
            FieldNode::Plane {
                normal: Vector3::new(0.0, 0.0, 1.0),
                offset: 0.0,
            },
            FieldNode::Pipe {
                vertices: vec![Point3::origin(), Point3::new(5.0, 0.0, 0.0)],
                radius: 0.5,
            },
        ];
        for node in &cases {
            assert!(
                (node.lipschitz_factor() - 1.0).abs() < 1e-10,
                "Expected L=1 for {node:?}, got {}",
                node.lipschitz_factor()
            );
        }
    }

    #[test]
    fn lipschitz_factor_loft_constant_radius_is_1() {
        let node = FieldNode::Loft {
            stations: vec![[-5.0, 2.0], [5.0, 2.0]],
        };
        assert!(
            (node.lipschitz_factor() - 1.0).abs() < 1e-10,
            "Constant-radius loft should have L=1, got {}",
            node.lipschitz_factor()
        );
    }

    #[test]
    fn lipschitz_factor_loft_tapered_is_gt_1() {
        let node = FieldNode::Loft {
            stations: vec![[-5.0, 3.0], [5.0, 1.0]],
        };
        let lip = node.lipschitz_factor();
        assert!(lip > 1.0, "Tapered loft should have L > 1, got {lip}");
    }

    #[test]
    fn lipschitz_factor_twist_is_gt_1() {
        let node = FieldNode::Twist(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(3.0, 3.0, 10.0),
            }),
            1.0,
        );
        let lip = node.lipschitz_factor();
        // Expected: sqrt(1 + (1.0 * sqrt(3^2+3^2))^2) = sqrt(1 + 18) ≈ 4.36
        let r_xy = 3.0_f64.hypot(3.0);
        let expected = (1.0 + r_xy * r_xy).sqrt();
        assert!(
            (lip - expected).abs() < 1e-10,
            "Twist L should be {expected}, got {lip}"
        );
    }

    #[test]
    fn lipschitz_factor_bend_is_gt_1() {
        let node = FieldNode::Bend(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(2.0, 1.0, 8.0),
            }),
            0.5,
        );
        let lip = node.lipschitz_factor();
        // Expected: sqrt(1 + (0.5 * sqrt(2^2+8^2))^2)
        let r_xz = 2.0_f64.hypot(8.0);
        let kr = 0.5 * r_xz;
        let expected = (1.0 + kr * kr).sqrt();
        assert!(
            (lip - expected).abs() < 1e-10,
            "Bend L should be {expected}, got {lip}"
        );
    }

    #[test]
    fn lipschitz_factor_propagates_through_booleans() {
        let twisted = FieldNode::Twist(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(3.0, 3.0, 10.0),
            }),
            1.0,
        );
        let twist_l = twisted.lipschitz_factor();

        let sphere = FieldNode::Sphere { radius: 2.0 };
        let union = FieldNode::Union(Box::new(twisted), Box::new(sphere));

        // Union should propagate max L
        assert!(
            (union.lipschitz_factor() - twist_l).abs() < 1e-10,
            "Union should propagate twist's L={twist_l}, got {}",
            union.lipschitz_factor()
        );
    }

    #[test]
    fn lipschitz_factor_propagates_through_transforms() {
        let twisted = FieldNode::Twist(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(3.0, 3.0, 10.0),
            }),
            1.0,
        );
        let twist_l = twisted.lipschitz_factor();

        let translated = FieldNode::Translate(Box::new(twisted), Vector3::new(10.0, 0.0, 0.0));
        assert!(
            (translated.lipschitz_factor() - twist_l).abs() < 1e-10,
            "Translate should preserve L={twist_l}, got {}",
            translated.lipschitz_factor()
        );
    }

    #[test]
    fn lipschitz_factor_repeat_preserves_child() {
        let twisted = FieldNode::Twist(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(2.0, 2.0, 5.0),
            }),
            0.5,
        );
        let twist_l = twisted.lipschitz_factor();

        let repeated = FieldNode::Repeat(Box::new(twisted), Vector3::new(10.0, 10.0, 10.0));
        assert!(
            (repeated.lipschitz_factor() - twist_l).abs() < 1e-10,
            "Repeat should preserve child's L={twist_l}, got {}",
            repeated.lipschitz_factor()
        );
    }

    #[test]
    fn lipschitz_factor_undistorted_tree_is_1() {
        // A complex tree with no distortion should have L=1
        let a = FieldNode::Sphere { radius: 3.0 };
        let b = FieldNode::Translate(
            Box::new(FieldNode::Cuboid {
                half_extents: Vector3::new(1.0, 1.0, 1.0),
            }),
            Vector3::new(5.0, 0.0, 0.0),
        );
        let node = FieldNode::SmoothUnion(Box::new(a), Box::new(b), 0.5);
        assert!(
            (node.lipschitz_factor() - 1.0).abs() < 1e-10,
            "Undistorted tree should have L=1, got {}",
            node.lipschitz_factor()
        );
    }
}
