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
}
