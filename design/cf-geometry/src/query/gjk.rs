//! GJK (Gilbert-Johnson-Keerthi) distance algorithm.
//!
//! Computes the minimum separating distance between two convex shapes, or
//! detects that they overlap. Works generically over any type implementing
//! [`SupportMap`].
//!
//! # Local space
//!
//! The algorithm operates on `SupportMap` implementors directly. For world-space
//! queries, the consuming layer (sim-core) wraps `(Shape, Pose)` into a type
//! implementing `SupportMap` that applies the pose transform.
//!
//! # References
//!
//! - Gilbert, Johnson, Keerthi: "A Fast Procedure for Computing the Distance
//!   Between Complex Objects in Three-Dimensional Space" (1988)
//! - van den Bergen: "Collision Detection in Interactive 3D Environments" (2003)

use nalgebra::{Point3, Vector3};

use crate::SupportMap;

/// Numerical tolerance.
const EPSILON: f64 = 1e-8;

/// Default max GJK iterations (35).
pub const GJK_MAX_ITERATIONS: usize = 35;

/// Result of a GJK distance query between two convex shapes.
#[derive(Debug, Clone)]
pub struct GjkDistance {
    /// Minimum separating distance (> 0 when separated).
    pub distance: f64,
    /// Closest point on shape A.
    pub point_a: Point3<f64>,
    /// Closest point on shape B.
    pub point_b: Point3<f64>,
}

/// Result of a GJK intersection query (internal, feeds EPA).
#[derive(Debug, Clone)]
pub struct GjkResult {
    /// Whether the shapes intersect (origin inside Minkowski difference).
    pub intersecting: bool,
    /// Final simplex (for EPA if intersecting).
    pub simplex: Simplex,
}

/// A point in Minkowski difference space, tracking the original support points.
#[derive(Debug, Clone, Copy, Default)]
pub struct MinkowskiPoint {
    /// Point in Minkowski space: `support_a - support_b`.
    pub point: Point3<f64>,
    /// Support point from shape A.
    pub support_a: Point3<f64>,
    /// Support point from shape B.
    pub support_b: Point3<f64>,
}

impl MinkowskiPoint {
    fn new(support_a: Point3<f64>, support_b: Point3<f64>) -> Self {
        Self {
            point: Point3::from(support_a - support_b),
            support_a,
            support_b,
        }
    }
}

/// Simplex used in GJK iteration (1–4 points).
#[derive(Debug, Clone, Default)]
pub struct Simplex {
    points: [MinkowskiPoint; 4],
    size: usize,
}

impl Simplex {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, point: MinkowskiPoint) {
        // Shift existing points to make room at index 0
        for i in (1..=self.size.min(3)).rev() {
            self.points[i] = self.points[i - 1];
        }
        self.points[0] = point;
        self.size = (self.size + 1).min(4);
    }

    pub const fn len(&self) -> usize {
        self.size
    }

    pub fn points(&self) -> &[MinkowskiPoint] {
        &self.points[..self.size]
    }

    fn set(&mut self, points: &[MinkowskiPoint]) {
        self.size = points.len().min(4);
        for (i, p) in points.iter().take(4).enumerate() {
            self.points[i] = *p;
        }
    }
}

// =============================================================================
// Public API
// =============================================================================

/// Compute minimum separating distance between two convex shapes.
///
/// Returns `Some(GjkDistance)` if the shapes are separated, with the distance
/// and witness points (closest points on each shape). Returns `None` if the
/// shapes overlap (use [`epa_penetration`](super::epa_penetration) instead).
///
/// # Arguments
///
/// * `a`, `b` — convex shapes implementing `SupportMap`
/// * `max_iterations` — iteration limit (35 is typical)
/// * `tolerance` — convergence tolerance (1e-6 is typical)
#[must_use]
pub fn gjk_distance(
    a: &impl SupportMap,
    b: &impl SupportMap,
    max_iterations: usize,
    tolerance: f64,
) -> Option<GjkDistance> {
    let center_a = a.aabb().center();
    let center_b = b.aabb().center();
    let dir_vec = center_b - center_a;
    let dir_norm = dir_vec.norm();

    let mut direction = if dir_norm > EPSILON {
        dir_vec / dir_norm
    } else {
        Vector3::x()
    };

    let mut simplex = Simplex::new();

    // First support point
    let first = support_minkowski(a, b, &direction);
    simplex.push(first);

    let mut closest_dist_sq = first.point.coords.norm_squared();
    direction = -first.point.coords;

    for _iteration in 0..max_iterations {
        let dir_norm_sq = direction.norm_squared();
        if dir_norm_sq < EPSILON * EPSILON {
            return None; // Origin on simplex — shapes touching
        }

        let dir_normalized = direction / dir_norm_sq.sqrt();
        let new_point = support_minkowski(a, b, &dir_normalized);

        // Negative dot means no new support point can reduce the distance.
        // However, we must NOT terminate early when the simplex is small
        // (size < 3): box support functions return corner vertices, and we
        // need multiple iterations to build a simplex that spans the closest
        // face for accurate witness point interpolation. Terminating with a
        // 1-point simplex gives wildly wrong normals for axis-aligned
        // box-sphere separation.
        let dot = new_point.point.coords.dot(&dir_normalized);
        if dot < EPSILON && simplex.len() >= 3 {
            break;
        }

        simplex.push(new_point);

        // Find closest point on simplex to origin
        let (closest_point, bary) = closest_point_on_simplex_to_origin(&simplex);
        let new_dist_sq = closest_point.norm_squared();

        // Convergence check
        let improvement = closest_dist_sq - new_dist_sq;
        if improvement < tolerance * tolerance
            || (closest_dist_sq > EPSILON && improvement / closest_dist_sq < tolerance)
        {
            let (wa, wb) = recover_witness_points(&simplex, &bary);
            return Some(GjkDistance {
                distance: new_dist_sq.sqrt(),
                point_a: wa,
                point_b: wb,
            });
        }

        closest_dist_sq = new_dist_sq;
        reduce_simplex_to_closest(&mut simplex, &bary);
        direction = -closest_point;
    }

    // Final result
    let (closest_point, bary) = closest_point_on_simplex_to_origin(&simplex);
    let dist = closest_point.norm();
    if dist < EPSILON {
        return None; // Touching
    }
    let (wa, wb) = recover_witness_points(&simplex, &bary);
    Some(GjkDistance {
        distance: dist,
        point_a: wa,
        point_b: wb,
    })
}

/// Check if two convex shapes intersect.
#[must_use]
pub fn gjk_intersection(a: &impl SupportMap, b: &impl SupportMap) -> bool {
    gjk_query(a, b, GJK_MAX_ITERATIONS).intersecting
}

// =============================================================================
// Internal GJK (used by EPA)
// =============================================================================

/// Full GJK query returning simplex for EPA.
pub fn gjk_query(a: &impl SupportMap, b: &impl SupportMap, max_iterations: usize) -> GjkResult {
    let center_a = a.aabb().center();
    let center_b = b.aabb().center();
    let dir_vec = center_b - center_a;
    let dir_norm = dir_vec.norm();

    let mut direction = if dir_norm > EPSILON {
        dir_vec / dir_norm
    } else {
        Vector3::x()
    };

    let mut simplex = Simplex::new();

    let first = support_minkowski(a, b, &direction);
    simplex.push(first);
    direction = -first.point.coords;

    for iteration in 0..max_iterations {
        if direction.norm_squared() < EPSILON * EPSILON {
            return GjkResult {
                intersecting: true,
                simplex,
            };
        }

        direction = direction.normalize();

        let new_point = support_minkowski(a, b, &direction);

        if new_point.point.coords.dot(&direction) < -EPSILON {
            return GjkResult {
                intersecting: false,
                simplex,
            };
        }

        simplex.push(new_point);

        if do_simplex(&mut simplex, &mut direction) {
            return GjkResult {
                intersecting: true,
                simplex,
            };
        }

        // Guard against infinite loops with degenerate simplices
        if iteration == max_iterations - 1 {
            break;
        }
    }

    GjkResult {
        intersecting: false,
        simplex,
    }
}

/// Compute Minkowski difference support point.
pub fn support_minkowski(
    a: &impl SupportMap,
    b: &impl SupportMap,
    direction: &Vector3<f64>,
) -> MinkowskiPoint {
    let support_a = a.support(direction);
    let support_b = b.support(&-direction);
    MinkowskiPoint::new(support_a, support_b)
}

// =============================================================================
// Simplex processing
// =============================================================================

fn do_simplex(simplex: &mut Simplex, direction: &mut Vector3<f64>) -> bool {
    match simplex.len() {
        2 => do_simplex_line(simplex, direction),
        3 => do_simplex_triangle(simplex, direction),
        4 => do_simplex_tetrahedron(simplex, direction),
        _ => false,
    }
}

fn do_simplex_line(simplex: &mut Simplex, direction: &mut Vector3<f64>) -> bool {
    let a = simplex.points[0].point;
    let b = simplex.points[1].point;

    let ab = b - a;
    let ao = -a.coords;

    if ab.dot(&ao) > 0.0 {
        *direction = triple_product(&ab, &ao, &ab);
    } else {
        simplex.set(&[simplex.points[0]]);
        *direction = ao;
    }

    false
}

fn do_simplex_triangle(simplex: &mut Simplex, direction: &mut Vector3<f64>) -> bool {
    let a = simplex.points[0].point;
    let b = simplex.points[1].point;
    let c = simplex.points[2].point;

    let ab = b - a;
    let ac = c - a;
    let ao = -a.coords;

    let abc = ab.cross(&ac);

    if abc.cross(&ac).dot(&ao) > 0.0 {
        if ac.dot(&ao) > 0.0 {
            simplex.set(&[simplex.points[0], simplex.points[2]]);
            *direction = triple_product(&ac, &ao, &ac);
        } else {
            return simplex_line_case(simplex, direction, &ab, &ao);
        }
    } else if ab.cross(&abc).dot(&ao) > 0.0 {
        return simplex_line_case(simplex, direction, &ab, &ao);
    } else if abc.dot(&ao) > 0.0 {
        *direction = abc;
    } else {
        simplex.set(&[simplex.points[0], simplex.points[2], simplex.points[1]]);
        *direction = -abc;
    }

    false
}

fn simplex_line_case(
    simplex: &mut Simplex,
    direction: &mut Vector3<f64>,
    ab: &Vector3<f64>,
    ao: &Vector3<f64>,
) -> bool {
    if ab.dot(ao) > 0.0 {
        simplex.set(&[simplex.points[0], simplex.points[1]]);
        *direction = triple_product(ab, ao, ab);
    } else {
        simplex.set(&[simplex.points[0]]);
        *direction = *ao;
    }
    false
}

fn do_simplex_tetrahedron(simplex: &mut Simplex, direction: &mut Vector3<f64>) -> bool {
    let a = simplex.points[0].point;
    let b = simplex.points[1].point;
    let c = simplex.points[2].point;
    let d = simplex.points[3].point;

    let ab = b - a;
    let ac = c - a;
    let ad = d - a;
    let ao = -a.coords;

    let abc = ab.cross(&ac);
    let acd = ac.cross(&ad);
    let adb = ad.cross(&ab);

    if abc.dot(&ao) > 0.0 {
        simplex.set(&[simplex.points[0], simplex.points[1], simplex.points[2]]);
        return do_simplex_triangle(simplex, direction);
    }

    if acd.dot(&ao) > 0.0 {
        simplex.set(&[simplex.points[0], simplex.points[2], simplex.points[3]]);
        return do_simplex_triangle(simplex, direction);
    }

    if adb.dot(&ao) > 0.0 {
        simplex.set(&[simplex.points[0], simplex.points[3], simplex.points[1]]);
        return do_simplex_triangle(simplex, direction);
    }

    true
}

/// Triple product: (A × B) × C = B(A·C) - A(B·C).
#[inline]
fn triple_product(a: &Vector3<f64>, b: &Vector3<f64>, c: &Vector3<f64>) -> Vector3<f64> {
    b * a.dot(c) - a * b.dot(c)
}

// =============================================================================
// Distance query helpers
// =============================================================================

/// Closest point on simplex to origin, with barycentric coordinates.
fn closest_point_on_simplex_to_origin(simplex: &Simplex) -> (Vector3<f64>, Vec<(usize, f64)>) {
    match simplex.len() {
        2 => {
            let a = simplex.points[0].point.coords;
            let b = simplex.points[1].point.coords;
            let ab = b - a;
            let ao = -a;
            let ab_sq = ab.norm_squared();
            if ab_sq < EPSILON * EPSILON {
                return (a, vec![(0, 1.0)]);
            }
            let t = (ao.dot(&ab) / ab_sq).clamp(0.0, 1.0);
            let closest = a + t * ab;
            (closest, vec![(0, 1.0 - t), (1, t)])
        }
        3 => closest_point_on_triangle_to_origin(simplex),
        // Size 1 or 4+: closest point is the first point
        _ => {
            let p = simplex.points[0].point.coords;
            (p, vec![(0, 1.0)])
        }
    }
}

/// Full Voronoi region analysis for triangle case.
#[allow(clippy::many_single_char_names, clippy::suboptimal_flops)]
fn closest_point_on_triangle_to_origin(simplex: &Simplex) -> (Vector3<f64>, Vec<(usize, f64)>) {
    let a = simplex.points[0].point.coords;
    let b = simplex.points[1].point.coords;
    let c = simplex.points[2].point.coords;

    let ab = b - a;
    let ac = c - a;
    let ao = -a;

    let d1 = ab.dot(&ao);
    let d2 = ac.dot(&ao);

    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, vec![(0, 1.0)]);
    }

    let bo = -b;
    let d3 = ab.dot(&bo);
    let d4 = ac.dot(&bo);

    if d3 >= 0.0 && d4 <= d3 {
        return (b, vec![(1, 1.0)]);
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        let closest = a + v * ab;
        return (closest, vec![(0, 1.0 - v), (1, v)]);
    }

    let co = -c;
    let d5 = ab.dot(&co);
    let d6 = ac.dot(&co);

    if d6 >= 0.0 && d5 <= d6 {
        return (c, vec![(2, 1.0)]);
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        let closest = a + w * ac;
        return (closest, vec![(0, 1.0 - w), (2, w)]);
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let closest = b + w * (c - b);
        return (closest, vec![(1, 1.0 - w), (2, w)]);
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    let closest = a + v * ab + w * ac;
    (closest, vec![(0, 1.0 - v - w), (1, v), (2, w)])
}

/// Recover world-space witness points from barycentric coordinates.
fn recover_witness_points(simplex: &Simplex, bary: &[(usize, f64)]) -> (Point3<f64>, Point3<f64>) {
    let mut wa = Vector3::zeros();
    let mut wb = Vector3::zeros();
    for &(idx, weight) in bary {
        wa += simplex.points[idx].support_a.coords * weight;
        wb += simplex.points[idx].support_b.coords * weight;
    }
    (Point3::from(wa), Point3::from(wb))
}

/// Reduce simplex to only contributing vertices.
fn reduce_simplex_to_closest(simplex: &mut Simplex, bary: &[(usize, f64)]) {
    let active: Vec<MinkowskiPoint> = bary
        .iter()
        .filter(|(_, w)| *w > EPSILON)
        .map(|&(idx, _)| simplex.points[idx])
        .collect();
    simplex.set(&active);
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::support_map::{
        SupportBox, SupportCapsule, SupportCylinder, SupportEllipsoid, SupportSphere,
    };
    use approx::assert_relative_eq;

    // Helper: offset a support map by translating its support point.
    // This simulates placing shapes at different positions.
    struct Translated<'a, S: SupportMap> {
        inner: &'a S,
        offset: Vector3<f64>,
    }

    impl<S: SupportMap> crate::Bounded for Translated<'_, S> {
        fn aabb(&self) -> crate::Aabb {
            let inner_aabb = self.inner.aabb();
            crate::Aabb::new(
                Point3::from(inner_aabb.min.coords + self.offset),
                Point3::from(inner_aabb.max.coords + self.offset),
            )
        }
    }

    impl<S: SupportMap> SupportMap for Translated<'_, S> {
        fn support(&self, direction: &Vector3<f64>) -> Point3<f64> {
            Point3::from(self.inner.support(direction).coords + self.offset)
        }
    }

    fn at<S: SupportMap>(shape: &S, x: f64, y: f64, z: f64) -> Translated<'_, S> {
        Translated {
            inner: shape,
            offset: Vector3::new(x, y, z),
        }
    }

    #[test]
    fn separated_spheres_distance() {
        let a = SupportSphere { radius: 1.0 };
        let b = SupportSphere { radius: 1.0 };

        let result = gjk_distance(&at(&a, 0.0, 0.0, 0.0), &at(&b, 4.0, 0.0, 0.0), 35, 1e-6);
        assert!(result.is_some());

        let r = result.unwrap();
        // Centers 4.0 apart, radii sum 2.0 → distance 2.0
        assert_relative_eq!(r.distance, 2.0, epsilon = 1e-4);
        assert_relative_eq!(r.point_a.x, 1.0, epsilon = 1e-3);
        assert_relative_eq!(r.point_b.x, 3.0, epsilon = 1e-3);
    }

    #[test]
    fn overlapping_spheres_return_none() {
        let a = SupportSphere { radius: 1.0 };
        let b = SupportSphere { radius: 1.0 };

        let result = gjk_distance(&at(&a, 0.0, 0.0, 0.0), &at(&b, 1.0, 0.0, 0.0), 35, 1e-6);
        assert!(result.is_none());
    }

    #[test]
    fn separated_boxes_distance() {
        let a = SupportBox {
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let b = SupportBox {
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };

        let result = gjk_distance(&at(&a, 0.0, 0.0, 0.0), &at(&b, 3.0, 0.0, 0.0), 35, 1e-6);
        assert!(result.is_some());

        let r = result.unwrap();
        // Separation = 3.0 - 0.5 - 0.5 = 2.0
        assert_relative_eq!(r.distance, 2.0, epsilon = 1e-4);
    }

    #[test]
    fn intersection_spheres() {
        let a = SupportSphere { radius: 1.0 };
        let b = SupportSphere { radius: 1.0 };

        assert!(gjk_intersection(
            &at(&a, 0.0, 0.0, 0.0),
            &at(&b, 1.5, 0.0, 0.0)
        ));
    }

    #[test]
    fn no_intersection_spheres() {
        let a = SupportSphere { radius: 1.0 };
        let b = SupportSphere { radius: 1.0 };

        assert!(!gjk_intersection(
            &at(&a, 0.0, 0.0, 0.0),
            &at(&b, 3.0, 0.0, 0.0)
        ));
    }

    #[test]
    fn intersection_sphere_box() {
        let a = SupportSphere { radius: 1.0 };
        let b = SupportBox {
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };

        assert!(gjk_intersection(
            &at(&a, 0.0, 0.0, 0.0),
            &at(&b, 1.0, 0.0, 0.0)
        ));
    }

    #[test]
    fn separated_capsules() {
        let a = SupportCapsule {
            half_length: 1.0,
            radius: 0.5,
        };
        let b = SupportCapsule {
            half_length: 1.0,
            radius: 0.5,
        };

        let result = gjk_distance(&at(&a, 0.0, 0.0, 0.0), &at(&b, 3.0, 0.0, 0.0), 35, 1e-6);
        assert!(result.is_some());

        let r = result.unwrap();
        // Separation = 3.0 - 0.5 - 0.5 = 2.0
        assert_relative_eq!(r.distance, 2.0, epsilon = 1e-3);
    }

    #[test]
    fn intersection_cylinders() {
        let a = SupportCylinder {
            half_length: 1.0,
            radius: 0.5,
        };
        let b = SupportCylinder {
            half_length: 1.0,
            radius: 0.5,
        };

        assert!(gjk_intersection(
            &at(&a, 0.0, 0.0, 0.0),
            &at(&b, 0.8, 0.0, 0.0)
        ));
    }

    #[test]
    fn separated_ellipsoids() {
        let a = SupportEllipsoid {
            radii: Vector3::new(2.0, 1.0, 0.5),
        };
        let b = SupportSphere { radius: 0.5 };

        let result = gjk_distance(&at(&a, 0.0, 0.0, 0.0), &at(&b, 4.0, 0.0, 0.0), 35, 1e-6);
        assert!(result.is_some());

        let r = result.unwrap();
        // Separation = 4.0 - 2.0 - 0.5 = 1.5
        assert_relative_eq!(r.distance, 1.5, epsilon = 1e-3);
    }

    #[test]
    fn convex_hull_intersection() {
        let vertices = vec![
            Point3::new(-0.5, -0.5, -0.5),
            Point3::new(0.5, -0.5, -0.5),
            Point3::new(0.5, 0.5, -0.5),
            Point3::new(-0.5, 0.5, -0.5),
            Point3::new(-0.5, -0.5, 0.5),
            Point3::new(0.5, -0.5, 0.5),
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(-0.5, 0.5, 0.5),
        ];
        let hull = crate::convex_hull(&vertices, None).unwrap();
        let sphere = SupportSphere { radius: 0.5 };

        assert!(gjk_intersection(
            &at(&hull, 0.0, 0.0, 0.0),
            &at(&sphere, 0.8, 0.0, 0.0)
        ));
    }

    #[test]
    fn just_touching_spheres() {
        let a = SupportSphere { radius: 1.0 };
        let b = SupportSphere { radius: 1.0 };

        let result = gjk_distance(&at(&a, 0.0, 0.0, 0.0), &at(&b, 2.0, 0.0, 0.0), 35, 1e-6);
        // At boundary — may return Some(~0) or None
        if let Some(r) = result {
            assert!(
                r.distance < 1e-3,
                "distance should be near zero, got {}",
                r.distance
            );
        }
    }

    #[test]
    fn coincident_centers() {
        let a = SupportSphere { radius: 1.0 };
        let b = SupportSphere { radius: 1.0 };

        // Same position — must overlap
        let result = gjk_distance(&at(&a, 0.0, 0.0, 0.0), &at(&b, 0.0, 0.0, 0.0), 35, 1e-6);
        assert!(result.is_none());
    }
}
