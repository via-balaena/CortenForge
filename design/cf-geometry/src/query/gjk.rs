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

/// MuJoCo's `mjMINVAL`.
const MIN_VAL: f64 = 1e-15;

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
/// and witness points (closest points on each shape). Returns `None` where
/// the iteration ends on a tetrahedron about the origin (an overlap; use
/// [`epa_penetration`](super::epa_penetration)), where `x` comes within
/// 1e-15 of the origin, or where every weight of the simplex comes out 0
/// (MuJoCo's "should not occur"). A `NaN` coordinate can give a `NaN`
/// distance.
///
/// The iteration is MuJoCo's GJK asked for a distance (`gjk`,
/// `engine_collision_gjk.c:171-296`), but for one weight: in `S3D`'s fourth
/// face MuJoCo leaves the fourth point's weight as an earlier face set it
/// (`:633-643`) and so can read a separated pair as an overlap, and this sets
/// it to 0 (registry row `D-GJK-S3D`). From the difference of the shapes'
/// centres ([`SupportMap::center`]), each step takes the Minkowski difference's
/// support point `s` along `−x / |x|` and stops once the Frank–Wolfe gap `x ·
/// (x − s)` is below `tolerance² / 2`; else `x` becomes the point nearest the
/// origin of the simplex with `s` added (MuJoCo's `subdistance`), the simplex
/// keeps the points that point weighs, and the iteration stops where `x` no
/// longer moves, or with an overlap where the simplex is a tetrahedron about
/// the origin. The distance is `|x|` as the last step began, as MuJoCo's.
/// MuJoCo takes the tolerance as 0 for two boxes, meshes or height fields
/// without a margin (`discreteGeoms`, `:159-167`); the shapes here carry no
/// type, so a caller passes 0 for them.
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
    let epsilon = 0.5 * tolerance * tolerance;
    let (center_a, center_b) = (a.center(), b.center());
    let mut x = center_a - center_b;
    let mut x_norm = x.norm();
    let mut simplex = [MinkowskiPoint::default(); 4];
    let mut lambda = [1.0, 0.0, 0.0, 0.0];
    let mut n = 0;

    for k in 0..max_iterations {
        let x_norm_sq = x.norm_squared();
        if x_norm_sq < MIN_VAL * MIN_VAL {
            return None; // x is the origin: the shapes touch
        }
        x_norm = x_norm_sq.sqrt();
        // `gjkSupport` (`:335`) scales x by the reciprocal of its norm.
        let support = support_minkowski(a, b, &-(x * (1.0 / x_norm)));

        // The Frank–Wolfe duality gap bounds how far |x| is above the distance.
        if x.dot(&(x - support.point.coords)) < epsilon {
            if k == 0 {
                // MuJoCo's `if (!k) n = 1`: the support point is the simplex,
                // its supports the witnesses, and the distance stays |x|.
                simplex[0] = support;
                n = 1;
            }
            break;
        }

        simplex[n] = support;
        lambda = subdistance(&simplex[..=n]);
        n = 0;
        for i in 0..4 {
            if lambda[i] != 0.0 {
                simplex[n] = simplex[i];
                lambda[n] = lambda[i];
                n += 1;
            }
        }
        if n == 0 {
            return None; // MuJoCo's "should not occur": every weight 0
        }

        let next = lincomb(&lambda[..n], simplex[..n].iter().map(|p| p.point.coords));
        if (next - x).iter().all(|d| d.abs() < MIN_VAL) {
            break;
        }
        x = next;
        if n == 4 {
            return None; // the origin is inside the tetrahedron: an overlap
        }
    }

    let (point_a, point_b) = if n == 0 {
        (center_a, center_b) // no iteration ran
    } else {
        (
            Point3::from(lincomb(
                &lambda[..n],
                simplex[..n].iter().map(|p| p.support_a.coords),
            )),
            Point3::from(lincomb(
                &lambda[..n],
                simplex[..n].iter().map(|p| p.support_b.coords),
            )),
        )
    };
    Some(GjkDistance {
        distance: x_norm,
        point_a,
        point_b,
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

/// MuJoCo's `subdistance` (`engine_collision_gjk.c:532-553`, after
/// Montanari et al., ACM Transactions on Graphics 2017): the barycentric
/// coordinates, over the simplex's one to four points, of its point nearest
/// the origin; a point the nearest point does not weigh takes exactly 0.
fn subdistance(simplex: &[MinkowskiPoint]) -> [f64; 4] {
    let s = |i: usize| simplex[i].point.coords;
    let mut lambda = [0.0; 4];
    match simplex.len() {
        4 => lambda = s3d(&s(0), &s(1), &s(2), &s(3)),
        3 => lambda[..3].copy_from_slice(&s2d(&s(0), &s(1), &s(2))),
        2 => lambda[..2].copy_from_slice(&s1d(&s(0), &s(1))),
        _ => lambda[0] = 1.0,
    }
    lambda
}

/// MuJoCo's `S3D` (`engine_collision_gjk.c:556-644`): a tetrahedron's
/// signed volumes; where the origin is outside, the nearest of the faces it
/// is outside.
#[allow(clippy::similar_names, reason = "MuJoCo's cofactor names C41..C44")]
fn s3d(s1: &Vector3<f64>, s2: &Vector3<f64>, s3: &Vector3<f64>, s4: &Vector3<f64>) -> [f64; 4] {
    let c41 = -det3(s2, s3, s4);
    let c42 = det3(s1, s3, s4);
    let c43 = -det3(s1, s2, s4);
    let c44 = det3(s1, s2, s3);
    let m_det = c41 + c42 + c43 + c44;
    let comp = [
        same_sign(m_det, c41),
        same_sign(m_det, c42),
        same_sign(m_det, c43),
        same_sign(m_det, c44),
    ];
    if comp.iter().all(|&inside| inside) {
        return [c41 / m_det, c42 / m_det, c43 / m_det, c44 / m_det];
    }

    // The faces opposite each point, and where each face's weights go.
    let faces: [([&Vector3<f64>; 3], [usize; 3]); 4] = [
        ([s2, s3, s4], [1, 2, 3]),
        ([s1, s3, s4], [0, 2, 3]),
        ([s1, s2, s4], [0, 1, 3]),
        ([s1, s2, s3], [0, 1, 2]),
    ];
    let mut lambda = [0.0; 4];
    let mut dmin = f64::MAX;
    for (i, ([p1, p2, p3], slots)) in faces.iter().enumerate() {
        if comp[i] {
            continue;
        }
        let face = s2d(p1, p2, p3);
        let d = lincomb(&face, [**p1, **p2, **p3]).norm_squared();
        // MuJoCo takes the first face whatever its distance. Its fourth face
        // leaves the fourth point's weight as an earlier face set it
        // (`:633-643`), so the weights sum past 1, the simplex keeps four
        // points and the loop reads the pair as an overlap; ours sets it to 0
        // (registry `D-GJK-S3D`).
        if i == 0 || d < dmin {
            lambda[i] = 0.0;
            for (slot, w) in slots.iter().zip(face) {
                lambda[*slot] = w;
            }
            dmin = d;
        }
    }
    lambda
}

/// MuJoCo's `S2D` (`engine_collision_gjk.c:647-777`): a triangle's signed
/// areas in the plane it shadows most; where the origin's projection is
/// outside, the nearest of the edges it is outside.
#[allow(
    clippy::similar_names,
    clippy::suboptimal_flops,
    reason = "MuJoCo's minor and cofactor names, and its sums as written"
)]
fn s2d(s1: &Vector3<f64>, s2: &Vector3<f64>, s3: &Vector3<f64>) -> [f64; 3] {
    let Some(p_o) = project_origin_plane(s1, s2, s3) else {
        let [l1, l2] = s1d(s1, s2);
        return [l1, l2, 0.0];
    };

    let m_14 = s2[1] * s3[2] - s2[2] * s3[1] - s1[1] * s3[2] + s1[2] * s3[1] + s1[1] * s2[2]
        - s1[2] * s2[1];
    let m_24 = s2[0] * s3[2] - s2[2] * s3[0] - s1[0] * s3[2] + s1[2] * s3[0] + s1[0] * s2[2]
        - s1[2] * s2[0];
    let m_34 = s2[0] * s3[1] - s2[1] * s3[0] - s1[0] * s3[1] + s1[1] * s3[0] + s1[0] * s2[1]
        - s1[1] * s2[0];

    // Leave out the axis along which the triangle's shadow is largest.
    let (mu1, mu2, mu3) = (m_14.abs(), m_24.abs(), m_34.abs());
    let (m_max, [ax0, ax1]) = if mu1 >= mu2 && mu1 >= mu3 {
        (m_14, [1, 2])
    } else if mu2 >= mu3 {
        (m_24, [0, 2])
    } else {
        (m_34, [0, 1])
    };
    let flat = |q: &Vector3<f64>| [q[ax0], q[ax1]];
    let (po, f1, f2, f3) = (flat(&p_o), flat(s1), flat(s2), flat(s3));

    // The signed areas of (p, s2, s3), (p, s1, s3) and (p, s1, s2).
    let c31 = po[0] * f2[1] + po[1] * f3[0] + f2[0] * f3[1]
        - po[0] * f3[1]
        - po[1] * f2[0]
        - f3[0] * f2[1];
    let c32 = po[0] * f3[1] + po[1] * f1[0] + f3[0] * f1[1]
        - po[0] * f1[1]
        - po[1] * f3[0]
        - f1[0] * f3[1];
    let c33 = po[0] * f1[1] + po[1] * f2[0] + f1[0] * f2[1]
        - po[0] * f2[1]
        - po[1] * f1[0]
        - f2[0] * f1[1];
    let comp = [
        same_sign(m_max, c31),
        same_sign(m_max, c32),
        same_sign(m_max, c33),
    ];
    if comp.iter().all(|&inside| inside) {
        return [c31 / m_max, c32 / m_max, c33 / m_max];
    }

    // The edges opposite each point, and where each edge's weights go.
    let edges: [([&Vector3<f64>; 2], [usize; 2]); 3] =
        [([s2, s3], [1, 2]), ([s1, s3], [0, 2]), ([s1, s2], [0, 1])];
    let mut lambda = [0.0; 3];
    let mut dmin = f64::MAX;
    for (i, ([p1, p2], slots)) in edges.iter().enumerate() {
        if comp[i] {
            continue;
        }
        let edge = s1d(p1, p2);
        let d = lincomb(&edge, [**p1, **p2]).norm_squared();
        // MuJoCo takes the first edge whatever its distance.
        if i == 0 || d < dmin {
            lambda[i] = 0.0;
            for (slot, w) in slots.iter().zip(edge) {
                lambda[*slot] = w;
            }
            dmin = d;
        }
    }
    lambda
}

/// MuJoCo's `S1D` (`engine_collision_gjk.c:780-810`): a segment's weights of
/// the origin's projection, measured along the axis the segment shadows
/// most; `[0, 1]`, the second point, where the projection is outside.
fn s1d(s1: &Vector3<f64>, s2: &Vector3<f64>) -> [f64; 2] {
    let p_o = project_origin_line(s1, s2);
    let mut mu_max = s1[0] - s2[0];
    let mut index = 0;
    for axis in 1..3 {
        let mu = s1[axis] - s2[axis];
        if mu.abs() >= mu_max.abs() {
            mu_max = mu;
            index = axis;
        }
    }
    let c1 = p_o[index] - s2[index];
    let c2 = s1[index] - p_o[index];
    if same_sign(mu_max, c1) && same_sign(mu_max, c2) {
        [c1 / mu_max, c2 / mu_max]
    } else {
        [0.0, 1.0]
    }
}

/// MuJoCo's `projectOriginPlane` (`engine_collision_gjk.c:473-507`): the
/// origin projected onto the plane through three points, its normal taken
/// from each pair of edges in turn; `None` where an edge pair's cross product
/// is exactly zero before a usable one is found.
#[allow(clippy::float_cmp, reason = "MuJoCo's exact tests against 0")]
fn project_origin_plane(
    v1: &Vector3<f64>,
    v2: &Vector3<f64>,
    v3: &Vector3<f64>,
) -> Option<Vector3<f64>> {
    let diff21 = v2 - v1;
    let diff31 = v3 - v1;
    let diff32 = v3 - v2;
    for (n, v) in [(diff32.cross(&diff21), v2), (diff21.cross(&diff31), v1)] {
        let nv = n.dot(v);
        let nn = n.dot(&n);
        if nn == 0.0 {
            return None;
        }
        if nv != 0.0 && nn > MIN_VAL {
            return Some(n * (nv / nn));
        }
    }
    let n = diff31.cross(&diff32);
    Some(n * (n.dot(v3) / n.dot(&n)))
}

/// MuJoCo's `projectOriginLine` (`engine_collision_gjk.c:510-520`): the
/// origin projected onto the line through two points.
fn project_origin_line(v1: &Vector3<f64>, v2: &Vector3<f64>) -> Vector3<f64> {
    let diff = v2 - v1;
    let scl = -(v2.dot(&diff) / diff.dot(&diff));
    v2 + diff * scl
}

/// MuJoCo's `sameSign2`, as a test: both positive or both negative.
fn same_sign(a: f64, b: f64) -> bool {
    (a > 0.0 && b > 0.0) || (a < 0.0 && b < 0.0)
}

/// MuJoCo's `det3`: `v1 · (v2 × v3)`, its terms as written.
#[allow(clippy::suboptimal_flops, reason = "MuJoCo's sum as written")]
fn det3(v1: &Vector3<f64>, v2: &Vector3<f64>, v3: &Vector3<f64>) -> f64 {
    v1[0] * (v2[1] * v3[2] - v2[2] * v3[1])
        + v1[1] * (v2[2] * v3[0] - v2[0] * v3[2])
        + v1[2] * (v2[0] * v3[1] - v2[1] * v3[0])
}

/// MuJoCo's `lincomb`: `Σ coef[i] · v[i]`, summed from the first.
fn lincomb(coef: &[f64], points: impl IntoIterator<Item = Vector3<f64>>) -> Vector3<f64> {
    let mut terms = coef.iter().zip(points).map(|(c, p)| p * *c);
    let first = terms.next().unwrap_or_else(Vector3::zeros);
    terms.fold(first, |sum, term| sum + term)
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
    use nalgebra::{Matrix3, Rotation3};

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

        fn center(&self) -> Point3<f64> {
            Point3::from(self.inner.center().coords + self.offset)
        }
    }

    // A support map turned by `rot`, then moved by `offset`; its box from its
    // supports along each axis.
    struct Turned<'a, S: SupportMap> {
        inner: &'a S,
        rot: Rotation3<f64>,
        offset: Vector3<f64>,
    }

    impl<S: SupportMap> crate::Bounded for Turned<'_, S> {
        fn aabb(&self) -> crate::Aabb {
            let (mut lo, mut hi) = (Vector3::zeros(), Vector3::zeros());
            for k in 0..3 {
                let axis = Vector3::ith(k, 1.0);
                hi[k] = self.support(&axis)[k];
                lo[k] = self.support(&-axis)[k];
            }
            crate::Aabb::new(Point3::from(lo), Point3::from(hi))
        }
    }

    impl<S: SupportMap> SupportMap for Turned<'_, S> {
        fn support(&self, direction: &Vector3<f64>) -> Point3<f64> {
            let local = self.inner.support(&(self.rot.inverse() * direction));
            Point3::from(self.rot * local.coords + self.offset)
        }

        fn center(&self) -> Point3<f64> {
            Point3::from(self.rot * self.inner.center().coords + self.offset)
        }
    }

    // A support map whose centre is given, not its bounding box's.
    struct Centred<'a, S: SupportMap> {
        inner: &'a S,
        centre: Point3<f64>,
    }

    impl<S: SupportMap> crate::Bounded for Centred<'_, S> {
        fn aabb(&self) -> crate::Aabb {
            self.inner.aabb()
        }
    }

    impl<S: SupportMap> SupportMap for Centred<'_, S> {
        fn support(&self, direction: &Vector3<f64>) -> Point3<f64> {
            self.inner.support(direction)
        }

        fn center(&self) -> Point3<f64> {
            self.centre
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

    /// An ellipsoid (radii 0.05, 0.04, 0.03) over a box (half-sizes 0.5, 0.5,
    /// 0.1), its lowest point `gap` above the box's top face, off the face's
    /// centre: the distance is the gap within 1e-9, its witnesses on the
    /// face and at the ellipsoid's lowest point. The loop this one replaced
    /// returned 0.8148 for a gap of 0.001.
    #[test]
    fn gjk_distance_box_ellipsoid_near_touching() {
        let slab = SupportBox {
            half_extents: Vector3::new(0.5, 0.5, 0.1),
        };
        let ellipsoid = SupportEllipsoid {
            radii: Vector3::new(0.05, 0.04, 0.03),
        };
        for gap in [0.001, 0.0005, 0.005, 0.009] {
            let z = 0.1 + 0.03 + gap;
            let result = gjk_distance(
                &at(&slab, 0.0, 0.0, 0.0),
                &at(&ellipsoid, 0.1, -0.05, z),
                35,
                1e-6,
            );
            assert!(result.is_some(), "gap {gap}: no distance");
            let result = result.unwrap();
            assert!(
                (result.distance - gap).abs() <= 1e-9,
                "gap {gap}: distance {}",
                result.distance
            );
            assert!(
                (result.point_a.z - 0.1).abs() <= 1e-9,
                "gap {gap}: on the face, {:?}",
                result.point_a
            );
            assert!(
                (result.point_b - Point3::new(0.1, -0.05, z - 0.03)).norm() <= 1e-4,
                "gap {gap}: at the ellipsoid's lowest point, {:?}",
                result.point_b
            );
        }
    }

    /// The iteration starts from the shapes' centres (`SupportMap::center`),
    /// not their bounding boxes': with no step to take, a sphere whose centre
    /// is given 0.25 along x and a sphere at 3 along x are 2.75 apart, the
    /// witnesses the two centres.
    #[test]
    fn gjk_distance_starts_from_the_shapes_centres() {
        let sphere = SupportSphere { radius: 0.5 };
        let a = Centred {
            inner: &sphere,
            centre: Point3::new(0.25, 0.0, 0.0),
        };
        let result = gjk_distance(&a, &at(&sphere, 3.0, 0.0, 0.0), 0, 1e-6);
        assert!(result.is_some(), "no distance");
        let result = result.unwrap();
        assert_eq!(result.distance, 2.75);
        assert_eq!(result.point_a, Point3::new(0.25, 0.0, 0.0));
        assert_eq!(result.point_b, Point3::new(3.0, 0.0, 0.0));
    }

    /// A segment (a capsule of radius 0, half-length 1, along z) and a point
    /// 3 away along x, across the segment's middle: the first support point
    /// along −x is the segment's top end, already within the Frank–Wolfe gap
    /// of the box centres' difference, so the loop stops at its first step.
    /// The distance is that difference's, 3, and the witnesses the support
    /// point's, the top end, as MuJoCo's (`if (!k) n = 1`).
    #[test]
    fn gjk_distance_stopping_at_the_first_step_keeps_the_centres_distance() {
        let segment = SupportCapsule {
            half_length: 1.0,
            radius: 0.0,
        };
        let point = SupportSphere { radius: 0.0 };
        let result = gjk_distance(
            &at(&segment, 0.0, 0.0, 0.0),
            &at(&point, 3.0, 0.0, 0.0),
            35,
            1e-6,
        );
        assert!(result.is_some(), "no distance");
        let result = result.unwrap();
        assert_eq!(result.distance, 3.0);
        assert_eq!(result.point_a, Point3::new(0.0, 0.0, 1.0));
        assert_eq!(result.point_b, Point3::new(3.0, 0.0, 0.0));
    }

    /// An ellipsoid (0.2, 0.15, 0.1) overlapping a box (0.5, 0.4, 0.3), its
    /// centre off each of the box's planes of symmetry: no distance.
    #[test]
    fn gjk_distance_of_an_overlap_in_three_dimensions_is_none() {
        let slab = SupportBox {
            half_extents: Vector3::new(0.5, 0.4, 0.3),
        };
        let ellipsoid = SupportEllipsoid {
            radii: Vector3::new(0.2, 0.15, 0.1),
        };
        let result = gjk_distance(
            &at(&slab, 0.0, 0.0, 0.0),
            &at(&ellipsoid, 0.13, -0.07, 0.31),
            35,
            1e-6,
        );
        assert!(result.is_none(), "{result:?}");
    }

    /// Unit spheres a nanometre apart: the distance, 1e-9 within 1e-15.
    /// MuJoCo's `gjk` stops on the origin only within 1e-15 (`mjMINVAL`,
    /// `engine_collision_gjk.c:194-196`); its caller `mjc_ccd` then takes a
    /// distance up to its tolerance as touching (`:2298`), which this does not.
    #[test]
    fn gjk_distance_reports_a_nanometre_gap() {
        let a = SupportSphere { radius: 1.0 };
        let b = SupportSphere { radius: 1.0 };
        let result = gjk_distance(
            &at(&a, 0.0, 0.0, 0.0),
            &at(&b, 2.0 + 1e-9, 0.0, 0.0),
            35,
            1e-6,
        );
        assert!(result.is_some(), "no distance");
        let distance = result.unwrap().distance;
        assert!((distance - 1e-9).abs() <= 1e-15, "distance {distance}");
    }

    /// Two boxes, each turned, 0.0402 apart (a random search's pair; the
    /// rotations are MuJoCo's `geom_xmat`), where MuJoCo's distance sensor
    /// reads 0: in `S3D`'s fourth face it keeps the weight an earlier face
    /// gave the fourth point (`engine_collision_gjk.c:633-643`), and with it
    /// kept this loop reads the pair as an overlap (measured). MuJoCo's own
    /// box–box function (`mjc_BoxBox`, native CCD disabled) reads
    /// 0.040212399850898445. Ours, the weight 0 and the tolerance 0 as
    /// MuJoCo's for two boxes: within 1e-12 of it (registry `D-GJK-S3D`).
    #[test]
    fn gjk_distance_of_boxes_mujocos_s3d_reads_as_touching() {
        let first = SupportBox {
            half_extents: Vector3::new(
                0.245_386_986_817_683_2,
                0.237_512_154_316_019_66,
                0.484_908_151_077_289_3,
            ),
        };
        let second = SupportBox {
            half_extents: Vector3::new(
                0.173_547_517_915_39,
                0.040_026_588_149_777_46,
                0.208_256_400_348_419_36,
            ),
        };
        let a = Turned {
            inner: &first,
            rot: Rotation3::from_matrix_unchecked(Matrix3::new(
                -0.062_732_013_780_880_61,
                0.749_904_151_704_075_1,
                0.658_565_454_380_949_6,
                -0.170_129_462_683_314_3,
                -0.658_242_153_021_689_2,
                0.733_330_235_236_797_9,
                0.983_422_930_571_624_2,
                -0.066_038_104_472_812_55,
                0.168_873_646_207_914_12,
            )),
            offset: Vector3::zeros(),
        };
        let b = Turned {
            inner: &second,
            rot: Rotation3::from_matrix_unchecked(Matrix3::new(
                0.312_895_849_063_831_5,
                0.429_042_914_882_963_4,
                -0.847_359_643_142_953_8,
                -0.388_020_336_156_218_34,
                0.872_053_700_132_505_8,
                0.298_265_926_338_260_3,
                0.866_911_994_592_207_2,
                0.235_466_603_309_121_95,
                0.439_339_358_990_547_55,
            )),
            offset: Vector3::new(
                -0.415_951_726_116_443_2,
                -0.501_729_772_562_780_1,
                0.081_432_428_137_459_25,
            ),
        };
        let result = gjk_distance(&a, &b, 35, 0.0);
        assert!(result.is_some(), "no distance");
        let distance = result.unwrap().distance;
        assert!(
            (distance - 0.040_212_399_850_898_445).abs() <= 1e-12,
            "distance {distance}"
        );
    }

    /// An ellipsoid and a box, each turned, 0.0686 apart (one of a random
    /// search's pairs; their rotations as it drew them): MuJoCo's distance
    /// sensor reads 0.0685786772243988 on the pose (the oracle), and ours
    /// within 1e-12.
    #[test]
    fn gjk_distance_of_a_turned_ellipsoid_and_box_as_mujocos() {
        let ellipsoid = SupportEllipsoid {
            radii: Vector3::new(
                0.129_446_800_682_902_49,
                0.312_324_856_413_955_74,
                0.263_060_756_458_427_1,
            ),
        };
        let slab = SupportBox {
            half_extents: Vector3::new(
                0.302_139_648_128_156_1,
                0.125_906_857_149_390_2,
                0.308_300_185_415_738_1,
            ),
        };
        let a = Turned {
            inner: &ellipsoid,
            rot: Rotation3::from_matrix_unchecked(Matrix3::from_column_slice(&[
                0.346_464_357_818_144_05,
                0.903_089_702_969_286_4,
                -0.253_754_679_075_100_96,
                0.773_923_104_517_057_4,
                -0.122_319_230_300_094_28,
                0.621_354_193_832_690_5,
                0.530_099_497_317_571,
                -0.411_663_690_759_385_3,
                -0.741_301_240_154_108_9,
            ])),
            offset: Vector3::zeros(),
        };
        let b = Turned {
            inner: &slab,
            rot: Rotation3::from_matrix_unchecked(Matrix3::from_column_slice(&[
                0.618_616_739_712_882,
                0.717_936_313_845_818_5,
                -0.319_187_685_552_69,
                -0.663_345_215_664_433_8,
                0.694_955_098_836_102_4,
                0.277_507_721_436_375_2,
                0.421_053_980_152_319_93,
                0.040_060_702_230_259_59,
                0.906_150_476_430_216_5,
            ])),
            offset: Vector3::new(
                0.058_982_747_720_683_695,
                -0.534_483_269_152_257,
                0.015_112_646_586_743_465,
            ),
        };
        let result = gjk_distance(&a, &b, 35, 1e-6);
        assert!(result.is_some(), "no distance");
        let distance = result.unwrap().distance;
        assert!(
            (distance - 0.068_578_677_224_398_8).abs() <= 1e-12,
            "distance {distance}"
        );
    }
}
