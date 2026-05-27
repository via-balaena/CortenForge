//! 2D silhouette polyline + point-to-silhouette signed distance.
//!
//! §F-1 + §F-2 of
//! `docs/CF_CAST_FLANGE_CONTINUITY_BOLT_PATTERN_RECON.md`. Replaces
//! [`crate::flange::FlangeSdf`]'s 3D-projected `body.evaluate` with a
//! 2D point-to-silhouette signed distance so the flange ring closes
//! around body concavities (heel pocket etc. on production
//! sock-over-capsule scans).
//!
//! # Why a separate 2D distance metric
//!
//! Per [[project-cf-cast-flange-perimeter-continuity-bookmark]] +
//! §F-S0 empirical probe ([`crate::flange::tests::s0_flange_continuity_probe`]):
//! the pre-fix bug was that `body.evaluate(P_projected_to_seam)`
//! returns the 3D distance from `P` to the nearest body surface,
//! which can collapse to the distance to a 3D wrap-around feature
//! (overhang, lid, shelf) at a small Y offset rather than the in-plane
//! distance to the silhouette curve. When that 3D distance drops
//! below `flange_inner_offset_m` (2 mm), `FlangeSdf`'s inner check
//! falsely declares the query point to be in the gasket-channel
//! margin and suppresses the flange material — producing the
//! workshop-visible "flange isn't all the way around" symptom.
//!
//! The 2D silhouette distance is the geometrically correct metric: it
//! measures distance to the body's seam-plane cross-section curve,
//! independent of any 3D structure off the seam plane.
//!
//! # Algorithm
//!
//! 1. Sample `body.evaluate` on a uniform 2D grid in (X, Z) at
//!    Y = `seam_plane_y`, step [`SILHOUETTE_GRID_STEP_M`] (0.5 mm).
//! 2. Marching squares (16-case table) emits zero-isoline line
//!    segments per cell. Saddle ambiguities (cases 5 + 10) resolved
//!    by sampling the cell center.
//! 3. [`Silhouette2d::signed_distance_to`] computes
//!    `±min_segment_distance` with sign from even/odd +X ray-cast.
//!
//! # Scope (§F-S1)
//!
//! Segments are stored as 2-vertex polylines without chaining into
//! ordered closed polylines. Point-to-segment distance + ray-cast
//! parity both work correctly on raw segment soup; ordered polylines
//! are deferred to §B (bolt-pattern arc-length placement).

// Grid + marching-squares code casts between usize grid indices and
// f64 coordinates throughout; indices are bounded by the grid
// dimensions (small thousands at most) so precision/truncation/sign
// loss are safe and intended. Match-same-arms is allowed because
// keeping all 16 standard MC cases enumerated mirrors textbook
// references and stays robust if a future change makes one branch
// diverge (e.g., orientation-aware polyline assembly for §B).
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::match_same_arms,
    clippy::similar_names
)]

use cf_design::Solid;
use nalgebra::Point3;

/// Marching-squares grid step (0.5 mm). Picked at recon §F-2: sub-mm
/// polyline accuracy + ~0.16 s precompute cost per layer at 200 ×
/// 200 mm grid.
pub const SILHOUETTE_GRID_STEP_M: f64 = 0.0005;

/// 2D point in the seam plane (X, Z coordinates; Y implicit).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    /// X coordinate.
    pub x: f64,
    /// Z coordinate.
    pub z: f64,
}

impl Point2 {
    /// Construct a 2D point from its X and Z coordinates.
    #[must_use]
    pub const fn new(x: f64, z: f64) -> Self {
        Self { x, z }
    }
}

/// Seam-plane silhouette of a body, stored as unordered line segments.
///
/// §F-S1 scope: each marching-squares cell contributes 0–2 segments;
/// ordered closed-polyline assembly is deferred to §B (bolt-pattern
/// arc-length placement needs ordering, but point-to-segment distance
/// + even/odd ray-cast both work correctly on raw segment soup).
#[derive(Debug, Clone)]
pub struct Silhouette2d {
    segments: Vec<[Point2; 2]>,
}

impl Silhouette2d {
    /// Build by sampling `body` SDF on a 2D grid at Y = `seam_plane_y`
    /// covering (X, Z) ∈ [`x_min`, `x_max`] × [`z_min`, `z_max`] with
    /// step [`SILHOUETTE_GRID_STEP_M`].
    ///
    /// The bounds should cover the body's seam-plane extent PLUS the
    /// flange's outward reach so the silhouette is captured everywhere
    /// the flange might lie. Callers pass the same expanded bounds
    /// used for [`Solid::from_sdf`]'s MC region.
    ///
    /// # Panics
    ///
    /// Panics if `x_max <= x_min` or `z_max <= z_min`.
    #[must_use]
    pub fn from_body_at_y(
        body: &Solid,
        seam_plane_y: f64,
        x_min: f64,
        x_max: f64,
        z_min: f64,
        z_max: f64,
    ) -> Self {
        assert!(
            x_max > x_min && z_max > z_min,
            "silhouette bounds must be positive: x [{x_min}, {x_max}] z [{z_min}, {z_max}]"
        );
        let step = SILHOUETTE_GRID_STEP_M;
        let nx = (((x_max - x_min) / step).ceil() as usize).max(1);
        let nz = (((z_max - z_min) / step).ceil() as usize).max(1);

        // Corner SDF grid: (nx + 1) × (nz + 1).
        let stride = nx + 1;
        let mut sdf = vec![0.0_f64; stride * (nz + 1)];
        for j in 0..=nz {
            let z = (j as f64).mul_add(step, z_min);
            for i in 0..=nx {
                let x = (i as f64).mul_add(step, x_min);
                sdf[j * stride + i] = body.evaluate(&Point3::new(x, seam_plane_y, z));
            }
        }

        // Marching squares per cell (i, j) with i ∈ [0, nx), j ∈ [0, nz).
        let mut segments: Vec<[Point2; 2]> = Vec::new();
        for j in 0..nz {
            for i in 0..nx {
                let s00 = sdf[j * stride + i]; // corner 0: bottom-left  (x0, z0)
                let s10 = sdf[j * stride + i + 1]; // corner 1: bottom-right (x1, z0)
                let s11 = sdf[(j + 1) * stride + i + 1]; // corner 2: top-right    (x1, z1)
                let s01 = sdf[(j + 1) * stride + i]; // corner 3: top-left     (x0, z1)
                let x0 = (i as f64).mul_add(step, x_min);
                let x1 = x0 + step;
                let z0 = (j as f64).mul_add(step, z_min);
                let z1 = z0 + step;
                let p00 = Point2::new(x0, z0);
                let p10 = Point2::new(x1, z0);
                let p11 = Point2::new(x1, z1);
                let p01 = Point2::new(x0, z1);

                // Inside = SDF < 0. Bit i set if corner i is inside.
                let mut case = 0u8;
                if s00 < 0.0 {
                    case |= 1;
                }
                if s10 < 0.0 {
                    case |= 2;
                }
                if s11 < 0.0 {
                    case |= 4;
                }
                if s01 < 0.0 {
                    case |= 8;
                }

                // Edge crossing accessors (lazy — only compute when used).
                let e_bottom = || interp(s00, s10, p00, p10);
                let e_right = || interp(s10, s11, p10, p11);
                let e_top = || interp(s01, s11, p01, p11);
                let e_left = || interp(s00, s01, p00, p01);

                match case {
                    0 | 15 => {}
                    1 => segments.push([e_left(), e_bottom()]),
                    2 => segments.push([e_bottom(), e_right()]),
                    3 => segments.push([e_left(), e_right()]),
                    4 => segments.push([e_right(), e_top()]),
                    5 => {
                        // Saddle (corners 0 + 2 inside): resolve via
                        // cell-center SDF — if center is inside, the
                        // two inside corners are connected through the
                        // center; emit segments that DON'T separate
                        // them. Else separate.
                        let center = body.evaluate(&Point3::new(
                            0.5_f64.mul_add(x1 - x0, x0),
                            seam_plane_y,
                            0.5_f64.mul_add(z1 - z0, z0),
                        ));
                        if center < 0.0 {
                            segments.push([e_left(), e_top()]);
                            segments.push([e_bottom(), e_right()]);
                        } else {
                            segments.push([e_left(), e_bottom()]);
                            segments.push([e_top(), e_right()]);
                        }
                    }
                    6 => segments.push([e_bottom(), e_top()]),
                    7 => segments.push([e_left(), e_top()]),
                    8 => segments.push([e_top(), e_left()]),
                    9 => segments.push([e_top(), e_bottom()]),
                    10 => {
                        // Saddle (corners 1 + 3 inside): mirror of case 5.
                        // - Center inside: 1+center+3 form one connected
                        //   inside region; cut off outside corners 0 + 2.
                        // - Center outside: 1 and 3 are isolated inside
                        //   islands; cut off each inside corner separately.
                        let center = body.evaluate(&Point3::new(
                            0.5_f64.mul_add(x1 - x0, x0),
                            seam_plane_y,
                            0.5_f64.mul_add(z1 - z0, z0),
                        ));
                        if center < 0.0 {
                            // Cut off outside corner 0 (left + bottom edges).
                            segments.push([e_left(), e_bottom()]);
                            // Cut off outside corner 2 (top + right edges).
                            segments.push([e_top(), e_right()]);
                        } else {
                            // Cut off inside corner 1 (bottom + right edges).
                            segments.push([e_bottom(), e_right()]);
                            // Cut off inside corner 3 (top + left edges).
                            segments.push([e_top(), e_left()]);
                        }
                    }
                    11 => segments.push([e_top(), e_right()]),
                    12 => segments.push([e_left(), e_right()]),
                    13 => segments.push([e_bottom(), e_right()]),
                    14 => segments.push([e_left(), e_bottom()]),
                    _ => unreachable!("case {case} out of 0..=15"),
                }
            }
        }

        Self { segments }
    }

    /// Signed distance from `(x, z)` to the silhouette curve.
    ///
    /// Negative = inside body (any silhouette polygon contains the
    /// query); positive = outside; magnitude is the Euclidean
    /// distance to the nearest segment.
    #[must_use]
    pub fn signed_distance_to(&self, x: f64, z: f64) -> f64 {
        let q = Point2::new(x, z);
        let unsigned = self
            .segments
            .iter()
            .map(|seg| point_to_segment_distance(q, seg[0], seg[1]))
            .fold(f64::INFINITY, f64::min);
        let crossings = self
            .segments
            .iter()
            .filter(|seg| ray_cast_crosses(q, seg[0], seg[1]))
            .count();
        if crossings % 2 == 1 {
            -unsigned
        } else {
            unsigned
        }
    }

    /// Number of segments in the silhouette (diagnostic).
    #[must_use]
    pub const fn segment_count(&self) -> usize {
        self.segments.len()
    }
}

/// Linear interpolation of the zero-crossing on an edge with corner
/// SDF values `a` (at `p_a`) and `b` (at `p_b`). Requires opposite
/// signs (caller checks via case index).
fn interp(a: f64, b: f64, p_a: Point2, p_b: Point2) -> Point2 {
    let t = a / (a - b);
    Point2::new(
        t.mul_add(p_b.x - p_a.x, p_a.x),
        t.mul_add(p_b.z - p_a.z, p_a.z),
    )
}

/// Euclidean distance from `q` to the line segment `a`→`b`.
fn point_to_segment_distance(q: Point2, a: Point2, b: Point2) -> f64 {
    let ab_x = b.x - a.x;
    let ab_z = b.z - a.z;
    let aq_x = q.x - a.x;
    let aq_z = q.z - a.z;
    let len2 = ab_x.mul_add(ab_x, ab_z * ab_z);
    let t = if len2 > 0.0 {
        (aq_x.mul_add(ab_x, aq_z * ab_z) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let proj_x = t.mul_add(ab_x, a.x);
    let proj_z = t.mul_add(ab_z, a.z);
    let dx = q.x - proj_x;
    let dz = q.z - proj_z;
    dx.mul_add(dx, dz * dz).sqrt()
}

/// Standard even/odd ray-cast: ray from `q` in +X direction crosses
/// segment `a`→`b` iff the segment straddles the horizontal line
/// `z = q.z` and the X-intersect is strictly greater than `q.x`.
fn ray_cast_crosses(q: Point2, a: Point2, b: Point2) -> bool {
    // Strict `>` on BOTH ends gives an effectively half-open convention:
    // an endpoint with `z == q.z` is classified "below" the ray, so two
    // chained segments sharing such a vertex contribute zero crossings
    // together (both straddle predicates evaluate equal and short-circuit).
    // Probability ≈ 0 for continuous coordinates from MC interpolation.
    let above_a = a.z > q.z;
    let above_b = b.z > q.z;
    if above_a == above_b {
        return false;
    }
    let t = (q.z - a.z) / (b.z - a.z);
    let x_intersect = t.mul_add(b.x - a.x, a.x);
    x_intersect > q.x
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::panic,
        clippy::expect_used,
        clippy::float_cmp
    )]

    use super::*;
    use nalgebra::{UnitQuaternion, Vector3};

    /// Cylinder along X, R=10 mm, length 60 mm. At Y=0 the silhouette
    /// is the rectangle X ∈ [-30, 30], Z ∈ [-10, 10].
    fn cylinder_along_x() -> Solid {
        Solid::cylinder(0.010, 0.030).rotate(UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            std::f64::consts::FRAC_PI_2,
        ))
    }

    #[test]
    fn cylinder_silhouette_signed_distance_matches_2d_analytical() {
        let body = cylinder_along_x();
        let sil = Silhouette2d::from_body_at_y(&body, 0.0, -0.050, 0.050, -0.025, 0.025);
        assert!(
            sil.segment_count() > 0,
            "marching squares must emit at least one segment for a non-trivial silhouette"
        );

        // Outside body, just past +Z edge (Z=10 mm); expect +5 mm.
        let d_out = sil.signed_distance_to(0.0, 0.015);
        assert!(
            (d_out - 0.005).abs() < SILHOUETTE_GRID_STEP_M,
            "expected ≈+5 mm outside +Z edge, got {d_out}"
        );

        // Inside body, at center; expect -10 mm.
        let d_in = sil.signed_distance_to(0.0, 0.0);
        assert!(
            (d_in - (-0.010)).abs() < SILHOUETTE_GRID_STEP_M,
            "expected ≈-10 mm at body center, got {d_in}"
        );

        // Outside body, well past +Z edge; expect +15 mm.
        let d_far = sil.signed_distance_to(0.0, 0.025);
        assert!(
            (d_far - 0.015).abs() < SILHOUETTE_GRID_STEP_M,
            "expected ≈+15 mm at Z=25 mm, got {d_far}"
        );

        // Outside body, well past +X cap (X=30 mm); expect +5 mm.
        let d_x = sil.signed_distance_to(0.035, 0.0);
        assert!(
            (d_x - 0.005).abs() < SILHOUETTE_GRID_STEP_M,
            "expected ≈+5 mm past +X cap, got {d_x}"
        );
    }

    #[test]
    fn c_shape_silhouette_distance_in_concavity_matches_2d() {
        // C-shape body matching the §F-S1 falsification fixture: outer
        // cuboid 80×30×40 mm with thin-Y notch on +Z face.
        let outer = Solid::cuboid(Vector3::new(0.040, 0.015, 0.020));
        let notch = Solid::cuboid(Vector3::new(0.020, 0.0015, 0.013))
            .translate(Vector3::new(0.0, 0.0, 0.010));
        let body = outer.subtract(notch);
        let sil = Silhouette2d::from_body_at_y(&body, 0.0, -0.060, 0.060, -0.040, 0.040);

        // Probe inside the C's mouth at (0, +10 mm). 2D distance to the
        // notch floor at (0, -3 mm) is 13 mm; expect ≈+13 mm.
        let d_concave = sil.signed_distance_to(0.0, 0.010);
        assert!(
            (d_concave - 0.013).abs() < 2.0 * SILHOUETTE_GRID_STEP_M,
            "expected ≈+13 mm at C's mouth probe (0, +10), got {d_concave}"
        );

        // Probe on a convex outer edge (X=0, Z=-25 mm), 5 mm past the
        // -Z outer face. Expect ≈+5 mm.
        let d_convex = sil.signed_distance_to(0.0, -0.025);
        assert!(
            (d_convex - 0.005).abs() < SILHOUETTE_GRID_STEP_M,
            "expected ≈+5 mm past -Z outer edge, got {d_convex}"
        );

        // Probe inside the body's solid spine (X=-30 mm, Z=0 mm), 10 mm
        // inboard of -X outer face. Expect ≈-10 mm.
        let d_inside = sil.signed_distance_to(-0.030, 0.0);
        assert!(
            (d_inside - (-0.010)).abs() < SILHOUETTE_GRID_STEP_M,
            "expected ≈-10 mm inside body spine, got {d_inside}"
        );
    }

    #[test]
    fn point_to_segment_distance_endpoint_and_perpendicular_cases() {
        let a = Point2::new(0.0, 0.0);
        let b = Point2::new(1.0, 0.0);
        // Perpendicular drop from above midpoint.
        assert!((point_to_segment_distance(Point2::new(0.5, 0.3), a, b) - 0.3).abs() < 1e-12);
        // Past endpoint b (clamp to b).
        assert!((point_to_segment_distance(Point2::new(2.0, 0.0), a, b) - 1.0).abs() < 1e-12);
        // Past endpoint a (clamp to a).
        assert!((point_to_segment_distance(Point2::new(-0.5, 0.0), a, b) - 0.5).abs() < 1e-12);
        // Degenerate (a == b) collapses to point distance.
        assert!((point_to_segment_distance(Point2::new(3.0, 4.0), a, a) - 5.0).abs() < 1e-12);
    }

    /// Saddle case 10 with center OUTSIDE (r < step/√2). Two disjoint
    /// spheres at the saddle cell's corner-1 and corner-3 positions;
    /// the cell center is outside both. Correct MC handling cuts off
    /// each inside corner separately (segments `[e_bottom, e_right]`
    /// and `[e_top, e_left]`), so the cell-center probe sees 0 crossings
    /// within the saddle cell + 1 from the cell to the right + 1 from
    /// the cell below = 2 crossings = even = OUTSIDE. If the saddle
    /// branches are swapped, the saddle cell contributes 1 crossing
    /// (segments cut across the X+ ray) → total 3 → odd → wrongly INSIDE.
    #[test]
    fn marching_squares_case_10_saddle_center_outside_classifies_outside() {
        let step = SILHOUETTE_GRID_STEP_M;
        let r = 0.6 * step;
        let body = Solid::sphere(r)
            .translate(Vector3::new(step, 0.0, 0.0))
            .union(Solid::sphere(r).translate(Vector3::new(0.0, 0.0, step)));
        let sil = Silhouette2d::from_body_at_y(&body, 0.0, -step, 2.0 * step, -step, 2.0 * step);
        let d = sil.signed_distance_to(0.5 * step, 0.5 * step);
        assert!(
            d > 0.0,
            "saddle-10 cell center must be classified OUTSIDE when \
             r < step/√2 (spheres don't reach center); got signed_distance \
             = {d} (negative = wrongly inside, indicates case-10 saddle \
             branches are swapped between center-inside and center-outside)"
        );
    }

    /// Saddle case 10 with center INSIDE (r > step/√2). Two overlapping
    /// spheres at the saddle cell's corner-1 and corner-3 positions;
    /// the cell center is inside the union. Correct MC handling cuts
    /// off the outside corners (segments `[e_left, e_bottom]` and
    /// `[e_top, e_right]`), so the saddle cell contributes 0 crossings
    /// to the +X cell-center probe. With contribution from the cell
    /// to the right of the saddle (case 1, segment crosses +X ray
    /// once), total = 1 crossing = odd = INSIDE. Swapped branches
    /// produce 2 crossings → wrongly OUTSIDE.
    #[test]
    fn marching_squares_case_10_saddle_center_inside_classifies_inside() {
        let step = SILHOUETTE_GRID_STEP_M;
        let r = 0.8 * step;
        let body = Solid::sphere(r)
            .translate(Vector3::new(step, 0.0, 0.0))
            .union(Solid::sphere(r).translate(Vector3::new(0.0, 0.0, step)));
        let sil = Silhouette2d::from_body_at_y(&body, 0.0, -step, 2.0 * step, -step, 2.0 * step);
        let d = sil.signed_distance_to(0.5 * step, 0.5 * step);
        assert!(
            d < 0.0,
            "saddle-10 cell center must be classified INSIDE when \
             r > step/√2 (overlapping spheres cover center); got \
             signed_distance = {d} (positive = wrongly outside, indicates \
             case-10 saddle branches are swapped between center-inside \
             and center-outside)"
        );
    }

    #[test]
    fn ray_cast_handles_strict_inequality_and_horizontal_segments() {
        let q = Point2::new(0.0, 0.0);
        // Segment fully above query → no crossing.
        assert!(!ray_cast_crosses(
            q,
            Point2::new(1.0, 1.0),
            Point2::new(2.0, 2.0)
        ));
        // Segment fully below → no crossing.
        assert!(!ray_cast_crosses(
            q,
            Point2::new(1.0, -1.0),
            Point2::new(2.0, -2.0)
        ));
        // Segment straddles, intersect at X=+1.5 > 0 → crossing.
        assert!(ray_cast_crosses(
            q,
            Point2::new(1.0, -1.0),
            Point2::new(2.0, 1.0)
        ));
        // Segment straddles, intersect at X=-0.5 < 0 → no crossing.
        assert!(!ray_cast_crosses(
            q,
            Point2::new(-1.0, -1.0),
            Point2::new(0.0, 1.0)
        ));
    }
}
