//! Wrapping geometry math — sphere and cylinder tangent/arc computations.
//!
//! Pure geometry: no Model/Data dependencies. Used by `tendon/spatial.rs`
//! for the 3D wrapping path computations in `mj_fwd_tendon_spatial`.

use nalgebra::{Vector2, Vector3};
use std::f64::consts::PI;

/// Result of a wrapping geometry computation.
///
/// `sphere_wrap`/`cylinder_wrap` return tangent points in the **geom-local
/// frame**. The caller (`mj_fwd_tendon_spatial`) transforms them to world frame.
pub enum WrapResult {
    /// Straight path is shorter — no wrapping around the obstacle.
    NoWrap,
    /// Path wraps around the obstacle, producing two tangent points and an arc.
    Wrapped {
        tangent_point_1: Vector3<f64>,
        tangent_point_2: Vector3<f64>,
        arc_length: f64,
    },
}

/// Test if 2D line segment a1→a2 intersects segment b1→b2.
///
/// Uses non-strict inequalities (`>=`, `<=`) matching MuJoCo's `is_intersect`:
/// endpoint-touching segments are considered intersecting.
fn segments_intersect_2d(
    a1: Vector2<f64>,
    a2: Vector2<f64>,
    b1: Vector2<f64>,
    b2: Vector2<f64>,
) -> bool {
    let da = a2 - a1;
    let db = b2 - b1;
    let denom = da.x * db.y - da.y * db.x; // 2D cross product
    if denom.abs() < 1e-20 {
        return false; // parallel or degenerate
    }
    let t = ((b1.x - a1.x) * db.y - (b1.y - a1.y) * db.x) / denom;
    let u = ((b1.x - a1.x) * da.y - (b1.y - a1.y) * da.x) / denom;
    (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)
}

/// Compute the directional arc angle between two tangent points on a circle.
///
/// Implements MuJoCo's `length_circle` algorithm. Unlike a simple `acos`
/// (which returns \[0, π\]), this function can return angles in \[0, 2π) by
/// using the 2D cross-product sign and the candidate index `ind` to determine
/// whether to take the reflex angle. Used by both sphere and cylinder wrapping.
fn directional_wrap_angle(t1: Vector2<f64>, t2: Vector2<f64>, ind: usize) -> f64 {
    // Base angle via acos: [0, π]
    let t1n = t1.normalize();
    let t2n = t2.normalize();
    let base_angle = t1n.dot(&t2n).clamp(-1.0, 1.0).acos();

    // 2D cross product determines rotational direction of t1 → t2.
    let cross = t1.y * t2.x - t1.x * t2.y;

    // MuJoCo's convention (from length_circle in engine_util_misc.c):
    // If (cross > 0 && ind == 1) || (cross < 0 && ind == 0):
    //     angle = 2π - base_angle   (take the reflex arc)
    if (cross > 0.0 && ind == 1) || (cross < 0.0 && ind == 0) {
        2.0 * PI - base_angle
    } else {
        base_angle
    }
}

/// Compute tangent point from external point `p` to a sphere of `radius` at origin.
///
/// The tangent line is in the half-plane defined by (origin, p, normal).
/// `normal` must be approximately unit length. `||p|| >= radius` is a precondition.
fn sphere_tangent_point(p: Vector3<f64>, radius: f64, normal: Vector3<f64>) -> Vector3<f64> {
    let d = p.norm();
    debug_assert!(d >= radius, "sphere_tangent_point: p is inside sphere");
    let cos_theta = radius / d;
    let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
    let u = p / d; // unit vector toward p (in wrapping plane)
    let v = normal.cross(&u).normalize(); // perpendicular to u, in wrapping plane
    radius * (u * cos_theta + v * sin_theta)
}

/// Compute both candidate tangent points from p1 and p2 to a sphere.
///
/// Uses **crossed pairing**: p1 uses `+normal`, p2 uses `-normal`, producing
/// a consistent wrapping path where the entry and exit tangent lines are on
/// opposite sides of the circle. Matches MuJoCo's `wrap_circle` convention.
fn compute_tangent_pair(
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    radius: f64,
    normal: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    (
        sphere_tangent_point(p1, radius, normal),
        sphere_tangent_point(p2, radius, -normal), // negated for p2 (crossed pairing)
    )
}

/// Compute the tangent point from an external 2D point `p` to a circle of `radius` at the origin.
///
/// `sign` (+1 or −1) selects which of the two tangent lines to use (clockwise vs
/// counterclockwise). 2D analog of [`sphere_tangent_point`].
fn circle_tangent_2d(p: Vector2<f64>, radius: f64, sign: f64) -> Vector2<f64> {
    let d = p.norm();
    debug_assert!(d >= radius, "circle_tangent_2d: p is inside circle");
    let cos_theta = radius / d;
    let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
    let u = p / d; // unit radial
    let v = Vector2::new(-u.y, u.x) * sign; // perpendicular (rotated 90°), signed
    radius * (u * cos_theta + v * sin_theta)
}

/// Compute both candidate 2D tangent-point pairs from `p1_xy` and `p2_xy` to a circle.
///
/// Uses **crossed pairing** (p2 uses `-sign`), the 2D analog of [`compute_tangent_pair`].
///
/// **Sign-to-ind mapping**: `sign=+1` → MuJoCo candidate `i=1` (`ind=1`),
/// `sign=−1` → MuJoCo candidate `i=0` (`ind=0`). See spec §4.8 for derivation.
fn compute_tangent_pair_2d(
    p1_xy: Vector2<f64>,
    p2_xy: Vector2<f64>,
    radius: f64,
    sign: f64,
) -> (Vector2<f64>, Vector2<f64>) {
    (
        circle_tangent_2d(p1_xy, radius, sign),
        circle_tangent_2d(p2_xy, radius, -sign), // negated for p2 (crossed pairing)
    )
}

/// Construct the 2D wrapping plane for sphere wrapping.
///
/// Returns `(axis0, axis1)` where `axis0 = normalize(p1)` and `axis1` is
/// perpendicular to `axis0` in the plane containing `(origin, p1, p2)`.
/// Handles the collinear degenerate case (`p1`, `O`, `p2` collinear) by
/// constructing an arbitrary perpendicular plane through `p1`.
fn sphere_wrapping_plane(p1: Vector3<f64>, p2: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let mut plane_normal = p1.cross(&p2);
    if plane_normal.norm() < 1e-10 * p1.norm() * p2.norm() {
        // Degenerate: p1, origin, p2 are collinear.
        // Construct an arbitrary perpendicular via the least-aligned cardinal axis.
        let u = p1.normalize();
        let min_axis = if u.x.abs() <= u.y.abs() && u.x.abs() <= u.z.abs() {
            Vector3::x()
        } else if u.y.abs() <= u.z.abs() {
            Vector3::y()
        } else {
            Vector3::z()
        };
        plane_normal = u.cross(&min_axis);
    }
    plane_normal = plane_normal.normalize();
    let axis0 = p1.normalize();
    let axis1 = plane_normal.cross(&axis0).normalize();
    (axis0, axis1)
}

/// 2D Newton solver for inverse (inside) tendon wrapping.
///
/// Given two 2D endpoint positions (in the wrapping plane, circle centered at
/// origin) and a circle radius, finds the single tangent point for an inverse
/// wrap path — the tendon passes *through* the geometry, touching the circle
/// surface at one point.
///
/// Implements MuJoCo's `wrap_inside()` from `engine_util_misc.c`.
///
/// Returns `None` (no wrap — tendon goes straight) or `Some(tangent_point)`.
/// `None` maps to MuJoCo's return `-1`; `Some` maps to return `0`.
#[allow(clippy::many_single_char_names)] // Newton solver math: z, f, g, a match MuJoCo variable names.
fn wrap_inside_2d(end0: Vector2<f64>, end1: Vector2<f64>, radius: f64) -> Option<Vector2<f64>> {
    const EPS: f64 = 1e-15; // MuJoCo's mjMINVAL

    // Step 1 — Validate inputs.
    let len0 = end0.norm();
    let len1 = end1.norm();
    if len0 <= radius || len1 <= radius || radius < EPS || len0 < EPS || len1 < EPS {
        return None;
    }

    // Step 2 — Segment-circle intersection check.
    let dif = end1 - end0;
    let dd = dif.norm_squared();
    if dd > EPS {
        let a = -(dif.dot(&end0)) / dd;
        if a > 0.0 && a < 1.0 {
            let nearest = end0 + a * dif;
            if nearest.norm() <= radius {
                return None; // straight path crosses circle
            }
        }
    }

    // Step 3 — Compute Newton equation parameters.
    let cap_a = radius / len0;
    let cap_b = radius / len1;
    let cos_g = (len0 * len0 + len1 * len1 - dd) / (2.0 * len0 * len1);

    if cos_g < -1.0 + EPS {
        return None; // near-antiparallel: no solution
    }

    // Step 4 — Prepare default fallback (after antiparallel check for NaN safety).
    let mid = 0.5 * (end0 + end1);
    let mid_norm = mid.norm();
    let fallback = if mid_norm > EPS {
        mid * (radius / mid_norm)
    } else {
        // Midpoint near-zero — should not happen after antiparallel guard,
        // but defend against numerical edge cases.
        return None;
    };

    if cos_g > 1.0 - EPS {
        return Some(fallback); // near-parallel / coincident: use fallback
    }

    let g = cos_g.acos();

    // Newton equation: f(z) = asin(A·z) + asin(B·z) - 2·asin(z) + G = 0
    let mut z: f64 = 1.0 - 1e-7; // initial guess near domain boundary
    let mut f = (cap_a * z).asin() + (cap_b * z).asin() - 2.0 * z.asin() + g;

    if f > 0.0 {
        return Some(fallback); // init on wrong side of root
    }

    // Step 5 — Newton iteration.
    let mut converged = false;
    for _ in 0..20 {
        if f.abs() < 1e-6 {
            converged = true;
            break;
        }

        let df = cap_a / (1.0 - z * z * cap_a * cap_a).sqrt().max(EPS)
            + cap_b / (1.0 - z * z * cap_b * cap_b).sqrt().max(EPS)
            - 2.0 / (1.0 - z * z).sqrt().max(EPS);

        if df > -EPS {
            return Some(fallback); // derivative non-negative; abort
        }

        let z_new = z - f / df;

        if z_new > z {
            return Some(fallback); // solver moving rightward; abort
        }

        z = z_new;
        f = (cap_a * z).asin() + (cap_b * z).asin() - 2.0 * z.asin() + g;

        if f > 1e-6 {
            return Some(fallback); // overshot positive; abort
        }
    }

    if !converged {
        return Some(fallback);
    }

    // Step 6 — Geometric reconstruction.
    let cross = end0.x * end1.y - end0.y * end1.x;
    let (vec, ang) = if cross > 0.0 {
        let v = end0.normalize();
        let a = z.asin() - (cap_a * z).asin();
        (v, a)
    } else {
        let v = end1.normalize();
        let a = z.asin() - (cap_b * z).asin();
        (v, a)
    };

    let tangent = Vector2::new(
        radius * (ang.cos() * vec.x - ang.sin() * vec.y),
        radius * (ang.sin() * vec.x + ang.cos() * vec.y),
    );

    Some(tangent)
}

/// Compute the shortest path around a sphere between two points.
///
/// Works in the geom-local frame where the sphere is centered at the origin.
/// Returns tangent points in the geom-local frame; the caller transforms to
/// world frame.
#[allow(clippy::similar_names)] // Dual-candidate algorithm requires paired naming (a1/a2, b1/b2).
pub fn sphere_wrap(
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    radius: f64,
    sidesite: Option<Vector3<f64>>,
) -> WrapResult {
    // 1. Early exits.
    if radius <= 0.0 {
        return WrapResult::NoWrap; // degenerate geom
    }
    if p1.norm() < radius || p2.norm() < radius {
        return WrapResult::NoWrap; // endpoint inside sphere
    }

    // 2. Check for sidesite inside sphere (inverse wrap).
    //    Uses 3D norm matching MuJoCo's mju_norm3(s) < radius.
    if let Some(ss) = sidesite {
        if ss.norm() < radius {
            let (axis0, axis1) = sphere_wrapping_plane(p1, p2);
            let end0 = Vector2::new(p1.dot(&axis0), p1.dot(&axis1));
            let end1 = Vector2::new(p2.dot(&axis0), p2.dot(&axis1));

            return match wrap_inside_2d(end0, end1, radius) {
                None => WrapResult::NoWrap,
                Some(t2d) => {
                    let t3d = axis0 * t2d.x + axis1 * t2d.y;
                    WrapResult::Wrapped {
                        tangent_point_1: t3d,
                        tangent_point_2: t3d, // identical — single tangent point
                        arc_length: 0.0,
                    }
                }
            };
        }
    }

    // 3. Check if the straight-line path misses the sphere.
    let d = p2 - p1;
    if d.norm_squared() < 1e-20 {
        return WrapResult::NoWrap; // coincident sites
    }
    let t_param = -(p1.dot(&d)) / d.norm_squared();
    let closest = p1 + t_param.clamp(0.0, 1.0) * d;
    if closest.norm() > radius {
        // Straight line clears the sphere. MuJoCo still allows wrapping if
        // a sidesite is present AND the closest point is on the opposite side
        // from the sidesite (sidesite-forced wrapping).
        match sidesite {
            None => return WrapResult::NoWrap,
            Some(ss) if closest.dot(&ss) >= 0.0 => return WrapResult::NoWrap,
            _ => {} // sidesite forces wrapping — fall through
        }
    }

    // 4. Construct 2D wrapping plane via shared helper.
    let (axis0, axis1) = sphere_wrapping_plane(p1, p2);
    let plane_normal = axis0.cross(&axis1);

    // Project endpoints and sidesite into the 2D wrapping plane.
    let p1_2d = Vector2::new(p1.dot(&axis0), p1.dot(&axis1)); // = (||p1||, 0)
    let p2_2d = Vector2::new(p2.dot(&axis0), p2.dot(&axis1));
    let ss_2d = sidesite.map(|ss| {
        let v = Vector2::new(ss.dot(&axis0), ss.dot(&axis1));
        if v.norm() > 1e-10 { v.normalize() } else { v }
    });

    // 4b. Compute both candidate tangent-point pairs (±normal).
    //     Candidate A: +normal (MuJoCo i=1), Candidate B: -normal (MuJoCo i=0).
    let (cand_a1, cand_a2) = compute_tangent_pair(p1, p2, radius, plane_normal);
    let (cand_b1, cand_b2) = compute_tangent_pair(p1, p2, radius, -plane_normal);
    // 2D projections for intersection testing:
    let proj_a1 = Vector2::new(cand_a1.dot(&axis0), cand_a1.dot(&axis1));
    let proj_a2 = Vector2::new(cand_a2.dot(&axis0), cand_a2.dot(&axis1));
    let proj_b1 = Vector2::new(cand_b1.dot(&axis0), cand_b1.dot(&axis1));
    let proj_b2 = Vector2::new(cand_b2.dot(&axis0), cand_b2.dot(&axis1));

    // 5. Select the best candidate using MuJoCo's goodness heuristic.
    //    Phase 1: Compute goodness score.
    let (mut good_a, mut good_b) = if let Some(sd) = ss_2d {
        // With sidesite: goodness = dot(normalized 2D midpoint, sidesite direction).
        let sum_a = proj_a1 + proj_a2;
        let sum_b = proj_b1 + proj_b2;
        let ga = if sum_a.norm() > 1e-10 {
            sum_a.normalize().dot(&sd)
        } else {
            -1e10
        };
        let gb = if sum_b.norm() > 1e-10 {
            sum_b.normalize().dot(&sd)
        } else {
            -1e10
        };
        (ga, gb)
    } else {
        // No sidesite: goodness = negative squared chord distance.
        let chord_a = (proj_a1 - proj_a2).norm_squared();
        let chord_b = (proj_b1 - proj_b2).norm_squared();
        (-chord_a, -chord_b)
    };

    // Phase 1b: Penalize self-intersecting candidates.
    if segments_intersect_2d(p1_2d, proj_a1, p2_2d, proj_a2) {
        good_a = -10000.0;
    }
    if segments_intersect_2d(p1_2d, proj_b1, p2_2d, proj_b2) {
        good_b = -10000.0;
    }

    // Phase 2: Select the better candidate with ind tracking.
    //          +normal → MuJoCo candidate i=1 → ind=1
    //          -normal → MuJoCo candidate i=0 → ind=0
    //          On tie, select candidate A (ind=1) to match MuJoCo's default.
    let (t1, t2, win1_2d, win2_2d, ind) = if good_a >= good_b {
        (cand_a1, cand_a2, proj_a1, proj_a2, 1)
    } else {
        (cand_b1, cand_b2, proj_b1, proj_b2, 0)
    };

    // Phase 3: Reject wrapping if the selected candidate self-intersects.
    if segments_intersect_2d(p1_2d, win1_2d, p2_2d, win2_2d) {
        return WrapResult::NoWrap;
    }

    // 6. Arc length via directional wrap angle.
    let wrap_angle = directional_wrap_angle(win1_2d, win2_2d, ind);
    let arc_length = radius * wrap_angle;

    WrapResult::Wrapped {
        tangent_point_1: t1,
        tangent_point_2: t2,
        arc_length,
    }
}

/// Compute the shortest path around an infinite cylinder between two points.
///
/// Works in the geom-local frame where the cylinder axis is Z and center is the
/// origin. The problem reduces to a 2D circle-tangent problem in XY (perpendicular
/// to the axis), plus Z-interpolation for the axial (helical) component.
/// Returns tangent points in the geom-local frame; the caller transforms to world frame.
#[allow(clippy::similar_names)] // Dual-candidate algorithm requires paired naming (a1/a2, b1/b2).
pub fn cylinder_wrap(
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    radius: f64,
    sidesite: Option<Vector3<f64>>,
) -> WrapResult {
    // 1. Project onto XY plane (perpendicular to cylinder axis).
    let p1_xy = Vector2::new(p1.x, p1.y);
    let p2_xy = Vector2::new(p2.x, p2.y);

    // 2. Early exits.
    if radius <= 0.0 {
        return WrapResult::NoWrap; // degenerate geom
    }
    if p1_xy.norm() < radius || p2_xy.norm() < radius {
        return WrapResult::NoWrap; // endpoint inside cylinder cross-section
    }

    // 2b. Check for sidesite inside cylinder (inverse wrap).
    //     Uses 3D norm matching MuJoCo's mju_norm3(s) < radius dispatch.
    //     This differs from the previous panic check which used 2D XY norm —
    //     a sidesite inside the cross-section but far along Z (norm3 > radius)
    //     now correctly uses normal wrap instead of inside wrap.
    if let Some(ss) = sidesite {
        if ss.norm() < radius {
            return match wrap_inside_2d(p1_xy, p2_xy, radius) {
                None => WrapResult::NoWrap,
                Some(t2d) => {
                    // Z interpolation with arc_length = 0.
                    let l0 = (p1_xy - t2d).norm();
                    let l1 = (p2_xy - t2d).norm();
                    let total = l0 + l1;
                    let tz = if total > 1e-10 {
                        p1.z + (p2.z - p1.z) * l0 / total
                    } else {
                        0.5 * (p1.z + p2.z)
                    };
                    let t3d = Vector3::new(t2d.x, t2d.y, tz);
                    WrapResult::Wrapped {
                        tangent_point_1: t3d,
                        tangent_point_2: t3d, // identical — single tangent point
                        arc_length: 0.0,      // no helical component
                    }
                }
            };
        }
    }

    // 3. Compute sidesite XY projection once (used in both early-exit and
    //    candidate selection). MuJoCo computes this once in mju_wrap and
    //    passes it to wrap_circle as the `side` parameter.
    let ss_dir = sidesite.map(|ss| {
        let ss_xy = Vector2::new(ss.x, ss.y);
        if ss_xy.norm() > 1e-10 {
            ss_xy.normalize()
        } else {
            ss_xy
        }
    });

    // 4. Check if the 2D line segment misses the cylinder cross-section.
    let d_xy = p2_xy - p1_xy;
    if d_xy.norm_squared() < 1e-20 {
        return WrapResult::NoWrap; // coincident sites in XY projection
    }
    let t_param = -(p1_xy.dot(&d_xy)) / d_xy.norm_squared();
    let closest = p1_xy + t_param.clamp(0.0, 1.0) * d_xy;
    if closest.norm() > radius {
        // Straight line clears the cylinder. Same sidesite-forced wrapping
        // logic as sphere (see §4.7 step 2).
        match ss_dir {
            None => return WrapResult::NoWrap,
            Some(sd) if closest.dot(&sd) >= 0.0 => return WrapResult::NoWrap,
            _ => {} // sidesite forces wrapping — fall through
        }
    }

    // 5. Compute both candidate 2D tangent-point pairs (±wrap direction).
    //    sign=+1 → MuJoCo candidate i=1 → ind=1
    //    sign=-1 → MuJoCo candidate i=0 → ind=0
    let (cand_a1, cand_a2) = compute_tangent_pair_2d(p1_xy, p2_xy, radius, 1.0); // MuJoCo i=1
    let (cand_b1, cand_b2) = compute_tangent_pair_2d(p1_xy, p2_xy, radius, -1.0); // MuJoCo i=0

    // 6. Select the best candidate using MuJoCo's goodness heuristic.
    //    Phase 1: Compute goodness score (sidesite alignment or chord distance).
    let (mut good_a, mut good_b) = if let Some(sd) = ss_dir {
        // With sidesite: goodness = dot(normalized midpoint, sidesite direction).
        let sum_a = cand_a1 + cand_a2;
        let sum_b = cand_b1 + cand_b2;
        let ga = if sum_a.norm() > 1e-10 {
            sum_a.normalize().dot(&sd)
        } else {
            -1e10
        };
        let gb = if sum_b.norm() > 1e-10 {
            sum_b.normalize().dot(&sd)
        } else {
            -1e10
        };
        (ga, gb)
    } else {
        // No sidesite: goodness = negative squared chord distance.
        let chord_a = (cand_a1 - cand_a2).norm_squared();
        let chord_b = (cand_b1 - cand_b2).norm_squared();
        (-chord_a, -chord_b)
    };

    // Phase 1b: Penalize self-intersecting candidates.
    if segments_intersect_2d(p1_xy, cand_a1, p2_xy, cand_a2) {
        good_a = -10000.0;
    }
    if segments_intersect_2d(p1_xy, cand_b1, p2_xy, cand_b2) {
        good_b = -10000.0;
    }

    // Phase 2: Select the better candidate with correct ind mapping.
    //          On tie (good_a == good_b), select candidate A (ind=1) to match
    //          MuJoCo's `i = (good[0] > good[1] ? 0 : 1)` which defaults to i=1.
    let (win1_xy, win2_xy, ind) = if good_a >= good_b {
        (cand_a1, cand_a2, 1) // sign=+1 → MuJoCo ind=1
    } else {
        (cand_b1, cand_b2, 0) // sign=-1 → MuJoCo ind=0
    };

    // Phase 3: Reject wrapping if the selected candidate self-intersects.
    if segments_intersect_2d(p1_xy, win1_xy, p2_xy, win2_xy) {
        return WrapResult::NoWrap;
    }

    // 7. Compute directional wrap angle (MuJoCo's length_circle algorithm).
    let wrap_angle = directional_wrap_angle(win1_xy, win2_xy, ind);

    // 8. Z-interpolation: path-length-proportional (MuJoCo formula).
    //    The axial (Z) coordinate is linearly interpolated based on the
    //    fraction of total 2D path length (straight1 + arc + straight2).
    //    This produces a helical path with uniform axial velocity.
    let len_straight1 = (p1_xy - win1_xy).norm(); // 2D straight: p1 → tangent1
    let len_arc = radius * wrap_angle; // 2D arc on cylinder surface
    let len_straight2 = (p2_xy - win2_xy).norm(); // 2D straight: tangent2 → p2
    let total_2d = len_straight1 + len_arc + len_straight2;
    if total_2d < 1e-10 {
        return WrapResult::NoWrap;
    }
    let t1_z = p1.z + (p2.z - p1.z) * len_straight1 / total_2d;
    let t2_z = p1.z + (p2.z - p1.z) * (len_straight1 + len_arc) / total_2d;

    // 9. Arc length of the helical path on the cylinder surface.
    let axial_disp = t2_z - t1_z; // axial travel on surface
    #[allow(clippy::imprecise_flops)] // explicit formula matches MuJoCo's C implementation
    let arc_length = (len_arc * len_arc + axial_disp * axial_disp).sqrt();

    let t1 = Vector3::new(win1_xy.x, win1_xy.y, t1_z);
    let t2 = Vector3::new(win2_xy.x, win2_xy.y, t2_z);

    WrapResult::Wrapped {
        tangent_point_1: t1,
        tangent_point_2: t2,
        arc_length,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod wrap_inside_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// T5. Newton convergence and tangency verification.
    ///
    /// Reference geometry: end0 = (2, 0), end1 = (1.5, 1.5√3), radius = 1.
    /// |end0| = 2, |end1| = 3, G = π/3 exactly.
    #[test]
    fn t5_newton_convergence_and_tangency() {
        let end0 = Vector2::new(2.0, 0.0);
        let end1 = Vector2::new(1.5, 1.5 * 3.0_f64.sqrt());
        let radius = 1.0;

        let result = wrap_inside_2d(end0, end1, radius);
        let t = result.expect("wrap_inside_2d should return Some for this geometry");

        // Tangent point lies on the circle.
        assert_relative_eq!(t.norm(), radius, epsilon = 1e-10);

        // Tangency (stationarity) condition:
        // τ · (û₀ + û₁) = 0, where τ is the circle tangent at t.
        let tau = Vector2::new(-t.y, t.x).normalize(); // tangent = perpendicular to radial
        let u0 = (t - end0).normalize();
        let u1 = (t - end1).normalize();
        let stationarity = tau.dot(&(u0 + u1)).abs();
        assert!(
            stationarity < 1e-8,
            "tangency condition violated: |τ·(û₀+û₁)| = {stationarity:.2e}"
        );
    }

    /// T11. Degenerate: segment crosses circle.
    ///
    /// Endpoints on opposite sides of the circle — the straight line
    /// intersects the circle. wrap_inside_2d must return None.
    #[test]
    fn t11_segment_crosses_circle() {
        let end0 = Vector2::new(2.0, 0.0);
        let end1 = Vector2::new(-2.0, 0.1); // opposite side, segment crosses circle
        let radius = 1.0;

        let result = wrap_inside_2d(end0, end1, radius);
        assert!(
            result.is_none(),
            "expected None when segment crosses circle"
        );
    }

    /// T12. Degenerate: endpoint exactly at circle surface.
    ///
    /// When |endpoint| = radius, the ≤ check rejects it. Returns None.
    #[test]
    fn t12_endpoint_at_surface() {
        let end0 = Vector2::new(1.0, 0.0); // exactly at radius
        let end1 = Vector2::new(2.0, 1.0);
        let radius = 1.0;

        let result = wrap_inside_2d(end0, end1, radius);
        assert!(
            result.is_none(),
            "expected None when endpoint is at circle surface"
        );
    }

    /// T13. Fallback behavior: near-degenerate geometry.
    ///
    /// Endpoints barely outside the circle (A ≈ 1, B ≈ 1) and close together
    /// on the same angular side so the connecting segment clears the circle.
    /// This triggers the cosG > 1-ε fallback path. Must return Some(fallback)
    /// with the fallback point on the circle surface.
    #[test]
    fn t13_fallback_on_circle() {
        let r = 1.0;
        // Both endpoints barely outside, nearly coincident (small angular separation).
        // Segment at x ≈ 1+1e-8 clears the circle. cosG ≈ 1 → fallback.
        let d = r + 1e-8;
        let end0 = Vector2::new(d, 1e-6);
        let end1 = Vector2::new(d, -1e-6);

        let result = wrap_inside_2d(end0, end1, r);
        let t = result.expect("near-degenerate should return Some(fallback), not None");

        // Fallback point must lie on the circle surface.
        assert_relative_eq!(t.norm(), r, epsilon = 1e-10);
    }
}
