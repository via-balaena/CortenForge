//! U3 (fit plan §7; `docs/SOFT_CONTACT_ARCHITECTURE_RECON.md` §15g step 6): why the rigid path asks the cavity
//! for room with no inset.
//!
//! The pose as written, [`slide_pose_at`], carries the scan rigidly. It walks the tip back along the centreline,
//! and turns the whole body by the rotation between the centreline's tangent at the seated tip and its tangent
//! where the tip now is. This module measures two other motions against it, on the room measure of
//! `what_the_sliding_contact_reaches_on_the_product_scan`: the moved contact's signed distance at the undeformed
//! cavity wall's nodes, negative where the wall must make room.
//! - **The slide** moves every point of the scan along the centreline by the tip's walk, keeping its place in the
//!   centreline's parallel-transported frame. It bends the scan to follow the curve, so it is not a rigid
//!   motion. The room it asks is what the scan's own cross-sections ask of the sections they pass.
//! - **The fitted pose** is the rigid motion closest to the slide: least squares over the part of the scan's
//!   surface the slide puts inside the device, each vertex weighted by its area.
//!
//! On a straight centreline the three are one translation, and on a planar circular arc the pose as written is
//! the slide (the tests below). Where the centreline's curvature changes, they part.
//!
//! ⛔ The scan never enters the repo. [`why_the_rigid_path_asks_room_on_the_product_scan`] prints to the
//! terminal, and the plan records only its ratios and verdict (Jon, 2026-09-26).

#![cfg(test)]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

use cf_design::Sdf as _;
use mesh_types::IndexedMesh;
use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};

use super::{
    Solid, TransformedSdf, point_along_polyline_at_arc_distance, slide_pose_at,
    smoothed_tangent_along_polyline, turn_between,
};

/// A centreline by arc length from the seated tip, straight past both ends along its end segments, with the
/// frame parallel transport carries along its smoothed tangent.
struct Track {
    points: Vec<Point3<f64>>,
    /// The arc length at each point.
    arcs: Vec<f64>,
    /// The frame at each point: two normals, then the smoothed tangent.
    frames: Vec<Matrix3<f64>>,
}

impl Track {
    fn new(centerline: &[Point3<f64>]) -> Self {
        assert!(centerline.len() >= 2, "a centreline needs two points");
        let mut arcs = vec![0.0];
        for pair in centerline.windows(2) {
            arcs.push(arcs[arcs.len() - 1] + (pair[1] - pair[0]).norm());
        }
        let first = smoothed_tangent_along_polyline(centerline, 0.0).unwrap();
        let seed = if first.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };
        let normal = (seed - first * seed.dot(&first)).normalize();
        let mut frames = vec![Matrix3::from_columns(&[
            normal,
            first.cross(&normal),
            first,
        ])];
        for &arc in &arcs[1..] {
            let tangent = smoothed_tangent_along_polyline(centerline, arc).unwrap();
            frames.push(turned(&frames[frames.len() - 1], &tangent));
        }
        Self {
            points: centerline.to_vec(),
            arcs,
            frames,
        }
    }

    fn length(&self) -> f64 {
        self.arcs[self.arcs.len() - 1]
    }

    /// The point at arc `s`.
    fn point(&self, s: f64) -> Point3<f64> {
        if s < 0.0 {
            let along = (self.points[1] - self.points[0]).normalize();
            return self.points[0] + along * s;
        }
        point_along_polyline_at_arc_distance(&self.points, s)
            .unwrap()
            .0
    }

    /// The frame at arc `s`. Within a segment the smoothed tangent turns on one great circle, so turning the
    /// frame at the segment's start straight onto it is the parallel transport.
    fn frame(&self, s: f64) -> Matrix3<f64> {
        let below = self.arcs.partition_point(|&arc| arc <= s).saturating_sub(1);
        let tangent = smoothed_tangent_along_polyline(&self.points, s.max(0.0)).unwrap();
        turned(&self.frames[below], &tangent)
    }

    /// The arc length of the closest point on the centreline, straight past both ends.
    fn arc_of(&self, p: Point3<f64>) -> f64 {
        let last = self.points.len() - 2;
        let mut best = (f64::INFINITY, 0.0);
        for (i, pair) in self.points.windows(2).enumerate() {
            let along = pair[1] - pair[0];
            let length = along.norm();
            if length < f64::EPSILON {
                continue;
            }
            let mut u = (p - pair[0]).dot(&along) / (length * length);
            if i > 0 {
                u = u.max(0.0);
            }
            if i < last {
                u = u.min(1.0);
            }
            let distance = (p - (pair[0] + along * u)).norm();
            if distance < best.0 {
                best = (distance, self.arcs[i] + u * length);
            }
        }
        best.1
    }

    /// Where the slide by `walk` carries `p`: `walk` further from the seated tip, at the same place in the
    /// frame.
    fn slid(&self, p: Point3<f64>, walk: f64) -> Point3<f64> {
        let s = self.arc_of(p);
        let local = self.frame(s).transpose() * (p - self.point(s));
        self.point(s + walk) + self.frame(s + walk) * local
    }
}

/// `frame` turned by the smallest rotation that takes its tangent onto `tangent`.
fn turned(frame: &Matrix3<f64>, tangent: &Vector3<f64>) -> Matrix3<f64> {
    let rotation =
        turn_between(&frame.column(2).into_owned(), tangent).unwrap_or_else(Rotation3::identity);
    rotation.matrix() * frame
}

/// The rigid motion that carries each `from` point closest to its `to` point, in least squares weighted by
/// `weights` (Kabsch). `None` when the weights sum to zero.
fn fitted_motion(
    from: &[Point3<f64>],
    to: &[Point3<f64>],
    weights: &[f64],
) -> Option<Isometry3<f64>> {
    let total: f64 = weights.iter().sum();
    if total <= 0.0 {
        return None;
    }
    let mean = |points: &[Point3<f64>]| {
        points
            .iter()
            .zip(weights)
            .fold(Vector3::zeros(), |sum, (p, w)| sum + p.coords * *w)
            / total
    };
    let (a, b) = (mean(from), mean(to));
    let mut covariance = Matrix3::zeros();
    for ((p, q), w) in from.iter().zip(to).zip(weights) {
        covariance += (p.coords - a) * (q.coords - b).transpose() * *w;
    }
    // `svd` iterates until it converges, and never does on a non-finite matrix.
    if !covariance.iter().all(|x| x.is_finite()) {
        return None;
    }
    let svd = covariance.try_svd(true, true, f64::EPSILON, 1000)?;
    let (u, v) = (svd.u?, svd.v_t?.transpose());
    let mut sign = Matrix3::identity();
    if (v * u.transpose()).determinant() < 0.0 {
        let smallest = svd.singular_values.imin();
        sign[(smallest, smallest)] = -1.0;
    }
    // U and V are orthogonal and the sign makes the product proper, so it is a rotation as it stands.
    let rotation = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
        v * sign * u.transpose(),
    ));
    Some(Isometry3::from_parts(
        Translation3::from(b - rotation * a),
        rotation,
    ))
}

/// Each vertex's share of the surface: a third of each triangle it is a corner of.
fn vertex_areas(mesh: &IndexedMesh) -> Vec<f64> {
    let mut areas = vec![0.0; mesh.vertices.len()];
    for face in &mesh.faces {
        let [a, b, c] = face.map(|v| mesh.vertices[v as usize]);
        let third = (b - a).cross(&(c - a)).norm() / 6.0;
        for v in face {
            areas[*v as usize] += third;
        }
    }
    areas
}

/// The fitted pose at walk `walk`: the rigid motion closest to the slide over the vertices the slide puts
/// where `inside` holds.
fn fitted_pose(
    track: &Track,
    surface: &[Point3<f64>],
    areas: &[f64],
    walk: f64,
    inside: impl Fn(Point3<f64>) -> bool,
) -> Option<Isometry3<f64>> {
    let (mut from, mut to, mut weights) = (Vec::new(), Vec::new(), Vec::new());
    for (&p, &area) in surface.iter().zip(areas) {
        let q = track.slid(p, walk);
        if inside(q) {
            from.push(p);
            to.push(q);
            weights.push(area);
        }
    }
    fitted_motion(&from, &to, &weights)
}

/// A planar test curve: from the seated tip along +x, turning at curvature `curvatures[i]` (about +z, positive
/// to the left) for `lengths[i]`, sampled every `spacing`. Returns the polyline and the exact point and heading
/// at any arc.
fn planar_curve(lengths: &[f64], curvatures: &[f64], spacing: f64) -> Vec<Point3<f64>> {
    let total: f64 = lengths.iter().sum();
    let count = (total / spacing).ceil() as usize;
    (0..=count)
        .map(|k| exact_planar(lengths, curvatures, total * k as f64 / count as f64).0)
        .collect()
}

/// The exact point and heading at arc `s` of [`planar_curve`]'s curve.
fn exact_planar(lengths: &[f64], curvatures: &[f64], s: f64) -> (Point3<f64>, f64) {
    let (mut at, mut heading, mut left) = (Point3::origin(), 0.0_f64, s);
    for (&length, &curvature) in lengths.iter().zip(curvatures) {
        let run = left.min(length);
        let turn = curvature * run;
        let chord = if curvature == 0.0 {
            Vector3::new(run * heading.cos(), run * heading.sin(), 0.0)
        } else {
            Vector3::new(
                ((heading + turn).sin() - heading.sin()) / curvature,
                (heading.cos() - (heading + turn).cos()) / curvature,
                0.0,
            )
        };
        at += chord;
        heading += turn;
        left -= run;
        if left <= 0.0 {
            break;
        }
    }
    (at, heading)
}

/// Points on a tube of `radius` around the planar curve, at `rings` arcs and `around` angles.
fn tube_surface(
    lengths: &[f64],
    curvatures: &[f64],
    radius: f64,
    rings: usize,
) -> Vec<Point3<f64>> {
    let total: f64 = lengths.iter().sum();
    let mut points = Vec::new();
    for k in 0..=rings {
        let (centre, heading) = exact_planar(lengths, curvatures, total * k as f64 / rings as f64);
        let side = Vector3::new(-heading.sin(), heading.cos(), 0.0);
        for j in 0..12 {
            let angle = std::f64::consts::TAU * f64::from(j) / 12.0;
            points.push(centre + (side * angle.cos() + Vector3::z() * angle.sin()) * radius);
        }
    }
    points
}

#[test]
fn a_fitted_motion_recovers_a_rigid_motion() {
    let motion = Isometry3::new(Vector3::new(0.3, -1.2, 2.0), Vector3::new(0.4, -0.7, 1.1));
    let from: Vec<Point3<f64>> = (0..20)
        .map(|k| {
            let k = f64::from(k);
            Point3::new(k.sin() * 3.0, (1.7 * k).cos(), 0.25 * k)
        })
        .collect();
    let to: Vec<Point3<f64>> = from.iter().map(|p| motion * p).collect();
    let weights: Vec<f64> = (0..20).map(|k| 1.0 + f64::from(k % 3)).collect();
    let fitted = fitted_motion(&from, &to, &weights).unwrap();
    for (p, q) in from.iter().zip(&to) {
        assert!(
            (fitted * p - q).norm() < 1e-12,
            "{}",
            (fitted * p - q).norm()
        );
    }
    assert!(fitted_motion(&from, &to, &[0.0; 20]).is_none());
}

#[test]
fn a_mirror_image_is_fitted_by_the_best_rotation() {
    // Points spread 3, 2 and 1 along x, y and z, mirrored in x. The covariance is diag(−18, 8, 2), whose
    // nearest rotation turns the smallest spread (z) with x: a half turn about y, carrying (x, y, z) to
    // (−x, y, −z). Kept as it is, the mirror is no rotation; flipping any other direction gives the identity or
    // a half turn about z.
    let from: Vec<Point3<f64>> = [3.0, 2.0, 1.0]
        .iter()
        .enumerate()
        .flat_map(|(axis, &spread)| {
            [spread, -spread].map(|at| {
                let mut p = Point3::origin();
                p[axis] = at;
                p
            })
        })
        .collect();
    let to: Vec<Point3<f64>> = from.iter().map(|p| Point3::new(-p.x, p.y, p.z)).collect();
    let fitted = fitted_motion(&from, &to, &[1.0; 6]).unwrap();
    for p in &from {
        let expected = Point3::new(-p.x, p.y, -p.z);
        assert!(
            (fitted * p - expected).norm() < 1e-12,
            "{p} → {}",
            fitted * p
        );
    }
}

#[test]
fn a_fitted_motion_follows_the_weight() {
    // Two halves of a set moved two ways: with the second half weighted zero, the fit is the first half's
    // motion exactly; weighted evenly, it is neither.
    let (first, second) = (
        Isometry3::new(Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.3)),
        Isometry3::new(Vector3::new(0.0, 2.0, 0.0), Vector3::new(0.2, 0.0, 0.0)),
    );
    let from: Vec<Point3<f64>> = (0..20)
        .map(|k| {
            let k = f64::from(k);
            Point3::new(k.sin() * 3.0, (1.7 * k).cos(), 0.25 * k)
        })
        .collect();
    let to: Vec<Point3<f64>> = from
        .iter()
        .enumerate()
        .map(|(k, p)| if k < 10 { first * p } else { second * p })
        .collect();
    let weights: Vec<f64> = (0..20).map(|k| if k < 10 { 2.5 } else { 0.0 }).collect();
    let fitted = fitted_motion(&from, &to, &weights).unwrap();
    for p in &from {
        assert!((fitted * p - first * p).norm() < 1e-12);
    }
    let even = fitted_motion(&from, &to, &[1.0; 20]).unwrap();
    assert!(from.iter().any(|p| (even * p - first * p).norm() > 0.1));
}

#[test]
fn a_vertex_takes_a_third_of_each_triangle_it_is_a_corner_of() {
    // A quadrilateral cut along one diagonal into triangles of area 1.5 and 3: the diagonal's ends are corners
    // of both.
    let mut mesh = IndexedMesh::new();
    for (x, y) in [(0.0, 0.0), (3.0, 0.0), (3.0, 1.0), (0.0, 2.0)] {
        mesh.vertices.push(Point3::new(x, y, 0.0));
    }
    mesh.faces.push([0, 1, 2]);
    mesh.faces.push([0, 2, 3]);
    let areas = vertex_areas(&mesh);
    for (area, expected) in areas.iter().zip([1.5, 0.5, 1.5, 1.0]) {
        assert!((area - expected).abs() < 1e-15, "{areas:?}");
    }
}

#[test]
fn the_track_follows_its_curve_past_both_ends() {
    let (lengths, curvatures) = ([30.0, 30.0], [0.02, -0.03]);
    let track = Track::new(&planar_curve(&lengths, &curvatures, 0.5));
    // The smoothed tangent's turn from the curve's, with h = 0.5: at an end it is the end segment's, κh/2 off;
    // at the join, the mean of the two segments' turns, (κ₂ − κ₁)h/4. Within an arc it is the curve's but for the
    // polyline's arc lagging the curve's (a chord is shorter than its arc), a lag that grows along the curve and
    // stays under 1e-5 of turn at these points. The segment's own tangent would be off by up to κh/2 (4.5e-3 at
    // 44.4).
    let expected_turn = [
        (0.0, 0.005),
        (7.3, 0.0),
        (30.0, -0.00625),
        (44.4, 0.0),
        (60.0, 0.0075),
    ];
    for (s, turn) in expected_turn {
        let (exact, heading) = exact_planar(&lengths, &curvatures, s);
        assert!((track.point(s) - exact).norm() < 1e-3, "{s}");
        let frame = track.frame(s);
        let tangent = frame.column(2);
        let off = tangent.y.atan2(tangent.x) - heading;
        assert!((off - turn).abs() < 1e-5, "{s}: {off}");
        assert!((frame.transpose() * frame - Matrix3::identity()).norm() < 1e-12);
        // Projected onto a chord, a point 4 off the curve lands within 4 × κh/2 of its arc (0.03 here).
        assert!(
            (track.arc_of(exact + frame.column(0) * 4.0) - s).abs() < 0.04,
            "{s}"
        );
    }
    // Straight on past either end, along the end segment: κh/2 off the curve's tangent, so 5 on, the point is
    // within 5κh/2 of the tangent line (0.025 at the start, 0.0375 at the end).
    assert!((track.point(-5.0) - Point3::new(-5.0, 0.0, 0.0)).norm() < 0.03);
    assert!((track.arc_of(Point3::new(-5.0, 2.0, 0.0)) + 5.0).abs() < 0.02);
    let (end, heading) = exact_planar(&lengths, &curvatures, 60.0);
    let beyond = end + Vector3::new(heading.cos(), heading.sin(), 0.0) * 5.0;
    assert!((track.point(65.0) - beyond).norm() < 0.05);
    assert!((track.arc_of(beyond) - 65.0).abs() < 2e-2);
}

#[test]
fn on_a_straight_centreline_the_three_motions_agree_and_a_taper_asks_its_own_room() {
    // A cone around the x axis narrowing away from the tip: radius 10 at the tip, 1 in 20 per unit of arc. Its
    // exact distance is the radial excess times the cosine of its half-angle.
    let taper = 0.05;
    let cone = move |p: Point3<f64>| (p.y.hypot(p.z) - (10.0 - taper * p.x)) / taper.hypot(1.0);
    let centerline: Vec<Point3<f64>> = (0..=10)
        .map(|k| Point3::new(10.0 * f64::from(k), 0.0, 0.0))
        .collect();
    let track = Track::new(&centerline);
    let surface = tube_surface(&[100.0], &[0.0], 8.0, 50);
    let areas = vec![1.0; surface.len()];
    let walk = 30.0;
    let fitted = fitted_pose(&track, &surface, &areas, walk, |q| q.x <= 100.0).unwrap();
    let tip = slide_pose_at(&centerline, 1.0 - walk / 100.0);
    for node in [Point3::new(60.0, 7.0, 0.0), Point3::new(95.0, 0.0, -5.25)] {
        let slide = cone(track.slid(node, -walk));
        assert!((cone(tip.inverse_transform_point(&node)) - slide).abs() < 1e-9);
        assert!((cone(fitted.inverse_transform_point(&node)) - slide).abs() < 1e-9);
        // The body section that reaches the node is `walk` nearer the tip, so `taper * walk` wider.
        let expected = (node.y.hypot(node.z) - (10.0 - taper * (node.x - walk))) / taper.hypot(1.0);
        assert!((slide - expected).abs() < 1e-9, "{slide} {expected}");
    }
}

#[test]
fn on_a_planar_arc_the_pose_as_written_is_the_slide() {
    // The pose as written reads the seated tip's tangent off the first segment alone, half a segment's turn
    // (κh/2) from the curve's, so the two differ by up to κh/2 times the reach: 0.004 here.
    let (lengths, curvatures) = ([100.0], [0.01]);
    let centerline = planar_curve(&lengths, &curvatures, 0.01);
    let track = Track::new(&centerline);
    let surface = tube_surface(&lengths, &curvatures, 8.0, 40);
    let walk = 35.0;
    let tip = slide_pose_at(&centerline, 1.0 - walk / track.length());
    let worst = surface
        .iter()
        .filter(|p| track.arc_of(**p) + walk <= 100.0)
        .map(|p| (tip * p - track.slid(*p, walk)).norm())
        .fold(0.0, f64::max);
    assert!(worst < 5e-3, "{worst}");
    // On a coarser sampling the same bound grows with the spacing: 0.09 at h = 0.25.
    let coarse = planar_curve(&lengths, &curvatures, 0.25);
    let coarse_tip = slide_pose_at(&coarse, 1.0 - walk / Track::new(&coarse).length());
    let coarse_worst = surface
        .iter()
        .filter(|p| track.arc_of(**p) + walk <= 100.0)
        .map(|p| (coarse_tip * p - track.slid(*p, walk)).norm())
        .fold(0.0, f64::max);
    assert!(coarse_worst > 0.05 && coarse_worst < 0.1, "{coarse_worst}");
}

#[test]
fn where_the_curvature_changes_the_pose_as_written_turns_the_far_end_off_the_curve() {
    // An S: 50 at curvature +1/100, then 50 at −1/100. Walked 25, the tip has turned a quarter radian, and the
    // pose as written turns the whole body by that about the tip. The body's far end, 75 from the tip, lands
    // where that rotation puts it; the curve there, at arc 100, has turned back to its start.
    let (lengths, curvatures) = ([50.0, 50.0], [0.01, -0.01]);
    let centerline = planar_curve(&lengths, &curvatures, 0.01);
    let track = Track::new(&centerline);
    let walk = 25.0;
    let tip = slide_pose_at(&centerline, 1.0 - walk / track.length());
    let (seated_tip, _) = exact_planar(&lengths, &curvatures, 0.0);
    let (moved_tip, turn) = exact_planar(&lengths, &curvatures, walk);
    let (far, _) = exact_planar(&lengths, &curvatures, 75.0);
    let rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), turn);
    let expected = moved_tip + rotation * (far - seated_tip);
    assert!(
        (tip * far - expected).norm() < 5e-3,
        "{}",
        (tip * far - expected).norm()
    );
    let (on_curve, _) = exact_planar(&lengths, &curvatures, 100.0);
    let off = (expected - on_curve).norm();
    assert!(off > 5.0, "{off}");
    // The fitted pose is fitted over the points the slide carries inside (arc 100 here), not the points that
    // start inside: it is the least-squares fit over that set, which no other rigid motion beats.
    let surface = tube_surface(&lengths, &curvatures, 8.0, 40);
    let areas: Vec<f64> = (0..surface.len()).map(|k| 1.0 + (k % 3) as f64).collect();
    let inside = |q: Point3<f64>| track.arc_of(q) <= 100.0;
    let fitted = fitted_pose(&track, &surface, &areas, walk, inside).unwrap();
    let (mut from, mut to, mut weights) = (Vec::new(), Vec::new(), Vec::new());
    for (p, area) in surface.iter().zip(&areas) {
        let q = track.slid(*p, walk);
        if inside(q) {
            from.push(*p);
            to.push(q);
            weights.push(*area);
        }
    }
    let direct = fitted_motion(&from, &to, &weights).unwrap();
    assert!(
        from.iter()
            .all(|p| (fitted * p - direct * p).norm() < 1e-12)
    );
    let every: Vec<_> = surface.iter().map(|p| track.slid(*p, walk)).collect();
    let over_all = fitted_motion(&surface, &every, &areas).unwrap();
    assert!(
        from.iter()
            .any(|p| (over_all * p - fitted * p).norm() > 0.1)
    );
    let misses = |pose: &Isometry3<f64>| -> f64 {
        from.iter()
            .zip(&to)
            .zip(&weights)
            .map(|((p, q), w)| w * (pose * p - q).norm_squared())
            .sum()
    };
    assert!(misses(&fitted) < misses(&tip) && misses(&fitted) < misses(&over_all));
}

#[test]
fn the_track_transports_its_frame_without_spin_along_a_helix() {
    // A helix of radius 10 rising 3 a radian: curvature 10/109 and torsion 3/109. Parallel transport turns the
    // frame against the Frenet frame at minus the torsion, so over arc S the normal turns −τS in the
    // (normal, binormal) plane. On a planar curve the transport cannot be told from any other turn.
    let (radius, rise) = (10.0_f64, 3.0_f64);
    let speed = radius.hypot(rise);
    let torsion = rise / (speed * speed);
    let at = |angle: f64| Point3::new(radius * angle.cos(), radius * angle.sin(), rise * angle);
    let centerline: Vec<Point3<f64>> = (0..=2000).map(|k| at(f64::from(k) * 0.005)).collect();
    let track = Track::new(&centerline);
    let phase = |s: f64| {
        let angle = s / speed;
        let normal = Vector3::new(-angle.cos(), -angle.sin(), 0.0);
        let binormal = Vector3::new(rise * angle.sin(), -rise * angle.cos(), radius) / speed;
        let n = track.frame(s).column(0).into_owned();
        n.dot(&binormal).atan2(n.dot(&normal))
    };
    // A frame that is not transported misses by radians; this one by 1.5e-5, which is not isolated.
    let span = 0.9 * track.length();
    let turned = (phase(span) - phase(0.0) + torsion * span).rem_euclid(std::f64::consts::TAU);
    let off = turned.min(std::f64::consts::TAU - turned);
    assert!(off < 1e-4, "{off} against a twist of {}", torsion * span);
}

/// The room each motion asks of the product's cavity wall, with no inset, over the pose as written's 64 poses.
///
/// The contact is the scan offset by the cavity's inset, as in
/// `what_the_sliding_contact_reaches_on_the_product_scan`'s "as written" row, so fully seated it is the cavity
/// itself. For each motion it prints the most room over the travel, where it is asked, how many wall nodes are
/// ever asked for more than the old bridge's `d̂`, and the room fully seated; then the most room at each pose.
/// Beside them: how far the slide is from rigid (the fitted pose's worst miss of the slide over the scan's
/// surface inside the device), and how far a wall node lands from itself when slid there and back.
///
/// ⛔ The scan is repo-excluded, so nothing here can gate; it asserts only that the instrument ran.
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn why_the_rigid_path_asks_room_on_the_product_scan() {
    const POSES: usize = 64;
    let (scan, centerline, caps, _) =
        super::tests::product_scene().expect("U3 measures the product scan, which must be present");
    let (g, _, _, boundary) = super::tests::sliding_product_scene().unwrap();
    let contact = Solid::from_sdf(g.intruder.clone(), g.bounds).offset(g.cavity_offset_m);
    let wall: Vec<Point3<f64>> = boundary
        .iter()
        .map(|p| Point3::from(*p))
        .filter(|p| contact.eval(*p).abs() < g.cell_size_m)
        .collect();
    let moved = |pose: Isometry3<f64>| {
        Solid::from_sdf(TransformedSdf::new(g.intruder.clone(), pose), g.bounds)
            .offset(g.cavity_offset_m)
    };
    let track = Track::new(&centerline);
    let length = track.length();
    let areas = vertex_areas(&scan);
    let inside = |q: Point3<f64>| {
        caps.iter()
            .all(|cap| (q - cap.centroid).dot(&cap.normal) < 0.0)
    };
    let d_hat = super::BRIDGE_CONTACT_DHAT_M;
    println!(
        "\n══ U3 · base_mold · {} cavity-wall nodes · {} scan vertices · no inset ══",
        wall.len(),
        scan.vertices.len()
    );

    const MOTIONS: [&str; 3] = ["as written", "fitted pose", "slide (not rigid)"];
    // Per motion: the most room (value, t, node), each node's most room, and the room fully seated.
    let mut deepest = [(f64::INFINITY, 0.0, Point3::origin()); 3];
    let mut lowest = [
        vec![f64::INFINITY; wall.len()],
        vec![f64::INFINITY; wall.len()],
        vec![f64::INFINITY; wall.len()],
    ];
    let mut profile = Vec::new();
    let (mut worst_miss, mut worst_return) = ((0.0, 0.0), 0.0_f64);
    // The largest move of a scan vertex inside the device between consecutive poses, per rigid motion.
    let (mut previous, mut largest_move): (Option<[Isometry3<f64>; 2]>, [f64; 2]) =
        (None, [0.0; 2]);
    for k in 1..=POSES {
        let t = k as f64 / POSES as f64;
        let walk = length * (1.0 - t);
        let tip_pose = slide_pose_at(&centerline, t);
        let tip = moved(tip_pose);
        let fitted_pose = fitted_pose(&track, &scan.vertices, &areas, walk, inside).unwrap();
        let fitted = moved(fitted_pose);
        let mut row = [f64::INFINITY; 3];
        for (n, node) in wall.iter().enumerate() {
            let back = track.slid(*node, -walk);
            worst_return = worst_return.max((track.slid(back, walk) - node).norm());
            let rooms = [tip.eval(*node), fitted.eval(*node), contact.eval(back)];
            for (m, room) in rooms.into_iter().enumerate() {
                row[m] = row[m].min(room);
                lowest[m][n] = lowest[m][n].min(room);
                if room < deepest[m].0 {
                    deepest[m] = (room, t, *node);
                }
            }
        }
        for (p, area) in scan.vertices.iter().zip(&areas) {
            let target = track.slid(*p, walk);
            if *area > 0.0 && inside(target) {
                let miss = (fitted_pose * p - target).norm();
                if miss > worst_miss.0 {
                    worst_miss = (miss, t);
                }
                if let Some(before) = previous {
                    for (m, pose) in [tip_pose, fitted_pose].iter().enumerate() {
                        largest_move[m] = largest_move[m].max((pose * p - before[m] * p).norm());
                    }
                }
            }
        }
        previous = Some([tip_pose, fitted_pose]);
        profile.push((t, row));
    }
    let seated = profile[POSES - 1].1;

    println!(
        "\n{:<20} {:>12} {:>8} {:>14} {:>9} {:>14} {:>12}",
        "motion", "most room mm", "at t", "node arc mm", "off mm", "ever < -d_hat", "seated min"
    );
    for m in 0..3 {
        let (room, t, node) = deepest[m];
        let arc = track.arc_of(node);
        let off = (node - track.point(arc)).norm();
        let ever = lowest[m].iter().filter(|&&r| r < -d_hat).count();
        println!(
            "{:<20} {:>12.4} {:>8.4} {:>14.2} {:>9.2} {:>9}/{:<4} {:>12.4}",
            MOTIONS[m],
            room * 1e3,
            t,
            arc * 1e3,
            off * 1e3,
            ever,
            wall.len(),
            seated[m] * 1e3,
        );
    }
    println!(
        "ratios to as written: fitted {:.3} · slide {:.3}",
        deepest[1].0 / deepest[0].0,
        deepest[2].0 / deepest[0].0
    );
    println!(
        "the slide's distance from rigid: the fitted pose misses it by at most {:.3} mm (t = {:.4}) · a node slid \
         there and back lands within {:.2e} mm of itself",
        worst_miss.0 * 1e3,
        worst_miss.1,
        worst_return * 1e3
    );
    println!(
        "the largest move of a scan vertex inside the device between poses: as written {:.3} mm · fitted {:.3} \
         mm · the tip's arc step {:.3} mm",
        largest_move[0] * 1e3,
        largest_move[1] * 1e3,
        length / POSES as f64 * 1e3
    );
    println!("\nmost room at each pose (mm): t · as written · fitted · slide");
    for (t, row) in &profile {
        println!(
            "{t:.4} {:>9.3} {:>9.3} {:>9.3}",
            row[0] * 1e3,
            row[1] * 1e3,
            row[2] * 1e3
        );
    }
    assert!(
        deepest.iter().all(|d| d.0.is_finite()),
        "every motion measured"
    );
}
