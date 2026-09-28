//! The scan's path (`sim_soft::lowering::path`, plan §16t, §16w): the centreline's transported frame, the slide,
//! and the least-squares rigid motion, against answers known in closed form.

// A fit that fails is the test's panic; the planar curve's sample count is a small whole number.
#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]

use mesh_types::IndexedMesh;
use nalgebra::{Isometry3, Matrix3, Point3, Vector3};
use sim_soft::lowering::Plane;
use sim_soft::lowering::path::{
    Centreline, FittedPath, MOST_INTERVALS, PathError, WALK_GRID, fitted_motion, travelled,
    vertex_areas,
};
use sim_soft::obstacle::SignedDistance;
use sim_soft_explicit::f64::{pose_interpolate, pose_sample_span, pose_to_world};
use sim_soft_explicit::fixtures::tube::Insertion;

/// A planar test curve: from the seated tip along +x, turning at curvature `curvatures[i]` (about +z, positive to
/// the left) for `lengths[i]`, sampled every `spacing`.
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

/// Points spread through space, for the fits.
fn spread() -> Vec<Point3<f64>> {
    (0..20)
        .map(|k| {
            let k = f64::from(k);
            Point3::new(k.sin() * 3.0, (1.7 * k).cos(), 0.25 * k)
        })
        .collect()
}

#[test]
fn a_fitted_motion_recovers_a_rigid_motion() {
    let motion = Isometry3::new(Vector3::new(0.3, -1.2, 2.0), Vector3::new(0.4, -0.7, 1.1));
    let from = spread();
    let to: Vec<Point3<f64>> = from.iter().map(|p| motion * p).collect();
    let weights: Vec<f64> = (0..20).map(|k| 1.0 + f64::from(k % 3)).collect();
    let fitted = fitted_motion(&from, &to, &weights).expect("a fit");
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
    // Points spread 3, 2 and 1 along x, y and z, mirrored in x. The covariance is diag(−18, 8, 2), whose nearest
    // rotation turns the smallest spread (z) with x: a half turn about y, carrying (x, y, z) to (−x, y, −z). Kept as
    // it is, the mirror is no rotation; flipping any other direction gives the identity or a half turn about z.
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
    let fitted = fitted_motion(&from, &to, &[1.0; 6]).expect("a fit");
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
    // Two halves of a set moved two ways: with the second half weighted zero, the fit is the first half's motion
    // exactly; weighted evenly, it is neither.
    let (first, second) = (
        Isometry3::new(Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.3)),
        Isometry3::new(Vector3::new(0.0, 2.0, 0.0), Vector3::new(0.2, 0.0, 0.0)),
    );
    let from = spread();
    let to: Vec<Point3<f64>> = from
        .iter()
        .enumerate()
        .map(|(k, p)| if k < 10 { first * p } else { second * p })
        .collect();
    let weights: Vec<f64> = (0..20).map(|k| if k < 10 { 2.5 } else { 0.0 }).collect();
    let fitted = fitted_motion(&from, &to, &weights).expect("a fit");
    for p in &from {
        assert!((fitted * p - first * p).norm() < 1e-12);
    }
    let even = fitted_motion(&from, &to, &[1.0; 20]).expect("a fit");
    assert!(from.iter().any(|p| (even * p - first * p).norm() > 0.1));
}

#[test]
fn a_vertex_takes_a_third_of_each_triangle_it_is_a_corner_of() {
    // A quadrilateral cut along one diagonal into triangles of area 1.5 and 3: the diagonal's ends are corners of
    // both.
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
fn the_centreline_follows_its_curve_past_both_ends() {
    let (lengths, curvatures) = ([30.0, 30.0], [0.02, -0.03]);
    let centreline = Centreline::new(&planar_curve(&lengths, &curvatures, 0.5)).expect("a curve");
    // The smoothed tangent's turn from the curve's, with h = 0.5: at an end it is the end segment's, κh/2 off; at
    // the join, the mean of the two segments' turns, (κ₂ − κ₁)h/4. Within an arc it is the curve's but for the
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
        assert!((centreline.point(s) - exact).norm() < 1e-3, "{s}");
        let frame = centreline.frame(s);
        let tangent = frame.column(2);
        let off = tangent.y.atan2(tangent.x) - heading;
        assert!((off - turn).abs() < 1e-5, "{s}: {off}");
        assert!((frame.transpose() * frame - Matrix3::identity()).norm() < 1e-12);
        // Projected onto a chord, a point 4 off the curve lands within 4 × κh/2 of its arc (0.03 here).
        assert!(
            (centreline.arc_of(exact + frame.column(0) * 4.0) - s).abs() < 0.04,
            "{s}"
        );
    }
    // Straight on past either end, along the end segment: κh/2 off the curve's tangent, so 5 on, the point is within
    // 5κh/2 of the tangent line (0.025 at the start, 0.0375 at the end).
    assert!((centreline.point(-5.0) - Point3::new(-5.0, 0.0, 0.0)).norm() < 0.03);
    assert!((centreline.arc_of(Point3::new(-5.0, 2.0, 0.0)) + 5.0).abs() < 0.02);
    let (end, heading) = exact_planar(&lengths, &curvatures, 60.0);
    let beyond = end + Vector3::new(heading.cos(), heading.sin(), 0.0) * 5.0;
    assert!((centreline.point(65.0) - beyond).norm() < 0.05);
    assert!((centreline.arc_of(beyond) - 65.0).abs() < 2e-2);
}

#[test]
fn the_centreline_transports_its_frame_without_spin_along_a_helix() {
    // A helix of radius 10 rising 3 a radian: curvature 10/109 and torsion 3/109. Parallel transport turns the frame
    // against the Frenet frame at minus the torsion, so over arc S the normal turns −τS in the (normal, binormal)
    // plane. On a planar curve the transport cannot be told from any other turn.
    let (radius, rise) = (10.0_f64, 3.0_f64);
    let speed = radius.hypot(rise);
    let torsion = rise / (speed * speed);
    let at = |angle: f64| Point3::new(radius * angle.cos(), radius * angle.sin(), rise * angle);
    let points: Vec<Point3<f64>> = (0..=2000).map(|k| at(f64::from(k) * 0.005)).collect();
    let centreline = Centreline::new(&points).expect("a helix");
    let phase = |s: f64| {
        let angle = s / speed;
        let normal = Vector3::new(-angle.cos(), -angle.sin(), 0.0);
        let binormal = Vector3::new(rise * angle.sin(), -rise * angle.cos(), radius) / speed;
        let n = centreline.frame(s).column(0).into_owned();
        n.dot(&binormal).atan2(n.dot(&normal))
    };
    // A frame that is not transported misses by radians; this one by 1.5e-5, which is not isolated.
    let span = 0.9 * centreline.length();
    let turned = (phase(span) - phase(0.0) + torsion * span).rem_euclid(std::f64::consts::TAU);
    let off = turned.min(std::f64::consts::TAU - turned);
    assert!(off < 1e-4, "{off} against a twist of {}", torsion * span);
}

#[test]
fn on_a_planar_arc_the_slide_is_rigid() {
    // A circular arc in a plane: sliding along it is a rotation about its centre, so every point the slide carries
    // is carried by one rigid motion, turned by κ times the walk. The polyline's slide misses one rigid motion by
    // 4.3e-4 here and its turn by 1.2e-7 (measured); what sets the miss is not isolated.
    let (lengths, curvatures) = ([100.0], [0.01]);
    let centreline = Centreline::new(&planar_curve(&lengths, &curvatures, 0.01)).expect("an arc");
    let from: Vec<Point3<f64>> = (0..=40)
        .flat_map(|k| {
            let (centre, heading) = exact_planar(&lengths, &curvatures, 60.0 * f64::from(k) / 40.0);
            let side = Vector3::new(-heading.sin(), heading.cos(), 0.0);
            (0..12).map(move |j| {
                let angle = std::f64::consts::TAU * f64::from(j) / 12.0;
                centre + (side * angle.cos() + Vector3::z() * angle.sin()) * 8.0
            })
        })
        .collect();
    let walk = 35.0;
    let to: Vec<Point3<f64>> = from.iter().map(|p| centreline.slid(*p, walk)).collect();
    let fitted = fitted_motion(&from, &to, &vec![1.0; from.len()]).expect("a fit");
    let worst = from
        .iter()
        .zip(&to)
        .map(|(p, q)| (fitted * p - q).norm())
        .fold(0.0, f64::max);
    assert!(worst < 1e-3, "{worst}");
    let rotation = fitted.rotation.angle();
    assert!((rotation - 0.01 * walk).abs() < 1e-4, "{rotation}");
}

#[test]
fn a_degenerate_centreline_is_refused() {
    let p = Point3::new(1.0, 2.0, 3.0);
    for points in [
        vec![p],
        vec![p, p, Point3::new(2.0, 2.0, 3.0)],
        vec![Point3::origin(), p, p],
        vec![Point3::origin(), Point3::new(f64::NAN, 0.0, 0.0)],
    ] {
        assert!(Centreline::new(&points).is_err(), "{points:?}");
    }
}

/// A closed tube of `radius` around the planar curve from its seated tip to arc `length`, `rings` rings of 12, capped
/// at both ends by a fan to its centre.
fn closed_tube(
    lengths: &[f64],
    curvatures: &[f64],
    length: f64,
    radius: f64,
    rings: u32,
) -> IndexedMesh {
    let mut mesh = IndexedMesh::new();
    let around = 12_u32;
    for k in 0..=rings {
        let (centre, heading) = exact_planar(
            lengths,
            curvatures,
            length * f64::from(k) / f64::from(rings),
        );
        let side = Vector3::new(-heading.sin(), heading.cos(), 0.0);
        for j in 0..around {
            let angle = std::f64::consts::TAU * f64::from(j) / f64::from(around);
            mesh.vertices
                .push(centre + (side * angle.cos() + Vector3::z() * angle.sin()) * radius);
        }
    }
    let ring = |k: u32, j: u32| k * around + j % around;
    for k in 0..rings {
        for j in 0..around {
            mesh.faces
                .push([ring(k, j), ring(k + 1, j), ring(k + 1, j + 1)]);
            mesh.faces
                .push([ring(k, j), ring(k + 1, j + 1), ring(k, j + 1)]);
        }
    }
    let (tip, _) = exact_planar(lengths, curvatures, 0.0);
    let (end, _) = exact_planar(lengths, curvatures, length);
    let tip_centre = u32::try_from(mesh.vertices.len()).expect("a small mesh");
    mesh.vertices.push(tip);
    mesh.vertices.push(end);
    for j in 0..around {
        mesh.faces.push([tip_centre, ring(0, j + 1), ring(0, j)]);
        mesh.faces
            .push([tip_centre + 1, ring(rings, j), ring(rings, j + 1)]);
    }
    mesh
}

/// The device's mouth: the plane through the curve's point at arc `s`, its normal the curve's heading there (out of
/// the device, which lies toward the tip).
fn mouth(lengths: &[f64], curvatures: &[f64], s: f64) -> Plane {
    let (at, heading) = exact_planar(lengths, curvatures, s);
    Plane::new(at, Vector3::new(heading.cos(), heading.sin(), 0.0)).expect("a plane")
}

#[test]
fn on_a_straight_path_the_join_and_the_start_are_where_the_geometry_puts_them() {
    // A 60 mm tube seated at the tip of a 100 mm straight centreline, the device's mouth at its end. The fit is
    // used while the tip ring is inside, walks under 100 mm: the join is the last grid walk short of it. Past it the
    // tube only moves along x. A wall of points inside the tube's seated place, 8 mm off the axis between 20 and
    // 100 mm, clears the tube by 5 mm once the tip is 5 mm past the last of them.
    let (lengths, curvatures) = ([0.1], [0.0]);
    let centreline = Centreline::new(&planar_curve(&lengths, &curvatures, 0.005)).expect("a line");
    let scan = closed_tube(&lengths, &curvatures, 0.06, 0.01, 30);
    let path = FittedPath::new(&scan, centreline, vec![mouth(&lengths, &curvatures, 0.1)])
        .expect("a path");
    let join = path.join().expect("the tube leaves the device");
    assert!((join - (0.1 - WALK_GRID)).abs() < 1e-12, "{join}");
    for walk in [0.0, 0.03, join, join + 0.013] {
        let pose = path.pose(walk).expect("a pose");
        assert!(pose.rotation.angle() < 1e-12, "{walk}");
        assert!(
            (pose.translation.vector - Vector3::new(walk, 0.0, 0.0)).norm() < 1e-12,
            "{walk}"
        );
    }
    let wall: Vec<Point3<f64>> = (0..=40)
        .flat_map(|k| {
            let x = 0.02 + 0.002 * f64::from(k);
            [Point3::new(x, 0.008, 0.0), Point3::new(x, 0.0, -0.008)]
        })
        .collect();
    let signed = SignedDistance::new(&scan).expect("a closed tube");
    let start = path.start(&wall, &signed, 0.005).expect("a start");
    assert!(start > 0.105 - 1e-9 && start < 0.105 + WALK_GRID, "{start}");
    let clearance = |walk: f64| {
        let pose = path.pose(walk).expect("a pose");
        wall.iter()
            .map(|p| signed.signed(pose.inverse_transform_point(p)))
            .fold(f64::INFINITY, f64::min)
    };
    assert!(clearance(start) >= 0.005 && clearance(start - WALK_GRID) < 0.005);
    // A node on the axis 200 mm out, which the tube reaches as it leaves. At a clearance of 35 mm the wall behind
    // needs walks from 135 mm, and the node clears the 60 mm tube for walks up to 105 mm or from 235 mm; the search
    // ends at the join plus the tube's diameter (its box's diagonal, 66.3 mm) and the clearance, 200.8 mm.
    let mut blocked = wall.clone();
    blocked.push(Point3::new(0.2, 0.0, 0.0));
    let diameter = 0.06_f64.hypot(0.02).hypot(0.02);
    let blocked_start = path.start(&blocked, &signed, 0.035);
    assert!(
        matches!(blocked_start, Err(PathError::NoStart { searched_to })
            if (searched_to - (join + diameter + 0.035)).abs() < 1e-12),
        "{blocked_start:?}"
    );
    assert!(path.start(&wall, &signed, 0.035).is_ok());
}

#[test]
fn the_fit_uses_the_points_the_slide_puts_inside() {
    // An S: 50 at curvature +1/100, then 50 at −1/100, the mouth at its end. Walked 25, the fit is the least-squares
    // fit over the vertices the slide carries inside, weighted by their areas, which no other rigid motion beats, and
    // not the fit over them all.
    let (lengths, curvatures) = ([0.05, 0.05], [10.0, -10.0]);
    let centreline = Centreline::new(&planar_curve(&lengths, &curvatures, 0.000_1)).expect("an S");
    let scan = closed_tube(&lengths, &curvatures, 0.1, 0.008, 40);
    let device = mouth(&lengths, &curvatures, 0.1);
    let path = FittedPath::new(&scan, centreline.clone(), vec![device]).expect("a path");
    let walk = 0.025;
    let fitted = path.fitted(walk).expect("a fit");
    let areas = vertex_areas(&scan);
    let (mut from, mut to, mut weights) = (Vec::new(), Vec::new(), Vec::new());
    for (p, area) in scan.vertices.iter().zip(&areas) {
        let q = centreline.slid(*p, walk);
        if *area > 0.0 && device.height(q) < 0.0 {
            from.push(*p);
            to.push(q);
            weights.push(*area);
        }
    }
    assert!(from.len() < scan.vertices.len() && from.len() > 100);
    let direct = fitted_motion(&from, &to, &weights).expect("a fit");
    assert!(
        from.iter()
            .all(|p| (fitted * p - direct * p).norm() < 1e-15)
    );
    let every: Vec<Point3<f64>> = scan
        .vertices
        .iter()
        .map(|p| centreline.slid(*p, walk))
        .collect();
    let over_all = fitted_motion(&scan.vertices, &every, &areas).expect("a fit");
    assert!(
        from.iter()
            .any(|p| (over_all * p - fitted * p).norm() > 1e-4)
    );
    let misses = |pose: &Isometry3<f64>| -> f64 {
        from.iter()
            .zip(&to)
            .zip(&weights)
            .map(|((p, q), w)| w * (pose * p - q).norm_squared())
            .sum()
    };
    assert!(misses(&fitted) < misses(&over_all));
}

#[test]
fn the_sampling_meets_its_bar_where_it_did_not_read() {
    // A 130 mm tube seated on a planar arc of curvature 10 /m, the mouth at 100 mm, so part of it always lies outside:
    // the fitted pose turns as it walks. At a bar of a micrometre, 64 intervals are not enough. The sampled path is
    // read at 16 times in every interval, none of them the quarters the sampling read, against the path itself.
    let (lengths, curvatures) = ([0.25], [10.0]);
    let centreline =
        Centreline::new(&planar_curve(&lengths, &curvatures, 0.000_1)).expect("an arc");
    let scan = closed_tube(&lengths, &curvatures, 0.13, 0.01, 65);
    let path = FittedPath::new(&scan, centreline, vec![mouth(&lengths, &curvatures, 0.1)])
        .expect("a path");
    let (start, loading, bar) = (0.07, 0.5, 1e-6);
    let sampled = path.sampled(start, loading, bar).expect("a sampling");
    assert!(sampled.errors.len() > 1, "{:?}", sampled.errors);
    assert!(sampled.errors.last().expect("an error").1 <= bar);
    let count = u32::try_from(sampled.poses.len()).expect("a count");
    assert!((sampled.interval * f64::from(count - 1) / loading - 1.0).abs() < 1e-15);
    let areas = vertex_areas(&scan);
    let mut worst = 0.0_f64;
    for k in 0..(count - 1) * 16 {
        let time = (f64::from(k) + 0.5) / 16.0 * sampled.interval;
        let around = pose_sample_span(time, 0.0, sampled.interval, count);
        let between = pose_interpolate(
            sampled.poses[around.lower as usize],
            sampled.poses[around.upper as usize],
            around.fraction,
        );
        let walk = start - travelled(time / loading, start);
        let on_path = path.pose(walk).expect("a pose");
        for (p, area) in scan.vertices.iter().zip(&areas) {
            let q = on_path * p;
            if *area > 0.0 && path.inside(q) {
                let off = Point3::from(pose_to_world(between, [p.x, p.y, p.z]));
                worst = worst.max((off - q).norm());
            }
        }
    }
    assert!(worst <= 1.01 * bar, "{worst} against a bar of {bar}");
    assert!(
        worst > 0.1 * bar,
        "the reading must see the interpolation: {worst}"
    );

    // The first count's error is the farthest stray at its quarters over the vertices inside the device; over every
    // vertex, the part of the tube outside the mouth reads farther (by 0.7 % here, far past the comparison's 1e-9).
    let first = sampled.errors[0];
    let as_pose = |walk: f64| {
        let pose = path.pose(walk).expect("a pose");
        let (q, t) = (pose.rotation, pose.translation.vector);
        sim_soft_explicit::f64::Pose {
            qw: q.w,
            qx: q.i,
            qy: q.j,
            qz: q.k,
            tx: t.x,
            ty: t.y,
            tz: t.z,
        }
    };
    let at = |u: f64| start - travelled(u, start);
    let intervals = f64::from(first.0);
    let (mut inside, mut every) = (0.0_f64, 0.0_f64);
    for k in 0..first.0 {
        let (a, b) = (
            as_pose(at(f64::from(k) / intervals)),
            as_pose(at(f64::from(k + 1) / intervals)),
        );
        for quarter in 1..=3 {
            let fraction = f64::from(quarter) / 4.0;
            let on_path = path
                .pose(at((f64::from(k) + fraction) / intervals))
                .expect("a pose");
            let between = pose_interpolate(a, b, fraction);
            for (p, area) in scan.vertices.iter().zip(&areas) {
                if *area > 0.0 {
                    let q = on_path * p;
                    let off = (Point3::from(pose_to_world(between, [p.x, p.y, p.z])) - q).norm();
                    every = every.max(off);
                    if path.inside(q) {
                        inside = inside.max(off);
                    }
                }
            }
        }
    }
    assert!(
        (first.1 / inside - 1.0).abs() < 1e-9,
        "{} against {inside}",
        first.1
    );
    assert!(
        every > (1.0 + 1e-6) * inside,
        "the fixture must put a farther stray outside the device: {every} against {inside}"
    );
}

#[test]
fn a_bar_the_sampling_cannot_meet_fails_at_the_cap() {
    let (lengths, curvatures) = ([0.1], [10.0]);
    let centreline = Centreline::new(&planar_curve(&lengths, &curvatures, 0.001)).expect("an arc");
    let scan = closed_tube(&lengths, &curvatures, 0.06, 0.01, 10);
    let path = FittedPath::new(&scan, centreline, vec![mouth(&lengths, &curvatures, 0.1)])
        .expect("a path");
    let too_fine = path.sampled(0.07, 0.5, 1e-15);
    assert!(
        matches!(too_fine, Err(PathError::TooCoarse { intervals, error })
            if intervals == MOST_INTERVALS && error > 1e-15),
        "{too_fine:?}"
    );
    assert_eq!(
        path.sampled(0.07, 0.0, 1e-6).err(),
        Some(PathError::Parameters)
    );
    assert_eq!(
        path.sampled(-0.01, 0.5, 1e-6).err(),
        Some(PathError::Parameters)
    );
}

#[test]
fn the_time_profile_is_the_tubes() {
    let travel = 0.137;
    let insertion = Insertion {
        start_gap: 0.0,
        depth: travel,
        loading_time: 1.0,
        hold: 0.2,
    };
    for k in 0..=1000 {
        let u = f64::from(k) / 1000.0;
        let expected = insertion.tip(u);
        assert!(
            (travelled(u, travel) - expected).abs() <= 1e-15 * travel,
            "{u}"
        );
    }
    assert!((travelled(1.0, travel) - travel).abs() <= 1e-16);
}

#[test]
fn a_scan_outside_the_device_has_no_fit_at_the_seat() {
    let (lengths, curvatures) = ([0.1], [0.0]);
    let centreline = Centreline::new(&planar_curve(&lengths, &curvatures, 0.005)).expect("a line");
    let scan = closed_tube(&lengths, &curvatures, 0.06, 0.01, 10);
    let behind = Plane::new(Point3::new(-0.01, 0.0, 0.0), Vector3::x()).expect("a plane");
    assert_eq!(
        FittedPath::new(&scan, centreline, vec![behind]).err(),
        Some(PathError::NoFitAtTheSeat)
    );
}

#[test]
fn the_obstacle_rides_the_sampled_path() {
    let (lengths, curvatures) = ([0.25], [10.0]);
    let centreline = Centreline::new(&planar_curve(&lengths, &curvatures, 0.001)).expect("an arc");
    let scan = closed_tube(&lengths, &curvatures, 0.13, 0.01, 20);
    let path = FittedPath::new(&scan, centreline, vec![mouth(&lengths, &curvatures, 0.1)])
        .expect("a path");
    let sampled = path.sampled(0.07, 0.5, 1e-4).expect("a sampling");
    let bake = sim_soft::obstacle::ObstacleBake {
        coarse_cell: 0.004,
        fine_cell: 0.002,
        band: 0.002,
        margin: 0.008,
    };
    let baked = sim_soft::obstacle::bake_obstacle(&scan, bake).expect("a bake");
    let obstacle = sampled.obstacle(baked.clone(), 0.4);
    assert_eq!(
        (obstacle.start, obstacle.interval, obstacle.friction),
        (0.0, sampled.interval, 0.4)
    );
    assert_eq!(obstacle.poses, sampled.poses);
    assert_eq!(obstacle.values, baked.values);
    assert_eq!(
        obstacle.fine.as_ref().map(|fine| &fine.values),
        Some(&baked.fine.values)
    );
    // Held at the seat after the loading: the identity, to the fit's rounding.
    let seat = obstacle.pose_at(10.0);
    assert!((seat.qw - 1.0).abs() < 1e-12 && seat.tx.hypot(seat.ty).hypot(seat.tz) < 1e-12);
}

#[test]
fn the_join_is_found_past_the_centrelines_end() {
    // A 50 mm centreline and a 60 mm tube, the mouth 80 mm out: the tip leaves the device 30 mm past the centreline's
    // end, where the slide carries on along its last segment.
    let (lengths, curvatures) = ([0.08], [0.0]);
    let centreline = Centreline::new(&planar_curve(&[0.05], &[0.0], 0.005)).expect("a line");
    let scan = closed_tube(&lengths, &curvatures, 0.06, 0.01, 30);
    let path = FittedPath::new(&scan, centreline, vec![mouth(&lengths, &curvatures, 0.08)])
        .expect("a path");
    let join = path.join().expect("the tube leaves the device");
    assert!((join - (0.08 - WALK_GRID)).abs() < 1e-12, "{join}");
    let pose = path.pose(join).expect("a pose");
    assert!((pose.translation.vector - Vector3::new(join, 0.0, 0.0)).norm() < 1e-12);
}

#[test]
fn past_the_join_the_pose_keeps_its_rotation_and_moves_along_the_centreline() {
    // On an arc the fitted pose turns up to the join; past it, it keeps the join's rotation and moves along the
    // centreline's direction at the join.
    let (lengths, curvatures) = ([0.25], [10.0]);
    let centreline =
        Centreline::new(&planar_curve(&lengths, &curvatures, 0.000_1)).expect("an arc");
    let scan = closed_tube(&lengths, &curvatures, 0.13, 0.01, 65);
    let path = FittedPath::new(
        &scan,
        centreline.clone(),
        vec![mouth(&lengths, &curvatures, 0.1)],
    )
    .expect("a path");
    let join = path.join().expect("the tube leaves the device");
    let (at, past) = (
        path.pose(join).expect("a pose"),
        path.pose(join + 0.01).expect("a pose"),
    );
    assert!(at.rotation.angle_to(&past.rotation) < 1e-12);
    let moved = past.translation.vector - at.translation.vector;
    let direction = centreline.tangent(join);
    assert!((moved - direction * 0.01).norm() < 1e-12, "{moved}");
    assert!(direction.y.abs() > 0.5, "the fixture must turn by the join");
}

#[test]
fn the_start_is_searched_from_the_join() {
    // A wall 3 mm outside the tube near its seat clears it by 2 mm from the seat on; the start is still the join.
    let (lengths, curvatures) = ([0.1], [0.0]);
    let centreline = Centreline::new(&planar_curve(&lengths, &curvatures, 0.005)).expect("a line");
    let scan = closed_tube(&lengths, &curvatures, 0.06, 0.01, 30);
    let path = FittedPath::new(&scan, centreline, vec![mouth(&lengths, &curvatures, 0.1)])
        .expect("a path");
    let wall: Vec<Point3<f64>> = (0..=10)
        .map(|k| Point3::new(0.02 + 0.002 * f64::from(k), 0.013, 0.0))
        .collect();
    let signed = SignedDistance::new(&scan).expect("a closed tube");
    assert!(
        wall.iter().all(|p| signed.signed(*p) >= 0.002),
        "clear at the seat"
    );
    let join = path.join().expect("the tube leaves the device");
    assert_eq!(path.start(&wall, &signed, 0.002), Ok(join));
    for clearance in [f64::NAN, f64::INFINITY, -0.001] {
        assert_eq!(
            path.start(&wall, &signed, clearance),
            Err(PathError::Parameters)
        );
    }
}

#[test]
fn points_on_one_line_do_not_fit() {
    // A flat ribbon along the centreline, 4 mm wide: with the device's side plane through its middle only one edge
    // of vertices lies inside, on a line, and the fit is not used; with the plane past it, both edges are, and it is.
    let centreline = Centreline::new(&planar_curve(&[0.1], &[0.0], 0.005)).expect("a line");
    let mut ribbon = IndexedMesh::new();
    for k in 0..=30 {
        let x = 0.002 * f64::from(k);
        ribbon.vertices.push(Point3::new(x, 0.0, 0.0));
        ribbon.vertices.push(Point3::new(x, 0.004, 0.0));
    }
    for k in 0..30 {
        let (a, b) = (2 * k, 2 * k + 1);
        ribbon.faces.push([a, a + 2, b]);
        ribbon.faces.push([b, a + 2, b + 2]);
    }
    let side = |y: f64| Plane::new(Point3::new(0.0, y, 0.0), Vector3::y()).expect("a plane");
    assert_eq!(
        FittedPath::new(&ribbon, centreline.clone(), vec![side(0.002)]).err(),
        Some(PathError::NoFitAtTheSeat)
    );
    assert!(FittedPath::new(&ribbon, centreline, vec![side(0.008)]).is_ok());
}

#[test]
fn a_short_segment_turns_the_tangent_as_its_direction_does() {
    // Ten 10 mm segments along x. A trim leaves short segments on the line at its ends: they turn nothing. A point
    // repeated a nanometre to one side makes a segment whose direction weighs as much as a long one's, and the tangent
    // at its ends turns 45°; the centreline is taken as given.
    let line: Vec<Point3<f64>> = (0..=10)
        .map(|k| Point3::new(0.01 * f64::from(k), 0.0, 0.0))
        .collect();
    let mut trimmed = line.clone();
    trimmed.insert(1, Point3::new(1e-9, 0.0, 0.0));
    trimmed.insert(trimmed.len() - 1, Point3::new(0.1 - 1e-9, 0.0, 0.0));
    let trimmed = Centreline::new(&trimmed).expect("a trimmed line");
    for s in [0.0, 1e-9, 0.05, 0.1] {
        assert!((trimmed.tangent(s) - Vector3::x()).norm() < 1e-12, "{s}");
    }
    let mut nudged = line;
    nudged.insert(6, Point3::new(0.05, 1e-9, 0.0));
    let nudged = Centreline::new(&nudged).expect("a nudged line");
    let tilt = nudged.tangent(0.05).angle(&Vector3::x()).to_degrees();
    assert!((tilt - 45.0).abs() < 1e-3, "{tilt}");
}
