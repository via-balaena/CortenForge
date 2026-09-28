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
//! The slide and the fitted pose are `sim_soft::lowering::path`'s, where the lowering builds step 7's path (plan
//! §16w); this probe measured them first with a copy of its own, and reads the same room through them.
//!
//! ⛔ The scan never enters the repo. [`why_the_rigid_path_asks_room_on_the_product_scan`] prints to the
//! terminal, and the plan records only its ratios and verdict (Jon, 2026-09-26).

#![cfg(test)]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

use cf_design::Sdf as _;
use nalgebra::{Isometry3, Point3, Rotation3, Vector3};
use sim_soft::lowering::Plane;
use sim_soft::lowering::path::{Centreline, FittedPath, fitted_motion, vertex_areas};

use super::{Solid, TransformedSdf, slide_pose_at};

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
fn on_a_straight_centreline_the_three_motions_agree_and_a_taper_asks_its_own_room() {
    // A cone around the x axis narrowing away from the tip: radius 10 at the tip, 1 in 20 per unit of arc. Its
    // exact distance is the radial excess times the cosine of its half-angle.
    let taper = 0.05;
    let cone = move |p: Point3<f64>| (p.y.hypot(p.z) - (10.0 - taper * p.x)) / taper.hypot(1.0);
    let centerline: Vec<Point3<f64>> = (0..=10)
        .map(|k| Point3::new(10.0 * f64::from(k), 0.0, 0.0))
        .collect();
    let track = Centreline::new(&centerline).unwrap();
    let surface = tube_surface(&[100.0], &[0.0], 8.0, 50);
    let walk = 30.0;
    let (from, to): (Vec<_>, Vec<_>) = surface
        .iter()
        .map(|&p| (p, track.slid(p, walk)))
        .filter(|&(_, q)| q.x <= 100.0)
        .unzip();
    let fitted = fitted_motion(&from, &to, &vec![1.0; from.len()]).unwrap();
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
    let track = Centreline::new(&centerline).unwrap();
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
    let coarse_tip = slide_pose_at(
        &coarse,
        1.0 - walk / Centreline::new(&coarse).unwrap().length(),
    );
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
    let track = Centreline::new(&centerline).unwrap();
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
    let track = Centreline::new(&centerline).unwrap();
    let length = track.length();
    let areas = vertex_areas(&scan);
    let device: Vec<Plane> = caps
        .iter()
        .map(|cap| Plane::new(cap.centroid, cap.normal).unwrap())
        .collect();
    let path = FittedPath::new(&scan, track.clone(), device).unwrap();
    let inside = |q: Point3<f64>| path.inside(q);
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
        let fitted_pose = path.fitted(walk).unwrap();
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
