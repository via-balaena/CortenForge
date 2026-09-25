//! Time integration, the obstacle's pose, and the contact law.

#![allow(clippy::unwrap_used, clippy::float_cmp)]

use sim_soft_explicit::f64 as shared;
use sim_soft_explicit::f64::{Pose, SdfSample};

const IDENTITY: Pose = Pose {
    qw: 1.0,
    qx: 0.0,
    qy: 0.0,
    qz: 0.0,
    tx: 0.0,
    ty: 0.0,
    tz: 0.0,
};

/// A rotation by `angle` about `axis` (unit), then a translation.
fn pose(angle: f64, axis: [f64; 3], translation: [f64; 3]) -> Pose {
    let (s, c) = (0.5 * angle).sin_cos();
    Pose {
        qw: c,
        qx: s * axis[0],
        qy: s * axis[1],
        qz: s * axis[2],
        tx: translation[0],
        ty: translation[1],
        tz: translation[2],
    }
}

fn close(a: [f64; 3], b: [f64; 3], tolerance: f64) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() <= tolerance)
}

// ---- time integration ----

#[test]
fn an_undamped_free_node_keeps_its_velocity() {
    let v = shared::advance_velocity([1.0, -2.0, 0.5], [0.0; 3], 1.0 / 3.0, 0.0, 0.01);
    assert_eq!(v, [1.0, -2.0, 0.5]);
    assert_eq!(
        shared::advance_displacement([1.0, 1.0, 1.0], v, 0.1),
        [1.1, 0.8, 1.05]
    );
}

#[test]
fn a_constant_force_accelerates_at_f_over_m() {
    let (mass, dt) = (2.0, 0.01);
    let v = shared::advance_velocity(
        [0.0; 3],
        [4.0, 0.0, -1.0],
        shared::inverse_mass(mass, false),
        0.0,
        dt,
    );
    assert!(close(v, [4.0 / mass * dt, 0.0, -1.0 / mass * dt], 1e-15));
}

#[test]
fn damping_decays_velocity_by_the_central_difference_factor() {
    let (alpha, dt) = (30.0, 0.002);
    let v = shared::advance_velocity([1.0, 0.0, 0.0], [0.0; 3], 1.0, alpha, dt);
    let factor = (1.0 - 0.5 * alpha * dt) / (1.0 + 0.5 * alpha * dt);
    assert!((v[0] - factor).abs() < 1e-15);
}

#[test]
fn a_held_node_does_not_move() {
    let inverse = shared::inverse_mass(2.0, true);
    assert_eq!(inverse, 0.0);
    let v = shared::advance_velocity([0.0; 3], [1e6, -1e6, 3.0], inverse, 5.0, 0.01);
    assert_eq!(v, [0.0; 3]);
    assert_eq!(shared::kinetic_energy(2.0, [3.0, 4.0, 0.0]), 25.0);
}

// ---- pose ----

#[test]
fn a_quarter_turn_about_z_maps_x_to_y() {
    let p = pose(
        std::f64::consts::FRAC_PI_2,
        [0.0, 0.0, 1.0],
        [1.0, 2.0, 3.0],
    );
    assert!(close(
        shared::pose_rotate(p, [1.0, 0.0, 0.0]),
        [0.0, 1.0, 0.0],
        1e-15
    ));
    assert!(close(
        shared::pose_to_world(p, [1.0, 0.0, 0.0]),
        [1.0, 3.0, 3.0],
        1e-15
    ));
}

#[test]
fn to_body_undoes_to_world() {
    let p = pose(1.3, [0.6, 0.0, 0.8], [0.1, -0.2, 0.3]);
    let point = [0.7, -1.1, 0.4];
    assert!(close(
        shared::pose_to_body(p, shared::pose_to_world(p, point)),
        point,
        1e-15
    ));
    assert!(close(
        shared::pose_unrotate(p, shared::pose_rotate(p, point)),
        point,
        1e-15
    ));
}

#[test]
fn interpolation_hits_the_ends_and_halves_the_angle() {
    let axis = [0.0, 0.6, 0.8];
    let a = pose(0.2, axis, [0.0, 0.0, 0.0]);
    let b = pose(1.0, axis, [2.0, -4.0, 6.0]);
    let start = shared::pose_interpolate(a, b, 0.0);
    let end = shared::pose_interpolate(a, b, 1.0);
    let middle = shared::pose_interpolate(a, b, 0.5);
    let expected = pose(0.6, axis, [1.0, -2.0, 3.0]);
    for (got, want) in [(start, a), (end, b), (middle, expected)] {
        assert!(
            (got.qw - want.qw).abs() < 1e-14
                && (got.qy - want.qy).abs() < 1e-14
                && (got.qz - want.qz).abs() < 1e-14
        );
        assert!(close(
            [got.tx, got.ty, got.tz],
            [want.tx, want.ty, want.tz],
            1e-14
        ));
    }
}

#[test]
fn interpolation_takes_the_shorter_arc_and_handles_equal_rotations() {
    let axis = [1.0, 0.0, 0.0];
    let a = pose(0.2, axis, [0.0; 3]);
    let b = pose(0.6, axis, [0.0; 3]);
    // The same rotation as `b`, written with the opposite sign.
    let b_negated = Pose {
        qw: -b.qw,
        qx: -b.qx,
        qy: -b.qy,
        qz: -b.qz,
        ..b
    };
    let v = [0.0, 1.0, 0.0];
    for s in [0.25, 0.5, 0.8] {
        let direct = shared::pose_interpolate(a, b, s);
        let via_negated = shared::pose_interpolate(a, b_negated, s);
        assert!(close(
            shared::pose_rotate(direct, v),
            shared::pose_rotate(via_negated, v),
            1e-14
        ));
    }
    let same = shared::pose_interpolate(a, a, 0.3);
    assert!((same.qw - a.qw).abs() < 1e-15 && (same.qx - a.qx).abs() < 1e-15);
}

/// The rotation angle the interpolation reaches at `s`, between rotations
/// `step` apart about one axis, against the angle `s · step` that spherical
/// interpolation reaches: the largest gap over `s`.
fn interpolation_gap(step: f64) -> f64 {
    let axis = [0.0, 0.6, 0.8];
    let (a, b) = (pose(0.0, axis, [0.0; 3]), pose(step, axis, [0.0; 3]));
    (1..100)
        .map(|i| {
            let s = f64::from(i) / 100.0;
            let q = shared::pose_interpolate(a, b, s);
            let angle = 2.0 * q.qx.hypot(q.qy).hypot(q.qz).atan2(q.qw);
            (angle - s * step).abs()
        })
        .fold(0.0, f64::max)
}

#[test]
fn interpolation_stays_close_to_spherical() {
    let (gap_small, gap_large) = (interpolation_gap(0.1), interpolation_gap(0.2));
    eprintln!(
        "MARGIN interpolation gap: {gap_small:e} rad at a 0.1 rad step, {gap_large:e} at 0.2"
    );
    // Measured 4.0e-6 rad at a 0.1 rad step.
    assert!(gap_small < 1e-5, "{gap_small:e}");
    // Doubling the step multiplies the gap by about 2³.
    let ratio = gap_large / gap_small;
    assert!((7.5..8.5).contains(&ratio), "ratio {ratio}");
}

#[test]
fn a_sample_span_clamps_and_splits_time() {
    let span = shared::pose_sample_span(0.25, 0.0, 0.1, 5);
    assert_eq!((span.lower, span.upper), (2, 3));
    assert!((span.fraction - 0.5).abs() < 1e-12);
    let before = shared::pose_sample_span(-1.0, 0.0, 0.1, 5);
    assert_eq!((before.lower, before.upper, before.fraction), (0, 1, 0.0));
    let after = shared::pose_sample_span(9.0, 0.0, 0.1, 5);
    assert_eq!((after.lower, after.upper, after.fraction), (4, 4, 0.0));
    let only = shared::pose_sample_span(0.3, 0.0, 0.1, 1);
    assert_eq!((only.lower, only.upper, only.fraction), (0, 0, 0.0));
    let none = shared::pose_sample_span(0.3, 0.0, 0.1, 0);
    assert_eq!((none.lower, none.upper), (0, 0));
}

// ---- contact ----

const K: f64 = 1000.0;

const fn surface(distance: f64) -> SdfSample {
    SdfSample {
        distance,
        normal: [0.0, 0.0, 1.0],
    }
}

#[test]
fn out_of_contact_there_is_no_force_and_the_anchor_follows() {
    let point = [0.1, 0.2, 0.3];
    let r = shared::obstacle_contact(IDENTITY, point, surface(0.01), [5.0, 5.0, 5.0], K, 0.3);
    assert_eq!(r.force, [0.0; 3]);
    assert_eq!(r.normal_force, 0.0);
    assert_eq!(r.anchor, point);
}

#[test]
fn penetration_pushes_out_along_the_normal() {
    let point = [0.1, 0.2, 0.3];
    let r = shared::obstacle_contact(IDENTITY, point, surface(-0.002), point, K, 0.3);
    assert!(close(r.force, [0.0, 0.0, K * 0.002], 1e-15));
    assert_eq!(r.normal_force, K * 0.002);
    assert_eq!(r.anchor, point);
}

#[test]
fn a_small_slip_sticks_and_a_large_one_slides_at_the_coulomb_limit() {
    let (friction, depth) = (0.3, 0.002);
    let limit = friction * K * depth;
    let anchor = [0.0, 0.0, 0.0];
    // Within the cone: pulled back elastically, anchor kept.
    let small = [0.4 * limit / K, 0.0, 0.0];
    let r = shared::obstacle_contact(IDENTITY, small, surface(-depth), anchor, K, friction);
    assert!(close(r.force, [-K * small[0], 0.0, K * depth], 1e-15));
    assert_eq!(r.anchor, anchor);
    // Past it: the pull is exactly μ f_n, opposite the slip, and the anchor is
    // dragged to within μ f_n / k of the node.
    let large = [3.0 * limit / K, 4.0 * limit / K, 0.0];
    let r = shared::obstacle_contact(IDENTITY, large, surface(-depth), anchor, K, friction);
    assert!(close(
        r.force,
        [-0.6 * limit, -0.8 * limit, K * depth],
        1e-15
    ));
    let offset = [large[0] - r.anchor[0], large[1] - r.anchor[1]];
    assert!((offset[0].hypot(offset[1]) - limit / K).abs() < 1e-15);
    // Frictionless: no tangential force, and the anchor tracks the node.
    let r = shared::obstacle_contact(IDENTITY, large, surface(-depth), anchor, K, 0.0);
    assert!(close(r.force, [0.0, 0.0, K * depth], 1e-15));
    assert!(close(r.anchor, large, 1e-15));
}

#[test]
fn the_force_is_returned_in_the_world_frame() {
    let p = pose(
        std::f64::consts::FRAC_PI_2,
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    );
    let point = [0.0, 0.0, 0.0];
    let r = shared::obstacle_contact(p, point, surface(-0.001), point, K, 0.3);
    // A quarter turn about x takes the body's +z normal to world −y.
    assert!(close(r.force, [0.0, -K * 0.001, 0.0], 1e-15));
}

#[test]
fn only_the_tangential_part_of_a_slip_is_resisted() {
    // A tilted surface, and a slip with a normal part: the friction force
    // is along the tangential part alone.
    let normal = [0.0, 0.6, 0.8];
    let sample = SdfSample {
        distance: -0.002,
        normal,
    };
    let slip = [0.0001, 0.0002, 0.0003];
    let normal_part = shared::vec3_dot(slip, normal);
    let tangential = shared::vec3_sub(slip, shared::vec3_scale(normal, normal_part));
    let r = shared::obstacle_contact(IDENTITY, slip, sample, [0.0; 3], K, 1.0);
    let expected = shared::vec3_add(
        shared::vec3_scale(normal, K * 0.002),
        shared::vec3_scale(tangential, -K),
    );
    assert!(
        close(r.force, expected, 1e-15),
        "{:?} against {expected:?}",
        r.force
    );
    assert!(
        shared::vec3_dot(
            shared::vec3_sub(r.force, shared::vec3_scale(normal, K * 0.002)),
            normal
        )
        .abs()
            < 1e-15
    );
}

#[test]
fn a_node_carried_by_the_obstacle_feels_no_friction() {
    // The anchor lives in the body frame: a node that moves with a rotating,
    // translating obstacle stays stuck with no tangential force.
    let before = pose(0.3, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]);
    let after = pose(0.5, [0.0, 0.0, 1.0], [0.01, -0.02, 0.0]);
    let body_point = [0.1, 0.05, 0.0];
    let sample = surface(-0.001);
    let first = shared::obstacle_contact(before, body_point, sample, body_point, K, 0.5);
    let world = shared::pose_to_world(after, body_point);
    let second = shared::obstacle_contact(
        after,
        shared::pose_to_body(after, world),
        sample,
        first.anchor,
        K,
        0.5,
    );
    let normal_world = shared::pose_rotate(after, [0.0, 0.0, K * 0.001]);
    assert!(
        close(second.force, normal_world, 1e-12),
        "{:?}",
        second.force
    );
    // The same node held still in the world while the obstacle turns under
    // it is pulled along, at most at the Coulomb limit.
    let held = shared::pose_to_world(before, body_point);
    let dragged = shared::obstacle_contact(
        after,
        shared::pose_to_body(after, held),
        sample,
        first.anchor,
        K,
        0.5,
    );
    let tangential = shared::vec3_sub(dragged.force, normal_world);
    let magnitude = shared::vec3_length(tangential);
    assert!(magnitude > 0.0 && magnitude <= 0.5 * K * 0.001 * (1.0 + 1e-12));
}

#[test]
fn the_penalty_stiffness_scales_with_mass_over_step_squared() {
    assert_eq!(
        shared::penalty_stiffness(2.0, 0.01, 0.5),
        0.5 * 2.0 / (0.01 * 0.01)
    );
}
