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

#[test]
fn a_constraint_removes_only_its_directions() {
    let v = [1.0, 2.0, 3.0];
    let none = [0.0; 3];
    assert_eq!(shared::constrain(v, none, none), v);
    assert_eq!(shared::constrain(v, [0.0, 0.0, 1.0], none), [1.0, 2.0, 0.0]);
    // A radial direction and an axial hold leave only the circumferential part.
    let radial = [0.6, 0.8, 0.0];
    let axial = [0.0, 0.0, 1.0];
    let held = shared::constrain(v, radial, axial);
    assert!(shared::vec3_dot(held, radial).abs() < 1e-15 && held[2] == 0.0);
    let circumferential = [-0.8, 0.6, 0.0];
    assert!(
        (shared::vec3_dot(held, circumferential) - shared::vec3_dot(v, circumferential)).abs()
            < 1e-15
    );
    assert!(close(shared::constrain(held, radial, axial), held, 1e-15));
}

#[test]
fn step_work_and_damping_loss_balance_the_kinetic_energy_exactly() {
    // Damped and constrained: ½ m |v⁺|² − ½ m |v⁻|² = step_work − damping_loss.
    let (mass, alpha, dt) = (0.003, 40.0, 1e-4);
    let (first, second) = ([0.6, 0.8, 0.0], [0.0; 3]);
    let previous = shared::constrain([0.2, -0.1, 0.3], first, second);
    let force = [1.5, -0.7, 2.0];
    let velocity = shared::constrain(
        shared::advance_velocity(previous, force, 1.0 / mass, alpha, dt),
        first,
        second,
    );
    let (before, after) = (
        shared::kinetic_energy(mass, previous),
        shared::kinetic_energy(mass, velocity),
    );
    let balance = shared::step_work(force, previous, velocity, dt)
        - shared::damping_loss(mass, alpha, previous, velocity, dt);
    assert!(
        (after - before - balance).abs() <= 1e-13 * before.max(after),
        "{} against {balance}",
        after - before
    );
    // Undamped, the work alone.
    let free = shared::advance_velocity(previous, force, 1.0 / mass, 0.0, dt);
    let change = shared::kinetic_energy(mass, free) - before;
    let work = shared::step_work(force, previous, free, dt);
    assert!((change - work).abs() <= 1e-13 * before.max(change.abs()));
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

// ---- contact: the kinematic law (plan §16o) ----

const K: f64 = 1000.0;
const FREE: [[f64; 3]; 2] = [[0.0; 3]; 2];

const fn surface(distance: f64) -> SdfSample {
    SdfSample {
        distance,
        normal: [0.0, 0.0, 1.0],
    }
}

#[test]
fn out_of_contact_there_is_no_force_and_the_anchor_follows() {
    let predicted = [0.1, 0.2, 0.3];
    let r = shared::kinematic_contact(IDENTITY, predicted, surface(0.01), [5.0; 3], K, 0.3, FREE);
    assert_eq!(r.force, [0.0; 3]);
    assert_eq!(r.normal_force, 0.0);
    assert_eq!(r.anchor, predicted);
    // In contact but frictionless: no tangential force, and the anchor goes
    // where the node lands.
    let r = shared::kinematic_contact(IDENTITY, predicted, surface(-0.002), [5.0; 3], K, 0.0, FREE);
    assert!(close(r.force, [0.0, 0.0, K * 0.002], 1e-15));
    assert!(close(r.anchor, [0.1, 0.2, 0.302], 1e-15));
}

#[test]
fn the_force_is_returned_in_the_world_frame() {
    let p = pose(
        std::f64::consts::FRAC_PI_2,
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    );
    let body = [0.0, 0.0, -0.001];
    let predicted = shared::pose_to_world(p, body);
    let r = shared::kinematic_contact(p, predicted, surface(-0.001), body, K, 0.3, FREE);
    // A quarter turn about x takes the body's +z normal to world −y.
    assert!(close(r.force, [0.0, -K * 0.001, 0.0], 1e-15));
}

#[test]
fn a_node_carried_by_the_obstacle_feels_no_friction() {
    // The anchor lives in the body frame: a node that moves with a rotating,
    // translating obstacle stays stuck with no tangential force.
    let before = pose(0.3, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]);
    let after = pose(0.5, [0.0, 0.0, 1.0], [0.01, -0.02, 0.0]);
    let body = [0.1, 0.05, -0.001];
    let sample = surface(-0.001);
    let first = shared::kinematic_contact(
        before,
        shared::pose_to_world(before, body),
        sample,
        [0.1, 0.05, 0.0],
        K,
        0.5,
        FREE,
    );
    let carried = shared::kinematic_contact(
        after,
        shared::pose_to_world(after, body),
        sample,
        first.anchor,
        K,
        0.5,
        FREE,
    );
    let normal_world = shared::pose_rotate(after, [0.0, 0.0, K * 0.001]);
    assert!(
        close(carried.force, normal_world, 1e-12),
        "{:?}",
        carried.force
    );
    // The same node held still in the world while the obstacle turns under
    // it is dragged along, at the Coulomb limit.
    let dragged = shared::kinematic_contact(
        after,
        shared::pose_to_world(before, body),
        sample,
        first.anchor,
        K,
        0.5,
        FREE,
    );
    let magnitude = shared::vec3_length(dragged.friction);
    assert!((magnitude - 0.5 * dragged.normal_force).abs() <= 1e-12 * magnitude);
    // Slipping, its anchor is where it lands, in the obstacle's frame.
    let landed = shared::vec3_add(
        shared::pose_to_world(before, body),
        shared::vec3_scale(dragged.force, 1.0 / K),
    );
    assert!(close(
        dragged.anchor,
        shared::pose_to_body(after, landed),
        1e-15
    ));
}

#[test]
fn the_friction_force_is_the_tangential_part_and_reaches_the_cone_only_when_slipping() {
    let (friction, depth) = (0.3, 0.002);
    let limit = friction * depth;
    let p = pose(0.4, [0.0, 1.0, 0.0], [0.01, 0.0, 0.0]);
    let normal_world = shared::pose_rotate(p, [0.0, 0.0, 1.0]);
    for (slip, ratio) in [(0.4, 0.4), (3.0, 1.0)] {
        let predicted = shared::pose_to_world(p, [slip * limit, 0.0, -depth]);
        let r =
            shared::kinematic_contact(p, predicted, surface(-depth), [0.0; 3], K, friction, FREE);
        let normal_part = shared::vec3_scale(normal_world, r.normal_force);
        assert!(close(
            shared::vec3_add(normal_part, r.friction),
            r.force,
            1e-12
        ));
        assert!(shared::vec3_dot(r.friction, normal_world).abs() < 1e-12);
        let read = shared::vec3_length(r.friction) / (friction * r.normal_force);
        assert!((read - ratio).abs() < 1e-9, "slip {slip}: ratio {read}");
    }
}

#[test]
fn the_kinematic_force_puts_the_node_on_the_surface_in_one_step() {
    let (mass, damping, dt) = (2.0e-3, 6.0, 8.0e-5);
    let k = shared::kinematic_stiffness(mass, 1.0 / mass, damping, dt);
    let depth = 3.0e-5;
    let predicted = [0.01, 0.02, -depth];
    let r = shared::kinematic_contact(
        IDENTITY,
        predicted,
        surface(-depth),
        predicted,
        k,
        0.0,
        FREE,
    );
    // The force, applied over the step through the damped update, moves the
    // node by exactly the depth along the normal.
    let moved = shared::advance_velocity([0.0; 3], r.force, 1.0 / mass, damping, dt);
    assert!(close(
        shared::vec3_scale(moved, dt),
        [0.0, 0.0, depth],
        1e-18
    ));
    assert!((r.normal_force - k * depth).abs() <= 1e-12 * k * depth);
    // A held node is left alone.
    let held = shared::kinematic_stiffness(mass, 0.0, damping, dt);
    let r = shared::kinematic_contact(
        IDENTITY,
        predicted,
        surface(-depth),
        predicted,
        held,
        0.3,
        FREE,
    );
    assert_eq!((r.force, r.normal_force), ([0.0; 3], 0.0));
}

#[test]
fn a_constrained_node_is_moved_in_its_free_directions_onto_the_surface() {
    // A plane tilted 30° about y; the node may not move along x.
    let (s, c) = (0.5_f64, 0.75_f64.sqrt());
    let normal = [s, 0.0, c];
    let depth = 1.0e-4;
    let predicted = [0.0, 0.0, -depth / c];
    let sample = SdfSample {
        distance: -depth,
        normal,
    };
    let only_yz = [[1.0, 0.0, 0.0], [0.0; 3]];
    let r = shared::kinematic_contact(IDENTITY, predicted, sample, predicted, K, 0.0, only_yz);
    let step = shared::vec3_scale(r.force, 1.0 / K);
    assert_eq!(step[0], 0.0);
    // The plane's distance at the moved point is zero.
    assert!((shared::vec3_dot(shared::vec3_add(predicted, step), normal)).abs() <= 1e-18);
}

#[test]
fn kinematic_friction_holds_a_node_at_its_anchor_until_coulomb_and_then_drags_it() {
    let depth = 1.0e-4;
    let anchor = [0.0, 0.0, 0.0];
    // Slid 2e-5 from its anchor: under μ·depth = 3e-5, so it is held there.
    let slid = [2.0e-5, 0.0, -depth];
    let r = shared::kinematic_contact(IDENTITY, slid, surface(-depth), anchor, K, 0.3, FREE);
    let landed = shared::vec3_add(slid, shared::vec3_scale(r.force, 1.0 / K));
    assert!(close(landed, anchor, 1e-18));
    assert_eq!(r.anchor, anchor);
    // Slid 5e-5: it slips back by μ·depth, and the anchor goes with it.
    let slid = [5.0e-5, 0.0, -depth];
    let r = shared::kinematic_contact(IDENTITY, slid, surface(-depth), anchor, K, 0.3, FREE);
    let landed = shared::vec3_add(slid, shared::vec3_scale(r.force, 1.0 / K));
    assert!(close(landed, [2.0e-5, 0.0, 0.0], 1e-18));
    assert!(close(r.anchor, landed, 1e-18));
    assert!(
        (shared::vec3_length(r.friction) - 0.3 * r.normal_force).abs() <= 1e-12 * r.normal_force
    );
}

#[test]
fn kinematic_friction_keeps_a_constrained_node_on_the_surface() {
    // A plane tilted in x–z; the node may not move along z. Its friction step
    // along the plane would need z, so it can take none of it.
    let (s, c) = (0.6, 0.8);
    let normal = [s, 0.0, c];
    let depth = 1.0e-4;
    let predicted = [0.0, 0.0, -depth / c];
    let sample = SdfSample {
        distance: -depth,
        normal,
    };
    let axial = [[0.0, 0.0, 1.0], [0.0; 3]];
    let far_along_the_plane = [1.0e-3 * c + depth / s, 0.0, -1.0e-3 * s - depth / c];
    let r = shared::kinematic_contact(
        IDENTITY,
        predicted,
        sample,
        far_along_the_plane,
        K,
        0.3,
        axial,
    );
    // What the boundary conditions leave of the step.
    let step = shared::constrain(shared::vec3_scale(r.force, 1.0 / K), axial[0], axial[1]);
    let landed = shared::vec3_add(predicted, step);
    assert!(
        shared::vec3_dot(landed, normal).abs() <= 1e-18,
        "{landed:?}"
    );
    assert_eq!(landed[2], predicted[2]);
}
