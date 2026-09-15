//! The reverse trike, as geometry.
//!
//! One vehicle expressed as a [`cf_design::Mechanism`]: a triangulated frame,
//! a seat frame with a reclined back, direct steering, a swingarm, three
//! wheels and a rider. Everything is welded except the seven joints that
//! actually move, so the machine has twelve degrees of freedom.
//!
//! This crate is the definition. What is done with it — a mass budget, an STL
//! export, a simulation — belongs to whoever consumes it, which is why
//! [`Trike`] hands back the metrics a consumer needs rather than using them.
//!
//! # Conventions
//!
//! Millimetres, as everywhere in `cf-design`. `x` is aft of the front contact
//! patch, `y` is lateral, `z` is up from the ground.
//!
//! # Why the metrics come with it
//!
//! Integrating a part's mass needs a grid cell fine enough for its thinnest
//! feature, and checking that integration needs the part's closed-form volume.
//! Both are properties of the geometry, known only where it is built, so they
//! travel with it rather than being guessed at the far end.

// ⚠ `similar_names` fires on every left/right pair. A symmetric vehicle has
// symmetric members, and naming them anything but left and right to satisfy a
// lint would make the geometry harder to read, not easier.
#![allow(clippy::similar_names, clippy::too_many_lines)]

use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, PI};

use anyhow::{Result, bail};
use cf_design::{JointDef, JointKind, LinkageDef, LinkageKind, Material, Mechanism, Part, Solid};
use nalgebra::{Point3, UnitQuaternion, Vector3};

/// A built trike: the assembly, and what a consumer needs to measure it.
pub struct Trike {
    /// The validated assembly.
    pub mechanism: Mechanism,
    /// Per part, by name: closed-form volume and the grid cell its thinnest
    /// feature needs.
    pub metrics: HashMap<String, PartMetrics>,
    /// Per part, by name: where it sits in the world frame.
    pub origins: HashMap<String, Vector3<f64>>,
}

/// What a consumer needs to integrate and check one part.
#[derive(Debug, Clone, Copy)]
pub struct PartMetrics {
    /// Closed-form volume in mm³, arithmetic on the same dimensions that built
    /// the solid and never routed through a grid.
    pub volume_mm3: f64,
    /// Grid spacing fine enough for the part's thinnest feature.
    pub cell_mm: f64,
}

/// Build the trike.
///
/// # Example
///
/// Everything a consumer needs is on [`Trike`]. This is compiled as an
/// outside caller, so it sees only the public surface — which is the point of
/// the crate existing.
///
/// ```
/// let t = cf_trike::trike()?;
///
/// // The assembly, ready for to_model, to_stl_kit or inspection.
/// assert_eq!(t.mechanism.parts().len(), 28);
///
/// // Fifteen degrees of freedom in the tree — welds cost nothing, and the
/// // tie rod's near end is a ball. The linkage at its far end takes three
/// // back, so the machine really has twelve.
/// let dof: usize = t.mechanism.joints().iter().map(|j| j.kind().dof()).sum();
/// let held: usize = t.mechanism.linkages().iter().map(|l| l.kind().constrained_dof()).sum();
/// assert_eq!(dof, 15);
/// assert_eq!(held, 3);
/// assert_eq!(dof - held, 12);
///
/// // Each part carries what it takes to weigh it and where it sits.
/// let spine = t.metrics[cf_trike::ROOT_PART];
/// assert!(spine.volume_mm3 > 0.0 && spine.cell_mm > 0.0);
/// assert!(t.origins[cf_trike::ROOT_PART].x > 0.0);
/// # Ok::<(), anyhow::Error>(())
/// ```
///
/// # Errors
///
/// Fails if a part is composed of no pieces, if the assembly does not validate,
/// or if a joint chain never reaches the world.
pub fn trike() -> Result<Trike> {
    let (plan, linkages) = plan()?;
    let metrics: HashMap<String, PartMetrics> = plan
        .iter()
        .map(|p| {
            (
                p.name.to_owned(),
                PartMetrics {
                    volume_mm3: p.piece.volume_mm3,
                    cell_mm: p.cell_mm,
                },
            )
        })
        .collect();
    let mechanism = assemble(plan, linkages)?;
    let origins = world_origins(&mechanism)?;
    Ok(Trike {
        mechanism,
        metrics,
        origins,
    })
}

// ── Materials ───────────────────────────────────────────────────────────

/// Mild steel tube — the frame, the uprights, the swingarm.
const STEEL_KG_M3: f64 = 7850.0;
/// 6061 — the front rims and the seat pan.
const ALUMINIUM_KG_M3: f64 = 2700.0;
/// A pneumatic tyre's casing is not solid rubber; this is the smeared density
/// of the 16″ front tyre's annulus, not the density of rubber.
const FRONT_TYRE_KG_M3: f64 = 420.0;
/// The printed rear rim.
const PLA_KG_M3: f64 = 1250.0;
/// The cast rear tyre — `cf_cast::wheel::NOMINAL_PU_95A_DENSITY_KG_M3`.
const PU_95A_KG_M3: f64 = 1050.0;
/// Whole-body density of a person, near enough to water.
const RIDER_KG_M3: f64 = 1010.0;

// ── Nominal geometry, shared with `TrikeSpec::iter1` ────────────────────

/// Front contact patch to rear contact patch.
pub const WHEELBASE_MM: f64 = 1250.0;
/// Centre to centre of the two front wheels.
pub const TRACK_MM: f64 = 900.0;
/// 16″ front.
pub const FRONT_RADIUS_MM: f64 = 203.2;
/// 11″ rear — the polyurethane one.
pub const REAR_RADIUS_MM: f64 = 140.0;
/// Caster: the steering axis leans this far back from vertical.
pub const CASTER_DEG: f64 = 8.0;

/// Spine and cross-member centreline height.
const FRAME_Z_MM: f64 = 150.0;
/// Frame tube outside diameter.
const FRAME_OD_MM: f64 = 31.75;
/// Frame tube wall.
const FRAME_WALL_MM: f64 = 2.0;
/// Half the frame tube's outside diameter — where members butt onto it.
const FRAME_R_MM: f64 = FRAME_OD_MM / 2.0;

/// Air between the spine's tail and the rear tyre.
const REAR_WHEEL_CLEARANCE_MM: f64 = 20.0;

/// The spine stops this far short of the rear contact patch; the swingarm
/// carries the rest.
///
/// ⚠ Derived. At a typed 100 mm the spine ran to x = 1150 while the rear tyre
/// reaches forward to x = 1110 — the wheel was 40 mm inside the frame.
///
/// ⚠ The frame radius is in here because [`Solid::pipe`] **domes its ends**: a
/// member reaches a full radius past its endpoint node. Setting the clearance
/// without that term gave 4.1 mm of real air where 20 was asked for.
const TAIL_SETBACK_MM: f64 = REAR_RADIUS_MM + REAR_WHEEL_CLEARANCE_MM + FRAME_R_MM;

/// The part everything else hangs from.
pub const ROOT_PART: &str = "frame_spine";

/// Pivot to rear axle.
const SWINGARM_LENGTH_MM: f64 = 350.0;
/// Half the spacing of the swingarm arms at the axle.
const SWINGARM_HALF_WIDTH_MM: f64 = 60.0;

/// How far a member that butts onto another sinks into it.
///
/// ⚠ Not cosmetic. Sitting a round tube exactly on top of another leaves a
/// single point of contact: geometrically tangent, numerically fragile — a
/// probe mesh may or may not land a vertex on it, which is why one upright
/// read a 3.6 mm gap and its mirror read none — and unweldable in the real
/// world. A saddled joint overlaps.
const WELD_OVERLAP_MM: f64 = 6.0;

/// Seat tube stock — lighter than the frame, it carries a person not a kerb.
const SEAT_TUBE_OD_MM: f64 = 25.4;
/// Seat tube wall.
const SEAT_TUBE_WALL_MM: f64 = 1.5;
/// Rail centres sit this far apart.
const SEAT_WIDTH_MM: f64 = 300.0;
/// Hip point — where the pan meets the back, and where the seat's load goes
/// into the spine. Set by leg length from the bottom bracket, not by taste.
///
/// ★ This is the layout's dominant lever. On a tadpole, weight aft is weight
/// off the paired axle, and the rider is half the machine. Measured over the
/// hip positions this geometry allows, holding everything else fixed:
///
/// | hip x | bottom bracket | share | threshold |
/// |---|---|---|---|
/// | 674 | -250 | 0.512 | 0.752 g |
/// | 560 | -364 | 0.592 | 0.870 g |
/// | **500** | **-424** | **0.634** | **0.932 g** |
/// | 450 | -474 | 0.669 | 0.983 g |
///
/// ⚠ Re-measured. The first sweep was taken before the seat was seated on
/// the frame and before the rims stopped being discs, and both moved the
/// centre of gravity: it read 0.860 g at hip 500 where this reads 0.932. The
/// ordering held, which is why the decision still stands, but a table of
/// numbers that no longer reproduce is worse than no table.
///
/// 500 is the chosen point. It takes the biggest step available for the price
/// of boom tube, and it puts the pedals 424 mm ahead of the front contact
/// patch — feet ahead of the front axle, which is what a tadpole recumbent
/// looks like, not a compromise.
const HIP_X_MM: f64 = 500.0;
/// Hip height above the ground.
///
/// ⚠ Derived, not chosen: the seat's cross tube rests on top of the spine, so
/// the hip is a spine radius plus a seat-tube radius above the spine's
/// centreline. Typed as 210 mm it left the whole seat — and the rider on it —
/// floating 31 mm clear of the frame, welded in the joint graph and touching
/// nothing.
const HIP_Z_MM: f64 = FRAME_Z_MM + FRAME_R_MM + SEAT_TUBE_OD_MM / 2.0 - WELD_OVERLAP_MM;
/// Pan length forward of the hip.
const PAN_LENGTH_MM: f64 = 250.0;
/// Seat back length from the hip.
const SEAT_BACK_LENGTH_MM: f64 = 500.0;
/// Recline, measured from vertical. A cruiser sits up more than a racer.
const SEAT_BACK_ANGLE_DEG: f64 = 45.0;
/// Steering arm: how far aft of the kingpin the tie rod picks up. Longer is
/// lighter steering and less feedback.
const STEER_ARM_AFT_MM: f64 = 120.0;
/// How far inboard the arm end sits from the kingpin.
const STEER_ARM_INBOARD_MM: f64 = 42.3;
/// Height of the steering linkage above the ground.
const STEER_LINKAGE_Z_MM: f64 = 175.0;
/// Steering arm and tie rod stock.
const STEER_TUBE_OD_MM: f64 = 19.05;
/// Steering tube wall.
const STEER_TUBE_WALL_MM: f64 = 1.5;
/// Where the rider's hands fall, beside the hip.
const GRIP_X_MM: f64 = 620.0;
/// Grip half-spacing.
const GRIP_Y_MM: f64 = 280.0;
/// Grip height.
const GRIP_Z_MM: f64 = 320.0;

/// Hip to pedal, extended. Sets where the bottom bracket goes, and with it
/// where the hip has to sit for a given wheelbase.
const LEG_REACH_MM: f64 = 950.0;
/// How much higher than the hip the pedals sit.
const LEG_RISE_MM: f64 = 220.0;
/// Torso capsule radius — a person across the shoulders, near enough.
const TORSO_RADIUS_MM: f64 = 170.0;
/// Torso capsule half-length along the seat back.
const TORSO_HALF_MM: f64 = 190.0;
/// Both legs together, as one capsule.
const LEGS_RADIUS_MM: f64 = 105.0;
/// Leg capsule half-length.
const LEGS_HALF_MM: f64 = 340.0;

/// Panel half-thickness for the pan and the back.
///
/// ⚠ 6 mm, not the 3 mm this started at. A reclined 3 mm plate is a bounding
/// box of 354x300x354 that is almost entirely air, and the grid integrator
/// came 0.835% off its closed form — over tolerance — because a tilted plate
/// three cells thick is nearly all boundary. A seat shell is not foil anyway.
const SEAT_PANEL_HALF_MM: f64 = 3.0;

/// Where the two front diagonals meet the spine. Further aft makes a shallower
/// triangle: stiffer in bending, heavier, and it eats the space the seat wants.
const DIAGONAL_APEX_X_MM: f64 = 450.0;

/// Upright (hub carrier) tube outside diameter.
const UPRIGHT_OD_MM: f64 = 25.4;
/// Upright tube wall — thicker than the frame's, it takes the steering loads.
const UPRIGHT_WALL_MM: f64 = 3.0;
/// Half the front wheel's width. The hub is its widest part.
const FRONT_WHEEL_HALF_WIDTH_MM: f64 = 25.0;
/// Inside of the front rim's section.
///
/// ⚠ 174, not the 160 this started at. A 20 mm-thick solid aluminium annulus
/// is a disc, not a rim: it came to 2.12 kg a wheel where a real 16-inch rim,
/// hub and spokes are about 0.9. A rim is a thin section — this one is 6 mm,
/// which is still generous for what is really a hollow box.
///
/// ⚠ Spokes are absent. They would add roughly 0.15 kg a wheel, at a radius
/// that matters more for rotating inertia than for the mass budget, and this
/// example does not model rotating inertia.
const FRONT_RIM_INNER_MM: f64 = 174.0;

/// Where the front rim ends and its tyre begins. Shared by both so the two
/// cannot drift apart into a gap or an interpenetration — a class the
/// closed-form check above is blind to.
const FRONT_RIM_OUTER_MM: f64 = 180.0;
/// Where the rear rim ends and the cast polyurethane tyre begins.
const REAR_RIM_OUTER_MM: f64 = 115.0;
/// Half the rear wheel's width — rim and tyre are the same width.
const REAR_WHEEL_HALF_WIDTH_MM: f64 = 12.5;

// ── Pieces: a solid and its closed-form volume, built together ──────────

/// A primitive together with its analytic volume.
///
/// The volume is arithmetic on the same dimensions that built the solid, and
/// never goes through [`mass_properties`](cf_design::mechanism::mass::mass_properties) —
/// that is the whole point of
/// carrying it.
struct Piece {
    /// The shape itself.
    pub solid: Solid,
    /// Its closed-form volume, computed beside it.
    pub volume_mm3: f64,
}

/// A tube running between two world points, built in its own frame.
///
/// Returns the piece and the world position of its centre, which is what the
/// joint anchor needs. [`Solid::pipe`] sweeps a polyline with a spherical
/// cross-section, so corners mitre themselves and the ends are domed — which
/// is what a fillet weld looks like where members meet.
///
/// ⚠ The ends are capped: an outer capsule minus an inner one is a closed
/// shell, not an open-ended tube. For a weldment that is the more honest
/// shape, and the closed form below accounts for it.
#[must_use]
fn tube_between(a: Point3<f64>, b: Point3<f64>, od: f64, wall: f64) -> (Piece, Vector3<f64>) {
    let mid = nalgebra::center(&a, &b);
    let (la, lb) = (a - mid, b - mid);
    let r_outer = od / 2.0;
    let r_inner = r_outer - wall;
    let length = (b - a).norm();
    let path = |r: f64| Solid::pipe(vec![Point3::from(la), Point3::from(lb)], r);
    let shell = |r: f64| PI * r * r * length + 4.0 / 3.0 * PI * r * r * r;
    (
        Piece {
            solid: path(r_outer).subtract(path(r_inner)),
            volume_mm3: shell(r_outer) - shell(r_inner),
        },
        mid.coords,
    )
}

/// A tube from `a` to `b`, with its own origin at `a` rather than its centre.
///
/// For a member that pivots about one end — a tie rod, a link — the body
/// origin has to BE that end, because a part is placed at its joint's anchor.
fn tube_from(a: Point3<f64>, b: Point3<f64>, od: f64, wall: f64) -> (Piece, Vector3<f64>) {
    let (piece, mid) = tube_between(a, b, od, wall);
    let shift = mid - a.coords;
    (
        Piece {
            solid: piece.solid.translate(shift),
            volume_mm3: piece.volume_mm3,
        },
        a.coords,
    )
}

/// Z-aligned tube, centred at the origin.
#[must_use]
fn tube(od: f64, wall: f64, length: f64) -> Piece {
    let r_outer = od / 2.0;
    let r_inner = r_outer - wall;
    Piece {
        solid: Solid::cylinder(r_outer, length / 2.0).subtract(Solid::cylinder(r_inner, length)),
        volume_mm3: PI * (r_outer * r_outer - r_inner * r_inner) * length,
    }
}

/// Z-aligned annulus, centred at the origin.
#[must_use]
fn annulus(r_outer: f64, r_inner: f64, half_width: f64) -> Piece {
    Piece {
        solid: Solid::cylinder(r_outer, half_width)
            .subtract(Solid::cylinder(r_inner, half_width * 2.0)),
        volume_mm3: PI * (r_outer * r_outer - r_inner * r_inner) * 2.0 * half_width,
    }
}

/// Z-aligned solid disc, centred at the origin.
#[must_use]
fn disc(radius: f64, half_width: f64) -> Piece {
    Piece {
        solid: Solid::cylinder(radius, half_width),
        volume_mm3: PI * radius * radius * 2.0 * half_width,
    }
}

/// Axis-aligned box, centred at the origin.
#[must_use]
fn slab(half: Vector3<f64>) -> Piece {
    Piece {
        solid: Solid::cuboid(half),
        volume_mm3: 8.0 * half.x * half.y * half.z,
    }
}

/// Z-aligned capsule, centred at the origin.
#[must_use]
fn capsule(radius: f64, half_height: f64) -> Piece {
    Piece {
        solid: Solid::capsule(radius, half_height),
        volume_mm3: PI * radius * radius * 2.0 * half_height
            + 4.0 / 3.0 * PI * radius * radius * radius,
    }
}

/// Turn a Z-aligned piece into a Y-aligned one — wheels and cross-members.
#[must_use]
fn onto_y(p: Piece) -> Piece {
    Piece {
        solid: p.solid.rotate(UnitQuaternion::from_axis_angle(
            &Vector3::x_axis(),
            FRAC_PI_2,
        )),
        volume_mm3: p.volume_mm3,
    }
}

/// Tilt a piece about the y axis, for anything that does not lie along one.
#[must_use]
fn tilted(p: Piece, angle_rad: f64) -> Piece {
    Piece {
        solid: p.solid.rotate(UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            angle_rad,
        )),
        volume_mm3: p.volume_mm3,
    }
}

/// Union of **disjoint** pieces: the analytic volume is their sum, which is
/// true only because nothing here overlaps anything else in the same part.
///
/// # Errors
///
/// Fails if given no pieces at all.
fn joined(pieces: Vec<Piece>) -> Result<Piece> {
    let mut it = pieces.into_iter();
    let Some(first) = it.next() else {
        bail!("a part needs at least one piece");
    };
    Ok(it.fold(first, |acc, p| Piece {
        solid: acc.solid.union(p.solid),
        volume_mm3: acc.volume_mm3 + p.volume_mm3,
    }))
}

// ── The plan ────────────────────────────────────────────────────────────

/// One part, its placement, and the cell size its thinnest feature needs.
struct PartPlan {
    name: &'static str,
    parent: &'static str,
    /// Joint anchor **in the parent's frame**, millimetres.
    anchor_mm: Vector3<f64>,
    kind: JointKind,
    axis: Vector3<f64>,
    /// `Some(range)` for an articulation, `None` for a weld.
    range_rad: Option<(f64, f64)>,
    material: Material,
    piece: Piece,
    /// Grid spacing for
    /// [`mass_properties`](cf_design::mechanism::mass::mass_properties). A tube
    /// wall needs roughly four
    /// cells across it; a capsule needs nothing like that.
    cell_mm: f64,
}

/// The steering axis, leaning `CASTER_DEG` back from vertical.
#[must_use]
pub fn steering_axis() -> Vector3<f64> {
    let caster = CASTER_DEG.to_radians();
    Vector3::new(caster.sin(), 0.0, caster.cos())
}

/// Build the part table, in tree order.
///
/// # Errors
///
/// Fails if a part is composed of no pieces.
fn plan() -> Result<(Vec<PartPlan>, Vec<LinkageDef>)> {
    let steel = Material::new("mild steel", STEEL_KG_M3);
    let aluminium = Material::new("aluminium 6061", ALUMINIUM_KG_M3);

    // ── The frame, as nodes and members ─────────────────────────────
    //
    // Uprights sit inboard of the wheels by half a wheel's width plus the
    // upright's own radius, so the contact patches land on the nominal track.
    let upright_y = TRACK_MM / 2.0 - (FRONT_WHEEL_HALF_WIDTH_MM + UPRIGHT_OD_MM / 2.0);

    // Nose, the two kingpin bases, the apex the diagonals meet, and the tail.
    let node = |x: f64, y: f64| Point3::new(x, y, FRAME_Z_MM);
    let front_centre = node(0.0, 0.0);
    let kingpin_l = node(0.0, upright_y);
    let kingpin_r = node(0.0, -upright_y);
    let apex = node(DIAGONAL_APEX_X_MM, 0.0);
    let tail = node(WHEELBASE_MM - TAIL_SETBACK_MM, 0.0);

    let member = |a, b| tube_between(a, b, FRAME_OD_MM, FRAME_WALL_MM);
    let (spine, spine_at) = member(front_centre, tail);
    let (cross, cross_at) = member(kingpin_l, kingpin_r);
    let (brace_left, brace_left_at) = member(kingpin_l, apex);
    let (brace_right, brace_right_at) = member(kingpin_r, apex);
    let upright_length = 74.125;
    let upright_z = FRAME_Z_MM + FRAME_R_MM + upright_length / 2.0 - WELD_OVERLAP_MM;

    // Swingarm: two arms converging on the pivot, which is ON the spine —
    // parallel arms at y = +/-60 straddled it and touched nothing.
    let pivot = Point3::new(WHEELBASE_MM - SWINGARM_LENGTH_MM, 0.0, REAR_RADIUS_MM);
    let rear_axle = |y: f64| Point3::new(WHEELBASE_MM, y, REAR_RADIUS_MM);
    let arm = |y: f64| tube_between(pivot, rear_axle(y), 25.4, 2.0);
    let (arm_left, arm_left_at) = arm(SWINGARM_HALF_WIDTH_MM);
    let (arm_right, arm_right_at) = arm(-SWINGARM_HALF_WIDTH_MM);

    // ── The seat, as a frame ────────────────────────────────────────
    //
    // Hip, pan front and back top. The back leans SEAT_BACK_ANGLE_DEG off
    // vertical; the rails run along both, and a cross tube at the hip carries
    // the rider's weight into the spine.
    let recline = SEAT_BACK_ANGLE_DEG.to_radians();
    let half_width = SEAT_WIDTH_MM / 2.0;
    let hip = |y: f64| Point3::new(HIP_X_MM, y, HIP_Z_MM);
    let pan_front = |y: f64| Point3::new(HIP_X_MM - PAN_LENGTH_MM, y, HIP_Z_MM + 20.0);
    let back_top = |y: f64| {
        Point3::new(
            HIP_X_MM + SEAT_BACK_LENGTH_MM * recline.sin(),
            y,
            HIP_Z_MM + SEAT_BACK_LENGTH_MM * recline.cos(),
        )
    };
    // ── Steering ────────────────────────────────────────────────────
    //
    // Each upright carries an arm aft of its kingpin; a tie rod across the two
    // arm ends makes the wheels turn together, and a bar from each upright
    // reaches back to the rider's hands — direct steering, as a tadpole has.
    let steer_member = |a, b| tube_between(a, b, STEER_TUBE_OD_MM, STEER_TUBE_WALL_MM);
    let kingpin_pickup = |sign: f64| Point3::new(0.0, sign * upright_y, STEER_LINKAGE_Z_MM);
    let arm_end = |sign: f64| {
        Point3::new(
            STEER_ARM_AFT_MM,
            sign * (upright_y - STEER_ARM_INBOARD_MM),
            STEER_LINKAGE_Z_MM,
        )
    };
    let grip = |sign: f64| Point3::new(GRIP_X_MM, sign * GRIP_Y_MM, GRIP_Z_MM);
    let upright_top =
        |sign: f64| Point3::new(0.0, sign * upright_y, upright_z + upright_length / 2.0);
    let (steer_arm_left, steer_arm_left_at) = steer_member(kingpin_pickup(1.0), arm_end(1.0));
    let (steer_arm_right, steer_arm_right_at) = steer_member(kingpin_pickup(-1.0), arm_end(-1.0));
    // ⚠ The rod pivots at its LEFT end, not its centre: a part is placed at
    // its joint's anchor, so the body origin has to be the rod end.
    let (tie_rod, tie_rod_at) = tube_from(
        arm_end(1.0),
        arm_end(-1.0),
        STEER_TUBE_OD_MM,
        STEER_TUBE_WALL_MM,
    );
    let (bar_left, bar_left_at) = steer_member(upright_top(1.0), grip(1.0));
    let (bar_right, bar_right_at) = steer_member(upright_top(-1.0), grip(-1.0));
    let upright_centre = |sign: f64| Vector3::new(0.0, sign * upright_y, upright_z);

    let seat_member = |a, b| tube_between(a, b, SEAT_TUBE_OD_MM, SEAT_TUBE_WALL_MM);
    let (pan_rail_left, pan_rail_left_at) = seat_member(pan_front(half_width), hip(half_width));
    let (pan_rail_right, pan_rail_right_at) = seat_member(pan_front(-half_width), hip(-half_width));
    let (back_rail_left, back_rail_left_at) = seat_member(hip(half_width), back_top(half_width));
    let (back_rail_right, back_rail_right_at) =
        seat_member(hip(-half_width), back_top(-half_width));
    let (seat_cross, seat_cross_at) = seat_member(hip(half_width), hip(-half_width));

    // Panels: the pan level between the rails, the back lying along them.
    let pan_panel_at = Vector3::new(
        HIP_X_MM - PAN_LENGTH_MM / 2.0,
        0.0,
        HIP_Z_MM + 10.0 + SEAT_TUBE_OD_MM / 2.0,
    );
    let back_panel_at = (hip(0.0).coords + back_top(0.0).coords) / 2.0
        + Vector3::new(-recline.cos(), 0.0, recline.sin()) * (SEAT_TUBE_OD_MM / 2.0);

    // ── The rider, as a posture ─────────────────────────────────────
    //
    // Two capsules, because one cannot be both a torso and a pair of legs.
    // The torso lies along the seat back; the legs run from the hip to the
    // bottom bracket. Their masses follow from their volumes, and the centre
    // of gravity follows from where a reclined person actually is — rather
    // than from a height picked while modelling.
    // Pedals sit a leg's reach ahead of the hip and a little above it; the
    // horizontal run is what is left of the leg after the rise.
    let leg_run = (LEG_REACH_MM * LEG_REACH_MM - LEG_RISE_MM * LEG_RISE_MM).sqrt();
    let bottom_bracket = Point3::new(HIP_X_MM - leg_run, 0.0, HIP_Z_MM + LEG_RISE_MM);
    let torso_at = (hip(0.0).coords + back_top(0.0).coords) / 2.0;
    let legs_at = (hip(0.0).coords + bottom_bracket.coords) / 2.0;
    let legs_dir = bottom_bracket - hip(0.0);

    // ── The loop the tree cannot hold ───────────────────────────────
    //
    // The tie rod has a rod end at each side. One is its tree joint, on the
    // left arm; the other cannot be, because a tree gives a part one parent.
    // Without this the rod was welded to the left arm and the right wheel
    // steered independently of it.
    let linkages = vec![LinkageDef::new(
        "tie_rod_right",
        "tie_rod",
        "steer_arm_r",
        LinkageKind::Ball,
        Point3::from(arm_end(-1.0) - arm_end(1.0)),
    )];

    let parts = vec![
        PartPlan {
            name: "frame_spine",
            parent: "world",
            anchor_mm: spine_at,
            kind: JointKind::Free,
            axis: Vector3::z(),
            range_rad: None,
            material: steel.clone(),
            piece: spine,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "frame_cross",
            parent: "frame_spine",
            anchor_mm: cross_at - spine_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: cross,
            cell_mm: 0.5,
        },
        // ★ The triangulation. Without these two the front end is a T: the
        // kingpins hang off a cross-member whose only tie to the spine is the
        // single joint at the nose, so a cornering load reaches the frame as
        // bending rather than as tension and compression down a diagonal.
        PartPlan {
            name: "frame_diag_l",
            parent: "frame_spine",
            anchor_mm: brace_left_at - spine_at,
            kind: JointKind::Fixed,
            axis: Vector3::z(),
            range_rad: None,
            material: steel.clone(),
            piece: brace_left,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "frame_diag_r",
            parent: "frame_spine",
            anchor_mm: brace_right_at - spine_at,
            kind: JointKind::Fixed,
            axis: Vector3::z(),
            range_rad: None,
            material: steel.clone(),
            piece: brace_right,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "upright_l",
            parent: "frame_cross",
            anchor_mm: Vector3::new(0.0, upright_y, upright_z - FRAME_Z_MM),
            kind: JointKind::Revolute,
            axis: steering_axis(),
            range_rad: Some((-0.6, 0.6)),
            material: steel.clone(),
            piece: tube(UPRIGHT_OD_MM, UPRIGHT_WALL_MM, upright_length),
            cell_mm: 0.5,
        },
        PartPlan {
            name: "upright_r",
            parent: "frame_cross",
            anchor_mm: Vector3::new(0.0, -upright_y, upright_z - FRAME_Z_MM),
            kind: JointKind::Revolute,
            axis: steering_axis(),
            range_rad: Some((-0.6, 0.6)),
            material: steel.clone(),
            piece: tube(UPRIGHT_OD_MM, UPRIGHT_WALL_MM, upright_length),
            cell_mm: 0.5,
        },
        PartPlan {
            name: "rim_fl",
            parent: "upright_l",
            anchor_mm: Vector3::new(0.0, TRACK_MM / 2.0 - upright_y, FRONT_RADIUS_MM - upright_z),
            kind: JointKind::Revolute,
            axis: Vector3::y(),
            range_rad: None,
            material: aluminium.clone(),
            piece: joined(vec![
                onto_y(annulus(FRONT_RIM_OUTER_MM, FRONT_RIM_INNER_MM, 15.0)),
                onto_y(disc(30.0, FRONT_WHEEL_HALF_WIDTH_MM)),
            ])?,
            // ⚠ 1.0, not 2.0: the rim section is 6 mm, and three cells across
            // a wall put the integrator 0.688% off its closed form.
            cell_mm: 1.0,
        },
        PartPlan {
            name: "rim_fr",
            parent: "upright_r",
            anchor_mm: Vector3::new(0.0, upright_y - TRACK_MM / 2.0, FRONT_RADIUS_MM - upright_z),
            kind: JointKind::Revolute,
            axis: Vector3::y(),
            range_rad: None,
            material: aluminium.clone(),
            piece: joined(vec![
                onto_y(annulus(FRONT_RIM_OUTER_MM, FRONT_RIM_INNER_MM, 15.0)),
                onto_y(disc(30.0, FRONT_WHEEL_HALF_WIDTH_MM)),
            ])?,
            // ⚠ 1.0, not 2.0: the rim section is 6 mm, and three cells across
            // a wall put the integrator 0.688% off its closed form.
            cell_mm: 1.0,
        },
        PartPlan {
            name: "tyre_fl",
            parent: "rim_fl",
            anchor_mm: Vector3::zeros(),
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: Material::new("16in pneumatic tyre", FRONT_TYRE_KG_M3),
            piece: onto_y(annulus(
                FRONT_RADIUS_MM,
                FRONT_RIM_OUTER_MM,
                FRONT_WHEEL_HALF_WIDTH_MM,
            )),
            cell_mm: 2.0,
        },
        PartPlan {
            name: "tyre_fr",
            parent: "rim_fr",
            anchor_mm: Vector3::zeros(),
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: Material::new("16in pneumatic tyre", FRONT_TYRE_KG_M3),
            piece: onto_y(annulus(
                FRONT_RADIUS_MM,
                FRONT_RIM_OUTER_MM,
                FRONT_WHEEL_HALF_WIDTH_MM,
            )),
            cell_mm: 2.0,
        },
        PartPlan {
            name: "swingarm",
            parent: "frame_spine",
            anchor_mm: arm_left_at - spine_at,
            kind: JointKind::Revolute,
            axis: Vector3::y(),
            range_rad: Some((-0.35, 0.35)),
            material: steel.clone(),
            piece: arm_left,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "swingarm_r",
            parent: "swingarm",
            anchor_mm: arm_right_at - arm_left_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: arm_right,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "rim_r",
            parent: "swingarm",
            anchor_mm: rear_axle(0.0).coords - arm_left_at,
            kind: JointKind::Revolute,
            axis: Vector3::y(),
            range_rad: None,
            material: Material::new("PLA", PLA_KG_M3),
            piece: onto_y(disc(REAR_RIM_OUTER_MM, REAR_WHEEL_HALF_WIDTH_MM)),
            cell_mm: 1.0,
        },
        PartPlan {
            name: "tyre_r",
            parent: "rim_r",
            anchor_mm: Vector3::zeros(),
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: Material::new("95A polyurethane", PU_95A_KG_M3),
            piece: onto_y(annulus(
                REAR_RADIUS_MM,
                REAR_RIM_OUTER_MM,
                REAR_WHEEL_HALF_WIDTH_MM,
            )),
            cell_mm: 1.0,
        },
        // ── Steering ────────────────────────────────────────────
        //
        // The arms and bars are children of their uprights, so they turn with
        // the wheel. The tie rod is not: it ties both arms, which is a loop.
        PartPlan {
            name: "steer_arm_l",
            parent: "upright_l",
            anchor_mm: steer_arm_left_at - upright_centre(1.0),
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: steer_arm_left,
            cell_mm: 0.4,
        },
        PartPlan {
            name: "steer_arm_r",
            parent: "upright_r",
            anchor_mm: steer_arm_right_at - upright_centre(-1.0),
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: steer_arm_right,
            cell_mm: 0.4,
        },
        PartPlan {
            name: "tie_rod",
            parent: "steer_arm_l",
            anchor_mm: tie_rod_at - steer_arm_left_at,
            // A rod end, not a weld. The other end is a linkage, because a
            // tree cannot give one part two parents.
            kind: JointKind::Ball,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: tie_rod,
            cell_mm: 0.4,
        },
        PartPlan {
            name: "bar_l",
            parent: "upright_l",
            anchor_mm: bar_left_at - upright_centre(1.0),
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: bar_left,
            cell_mm: 0.4,
        },
        PartPlan {
            name: "bar_r",
            parent: "upright_r",
            anchor_mm: bar_right_at - upright_centre(-1.0),
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: bar_right,
            cell_mm: 0.4,
        },
        // ── The seat frame ──────────────────────────────────────
        // The cross tube at the hip is the load path: the rider's weight
        // reaches the spine through it, not through the panels.
        PartPlan {
            name: "seat_cross",
            parent: "frame_spine",
            anchor_mm: seat_cross_at - spine_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: seat_cross,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "seat_pan_rail_left",
            parent: "seat_cross",
            anchor_mm: pan_rail_left_at - seat_cross_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: pan_rail_left,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "seat_pan_rail_right",
            parent: "seat_cross",
            anchor_mm: pan_rail_right_at - seat_cross_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: pan_rail_right,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "seat_back_rail_left",
            parent: "seat_cross",
            anchor_mm: back_rail_left_at - seat_cross_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: back_rail_left,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "seat_back_rail_right",
            parent: "seat_cross",
            anchor_mm: back_rail_right_at - seat_cross_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel,
            piece: back_rail_right,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "seat_pan",
            parent: "seat_pan_rail_left",
            anchor_mm: pan_panel_at - pan_rail_left_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: aluminium.clone(),
            piece: slab(Vector3::new(
                PAN_LENGTH_MM / 2.0,
                half_width,
                SEAT_PANEL_HALF_MM,
            )),
            cell_mm: 1.0,
        },
        PartPlan {
            name: "seat_back",
            parent: "seat_back_rail_left",
            anchor_mm: back_panel_at - back_rail_left_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: aluminium,
            piece: tilted(
                slab(Vector3::new(
                    SEAT_BACK_LENGTH_MM / 2.0,
                    half_width,
                    SEAT_PANEL_HALF_MM,
                )),
                FRAC_PI_2 - recline,
            ),
            cell_mm: 1.0,
        },
        PartPlan {
            name: "rider_torso",
            parent: "seat_back",
            anchor_mm: torso_at - back_panel_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: Material::new("rider torso", RIDER_KG_M3),
            piece: tilted(capsule(TORSO_RADIUS_MM, TORSO_HALF_MM), FRAC_PI_2 - recline),
            cell_mm: 4.0,
        },
        PartPlan {
            name: "rider_legs",
            parent: "seat_pan",
            anchor_mm: legs_at - pan_panel_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: Material::new("rider legs", RIDER_KG_M3),
            piece: tilted(
                capsule(LEGS_RADIUS_MM, LEGS_HALF_MM),
                legs_dir.x.atan2(legs_dir.z),
            ),
            cell_mm: 4.0,
        },
    ];
    Ok((parts, linkages))
}

/// Turn the plan into a validated [`Mechanism`], consuming the solids.
///
/// # Errors
///
/// Fails if the assembly does not validate — a joint naming a part that does
/// not exist, a duplicate name, an orphan, or a part both welded and hinged.
fn assemble(plan: Vec<PartPlan>, linkages: Vec<LinkageDef>) -> Result<Mechanism> {
    let mut builder = Mechanism::builder("reverse trike");
    for linkage in linkages {
        builder = builder.linkage(linkage);
    }
    for p in plan {
        // ⚠ Every part's solid is already built where the part goes, relative
        // to its own body origin — so the joint IS at that origin. Saying so
        // matters: without an explicit joint origin `to_model` bbox-aligns an
        // articulated part's geometry to its anchor, which is right for a
        // finger segment modelled at the origin and wrong for a vehicle. It
        // was displacing the front wheels 180 mm and the swingarm 188.
        builder = builder
            .part(Part::new(p.name, p.piece.solid, p.material).with_joint_origin(Vector3::zeros()));
        let joint = JointDef::new(
            format!("{}_joint", p.name),
            p.parent,
            p.name,
            p.kind,
            Point3::from(p.anchor_mm),
            p.axis,
        );
        builder = builder.joint(match p.range_rad {
            Some((lo, hi)) => joint.with_range(lo, hi),
            None => joint,
        });
    }
    let errors = builder.validate();
    if !errors.is_empty() {
        bail!("the mechanism does not validate: {errors:?}");
    }
    Ok(builder.build())
}

// ── The walk cf-design does not have ────────────────────────────────────

/// World-frame origin of every part, by summing joint anchors to the root.
///
/// ⚠ Translations only. That is exact **at the reference configuration** here
/// because every part's orientation is baked into its solid rather than into a
/// joint, so no parent is rotated relative to its own parent. A mechanism that
/// posed its joints would need the rotations composed too — which is what
/// `to_model` plus a forward kinematics pass already does, at the cost of
/// meshing every part.
///
/// # Errors
///
/// Fails if a part is the child of more than one joint, or if a joint chain
/// never reaches the world.
pub fn world_origins(mechanism: &Mechanism) -> Result<HashMap<String, Vector3<f64>>> {
    // ⚠ One joint per child. `to_model` places a body from the *first* joint
    // naming it as child (`model_builder.rs:262`, and `:808` again for the
    // mesh offset); a map would silently keep the last, so the two placements
    // would disagree. Refuse instead of diverging.
    let mut parent_of: HashMap<&str, (&str, Vector3<f64>)> = HashMap::new();
    for j in mechanism.joints() {
        if parent_of
            .insert(j.child(), (j.parent(), j.anchor().coords))
            .is_some()
        {
            bail!(
                "part {} is the child of more than one joint, so its placement \
                 here and in `to_model` would differ",
                j.child()
            );
        }
    }

    let mut origins = HashMap::new();
    for part in mechanism.parts() {
        let mut here = Vector3::zeros();
        let mut cursor = part.name();
        let mut hops = 0;
        while let Some(&(parent, anchor)) = parent_of.get(cursor) {
            here += anchor;
            if parent == "world" {
                break;
            }
            cursor = parent;
            hops += 1;
            // ⚠ Not redundant: `MechanismBuilder::validate` checks duplicates,
            // cross-references and orphans, but NOT cycles — a part that is its
            // own ancestor builds happily and only surfaces at `to_model`, as
            // `MechanismError::PartNotReachable`. Without this the walk spins.
            if hops > mechanism.parts().len() {
                bail!(
                    "the joint chain from {} never reaches the world",
                    part.name()
                );
            }
        }
        origins.insert(part.name().to_owned(), here);
    }
    Ok(origins)
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    /// The three collections on [`Trike`] describe the same machine.
    ///
    /// A consumer indexes `metrics` and `origins` by the names it reads off
    /// `mechanism.parts()`. Nothing else checks that those key sets agree, and
    /// a part added to the plan but not to the metric walk would read as a
    /// missing key at the call site rather than as a build failure here.
    #[test]
    fn every_part_has_metrics_and_an_origin() {
        let t = trike().unwrap();
        let names: Vec<&str> = t.mechanism.parts().iter().map(Part::name).collect();

        assert_eq!(names.len(), t.metrics.len(), "metrics: {names:?}");
        assert_eq!(names.len(), t.origins.len(), "origins: {names:?}");
        for name in &names {
            assert!(t.metrics.contains_key(*name), "no metrics for {name}");
            assert!(t.origins.contains_key(*name), "no origin for {name}");
        }
    }

    /// Every metric is a quantity you could weigh or mesh with.
    ///
    /// A zero or non-finite cell is not a rounding problem: `seat_pan` once
    /// exported as an 84-byte STL because its cell was coarser than the pan
    /// was thick, and the volume beside it stayed plausible throughout.
    #[test]
    fn every_metric_is_physical() {
        let t = trike().unwrap();
        for (name, m) in &t.metrics {
            assert!(
                m.volume_mm3.is_finite() && m.volume_mm3 > 0.0,
                "{name} has volume {}",
                m.volume_mm3
            );
            assert!(
                m.cell_mm.is_finite() && m.cell_mm > 0.0,
                "{name} has cell {}",
                m.cell_mm
            );
        }
    }

    /// Each upright's outer wall meets its wheel's inboard face.
    ///
    /// ⚠ This replaced a check that the front wheels sit `TRACK_MM` apart.
    /// They do — and they do so for any upright position, because the wheel's
    /// anchor is `TRACK_MM / 2 - upright_y` on a parent at `upright_y`, which
    /// cancels. That assertion compared `TRACK_MM` with itself; moving the
    /// uprights 3 mm left it green. What the track actually buys is where the
    /// contact patches land, and that depends on this fit.
    #[test]
    fn each_upright_meets_the_wheel_it_carries() {
        let t = trike().unwrap();
        for (upright, rim) in [("upright_l", "rim_fl"), ("upright_r", "rim_fr")] {
            let outer_wall = t.origins[upright].y.abs() + UPRIGHT_OD_MM / 2.0;
            let inboard_face = t.origins[rim].y.abs() - FRONT_WHEEL_HALF_WIDTH_MM;
            assert!(
                (outer_wall - inboard_face).abs() < 1e-9,
                "{upright} ends at {outer_wall} mm and {rim} starts at \
                 {inboard_face} mm — a gap here moves the contact patch off \
                 the {TRACK_MM} mm track the rollover threshold is read from"
            );
        }
    }

    /// The rear axle lands a wheelbase behind the front, through the chain.
    ///
    /// `rim_r` reaches the world through the swingarm and the spine, so this
    /// is a claim about [`world_origins`] summing that chain correctly — not
    /// about the constant, which the rear axle is placed from directly.
    #[test]
    fn the_rear_axle_lands_a_wheelbase_behind_the_front() {
        let t = trike().unwrap();
        let wheelbase = t.origins["rim_r"].x - t.origins["rim_fl"].x;
        assert!(
            (wheelbase - WHEELBASE_MM).abs() < 1e-9,
            "the walk puts the axles {wheelbase} mm apart, not {WHEELBASE_MM}"
        );
        assert!(
            t.origins["rim_r"].y.abs() < 1e-9,
            "the lone wheel must sit on the centreline — off it, the roll \
             moment no longer belongs wholly to the paired axle"
        );
    }

    /// The steering axis leans back by the caster angle, and only back.
    ///
    /// ⚠ The oracle is `atan2` on the returned vector, not the `sin`/`cos`
    /// the function itself used — an oracle built from the same trig would
    /// agree with a sign error.
    #[test]
    fn the_steering_axis_leans_back_by_the_caster_angle() {
        let axis = steering_axis();
        assert!(
            (axis.norm() - 1.0).abs() < 1e-12,
            "axis is not a unit vector"
        );
        assert!(
            axis.y.abs() < 1e-12,
            "a caster angle leans back, not sideways"
        );

        let lean = axis.x.atan2(axis.z).to_degrees();
        assert!(
            (lean - CASTER_DEG).abs() < 1e-9,
            "the axis leans {lean} deg, CASTER_DEG says {CASTER_DEG}"
        );
    }

    /// The tie rod's far end ties a part that exists, and takes three dof.
    #[test]
    fn the_linkage_ties_two_parts_that_exist() {
        let t = trike().unwrap();
        let names: Vec<&str> = t.mechanism.parts().iter().map(Part::name).collect();

        assert_eq!(t.mechanism.linkages().len(), 1);
        for l in t.mechanism.linkages() {
            assert!(
                names.contains(&l.a()),
                "{} names no part {}",
                l.name(),
                l.a()
            );
            assert!(
                names.contains(&l.b()),
                "{} names no part {}",
                l.name(),
                l.b()
            );
            assert_eq!(l.kind().constrained_dof(), 3);
        }
    }

    /// Building it twice gives the same machine.
    ///
    /// The metrics and origins travel in `HashMap`s, whose iteration order is
    /// not stable; anything derived by folding over one would wander between
    /// runs while every single-part assertion still passed.
    #[test]
    fn building_it_twice_gives_the_same_machine() {
        let a = trike().unwrap();
        let b = trike().unwrap();

        for (name, m) in &a.metrics {
            let n = b.metrics[name];
            assert!(
                (m.volume_mm3 - n.volume_mm3).abs() < f64::EPSILON
                    && (m.cell_mm - n.cell_mm).abs() < f64::EPSILON,
                "{name} differs between builds"
            );
            assert_eq!(a.origins[name], b.origins[name], "{name} moved");
        }
    }

    /// `world_origins` refuses a part with two parents rather than picking one.
    ///
    /// `to_model` places a body from the *first* joint naming it as a child.
    /// A walk that kept the last would disagree with the model it is meant to
    /// describe — and would do so quietly, which is the whole reason this
    /// returns a `Result`.
    #[test]
    fn world_origins_refuses_a_part_with_two_parents() {
        let ball = |name: &str| {
            Part::new(
                name,
                Solid::sphere(10.0),
                Material::new("steel", STEEL_KG_M3),
            )
        };
        let joint = |name: &str, parent: &str, at: f64| {
            JointDef::new(
                name,
                parent,
                "child",
                JointKind::Revolute,
                Point3::new(at, 0.0, 0.0),
                Vector3::y(),
            )
        };
        let m = Mechanism::builder("two parents")
            .part(ball("root"))
            .part(ball("other"))
            .part(ball("child"))
            .joint(joint("j0", "root", 10.0))
            .joint(joint("j1", "other", 90.0))
            .build();

        let err = world_origins(&m).unwrap_err().to_string();
        assert!(
            err.contains("child") && err.contains("more than one joint"),
            "unhelpful refusal: {err}"
        );
    }
}
