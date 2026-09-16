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
use cf_design::{
    Bushing, JointDef, JointKind, LinkageDef, LinkageKind, Material, Mechanism, Part, Solid,
};
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
    /// the solid and never routed through a grid — `None` for authored
    /// geometry, which has no elementary volume and is checked by refining the
    /// grid instead.
    pub volume_mm3: Option<f64>,
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
/// assert_eq!(t.mechanism.parts().len(), 36);
///
/// // Twenty-three degrees of freedom in the tree — welds cost nothing, and
/// // three joints are balls, worth three each. The three linkages take nine
/// // back, so the machine really has fourteen: it steers, three wheels turn,
/// // the swingarm swings, and each front wheel moves in bump.
/// let dof: usize = t.mechanism.joints().iter().map(|j| j.kind().dof()).sum();
/// let held: usize = t.mechanism.linkages().iter().map(|l| l.kind().constrained_dof()).sum();
/// assert_eq!(dof, 23);
/// assert_eq!(held, 9);
/// assert_eq!(dof - held, 14);
///
/// // Each part carries what it takes to weigh it and where it sits.
/// let spine = t.metrics[cf_trike::ROOT_PART];
/// assert!(spine.volume_mm3.is_some_and(|v| v > 0.0) && spine.cell_mm > 0.0);
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
pub const WHEELBASE_MM: f64 = 2650.0;
/// Centre to centre of the two front wheels.
pub const TRACK_MM: f64 = 1750.0;
/// 16″ front.
pub const FRONT_RADIUS_MM: f64 = FRONT_RIM_R_MM + FRONT_SECTION_MM * FRONT_ASPECT;
/// 11″ rear — the polyurethane one.
pub const REAR_RADIUS_MM: f64 = REAR_RIM_R_MM + REAR_SECTION_MM * REAR_ASPECT;

/// Front tyre: **235/40R17**. Section width, millimetres.
///
/// ★ Sized by LOAD, not ambition — 243 kg per front wheel. An Ariel Atom 4 is
/// 595 kg on 235/40R17. A wider tyre on this mass never reaches its grip.
const FRONT_SECTION_MM: f64 = 235.0;
/// Front aspect ratio — sidewall height as a fraction of section width.
const FRONT_ASPECT: f64 = 0.40;
/// Front rim radius: 17 inches across the bead seats.
const FRONT_RIM_R_MM: f64 = 17.0 * 25.4 / 2.0;
/// Rear tyre: **275/35R18**. It carries 298 kg but does ALL the rear drive and
/// braking, so it is sized for work rather than load.
///
/// ⚠ 305 and 335 were checked and are **underloaded** at this mass: they never
/// get warm or loaded enough to grip. Wider needs more mass on the rear, which
/// is the rollover trade again.
const REAR_SECTION_MM: f64 = 275.0;
/// Rear aspect ratio.
const REAR_ASPECT: f64 = 0.35;
/// Rear rim radius: 18 inches.
const REAR_RIM_R_MM: f64 = 18.0 * 25.4 / 2.0;
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
/// How far the rider settles into the seat.
///
/// Flesh and a cushion compress, so a rider rests slightly *in* the seat
/// rather than balanced on it. It also keeps the contact off a knife edge:
/// the weld gate wants parts that touch, and exact tangency leaves that to
/// the last bit of a float.
const RIDER_SETTLE_MM: f64 = 5.0;

/// Where the two front diagonals meet the spine.
///
/// Further aft makes a shallower triangle: stiffer in bending, heavier, and it
/// eats the space the seat wants. ⚠ That last clause stopped being a caution
/// and became a measurement when the diagonals were re-aimed at the tower
/// tops — at 450 they ran clean through the seat pan, the rider's legs and his
/// torso. 230 puts them on the spine ahead of the pan.
const DIAGONAL_APEX_X_MM: f64 = 230.0;

/// How far the wheel centre sits outboard of the kingpin line.
///
/// ★ The kingpin offset, and a real suspension number: it is the lever the
/// contact patch has about the steering axis, so it sets how much the bars
/// fight back over bumps and under braking.
///
/// ⚠ 37.7 is inherited, not chosen. It is what the stand-in upright happened
/// to give — half a wheel's width plus a 25.4 mm tube's radius — and it is
/// stated as a number here because the tube it came from no longer exists.
/// It wants choosing on its own terms.
const KINGPIN_OFFSET_MM: f64 = 60.0;
/// Where the kingpin line sits, half a track in from the wheel.
const UPRIGHT_Y_MM: f64 = TRACK_MM / 2.0 - KINGPIN_OFFSET_MM;
/// Radius of the front rim's hub disc — what the bearing housing must fit in.
const FRONT_HUB_R_MM: f64 = 30.0;
/// Outer radius of the upright's wheel-bearing housing. Sized to live inside
/// the rim's hub rather than beside it.
const BEARING_BOSS_R_MM: f64 = 27.0;
/// Half-length of the same, along the axle.
const BEARING_BOSS_HALF_MM: f64 = 21.0;
/// Bore through it, for the stub axle.
const BEARING_BORE_R_MM: f64 = 16.0;
/// Radius of the boss carrying a ball joint at each end of the upright.
const BALL_BOSS_R_MM: f64 = 18.0;
/// Half-height of the same, along the steering axis.
const BALL_BOSS_HALF_MM: f64 = 12.0;
/// Radius of the strut tying the two ball joints past the bearing.
const UPRIGHT_STRUT_R_MM: f64 = 12.0;
/// Wall around a bushing, between its outer sleeve and fresh air.
const BUSH_HOUSING_WALL_MM: f64 = 3.5;
/// Radius of the ball-joint cup at a wishbone's outboard end.
const BALL_CUP_R_MM: f64 = 20.0;
/// Half-height of the same, along the ball joint's axis.
const BALL_CUP_HALF_MM: f64 = 12.0;
/// Outer radius of a wishbone's legs.
const WISHBONE_LEG_R_MM: f64 = 11.0;
/// Wall of the same.
///
/// ⚠ A wishbone leg is a TUBE. Built solid it weighs three times as much for
/// the same outside diameter and almost no extra stiffness — bending goes as
/// the fourth power of radius, so the metal near the axis is carrying nothing
/// but itself.
const WISHBONE_LEG_WALL_MM: f64 = 2.5;
/// Wall left around the ball cup's socket.
const BALL_CUP_WALL_MM: f64 = 5.0;

/// Fillet radius where the upright's members blend.
///
/// ⚠ Not decoration. Every one of these joins is a corner in the load path
/// between a ball joint and the wheel, and a sharp one is where it breaks.
const UPRIGHT_FILLET_MM: f64 = 8.0;

/// Inboard pickup for the LOWER wishbone, from the centreline.
const LOWER_PICKUP_Y_MM: f64 = 300.0;
/// Inboard pickup for the UPPER wishbone.
///
/// ★ **Further outboard than the lower one on purpose**, which makes the upper
/// arm SHORTER. A short upper arm pulls the top of the wheel inboard as the
/// suspension rises, so the wheel gains negative camber in bump — which is what
/// keeps the tyre flat on the road when the body rolls in a corner.
const UPPER_PICKUP_Y_MM: f64 = 380.0;
/// Fore-aft half-spread of a wishbone's two frame pickups.
///
/// ★ This is what makes a wishbone a wishbone rather than a link: two pickups
/// give the arm a pivot **axis**, and the spread between them is the lever
/// that carries braking and cornering loads into the frame.
const ARM_PICKUP_HALF_SPREAD_MM: f64 = 120.0;
/// Height of the lower wishbone's frame pivot — level with the lower ball,
/// so the arm runs flat at rest.
const LOWER_PIVOT_Z_MM: f64 = FRAME_Z_MM;
/// Height of the upper wishbone's frame pivot, on top of the tower.
const UPPER_PIVOT_Z_MM: f64 = 480.0;
/// Lower ball joint, on the upright.
const LOWER_BALL_Z_MM: f64 = FRAME_Z_MM;
/// Upper ball joint. The gap to the lower ball is the upright's working length.
///
/// ⚠ **280 mm, not 140.** At 140 the two arms were nearly parallel over a
/// 735 mm span and fouled each other within the declared travel — the bump
/// sweep said so the moment the vehicle was re-based to car dimensions. Ball
/// separation has to scale with arm length or the linkage has no room to work.
const UPPER_BALL_Z_MM: f64 = 430.0;
/// Wishbone tube stock.
const ARM_OD_MM: f64 = 22.2;
/// Wishbone tube wall.
const ARM_WALL_MM: f64 = 2.0;
/// Tower stock — it carries the upper wishbone above the frame.
const TOWER_OD_MM: f64 = 25.4;
/// Tower wall.
const TOWER_WALL_MM: f64 = 2.0;

/// Bore radius of the bushing at every inboard wishbone pivot.
const BUSH_BORE_R_MM: f64 = 5.0;
/// Outer radius of the same.
const BUSH_OUTER_R_MM: f64 = 14.0;
/// Length of the same.
const BUSH_LENGTH_MM: f64 = 30.0;
/// Hardness of the polyurethane in it.
///
/// ⚠⚠ 95A is the top of the Shore A scale, where the hardness-to-modulus
/// correlation is steepest — five points doubles it. Read the rate this
/// produces as an order of magnitude until the real material is measured.
const BUSH_SHORE_A: f64 = 95.0;
/// Half the front wheel's width. The hub is its widest part.
const FRONT_WHEEL_HALF_WIDTH_MM: f64 = FRONT_SECTION_MM / 2.0;
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
const FRONT_RIM_INNER_MM: f64 = FRONT_RIM_R_MM - 6.0;

/// Where the front rim ends and its tyre begins. Shared by both so the two
/// cannot drift apart into a gap or an interpenetration — a class the
/// closed-form check above is blind to.
const FRONT_RIM_OUTER_MM: f64 = FRONT_RIM_R_MM;
/// Where the rear rim ends and the cast polyurethane tyre begins.
const REAR_RIM_OUTER_MM: f64 = REAR_RIM_R_MM;
/// Half the rear wheel's width — rim and tyre are the same width.
const REAR_WHEEL_HALF_WIDTH_MM: f64 = REAR_SECTION_MM / 2.0;

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
    /// Its closed-form volume, computed beside it — `None` when there is not
    /// one.
    ///
    /// ⚠ Stock members have closed forms because they are unions of
    /// **disjoint** primitives. Authored geometry does not: a part with
    /// blended fillets, a bearing pocket and a boss has no elementary volume,
    /// and inventing one would be worse than admitting it. Those are checked
    /// by refining the grid instead — see the mass oracle in
    /// `example-trike-mass-budget`.
    pub volume_mm3: Option<f64>,
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
            volume_mm3: Some(shell(r_outer) - shell(r_inner)),
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

/// Z-aligned annulus, centred at the origin.
#[must_use]
fn annulus(r_outer: f64, r_inner: f64, half_width: f64) -> Piece {
    Piece {
        solid: Solid::cylinder(r_outer, half_width)
            .subtract(Solid::cylinder(r_inner, half_width * 2.0)),
        volume_mm3: Some(PI * (r_outer * r_outer - r_inner * r_inner) * 2.0 * half_width),
    }
}

/// Z-aligned solid disc, centred at the origin.
#[must_use]
fn disc(radius: f64, half_width: f64) -> Piece {
    Piece {
        solid: Solid::cylinder(radius, half_width),
        volume_mm3: Some(PI * radius * radius * 2.0 * half_width),
    }
}

/// Axis-aligned box, centred at the origin.
#[must_use]
fn slab(half: Vector3<f64>) -> Piece {
    Piece {
        solid: Solid::cuboid(half),
        volume_mm3: Some(8.0 * half.x * half.y * half.z),
    }
}

/// Z-aligned capsule, centred at the origin.
#[must_use]
fn capsule(radius: f64, half_height: f64) -> Piece {
    Piece {
        solid: Solid::capsule(radius, half_height),
        volume_mm3: Some(
            PI * radius * radius * 2.0 * half_height + 4.0 / 3.0 * PI * radius * radius * radius,
        ),
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

/// The front upright — the first part here that is designed rather than stood
/// in for.
///
/// It carries four interfaces and exists to hold them in the right places:
/// a ball joint at each end of the steering axis, a wheel bearing outboard,
/// and the strut that ties them. The line through the two balls IS the
/// steering axis, so the geometry is what makes caster real.
///
/// ⚠ **No closed-form volume.** The members are blended, not disjoint, and
/// the bore is subtracted — there is no elementary volume, and the honest
/// answer is `None` rather than a sum that quietly double-counts every
/// fillet. Its mass is checked by refining the grid instead.
fn front_upright(sign: f64) -> Piece {
    let axis = steering_axis();
    let upper = axis * (UPPER_BALL_Z_MM - LOWER_BALL_Z_MM);
    let wheel = Vector3::new(
        0.0,
        sign * KINGPIN_OFFSET_MM,
        FRONT_RADIUS_MM - LOWER_BALL_Z_MM,
    );
    // Bosses lie along the steering axis; the bearing lies along the axle.
    let lean = UnitQuaternion::rotation_between(&Vector3::z(), &axis)
        .unwrap_or_else(UnitQuaternion::identity);
    let boss = |at: Vector3<f64>| {
        Solid::cylinder(BALL_BOSS_R_MM, BALL_BOSS_HALF_MM)
            .rotate(lean)
            .translate(at)
    };
    let housing = Solid::cylinder(BEARING_BOSS_R_MM, BEARING_BOSS_HALF_MM)
        .rotate(UnitQuaternion::from_axis_angle(
            &Vector3::x_axis(),
            FRAC_PI_2,
        ))
        .translate(wheel);
    // One strut from ball to ball, mitred as it passes the bearing.
    let strut = Solid::pipe(
        vec![Point3::origin(), Point3::from(wheel), Point3::from(upper)],
        UPRIGHT_STRUT_R_MM,
    );

    let body = Solid::smooth_union_all(
        vec![boss(Vector3::zeros()), boss(upper), housing, strut],
        UPRIGHT_FILLET_MM,
    );
    // Through-bore for the stub axle, and it must clear the housing on both
    // faces or it is a blind pocket.
    let bore = Solid::cylinder(BEARING_BORE_R_MM, BEARING_BOSS_HALF_MM * 2.0)
        .rotate(UnitQuaternion::from_axis_angle(
            &Vector3::x_axis(),
            FRAC_PI_2,
        ))
        .translate(wheel);

    Piece {
        solid: body.subtract(bore),
        volume_mm3: None,
    }
}

/// The lower wishbone — one fabricated piece, not two welded legs.
///
/// Three interfaces: a bushing housing at each inboard pickup, and a
/// ball-joint cup where the legs converge. Its own origin is the midpoint of
/// the two pickups, which is a point ON its pivot axis, so the joint anchor
/// and the geometry agree.
///
/// ⚠ **No closed-form volume**, for the reason the legs used to be two parts:
/// they overlap at the ball cup, and `joined` sums analytic volumes only for
/// pieces that do not. Authoring the arm as one blended solid says that
/// honestly instead of splitting it to dodge the arithmetic.
fn lower_wishbone(sign: f64) -> Piece {
    let ball = Vector3::new(0.0, sign * (UPRIGHT_Y_MM - LOWER_PICKUP_Y_MM), 0.0);
    let pickup = |x: f64| Vector3::new(x, 0.0, 0.0);
    let spread = ARM_PICKUP_HALF_SPREAD_MM;

    // Housings lie along the pivot axis, which is x.
    let along_x = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), FRAC_PI_2);
    let housing = |at: Vector3<f64>| {
        Solid::cylinder(BUSH_OUTER_R_MM + BUSH_HOUSING_WALL_MM, BUSH_LENGTH_MM / 2.0)
            .rotate(along_x)
            .translate(at)
    };
    // The cup takes the ball on the upright, so it shares its axis.
    let cup = Solid::cylinder(BALL_CUP_R_MM, BALL_CUP_HALF_MM)
        .rotate(
            UnitQuaternion::rotation_between(&Vector3::z(), &steering_axis())
                .unwrap_or_else(UnitQuaternion::identity),
        )
        .translate(ball);
    let leg = |x: f64, r: f64| Solid::pipe(vec![Point3::from(pickup(x)), Point3::from(ball)], r);

    let body = Solid::smooth_union_all(
        vec![
            housing(pickup(-spread)),
            housing(pickup(spread)),
            cup,
            leg(-spread, WISHBONE_LEG_R_MM),
            leg(spread, WISHBONE_LEG_R_MM),
        ],
        UPRIGHT_FILLET_MM,
    );

    // Everything that comes out: the two bushing bores, the ball socket, and
    // the bore down each leg. The legs are tubes, not rods.
    let bore = |at: Vector3<f64>| {
        Solid::cylinder(BUSH_OUTER_R_MM, BUSH_LENGTH_MM)
            .rotate(along_x)
            .translate(at)
    };
    let inner = WISHBONE_LEG_R_MM - WISHBONE_LEG_WALL_MM;
    let socket = Solid::cylinder(BALL_CUP_R_MM - BALL_CUP_WALL_MM, BALL_CUP_HALF_MM)
        .rotate(
            UnitQuaternion::rotation_between(&Vector3::z(), &steering_axis())
                .unwrap_or_else(UnitQuaternion::identity),
        )
        .translate(ball);

    Piece {
        solid: body
            .subtract(bore(pickup(-spread)))
            .subtract(bore(pickup(spread)))
            .subtract(leg(-spread, inner))
            .subtract(leg(spread, inner))
            .subtract(socket),
        volume_mm3: None,
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
    // ⚠ Sums only when every piece knows its own volume. One authored piece
    // with no closed form makes the union's unknown too, which is the honest
    // answer rather than a total that silently omits a term.
    Ok(it.fold(first, |acc, p| Piece {
        solid: acc.solid.union(p.solid),
        volume_mm3: match (acc.volume_mm3, p.volume_mm3) {
            (Some(a), Some(b)) => Some(a + b),
            _ => None,
        },
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
    /// The elastomer bushing at this joint, if it has one.
    ///
    /// ⚠ Its rate reaches the joint through
    /// [`Bushing::joint_stiffness`], not the newton-metre figure: the
    /// mechanism's torque unit is a microjoule, and the raw number would be a
    /// millionth of what was meant.
    bushing: Option<Bushing>,
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
    let upright_y = UPRIGHT_Y_MM;

    // Nose, the two kingpin bases, the apex the diagonals meet, and the tail.
    let node = |x: f64, y: f64| Point3::new(x, y, FRAME_Z_MM);
    let node_at = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
    let front_centre = node(0.0, 0.0);
    // ⚠ The frame ends at the suspension pickups, not out at the wheel.
    //
    // With rigid uprights it ran all the way to the kingpin, and that point is
    // now the lower BALL JOINT — so the old cross-member and both diagonals
    // reached 292 mm past the pickups, through the volume the lower wishbone
    // swings in, welding the suspension solid. Every gate passed: the bump
    // sweep only tested the travel extremes and never the rest pose, and a
    // scan of non-joined pairs is what found it.
    // ⚠ The cross-member spans the OUTERMOST pickup, because it carries both:
    // the lower wishbone pivots on it inboard, and the tower stands on its end.
    // Ending it at the lower pickup left the towers floating 52 mm off it.
    let cross_half = if UPPER_PICKUP_Y_MM > LOWER_PICKUP_Y_MM {
        UPPER_PICKUP_Y_MM
    } else {
        LOWER_PICKUP_Y_MM
    };
    let pickup_l = node(0.0, cross_half);
    let pickup_r = node(0.0, -cross_half);
    let apex = node(DIAGONAL_APEX_X_MM, 0.0);
    let tail = node(WHEELBASE_MM - TAIL_SETBACK_MM, 0.0);

    let member = |a, b| tube_between(a, b, FRAME_OD_MM, FRAME_WALL_MM);
    let (spine, spine_at) = member(front_centre, tail);
    let (cross, cross_at) = member(pickup_l, pickup_r);
    // ★ The diagonals brace the TOWER TOPS back to the spine, not the pickups.
    //
    // In the wishbone's own plane a diagonal ending at the pickup sits on the
    // arm's pivot axis, so the arm can never move off it — the bump sweep said
    // so the moment the frame was narrowed. Tying the tower top back instead
    // reacts the upper arm's loads and leaves the lower arm's plane empty.
    let tower_top = |sign: f64| node_at(0.0, sign * UPPER_PICKUP_Y_MM, UPPER_PIVOT_Z_MM);
    let (brace_left, brace_left_at) = member(tower_top(1.0), apex);
    let (brace_right, brace_right_at) = member(tower_top(-1.0), apex);
    // ── The front suspension ────────────────────────────────────────
    //
    // Double wishbone. The upright is held at two ball joints, and the line
    // through them **is** the steering axis — caster is a geometric fact
    // here rather than a declared axis. Each wishbone picks up on the frame
    // at two points spread fore and aft, which is what gives it a pivot axis
    // instead of a point.
    let lower_ball = |sign: f64| Point3::new(0.0, sign * upright_y, LOWER_BALL_Z_MM);
    let upper_ball =
        |sign: f64| lower_ball(sign) + steering_axis() * (UPPER_BALL_Z_MM - LOWER_BALL_Z_MM);
    // A wishbone's body origin: the midpoint of its two frame pickups, which
    // is a point ON its pivot axis, so the joint anchor and the geometry agree.
    let pivot_mid = |sign: f64, y: f64, z: f64| Vector3::new(0.0, sign * y, z);
    let pickup = |sign: f64, y: f64, z: f64, x: f64| Point3::new(x, sign * y, z);
    let leg = |from: Point3<f64>, to: Point3<f64>, origin: Vector3<f64>| {
        let (p, at) = tube_between(from, to, ARM_OD_MM, ARM_WALL_MM);
        Piece {
            solid: p.solid.translate(at - origin),
            volume_mm3: p.volume_mm3,
        }
    };
    // ⚠ Two parts per wishbone, welded, not one part of two legs: `joined`
    // sums analytic volumes and is only right for pieces that do not overlap.
    // These meet at the ball joint. The swingarm is built the same way.
    let wishbone_leg = |sign: f64, y: f64, z: f64, ball: Point3<f64>, x: f64| {
        leg(pickup(sign, y, z, x), ball, pivot_mid(sign, y, z))
    };
    let tower_base = |sign: f64| Point3::new(0.0, sign * UPPER_PICKUP_Y_MM, FRAME_Z_MM);
    let tower = |sign: f64| {
        tube_from(
            tower_base(sign),
            Point3::new(0.0, sign * UPPER_PICKUP_Y_MM, UPPER_PIVOT_Z_MM),
            TOWER_OD_MM,
            TOWER_WALL_MM,
        )
    };
    let bush = Bushing::from_shore_a(
        BUSH_BORE_R_MM,
        BUSH_OUTER_R_MM,
        BUSH_LENGTH_MM,
        BUSH_SHORE_A,
    );

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
    // On the steering axis at the linkage height, so the steer arm picks up
    // where the upright actually turns.
    let kingpin_pickup = |sign: f64| {
        lower_ball(sign)
            + steering_axis()
                * ((STEER_LINKAGE_Z_MM - LOWER_BALL_Z_MM) / CASTER_DEG.to_radians().cos())
    };
    let arm_end = |sign: f64| {
        Point3::new(
            STEER_ARM_AFT_MM,
            sign * (upright_y - STEER_ARM_INBOARD_MM),
            STEER_LINKAGE_Z_MM,
        )
    };
    let grip = |sign: f64| Point3::new(GRIP_X_MM, sign * GRIP_Y_MM, GRIP_Z_MM);
    let upright_top = upper_ball;
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
    let upright_centre = |sign: f64| lower_ball(sign).coords;

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
    // Out of the rails by half a tube, so the panel sits ON them.
    let back_normal = Vector3::new(-recline.cos(), 0.0, recline.sin());
    let back_panel_at =
        (hip(0.0).coords + back_top(0.0).coords) / 2.0 + back_normal * (SEAT_TUBE_OD_MM / 2.0);

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
    // ⚠ The rider's hip joint is NOT the seat's hip node. `hip()` is where the
    // rails meet — a structural point on the frame — and taking it as the
    // rider's put a 105 mm-radius thigh axis on the pan's centreline, so the
    // legs ran 96 mm through the panel they are supposed to rest on. A person
    // sits ON the pan: the hip joint is a thigh's radius above its face.
    //
    // ★ Its height is set by where the leg TOUCHES, not by the thigh radius.
    // The capsule is shorter than the leg it stands for, so its rear cap hangs
    // below the axis and reaches the pan first; lifting by a plain radius left
    // the rider floating 31 mm above the seat, which the weld gate caught.
    let leg_axis_rise = LEG_RISE_MM / LEG_REACH_MM;
    let cap_back_from_hip = LEG_REACH_MM / 2.0 - LEGS_HALF_MM;
    let rider_hip = Point3::new(
        HIP_X_MM,
        0.0,
        pan_panel_at.z + SEAT_PANEL_HALF_MM + LEGS_RADIUS_MM
            - cap_back_from_hip * leg_axis_rise
            - RIDER_SETTLE_MM,
    );
    // Pedals keep their reach and rise from the hip, so lifting the rider
    // lifts them with it rather than stretching the leg.
    let bottom_bracket = Point3::new(HIP_X_MM - leg_run, 0.0, rider_hip.z + LEG_RISE_MM);
    // ⚠ Clear of the seat back, not in it. This was the rail centreline, so a
    // 170 mm-radius torso was centred in the plane of the panel and half the
    // rider sat behind the seat. The masses and volumes were right throughout;
    // only the position was wrong, which is why every gate passed and it took
    // looking at the assembly to see it.
    //
    // Half a tube to the panel, its own half-thickness to the panel's face,
    // and a torso radius from there.
    let torso_at = (hip(0.0).coords + back_top(0.0).coords) / 2.0
        + back_normal
            * (SEAT_TUBE_OD_MM / 2.0 + SEAT_PANEL_HALF_MM + TORSO_RADIUS_MM - RIDER_SETTLE_MM);
    let legs_at = (rider_hip.coords + bottom_bracket.coords) / 2.0;
    let legs_dir = bottom_bracket - rider_hip;

    // ── The loop the tree cannot hold ───────────────────────────────
    //
    // The tie rod has a rod end at each side. One is its tree joint, on the
    // left arm; the other cannot be, because a tree gives a part one parent.
    // Without this the rod was welded to the left arm and the right wheel
    // steered independently of it.
    let upper_ball_linkage = |sign: f64, tag: &str, upright: &str| {
        LinkageDef::new(
            format!("upper_ball_{tag}"),
            format!("arm_upper_{tag}"),
            upright,
            LinkageKind::Ball,
            Point3::from(
                upper_ball(sign).coords - pivot_mid(sign, UPPER_PICKUP_Y_MM, UPPER_PIVOT_Z_MM),
            ),
        )
    };
    let linkages = vec![
        upper_ball_linkage(1.0, "l", "upright_l"),
        upper_ball_linkage(-1.0, "r", "upright_r"),
        LinkageDef::new(
            "tie_rod_right",
            "tie_rod",
            "steer_arm_r",
            LinkageKind::Ball,
            Point3::from(arm_end(-1.0) - arm_end(1.0)),
        ),
    ];

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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
        },
        PartPlan {
            name: "tower_l",
            parent: "frame_cross",
            anchor_mm: tower_base(1.0).coords - cross_at,
            kind: JointKind::Fixed,
            axis: Vector3::z(),
            range_rad: None,
            material: steel.clone(),
            piece: tower(1.0).0,
            cell_mm: 0.5,
            bushing: None,
        },
        PartPlan {
            name: "arm_lower_l",
            parent: "frame_cross",
            anchor_mm: pivot_mid(1.0, LOWER_PICKUP_Y_MM, LOWER_PIVOT_Z_MM) - cross_at,
            kind: JointKind::Revolute,
            axis: Vector3::x(),
            range_rad: Some((-0.35, 0.35)),
            material: steel.clone(),
            piece: lower_wishbone(1.0),
            cell_mm: 0.4,
            bushing: Some(bush),
        },
        PartPlan {
            name: "arm_upper_l",
            parent: "tower_l",
            anchor_mm: pivot_mid(1.0, UPPER_PICKUP_Y_MM, UPPER_PIVOT_Z_MM) - tower_base(1.0).coords,
            kind: JointKind::Revolute,
            axis: Vector3::x(),
            range_rad: Some((-0.35, 0.35)),
            material: steel.clone(),
            piece: wishbone_leg(
                1.0,
                UPPER_PICKUP_Y_MM,
                UPPER_PIVOT_Z_MM,
                upper_ball(1.0),
                -ARM_PICKUP_HALF_SPREAD_MM,
            ),
            cell_mm: 0.4,
            bushing: Some(bush),
        },
        PartPlan {
            name: "arm_upper_l_aft",
            parent: "arm_upper_l",
            anchor_mm: Vector3::zeros(),
            kind: JointKind::Fixed,
            axis: Vector3::x(),
            range_rad: None,
            material: steel.clone(),
            piece: wishbone_leg(
                1.0,
                UPPER_PICKUP_Y_MM,
                UPPER_PIVOT_Z_MM,
                upper_ball(1.0),
                ARM_PICKUP_HALF_SPREAD_MM,
            ),
            cell_mm: 0.4,
            bushing: None,
        },
        PartPlan {
            name: "upright_l",
            parent: "arm_lower_l",
            anchor_mm: lower_ball(1.0).coords - pivot_mid(1.0, LOWER_PICKUP_Y_MM, LOWER_PIVOT_Z_MM),
            kind: JointKind::Ball,
            axis: steering_axis(),
            range_rad: None,
            material: steel.clone(),
            piece: front_upright(1.0),
            cell_mm: 0.5,
            bushing: None,
        },
        PartPlan {
            name: "tower_r",
            parent: "frame_cross",
            anchor_mm: tower_base(-1.0).coords - cross_at,
            kind: JointKind::Fixed,
            axis: Vector3::z(),
            range_rad: None,
            material: steel.clone(),
            piece: tower(-1.0).0,
            cell_mm: 0.5,
            bushing: None,
        },
        PartPlan {
            name: "arm_lower_r",
            parent: "frame_cross",
            anchor_mm: pivot_mid(-1.0, LOWER_PICKUP_Y_MM, LOWER_PIVOT_Z_MM) - cross_at,
            kind: JointKind::Revolute,
            axis: Vector3::x(),
            range_rad: Some((-0.35, 0.35)),
            material: steel.clone(),
            piece: lower_wishbone(-1.0),
            cell_mm: 0.4,
            bushing: Some(bush),
        },
        PartPlan {
            name: "arm_upper_r",
            parent: "tower_r",
            anchor_mm: pivot_mid(-1.0, UPPER_PICKUP_Y_MM, UPPER_PIVOT_Z_MM)
                - tower_base(-1.0).coords,
            kind: JointKind::Revolute,
            axis: Vector3::x(),
            range_rad: Some((-0.35, 0.35)),
            material: steel.clone(),
            piece: wishbone_leg(
                -1.0,
                UPPER_PICKUP_Y_MM,
                UPPER_PIVOT_Z_MM,
                upper_ball(-1.0),
                -ARM_PICKUP_HALF_SPREAD_MM,
            ),
            cell_mm: 0.4,
            bushing: Some(bush),
        },
        PartPlan {
            name: "arm_upper_r_aft",
            parent: "arm_upper_r",
            anchor_mm: Vector3::zeros(),
            kind: JointKind::Fixed,
            axis: Vector3::x(),
            range_rad: None,
            material: steel.clone(),
            piece: wishbone_leg(
                -1.0,
                UPPER_PICKUP_Y_MM,
                UPPER_PIVOT_Z_MM,
                upper_ball(-1.0),
                ARM_PICKUP_HALF_SPREAD_MM,
            ),
            cell_mm: 0.4,
            bushing: None,
        },
        PartPlan {
            name: "upright_r",
            parent: "arm_lower_r",
            anchor_mm: lower_ball(-1.0).coords
                - pivot_mid(-1.0, LOWER_PICKUP_Y_MM, LOWER_PIVOT_Z_MM),
            kind: JointKind::Ball,
            axis: steering_axis(),
            range_rad: None,
            material: steel.clone(),
            piece: front_upright(-1.0),
            cell_mm: 0.5,
            bushing: None,
        },
        PartPlan {
            name: "rim_fl",
            parent: "upright_l",
            anchor_mm: Vector3::new(
                0.0,
                TRACK_MM / 2.0 - upright_y,
                FRONT_RADIUS_MM - LOWER_BALL_Z_MM,
            ),
            kind: JointKind::Revolute,
            axis: Vector3::y(),
            range_rad: None,
            material: aluminium.clone(),
            piece: joined(vec![
                onto_y(annulus(FRONT_RIM_OUTER_MM, FRONT_RIM_INNER_MM, 15.0)),
                onto_y(disc(FRONT_HUB_R_MM, FRONT_WHEEL_HALF_WIDTH_MM)),
            ])?,
            // ⚠ 1.0, not 2.0: the rim section is 6 mm, and three cells across
            // a wall put the integrator 0.688% off its closed form.
            cell_mm: 1.0,
            bushing: None,
        },
        PartPlan {
            name: "rim_fr",
            parent: "upright_r",
            anchor_mm: Vector3::new(
                0.0,
                upright_y - TRACK_MM / 2.0,
                FRONT_RADIUS_MM - LOWER_BALL_Z_MM,
            ),
            kind: JointKind::Revolute,
            axis: Vector3::y(),
            range_rad: None,
            material: aluminium.clone(),
            piece: joined(vec![
                onto_y(annulus(FRONT_RIM_OUTER_MM, FRONT_RIM_INNER_MM, 15.0)),
                onto_y(disc(FRONT_HUB_R_MM, FRONT_WHEEL_HALF_WIDTH_MM)),
            ])?,
            // ⚠ 1.0, not 2.0: the rim section is 6 mm, and three cells across
            // a wall put the integrator 0.688% off its closed form.
            cell_mm: 1.0,
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
            bushing: None,
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
        let joint = match p.range_rad {
            Some((lo, hi)) => joint.with_range(lo, hi),
            None => joint,
        };
        // A bushing is a rate, and a rate the model can feel — see
        // `Bushing::joint_stiffness` for why the conversion is not optional.
        // Damping is a fiftieth of it: enough to settle the arm without
        // pretending this is a characterised loss factor.
        builder = builder.joint(match p.bushing {
            Some(b) => joint
                .with_stiffness(b.joint_stiffness())
                .with_damping(b.joint_stiffness() / 50.0),
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
            // ⚠ `None` is allowed and means "authored geometry, no elementary
            // volume"; a *present* volume still has to be a real one.
            if let Some(v) = m.volume_mm3 {
                assert!(v.is_finite() && v > 0.0, "{name} has volume {v}");
            }
            assert!(
                m.cell_mm.is_finite() && m.cell_mm > 0.0,
                "{name} has cell {}",
                m.cell_mm
            );
        }
    }

    /// The bearing bore goes through the upright, and there is metal round it.
    ///
    /// ⚠ This replaced a check that the upright's outer wall met the wheel's
    /// inboard face. That was a statement about a 25.4 mm tube, and the tube
    /// is gone — the upright is authored now, so the question is whether the
    /// shape it describes is a housing or a lump.
    ///
    /// ★ Measured on the solid, not on the constants that built it. Asserting
    /// `BEARING_BORE_R_MM < BEARING_BOSS_R_MM` is arithmetic the compiler
    /// could do; whether a subtracted cylinder actually clears both faces is
    /// not, and a bore that stops short is a blind pocket you cannot get an
    /// axle through.
    #[test]
    fn the_bearing_bore_runs_right_through_the_upright() {
        let t = trike().unwrap();
        let upright = t
            .mechanism
            .parts()
            .iter()
            .find(|p| p.name() == "upright_l")
            .unwrap();
        let solid = upright.solid();
        let wheel_y = KINGPIN_OFFSET_MM;
        let wheel_z = FRONT_RADIUS_MM - LOWER_BALL_Z_MM;

        // Along the axle, from one face of the boss to the other: all air.
        for k in -10..=10 {
            let y = wheel_y + f64::from(k) / 10.0 * BEARING_BOSS_HALF_MM;
            let at = Point3::new(0.0, y, wheel_z);
            assert!(
                solid.evaluate(&at) > 0.0,
                "the bore is blocked at y {y:.1} — it is a pocket, not a bore"
            );
        }

        // And a ring of metal around it, or the bore has eaten the housing.
        let mid_r = f64::midpoint(BEARING_BORE_R_MM, BEARING_BOSS_R_MM);
        for (dx, dz) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
            let at = Point3::new(dx * mid_r, wheel_y, wheel_z + dz * mid_r);
            assert!(
                solid.evaluate(&at) < 0.0,
                "no metal at {at:?} — the housing is a rim of nothing"
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

    /// The exported file describes the same machine the model does.
    ///
    /// ⚠ This reads the artifact, not the builder. `to_mjcf` once wrote all
    /// 28 mesh assets into an empty `<worldbody>` — every check that asked
    /// the `Mechanism` what it held was green throughout, because the
    /// `Mechanism` was right and only the file was wrong.
    ///
    /// ⚠ Structure only. At 20 mm, eight parts thinner than the cell mesh to
    /// nothing and their `<mesh>` assets come out empty, which MuJoCo will
    /// not load; 4 mm leaves none empty and costs 20 MB. This asserts the
    /// body tree, not that the file compiles.
    #[test]
    fn the_exported_file_has_a_body_for_every_part() {
        let t = trike().unwrap();
        let xml = t.mechanism.to_mjcf(20.0);

        for part in t.mechanism.parts() {
            assert!(
                xml.contains(&format!("<body name=\"{}\"", part.name())),
                "{} has a mesh asset but no body",
                part.name()
            );
        }
        assert_eq!(
            xml.matches("<freejoint").count(),
            1,
            "the frame reaches the world exactly once"
        );
    }

    /// Every loop closes onto parts that exist, and each takes three dof.
    ///
    /// Three of them: the tie rod's far end, and the upper ball joint on each
    /// wishbone — a double wishbone is two loops, because the upright is held
    /// by two arms and a tree gives it one parent.
    #[test]
    fn the_linkage_ties_two_parts_that_exist() {
        let t = trike().unwrap();
        let names: Vec<&str> = t.mechanism.parts().iter().map(Part::name).collect();

        assert_eq!(t.mechanism.linkages().len(), 3);
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

    /// The four inboard wishbone pivots are bushed, and nothing else is.
    ///
    /// ⚠ The rate is checked as a **physical** quantity — newton-metres per
    /// radian, recovered from the model's own units — because that is the
    /// number a suspension engineer would recognise. A bushing handed over in
    /// N·m/rad rather than through `joint_stiffness` lands a million times
    /// softer, and a test that only asked "is there a stiffness?" would pass.
    #[test]
    fn the_wishbone_pivots_are_bushed_and_nothing_else_is() {
        let t = trike().unwrap();
        let bushed: Vec<&str> = t
            .mechanism
            .joints()
            .iter()
            .filter(|j| j.stiffness().is_some_and(|k| k > 0.0))
            .map(cf_design::JointDef::child)
            .collect();

        assert_eq!(
            bushed.len(),
            4,
            "expected the four inboard pivots, got {bushed:?}"
        );
        for side in ["l", "r"] {
            for height in ["lower", "upper"] {
                let want = format!("arm_{height}_{side}");
                assert!(bushed.contains(&want.as_str()), "{want} is not bushed");
            }
        }

        for j in t.mechanism.joints() {
            let Some(k) = j.stiffness() else { continue };
            // Model torque is kg·mm²/s², a millionth of a newton-metre.
            let n_m_per_rad = k / 1e6;
            assert!(
                (20.0..2000.0).contains(&n_m_per_rad),
                "{} carries {n_m_per_rad:.1} N·m/rad, which is not a \
                 suspension bushing — a rate this far out is the unit \
                 conversion, not the design",
                j.child()
            );
        }
    }

    /// The rider sits against the seat back, not inside it.
    ///
    /// ⚠ This is the one defect on this vehicle that no number could show.
    /// The torso was centred on the seat-back rails, so a 170 mm-radius
    /// capsule had half of itself behind the panel. Its mass was right, its
    /// volume was right to 0.008%, every weld and clash and simulation gate
    /// passed — and the rider was 186 mm inside the seat, which put the
    /// centre of gravity 69 mm too low and the rollover threshold 0.11 g too
    /// high. It took rendering the assembly and looking at it.
    #[test]
    fn the_rider_clears_the_seat_back() {
        let t = trike().unwrap();
        let recline = SEAT_BACK_ANGLE_DEG.to_radians();
        let normal = Vector3::new(-recline.cos(), 0.0, recline.sin());
        let gap = (t.origins["rider_torso"] - t.origins["seat_back"]).dot(&normal);

        let want = TORSO_RADIUS_MM + SEAT_PANEL_HALF_MM - RIDER_SETTLE_MM;
        assert!(
            gap >= want - 1e-9,
            "the torso axis is {gap:.1} mm off the seat back and needs \
             {want:.1} — the rider is {:.0} mm inside the seat",
            want - gap
        );
    }

    /// The legs rest on the seat pan rather than passing through it.
    ///
    /// Same defect as the torso, and found the same way. The seat's hip node
    /// is a point on the frame where the rails meet; taking it as the rider's
    /// hip joint put a 105 mm-radius thigh axis on the pan's centreline, so
    /// the legs ran 96 mm through the panel they sit on.
    ///
    /// ⚠ Checked from the placement, not from the derivation: the capsule's
    /// rear cap is the part that reaches the pan, and this asks where that cap
    /// ended up. The floating case — the other way to get this wrong — is
    /// caught downstream by the weld oracle, which fired at 31 mm.
    #[test]
    fn the_legs_rest_on_the_seat_pan() {
        let t = trike().unwrap();
        let rise = LEG_RISE_MM / LEG_REACH_MM;
        let cap_centre_z = t.origins["rider_legs"].z - LEGS_HALF_MM * rise;
        let pan_face_z = t.origins["seat_pan"].z + SEAT_PANEL_HALF_MM;

        let stand_off = cap_centre_z - pan_face_z;
        let want = LEGS_RADIUS_MM - RIDER_SETTLE_MM;
        assert!(
            stand_off >= want - 1e-9,
            "the thigh axis stands {stand_off:.1} mm off the pan and needs \
             {want:.1} — the legs are {:.0} mm into the seat",
            want - stand_off
        );
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
            assert_eq!(m.volume_mm3, n.volume_mm3, "{name} differs between builds");
            assert!(
                (m.cell_mm - n.cell_mm).abs() < f64::EPSILON,
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
