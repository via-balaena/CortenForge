//! The reverse trike's mass budget, **derived from geometry** instead of typed.
//!
//! `cf-vehicle` takes a mass budget and derives every load from it, but the
//! budget itself was five hand-typed estimates. This example is the other
//! half: it expresses the trike as a [`Mechanism`] — the assembly primitive
//! that already exists in `cf-design` — and composes a budget out of
//! [`mass_properties`], so the numbers come from shapes.
//!
//! Run with: `cargo run --release -p example-trike-mass-budget`
//!
//! # The layering this deliberately preserves
//!
//! `cf-vehicle` declares **no dependencies at all** — it stays geometry-free
//! and takes a budget — and this arc does not give it one. This crate is the
//! composition point, which is why it lives in `examples/integration` rather
//! than inside either library.
//!
//! # What the oracle checks
//!
//! 1. Every part's grid-integrated mass matches its **closed-form** mass. The
//!    closed form is arithmetic on the same dimensions, evaluated without
//!    touching the integrator, so what it checks is the **integrator**: a unit
//!    slip, a wrong density, a cell too coarse to resolve a tube wall.
//!    ⚠ It does **not** catch a wrong dimension: the closed form is built from
//!    the same `r_outer` and `r_inner` as the solid, so a wrong one moves both
//!    and the check stays satisfied. Dimensions are covered by (2) for the
//!    three that set the contact patches and by (3) as a regression, and the
//!    rim/tyre mating radii are shared constants so that pair cannot drift at
//!    all.
//! 2. The wheelbase and track **read back out of the joint anchors** equal the
//!    nominal numbers `cf-vehicle` was given. Placement is a chain of anchors;
//!    a single mis-typed offset moves a contact patch.
//! 3. The derived budget makes a well-formed [`TrikeSpec`], and its rollover
//!    threshold is pinned.
//!
//! # Conventions
//!
//! Geometry is millimetres, as everywhere in `cf-design`; `cf-vehicle` is
//! metres. `x` is aft of the front contact patch, `y` is lateral, `z` is up
//! from the ground — the same frame `cf-vehicle` documents, extended to 3D.
//!
//! # What building this found that reading could not
//!
//! - ✅ **There was no `Fixed`/weld joint.** [`JointKind`] was Revolute,
//!   Prismatic, Ball or Free, and `with_range` *panics* when `min >= max`, so
//!   a zero-range revolute was rejected too. A trike frame is a weldment, so
//!   every welded member here carried a 1e-9 radian revolute — a real degree
//!   of freedom the solver still solved. [`JointKind::Fixed`] now exists and
//!   this example uses it; the six welds cost the solver nothing.
//! - ✅ **A `Part` carries exactly one `Material`**, so a rim and the tyre
//!   moulded onto it cannot be one part. Welded, they are two parts and one
//!   rigid body — the same fix, which is why they were one gap and not two.
//! - ⚠ **`mass_properties` grids the solid's AABB at a uniform cell**, so cost
//!   follows the bounding box and the thinnest feature, not the amount of
//!   material. *Where* a part sits is free — `bounds.rs:185` shifts a
//!   translated box without growing it — but *how many parts* it is is not.
//!   Every run prints the comparison, and the size of it depends on how the
//!   members lie. Two axis-aligned members — a spine and a cross — cost 15x
//!   less split than merged. Add the two **diagonals** and the advantage falls
//!   to 2x: a diagonal tube's bounding box is the box of the rotated tube, so
//!   each diagonal already spans most of the frame and splitting buys much
//!   less. ⇒ Splitting a weldment and welding it is still the cheaper option,
//!   but "an order of magnitude" was only true while every member ran along an
//!   axis.
//! - ⚠ **Nothing aggregates an assembly.** `subtree_com[0]` is the whole-model
//!   centre of mass, but it exists only after `to_model` plus a forward
//!   kinematics pass. [`world_origins`] below is that walk, done directly on
//!   the joint anchors, and is the thing to extract if this shape proves out.

// ⚠ `similar_names` fires on every left/right pair. A symmetric vehicle has
// symmetric members, and naming them anything but left and right to satisfy a
// lint would make the geometry harder to read, not easier.
#![allow(clippy::too_many_lines, clippy::similar_names)]

use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, PI};

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use cf_design::mechanism::mass::mass_properties;
use cf_design::{Aabb, IndexedMesh, JointDef, JointKind, Material, Mechanism, Part, Solid};
use cf_vehicle::analysis::rollover_threshold_g;
use cf_vehicle::{CorneringLoads, MassItem, StaticLoads, TrikeSpec};
use nalgebra::{Point3, UnitQuaternion, Vector3};

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
const WHEELBASE_MM: f64 = 1250.0;
/// Centre to centre of the two front wheels.
const TRACK_MM: f64 = 900.0;
/// 16″ front.
const FRONT_RADIUS_MM: f64 = 203.2;
/// 11″ rear — the polyurethane one.
const REAR_RADIUS_MM: f64 = 140.0;
/// Caster: the steering axis leans this far back from vertical.
const CASTER_DEG: f64 = 8.0;

/// Spine and cross-member centreline height.
const FRAME_Z_MM: f64 = 150.0;
/// Frame tube outside diameter.
const FRAME_OD_MM: f64 = 31.75;
/// Frame tube wall.
const FRAME_WALL_MM: f64 = 2.0;
/// Half the frame tube's outside diameter — where members butt onto it.
const FRAME_R_MM: f64 = FRAME_OD_MM / 2.0;

/// The spine stops this far short of the rear contact patch; the swingarm
/// carries the rest.
const TAIL_SETBACK_MM: f64 = 100.0;

/// Seat tube stock — lighter than the frame, it carries a person not a kerb.
const SEAT_TUBE_OD_MM: f64 = 25.4;
/// Seat tube wall.
const SEAT_TUBE_WALL_MM: f64 = 1.5;
/// Rail centres sit this far apart.
const SEAT_WIDTH_MM: f64 = 300.0;
/// Hip point — where the pan meets the back, and where the seat's load goes
/// into the spine. Set by leg length from the bottom bracket, not by taste.
const HIP_X_MM: f64 = 674.0;
/// Hip height above the ground.
const HIP_Z_MM: f64 = 210.0;
/// Pan length forward of the hip.
const PAN_LENGTH_MM: f64 = 250.0;
/// Seat back length from the hip.
const SEAT_BACK_LENGTH_MM: f64 = 500.0;
/// Recline, measured from vertical. A cruiser sits up more than a racer.
const SEAT_BACK_ANGLE_DEG: f64 = 45.0;
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
/// Where the front rim ends and its tyre begins. Shared by both so the two
/// cannot drift apart into a gap or an interpenetration — a class the
/// closed-form check above is blind to.
const FRONT_RIM_OUTER_MM: f64 = 180.0;
/// Where the rear rim ends and the cast polyurethane tyre begins.
const REAR_RIM_OUTER_MM: f64 = 115.0;
/// Half the rear wheel's width — rim and tyre are the same width.
const REAR_WHEEL_HALF_WIDTH_MM: f64 = 12.5;

/// How far a grid-integrated mass may sit from its closed form.
///
/// Set just above the worst part measured (0.318%, `rim_fr`), so a coarsened
/// cell trips it rather than passing quietly. ⚠ A changed *dimension* does not
/// trip it — see the note on oracle 1.
const MASS_TOLERANCE: f64 = 0.005;

/// Welds in the assembly: three frame members and seven seat members onto the
/// spine, all three tyres onto their rims, and the rider's two halves.
const EXPECTED_WELDS: usize = 15;

/// Degrees of freedom the machine actually has: the free body, two steering
/// pivots, three wheels spinning, and the swingarm.
const EXPECTED_DOF: usize = 12;

/// Parts the plan is expected to produce.
///
/// ⚠ Asserted because every oracle below walks a collection, and a walk over
/// an empty one passes without doing anything. An empty `Mechanism` builds
/// happily — `validate` skips the orphan check below two parts — so nothing
/// upstream would object.
const EXPECTED_PARTS: usize = 22;

/// Members welded into the frame, whose grid cost is compared: spine,
/// cross-member and the two diagonals.
const WELDED_FRAME_MEMBERS: usize = 4;

/// How far to move the heaviest item's centre of mass when probing how much
/// of the answer is a choice rather than a measurement.
const CG_PROBE_MM: f64 = 50.0;

/// Floor for the refinement in [`export_stls`]. A part still empty here has a
/// feature finer than a third of a millimetre and wants saying so, not
/// halving again.
const MIN_STL_TOLERANCE_MM: f64 = 0.25;

/// Default meshing tolerance for `--out`, in millimetres.
///
/// ⚠ This is for *looking at* the vehicle, not for printing it. At 1.0 mm the
/// assembly comes to 6.4 M triangles and ~300 MB of STL — the rider capsule
/// alone is 2.8 M — because the tolerance is a cell size and the vehicle is
/// 1.25 m long. Override with `--tolerance` when a wall section matters.
const STL_TOLERANCE_MM: f64 = 4.0;

/// Polyurethane on asphalt, at the optimistic end of 0.6-1.0.
const TYRE_MU: f64 = 1.0;

/// How far the composed budget may drift before the pins below fire.
///
/// Loose enough to survive a last-ulp libm difference between platforms,
/// tight enough that any real change of geometry is caught: 1e-6 of 90 kg is
/// 90 mg, and 1e-6 of 0.315 m is 0.3 µm.
const PIN_TOLERANCE: f64 = 1e-6;

// ── Pieces: a solid and its closed-form volume, built together ──────────

/// A primitive together with its analytic volume.
///
/// The volume is arithmetic on the same dimensions that built the solid, and
/// never goes through [`mass_properties`] — that is the whole point of
/// carrying it.
struct Piece {
    solid: Solid,
    volume_mm3: f64,
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

/// Z-aligned tube, centred at the origin.
fn tube(od: f64, wall: f64, length: f64) -> Piece {
    let r_outer = od / 2.0;
    let r_inner = r_outer - wall;
    Piece {
        solid: Solid::cylinder(r_outer, length / 2.0).subtract(Solid::cylinder(r_inner, length)),
        volume_mm3: PI * (r_outer * r_outer - r_inner * r_inner) * length,
    }
}

/// Z-aligned annulus, centred at the origin.
fn annulus(r_outer: f64, r_inner: f64, half_width: f64) -> Piece {
    Piece {
        solid: Solid::cylinder(r_outer, half_width)
            .subtract(Solid::cylinder(r_inner, half_width * 2.0)),
        volume_mm3: PI * (r_outer * r_outer - r_inner * r_inner) * 2.0 * half_width,
    }
}

/// Z-aligned solid disc, centred at the origin.
fn disc(radius: f64, half_width: f64) -> Piece {
    Piece {
        solid: Solid::cylinder(radius, half_width),
        volume_mm3: PI * radius * radius * 2.0 * half_width,
    }
}

/// Axis-aligned box, centred at the origin.
fn slab(half: Vector3<f64>) -> Piece {
    Piece {
        solid: Solid::cuboid(half),
        volume_mm3: 8.0 * half.x * half.y * half.z,
    }
}

/// Z-aligned capsule, centred at the origin.
fn capsule(radius: f64, half_height: f64) -> Piece {
    Piece {
        solid: Solid::capsule(radius, half_height),
        volume_mm3: PI * radius * radius * 2.0 * half_height
            + 4.0 / 3.0 * PI * radius * radius * radius,
    }
}

/// Turn a Z-aligned piece into a Y-aligned one — wheels and cross-members.
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
fn tilted(p: Piece, angle_rad: f64) -> Piece {
    Piece {
        solid: p.solid.rotate(UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            angle_rad,
        )),
        volume_mm3: p.volume_mm3,
    }
}

/// Turn a Z-aligned piece into an X-aligned one — the spine and the swingarm.
fn onto_x(p: Piece) -> Piece {
    Piece {
        solid: p.solid.rotate(UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            FRAC_PI_2,
        )),
        volume_mm3: p.volume_mm3,
    }
}

/// Move a piece within its own part frame.
fn shifted(p: Piece, offset: Vector3<f64>) -> Piece {
    Piece {
        solid: p.solid.translate(offset),
        volume_mm3: p.volume_mm3,
    }
}

/// Union of **disjoint** pieces: the analytic volume is their sum, which is
/// true only because nothing here overlaps anything else in the same part.
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
    /// Grid spacing for [`mass_properties`]. A tube wall needs roughly four
    /// cells across it; a capsule needs nothing like that.
    cell_mm: f64,
}

/// The steering axis, leaning `CASTER_DEG` back from vertical.
fn steering_axis() -> Vector3<f64> {
    let caster = CASTER_DEG.to_radians();
    Vector3::new(caster.sin(), 0.0, caster.cos())
}

/// Build the part table, in tree order.
fn plan() -> Result<Vec<PartPlan>> {
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
    let spine_x = spine_at.x;
    let upright_length = 74.125;
    let upright_z = FRAME_Z_MM + FRAME_R_MM + upright_length / 2.0;

    let swingarm_length = 350.0;
    let swingarm_x = WHEELBASE_MM - swingarm_length / 2.0;

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

    Ok(vec![
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
                onto_y(annulus(FRONT_RIM_OUTER_MM, 160.0, 15.0)),
                onto_y(disc(30.0, FRONT_WHEEL_HALF_WIDTH_MM)),
            ])?,
            cell_mm: 2.0,
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
                onto_y(annulus(FRONT_RIM_OUTER_MM, 160.0, 15.0)),
                onto_y(disc(30.0, FRONT_WHEEL_HALF_WIDTH_MM)),
            ])?,
            cell_mm: 2.0,
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
            anchor_mm: Vector3::new(swingarm_x - spine_x, 0.0, REAR_RADIUS_MM - FRAME_Z_MM),
            kind: JointKind::Revolute,
            axis: Vector3::y(),
            range_rad: Some((-0.35, 0.35)),
            material: steel.clone(),
            piece: joined(vec![
                shifted(
                    onto_x(tube(25.4, 2.0, swingarm_length)),
                    Vector3::new(0.0, 60.0, 0.0),
                ),
                shifted(
                    onto_x(tube(25.4, 2.0, swingarm_length)),
                    Vector3::new(0.0, -60.0, 0.0),
                ),
            ])?,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "rim_r",
            parent: "swingarm",
            anchor_mm: Vector3::new(WHEELBASE_MM - swingarm_x, 0.0, 0.0),
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
            parent: "frame_spine",
            anchor_mm: pan_rail_left_at - spine_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: pan_rail_left,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "seat_pan_rail_right",
            parent: "frame_spine",
            anchor_mm: pan_rail_right_at - spine_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: pan_rail_right,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "seat_back_rail_left",
            parent: "frame_spine",
            anchor_mm: back_rail_left_at - spine_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel.clone(),
            piece: back_rail_left,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "seat_back_rail_right",
            parent: "frame_spine",
            anchor_mm: back_rail_right_at - spine_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: steel,
            piece: back_rail_right,
            cell_mm: 0.5,
        },
        PartPlan {
            name: "seat_pan",
            parent: "frame_spine",
            anchor_mm: pan_panel_at - spine_at,
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
            parent: "frame_spine",
            anchor_mm: back_panel_at - spine_at,
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
            parent: "frame_spine",
            anchor_mm: torso_at - spine_at,
            kind: JointKind::Fixed,
            axis: Vector3::y(),
            range_rad: None,
            material: Material::new("rider torso", RIDER_KG_M3),
            piece: tilted(capsule(TORSO_RADIUS_MM, TORSO_HALF_MM), FRAC_PI_2 - recline),
            cell_mm: 4.0,
        },
        PartPlan {
            name: "rider_legs",
            parent: "frame_spine",
            anchor_mm: legs_at - spine_at,
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
    ])
}

/// Turn the plan into a validated [`Mechanism`], consuming the solids.
fn assemble(plan: Vec<PartPlan>) -> Result<Mechanism> {
    let mut builder = Mechanism::builder("reverse trike");
    for p in plan {
        builder = builder.part(Part::new(p.name, p.piece.solid, p.material));
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
fn world_origins(mechanism: &Mechanism) -> Result<HashMap<String, Vector3<f64>>> {
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

// ── Derivation ──────────────────────────────────────────────────────────

/// Cells [`mass_properties`] evaluates for this box at this spacing.
///
/// The same arithmetic it does internally — it expands the bounds by half a
/// cell each side, then takes `ceil(size / cell)` per axis — so this is the
/// real cost, not an estimate of it.
fn grid_cells(bounds: &Aabb, cell_mm: f64) -> f64 {
    let size = bounds.max - bounds.min;
    ((size.x + cell_mm) / cell_mm).ceil()
        * ((size.y + cell_mm) / cell_mm).ceil()
        * ((size.z + cell_mm) / cell_mm).ceil()
}

/// Smallest box containing both.
fn merged(a: &Aabb, b: &Aabb) -> Aabb {
    Aabb::new(
        Point3::from(a.min.coords.inf(&b.min.coords)),
        Point3::from(a.max.coords.sup(&b.max.coords)),
    )
}

/// One part's mass, both ways, where its centre of mass sits, and what it
/// cost to find out.
struct Derived {
    name: String,
    grid_kg: f64,
    closed_form_kg: f64,
    world_com_mm: Vector3<f64>,
    /// The part's box, placed in the world frame.
    world_bounds: Aabb,
    cell_mm: f64,
}

impl Derived {
    /// How far the grid integrator sits from the closed form.
    fn relative_error(&self) -> f64 {
        (self.grid_kg - self.closed_form_kg).abs() / self.closed_form_kg
    }
}

/// Integrate every part and place its centre of mass in the world frame.
fn derive(
    mechanism: &Mechanism,
    cells: &HashMap<&'static str, (f64, f64)>,
    origins: &HashMap<String, Vector3<f64>>,
) -> Result<Vec<Derived>> {
    let mut out = Vec::new();
    for part in mechanism.parts() {
        let Some(&(volume_mm3, cell_mm)) = cells.get(part.name()) else {
            bail!("no cell size recorded for part {}", part.name());
        };
        let Some(&origin) = origins.get(part.name()) else {
            bail!("no world origin resolved for part {}", part.name());
        };
        let density = part.material().density;
        let Some(props) = mass_properties(part.solid(), density, cell_mm) else {
            bail!("mass_properties found no interior for part {}", part.name());
        };
        let Some(local) = part.solid().bounds() else {
            bail!("part {} has no finite bounds", part.name());
        };
        out.push(Derived {
            name: part.name().to_owned(),
            grid_kg: props.mass,
            closed_form_kg: volume_mm3 * 1e-9 * density,
            world_com_mm: origin + props.center_of_mass.coords,
            world_bounds: Aabb::new(
                Point3::from(local.min.coords + origin),
                Point3::from(local.max.coords + origin),
            ),
            cell_mm,
        });
    }
    Ok(out)
}

// ── Looking at it ───────────────────────────────────────────────────────

/// Mesh every part and write it to `dir` as an STL, one file per part.
///
/// ⚠ Opt-in via `--out <dir>`. `xtask run-validators` invokes this example
/// with **no arguments**, and a validator that writes files on every CI run
/// would leave litter behind; the asserted zero-argument path stays read-only.
///
/// ⚠ **A part can mesh to nothing.** [`Mechanism::to_stl_kit`] meshes every
/// part at one tolerance, and that tolerance is a *cell size*: the 3 mm seat
/// pan vanished entirely at the 4 mm default that suits a 1.25 m frame, and
/// wrote an 84-byte STL containing no triangles — a valid, correctly named,
/// empty file. So each part is meshed at the requested tolerance and only what
/// vanishes is refined, halving down to [`MIN_STL_TOLERANCE_MM`].
///
/// ⚠ Refining *everything* to its mass-integration cell instead was measured
/// at 8.1 M triangles and 388 MB: that cell is chosen for integration
/// accuracy, and a 2 mm wall does not need 0.5 mm triangles to look right.
fn export_stls(
    mechanism: &Mechanism,
    origins: &HashMap<String, Vector3<f64>>,
    dir: &Path,
    tolerance_mm: f64,
) -> Result<()> {
    std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    let mut assembly = IndexedMesh::default();
    let mut total = 0usize;

    for part in mechanism.parts() {
        // Mesh at what was asked for, and refine only what vanishes. The
        // tolerance is a cell size, so a part thinner than one cell meshes to
        // nothing at all — a silent, correctly named, empty file.
        let mut tol = tolerance_mm;
        let mut mesh = part.solid().mesh(tol).geometry;
        while mesh.faces.is_empty() && tol > MIN_STL_TOLERANCE_MM {
            tol /= 2.0;
            mesh = part.solid().mesh(tol).geometry;
        }
        if mesh.faces.is_empty() {
            bail!(
                "part {} meshed to nothing even at {MIN_STL_TOLERANCE_MM} mm — \
                 its thinnest feature is finer than that",
                part.name()
            );
        }

        // ⚠ Place it. A part's solid is in its OWN frame; where it sits is in
        // the joint anchors. Writing the mesh as-meshed puts every part on the
        // origin, so opening the folder shows thirteen parts in a heap rather
        // than a vehicle.
        let Some(&origin) = origins.get(part.name()) else {
            bail!("no world origin resolved for part {}", part.name());
        };
        for v in &mut mesh.vertices {
            *v += origin;
        }

        let base = u32::try_from(assembly.vertices.len())
            .with_context(|| "assembly exceeded u32 vertices")?;
        assembly.vertices.extend(mesh.vertices.iter().copied());
        assembly.faces.extend(
            mesh.faces
                .iter()
                .map(|f| [f[0] + base, f[1] + base, f[2] + base]),
        );

        let path = dir.join(format!("{}.stl", part.name()));
        mesh_io::save_stl(&mesh, &path, true)
            .with_context(|| format!("writing {}", path.display()))?;
        let refined = if tol < tolerance_mm { " (refined)" } else { "" };
        println!(
            "  {:<12} {:>8} triangles at {:>5} mm{refined}",
            part.name(),
            mesh.faces.len(),
            tol,
        );
        total += mesh.faces.len();
    }

    // One file with the whole thing in it, so "look at the trike" is a
    // single open rather than thirteen.
    let whole = dir.join("trike_assembled.stl");
    mesh_io::save_stl(&assembly, &whole, true)
        .with_context(|| format!("writing {}", whole.display()))?;

    let (lo, hi) = bounds_of(&assembly)?;
    println!(
        "  {total} triangles -> {}\n  assembled: {} spans x {:.0}..{:.0}  y {:.0}..{:.0}  z {:.0}..{:.0} mm",
        dir.display(),
        whole.file_name().unwrap_or_default().to_string_lossy(),
        lo.x,
        hi.x,
        lo.y,
        hi.y,
        lo.z,
        hi.z
    );

    // The assembly must actually span the vehicle. If placement silently
    // regressed, every part would sit on the origin and this would collapse.
    let span_x = hi.x - lo.x;
    if span_x < WHEELBASE_MM * 0.9 {
        bail!(
            "the assembled mesh spans only {span_x:.0} mm in x, but the wheelbase \
             is {WHEELBASE_MM} mm — the parts are not placed"
        );
    }
    Ok(())
}

/// Axis-aligned extent of a mesh.
fn bounds_of(mesh: &IndexedMesh) -> Result<(Vector3<f64>, Vector3<f64>)> {
    let Some(first) = mesh.vertices.first() else {
        bail!("cannot bound an empty mesh");
    };
    let mut lo = first.coords;
    let mut hi = first.coords;
    for v in &mesh.vertices {
        lo = lo.inf(&v.coords);
        hi = hi.sup(&v.coords);
    }
    Ok((lo, hi))
}

// ── Entry point ─────────────────────────────────────────────────────────

fn main() -> Result<()> {
    // ⚠ Arguments are optional and the zero-argument path is the asserted one.
    let args: Vec<String> = std::env::args().skip(1).collect();
    let flag = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1))
            .cloned()
    };
    let out_dir = flag("--out").map(PathBuf::from);
    let tolerance_mm = match flag("--tolerance") {
        Some(t) => t
            .parse::<f64>()
            .with_context(|| format!("--tolerance {t} is not a number"))?,
        None => STL_TOLERANCE_MM,
    };
    if !(tolerance_mm > 0.0 && tolerance_mm.is_finite()) {
        bail!("--tolerance must be positive and finite, got {tolerance_mm}");
    }

    let plan = plan()?;
    let cells: HashMap<&'static str, (f64, f64)> = plan
        .iter()
        .map(|p| (p.name, (p.piece.volume_mm3, p.cell_mm)))
        .collect();

    let mechanism = assemble(plan)?;
    let origins = world_origins(&mechanism)?;
    let derived = derive(&mechanism, &cells, &origins)?;

    if derived.len() != EXPECTED_PARTS {
        bail!(
            "derived {} parts, expected {EXPECTED_PARTS} — every oracle below \
             walks this collection, and a short walk passes quietly",
            derived.len()
        );
    }

    // ── Oracle 0: the articulation is what it is meant to be ────────
    //
    // Welds are free: `JointKind::Fixed` emits no joint and no coordinate, so
    // the six here cost the solver nothing. They were 1e-9 rad revolutes
    // before cf-design grew a weld, and those were six real degrees of
    // freedom pretending to be none.
    let welds = mechanism
        .joints()
        .iter()
        .filter(|j| j.kind().is_weld())
        .count();
    let dof: usize = mechanism.joints().iter().map(|j| j.kind().dof()).sum();
    println!(
        "reverse trike — {} parts, {welds} welds, {dof} dof",
        mechanism.parts().len()
    );
    if welds != EXPECTED_WELDS {
        bail!("{welds} welds, expected {EXPECTED_WELDS}");
    }
    if dof != EXPECTED_DOF {
        bail!("{dof} degrees of freedom, expected {EXPECTED_DOF}");
    }
    println!();
    println!(
        "{:<12} {:>10} {:>12} {:>9}   {:>8} {:>8} {:>8} {:>7} {:>9}",
        "part", "grid kg", "closed kg", "rel err", "com x", "com y", "com z", "cell", "cells"
    );
    for d in &derived {
        println!(
            "{:<12} {:>10.4} {:>12.4} {:>8.3}% {:>9.1} {:>8.1} {:>8.1} {:>7.1} {:>8.2}M",
            d.name,
            d.grid_kg,
            d.closed_form_kg,
            d.relative_error() * 100.0,
            d.world_com_mm.x,
            d.world_com_mm.y,
            d.world_com_mm.z,
            d.cell_mm,
            grid_cells(&d.world_bounds, d.cell_mm) / 1e6,
        );
    }

    // ── Oracle 1: the integrator against closed form ────────────────
    let worst = derived
        .iter()
        .max_by(|a, b| a.relative_error().total_cmp(&b.relative_error()));
    if let Some(w) = worst {
        println!(
            "\nworst grid-vs-closed-form error: {:.3}% on {} (tolerance {:.1}%)",
            w.relative_error() * 100.0,
            w.name,
            MASS_TOLERANCE * 100.0
        );
        if w.relative_error() > MASS_TOLERANCE {
            bail!(
                "part {} integrated to {:.4} kg but its closed form is {:.4} kg — \
                 {:.3}% apart, over the {:.1}% tolerance",
                w.name,
                w.grid_kg,
                w.closed_form_kg,
                w.relative_error() * 100.0,
                MASS_TOLERANCE * 100.0
            );
        }
    }

    // ── What the missing weld joint costs, measured ─────────────────
    //
    // `mass_properties` evaluates a uniform grid over the solid's bounding
    // box, so cost follows the box and the thinnest feature, not the amount
    // of material. Translation does not grow a box (`bounds.rs:185` shifts
    // min and max by the offset), so WHERE a part sits is free. What is not
    // free is how many parts it is: the spine and the cross-member are one
    // weldment, and integrating them as one part means gridding the empty
    // box that spans both.
    let weld_members: Vec<&Derived> = derived
        .iter()
        .filter(|d| d.name.starts_with("frame_"))
        .collect();
    if weld_members.len() != WELDED_FRAME_MEMBERS {
        bail!(
            "found {} welded frame members, expected {WELDED_FRAME_MEMBERS}",
            weld_members.len()
        );
    }
    if let Some((first, rest)) = weld_members.split_first() {
        let as_members: f64 = weld_members
            .iter()
            .map(|d| grid_cells(&d.world_bounds, d.cell_mm))
            .sum();
        let box_of_all = rest
            .iter()
            .fold(first.world_bounds, |acc, d| merged(&acc, &d.world_bounds));
        let as_one = grid_cells(&box_of_all, first.cell_mm);
        println!(
            "\nframe weldment at a {:.1} mm cell: {:.1}M cells as {} members, \
             {:.1}M as one part ({:.1}x)",
            first.cell_mm,
            as_members / 1e6,
            weld_members.len(),
            as_one / 1e6,
            as_one / as_members,
        );
    }

    // ── Oracle 2: the geometry the anchors actually describe ────────
    let axle = |name: &str| -> Result<Vector3<f64>> {
        origins
            .get(name)
            .copied()
            .ok_or_else(|| anyhow::anyhow!("no origin for {name}"))
    };
    let front_l = axle("rim_fl")?;
    let front_r = axle("rim_fr")?;
    let rear = axle("rim_r")?;
    let derived_track_mm = front_l.y - front_r.y;
    let derived_wheelbase_mm = rear.x - front_l.x;
    println!(
        "track from the anchors: {derived_track_mm:.4} mm (nominal {TRACK_MM})\n\
         wheelbase from the anchors: {derived_wheelbase_mm:.4} mm (nominal {WHEELBASE_MM})"
    );
    if (derived_track_mm - TRACK_MM).abs() > 1e-9 {
        bail!(
            "the joint anchors place the front wheels {derived_track_mm} mm apart, not {TRACK_MM}"
        );
    }
    if (derived_wheelbase_mm - WHEELBASE_MM).abs() > 1e-9 {
        bail!(
            "the joint anchors put the rear axle {derived_wheelbase_mm} mm aft, not {WHEELBASE_MM}"
        );
    }
    if (front_l.z - FRONT_RADIUS_MM).abs() > 1e-9 || (rear.z - REAR_RADIUS_MM).abs() > 1e-9 {
        bail!(
            "an axle is not at its rolling radius: front {:.4}, rear {:.4}",
            front_l.z,
            rear.z
        );
    }

    // ── The budget, and what it says against the typed one ──────────
    let masses: Vec<MassItem> = derived
        .iter()
        .map(|d| {
            MassItem::new(
                d.name.clone(),
                d.grid_kg,
                d.world_com_mm.x / 1000.0,
                d.world_com_mm.z / 1000.0,
            )
        })
        .collect();
    let spec = TrikeSpec {
        masses,
        ..TrikeSpec::iter1()
    };
    spec.assert_well_formed();

    let typed = TrikeSpec::iter1();
    println!("\n{:<28} {:>12} {:>12}", "", "derived", "typed");
    let row = |label: &str, a: f64, b: f64| {
        println!("{label:<28} {a:>12.4} {b:>12.4}");
    };
    row(
        "total mass (kg)",
        spec.total_mass_kg(),
        typed.total_mass_kg(),
    );
    row("cg x (m)", spec.cg_x_m(), typed.cg_x_m());
    row("cg z (m)", spec.cg_z_m(), typed.cg_z_m());
    row(
        "paired axle share",
        spec.paired_axle_share(),
        typed.paired_axle_share(),
    );
    row(
        "rollover threshold (g)",
        rollover_threshold_g(&spec),
        rollover_threshold_g(&typed),
    );

    // ── Oracle 3: the composed budget, pinned ───────────────────────
    //
    // These are what this geometry weighs and where it balances. They are a
    // regression gate, not a design target: change a tube, change a rider,
    // and they are supposed to fire so the new numbers get read.
    for (label, got, want) in [
        ("total mass (kg)", spec.total_mass_kg(), 102.819_250_020),
        ("cg x (m)", spec.cg_x_m(), 0.602_390_420),
        ("cg z (m)", spec.cg_z_m(), 0.337_627_432),
        (
            "rollover threshold (g)",
            rollover_threshold_g(&spec),
            0.690_522_826,
        ),
    ] {
        if (got - want).abs() > want.abs() * PIN_TOLERANCE {
            bail!("{label} came out {got:.9}, pinned at {want:.9}");
        }
    }

    // ── How much of this is a choice? ───────────────────────────────
    //
    // ⚠ The heaviest item dominates the centre of gravity, and its height here
    // is a seat height picked while modelling, not a measurement. A threshold
    // quoted to four digits off a chosen number reads far more certain than it
    // is, so the choice is priced rather than caveated.
    if let Some(heaviest) = derived
        .iter()
        .max_by(|a, b| a.grid_kg.total_cmp(&b.grid_kg))
    {
        // ⚠ cf-vehicle asserts rather than returns: `rollover_threshold_g`,
        // `effective_cg_height_m` and `paired_axle_share` all call
        // `assert_well_formed`, which PANICS on a mass below the ground plane.
        // Probing further down than the item's own height would abort with a
        // panic trace instead of an explanation — measured at a 400 mm probe,
        // exit 101. Refuse first, in this crate's own idiom.
        if heaviest.world_com_mm.z < CG_PROBE_MM {
            bail!(
                "cannot probe {CG_PROBE_MM} mm below {}: it sits at {:.1} mm and \
                 cf-vehicle panics on a mass under the ground plane",
                heaviest.name,
                heaviest.world_com_mm.z
            );
        }
        let probe = |dz_mm: f64| {
            let shifted: Vec<MassItem> = derived
                .iter()
                .map(|d| {
                    let z_mm = if d.name == heaviest.name {
                        d.world_com_mm.z + dz_mm
                    } else {
                        d.world_com_mm.z
                    };
                    MassItem::new(
                        d.name.clone(),
                        d.grid_kg,
                        d.world_com_mm.x / 1000.0,
                        z_mm / 1000.0,
                    )
                })
                .collect();
            let s = TrikeSpec {
                masses: shifted,
                ..TrikeSpec::iter1()
            };
            let track_needed =
                2.0 * s.effective_cg_height_m() * TYRE_MU / s.paired_axle_share() - s.track_m;
            (rollover_threshold_g(&s), track_needed * 1000.0)
        };
        println!(
            "\nheaviest item is {} at {:.1} kg, {:.0}% of the total, so it sets the cg.",
            heaviest.name,
            heaviest.grid_kg,
            100.0 * heaviest.grid_kg / spec.total_mass_kg()
        );
        for dz in [-CG_PROBE_MM, 0.0, CG_PROBE_MM] {
            let (threshold, track) = probe(dz);
            println!(
                "  its height {dz:+5.0} mm -> threshold {threshold:.4} g, needs {track:+7.1} mm of track"
            );
        }
    }

    let statics = StaticLoads::of(&spec);
    println!(
        "\nstatic: {:.1} N on each front wheel, {:.1} N on the rear",
        statics.per_paired_wheel_n, statics.single_wheel_n
    );
    let corner = CorneringLoads::at(&spec, 0.5);
    println!(
        "0.5 g corner: outer front {:.1} N, inner front {:.1} N",
        corner.outer_wheel_n, corner.inner_wheel_n,
    );

    // The design rule cf-vehicle documents — slide before it tips — is a
    // property of the budget, so it is a finding here, not a gate. Reporting
    // it red would make CI fail on a design question this example exists to
    // ask. ⚠ It reads differently for the two budgets, and that difference is
    // the reason this crate was written.
    for (which, s) in [("derived", &spec), ("typed", &typed)] {
        let c = CorneringLoads::at(s, 0.5);
        // Track that would put the threshold exactly at mu, from
        // threshold = share * track / (2 * h_eff).
        let track_to_clear = 2.0 * s.effective_cg_height_m() * TYRE_MU / s.paired_axle_share();
        println!(
            "{:<8} threshold {:.4} g at mu {TYRE_MU:.1} — {}; needs {:+.1} mm of track{}",
            which,
            rollover_threshold_g(s),
            if c.slides_before_it_tips(TYRE_MU) {
                "slides before it tips"
            } else {
                "TIPS BEFORE IT SLIDES"
            },
            (track_to_clear - s.track_m) * 1000.0,
            // ⚠ Conditional, so it disappears on its own once a suspension is
            // laid out: a rigid spec cannot roll, so its centre of gravity
            // never moves outboard in a corner and every threshold it reports
            // is a ceiling. cf-vehicle measures the gap — a 20 N/mm wheel rate
            // turns the typed +14.9 mm into +30.7 mm.
            if s.roll.is_none() {
                "  [RIGID — upper bound]"
            } else {
                ""
            },
        );
    }

    if let Some(dir) = out_dir {
        println!("\nmeshing the assembly:");
        export_stls(&mechanism, &origins, &dir, tolerance_mm)?;
    }

    println!("\nOK");
    Ok(())
}
