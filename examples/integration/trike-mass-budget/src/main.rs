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
//! - ⛔ **There is no `Fixed`/weld joint.** [`JointKind`] is Revolute,
//!   Prismatic, Ball or Free, and `with_range` *panics* when `min >= max`, so
//!   a zero-range revolute is rejected too. A trike frame is a weldment and
//!   every welded member here uses [`WELD_RANGE_RAD`], a range narrow enough
//!   to be a weld and wide enough to be accepted.
//! - ⛔ **A `Part` carries exactly one `Material`.** A wheel is a rim and a
//!   tyre; a rigid body of two materials cannot have its mass derived, so each
//!   material is its own part welded to the next, inflating the part count.
//! - ⚠ **`mass_properties` grids the solid's AABB at a uniform cell**, so cost
//!   follows the bounding box and the thinnest feature, not the amount of
//!   material. *Where* a part sits is free — `bounds.rs:185` shifts a
//!   translated box without growing it — but *how many parts* it is is not,
//!   and that is decided by the missing weld joint above. Every run prints the
//!   comparison: the spine and cross-member cost an **order of magnitude**
//!   more as the one weldment they physically are than as two members,
//!   because that single box spans both and is nearly all air.
//! - ⚠ **Nothing aggregates an assembly.** `subtree_com[0]` is the whole-model
//!   centre of mass, but it exists only after `to_model` plus a forward
//!   kinematics pass. [`world_origins`] below is that walk, done directly on
//!   the joint anchors, and is the thing to extract if this shape proves out.

#![allow(clippy::too_many_lines)]

use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, PI};

use anyhow::{Result, bail};
use cf_design::mechanism::mass::mass_properties;
use cf_design::{Aabb, JointDef, JointKind, Material, Mechanism, Part, Solid};
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

/// A weld, expressed in the only vocabulary [`JointKind`] offers: a revolute
/// whose range is too narrow to be motion. `with_range` rejects `0.0, 0.0`.
const WELD_RANGE_RAD: f64 = 1e-9;

/// How far a grid-integrated mass may sit from its closed form.
///
/// Set just above the worst part measured (0.318%, `rim_fr`), so a coarsened
/// cell trips it rather than passing quietly. ⚠ A changed *dimension* does not
/// trip it — see the note on oracle 1.
const MASS_TOLERANCE: f64 = 0.005;

/// Parts the plan is expected to produce.
///
/// ⚠ Asserted because every oracle below walks a collection, and a walk over
/// an empty one passes without doing anything. An empty `Mechanism` builds
/// happily — `validate` skips the orphan check below two parts — so nothing
/// upstream would object.
const EXPECTED_PARTS: usize = 13;

/// Number of members welded into the frame, whose grid cost is compared.
const WELDED_FRAME_MEMBERS: usize = 2;

/// How far to move the heaviest item's centre of mass when probing how much
/// of the answer is a choice rather than a measurement.
const CG_PROBE_MM: f64 = 50.0;

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

    // Spine: butts onto the cross-member's outside and runs to the tail.
    let spine_length = WHEELBASE_MM - FRAME_R_MM - TAIL_SETBACK_MM;
    let spine_x = FRAME_R_MM + spine_length / 2.0;

    // Uprights sit inboard of the wheels by half a wheel's width plus the
    // upright's own radius, so the contact patches land on the nominal track.
    let upright_y = TRACK_MM / 2.0 - (FRONT_WHEEL_HALF_WIDTH_MM + UPRIGHT_OD_MM / 2.0);
    let upright_length = 74.125;
    let upright_z = FRAME_Z_MM + FRAME_R_MM + upright_length / 2.0;

    let swingarm_length = 350.0;
    let swingarm_x = WHEELBASE_MM - swingarm_length / 2.0;

    let seat_pan_half = Vector3::new(200.0, 175.0, 1.5);
    let seat_pan_z = FRAME_Z_MM + FRAME_R_MM + seat_pan_half.z;
    let seat_pan_x = 560.0;

    let rider_radius = 170.0;
    let rider_half_length = 310.0;

    Ok(vec![
        PartPlan {
            name: "frame_spine",
            parent: "world",
            anchor_mm: Vector3::new(spine_x, 0.0, FRAME_Z_MM),
            kind: JointKind::Free,
            axis: Vector3::z(),
            range_rad: None,
            material: steel.clone(),
            piece: onto_x(tube(FRAME_OD_MM, FRAME_WALL_MM, spine_length)),
            cell_mm: 0.5,
        },
        PartPlan {
            name: "frame_cross",
            parent: "frame_spine",
            anchor_mm: Vector3::new(-spine_x, 0.0, 0.0),
            kind: JointKind::Revolute,
            axis: Vector3::y(),
            range_rad: Some((-WELD_RANGE_RAD, WELD_RANGE_RAD)),
            material: steel.clone(),
            piece: onto_y(tube(FRAME_OD_MM, FRAME_WALL_MM, upright_y * 2.0)),
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
            kind: JointKind::Revolute,
            axis: Vector3::y(),
            range_rad: Some((-WELD_RANGE_RAD, WELD_RANGE_RAD)),
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
            kind: JointKind::Revolute,
            axis: Vector3::y(),
            range_rad: Some((-WELD_RANGE_RAD, WELD_RANGE_RAD)),
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
            material: steel,
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
            kind: JointKind::Revolute,
            axis: Vector3::y(),
            range_rad: Some((-WELD_RANGE_RAD, WELD_RANGE_RAD)),
            material: Material::new("95A polyurethane", PU_95A_KG_M3),
            piece: onto_y(annulus(
                REAR_RADIUS_MM,
                REAR_RIM_OUTER_MM,
                REAR_WHEEL_HALF_WIDTH_MM,
            )),
            cell_mm: 1.0,
        },
        PartPlan {
            name: "seat_pan",
            parent: "frame_spine",
            anchor_mm: Vector3::new(seat_pan_x - spine_x, 0.0, seat_pan_z - FRAME_Z_MM),
            kind: JointKind::Revolute,
            axis: Vector3::y(),
            range_rad: Some((-WELD_RANGE_RAD, WELD_RANGE_RAD)),
            material: aluminium,
            piece: slab(seat_pan_half),
            cell_mm: 0.5,
        },
        PartPlan {
            name: "rider",
            parent: "seat_pan",
            anchor_mm: Vector3::new(0.0, 0.0, rider_radius),
            kind: JointKind::Revolute,
            axis: Vector3::y(),
            range_rad: Some((-WELD_RANGE_RAD, WELD_RANGE_RAD)),
            material: Material::new("rider", RIDER_KG_M3),
            piece: onto_x(capsule(rider_radius, rider_half_length)),
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

// ── Entry point ─────────────────────────────────────────────────────────

fn main() -> Result<()> {
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

    println!("reverse trike — {} parts\n", mechanism.parts().len());
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
        .filter(|d| d.name == "frame_spine" || d.name == "frame_cross")
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
        ("total mass (kg)", spec.total_mass_kg(), 89.950_362_142),
        ("cg x (m)", spec.cg_x_m(), 0.536_293_394),
        ("cg z (m)", spec.cg_z_m(), 0.315_023_043),
        (
            "rollover threshold (g)",
            rollover_threshold_g(&spec),
            0.815_605_029,
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

    println!("\nOK");
    Ok(())
}
