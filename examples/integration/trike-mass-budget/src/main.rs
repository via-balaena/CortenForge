//! The trike's mass budget, **derived from geometry** instead of typed.
//!
//! `cf-vehicle` takes a mass budget and derives every load from it, but the
//! budget itself was five hand-typed estimates. This example is the other
//! half: it takes the vehicle [`cf_trike`] defines and composes a budget out
//! of [`mass_properties`], so the numbers come from shapes.
//!
//! Run with: `cargo run --release -p example-trike-mass-budget`, and add
//! `--out <dir>` to write one STL per part plus an assembled one.
//!
//! # The layering this preserves
//!
//! `cf-vehicle` declares **no dependencies at all** — it stays geometry-free
//! and takes a budget — and this arc does not give it one. `cf-trike` knows
//! nothing about loads. This crate is the composition point.
//!
//! # What the oracles check
//!
//! 1. Every part's grid-integrated mass against its **closed form**, which is
//!    arithmetic on the same dimensions and never touches the integrator.
//!    ⚠ It does **not** catch a wrong dimension: the formula and the solid
//!    share the radius, so a wrong one moves both and the check stays
//!    satisfied.
//! 2. Wheelbase, track and rolling radii **read back out of the joint
//!    anchors** — placement is a chain of anchors and one mis-typed offset
//!    moves a contact patch.
//! 3. That **every weld joins metal that touches**. A part welded in the joint
//!    graph can float in space with every other check green; ten did.
//! 4. That the rear wheel is not **inside** the frame, and that the steering
//!    **turns** without the tyre entering a frame member.
//! 5. That the assembly **simulates**: it builds, steps without diverging, and
//!    holds its welds to tens of microns.
//! 6. The composed mass, centre of gravity and rollover threshold, pinned.

#![allow(clippy::too_many_lines)]

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use cf_design::mechanism::mass::mass_properties;
use cf_design::{Aabb, IndexedMesh, Mechanism, Part};
use cf_trike::{
    FRONT_RADIUS_MM, PartMetrics, REAR_RADIUS_MM, ROOT_PART, TRACK_MM, Trike, WHEELBASE_MM,
};
use cf_vehicle::analysis::rollover_threshold_g;
use cf_vehicle::{CorneringLoads, MassItem, StaticLoads, TrikeSpec};
use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_core::Model;

/// How far a grid-integrated mass may sit from its closed form.
///
/// Set just above the worst part measured — currently 0.350% on `seat_back`,
/// a 6 mm panel reclined 45 degrees — so a coarsened cell trips it rather
/// than passing quietly. ⚠ A changed *dimension* does not
/// trip it — see the note on oracle 1.
const MASS_TOLERANCE: f64 = 0.005;
/// How far the composed budget may drift before the pins below fire.
///
/// Loose enough to survive a last-ulp libm difference between platforms,
/// tight enough that any real change of geometry is caught: 1e-6 of 100 kg is
/// 0.1 g, and 1e-6 of 0.3 m is 0.3 µm.
const PIN_TOLERANCE: f64 = 1e-6;
/// Parts the plan is expected to produce.
///
/// ⚠ Asserted because every oracle below walks a collection, and a walk over
/// an empty one passes without doing anything. An empty `Mechanism` builds
/// happily — `validate` skips the orphan check below two parts — so nothing
/// upstream would object.
const EXPECTED_PARTS: usize = 34;
/// Welds in the assembly: three frame members and seven seat members onto the
/// spine, all three tyres onto their rims, the rider's two halves, the two
/// suspension towers, and the aft leg of each UPPER wishbone onto its fore
/// leg. The lower wishbones are one authored piece each and need no weld.
const EXPECTED_WELDS: usize = 22;

/// Loops the joint tree cannot hold: the tie rod's far end, and the upper
/// ball joint on each wishbone.
///
/// ★ A double wishbone **is** a loop. The upright is held by two arms and a
/// tree gives it one parent, so the second arm closes through a constraint.
const EXPECTED_LINKAGES: usize = 3;
/// Degrees of freedom in the **tree**: the free body, three wheels spinning,
/// the swingarm, the tie rod's near rod end (a ball, worth three), four
/// wishbones swinging, and each upright on its lower ball joint (three each).
///
/// ⚠ Not what the machine has. The three linkages take nine back, so the
/// trike really has fourteen — the twelve it had, plus a bump degree of
/// freedom at each front wheel.
const EXPECTED_DOF: usize = 23;
/// Members welded into the frame, whose grid cost is compared: spine,
/// cross-member and the two diagonals.
const WELDED_FRAME_MEMBERS: usize = 4;
/// How far to move the heaviest item's centre of mass when probing how much
/// of the answer is a choice rather than a measurement.
const CG_PROBE_MM: f64 = 50.0;
/// Polyurethane on asphalt, at the optimistic end of 0.6-1.0.
const TYRE_MU: f64 = 1.0;
/// Default meshing tolerance for `--out`, in millimetres.
///
/// ⚠ This is for *looking at* the vehicle, not for printing it. The tolerance
/// is a cell size and the vehicle is 1.25 m long, so at 1.0 mm the assembly
/// comes to just under a gigabyte of STL against 52 MB at this default.
/// Override with `--tolerance` when a wall section matters.
const STL_TOLERANCE_MM: f64 = 4.0;
/// Floor for the refinement in [`export_stls`]. A part still empty here has a
/// feature finer than a third of a millimetre and wants saying so, not
/// halving again.
const MIN_STL_TOLERANCE_MM: f64 = 0.25;

/// How far a linkage may let the two points it holds drift apart.
const MAX_LINKAGE_GAP_MM: f64 = 1.0;

/// How far a part's geometry may sit from its own solid in the physics model.
/// Measured at 0.00 mm once every part declares its joint origin.
const MAX_GEOM_DISPLACEMENT_MM: f64 = 0.5;

/// Voxel resolution for the simulation check. Coarse on purpose: this asks
/// whether the assembly is simulable, not what it collides with, and 8 mm cost
/// four times as long for the same answer.
const SIM_RESOLUTION_MM: f64 = 20.0;
/// Steps to take. Long enough for an unstable model to diverge.
const SIM_STEPS: usize = 50;
/// How far a welded body may move **in the root's frame**. Measured at 0.0 um.
///
/// ⚠ **This is a STABILITY check, not a rigidity one, and the difference is
/// worth stating.** A welded body has no joint between it and the root, so its
/// offset in the root's frame is constant by construction — every mutation that
/// breaks that (emitting a joint for a weld, re-declaring a weld as a hinge)
/// changes the dof or weld COUNT, and those gates fire first. What is left for
/// this one to catch is a rigid chain that wanders anyway: an unstable solve.
///
/// ⚠ Held at 10 um rather than the 1 mm it carried while the measurement still
/// included the root's rotation. A tolerance three orders above the quantity it
/// bounds is not a gate.
const MAX_WELD_DRIFT_MM: f64 = 0.01;
/// Full steering lock, in degrees — the range `upright_*` is given in radians.
const STEER_LOCK_DEG: f64 = 34.0;
/// Where the architecture wants the centre of gravity, longitudinally.
///
/// ★ 55 % of the wheelbase back from the front axle — the Porsche-balanced
/// 45/55 that the rollover table says is affordable at a 240 mm cg and a
/// 1750 mm track, and not before.
const TARGET_CG_X_M: f64 = 2.650 * 0.55;
/// And how low it has to sit for that balance to clear a mu of 1.5.
const TARGET_CG_Z_M: f64 = 0.240;
/// Mesh tolerance for the steering-clash probe.
const STEER_PROBE_MM: f64 = 6.0;
/// Mesh tolerance for the interpenetration scan.
const PAIR_PROBE_MM: f64 = 6.0;
/// What share of a part may be inside another and still count as node contact.
///
/// ★ **Extent is the discriminator, not adjacency** — measured on this
/// assembly, parts meeting at a shared node ran 2-36 points while true
/// interpenetration ran 257-6735. Two orders of magnitude apart.
///
/// ⚠ Graph distance looked like it should work and does NOT: six of the eight
/// pairs more than two hops apart were legitimate — a brace ending on a tower,
/// panels resting on their rails, a bar and an arm sharing a ball joint. Far
/// apart in the TREE, adjacent in SPACE.
///
/// ⚠ A **fraction**, not a point count. An absolute count is tuned to one
/// assembly's size: 120 points read node contact correctly on a 108 kg trike
/// and misread it on the same vehicle at car scale.
const MAX_NODE_OVERLAP: f64 = 0.06;
/// Suspension travel swept for clashes, in degrees — the wishbones are given
/// `+/-0.35 rad`, and this is that range in the units the message prints.
const BUMP_TRAVEL_DEG: f64 = 20.0;
/// Mesh tolerance for the weld-contact probe.
const WELD_PROBE_MM: f64 = 2.0;

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
    /// The closed form to check against, or `None` for authored geometry —
    /// see [`Derived::relative_error`].
    closed_form_kg: Option<f64>,
    /// The same part integrated on a grid one step finer. Only carried when
    /// there is no closed form, because that refinement is what replaces it.
    refined_kg: Option<f64>,
    world_com_mm: Vector3<f64>,
    /// The part's box, placed in the world frame.
    world_bounds: Aabb,
    cell_mm: f64,
}

impl Derived {
    /// How far the grid integrator sits from the truth, however it is known.
    ///
    /// ★ Two ways, because authored geometry has only one of them. A part
    /// built from **disjoint** primitives has an elementary volume, and the
    /// grid is checked against it. A part with blended fillets, a pocket and a
    /// boss has no such volume — inventing one would be worse than admitting
    /// it — so the grid is checked against **itself, refined**: integrate at
    /// the cell, integrate at half the cell, and require them to agree.
    ///
    /// ⚠ A convergence check is weaker. It catches a grid too coarse for the
    /// feature, which is the failure that actually happens here, but it cannot
    /// catch a solid that is the wrong shape in a way that refines smoothly.
    fn relative_error(&self) -> f64 {
        match (self.closed_form_kg, self.refined_kg) {
            (Some(closed), _) => (self.grid_kg - closed).abs() / closed,
            (None, Some(fine)) => (self.grid_kg - fine).abs() / fine,
            (None, None) => f64::INFINITY,
        }
    }

    /// What the grid was compared against, for the table.
    fn reference_kg(&self) -> Option<f64> {
        self.closed_form_kg.or(self.refined_kg)
    }
}

/// Integrate every part and place its centre of mass in the world frame.
fn derive(
    mechanism: &Mechanism,
    metrics: &HashMap<String, PartMetrics>,
    origins: &HashMap<String, Vector3<f64>>,
) -> Result<Vec<Derived>> {
    let mut out = Vec::new();
    for part in mechanism.parts() {
        let Some(&PartMetrics {
            volume_mm3,
            cell_mm,
        }) = metrics.get(part.name())
        else {
            bail!("no metrics recorded for part {}", part.name());
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
            closed_form_kg: volume_mm3.map(|v| v * 1e-9 * density),
            refined_kg: match volume_mm3 {
                Some(_) => None,
                // No closed form: integrate again at half the cell. If the
                // coarse pass had missed a feature, halving it would move the
                // answer.
                None => match mass_properties(part.solid(), density, cell_mm / 2.0) {
                    Some(fine) => Some(fine.mass),
                    None => bail!(
                        "part {} vanished at half its cell, so nothing checks its mass",
                        part.name()
                    ),
                },
            },
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
/// pan, 3 mm thick at the time, vanished entirely at the 4 mm default that
/// suits a 1.25 m frame and wrote an 84-byte STL containing no triangles — a
/// valid, correctly named, empty file. So each part is meshed at the requested tolerance and only what
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
    // ⚠ Parts go in their own directory, and the merged file stays out of it.
    // `cf-view --assembly` spawns EVERY stl in a directory at its world
    // position, so a merged copy sitting beside the parts draws the whole
    // vehicle twice — once in pieces and once on top of itself.
    let parts_dir = dir.join("parts");
    std::fs::create_dir_all(&parts_dir)
        .with_context(|| format!("creating {}", parts_dir.display()))?;
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

        let path = parts_dir.join(format!("{}.stl", part.name()));
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
    println!(
        "  assembled, part by part, with a visibility toggle each:\n    \
         cargo run --release -p cf-viewer --bin cf-view -- --assembly {}",
        parts_dir.display()
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

/// World position of a body at the reference configuration: `body_pos` is
/// relative to the parent, so composing the chain is a sum.
fn body_world(model: &Model, mut body: usize) -> Vector3<f64> {
    let mut at = Vector3::zeros();
    for _ in 0..model.nbody {
        if body == 0 {
            break;
        }
        at += model.body_pos[body];
        body = model.body_parent[body];
    }
    at
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

    let Trike {
        mechanism,
        metrics,
        origins,
    } = cf_trike::trike()?;
    let derived = derive(&mechanism, &metrics, &origins)?;

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
    let held: usize = mechanism
        .linkages()
        .iter()
        .map(|l| l.kind().constrained_dof())
        .sum();
    println!(
        "reverse trike — {} parts, {welds} welds, {dof} dof in the tree, {} \
         linkage holding {held} of them: {} left",
        mechanism.parts().len(),
        mechanism.linkages().len(),
        dof - held
    );
    if mechanism.linkages().len() != EXPECTED_LINKAGES {
        bail!(
            "{} linkages, expected {EXPECTED_LINKAGES}",
            mechanism.linkages().len()
        );
    }
    if welds != EXPECTED_WELDS {
        bail!("{welds} welds, expected {EXPECTED_WELDS}");
    }
    if dof != EXPECTED_DOF {
        bail!("{dof} degrees of freedom, expected {EXPECTED_DOF}");
    }
    println!();
    println!(
        "{:<12} {:>10} {:>12} {:>9}   {:>8} {:>8} {:>8} {:>7} {:>9}",
        "part", "grid kg", "checked vs", "rel err", "com x", "com y", "com z", "cell", "cells"
    );
    for d in &derived {
        println!(
            "{:<12} {:>10.4} {:>12.4} {:>8.3}% {:>9.1} {:>8.1} {:>8.1} {:>7.1} {:>8.2}M",
            d.name,
            d.grid_kg,
            d.reference_kg().unwrap_or(f64::NAN),
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
            "\nworst grid error (closed form, or the grid refined): {:.3}% on {} (tolerance {:.1}%)",
            w.relative_error() * 100.0,
            w.name,
            MASS_TOLERANCE * 100.0
        );
        if w.relative_error() > MASS_TOLERANCE {
            bail!(
                "part {} integrated to {:.4} kg against a reference of {:.4} kg — \
                 {:.3}% apart, over the {:.1}% tolerance",
                w.name,
                w.grid_kg,
                w.reference_kg().unwrap_or(f64::NAN),
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

    // ── Oracle 1b: every weld joins metal that touches ──────────────
    //
    // Nothing else here can see this. The mass gate checks each part alone,
    // the anchor gates check three dimensions, and a part welded in the joint
    // graph can float in space with all of them green. It did: the seat and
    // the rider hung 31 mm clear of the frame, and the swingarm straddled the
    // spine without reaching it — ten disconnected welds in all.
    //
    // ⚠ Welds only. A revolute is a bearing, and a wheel on an axle does not
    // touch the arm that carries it, so requiring contact there would be
    // asking the model to draw an axle it has no reason to draw.
    {
        let by_name: HashMap<&str, &Part> =
            mechanism.parts().iter().map(|p| (p.name(), p)).collect();
        let mut worst: Option<(String, f64)> = None;
        for joint in mechanism.joints().iter().filter(|j| j.kind().is_weld()) {
            let (Some(child), Some(parent)) =
                (by_name.get(joint.child()), by_name.get(joint.parent()))
            else {
                continue;
            };
            let (Some(&co), Some(&po)) = (origins.get(joint.child()), origins.get(joint.parent()))
            else {
                bail!(
                    "no world origin for {} or {}",
                    joint.child(),
                    joint.parent()
                );
            };
            // ⚠ The probe tolerance is load-bearing. At 6 mm a 6 mm panel
            // meshes to nothing and reads as an infinite gap, and a round tube
            // sitting on a round tube reads several millimetres apart because
            // no vertex lands on the tangent point.
            let probe = child.solid().mesh(WELD_PROBE_MM).geometry;
            if probe.vertices.is_empty() {
                bail!(
                    "part {} meshed to nothing at the {WELD_PROBE_MM} mm probe \
                     tolerance, so its contact cannot be checked",
                    joint.child()
                );
            }
            let clearance = probe
                .vertices
                .iter()
                .map(|v| parent.solid().evaluate(&Point3::from(v.coords + co - po)))
                .fold(f64::INFINITY, f64::min);
            if worst.as_ref().is_none_or(|(_, w)| clearance > *w) {
                worst = Some((
                    format!("{} -> {}", joint.parent(), joint.child()),
                    clearance,
                ));
            }
        }
        match worst {
            None => bail!("no welds found to check — the assembly lost its welds"),
            Some((where_, clearance)) => {
                println!(
                    "loosest weld: {where_} at {clearance:+.2} mm (negative is metal in metal)"
                );
                if clearance > 0.0 {
                    bail!(
                        "weld {where_} joins parts {clearance:.2} mm apart — welded in \
                         the joint graph, floating in space"
                    );
                }
            }
        }
    }

    // ── Oracle 1c: the rear wheel does not live inside the frame ────
    //
    // Measured off the built parts rather than off the constants, so it still
    // means something if either moves. ⚠ Deliberately arithmetic on world
    // bounding boxes: `Solid::evaluate` on a CSG solid returns a bound, not a
    // distance — it reported this very clash as 0.8 mm deep when it is 40 —
    // so its sign can be trusted and its magnitude cannot.
    {
        let bound = |name: &str| -> Result<Aabb> {
            let d = derived
                .iter()
                .find(|d| d.name == name)
                .ok_or_else(|| anyhow::anyhow!("no part {name}"))?;
            Ok(d.world_bounds)
        };
        let spine_tail = bound("frame_spine")?.max.x;
        let tyre_front = bound("tyre_r")?.min.x;
        let clearance = tyre_front - spine_tail;
        println!(
            "rear wheel to spine tail: {clearance:+.1} mm (spine ends {spine_tail:.0}, \
             tyre starts {tyre_front:.0})"
        );
        if clearance <= 0.0 {
            bail!(
                "the rear tyre reaches x={tyre_front:.0} and the spine runs to \
                 x={spine_tail:.0} — the wheel is {:.0} mm inside the frame",
                -clearance
            );
        }
    }

    // ── Oracle 1d: the steering turns without hitting the frame ─────
    //
    // Every other check here is at the reference pose. A vehicle that is fine
    // at rest and jams at full lock is still broken, and nothing above would
    // notice: the masses are right, the welds touch, the contact patches are
    // where they belong.
    //
    // ⚠ Sign only. `Solid::evaluate` on a CSG solid is a bound — it read a
    // 40 mm interpenetration as 0.8 — so this asks whether any point of the
    // turned wheel is INSIDE a frame member, never how far from it.
    {
        let by_name: HashMap<&str, &Part> =
            mechanism.parts().iter().map(|p| (p.name(), p)).collect();
        let kingpin_axis = nalgebra::Unit::new_normalize(cf_trike::steering_axis());
        let pivot = *origins
            .get("upright_l")
            .ok_or_else(|| anyhow::anyhow!("no upright to steer about"))?;
        let tyre = by_name
            .get("tyre_fl")
            .ok_or_else(|| anyhow::anyhow!("no front tyre"))?;
        let tyre_origin = *origins
            .get("tyre_fl")
            .ok_or_else(|| anyhow::anyhow!("no front tyre origin"))?;
        let probe = tyre.solid().mesh(STEER_PROBE_MM).geometry;
        if probe.vertices.is_empty() {
            bail!("the front tyre meshed to nothing at {STEER_PROBE_MM} mm");
        }
        for lock_deg in [STEER_LOCK_DEG, -STEER_LOCK_DEG] {
            let rot = UnitQuaternion::from_axis_angle(&kingpin_axis, lock_deg.to_radians());
            for member in ["frame_cross", "frame_diag_l"] {
                let fixed = by_name
                    .get(member)
                    .ok_or_else(|| anyhow::anyhow!("no member {member}"))?;
                let fixed_origin = *origins
                    .get(member)
                    .ok_or_else(|| anyhow::anyhow!("no origin for {member}"))?;
                let inside = probe.vertices.iter().any(|v| {
                    let world = pivot + rot * (v.coords + tyre_origin - pivot);
                    fixed.solid().evaluate(&Point3::from(world - fixed_origin)) < 0.0
                });
                if inside {
                    bail!("at {lock_deg:+.0} deg of lock the front tyre enters {member}");
                }
            }
        }
        println!("steering sweeps +/-{STEER_LOCK_DEG:.0} deg clear of the frame");
    }

    // ── Oracle 1g: nothing occupies the same space as anything else ─
    //
    // ★ The check that catches a part which is the right SHAPE, the right MASS
    // and in the WRONG PLACE — which no mass or volume gate can see, because
    // each of them reads one part at a time in its own frame. It found the
    // frame still braced 292 mm past the suspension pickups, through the
    // volume the lower wishbone swings in, after three other gates passed.
    //
    // ⚠ Lives in `cf-assembly-checks` and knows nothing about vehicles.
    {
        let found = cf_assembly_checks::overlapping_pairs(&mechanism, &origins, PAIR_PROBE_MM);
        let bulk: Vec<_> = found
            .iter()
            .filter(|o| o.fraction > MAX_NODE_OVERLAP)
            .collect();
        println!(
            "{} part pairs touch without being joined; deepest {:.1}% of {}",
            found.len(),
            found.first().map_or(0.0, |o| o.fraction) * 100.0,
            found.first().map_or("none", |o| o.a.as_str()),
        );
        if !bulk.is_empty() {
            bail!(
                "these pairs interpenetrate rather than meeting at a node:\n  {}",
                bulk.iter()
                    .map(|o| {
                        format!(
                            "{} <-> {}: {:.1}% of one is inside the other ({} points)",
                            o.a,
                            o.b,
                            o.fraction * 100.0,
                            o.points
                        )
                    })
                    .collect::<Vec<_>>()
                    .join("\n  ")
            );
        }
    }

    // ── Oracle 1f: the suspension moves without hitting anything ────
    //
    // The steering sweep above moves the wheel. Nothing moved the suspension,
    // which is the one motion the wishbones exist to have — so ten parts and
    // a degree of freedom per front wheel went in with no clash check at all.
    //
    // ★ A wishbone's travel is an **exact** rotation about its own pivot axis,
    // which is the x axis through its body origin. That is what makes this a
    // measurement rather than an approximation: no constraint solving is
    // needed to know where the arm goes.
    //
    // ⚠ Sign only, for the reason the steering sweep gives: `Solid::evaluate`
    // on a CSG solid is a bound, not a distance.
    //
    // ⚠ A part is never swept against what it is JOINED to. The arms pivot on
    // those, so they touch by construction, and a scan that included them
    // would report the joint as a collision and be switched off.
    {
        let by_name: HashMap<&str, &Part> =
            mechanism.parts().iter().map(|p| (p.name(), p)).collect();
        let axis = nalgebra::Unit::new_normalize(Vector3::x());
        let mut swept = 0_usize;

        // Joined pairs, read off the assembly rather than listed here. A
        // hardcoded list of exceptions is a list that rots: this one would
        // have had to grow every time a part was added, and the first thing
        // the scan reported was a wishbone "entering" the upright hanging
        // off its own ball joint.
        //
        // ⚠ Inert as the list below stands — removing this changes nothing,
        // because the parts it would excuse are already out of that list for
        // travelling with the arm. It is here so that adding a part to the
        // list cannot resurrect the false positive that produced it.
        let mut joined: HashSet<(&str, &str)> = HashSet::new();
        for j in mechanism.joints() {
            joined.insert((j.parent(), j.child()));
            joined.insert((j.child(), j.parent()));
        }
        for l in mechanism.linkages() {
            joined.insert((l.a(), l.b()));
            joined.insert((l.b(), l.a()));
        }

        for arm in [
            "arm_lower_l",
            "arm_lower_r",
            "arm_upper_l",
            "arm_upper_l_aft",
            "arm_upper_r",
            "arm_upper_r_aft",
        ] {
            let part = by_name
                .get(arm)
                .ok_or_else(|| anyhow::anyhow!("no part {arm}"))?;
            let origin = *origins
                .get(arm)
                .ok_or_else(|| anyhow::anyhow!("no origin for {arm}"))?;
            // The pivot is the arm's own body origin — the midpoint of its two
            // frame pickups, which is a point on its axis.
            let pivot = origin;
            let probe = part.solid().mesh(STEER_PROBE_MM).geometry;
            if probe.vertices.is_empty() {
                bail!("{arm} meshed to nothing at {STEER_PROBE_MM} mm");
            }

            for deg in [BUMP_TRAVEL_DEG, -BUMP_TRAVEL_DEG] {
                let rot = UnitQuaternion::from_axis_angle(&axis, deg.to_radians());
                // ⚠ Only parts that do NOT travel with this arm. The
                // upright, its wheel, the steer arm and the bar all ride on
                // the wishbone, so sweeping the arm against their resting
                // pose would compare a part with where its own passengers
                // used to be. What is left is genuinely independent: the
                // frame, the towers, the seat, and the other wishbones.
                for other in [
                    "frame_spine",
                    "frame_cross",
                    "frame_diag_l",
                    "frame_diag_r",
                    "tower_l",
                    "tower_r",
                    "seat_cross",
                    "arm_lower_l",
                    "arm_lower_r",
                    "arm_upper_l",
                    "arm_upper_l_aft",
                    "arm_upper_r",
                    "arm_upper_r_aft",
                ] {
                    if other == arm || joined.contains(&(arm, other)) {
                        continue;
                    }
                    let fixed = by_name
                        .get(other)
                        .ok_or_else(|| anyhow::anyhow!("no part {other}"))?;
                    let fixed_origin = *origins
                        .get(other)
                        .ok_or_else(|| anyhow::anyhow!("no origin for {other}"))?;
                    let inside = probe.vertices.iter().any(|v| {
                        let world = pivot + rot * (v.coords + origin - pivot);
                        fixed.solid().evaluate(&Point3::from(world - fixed_origin)) < 0.0
                    });
                    if inside {
                        bail!(
                            "at {deg:+.0} deg of travel {arm} enters {other} — the \
                             suspension cannot move through its declared range"
                        );
                    }
                    swept += 1;
                }
            }
        }
        if swept == 0 {
            bail!("the bump sweep compared nothing, so it proves nothing");
        }
        println!(
            "suspension sweeps +/-{BUMP_TRAVEL_DEG:.0} deg clear \
             ({swept} arm-against-part checks; the wishbones foul each other \
             by 30)"
        );
    }

    // ── Oracle 1e: the assembly simulates, and the welds hold ───────
    //
    // Everything above reads geometry. This builds the physics model and steps
    // it, which is the only check here that the vehicle is a vehicle and not
    // just a set of shapes: `to_model` can fail on an unreachable part or one
    // with no finite bounds, and a model that is built can still go unstable.
    //
    // It also cross-checks the degree-of-freedom count against a second,
    // independent path: this file sums `JointKind::dof()`, and the physics
    // layer counts `nv` for itself.
    //
    // ⚠ What it does NOT prove. The model is stepped in free fall — no ground,
    // no contacts — so this says nothing about whether the vehicle stands up,
    // rolls, or corners. It says the assembly builds (56 geoms over 28 parts),
    // integrates without diverging, and holds its welds. Standing it on a
    // ground plane needs one, and a bare `Plane` has no finite bounds, so a
    // `Mechanism` cannot carry it.
    {
        let model = mechanism
            .to_model(SIM_RESOLUTION_MM, SIM_RESOLUTION_MM)
            .map_err(|e| anyhow::anyhow!("to_model failed: {e:?}"))?;
        if model.nv != EXPECTED_DOF {
            bail!(
                "the physics model has {} degrees of freedom, this file counts \
                 {EXPECTED_DOF}",
                model.nv
            );
        }

        // A part reached only through welds cannot move relative to the root.
        // Anything past a hinge may, and does: in free fall the wheels and
        // uprights turn under their own weight, which is why measuring every
        // body indiscriminately showed 10.8 mm of "drift" that was not drift.
        let mut parent_of: HashMap<String, (String, bool)> = HashMap::new();
        for j in mechanism.joints() {
            parent_of.insert(
                j.child().to_owned(),
                (j.parent().to_owned(), j.kind().is_weld()),
            );
        }
        let welded_to_root = |start: &str| {
            let mut n = start.to_owned();
            for _ in 0..64 {
                if n == ROOT_PART {
                    return true;
                }
                match parent_of.get(&n) {
                    None => return true,
                    Some((p, true)) => n = p.clone(),
                    Some((_, false)) => return false,
                }
            }
            false
        };
        let body = |name: &str| {
            model
                .body_name
                .iter()
                .position(|b| b.as_deref() == Some(name))
        };
        let root = body(ROOT_PART).ok_or_else(|| anyhow::anyhow!("no {ROOT_PART} body"))?;
        let rigid: Vec<(String, usize)> = mechanism
            .parts()
            .iter()
            .map(Part::name)
            .filter(|n| welded_to_root(n))
            .filter_map(|n| body(n).map(|i| (n.to_owned(), i)))
            .collect();
        if rigid.len() < 2 {
            bail!(
                "only {} bodies are welded to the root — nothing to check",
                rigid.len()
            );
        }

        // ⚠ The model's geometry must sit where the solid says. `to_model`
        // bbox-aligns an articulated part to its joint anchor unless the part
        // declares a joint origin — right for a finger segment modelled at the
        // origin, wrong for a vehicle whose solids are already placed. It was
        // displacing the front wheels 180 mm and the swingarm 188, and nothing
        // here would have noticed: the masses come from the solids, and free
        // fall has no contacts to be in the wrong place for.
        for d in &derived {
            let Some(b) = body(&d.name) else {
                bail!("no body for part {}", d.name);
            };
            let in_model = body_world(&model, b) + model.body_ipos[b];
            let displaced = (in_model - d.world_com_mm).norm();
            if displaced > MAX_GEOM_DISPLACEMENT_MM {
                bail!(
                    "part {} sits {displaced:.1} mm from its solid in the physics \
                     model — its geometry has been aligned somewhere else",
                    d.name
                );
            }
        }

        // ⚠ The mechanism's linkages must reach the model. Counting them on
        // the mechanism proves only that they were declared; if `to_model`
        // dropped them, `nv` would be unchanged — a linkage costs no degree of
        // freedom — and nothing else here would object.
        if model.neq != EXPECTED_LINKAGES {
            bail!(
                "the physics model holds {} equality constraints, and the \
                 mechanism declares {EXPECTED_LINKAGES}",
                model.neq
            );
        }

        let mut data = model.make_data();
        data.forward(&model)
            .map_err(|e| anyhow::anyhow!("forward kinematics failed: {e:?}"))?;
        // ⚠ In the ROOT'S FRAME, not merely relative to its position.
        //
        // A welded body has no joint between it and the root, so its offset in
        // the root's frame is constant by construction and any motion measured
        // here is a defect. Subtracting only `xpos[root]` leaves the root's
        // ROTATION in: the vehicle turns on its free joint, a body 380 mm off
        // centreline sweeps an arc, and that reads as drift. At a 900 mm track
        // it hid inside the tolerance; at 1750 mm it reported 2.7 mm of drift
        // for a part that cannot move.
        let in_root = |data: &sim_core::Data, b: usize| {
            data.xmat[root].transpose() * (data.xpos[b] - data.xpos[root])
        };
        let start: Vec<Vector3<f64>> = rigid.iter().map(|(_, b)| in_root(&data, *b)).collect();
        for step in 0..SIM_STEPS {
            data.step(&model)
                .map_err(|e| anyhow::anyhow!("step {step} failed: {e:?}"))?;
        }
        if !data.xpos.iter().all(|p| p.iter().all(|v| v.is_finite())) {
            bail!("a body position went non-finite within {SIM_STEPS} steps");
        }
        let mut worst = (String::new(), 0.0_f64);
        for (i, (name, b)) in rigid.iter().enumerate() {
            let drift = (in_root(&data, *b) - start[i]).norm();
            if drift > worst.1 {
                worst = (name.clone(), drift);
            }
        }
        // The linkage's own claim: the two points it holds stay together while
        // the machine moves. cf-design proves this on a four-bar; this proves
        // it on the vehicle, where the rod ties two steering arms that the
        // joint tree leaves free of each other.
        let mut held_apart: f64 = 0.0;
        for eq in 0..model.neq {
            let (a, b) = (model.eq_obj1id[eq], model.eq_obj2id[eq]);
            let d = model.eq_data[eq];
            let pa = data.xpos[a] + data.xmat[a] * Vector3::new(d[0], d[1], d[2]);
            let pb = data.xpos[b] + data.xmat[b] * Vector3::new(d[3], d[4], d[5]);
            held_apart = held_apart.max((pa - pb).norm());
        }
        if held_apart > MAX_LINKAGE_GAP_MM {
            bail!(
                "a linkage let its ends drift {held_apart:.3} mm apart over \
                 {SIM_STEPS} steps — it is supposed to hold them together"
            );
        }
        println!("  linkage ends held to {:.1} um", held_apart * 1000.0);

        println!(
            "simulated {SIM_STEPS} steps: {} welded bodies, worst drift {:.1} um ({})",
            rigid.len(),
            worst.1 * 1000.0,
            worst.0
        );
        if worst.1 > MAX_WELD_DRIFT_MM {
            bail!(
                "{} moved {:.3} mm relative to the frame over {SIM_STEPS} steps — \
                 it is welded to it",
                worst.0,
                worst.1
            );
        }
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
    // ⚠ Geometry comes from cf-trike, not from cf-vehicle's own sample.
    //
    // `TrikeSpec::iter1()` is cf-vehicle's illustrative spec and it still
    // describes the rideable trike this vehicle used to be. Inheriting its
    // wheelbase left a centre of gravity at 1.27 m sitting outside a 1.25 m
    // wheelbase, and cf-vehicle rightly panicked. **cf-trike owns the
    // dimensions; cf-vehicle does the analysis.**
    let geometry = TrikeSpec {
        wheelbase_m: cf_trike::WHEELBASE_MM / 1000.0,
        track_m: cf_trike::TRACK_MM / 1000.0,
        front_wheel_radius_m: cf_trike::FRONT_RADIUS_MM / 1000.0,
        rear_wheel_radius_m: cf_trike::REAR_RADIUS_MM / 1000.0,
        steering_axis_angle_deg: 90.0 - cf_trike::CASTER_DEG,
        masses: Vec::new(),
        ..TrikeSpec::iter1()
    };
    let spec = TrikeSpec { masses, ..geometry };
    spec.assert_well_formed();

    // ★ The second column is now the ARCHITECTURAL TARGET, not a stale guess:
    // 700 kg at 45/55 with the centre of gravity at 240 mm is what the design
    // is aiming for, so the gap between the columns is the work remaining.
    let typed = TrikeSpec {
        masses: vec![
            MassItem::new("target: sprung mass", 520.0, TARGET_CG_X_M, TARGET_CG_Z_M),
            MassItem::new("target: unsprung", 180.0, TARGET_CG_X_M, 0.31),
            MassItem::new("driver", 85.0, 1.05, 0.35),
        ],
        ..geometry
    };
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
    let mut drifted: Vec<String> = Vec::new();
    for (label, got, want) in [
        ("total mass (kg)", spec.total_mass_kg(), 250.529_923_465),
        ("cg x (m)", spec.cg_x_m(), 1.280_271_555),
        ("cg z (m)", spec.cg_z_m(), 0.355_432_882),
        (
            "rollover threshold (g)",
            rollover_threshold_g(&spec),
            1.272_445_090,
        ),
    ] {
        if (got - want).abs() > want.abs() * PIN_TOLERANCE {
            // ⚠ Collected, not bailed on. These four move together whenever
            // the geometry changes, and failing at the first one costs a
            // whole run per number to read the rest.
            drifted.push(format!("{label} came out {got:.9}, pinned at {want:.9}"));
        }
    }
    if !drifted.is_empty() {
        bail!("the pinned budget moved:\n  {}", drifted.join("\n  "));
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
                ..geometry.clone()
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
