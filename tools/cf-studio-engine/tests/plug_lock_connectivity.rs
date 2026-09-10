//! A cast either honours the cavity inset it was given or declines it. What it
//! must never do is emit the mold anyway with the plug's floor lock lying
//! loose beside it.
//!
//! ★★★ THE DEFECT, and it is CLOSED — read the tables below as history.
//! `cf_cast::add_plug_pins` took the ribbon and nothing else, and both
//! transforms it returned were anchored on the ribbon's cap-plane. The ribbon
//! is the cleaned scan's centerline: it does not move when `cavity_inset_m`
//! changes. The plug is `scan.offset(-inset)`, and it does. Past a threshold
//! the plug's base had climbed clear of the lock and the two meshed as separate
//! bodies — which shipped as a loose pyramid in the print, until the refusal
//! that landed with this file. `cf-studio-gui` offers 0-30 mm.
//!
//! It now takes the plug's mesh cell size and READS the plug, standing the lock
//! on a PEDESTAL that carries it back down to the cap plane
//! (`cf_cast::build_plug_lock_pedestal_transform`). The refusal stays as the
//! backstop for insets no column can reach, which is what the sweep still
//! gates; `an_inset_that_lifts_the_plug_clear_of_its_lock_is_cast_anyway`
//! gates the capability that replaced it.
//!
//! ★★ WHY A CONE. A straight tube CANNOT show this: its inward offset shrinks
//! it laterally and the cap-plane cut keeps the base pinned, so the body meets
//! the lock at every inset (measured 2026-09-07: one component at 0-14 mm, z
//! fixed at [-4, 30]). The body only recedes where the scan TAPERS toward the
//! cap plane, and a real limb scan does. For a cone of half-angle `a` the base
//! lifts off at `inset > r0 * cos(a)` and then climbs `1 / sin(a)` per mm of
//! inset — 2.37 here, against 1.78 measured on `~/scans/base_mold`, whose base
//! flares at ~34 degrees. Same law, different cone.
//!
//! Measured 2026-09-07 on this fixture, at 0.6 s per cast:
//!
//! | inset | components | body base | lock |
//! |---|---|---|---|
//! | 0 - 5.5 mm | 1 | z = -4.000 | fused |
//! | 6.0 mm | **2** | z = +4.462 | 8.341 x 8.341 x 8.000 at z = [-4, +4] |
//! | 8.0 mm | **2** | z = +9.225 | same size, same place |
//! | 11.0 mm | **2** | z = +16.370 | same size, same place |
//!
//! The lock is in the same place in every row. That is the whole finding.
//!
//! Those rows are the state BEFORE the fix that ships with this file: across
//! the range the GUI offers, 0 - 5.5 mm was correct, 6 - 19 mm was SILENTLY
//! WRONG, and from 20 mm the cast already declined — though as `marching cubes
//! produced an empty mesh for plug layer 0`, the mesher complaining downstream
//! rather than the cast telling the operator their inset is larger than this
//! scan can take. The middle band is what this arc closed; the wording of the
//! top one belongs with reconciling the offered range against the honourable
//! one, which is still open.
//!
//! ⚠ SCOPE. The cup pieces fragment too (2 -> 4 components at 11 mm), and both
//! halves carry a 16-face sliver even at 2 mm. Cup-piece connectivity was the
//! registration-pin arc and is not this one; recorded, not gated here.

// `panic!` with context is the integration-test idiom here (same convention as
// `design/cf-cast/tests/*.rs`): the crate denies it for library code, where
// errors are values, but a test failure has to be readable.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use cf_studio_engine::{PartId, PartSelection};
use cortenforge::mesh::repair::components::find_connected_components;

mod common;
use common::{CastOutcome, cast_synthetic, open_cone};

/// Insets to sweep, in mm. 5 is the shipped default and the one configuration
/// that has been physically poured; 0 is the degenerate floor. 20 is past the
/// point where this cone has any plug left to mesh, and is here so the refusal
/// branch below is exercised rather than merely allowed for.
const INSETS_MM: [f64; 6] = [0.0, 2.0, 5.0, 8.0, 11.0, 20.0];

/// Where the lock's base sits, in mm, in every row of the table above — it is
/// symmetric across the cap plane with a 4 mm half-length, so it protrudes 4 mm
/// BELOW the plug's cap-plane face and into the cup floor, which is where the
/// socket that receives it is cut.
const LOCK_BASE_Z_MM: f64 = -4.0;

/// Marching-cubes cell size for the sweep, in meters.
///
/// ⚠ Load-bearing, not a speed knob: the detachment threshold moves with it.
/// On THIS cone the plug detaches from 6.0 mm of inset at this cell size,
/// 7.0 mm at 1.5 mm cells and 8.0 mm at 1.0 mm — finer resolves more of the
/// thin tip that bridges the last of the gap, so it tolerates more inset.
///
/// ⛔ That direction is this fixture's, NOT a law. `~/scans/base_mold` REVERSES
/// it, and a third measured point killed the resolution story twice: what is
/// really moving is sub-cell GRID ALIGNMENT. Do not carry the mechanism to
/// another scan; carry only that the threshold is cell-size dependent.
///
/// The sweep survives that: measured 2026-09-07, both sweep tests pass at 3.0,
/// 1.5 AND 1.0 mm cells (2.9 / 5.6 / 7.3 s), since every inset in
/// [`INSETS_MM`] either casts whole or is declined at each of them. 3 mm is
/// here for the cost, not because the gate needs it.
///
/// ⚠ The pedestal gate below does NOT share it — see [`PEDESTAL_CELL_SIZE_M`],
/// which asks at a cell size the product actually offers.
const CELL_SIZE_M: f64 = 0.003;

/// What a cast at one inset produced here: a refusal, or the plug's pieces
/// largest first.
///
/// ⚠ A refusal is a LEGITIMATE answer and the sweep accepts it — an inset that
/// leaves the plug unreachable by its floor lock is one the cast may decline
/// rather than honour. What it must never do is emit the mold anyway with a
/// loose pyramid in it.
enum Outcome {
    Refused(String),
    Cast(Vec<Piece>),
}

/// One connected piece of the emitted plug: its face count and its z-range.
struct Piece {
    faces: usize,
    z_min: f64,
    z_max: f64,
    extent: [f64; 3],
}

/// Cast the cone at `inset_m` and return the emitted plug's pieces.
///
/// One layer and layer 0's plug only: the lock is a layer-0 cap-plane feature,
/// and the cup halves cost more than the whole rest of the gate.
///
/// ⚠ `caller` is what keeps the two tests apart — see [`cast_synthetic`], which
/// deletes the directory it builds in.
fn cast_at(caller: &str, inset_m: f64, cell_size_m: f64) -> Outcome {
    let outcome = cast_synthetic(
        &format!("plug-lock-{caller}"),
        open_cone(0.006, 0.020, 0.030, 24, 13),
        inset_m,
        cell_size_m,
        &PartSelection::from_ids([PartId::Plug { layer_index: 0 }]),
    );
    let emitted = match outcome {
        CastOutcome::Refused(msg) => return Outcome::Refused(msg),
        CastOutcome::Cast(meshes) => meshes,
    };
    // The selection above asks for exactly one part, so exactly one STL comes
    // back — asserted rather than assumed, because `cast_synthetic` returns
    // cup halves ahead of plugs and a widened selection would silently make
    // `[0]` a cup.
    assert_eq!(
        emitted.len(),
        1,
        "plug-only selection emits one STL, got {:?}",
        emitted.iter().map(|(n, _)| n).collect::<Vec<_>>()
    );
    let (_, mesh) = &emitted[0];
    let mut pieces: Vec<Piece> = find_connected_components(mesh)
        .components
        .iter()
        .map(|c| {
            let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
            for &f in c {
                for &v in &mesh.faces[f as usize] {
                    let p = mesh.vertices[v as usize];
                    for k in 0..3 {
                        lo[k] = lo[k].min(p[k]);
                        hi[k] = hi[k].max(p[k]);
                    }
                }
            }
            Piece {
                faces: c.len(),
                z_min: lo[2],
                z_max: hi[2],
                extent: [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]],
            }
        })
        .collect();
    pieces.sort_by_key(|p| std::cmp::Reverse(p.faces));
    Outcome::Cast(pieces)
}

/// The pieces, one per line — the failure evidence, since the fixture is gone
/// by the time an assertion runs.
fn report(pieces: &[Piece]) -> String {
    pieces
        .iter()
        .map(|p| {
            format!(
                "\n    {:>6} faces  extent=[{:.3}, {:.3}, {:.3}] mm  z=[{:.3}, {:.3}]",
                p.faces, p.extent[0], p.extent[1], p.extent[2], p.z_min, p.z_max
            )
        })
        .collect()
}

/// The invariant: whatever a cast does with an inset, it never hands the
/// workshop a plug in two pieces.
///
/// Written RED — it failed at 8 and 11 mm at `0b0b0786`, one commit before the
/// refusal that turns it green.
///
/// ★ It stays shaped as a VERDICT rather than a geometry assertion, and that
/// outlives the choice it was written under. Refusing the inset (what the cast
/// now does), dropping the lock for the documented hand-positioning mode, and
/// growing the lock into a pedestal that spans the gap all satisfy it — so
/// replacing the refusal with a pedestal later needs no edit here.
///
/// ⚠ The whole sweep runs before anything is asserted. Failing on the first
/// bad inset would stop at 8 mm and leave the refusal at 20 mm unreached — a
/// branch no run ever enters is not a gate — and it would report one row of a
/// defect whose shape is the SPREAD across the range.
#[test]
fn a_cast_never_emits_a_detached_plug_lock() {
    let mut problems = Vec::new();
    for inset_mm in INSETS_MM {
        match cast_at("sweep", inset_mm / 1e3, CELL_SIZE_M) {
            // Declining is an answer, as long as it says what it declined —
            // otherwise an unrelated failure (a missing prep file, say) would
            // read as an acceptable one. The pairing test below is what stops a
            // blanket refusal from passing this one.
            Outcome::Refused(msg) => {
                if !msg.contains("plug") {
                    problems.push(format!(
                        "{inset_mm} mm: declined without saying the plug was what \
                         it could not build: {msg}"
                    ));
                }
            }
            Outcome::Cast(pieces) => {
                if pieces.len() != 1 {
                    problems.push(format!(
                        "{inset_mm} mm: the plug ships in {} pieces — the floor \
                         lock is anchored to the ribbon, which does not move with \
                         the cavity inset, so the plug climbs away from it:{}",
                        pieces.len(),
                        report(&pieces)
                    ));
                }
            }
        }
    }
    // ⚠ The header stays neutral because the sweep has TWO failure modes and
    // only one of them is the anchor. Naming the ribbon here would misdiagnose
    // a refusal that simply failed to say what it declined — each problem line
    // carries its own cause instead.
    assert!(
        problems.is_empty(),
        "a cast must hand back one connected plug or say what it declined:\n{}",
        problems.join("\n")
    );
}

/// The insets the pedestal is for: past the point where this cone's plug has
/// climbed clear of its floor lock, and short of the point where the cone has
/// no plug left to mesh at all (20 mm, which keeps its empty-mesh refusal).
///
/// Both are rows of the sweep above — 8 and 11 mm shipped a loose pyramid
/// before #893 and were declined by name after it. This is where that band
/// gets its capability back.
const PEDESTAL_INSETS_MM: [f64; 2] = [8.0, 11.0];

/// Marching-cubes cell size for the pedestal gate, in meters — and it is NOT
/// [`CELL_SIZE_M`].
///
/// 1.5 mm is `Fast`, the coarser of the two the wizard actually offers. The
/// pedestal engages one cell deep, so the cell size is a demand on the plug as
/// well as on the mesher, and at the 3 mm the sweep runs for its cost this cone
/// cannot meet it: measured 2026-09-09, the deepest ANY point of its remaining
/// tip lies inside the plug at 11 mm of inset is 2.48 mm, against 3 mm asked.
/// It declines instead — the safe direction, and a cell size no operator can
/// select.
///
/// ⚠ So the capability claim below is scoped to shipped quality. Casting the
/// sweep's own 3 mm here would gate a resolution the product never uses and
/// would report the fixture's size as the feature's limit.
const PEDESTAL_CELL_SIZE_M: f64 = 0.0015;

/// ★★★ THE CAPABILITY, and the reason the sweep above cannot stand alone: it
/// ACCEPTS a refusal, so it stayed green for the whole time these insets could
/// not be cast at all.
///
/// An inset that lifts the plug's base clear of its floor lock is cast anyway,
/// with the lock carried back down to the cap plane on a pedestal — a column
/// derived from the plug that `cf_cast::add_plug_pins` used to receive and
/// ignore. `cf-studio-gui` offers 0-30 mm and the operator's own scan refused
/// at 7.
///
/// ⚠ Written RED: both rows refused by name at the commit before the pedestal.
#[test]
fn an_inset_that_lifts_the_plug_clear_of_its_lock_is_cast_anyway() {
    let mut problems = Vec::new();
    for inset_mm in PEDESTAL_INSETS_MM {
        match cast_at("pedestal", inset_mm / 1e3, PEDESTAL_CELL_SIZE_M) {
            Outcome::Refused(msg) => problems.push(format!(
                "{inset_mm} mm: declined, when the plug should have ridden down \
                 to the cap plane on a column: {msg}"
            )),
            Outcome::Cast(pieces) => {
                if pieces.len() != 1 {
                    problems.push(format!(
                        "{inset_mm} mm: the plug ships in {} pieces:{}",
                        pieces.len(),
                        report(&pieces)
                    ));
                } else if (pieces[0].z_min - LOCK_BASE_Z_MM).abs() >= 0.1 {
                    problems.push(format!(
                        "{inset_mm} mm: the plug's lowest point is {:.3} mm, not \
                         the {LOCK_BASE_Z_MM} mm the floor lock reaches — the \
                         column carried the plug down but not the lock:{}",
                        pieces[0].z_min,
                        report(&pieces)
                    ));
                }
            }
        }
    }
    assert!(
        problems.is_empty(),
        "an inset the plug has climbed away from is one the pedestal makes \
         castable:\n{}",
        problems.join("\n")
    );
}

/// The pairing, and it carries its own weight twice over.
///
/// ⚠ Without it the invariant above passes VACUOUSLY two ways: a cast that
/// refuses every inset satisfies it, and so does one that never emits a lock at
/// all — no pyramid, no second piece. Verified 2026-09-07 by disabling the lock
/// in the cf-cast-cli bridge, which leaves ONE component at 0 mm inset with
/// `z_min` 0.001 instead of -4.000.
///
/// ★★ It is also the guard on the one configuration that has been physically
/// poured. 5 mm is the GUI default and `base_mold`'s validated config; a fix
/// for the high insets that perturbs this one has broken something real, and a
/// print, not a test run, is what re-validates it.
#[test]
fn the_poured_configuration_still_gets_its_floor_lock() {
    let Outcome::Cast(pieces) = cast_at("poured", 0.005, CELL_SIZE_M) else {
        panic!("5 mm is the shipped default and the poured config — it must cast");
    };
    assert_eq!(
        pieces.len(),
        1,
        "the poured configuration casts as one piece:{}",
        report(&pieces)
    );
    assert!(
        (pieces[0].z_min - LOCK_BASE_Z_MM).abs() < 0.1,
        "the plug's lowest point is {:.3} mm, not the {LOCK_BASE_Z_MM} mm the \
         floor lock should reach — without it the plug has nothing to seat \
         into:{}",
        pieces[0].z_min,
        report(&pieces),
    );
}
