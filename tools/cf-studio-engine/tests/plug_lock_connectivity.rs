//! A cast either honours the cavity inset it was given or declines it. What it
//! must never do is emit the mold anyway with the plug's floor lock lying
//! loose beside it.
//!
//! ★★★ THE DEFECT. `cf_cast::add_plug_pins(plug, ribbon)` takes the ribbon and
//! nothing else, and both transforms it returns are anchored on the ribbon's
//! cap-plane. The ribbon is the cleaned scan's centerline: it does not move
//! when `cavity_inset_m` changes. The plug is `scan.offset(-inset)`, and it
//! does. Past a threshold the plug's base has climbed clear of the lock and the
//! two mesh as separate bodies — which shipped as a loose pyramid in the print
//! and a plug with nothing holding it in the mold, until the refusal that lands
//! with this file. `cf-studio-gui` offers 0-30 mm.
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

use std::path::{Path, PathBuf};

use cf_studio_core::{DesignDraft, LayerDraft, RidgeOptions};
use cf_studio_engine::{CastMode, EditSession, PartId, PartSelection, generate_molds_for_design};
use cortenforge::mesh::io::load_stl;
use cortenforge::mesh::repair::{components::find_connected_components, weld_vertices};
use cortenforge::mesh::types::{IndexedMesh, Point3};

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
/// ⚠ Load-bearing, not a speed knob: the detachment threshold moves with it,
/// because what bridges the last of the gap is the thin tip of the offset solid
/// and a coarse grid cannot resolve it. On this cone the plug detaches from
/// 6.0 mm of inset at this cell size, 7.0 mm at 1.5 mm cells, and 8.0 mm at
/// 1.0 mm.
///
/// The sweep survives that: measured 2026-09-07, both tests pass at 3.0, 1.5
/// AND 1.0 mm cells (2.9 / 5.6 / 7.3 s), since every inset in [`INSETS_MM`]
/// either casts whole or is declined at each of them. 3 mm is here for the
/// cost, not because the gate needs it.
const CELL_SIZE_M: f64 = 0.003;

/// A cone open at both ends — `r0` at the base, `r1` at the top, `h` tall. Open
/// so cap detection finds the two boundary loops the centerline is fitted
/// between; tapered so the plug's base recedes from the cap plane under an
/// inward offset, which is the condition the defect needs.
fn open_cone(r0: f64, r1: f64, h: f64, segs: usize, rings: usize) -> IndexedMesh {
    let mut vertices = Vec::new();
    for i in 0..rings {
        let f = i as f64 / (rings - 1) as f64;
        let (z, r) = (h * f, r0 + (r1 - r0) * f);
        for s in 0..segs {
            let a = std::f64::consts::TAU * s as f64 / segs as f64;
            vertices.push(Point3::new(r * a.cos(), r * a.sin(), z));
        }
    }
    let mut faces = Vec::new();
    for i in 0..rings - 1 {
        let b = (i * segs) as u32;
        let t = ((i + 1) * segs) as u32;
        for s in 0..segs {
            let s2 = ((s + 1) % segs) as u32;
            let s = s as u32;
            faces.push([b + s, b + s2, t + s2]);
            faces.push([b + s, t + s2, t + s]);
        }
    }
    IndexedMesh { vertices, faces }
}

/// One connected piece of the emitted plug: its face count and its z-range.
struct Piece {
    faces: usize,
    z_min: f64,
    z_max: f64,
    extent: [f64; 3],
}

/// What a cast at one inset produced: either a refusal, or the plug's pieces
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

/// Cast the cone at `inset_m`, in a fixture directory named for `caller`.
///
/// One layer and layer 0's plug only: the lock is a layer-0 cap-plane feature,
/// and the cup halves cost more than the whole rest of the gate.
///
/// ⚠ `caller` is what keeps the two tests apart, and it is load-bearing rather
/// than decorative. Both of them cast 5 mm, `cargo test` runs them on parallel
/// threads of ONE process, and this function opens by DELETING the directory it
/// is about to build in — so a name derived from the inset alone gives both the
/// same path and lets one wipe the other's fixture mid-cast.
fn cast_at(caller: &str, inset_m: f64) -> Outcome {
    let label = format!("plug-lock-{caller}-{}", (inset_m * 1e4).round() as i64);
    let dir = std::env::temp_dir().join(format!("cf-studio-engine-{label}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();

    let mut session = EditSession::from_mesh(
        PathBuf::from("synthetic.stl"),
        open_cone(0.006, 0.020, 0.030, 24, 13),
    );
    let scan = session.detect_caps();
    assert_eq!(scan.loop_count, 2, "an open cone has two boundary loops");
    session
        .save(&dir, "synthetic", "mm", 0)
        .expect("prep saves");

    let draft = DesignDraft {
        cavity_inset_m: inset_m,
        layers: vec![LayerDraft {
            thickness_m: 0.006,
            material_key: "ECOFLEX_00_30".to_string(),
            slacker_fraction: 0.25,
        }],
    };
    let cast = generate_molds_for_design(
        &dir.join("synthetic.cleaned.stl"),
        &dir.join("synthetic.prep.toml"),
        &draft,
        CELL_SIZE_M,
        &RidgeOptions::default(),
        &PartSelection::from_ids([PartId::Plug { layer_index: 0 }]),
        CastMode::Detachable,
        Some(Path::new("out")),
    );
    let out = match cast {
        Ok(out) => out,
        Err(e) => {
            let _ = std::fs::remove_dir_all(&dir);
            return Outcome::Refused(e.to_string());
        }
    };

    // Read the STL while the fixture still exists — a returned `PathBuf`
    // outlives the file it names.
    let plug = out
        .plug_stls
        .first()
        .expect("a cast that succeeded emitted the plug it was asked for");
    let mut mesh = load_stl(plug).unwrap();
    let _ = std::fs::remove_dir_all(&dir);

    // Weld first: marching cubes emits per-triangle vertices, so an unwelded
    // mesh has as many components as it has faces. 1 um, the tolerance
    // `design/cf-cast/tests/iter_connectivity_inspector.rs` uses on the same
    // question.
    weld_vertices(&mut mesh, 1e-6);
    let mut pieces: Vec<Piece> = find_connected_components(&mesh)
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
        match cast_at("sweep", inset_mm / 1e3) {
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
                        "{inset_mm} mm: the plug ships in {} pieces:{}",
                        pieces.len(),
                        report(&pieces)
                    ));
                }
            }
        }
    }
    assert!(
        problems.is_empty(),
        "the floor lock is anchored to the ribbon, which does not move with the \
         cavity inset — so the plug climbs away from it:\n{}",
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
    let Outcome::Cast(pieces) = cast_at("poured", 0.005) else {
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
