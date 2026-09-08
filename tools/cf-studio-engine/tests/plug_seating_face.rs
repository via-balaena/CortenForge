//! The plug's seating face may be dimpled, but it may never be PROUD — checked
//! on the STL that SHIPS, not on the SDF that produced it.
//!
//! ★★★ WHY THIS EXISTS. Every flatness assertion in the tree before this file
//! was a unit test on the SDF — `cf_cast::piece::tests::
//! mating_face_is_mathematically_flat_and_coplanar` and `mesh_csg::tests::
//! f4_synthetic_presdf_seam_is_bit_precise_flat`. The SDF being mathematically
//! flat is not the claim anyone depends on; the claim is that the mesh handed
//! to the slicer is. That is the same gap that let a detached plug lock ship
//! through a green suite until #893: the geometry was right, the artifact was
//! not.
//!
//! ## The claim, and why it is ONE-SIDED
//!
//! The plug's cap plane is the seating face, trimmed to z = +1 µm by
//! `build_plug_cap_trim_transform`. Its EDGE carries a chamfer band that
//! `docs/CF_CAST_CAP_PLANE_FLATNESS_BOOKMARK.md` measured on 2026-05-25 and
//! ACCEPTED as expected marching-cubes quantization, after a git-bisect found
//! the PR #255 era had a bit-precisely identical band. That decision stands.
//!
//! So this gates the property the chamfer does not violate and a real defect
//! does: **nothing may protrude BELOW the trim plane.** Material missing above
//! it is a recessed dimple and still seats; material below it is what breaks
//! the seal and the print. Gating it one-sided admits the accepted chamfer by
//! construction rather than reopening a settled call.
//!
//! ## ⚠ The cell sizes are the ones that SHIP
//!
//! `cell_size_m_for_quality` offers **0.5 mm Fine** (the print default) and
//! **1.5 mm Fast preview**, and this gate sweeps both — a fine grid resolves
//! thin features a coarse one misses, and the claim here is about the artifact
//! a printer actually receives.
//!
//! An earlier revision of this PR also gated the CUP SEAM, at 3 mm cells, where
//! it measured beautiful numbers that did not survive contact with either real
//! quality. 3 mm is not a shippable size: `tools/cf-studio-gui/src/lib.rs` says
//! outright that *"3 mm is never offered (it drops the flange web)"*, so those
//! numbers described a cup nobody prints.
//!
//! ⚠ That is NOT an indictment of `plug_lock_connectivity`, which casts at 3 mm
//! deliberately and is right to. A dropped flange web is a CUP feature and
//! cannot reach a plug-only cast, and that gate treats a refusal as an
//! acceptable answer, so it tolerates the detach threshold moving with the
//! grid. A seam gate could do neither.

// `unwrap`/`expect`/`panic` are the integration-test idiom here: the crate
// denies them for library code, where errors are values, but a test failure
// has to be readable.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use cf_studio_engine::{PartId, PartSelection};
use cortenforge::mesh::types::IndexedMesh;

mod common;
use common::{CastOutcome, cast_synthetic, open_cone};

/// The mesh cell sizes Cendrillon actually offers, in meters — `Fast preview`
/// and `Fine`, per `cf_studio_gui::cell_size_m_for_quality`.
///
/// ⚠ Both, not one. The plug's cell size follows this quality knob (the
/// `CanalSpec::plug_mesh_cell_size_m` override applies only on the canal path,
/// which Cendrillon does not take), and cell size moves real geometry: the
/// floor lock detaches from 12 mm of inset at 1.5 mm cells and 14 mm at 0.5 mm.
/// A gate pinned to one size cannot see a defect that only appears at the
/// other. → [[feedback-vary-the-fixture-input-not-just-the-assertion]]
const SHIPPED_CELL_SIZES_M: [f64; 2] = [0.0015, 0.0005];

/// How far a plug vertex may sit BELOW the cap-plane trim, in mm.
///
/// 0.1 µm — two orders under the accepted edge-chamfer band and far under any
/// real protrusion, while leaving room for f64 noise. Measured: exactly zero
/// vertices below, at every inset and every cell size swept here.
const PLUG_PROTRUSION_MAX_MM: f64 = 1e-4;

/// The plug's cap-plane trim height, in mm: `build_plug_cap_trim_transform`
/// shifts the plane 1 µm inward so the trim bites rather than grazing.
const PLUG_TRIM_Z_MM: f64 = 0.001;

/// How steeply a facet must face downward to count as seating face: -cos 15°,
/// since a normal pointing straight down is -1, not +1.
///
/// The seating face is planar, so its own facets sit at exactly -1. The margin
/// is for the chamfered rim, not for admitting steep walls.
const SEATING_FACE_MAX_TILT_COS: f64 = -0.966;

/// Below this height, in mm, a downward facet belongs to the floor lock rather
/// than to the seating face.
///
/// ⚠ Coupled to the lock's geometry, not free. The lock is symmetric across the
/// cap plane with a 4 mm half-length, so its own base sits at z = -4 mm
/// (`plug_lock_connectivity::LOCK_BASE_Z_MM`) and this clears it by 1 mm. If
/// that half-length ever shrinks past 3 mm the lock's base lands ABOVE this
/// line, gets counted as seating face, and this gate starts reporting a
/// millimetres-deep protrusion that is really the lock doing its job.
const SEATING_FACE_MIN_Z_MM: f64 = -3.0;

/// The scan this gate drives.
///
/// r0 = 12 mm rather than the 6 mm `plug_lock_connectivity` uses: that cone
/// lifts off at `r0·cos a` = 5.44 mm and its seating face has already shrunk to
/// nothing by 2 mm of inset, which would leave this gate measuring an empty set.
fn wide_cone() -> IndexedMesh {
    open_cone(0.012, 0.026, 0.030, 24, 13)
}

fn normal(mesh: &IndexedMesh, f: [u32; 3]) -> [f64; 3] {
    let p = |i: u32| mesh.vertices[i as usize];
    let (a, b, c) = (p(f[0]), p(f[1]), p(f[2]));
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len == 0.0 {
        [0.0, 0.0, 0.0]
    } else {
        [n[0] / len, n[1] / len, n[2] / len]
    }
}

/// The heights, in mm, of every vertex on the plug's cap-plane seating face:
/// the downward-facing facets that are not the floor lock.
fn seating_face_heights(mesh: &IndexedMesh) -> Vec<f64> {
    mesh.faces
        .iter()
        .copied()
        .filter(|&f| normal(mesh, f)[2] < SEATING_FACE_MAX_TILT_COS)
        .filter(|f| {
            f.iter()
                .all(|&v| mesh.vertices[v as usize][2] > SEATING_FACE_MIN_Z_MM)
        })
        .flat_map(|f| {
            f.iter()
                .map(|&v| mesh.vertices[v as usize][2])
                .collect::<Vec<_>>()
        })
        .collect()
}

/// Cast the wide cone's layer-0 plug at `cell_size_m`, or say why it declined.
fn cast_plug(caller: &str, inset_mm: f64, cell_size_m: f64) -> CastOutcome {
    cast_synthetic(
        caller,
        wide_cone(),
        inset_mm / 1e3,
        cell_size_m,
        &PartSelection::from_ids([PartId::Plug { layer_index: 0 }]),
    )
}

/// The invariant: whatever inset and quality a cast is given, the plug it hands
/// the workshop never stands proud of its own seating plane.
#[test]
fn the_plug_never_protrudes_below_its_cap_plane_trim() {
    let mut problems = Vec::new();
    let mut measured = 0usize;
    // 0 is the degenerate floor; 5 is the shipped default and the one
    // configuration that has been physically poured.
    for cell_size_m in SHIPPED_CELL_SIZES_M {
        for inset_mm in [0.0_f64, 2.0, 5.0] {
            let caller = format!("seating-{}-{inset_mm}", (cell_size_m * 1e4).round() as i64);
            let emitted = match cast_plug(&caller, inset_mm, cell_size_m) {
                CastOutcome::Cast(emitted) => emitted,
                // ⚠ Carry the reason. A refusal reported as "it must cast"
                // reads identically whether the plug detached or a prep file
                // went missing, and the second is not this gate's business.
                CastOutcome::Refused(why) => {
                    problems.push(format!(
                        "{inset_mm} mm at {cell_size_m} m cells: the wide cone must cast \
                         here, but it declined: {why}"
                    ));
                    continue;
                }
            };
            assert_eq!(
                emitted.len(),
                1,
                "plug-only selection emits one STL, got {:?}",
                emitted.iter().map(|(n, _)| n).collect::<Vec<_>>()
            );
            let seating = seating_face_heights(&emitted[0].1);
            // ⚠ No seating face means nothing was checked, and a plug with no
            // flat bottom does not seat. The wide cone is chosen so this holds;
            // if it ever stops, the gate must be told, not quietly satisfied.
            assert!(
                !seating.is_empty(),
                "{inset_mm} mm at {cell_size_m} m cells: the plug has no \
                 cap-plane seating face at all"
            );
            measured += seating.len();

            let floor = PLUG_TRIM_Z_MM - PLUG_PROTRUSION_MAX_MM;
            let proud: Vec<f64> = seating.into_iter().filter(|z| *z < floor).collect();
            if !proud.is_empty() {
                let worst = proud.iter().cloned().fold(f64::MAX, f64::min);
                problems.push(format!(
                    "{inset_mm} mm at {cell_size_m} m cells: {} seating-face vertices sit \
                     below the {PLUG_TRIM_Z_MM} mm trim plane, worst {worst:.6} mm \
                     ({:.1} µm proud). A recessed dimple seats; a proud one does not.",
                    proud.len(),
                    (PLUG_TRIM_Z_MM - worst) * 1000.0,
                ));
            }
        }
    }
    // ⚠ The whole sweep runs before anything is asserted: a defect that appears
    // at one quality and not the other has a SHAPE, and failing on the first
    // one would report a single row of it.
    assert!(
        problems.is_empty(),
        "the plug's seating face must never protrude:\n{}",
        problems.join("\n")
    );
    println!("checked {measured} seating-face vertices across 3 insets x 2 qualities");
}

/// ★★★ THE GATE ABOVE IS VACUOUS UNTIL IT IS MADE TO FAIL.
///
/// The risk on it is not its predicate — `z < floor` is hard to get wrong — it
/// is the SELECTOR. If `seating_face_heights` returned a set that cannot
/// contain a proud vertex by construction, the gate would pass forever while
/// measuring nothing. This pushes the real seating face below the trim plane
/// and requires the selector to surface it.
#[test]
fn the_protrusion_check_catches_a_proud_seating_face() {
    for cell_size_m in SHIPPED_CELL_SIZES_M {
        let caller = format!("seating-control-{}", (cell_size_m * 1e4).round() as i64);
        let emitted = match cast_plug(&caller, 5.0, cell_size_m) {
            CastOutcome::Cast(emitted) => emitted,
            CastOutcome::Refused(why) => panic!(
                "5 mm is the shipped default and the poured config — it must cast, \
                 but it declined: {why}"
            ),
        };
        let mut mesh = emitted[0].1.clone();

        let floor = PLUG_TRIM_Z_MM - PLUG_PROTRUSION_MAX_MM;
        let before = seating_face_heights(&mesh);
        assert!(
            !before.is_empty() && before.iter().all(|z| *z >= floor),
            "at {cell_size_m} m cells the control must start with a seating face \
             that is not proud"
        );

        // Drop every vertex sitting on the trim plane by 50 µm — the plug's
        // bottom now stands proud of where the cup floor expects it.
        const DROP_MM: f64 = 0.05;
        let mut moved = 0usize;
        for v in &mut mesh.vertices {
            if (v[2] - PLUG_TRIM_Z_MM).abs() < 1e-9 {
                v[2] -= DROP_MM;
                moved += 1;
            }
        }
        assert!(
            moved > 0,
            "at {cell_size_m} m cells the control must actually move the seating face"
        );

        let proud = seating_face_heights(&mesh)
            .iter()
            .filter(|z| **z < floor)
            .count();
        assert!(
            proud > 0,
            "a seating face dropped {:.0} µm below the trim plane across {moved} vertices \
             was not seen as proud at {cell_size_m} m cells — the selector does not reach \
             the face it claims to",
            DROP_MM * 1000.0
        );
    }
}
