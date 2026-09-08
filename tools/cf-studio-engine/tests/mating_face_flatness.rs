//! The mold's two mating faces have to be flat on the STL that SHIPS, not just
//! in the SDF that produced it.
//!
//! ★★★ WHY THIS EXISTS. Every flatness assertion in the tree before this file
//! was a unit test on the SDF — `cf_cast::piece::tests::
//! mating_face_is_mathematically_flat_and_coplanar` and `mesh_csg::tests::
//! f4_synthetic_presdf_seam_is_bit_precise_flat`. The SDF being mathematically
//! flat is not the claim anyone depends on; the claim is that the mesh handed
//! to the slicer is. That gap is exactly the class that shipped a detached plug
//! lock through a green suite until #893 — the geometry was right and the
//! artifact was not.
//!
//! ## The two faces, and why they do NOT get the same rule
//!
//! **The cup seam** is the hard one. The workshop's proven configuration has
//! GASKET NONE: the bolt clamp IS the seal, so a seam that domes or bulges
//! leaks. Nothing may round it — no fillet, gusset, `smooth_union`, or post-MC
//! slab that touches the mating plane. Gated two-sided, tight.
//!
//! **The plug's cap plane** is the seating face, trimmed to z = +1 µm by
//! `build_plug_cap_trim_transform`. Its EDGE carries a chamfer band that
//! `docs/CF_CAST_CAP_PLANE_FLATNESS_BOOKMARK.md` measured on 2026-05-25 and
//! ACCEPTED as expected marching-cubes quantization, after a git-bisect showed
//! the PR #255 era had a bit-precisely identical band. That decision stands.
//! So this file gates the property the chamfer does not violate and a real
//! defect does: **nothing may protrude BELOW the trim plane.** Material missing
//! above it is a recessed dimple and still seats; material below it is what
//! breaks the seal and the print.
//!
//! Measured 2026-09-08 on `main` `dfa11ac2`, this fixture, 3 mm cells: seam
//! deviation within ±0.27 µm on both halves, and not one plug vertex below the
//! trim plane at any inset that HAS a seating face.
//!
//! ⚠ That qualifier is load-bearing. On a narrow cone (r0 = 6 mm, lift-off at
//! `r0·cos a` = 5.44 mm) the seating face has shrunk to nothing by 2 mm of
//! inset, so "zero vertices below the plane" there is vacuously true of an
//! empty set. This gate runs the wide cone and asserts the face exists before
//! believing anything it says about it.

// `unwrap`/`expect`/`panic` are the integration-test idiom here: the crate
// denies them for library code, where errors are values, but a test failure
// has to be readable.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use cf_studio_engine::{PartId, PartSelection, PieceSide};
use cortenforge::mesh::types::IndexedMesh;

mod common;
use common::{CastOutcome, cast_synthetic, open_cone};

/// Marching-cubes cell size, in meters. Matches `plug_lock_connectivity` so the
/// two gates describe the same artifact.
const CELL_SIZE_M: f64 = 0.003;

/// How far a seam vertex may sit off its own plane, in microns.
///
/// ★ The line sits between two MEASURED points, not at a comfortable round
/// number with headroom quoted after the fact. What ships today strays
/// **0.27 µm**. Re-introducing a defect this pipeline actually had — the
/// pre-§M-S1 `RIBBON_PIECE_OVERLAP_M = 0.5 mm`, whose comment at
/// `piece.rs:690` records that it "created a stepped mating face" — puts the
/// seam at **21.14 µm**. 10 µm passes production and fails the regression, and
/// both ends of that were run, not reasoned.
const SEAM_FLATNESS_MAX_UM: f64 = 10.0;

/// How far a plug vertex may sit BELOW the cap-plane trim, in mm.
///
/// 0.1 µm — two orders under the accepted edge-chamfer band, and far under any
/// real protrusion, while leaving room for f64 noise. Measured: exactly zero
/// vertices below, at every inset on both cones.
const PLUG_PROTRUSION_MAX_MM: f64 = 1e-4;

/// The plug's cap-plane trim height, in mm: `build_plug_cap_trim_transform`
/// shifts the plane 1 µm inward so the trim bites rather than grazing.
const PLUG_TRIM_Z_MM: f64 = 0.001;

/// A planar face of a mesh: the plane, how much area sits on it, and how far
/// each of its vertices strays from it.
struct Plane {
    normal: [f64; 3],
    /// Signed distance from the origin along `normal`, in mm.
    offset: f64,
    area: f64,
    /// Per-vertex deviation from the cluster's median plane, in microns.
    devs_um: Vec<f64>,
}

impl Plane {
    /// The worst stray either side of the plane, in microns.
    fn worst_um(&self) -> f64 {
        self.devs_um.iter().fold(0.0_f64, |w, d| w.max(d.abs()))
    }
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

fn area(mesh: &IndexedMesh, f: [u32; 3]) -> f64 {
    let p = |i: u32| mesh.vertices[i as usize];
    let (a, b, c) = (p(f[0]), p(f[1]), p(f[2]));
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    0.5 * (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt()
}

/// Every planar face of `mesh`, largest area first.
///
/// ⚠ Faces are bucketed by normal AND plane offset together. Bucketing on the
/// normal alone lumps every parallel plane into one: measured on a mold half,
/// the `+Z` cluster then spanned 39.25 mm and reported that as its "deviation"
/// — a nonsense number that still arrived as a clean pass, because nothing
/// about it was an error. → [[feedback-your-verification-tooling-lies-quietly]]
///
/// The buckets are a `BTreeMap` rather than a `HashMap` so the returned order
/// is the same on every run — hash iteration order is how a scan trim ended up
/// picking the wrong end 50/50.
fn planar_faces(mesh: &IndexedMesh) -> Vec<Plane> {
    // 1/500 in each normal component, and 0.05 mm of offset: wide enough to
    // hold one marching-cubes plane together, narrow enough to keep two real
    // planes apart.
    let nkey = |n: [f64; 3]| {
        (
            (n[0] * 500.0).round() as i64,
            (n[1] * 500.0).round() as i64,
            (n[2] * 500.0).round() as i64,
        )
    };
    let mut buckets: BTreeMap<((i64, i64, i64), i64), (f64, [f64; 3], Vec<f64>)> = BTreeMap::new();
    for &f in &mesh.faces {
        let n = normal(mesh, f);
        if n == [0.0, 0.0, 0.0] {
            continue;
        }
        let offs: Vec<f64> = f
            .iter()
            .map(|&v| {
                let p = mesh.vertices[v as usize];
                p[0] * n[0] + p[1] * n[1] + p[2] * n[2]
            })
            .collect();
        let mid = offs.iter().sum::<f64>() / 3.0;
        let entry = buckets
            .entry((nkey(n), (mid / 0.05).round() as i64))
            .or_insert((0.0, n, Vec::new()));
        entry.0 += area(mesh, f);
        entry.1 = n;
        entry.2.extend(offs);
    }
    let mut planes: Vec<Plane> = buckets
        .into_values()
        .map(|(area, normal, mut offs)| {
            offs.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let offset = offs[offs.len() / 2];
            let devs_um = offs.iter().map(|o| (o - offset) * 1000.0).collect();
            Plane {
                normal,
                offset,
                area,
                devs_um,
            }
        })
        .collect();
    planes.sort_by(|a, b| b.area.partial_cmp(&a.area).unwrap());
    planes
}

/// The mating plane of two cup halves: the plane they SHARE — coincident in
/// space, with the two halves facing each other across it.
///
/// ★ That is the definition of a mating face, and it is why this is not simply
/// "the largest flat face". On this fixture the largest flat face of a mold
/// half is the box's own top (742 mm² at z = 35), comfortably bigger than the
/// seam (525 mm²). Pairing the halves picks the seam out on the property that
/// makes it the seam.
fn shared_mating_plane<'a>(a: &'a [Plane], b: &'a [Plane]) -> Option<(&'a Plane, &'a Plane)> {
    let mut best: Option<(&Plane, &Plane)> = None;
    for pa in a {
        for pb in b {
            let dot = pa.normal[0] * pb.normal[0]
                + pa.normal[1] * pb.normal[1]
                + pa.normal[2] * pb.normal[2];
            // Facing each other, and the same plane in space: with antiparallel
            // normals the two signed offsets must sum to zero.
            if dot > -0.99 || (pa.offset + pb.offset).abs() > 0.05 {
                continue;
            }
            let weight = pa.area.min(pb.area);
            if best.is_none_or(|(ba, bb)| weight > ba.area.min(bb.area)) {
                best = Some((pa, pb));
            }
        }
    }
    best
}

/// Cast the wide cone and return its two cup halves' planar faces.
///
/// r0 = 12 mm rather than the 6 mm `plug_lock_connectivity` uses: that cone
/// lifts off at `r0·cos a` = 5.44 mm and its seating face has already shrunk to
/// nothing by 2 mm of inset, which would make the plug half of this gate pass
/// on an empty set.
fn cup_halves(caller: &str, inset_mm: f64) -> Vec<(String, IndexedMesh)> {
    let outcome = cast_synthetic(
        &format!("flatness-{caller}"),
        open_cone(0.012, 0.026, 0.030, 24, 13),
        inset_mm / 1e3,
        CELL_SIZE_M,
        &PartSelection::from_ids([
            PartId::Cup {
                layer_index: 0,
                side: PieceSide::Negative,
            },
            PartId::Cup {
                layer_index: 0,
                side: PieceSide::Positive,
            },
        ]),
    );
    match outcome {
        CastOutcome::Refused(msg) => {
            panic!("{inset_mm} mm must cast on the wide cone, but: {msg}")
        }
        CastOutcome::Cast(meshes) => meshes,
    }
}

/// THE HARD ONE. The bolt clamp is the seal, so the seam the two halves clamp
/// across must be flat on the mesh that ships.
#[test]
fn the_cup_halves_seam_is_flat_on_the_shipped_mesh() {
    let mut problems = Vec::new();
    // 0 is the degenerate floor; 5 is the shipped default and the one
    // configuration that has been physically poured.
    for inset_mm in [0.0_f64, 5.0] {
        let halves = cup_halves(&format!("seam-{inset_mm}"), inset_mm);
        assert_eq!(
            halves.len(),
            2,
            "{inset_mm} mm: a two-sided selection emits two halves, got {:?}",
            halves.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
        let a = planar_faces(&halves[0].1);
        let b = planar_faces(&halves[1].1);
        let Some((pa, pb)) = shared_mating_plane(&a, &b) else {
            problems.push(format!(
                "{inset_mm} mm: the two halves share no mating plane at all — \
                 they cannot clamp"
            ));
            continue;
        };
        for (piece, plane) in [(&halves[0].0, pa), (&halves[1].0, pb)] {
            // ⚠ An empty plane would satisfy any bound. Assert the evidence
            // exists before believing what it says.
            assert!(
                plane.devs_um.len() >= 3,
                "{inset_mm} mm {piece}: the mating plane has {} vertices — \
                 nothing was measured",
                plane.devs_um.len()
            );
            // ⚠ A sliver satisfies any flatness bound trivially. The clamping
            // face is tens of mm² (measured 525 and 199 here), so anything
            // near zero means the pairing found something that is not the seam
            // and the bound below would be meaningless.
            assert!(
                plane.area > 10.0,
                "{inset_mm} mm {piece}: the mating plane is only {:.3} mm² — \
                 too small to be the face the halves clamp across",
                plane.area
            );
            let worst = plane.worst_um();
            if worst > SEAM_FLATNESS_MAX_UM {
                problems.push(format!(
                    "{inset_mm} mm {piece}: the seam strays {worst:.3} µm from its own \
                     plane (limit {SEAM_FLATNESS_MAX_UM}). The clamp IS the seal — nothing \
                     may round or bulge this face. n=[{:.4}, {:.4}, {:.4}] offset={:.4} mm \
                     area={:.1} mm² over {} verts",
                    plane.normal[0],
                    plane.normal[1],
                    plane.normal[2],
                    plane.offset,
                    plane.area,
                    plane.devs_um.len(),
                ));
            }
        }
    }
    assert!(
        problems.is_empty(),
        "the seam mating face must ship flat:\n{}",
        problems.join("\n")
    );
}

/// The heights, in mm, of every vertex on the plug's cap-plane seating face:
/// the downward-facing facets above the floor lock's own base at z = -4 mm.
fn seating_face_heights(mesh: &IndexedMesh) -> Vec<f64> {
    mesh.faces
        .iter()
        .copied()
        .filter(|&f| normal(mesh, f)[2] < -0.966)
        .filter(|f| f.iter().all(|&v| mesh.vertices[v as usize][2] > -3.0))
        .flat_map(|f| {
            f.iter()
                .map(|&v| mesh.vertices[v as usize][2])
                .collect::<Vec<_>>()
        })
        .collect()
}

/// The plug's seating face may be dimpled, but it may never be PROUD.
///
/// ★ One-sided on purpose. The cap-plane edge chamfer is accepted geometry
/// (2026-05-25, after a bisect), and it lies entirely above the trim plane — so
/// this gate admits it by construction rather than re-opening a settled call.
/// What it catches is the thing the chamfer is not: material pushed below the
/// plane, which stops the plug seating and breaks the seal.
#[test]
fn the_plug_never_protrudes_below_its_cap_plane_trim() {
    let mut problems = Vec::new();
    let mut measured = 0usize;
    for inset_mm in [0.0_f64, 2.0, 5.0] {
        let outcome = cast_synthetic(
            &format!("flatness-plug-{inset_mm}"),
            open_cone(0.012, 0.026, 0.030, 24, 13),
            inset_mm / 1e3,
            CELL_SIZE_M,
            &PartSelection::from_ids([PartId::Plug { layer_index: 0 }]),
        );
        let CastOutcome::Cast(emitted) = outcome else {
            panic!("{inset_mm} mm must cast on the wide cone");
        };
        let (_, mesh) = &emitted[0];

        let seating = seating_face_heights(mesh);
        // ⚠ No seating face means nothing was checked, and a plug with no flat
        // bottom does not seat. The wide cone is chosen so this holds; if it
        // ever stops, the gate must be told, not quietly satisfied.
        assert!(
            !seating.is_empty(),
            "{inset_mm} mm: the plug has no cap-plane seating face at all"
        );
        measured += seating.len();

        let floor = PLUG_TRIM_Z_MM - PLUG_PROTRUSION_MAX_MM;
        let proud: Vec<f64> = seating.into_iter().filter(|z| *z < floor).collect();
        if !proud.is_empty() {
            let worst = proud.iter().cloned().fold(f64::MAX, f64::min);
            problems.push(format!(
                "{inset_mm} mm: {} seating-face vertices sit below the {PLUG_TRIM_Z_MM} mm \
                 trim plane, worst {worst:.6} mm ({:.1} µm proud). A recessed dimple seats; \
                 a proud one does not.",
                proud.len(),
                (PLUG_TRIM_Z_MM - worst) * 1000.0,
            ));
        }
    }
    assert!(
        problems.is_empty(),
        "the plug's seating face must never protrude:\n{}",
        problems.join("\n")
    );
    println!("checked {measured} seating-face vertices across 3 insets");
}

/// ★★★ THE GATE ABOVE IS VACUOUS UNTIL IT IS MADE TO FAIL.
///
/// Both checks passed the moment they were written, which is exactly when a
/// gate is least trustworthy. This re-runs the seam measurement against a cup
/// half whose mating face has been deliberately domed, and requires it to
/// report the dome — proving the flatness check reads the geometry rather than
/// the fact that a cast succeeded.
#[test]
fn the_flatness_check_catches_a_domed_seam() {
    let halves = cup_halves("negative-control", 5.0);
    let a = planar_faces(&halves[0].1);
    let b = planar_faces(&halves[1].1);
    let (pa, _) = shared_mating_plane(&a, &b).expect("the halves share a mating plane");
    let baseline = pa.worst_um();
    assert!(
        baseline <= SEAM_FLATNESS_MAX_UM,
        "the control starts flat: {baseline:.3} µm"
    );
    let seam_normal = pa.normal;
    let seam_offset = pa.offset;
    let seam_area = pa.area;

    // Bulge the seam into a DOME: displacement greatest at the face's centre
    // and tapering to nothing at its rim.
    //
    // ⚠ It has to vary with position. The first draft of this control moved
    // every seam vertex by the same 50 µm, which TRANSLATES the plane — and a
    // translated plane is still perfectly flat, so the check passed it and was
    // right to. A constant offset is not a defect; curvature is.
    let mut domed = halves[0].1.clone();
    let on_seam = |v: &[f64; 3]| {
        (v[0] * seam_normal[0] + v[1] * seam_normal[1] + v[2] * seam_normal[2] - seam_offset).abs()
            < 0.01
    };
    let seam_verts: Vec<[f64; 3]> = domed
        .vertices
        .iter()
        .map(|v| [v[0], v[1], v[2]])
        .filter(|v| on_seam(v))
        .collect();
    assert!(
        seam_verts.len() > 100,
        "the control needs a seam to dome, found {} vertices",
        seam_verts.len()
    );
    let n = seam_verts.len() as f64;
    let centre = [
        seam_verts.iter().map(|v| v[0]).sum::<f64>() / n,
        seam_verts.iter().map(|v| v[1]).sum::<f64>() / n,
        seam_verts.iter().map(|v| v[2]).sum::<f64>() / n,
    ];
    let r_max = seam_verts
        .iter()
        .map(|v| {
            let d = [v[0] - centre[0], v[1] - centre[1], v[2] - centre[2]];
            (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
        })
        .fold(0.0_f64, f64::max);
    assert!(
        r_max > 1.0,
        "the seam must have real extent, got {r_max:.3} mm"
    );

    // 50 µm of crown — five times the bound, and the order of magnitude a
    // fillet or smooth-union leaves behind.
    const CROWN_MM: f64 = 0.05;
    let mut moved = 0usize;
    for v in &mut domed.vertices {
        let p = [v[0], v[1], v[2]];
        if !on_seam(&p) {
            continue;
        }
        let d = [p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]];
        let r = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() / r_max;
        let bulge = CROWN_MM * (1.0 - r * r);
        for k in 0..3 {
            v[k] += seam_normal[k] * bulge;
        }
        moved += 1;
    }
    assert!(moved > 0, "the control must actually perturb the seam");

    // A dome destroys the large flat face: what was one plane becomes many
    // small ones at slightly different normals. So the seam must no longer
    // appear as a plane that is both comparably large AND within bound.
    let survivor = planar_faces(&domed).into_iter().find(|p| {
        let dot = p.normal[0] * seam_normal[0]
            + p.normal[1] * seam_normal[1]
            + p.normal[2] * seam_normal[2];
        dot > 0.99 && p.area >= 0.5 * seam_area && p.worst_um() <= SEAM_FLATNESS_MAX_UM
    });
    assert!(
        survivor.is_none(),
        "a seam crowned by {:.0} µm across {moved} vertices still read as a flat \
         {:.1} mm² face — the flatness check does not measure what it claims",
        CROWN_MM * 1000.0,
        survivor.map_or(0.0, |p| p.area),
    );
}

/// ★★★ AND THE PLUG GATE IS VACUOUS UNTIL IT IS MADE TO FAIL TOO.
///
/// The risk on that gate is not its predicate — `z < floor` is hard to get
/// wrong — it is the SELECTOR. If `seating_face_heights` returned a set that
/// cannot contain a proud vertex by construction, the gate would pass forever
/// while measuring nothing. This pushes the real seating face below the trim
/// plane and requires the selector to surface it.
#[test]
fn the_protrusion_check_catches_a_proud_seating_face() {
    let outcome = cast_synthetic(
        "flatness-plug-control",
        open_cone(0.012, 0.026, 0.030, 24, 13),
        0.005,
        CELL_SIZE_M,
        &PartSelection::from_ids([PartId::Plug { layer_index: 0 }]),
    );
    let CastOutcome::Cast(emitted) = outcome else {
        panic!("5 mm is the shipped default and the poured config — it must cast");
    };
    let mut mesh = emitted[0].1.clone();

    let floor = PLUG_TRIM_Z_MM - PLUG_PROTRUSION_MAX_MM;
    let before = seating_face_heights(&mesh);
    assert!(
        !before.is_empty() && before.iter().all(|z| *z >= floor),
        "the control starts with a seating face that is not proud"
    );

    // Drop every vertex sitting on the trim plane by 50 µm — the plug's bottom
    // now stands proud of where the cup floor expects it.
    const DROP_MM: f64 = 0.05;
    let mut moved = 0usize;
    for v in &mut mesh.vertices {
        if (v[2] - PLUG_TRIM_Z_MM).abs() < 1e-9 {
            v[2] -= DROP_MM;
            moved += 1;
        }
    }
    assert!(moved > 0, "the control must actually move the seating face");

    let after = seating_face_heights(&mesh);
    let proud = after.iter().filter(|z| **z < floor).count();
    assert!(
        proud > 0,
        "a seating face dropped {:.0} µm below the trim plane across {moved} vertices \
         was not seen as proud — the selector does not reach the face it claims to",
        DROP_MM * 1000.0
    );
}
