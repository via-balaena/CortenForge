//! The synthetic scan the cast gates in this directory are driven from, and the
//! one-call cast that turns it into shipped STLs.
//!
//! ★ Synthetic on purpose. The real configuration lives in `~/scans`, outside
//! the repo, where a gate rots silently because nothing makes it run — which is
//! how the end-to-end mold path sat `#[ignore]`d behind an empty fixture
//! directory from 2026-05-21. A cone costs under a second and runs everywhere.

// `unwrap`/`expect`/`panic` are the integration-test idiom here (same
// convention as `design/cf-cast/tests/*.rs`): the crate denies them for library
// code, where errors are values, but a test failure has to be readable.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::path::{Path, PathBuf};

use cf_studio_core::{DesignDraft, LayerDraft, PrepInput, RidgeOptions};
use cf_studio_engine::{
    CastMode, EditSession, PROBE_LAYER_THICKNESS_M, PartSelection, generate_molds_for_design,
};
use cortenforge::mesh::io::load_stl;
use cortenforge::mesh::repair::weld_vertices;
use cortenforge::mesh::types::{IndexedMesh, Point3};

/// A cone open at both ends — `r0` at the base, `r1` at the top, `h` tall. Open
/// so cap detection finds the two boundary loops the centerline is fitted
/// between; tapered so the plug's base recedes from the cap plane under an
/// inward offset.
///
/// ★★ The taper is what makes this fixture able to show anything. A straight
/// tube's inward offset shrinks it laterally while the cap-plane cut keeps its
/// base pinned, so the plug meets the cap plane at every inset and neither the
/// detachment of `plug_lock_connectivity` nor the seating-face changes of
/// `plug_seating_face` ever appear.
pub fn open_cone(r0: f64, r1: f64, h: f64, segs: usize, rings: usize) -> IndexedMesh {
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

/// What a cast produced: either a refusal, or the STLs it wrote.
///
/// ⚠ A refusal is a LEGITIMATE answer. An inset that leaves the plug
/// unreachable by its floor lock is one the cast may decline rather than
/// honour — see `CastError::PlugMatingFeatureDetached`. What it must never do
/// is emit the mold anyway. Every gate here has to decide for itself what a
/// refusal means for its own claim.
pub enum CastOutcome {
    /// The cast declined, carrying the reason it gave.
    Refused(String),
    /// The cast succeeded: `(file name, welded mesh)` per STL written.
    Cast(Vec<(String, IndexedMesh)>),
}

/// Cast `scan` at `inset_m`, emitting only `parts`, and load what it wrote.
///
/// ⚠ `caller` keeps concurrent casts apart and is load-bearing rather than
/// decorative. Tests in one binary run on parallel threads of ONE process, and
/// this function opens by DELETING the directory it is about to build in — so
/// two casts whose label collides let one wipe the other's fixture mid-run.
/// Give every call site a distinct `caller`.
///
/// Meshes come back WELDED. Marching cubes emits per-triangle vertices, so an
/// unwelded mesh has as many connected components as it has faces. Welding
/// merges coincident vertices without moving any of them, so it is safe for
/// geometry gates as well as connectivity ones.
///
/// ⚠ The tolerance is `1e-6` in the mesh's own units, and cf-cast writes STLs
/// in MILLIMETRES — so it is 1 nanometre, not the 1 µm that
/// `design/cf-cast/tests/iter_connectivity_inspector.rs` calls the same
/// constant. That is fine here rather than lucky: the corners marching cubes
/// shares between triangles are bit-identical, so even a zero tolerance would
/// merge them. Do not read this as headroom for near-coincident vertices —
/// there is none.
// ⚠ `common` is compiled separately INTO each test binary, so an item every
// binary does not use reads as dead code in the ones that do not. This is used
// by `plug_lock_connectivity` and `plug_seating_face`; `plug_fit_preflight`
// drives the ridged variant below instead.
#[allow(dead_code)]
pub fn cast_synthetic(
    caller: &str,
    scan: IndexedMesh,
    inset_m: f64,
    cell_size_m: f64,
    parts: &PartSelection,
) -> CastOutcome {
    cast_synthetic_with_ridges(
        caller,
        scan,
        inset_m,
        cell_size_m,
        parts,
        &RidgeOptions::default(),
    )
}

/// As [`cast_synthetic`], with the surface texture spelled out.
///
/// ⚠ Ridges are not cosmetic to a cast gate. Enabling them routes layer 0's
/// plug through the canal path, which composes displacement onto the plug AND
/// overrides its mesh cell size (`plug_mesh_cell_size_m`, 0.5 mm by default)
/// regardless of the cell size asked for. Measured 2026-09-08 on
/// `~/scans/base_mold`: ridges ON refuse a 5 mm inset that ridges OFF cast.
pub fn cast_synthetic_with_ridges(
    caller: &str,
    scan: IndexedMesh,
    inset_m: f64,
    cell_size_m: f64,
    parts: &PartSelection,
    ridges: &RidgeOptions,
) -> CastOutcome {
    cast_synthetic_with_layers(
        caller,
        scan,
        inset_m,
        cell_size_m,
        parts,
        ridges,
        &[PROBE_LAYER_THICKNESS_M],
    )
}

/// As [`cast_synthetic_with_ridges`], with the layer stack spelled out.
///
/// ⚠ The stack is not inert geometry. Every thickness feeds
/// `sdf_bounds_padding_m`, the SDF box is the scan AABB expanded by it, and the
/// mesher anchors its lattice at `bounds.min` and steps by exactly one cell —
/// so a different stack TRANSLATES the sampling grid under the plug. That is
/// the whole reason `the_invented_layer_stack_does_not_change_the_verdict`
/// exists, and why the oracle gate above passes the engine's own
/// [`PROBE_LAYER_THICKNESS_M`] rather than a copy of the number.
#[allow(dead_code)]
pub fn cast_synthetic_with_layers(
    caller: &str,
    scan: IndexedMesh,
    inset_m: f64,
    cell_size_m: f64,
    parts: &PartSelection,
    ridges: &RidgeOptions,
    layers_m: &[f64],
) -> CastOutcome {
    let label = format!("{caller}-{}", (inset_m * 1e4).round() as i64);
    let dir = std::env::temp_dir().join(format!("cf-studio-engine-{label}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();

    let mut session = EditSession::from_mesh(PathBuf::from("synthetic.stl"), scan);
    let detected = session.detect_caps();
    assert_eq!(
        detected.loop_count, 2,
        "an open cone has two boundary loops"
    );
    session
        .save(&dir, "synthetic", "mm", 0)
        .expect("prep saves");

    let draft = DesignDraft {
        cavity_inset_m: inset_m,
        layers: layers_m
            .iter()
            .map(|thickness_m| LayerDraft {
                thickness_m: *thickness_m,
                material_key: "ECOFLEX_00_30".to_string(),
                slacker_fraction: 0.25,
            })
            .collect(),
    };
    let cast = generate_molds_for_design(
        &PrepInput {
            cleaned_stl: dir.join("synthetic.cleaned.stl"),
            prep_toml: dir.join("synthetic.prep.toml"),
        },
        &draft,
        cell_size_m,
        ridges,
        parts,
        CastMode::Detachable,
        Some(Path::new("out")),
    );
    let out = match cast {
        Ok(out) => out,
        Err(e) => {
            let _ = std::fs::remove_dir_all(&dir);
            return CastOutcome::Refused(e.to_string());
        }
    };

    // Read the STLs while the fixture still exists — a returned `PathBuf`
    // outlives the file it names.
    let mut loaded = Vec::new();
    for path in out.mold_stls.iter().chain(&out.plug_stls) {
        let mut mesh = load_stl(path).unwrap();
        weld_vertices(&mut mesh, 1e-6);
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        loaded.push((name, mesh));
    }
    let _ = std::fs::remove_dir_all(&dir);
    CastOutcome::Cast(loaded)
}
