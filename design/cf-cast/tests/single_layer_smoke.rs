//! Stage 2 single-layer integration smoke: build a one-entry
//! `CastSpec::layers` against a known-simple body + capsule plug +
//! cuboid bounding region, run `export_molds`, and verify one mold +
//! one plug STL land on disk in `CARGO_TARGET_TMPDIR`.
//!
//! The output path is intentionally stable across runs so the user can
//! re-open the STLs in a slicer / mesh viewer for the eyes-on-pixels
//! pass without re-running the test.

// `expect()` and explicit `panic!` are the cf-cast integration-test
// idiom for surfacing failure context. Workspace policy warns on
// `expect_used` / `panic` for library code (deny in lib.rs); test
// code makes failures readable by escalating to a clear panic
// message. Same convention as `sim/L0/soft/tests/*.rs`.
#![allow(clippy::expect_used, clippy::panic)]

use std::path::PathBuf;

use cf_cast::{CastLayer, CastSpec, DEFAULT_MASS_BUDGET_KG, MoldingMaterial};
use cf_design::Solid;
use mesh_printability::PrinterConfig;
use nalgebra::Vector3;

/// Build the Stage 2 reference geometry — a one-entry `layers`
/// vector wrapping the Stage 1 cuboid body + capsule plug +
/// cuboid bounding region. Kept inline (rather than pulling from
/// `cf-cast`'s private `tests` module) so the integration boundary
/// is honest.
///
/// A cuboid body (not a sphere) keeps the one-piece mold cup
/// topologically open at the top: clipping above `body.z_max` carves
/// the cuboid wall above the body and exposes the body's
/// xy-cross-section as a real rectangular hole in the cup. A sphere
/// would close at a single tangent point, failing the F4 trapped-
/// volume detector. Multi-layer production fixtures use the same
/// open-top construction with the layered-silicone-device's cavity-
/// fit body in place of this smoke cuboid.
fn reference_spec() -> CastSpec {
    let layer_body = Solid::cuboid(Vector3::new(0.025, 0.025, 0.020));
    let plug = Solid::capsule(0.008, 0.020).translate(Vector3::new(0.0, 0.0, 0.040));
    let bounding_region = Solid::cuboid(Vector3::new(0.040, 0.040, 0.030));
    CastSpec {
        layers: vec![CastLayer {
            body: layer_body,
            material: MoldingMaterial {
                display_name: "Ecoflex 00-30".to_string(),
                density_kg_m3: 1070.0,
                anchor_key: Some("ECOFLEX_00_30"),
            },
        }],
        plug,
        bounding_region,
        wall_thickness_m: 0.005,
        mesh_cell_size_m: 0.002,
        printer_config: PrinterConfig::fdm_default(),
        mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        scan_mesh_for_plug_layer_0: None,
        plug_layer_0_mesh_cell_size_m: None,
        plug_layer_0_field_skin_m: None,
    }
}

#[test]
fn stage_2_single_layer_export_writes_one_mold_and_one_plug() {
    let out_dir: PathBuf = [env!("CARGO_TARGET_TMPDIR"), "cf-cast-stage-2"]
        .iter()
        .collect();
    // Best-effort clean — `remove_dir_all` returns `NotFound` on the
    // first run, which is fine. Anything else (permission, busy) is
    // surfaced for diagnosis rather than silently ignored.
    match std::fs::remove_dir_all(&out_dir) {
        Ok(()) => {}
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => panic!("failed to clean {out_dir:?}: {e}"),
    }

    let spec = reference_spec();
    let report = spec
        .export_molds(&out_dir)
        .expect("Stage 2 single-layer export should succeed on reference geometry");

    assert_eq!(report.molds.len(), 1, "single-layer spec → single mold");
    let mold = &report.molds[0];
    assert_eq!(mold.layer_index, 0);
    assert_eq!(mold.material_display_name, "Ecoflex 00-30");

    // Files exist, non-empty. Binary-STL size formula:
    // 80-byte header + 4-byte face count + 50 bytes per triangle.
    let mold_meta = std::fs::metadata(&mold.path).expect("mold_layer_0.stl should exist");
    let plug_meta = std::fs::metadata(&report.plug_path).expect("plug.stl should exist");
    let expected_mold_size = 84 + (mold.summary.face_count as u64) * 50;
    let expected_plug_size = 84 + (report.plug_summary.face_count as u64) * 50;
    assert_eq!(mold_meta.len(), expected_mold_size, "mold STL byte size");
    assert_eq!(plug_meta.len(), expected_plug_size, "plug.stl byte size");

    // F4 gate: zero blocking Critical issues guaranteed by
    // export_molds contract; pin it. (Non-blocking Criticals — mold
    // overhangs and MC self-intersection noise — are tolerated and
    // surface in `validation.issues` with their original Critical
    // severity for caller inspection.)
    assert!(!mold.summary.aabb_mm.is_empty(), "mold AABB populated");
    assert!(
        !report.plug_summary.aabb_mm.is_empty(),
        "plug AABB populated"
    );

    // Unit-conversion sanity at the integration boundary: the plug is
    // a capsule with radius 8 mm + half-height 20 mm, translated to
    // (0, 0, 40 mm). Its z-extent in mm should span roughly [12, 68]
    // (40 ± (20 + 8)). Pins that meters → mm scale is applied exactly
    // once (a missed scale would put it at [0.012, 0.068]; a doubled
    // scale at [12000, 68000]).
    let plug_z_min = report.plug_summary.aabb_mm.min.z;
    let plug_z_max = report.plug_summary.aabb_mm.max.z;
    assert!(
        (8.0..16.0).contains(&plug_z_min) && (64.0..72.0).contains(&plug_z_max),
        "plug z-range should be near [12, 68] mm: [{plug_z_min}, {plug_z_max}]"
    );

    // Mold cup z-extent: bounding region half-extent 30 mm carved
    // above body z_max (+20 mm) by the clip. mm-frame mold should
    // span roughly [-30, +20] mm in z.
    let mold_z_min = mold.summary.aabb_mm.min.z;
    let mold_z_max = mold.summary.aabb_mm.max.z;
    assert!(
        (-32.0..-28.0).contains(&mold_z_min) && (18.0..22.0).contains(&mold_z_max),
        "mold z-range should be near [-30, +20] mm: [{mold_z_min}, {mold_z_max}]"
    );

    // Make the output paths easy to find post-run.
    println!("mold STL: {}", mold.path.display());
    println!("plug STL: {}", report.plug_path.display());
    println!(
        "mold: {} verts, {} faces, bounds {:?}",
        mold.summary.vertex_count, mold.summary.face_count, mold.summary.aabb_mm
    );
    println!(
        "plug: {} verts, {} faces, bounds {:?}",
        report.plug_summary.vertex_count,
        report.plug_summary.face_count,
        report.plug_summary.aabb_mm
    );
}
