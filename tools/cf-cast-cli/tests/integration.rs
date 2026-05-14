//! End-to-end integration test exercising the full scan→cast bridge
//! pipeline on a synthetic 30 mm cube fixture.
//!
//! Runs in `--release` in ~2-5 seconds at 5 mm cell size and produces
//! a 1-layer cast (2 piece STLs + 1 plug STL + procedure.md = 4
//! files). The cube + tight bounding margin are sized to keep the
//! per-piece F4 gate happy without tripping ExceedsBuildVolume.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::path::PathBuf;

use mesh_io::save_stl;
use nalgebra::Point3;

/// Build a 30 mm cube IndexedMesh in METERS (scan-frame convention).
fn cube_mesh_m() -> cf_geometry::IndexedMesh {
    let mut m = cf_geometry::IndexedMesh::new();
    let h = 0.015; // 30 mm cube half-extent (in meters)
    let coords = [
        (-h, -h, -h),
        (h, -h, -h),
        (h, h, -h),
        (-h, h, -h),
        (-h, -h, h),
        (h, -h, h),
        (h, h, h),
        (-h, h, h),
    ];
    for (x, y, z) in coords {
        m.vertices.push(Point3::new(x, y, z));
    }
    for f in [
        // -z
        [0, 2, 1],
        [0, 3, 2],
        // +z
        [4, 5, 6],
        [4, 6, 7],
        // -y
        [0, 1, 5],
        [0, 5, 4],
        // +y
        [2, 3, 7],
        [2, 7, 6],
        // -x
        [0, 4, 7],
        [0, 7, 3],
        // +x
        [1, 2, 6],
        [1, 6, 5],
    ] {
        m.faces.push(f);
    }
    m
}

#[test]
fn end_to_end_single_layer_synthetic_cube_writes_three_stls_plus_procedure() {
    let tmp = tempfile::tempdir().unwrap();
    let scan_stl = tmp.path().join("scan.cleaned.stl");
    let prep_toml = tmp.path().join("scan.cleaned.prep.toml");
    let cast_toml = tmp.path().join("cast.toml");

    // The whole bridge pipeline works in meters — cf-scan-prep emits
    // cleaned STLs in meters, and `save_stl` / `load_stl` round-trip
    // coordinate values verbatim (no unit conversion; the `true` arg
    // is the binary-format flag). So the fixture is built in meters
    // and the load round-trip reproduces meter-scale values straight
    // into the SDF.
    let mesh = cube_mesh_m();
    save_stl(&mesh, &scan_stl, true).unwrap();

    // Minimal .prep.toml with a centerline along +X across the cube.
    // Three points, ~22 mm total arc length (just under the cube
    // size so the centerline stays inside the body).
    std::fs::write(
        &prep_toml,
        r#"
[scan_prep]
source_stl = "raw.stl"
tool_version = "0.0.0-test"
generated_at = "2026-05-13T00:00:00Z"
stl_units_at_load = "m"

[centerline]
points_m = [
    [-0.011, 0.0, 0.0],
    [0.0, 0.0, 0.0],
    [0.011, 0.0, 0.0],
]
algorithm = "cross_section_centroids"
"#,
    )
    .unwrap();

    // 1-layer cast at coarse 5 mm cells — F4 piece-min-wall-mm
    // lowered to 0.1 to tolerate seam stair-stepping at this
    // coarse cell size.
    std::fs::write(
        &cast_toml,
        r#"
[scan]
cleaned_stl = "scan.cleaned.stl"
prep_toml = "scan.cleaned.prep.toml"

[cast]
mesh_cell_size_m = 0.005
bounding_margin_m = 0.015
piece_min_wall_mm = 0.1
split_normal = [0.0, 0.0, -1.0]
output_dir = "out"

[[layers]]
thickness_m = 0.006
material = "ECOFLEX_00_30"

[plug_pins]
enabled = false

[pour_gate]
enabled = false

[registration_pins]
enabled = false
"#,
    )
    .unwrap();

    let report = cf_cast_cli::run(&cast_toml, None).expect("cf-cast-cli end-to-end run");
    assert_eq!(report.layer_count, 1);
    assert_eq!(report.v2.layers.len(), 1);
    // 1 layer → 2 piece STLs + 1 plug STL + procedure.md = 4 files.
    let out_dir: PathBuf = tmp.path().join("out");
    let piece_0 = out_dir.join("mold_layer_0_piece_0.stl");
    let piece_1 = out_dir.join("mold_layer_0_piece_1.stl");
    let plug = out_dir.join("plug_layer_0.stl");
    let procedure = out_dir.join("procedure.md");
    for p in [&piece_0, &piece_1, &plug, &procedure] {
        assert!(p.exists(), "expected output {} to exist", p.display());
    }
    // Procedure should mention the chosen material.
    let proc_text = std::fs::read_to_string(&procedure).unwrap();
    assert!(
        proc_text.contains("Ecoflex 00-30"),
        "procedure must reference the layer material; got:\n{proc_text}"
    );
    // Pour mass should be positive + under default budget.
    let mass_kg = report.v2.layers[0].pour_volume.pour_mass_kg;
    assert!(mass_kg > 0.0, "pour mass must be positive: {mass_kg}");
    assert!(
        mass_kg < cf_cast::DEFAULT_MASS_BUDGET_KG,
        "pour mass {mass_kg} exceeds default budget"
    );
}
