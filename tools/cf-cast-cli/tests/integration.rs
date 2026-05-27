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
wall_thickness_m = 0.005
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
    // Slice 9.6c — inline-layers path defaults cavity_inset_m to 0.0,
    // so the Press-Fit Reservation section must NOT appear.
    assert!(
        !proc_text.contains("## Press-Fit Reservation"),
        "inline-layers procedure must NOT contain Press-Fit section (inset = 0); got:\n{proc_text}",
    );
    // Pour mass should be positive + under default budget.
    let mass_kg = report.v2.layers[0].pour_volume.pour_mass_kg;
    assert!(mass_kg > 0.0, "pour mass must be positive: {mass_kg}");
    assert!(
        mass_kg < cf_cast::DEFAULT_MASS_BUDGET_KG,
        "pour mass {mass_kg} exceeds default budget"
    );
}

/// Slice 9 — same end-to-end pipeline but with the layer stack
/// derived from a `<scan>.design.toml` produced by cf-device-design
/// (here hand-written to exercise the schema). The cast.toml omits
/// its own `[[layers]]` and instead points at the design TOML; the
/// cf-cast-cli pipeline must lift the design's layer into the cast,
/// then produce identical output to the inline-layers variant above.
#[test]
fn end_to_end_design_sourced_layers_produce_same_mold_artifacts() {
    let tmp = tempfile::tempdir().unwrap();
    let scan_stl = tmp.path().join("scan.cleaned.stl");
    let prep_toml = tmp.path().join("scan.cleaned.prep.toml");
    let design_toml = tmp.path().join("scan.design.toml");
    let cast_toml = tmp.path().join("cast.toml");

    let mesh = cube_mesh_m();
    save_stl(&mesh, &scan_stl, true).unwrap();

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

    // cf-device-design `.design.toml` shape — hand-written to mirror
    // what cf-device-design::design_toml::save_design_toml emits.
    // Single ECOFLEX_00_30 layer at 6 mm, matching the inline-layers
    // case above so the mold output is comparable.
    std::fs::write(
        &design_toml,
        r#"
[device_design]
tool_version = "1.0.0"
generated_at = "2026-05-15T22:34:00Z"
schema_version = 1

[scan_ref]
cleaned_stl = "scan.cleaned.stl"

[cavity]
inset_m = 0.003
visible = true

[[layers]]
thickness_m = 0.006
material_anchor_key = "ECOFLEX_00_30"
slacker_fraction = 0.0
visible = true
"#,
    )
    .unwrap();

    // cast.toml with `[design]` block + NO `[[layers]]` — the lift
    // populates them at run time.
    std::fs::write(
        &cast_toml,
        r#"
[scan]
cleaned_stl = "scan.cleaned.stl"
prep_toml = "scan.cleaned.prep.toml"

[design]
path = "scan.design.toml"

[cast]
mesh_cell_size_m = 0.005
wall_thickness_m = 0.005
piece_min_wall_mm = 0.1
split_normal = [0.0, 0.0, -1.0]
output_dir = "out"

[plug_pins]
enabled = false

[pour_gate]
enabled = false

"#,
    )
    .unwrap();

    let report = cf_cast_cli::run(&cast_toml, None).expect("cf-cast-cli design-sourced run");
    assert_eq!(report.layer_count, 1);
    assert_eq!(report.v2.layers.len(), 1);
    let out_dir: PathBuf = tmp.path().join("out");
    for p in [
        out_dir.join("mold_layer_0_piece_0.stl"),
        out_dir.join("mold_layer_0_piece_1.stl"),
        out_dir.join("plug_layer_0.stl"),
        out_dir.join("procedure.md"),
    ] {
        assert!(p.exists(), "expected output {} to exist", p.display());
    }
    // Procedure should still surface the design-lifted material.
    let proc_text = std::fs::read_to_string(out_dir.join("procedure.md")).unwrap();
    assert!(
        proc_text.contains("Ecoflex 00-30"),
        "procedure must reference the design-lifted material; got:\n{proc_text}"
    );
}

/// Slice 9 — cast.toml with BOTH `[design]` and non-empty
/// `[[layers]]` is rejected at validation (two sources of truth).
/// Hand-builds a minimal CastConfig to exercise the gate without
/// going through the full pipeline.
#[test]
fn design_plus_inline_layers_rejected() {
    let toml_text = r#"
[scan]
cleaned_stl = "scan.cleaned.stl"
prep_toml = "scan.cleaned.prep.toml"

[design]
path = "scan.design.toml"

[[layers]]
thickness_m = 0.006
material = "ECOFLEX_00_30"
"#;
    let cfg = cf_cast_cli::CastConfig::from_toml_str(toml_text).unwrap();
    let err = cfg.validate_layer_source().unwrap_err();
    let msg = err.to_string();
    assert!(
        msg.contains("`[design]` is set"),
        "expected layer-source error, got: {msg}"
    );
}

/// Slice 9.6 — design-sourced run with a non-zero `cavity.inset_m`
/// shifts plug + every layer outer surface inward, so the cured
/// part's inner cavity is `inset_m` smaller than the scan. Exercised
/// here on the same 30 mm cube fixture as the prior tests with a
/// 2 mm inset; the mold pieces + plug + procedure.md must still be
/// produced (the cube minus 2 mm inset minus the 0.1 mm seam wall is
/// well above the min-wall gate), and the procedure's material name
/// reference is unchanged.
#[test]
fn end_to_end_design_sourced_with_cavity_inset_writes_artifacts() {
    let tmp = tempfile::tempdir().unwrap();
    let scan_stl = tmp.path().join("scan.cleaned.stl");
    let prep_toml = tmp.path().join("scan.cleaned.prep.toml");
    let design_toml = tmp.path().join("scan.design.toml");
    let cast_toml = tmp.path().join("cast.toml");

    let mesh = cube_mesh_m();
    save_stl(&mesh, &scan_stl, true).unwrap();

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

    // design.toml with cavity.inset_m = 0.002 (2 mm press-fit
    // reservation). Layer-0 silicone thickness 6 mm — outer surface
    // ends up at +4 mm vs scan surface (thickness − inset).
    std::fs::write(
        &design_toml,
        r#"
[device_design]
tool_version = "1.0.0"
generated_at = "2026-05-15T22:34:00Z"
schema_version = 1

[scan_ref]
cleaned_stl = "scan.cleaned.stl"

[cavity]
inset_m = 0.002
visible = true

[[layers]]
thickness_m = 0.006
material_anchor_key = "ECOFLEX_00_30"
slacker_fraction = 0.0
visible = true
"#,
    )
    .unwrap();

    std::fs::write(
        &cast_toml,
        r#"
[scan]
cleaned_stl = "scan.cleaned.stl"
prep_toml = "scan.cleaned.prep.toml"

[design]
path = "scan.design.toml"

[cast]
mesh_cell_size_m = 0.005
wall_thickness_m = 0.005
piece_min_wall_mm = 0.1
split_normal = [0.0, 0.0, -1.0]
output_dir = "out"

[plug_pins]
enabled = false

[pour_gate]
enabled = false

"#,
    )
    .unwrap();

    let report =
        cf_cast_cli::run(&cast_toml, None).expect("cf-cast-cli design-sourced + inset run");
    assert_eq!(report.layer_count, 1);
    let out_dir: PathBuf = tmp.path().join("out");
    for p in [
        out_dir.join("mold_layer_0_piece_0.stl"),
        out_dir.join("mold_layer_0_piece_1.stl"),
        out_dir.join("plug_layer_0.stl"),
        out_dir.join("procedure.md"),
    ] {
        assert!(p.exists(), "expected output {} to exist", p.display());
    }
    // Slice 9.6c — procedure.md must surface the press-fit
    // reservation when cavity.inset_m > 0.
    let proc_text = std::fs::read_to_string(out_dir.join("procedure.md")).unwrap();
    assert!(
        proc_text.contains("## Press-Fit Reservation"),
        "procedure.md must contain the Press-Fit Reservation section when inset > 0; got:\n{proc_text}",
    );
    assert!(
        proc_text.contains("**2.00 mm**"),
        "procedure.md must surface the inset value (2 mm) with 2-dp mm formatting; got:\n{proc_text}",
    );
    // Slice 9.5 — design.toml had slacker_fraction = 0.0 for the
    // single layer, so the Slacker Recipe section MUST NOT appear.
    assert!(
        !proc_text.contains("## Slacker Recipe"),
        "slacker_fraction = 0 must omit the Slacker Recipe section; got:\n{proc_text}",
    );
}

/// Slice 9.5 — design-sourced run with a non-zero `slacker_fraction`
/// produces a `## Slacker Recipe` section in procedure.md alongside
/// the existing Materials Summary table. Single-layer 6 mm Ecoflex
/// 00-30 with 15 % Slacker on a 30 mm cube fixture.
#[test]
fn end_to_end_design_sourced_with_slacker_fraction_surfaces_recipe_in_procedure() {
    let tmp = tempfile::tempdir().unwrap();
    let scan_stl = tmp.path().join("scan.cleaned.stl");
    let prep_toml = tmp.path().join("scan.cleaned.prep.toml");
    let design_toml = tmp.path().join("scan.design.toml");
    let cast_toml = tmp.path().join("cast.toml");

    let mesh = cube_mesh_m();
    save_stl(&mesh, &scan_stl, true).unwrap();

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

    std::fs::write(
        &design_toml,
        r#"
[device_design]
tool_version = "1.0.0"
generated_at = "2026-05-15T22:34:00Z"
schema_version = 1

[scan_ref]
cleaned_stl = "scan.cleaned.stl"

[cavity]
inset_m = 0.0
visible = true

[[layers]]
thickness_m = 0.006
material_anchor_key = "ECOFLEX_00_30"
slacker_fraction = 0.15
visible = true
"#,
    )
    .unwrap();

    std::fs::write(
        &cast_toml,
        r#"
[scan]
cleaned_stl = "scan.cleaned.stl"
prep_toml = "scan.cleaned.prep.toml"

[design]
path = "scan.design.toml"

[cast]
mesh_cell_size_m = 0.005
wall_thickness_m = 0.005
piece_min_wall_mm = 0.1
split_normal = [0.0, 0.0, -1.0]
output_dir = "out"

[plug_pins]
enabled = false

[pour_gate]
enabled = false

"#,
    )
    .unwrap();

    let report = cf_cast_cli::run(&cast_toml, None).expect("cf-cast-cli slacker run");
    let proc_text = std::fs::read_to_string(report.procedure_path).unwrap();
    assert!(
        proc_text.contains("## Slacker Recipe"),
        "procedure.md must contain the Slacker Recipe section when slacker > 0; got:\n{proc_text}",
    );
    assert!(
        proc_text.contains("15.0%"),
        "procedure.md must surface the slacker fraction (15%); got:\n{proc_text}",
    );
    // No inset → no Press-Fit section.
    assert!(
        !proc_text.contains("## Press-Fit Reservation"),
        "inset = 0 must omit Press-Fit section; got:\n{proc_text}",
    );
}

/// Compute the signed volume of a cf-cast-exported STL via the
/// divergence theorem (`Σ a · (b × c) / 6`). Mirrors cf-device-design's
/// `signed_volume_m3` (tools/cf-device-design/src/main.rs:774);
/// reproduced here to keep cf-cast-cli's test crate independent of
/// cf-device-design's egui-heavy dep tree. cf-cast exports STLs in
/// the mm frame (`solid_to_mm_mesh` scales m → mm before save), so
/// the raw sum is in mm³ — divide by 1000 to land in cm³.
/// `.abs()` matches cf-device-design's posture against winding-sign
/// noise on near-closed meshes.
fn mesh_volume_cm3(path: &std::path::Path) -> f64 {
    let mesh = mesh_io::load_stl(path).expect("load STL");
    let mut six_volume_mm3 = 0.0_f64;
    for face in &mesh.faces {
        let a = mesh.vertices[face[0] as usize].coords;
        let b = mesh.vertices[face[1] as usize].coords;
        let c = mesh.vertices[face[2] as usize].coords;
        six_volume_mm3 += a.dot(&b.cross(&c));
    }
    (six_volume_mm3 / 6.0).abs() / 1000.0
}

/// B1 mold-wall regression — after the Option A swap
/// (`outermost.offset(wall_thickness_m)` replaces the cuboid
/// bounding region; see `docs/CF_CAST_MOLD_WALL_RECON.md`), each
/// mold piece must be a CONTOUR-following rind, not a brick. On
/// the 30 mm cube fixture with a single 6 mm layer + 5 mm wall:
///
/// - outer surface ≈ cube at half-side 0.021 m → surface area
///   ≈ `6 × 0.042² = 0.010584 m² = 105.84 cm²`
/// - target shell volume ≈ `surface × wall = 105.84 × 0.5 = 52.92 cm³`
///
/// The piece-volume bound is `2 × shell_target_cm³` per piece — loose
/// enough to absorb MC stair-step at 5 mm cells, tight enough that a
/// regression re-introducing the cuboid (which would jump per-piece
/// volume ~10× per the iter-1 sock recon table at §3) would trip.
#[test]
fn cube_fixture_mold_pieces_follow_contour_not_cuboid() {
    let tmp = tempfile::tempdir().unwrap();
    let scan_stl = tmp.path().join("scan.cleaned.stl");
    let prep_toml = tmp.path().join("scan.cleaned.prep.toml");
    let cast_toml = tmp.path().join("cast.toml");

    let mesh = cube_mesh_m();
    save_stl(&mesh, &scan_stl, true).unwrap();

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

    std::fs::write(
        &cast_toml,
        r#"
[scan]
cleaned_stl = "scan.cleaned.stl"
prep_toml = "scan.cleaned.prep.toml"

[cast]
mesh_cell_size_m = 0.005
wall_thickness_m = 0.005
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

"#,
    )
    .unwrap();

    cf_cast_cli::run(&cast_toml, None).expect("cf-cast-cli end-to-end run");
    let out_dir: PathBuf = tmp.path().join("out");

    // Compute the analytical shell-target per the docstring above:
    // cube half-side 15 mm + 6 mm layer = 21 mm; surface area
    // ≈ 6 × (2 × 21 mm)² = 1.0584e-2 m² = 105.84 cm². Shell-target
    // volume = surface × wall_thickness = 105.84 cm² × 0.5 cm
    // = 52.92 cm³.
    let outer_half_cm: f64 = 1.5 + 0.6;
    let surface_area_cm2 = 6.0 * (2.0 * outer_half_cm).powi(2);
    let shell_target_cm3 = surface_area_cm2 * 0.5;
    let per_piece_bound_cm3 = 2.0 * shell_target_cm3;

    for piece in ["mold_layer_0_piece_0.stl", "mold_layer_0_piece_1.stl"] {
        let p = out_dir.join(piece);
        let v_cm3 = mesh_volume_cm3(&p);
        assert!(
            v_cm3 < per_piece_bound_cm3,
            "{piece} volume {v_cm3:.2} cm³ exceeds 2 × shell_target \
             {per_piece_bound_cm3:.2} cm³ — contour-following rind \
             regressed to cuboid envelope?",
        );
        // Sanity floor: pieces must have non-trivial volume (catch
        // a degenerate-empty-piece regression separately from the
        // cuboid-shaped-regression upper bound).
        assert!(
            v_cm3 > 0.1,
            "{piece} volume {v_cm3} cm³ is degenerately small"
        );
    }
}

/// Slice 9 — cast.toml with neither `[design]` nor `[[layers]]` is
/// rejected at validation (no layer source at all).
#[test]
fn neither_design_nor_layers_rejected() {
    let toml_text = r#"
[scan]
cleaned_stl = "scan.cleaned.stl"
prep_toml = "scan.cleaned.prep.toml"
"#;
    let cfg = cf_cast_cli::CastConfig::from_toml_str(toml_text).unwrap();
    let err = cfg.validate_layer_source().unwrap_err();
    let msg = err.to_string();
    assert!(
        msg.contains("neither layer source is set"),
        "expected layer-source error, got: {msg}"
    );
}
