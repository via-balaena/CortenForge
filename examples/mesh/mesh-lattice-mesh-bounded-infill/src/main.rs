// `unreachable!()` calls in this binary are diagnostic guards on
// `let-else` branches that cannot fire (validated `for_fdm` preset +
// 50 mm cube fixture is well within `2 × shell_thickness +
// cell_size`, so post-validation `generate_infill` cannot return
// `Err`). `xtask grade`'s Safety criterion counts un-justified
// `unreachable!()`; allow at file level since every call site is a
// post-validation `Result::Err` impossibility, not a real panic.
#![allow(clippy::unreachable)]
//! mesh-lattice-mesh-bounded-infill — FDM-style shell + lattice
//! composite via `generate_infill` on a hand-authored 50 mm
//! watertight cube.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.9. Mesh-bounded
//! counterpart to §5.8 `mesh-lattice-shape-bounded` (analytical-SDF
//! trim); both ship in v1.0.
//!
//! Demonstrates the canonical FDM-infill workflow on a watertight
//! input mesh: outer shell from inward offset, lattice in the
//! mesh-bounded interior, optional bridging connections from lattice
//! nodes to the inner shell, optional solid caps near the bbox top
//! and bottom. All five F6 gap-fixes (`§6.2 #24-#28`) are landed; the
//! anchors below lock the post-fix numerical contract on the
//! `for_fdm + cell_size 10` configuration over the 50 mm cube.
//!
//! Locked anchors (default config, `solid_caps = true`):
//!
//! - mesh: 226 662 verts / 78 308 tris.
//! - shell: 224 672 verts / 74 900 tris (real inward-offset hollow
//!   shell, gap a post-fix).
//! - lattice: 1 456 verts / 2 496 tris (104 cubic struts × 14/24 per
//!   cylinder).
//! - caps: 16 verts / 24 tris (2 boxes × 8/12; gap c post-fix).
//! - connections: 37 struts × 14/24 = 518 verts / 888 tris (gap b
//!   post-fix; bridging struts from lattice nodes to inner shell).
//! - `shell_volume` = 17 195.179 mm³ (signed-volume integral; gap d).
//! - `lattice_volume` = 28.821 mm³ (signed-volume integral; gap d).
//! - `interior_volume` = 53 157.376 mm³ = 37.6³ (mesh-SDF
//!   intersection on offset shell; gap e — bit-equal to AABB inset
//!   on the cube, structurally different on non-cube fixtures).
//! - `actual_density` = 0.010 402.
//!
//! Comparison anchor: a second `generate_infill` run with
//! `solid_caps = false` differentiates the gap-c carving — the wide
//! top region (above `SIDE − inset − cell_size = 33.8 mm`) holds 0
//! lattice verts WITH caps and 448 verts WITHOUT.

use std::path::PathBuf;

use anyhow::{Result, ensure};
use example_mesh_mesh_lattice_mesh_bounded_infill::{
    SIDE, cube_50mm, cube_at_origin, verify_fixture,
};
use mesh_io::save_ply;
use mesh_lattice::{InfillParams, InfillResult, LatticeError, generate_infill};
use mesh_types::IndexedMesh;

// =============================================================================
// Run-config constants
// =============================================================================

/// Cell size override in mm — the `for_fdm` preset's default is `5.0`
/// (`infill.rs:102` — `LatticeParams::cubic(5.0)`), yielding ~514
/// cells in the post-shell interior of the 50 mm cube. Overriding to
/// `10.0` via `.with_cell_size(10.0)` yields ~54 cells — coarser,
/// more visually readable in cross-section, and the canonical
/// pedagogical config for this example.
const CELL_SIZE_OVERRIDE: f64 = 10.0;

/// Edge length in mm of the tiny cube fed to the `InteriorTooSmall`
/// error-path anchor.
const TINY_CUBE_SIDE: f64 = 5.0;

/// Cell size in mm for the `InteriorTooSmall` error-path anchor —
/// `inset = 4 × 0.5 + 1.2 = 3.2` mm, so `5 − 2 × 3.2 = −1.4` mm is
/// negative, triggering `infill.rs:355`.
const TINY_CELL_SIZE: f64 = 4.0;

// =============================================================================
// Geometry constants — derived for the for_fdm + cell_size 10 config
// =============================================================================

/// AABB inset in mm: `cell_size × 0.5 + shell_thickness = 10 × 0.5 +
/// 1.2 = 6.2`. The lattice iteration bbox starts at this distance
/// from each face of the input AABB.
const INSET: f64 = CELL_SIZE_OVERRIDE.mul_add(0.5, 1.2);

/// Top-wide region z-threshold in mm: `SIDE − INSET − CELL_SIZE_OVERRIDE
/// = 33.8`. Pre-fix, the lattice extended to the top of the AABB-
/// bounded interior `[INSET, SIDE − INSET] = [6.2, 43.8]`; cylinder
/// verts at the iz=3 strut row (z ≈ 36.2) lived above this threshold
/// (448 of them on the cube fixture). Post-gap-c-fix, the carved cap
/// band `[39.8, 43.8]` shrinks the lattice domain to `[10.2, 39.8]`,
/// so the highest strut row drops to z ≈ 30.2 and the wide region
/// holds zero lattice verts. The same threshold counts 448 verts in
/// the no-caps comparison run, demonstrating gap c is doing real
/// work.
const TOP_WIDE_Z: f64 = SIDE - INSET - CELL_SIZE_OVERRIDE;

// =============================================================================
// Locked numerical anchors — post-fix, for_fdm + cell_size 10,
// solid_caps = true (default)
// =============================================================================

/// `mesh.vertex_count()` = `shell.vc` (224 672) + `lattice.vc`
/// (1 456) + `cap_verts` (16 = 2 × 8) + `connection_verts` (518 =
/// 37 × 14).
const EXPECTED_MESH_VC: usize = 226_662;

/// `mesh.face_count()` = `shell.fc` (74 900) + `lattice.fc` (2 496)
/// + `cap_tris` (24 = 2 × 12) + `connection_tris` (888 = 37 × 24).
const EXPECTED_MESH_TC: usize = 78_308;

/// `result.shell.vertex_count()` — real inward-offset hollow shell
/// (gap a post-fix); decisively NOT equal to `fixture.vertex_count() = 8`.
const EXPECTED_SHELL_VC: usize = 224_672;

/// `result.shell.face_count()`.
const EXPECTED_SHELL_FC: usize = 74_900;

/// `result.lattice.vertex_count()` — 104 cubic struts × 14 verts per
/// 6-segment cylinder.
const EXPECTED_LATTICE_VC: usize = 1_456;

/// `result.lattice.face_count()` — 104 cubic struts × 24 tris per
/// cylinder.
const EXPECTED_LATTICE_FC: usize = 2_496;

/// Cap geometry per gap-c fix: two extruded boxes (`combine_meshes`
/// of two cube sub-meshes) at top + bottom of the inset interior, 8
/// verts and 12 tris each.
const EXPECTED_CAP_VERTS: usize = 16;
const EXPECTED_CAP_TRIS: usize = 24;

/// Lattice-to-shell connection count per gap-b fix — each of 37
/// lattice nodes within `2 × cell_size = 20 mm` of the inner shell
/// (filtered to NOT lie within the carved cap bands) emits one
/// 14-vert / 24-tri cylinder bridge to its `closest_point` on the
/// inner shell.
const EXPECTED_CONNECTION_COUNT: usize = 37;

/// Cylinder geometry: `mesh-lattice` strut cylinders generate 6
/// axial segments + 2 cap rings = 14 verts, 24 tris.
const STRUT_VERTS_PER_CYLINDER: usize = 14;
const STRUT_TRIS_PER_CYLINDER: usize = 24;

/// `result.shell_volume` — signed-volume integral on the offset
/// shell (gap d post-fix). Analytical lower bound: `50³ − 47.6³ =
/// 17 149.824` mm³; the `+45.355` mm³ delta is the marching-cubes
/// chamfer on the inner offset (sharp creases bevel under linear
/// interpolation between corners bracketing `shell_sdf == 0`).
const EXPECTED_SHELL_VOLUME: f64 = 17_195.178_666_783_733;

/// `result.lattice_volume` — signed-volume integral on the lattice
/// mesh (gap d post-fix).
const EXPECTED_LATTICE_VOLUME: f64 = 28.821_325_438_050_68;

/// `result.interior_volume` — mesh-SDF intersection on the offset
/// shell (gap e post-fix). Bit-equal to AABB inset on the cube
/// fixture: `(SIDE − 2 × INSET)³ = 37.6³`. The trailing `…974` is
/// f64 representation of `37.6` (= `37.599_999_999_999_998` to 16
/// digits); cubed it lands a few ulps below the mathematical
/// `53 157.376`.
const EXPECTED_INTERIOR_VOLUME: f64 = 53_157.375_999_999_975;

/// `result.actual_density` reported by the cubic-strut volume
/// heuristic at this configuration: thin struts in a 53 cm³
/// interior land far below `params.infill_percentage = 0.2`. The
/// `for_fdm` preset trades visual readability of the strut grid for
/// a literal density match; the anchor locks the achieved value,
/// not the target.
const EXPECTED_ACTUAL_DENSITY: f64 = 0.010_401_688_699_471_42;

/// Tolerance for signed-volume integrals — the mesh-bounded integral
/// pipeline (offset MC + signed-volume sum) is BLAS/LAPACK-free and
/// deterministic across platforms; 1e-9 catches any algorithmic
/// drift.
const VOLUME_TOL: f64 = 1e-9;

/// Tolerance for `actual_density` (passes through f64 divisions in
/// `estimate_strut_volume`).
const DENSITY_TOL: f64 = 1e-9;

/// Tolerance for `total_volume = shell_volume + lattice_volume`
/// arithmetic.
const TOTAL_VOLUME_TOL: f64 = 1e-9;

// =============================================================================
// PLY output paths — per spec §5.9 lines 802-806
// =============================================================================

const INPUT_PLY: &str = "out/input.ply";
const SHELL_PLY: &str = "out/shell.ply";
const LATTICE_PLY: &str = "out/lattice.ply";
const COMPOSITE_PLY: &str = "out/composite.ply";

// =============================================================================
// Entry point
// =============================================================================

fn main() -> Result<()> {
    let fixture = cube_50mm();
    verify_fixture(&fixture)?;

    verify_validate_paths()?;
    verify_error_paths()?;

    let result = run_default(&fixture);
    verify_post_fix_anchors(&fixture, &result)?;

    let result_no_caps = run_no_caps(&fixture);
    verify_comparison_anchors(&result, &result_no_caps)?;

    write_plys(&fixture, &result)?;

    print_summary(&result, &result_no_caps);
    Ok(())
}

// =============================================================================
// validate-path anchors — all 3 presets + the HE-3 negative path
// =============================================================================

fn verify_validate_paths() -> Result<()> {
    ensure!(
        InfillParams::for_fdm().validate().is_ok(),
        "for_fdm().validate() should be Ok",
    );
    ensure!(
        InfillParams::for_lightweight().validate().is_ok(),
        "for_lightweight().validate() should be Ok",
    );
    ensure!(
        InfillParams::for_strong().validate().is_ok(),
        "for_strong().validate() should be Ok",
    );

    // HE-3 negative-validate via direct field assignment — the
    // `with_shell_thickness` builder clamps to `max(0.0)` per
    // `infill.rs:169-171`, so the negative-`shell_thickness` branch
    // through `validate` (returns `InvalidShellThickness` at
    // `infill.rs:205`) is reachable only by direct construction.
    let mut params = InfillParams::for_fdm();
    params.shell_thickness = -1.0;
    let err = params.validate();
    ensure!(
        matches!(
            err,
            Err(LatticeError::InvalidShellThickness(t)) if (t - -1.0).abs() < 1e-12
        ),
        "negative-validate via direct field assignment did not return InvalidShellThickness(-1.0): got {err:?}",
    );

    Ok(())
}

// =============================================================================
// Error-path anchors — EmptyMesh + InteriorTooSmall
// =============================================================================

fn verify_error_paths() -> Result<()> {
    let empty = IndexedMesh::new();
    let params_default = InfillParams::for_fdm().with_cell_size(CELL_SIZE_OVERRIDE);
    let empty_result = generate_infill(&empty, &params_default);
    ensure!(
        matches!(empty_result, Err(LatticeError::EmptyMesh)),
        "generate_infill(empty, params) should be Err(EmptyMesh): got {empty_result:?}",
    );

    let tiny = cube_at_origin(TINY_CUBE_SIDE);
    let tiny_params = InfillParams::for_fdm().with_cell_size(TINY_CELL_SIZE);
    let tiny_result = generate_infill(&tiny, &tiny_params);
    ensure!(
        matches!(tiny_result, Err(LatticeError::InteriorTooSmall)),
        "generate_infill(tiny_5mm, cell_size=4) should be Err(InteriorTooSmall): got {tiny_result:?}",
    );

    Ok(())
}

// =============================================================================
// Default and no-caps generate_infill runs
// =============================================================================

fn run_default(fixture: &IndexedMesh) -> InfillResult {
    let params = InfillParams::for_fdm().with_cell_size(CELL_SIZE_OVERRIDE);
    let Ok(result) = generate_infill(fixture, &params) else {
        unreachable!(
            "validated for_fdm preset + cell_size {CELL_SIZE_OVERRIDE} on a 50 mm cube cannot fail \
             (interior side = 50 - 2 × (5 + 1.2) = 37.6 mm > 0)"
        );
    };
    result
}

fn run_no_caps(fixture: &IndexedMesh) -> InfillResult {
    let params = InfillParams::for_fdm()
        .with_cell_size(CELL_SIZE_OVERRIDE)
        .with_solid_caps(false);
    let Ok(result) = generate_infill(fixture, &params) else {
        unreachable!(
            "validated for_fdm preset + cell_size {CELL_SIZE_OVERRIDE} (solid_caps = false) \
             on a 50 mm cube cannot fail"
        );
    };
    result
}

// =============================================================================
// Post-fix anchors on the default run (all five F6 gaps closed)
// =============================================================================

fn verify_post_fix_anchors(fixture: &IndexedMesh, result: &InfillResult) -> Result<()> {
    verify_gap_a_shell(fixture, result)?;
    verify_lattice_geometry(result)?;
    verify_combined_mesh_totals(result)?;
    verify_gap_b_connections(result)?;
    verify_gap_c_cap_carving(result)?;
    verify_gap_d_volumes(result)?;
    verify_gap_e_interior_and_density(result)?;
    Ok(())
}

fn verify_gap_a_shell(fixture: &IndexedMesh, result: &InfillResult) -> Result<()> {
    // Shell is a real inward-offset hollow shell, not a clone of the
    // input mesh.
    ensure!(
        result.shell.vertex_count() != fixture.vertex_count(),
        "gap-a: shell.vertex_count ({}) should differ from fixture.vertex_count ({}); shell is mesh.clone() pre-fix",
        result.shell.vertex_count(),
        fixture.vertex_count(),
    );
    ensure!(
        result.shell.vertex_count() == EXPECTED_SHELL_VC,
        "shell.vertex_count = {}, expected {EXPECTED_SHELL_VC}",
        result.shell.vertex_count(),
    );
    ensure!(
        result.shell.face_count() == EXPECTED_SHELL_FC,
        "shell.face_count = {}, expected {EXPECTED_SHELL_FC}",
        result.shell.face_count(),
    );
    Ok(())
}

fn verify_lattice_geometry(result: &InfillResult) -> Result<()> {
    ensure!(
        result.lattice.vertex_count() == EXPECTED_LATTICE_VC,
        "lattice.vertex_count = {}, expected {EXPECTED_LATTICE_VC}",
        result.lattice.vertex_count(),
    );
    ensure!(
        result.lattice.face_count() == EXPECTED_LATTICE_FC,
        "lattice.face_count = {}, expected {EXPECTED_LATTICE_FC}",
        result.lattice.face_count(),
    );
    Ok(())
}

fn verify_combined_mesh_totals(result: &InfillResult) -> Result<()> {
    // shell + lattice + 2 cap boxes + 37 connection-strut cylinders.
    ensure!(
        result.vertex_count() == EXPECTED_MESH_VC,
        "mesh.vertex_count = {}, expected {EXPECTED_MESH_VC}",
        result.vertex_count(),
    );
    ensure!(
        result.triangle_count() == EXPECTED_MESH_TC,
        "mesh.triangle_count = {}, expected {EXPECTED_MESH_TC}",
        result.triangle_count(),
    );
    Ok(())
}

fn verify_gap_b_connections(result: &InfillResult) -> Result<()> {
    // Connections inject extra verts/tris beyond shell + lattice (+
    // caps).
    ensure!(
        result.vertex_count() > result.shell.vertex_count() + result.lattice.vertex_count(),
        "gap-b: mesh.vertex_count ({}) should exceed shell + lattice ({} + {} = {}); no connection verts injected pre-fix",
        result.vertex_count(),
        result.shell.vertex_count(),
        result.lattice.vertex_count(),
        result.shell.vertex_count() + result.lattice.vertex_count(),
    );
    let connection_verts = result.vertex_count()
        - result.shell.vertex_count()
        - result.lattice.vertex_count()
        - EXPECTED_CAP_VERTS;
    ensure!(
        connection_verts == EXPECTED_CONNECTION_COUNT * STRUT_VERTS_PER_CYLINDER,
        "connection_verts = {}, expected {} = {} connections × {} verts/cylinder",
        connection_verts,
        EXPECTED_CONNECTION_COUNT * STRUT_VERTS_PER_CYLINDER,
        EXPECTED_CONNECTION_COUNT,
        STRUT_VERTS_PER_CYLINDER,
    );
    let connection_tris = result.triangle_count()
        - result.shell.face_count()
        - result.lattice.face_count()
        - EXPECTED_CAP_TRIS;
    ensure!(
        connection_tris == EXPECTED_CONNECTION_COUNT * STRUT_TRIS_PER_CYLINDER,
        "connection_tris = {}, expected {} = {} connections × {} tris/cylinder",
        connection_tris,
        EXPECTED_CONNECTION_COUNT * STRUT_TRIS_PER_CYLINDER,
        EXPECTED_CONNECTION_COUNT,
        STRUT_TRIS_PER_CYLINDER,
    );
    Ok(())
}

fn verify_gap_c_cap_carving(result: &InfillResult) -> Result<()> {
    // Top wide-region (above SIDE − inset − cell_size = 33.8 mm)
    // holds ZERO lattice verts post-cap-carving. Pre-fix: 448 verts
    // at the iz=3 strut row z ≈ 36.2 (also surfaced in the no-caps
    // comparison run).
    let top_wide_count = count_top_wide(result);
    ensure!(
        top_wide_count == 0,
        "gap-c: lattice should hold zero verts at z > {TOP_WIDE_Z} mm (carved cap band shrinks lattice domain to [10.2, 39.8] mm); found {top_wide_count}",
    );
    Ok(())
}

fn verify_gap_d_volumes(result: &InfillResult) -> Result<()> {
    // shell_volume / lattice_volume are signed-volume integrals, not
    // bbox-heuristic. The offset shell signed volume bounds tightly
    // to the analytical lower bound 50³ − 47.6³ = 17 149.824 mm³ +
    // a 45.355 mm³ chamfer from MC linear interpolation on the inner
    // offset.
    ensure!(
        (result.shell_volume - EXPECTED_SHELL_VOLUME).abs() < VOLUME_TOL,
        "shell_volume = {:.17}, expected {EXPECTED_SHELL_VOLUME} ± {VOLUME_TOL}",
        result.shell_volume,
    );
    ensure!(
        result.shell_volume < SIDE * SIDE * SIDE,
        "gap-d: shell_volume ({}) should be < SIDE³ ({}) — shell is hollow post-fix, not the input cube",
        result.shell_volume,
        SIDE * SIDE * SIDE,
    );
    ensure!(
        (result.lattice_volume - EXPECTED_LATTICE_VOLUME).abs() < VOLUME_TOL,
        "lattice_volume = {:.17}, expected {EXPECTED_LATTICE_VOLUME} ± {VOLUME_TOL}",
        result.lattice_volume,
    );

    // total_volume = shell_volume + lattice_volume (interior_volume
    // is reported separately and is NOT part of total_volume per
    // `InfillResult::total_volume`).
    let expected_total = EXPECTED_SHELL_VOLUME + EXPECTED_LATTICE_VOLUME;
    ensure!(
        (result.total_volume() - expected_total).abs() < TOTAL_VOLUME_TOL,
        "total_volume = {:.17}, expected shell_volume + lattice_volume = {EXPECTED_SHELL_VOLUME} + {EXPECTED_LATTICE_VOLUME} = {expected_total}",
        result.total_volume(),
    );
    Ok(())
}

fn verify_gap_e_interior_and_density(result: &InfillResult) -> Result<()> {
    // interior_volume from mesh-SDF intersection on the offset shell.
    // Bit-equal to AABB inset on the cube (37.6³ = 53 157.376); the
    // structural difference shows up only on non-cube fixtures.
    ensure!(
        (result.interior_volume - EXPECTED_INTERIOR_VOLUME).abs() < VOLUME_TOL,
        "interior_volume = {:.17}, expected {EXPECTED_INTERIOR_VOLUME} ± {VOLUME_TOL}",
        result.interior_volume,
    );
    ensure!(
        result.interior_volume > 0.0,
        "interior_volume should be > 0: got {}",
        result.interior_volume,
    );

    // actual_density — empirical lock at the heuristic value (see
    // EXPECTED_ACTUAL_DENSITY doc).
    ensure!(
        (0.0..=1.0).contains(&result.actual_density),
        "actual_density should be in [0, 1]: got {}",
        result.actual_density,
    );
    ensure!(
        (result.actual_density - EXPECTED_ACTUAL_DENSITY).abs() < DENSITY_TOL,
        "actual_density = {:.17}, expected {EXPECTED_ACTUAL_DENSITY} ± {DENSITY_TOL}",
        result.actual_density,
    );
    Ok(())
}

// =============================================================================
// Comparison anchors — solid_caps true vs solid_caps false
// =============================================================================

fn verify_comparison_anchors(with_caps: &InfillResult, without_caps: &InfillResult) -> Result<()> {
    let with_top_wide = count_top_wide(with_caps);
    let without_top_wide = count_top_wide(without_caps);

    // Comparison anchor #1 — gap-c witness flips when caps are
    // removed: wide top region (z > 33.8) holds zero lattice verts
    // WITH caps and 448 verts WITHOUT (the cells_z = 4 strut layer
    // that cap-carving gets rid of).
    ensure!(
        with_top_wide == 0,
        "comparison: with-caps lattice should hold zero verts at z > {TOP_WIDE_Z} mm; found {with_top_wide}",
    );
    ensure!(
        without_top_wide > 0,
        "comparison: without-caps lattice should hold > 0 verts at z > {TOP_WIDE_Z} mm \
         (gap-c carving differentiates the two configs); found {without_top_wide}",
    );

    // Comparison anchor #2 — without caps, the lattice fills the
    // un-carved interior, so `lattice.vertex_count` is strictly
    // larger.
    ensure!(
        with_caps.lattice.vertex_count() < without_caps.lattice.vertex_count(),
        "comparison: with-caps lattice.vc ({}) should be < without-caps lattice.vc ({})",
        with_caps.lattice.vertex_count(),
        without_caps.lattice.vertex_count(),
    );

    // Comparison anchor #3 — empirically, the un-carved cells_z = 4
    // strut layer adds more lattice tris (~960) than the 24 cap-box
    // tris that get removed, so `mesh.triangle_count` is HIGHER
    // without caps.
    ensure!(
        with_caps.triangle_count() < without_caps.triangle_count(),
        "comparison: with-caps mesh.tc ({}) should be < without-caps mesh.tc ({}) — un-carved strut layer dwarfs the 24 cap tris",
        with_caps.triangle_count(),
        without_caps.triangle_count(),
    );

    Ok(())
}

fn count_top_wide(result: &InfillResult) -> usize {
    result
        .lattice
        .vertices
        .iter()
        .filter(|v| v.z > TOP_WIDE_Z)
        .count()
}

// =============================================================================
// PLY outputs — per spec §5.9 lines 802-806
// =============================================================================

fn write_plys(fixture: &IndexedMesh, result: &InfillResult) -> Result<()> {
    write_one(fixture, INPUT_PLY)?;
    write_one(&result.shell, SHELL_PLY)?;
    write_one(&result.lattice, LATTICE_PLY)?;
    write_one(&result.mesh, COMPOSITE_PLY)?;
    Ok(())
}

fn write_one(mesh: &IndexedMesh, path: &str) -> Result<()> {
    let path = PathBuf::from(path);
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    save_ply(mesh, &path, true)?;
    Ok(())
}

// =============================================================================
// Summary
// =============================================================================

fn print_summary(result: &InfillResult, no_caps: &InfillResult) {
    println!("=== mesh-lattice-mesh-bounded-infill (post-fix anchors) ===");
    println!();
    println!("Fixture: 50 mm × 50 mm × 50 mm watertight cube");
    println!(
        "  - 8 vertices in [0, {SIDE}]³, 12 outward-CCW triangles, signed_volume = {:.3} mm³",
        SIDE * SIDE * SIDE,
    );
    println!();
    println!(
        "Default config: generate_infill(&fixture, &for_fdm.with_cell_size({CELL_SIZE_OVERRIDE}))"
    );
    println!(
        "  - shell.{{vc, fc}}    = {}, {}",
        result.shell.vertex_count(),
        result.shell.face_count(),
    );
    println!(
        "  - lattice.{{vc, fc}}  = {}, {}",
        result.lattice.vertex_count(),
        result.lattice.face_count(),
    );
    println!(
        "  - cap geometry      = {EXPECTED_CAP_VERTS} verts / {EXPECTED_CAP_TRIS} tris (2 boxes × 8/12)",
    );
    println!(
        "  - connections       = {EXPECTED_CONNECTION_COUNT} struts × {STRUT_VERTS_PER_CYLINDER}/{STRUT_TRIS_PER_CYLINDER} per cylinder = {} verts / {} tris",
        EXPECTED_CONNECTION_COUNT * STRUT_VERTS_PER_CYLINDER,
        EXPECTED_CONNECTION_COUNT * STRUT_TRIS_PER_CYLINDER,
    );
    println!(
        "  - mesh.{{vc, tc}}     = {}, {} (= shell + lattice + caps + connections)",
        result.vertex_count(),
        result.triangle_count(),
    );
    println!();
    println!("Volume reports:");
    println!(
        "  - shell_volume      = {:.6} mm³  (signed-volume integral on hollow shell)",
        result.shell_volume,
    );
    println!(
        "  - lattice_volume    = {:.6} mm³  (signed-volume integral on lattice)",
        result.lattice_volume,
    );
    println!(
        "  - interior_volume   = {:.6} mm³  (mesh-SDF intersection on offset shell; = 37.6³ on the cube)",
        result.interior_volume,
    );
    println!(
        "  - total_volume      = {:.6} mm³  (= shell + lattice)",
        result.total_volume(),
    );
    println!("  - actual_density    = {:.6}", result.actual_density);
    println!();
    println!("Comparison: solid_caps = true vs false");
    println!(
        "  - mesh.tc           {:>7} vs {:<7} (without > with by {} — un-carved cells_z layer)",
        result.triangle_count(),
        no_caps.triangle_count(),
        no_caps.triangle_count() - result.triangle_count(),
    );
    println!(
        "  - lattice.vc        {:>7} vs {:<7} (caps carve the cap-band domain)",
        result.lattice.vertex_count(),
        no_caps.lattice.vertex_count(),
    );
    println!(
        "  - top wide z>{TOP_WIDE_Z:.1}    {:>7} vs {:<7} (gap-c witness — without caps, the strut layer at z ≈ 36.2 repopulates)",
        count_top_wide(result),
        count_top_wide(no_caps),
    );
    println!();
    println!("Output written:");
    println!("  - {INPUT_PLY}      (50 mm cube fixture)");
    println!("  - {SHELL_PLY}      (offset hollow shell)");
    println!("  - {LATTICE_PLY}    (interior lattice)");
    println!("  - {COMPOSITE_PLY}  (combined mesh = shell + lattice + caps + connections)");
}
