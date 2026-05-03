// `unreachable!()` calls in this binary are diagnostic guards on
// `let-else` branches that cannot fire (validated `for_fdm` preset +
// 50 mm cube fixture is well within `2 × shell_thickness +
// cell_size`, so `generate_infill` cannot return `Err` on the smoke
// path; same logic for the negative-validate anchor below).
// `xtask grade`'s Safety criterion counts un-justified `unreachable!()`
// macros; allow at file level since every call site is a post-
// validation `Result::Err` impossibility, not a real panic site.
#![allow(clippy::unreachable)]
//! mesh-lattice-mesh-bounded-infill — FDM-style shell + lattice
//! composite via `generate_infill` on a hand-authored 50 mm
//! watertight cube.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.9. **This is the
//! pre-fix anchor capture commit (`§6.2 #23` follow-on)**, the
//! second commit in PR2's mesh-bounded-infill arc.
//!
//! At this commit (and through the entire F6 gap-fix sub-arc at
//! `§6.2 #24-#28`), `main.rs` runs only **stable** anchors — those
//! that are true at the v0.7 baseline AND remain true after every
//! gap fix lands:
//!
//! - the math-pass anchors on the 50 mm cube fixture itself
//!   (`fixture::verify_fixture`: 8 verts in `[0, 50]³`, 12 outward-
//!   CCW windings, AABB at `(0, 0, 0)` / `(50, 50, 50)`,
//!   `signed_volume == 125_000.0`);
//! - the `InfillParams::for_fdm` / `for_lightweight` / `for_strong`
//!   preset `validate()` paths;
//! - the negative-validate path via direct field assignment per
//!   `mesh/MESH_V1_EXAMPLES_SCOPE.md` HE-3 (the `with_shell_thickness`
//!   builder clamps to `max(0.0)` per `infill.rs:161-164`, so the
//!   negative-`shell_thickness` branch through `validate` is reachable
//!   only by direct construction);
//! - the `EmptyMesh` and `InteriorTooSmall` error paths
//!   (`infill.rs:295-297` and `infill.rs:340-345`);
//! - smoke-call anchors on `generate_infill(&fixture, &for_fdm)` that
//!   the F6 gap-fix sub-arc does not invalidate: `actual_density` is
//!   finite and within `[0.0, 1.0]`, `interior_volume > 0.0`,
//!   `vertex_count > 0`, `triangle_count > 0`, `total_volume` is
//!   finite.
//!
//! The **v0.7 baseline witnesses** (gap-a/b/c/d/e — values that flip
//! as each gap-fix lands) live in the audit-trail binary
//! `src/bin/pre_fix_check.rs`, runnable via
//! `cargo run --bin pre_fix_check --release`. That binary compiles
//! through the entire F6 sub-arc but is never auto-invoked by CI
//! (`cargo test -p ...` does not run secondary binaries); its
//! captured stdout at this commit IS the audit-trail evidence that
//! gaps a-e existed numerically before the fixes landed.
//!
//! At `§6.2 #29` (post-fix anchor commit), the audit-trail binary
//! and its `[[bin]]` entry are deleted, and `main.rs` is rewritten
//! to assert post-fix anchors (real shell `vertex_count`, signed-
//! volume integrals on shell + lattice, solid caps cover top/bottom,
//! lattice-to-shell connections present). Composite counterpart to
//! §5.8 `mesh-lattice-shape-bounded` (the analytical-SDF-trimmed
//! path); both ship in v1.0.

use std::path::Path;

use anyhow::{Result, ensure};
use example_mesh_mesh_lattice_mesh_bounded_infill::{
    SIDE, cube_50mm, cube_at_origin, verify_fixture,
};
use mesh_io::save_ply;
use mesh_lattice::{InfillParams, LatticeError, generate_infill};
use mesh_types::IndexedMesh;

// =============================================================================
// Fixture / smoke-call constants
// =============================================================================

/// Cell size override in mm — the `for_fdm` preset's default is
/// `5.0` (per `infill.rs:94-105`: `LatticeParams::cubic(5.0)`),
/// which yields ~514 cells in the post-shell interior of the 50 mm
/// cube. Overriding to `10.0` via `.with_cell_size(10.0)` yields
/// ~54 cells — coarser, more visually readable in cross-section,
/// and identical reasoning to the in-tree `test_generate_infill_basic`
/// at `mesh-lattice/src/infill.rs:498-507`.
const CELL_SIZE_OVERRIDE: f64 = 10.0;

/// Edge length in mm of the tiny cube used for the
/// `InteriorTooSmall` error-path anchor — combined with the
/// `for_fdm + cell_size 4` parameters (`inset = 4 × 0.5 + 1.2 = 3.2`
/// mm, so `5 - 2 × 3.2 = -1.4` mm is negative, triggering the check
/// at `infill.rs:340-345`).
const TINY_CUBE_SIDE: f64 = 5.0;

/// Cell size in mm for the `InteriorTooSmall` error-path anchor —
/// see [`TINY_CUBE_SIDE`] for the inset arithmetic.
const TINY_CELL_SIZE: f64 = 4.0;

/// Output path for the 50 mm cube fixture PLY.
const OUTPUT_PATH: &str = "out/input.ply";

// =============================================================================
// Entry point
// =============================================================================

fn main() -> Result<()> {
    let fixture = cube_50mm();
    verify_fixture(&fixture)?;

    verify_validate_paths()?;
    verify_error_paths()?;

    let smoke = run_smoke_path(&fixture);
    verify_smoke_anchors(&smoke)?;

    write_fixture_ply(&fixture)?;

    print_summary(&smoke);
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
    // `infill.rs:161-164`, so the negative-`shell_thickness` branch
    // through `validate` (`infill.rs:196-198`) is reachable only by
    // direct construction.
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
// Smoke path — exercises the success branch of generate_infill
// =============================================================================

/// Runs `generate_infill` on the 50 mm cube fixture under
/// `for_fdm + cell_size 10`.
///
/// The resulting `InfillResult` is then checked against
/// [`verify_smoke_anchors`] for stable-across-F6 invariants.
fn run_smoke_path(fixture: &IndexedMesh) -> mesh_lattice::InfillResult {
    let params = InfillParams::for_fdm().with_cell_size(CELL_SIZE_OVERRIDE);
    let Ok(result) = generate_infill(fixture, &params) else {
        unreachable!(
            "validated for_fdm preset + cell_size {CELL_SIZE_OVERRIDE} on a 50 mm cube cannot fail \
             (interior side = 50 - 2 × (5 + 1.2) = 37.6 mm > 0)"
        );
    };
    result
}

/// Anchors that hold at the v0.7 baseline AND through every F6 gap-
/// fix commit (`§6.2 #24-#28`). Anchors that flip as gaps close
/// (`shell.vertex_count() == 8`, bbox-heuristic volumes, etc.) live
/// in `bin/pre_fix_check.rs`.
fn verify_smoke_anchors(result: &mesh_lattice::InfillResult) -> Result<()> {
    ensure!(
        result.actual_density.is_finite(),
        "actual_density should be finite: got {}",
        result.actual_density,
    );
    ensure!(
        (0.0..=1.0).contains(&result.actual_density),
        "actual_density should be in [0, 1]: got {}",
        result.actual_density,
    );

    ensure!(
        result.interior_volume > 0.0,
        "interior_volume should be > 0: got {}",
        result.interior_volume,
    );

    ensure!(
        result.vertex_count() > 0,
        "result.vertex_count() should be > 0: got {}",
        result.vertex_count(),
    );
    ensure!(
        result.triangle_count() > 0,
        "result.triangle_count() should be > 0: got {}",
        result.triangle_count(),
    );

    ensure!(
        result.total_volume().is_finite(),
        "total_volume should be finite: got {}",
        result.total_volume(),
    );

    Ok(())
}

// =============================================================================
// PLY output
// =============================================================================

fn write_fixture_ply(fixture: &IndexedMesh) -> Result<()> {
    let path = Path::new(OUTPUT_PATH);
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    save_ply(fixture, path, true)?;
    Ok(())
}

// =============================================================================
// Summary
// =============================================================================

fn print_summary(result: &mesh_lattice::InfillResult) {
    println!("=== mesh-lattice-mesh-bounded-infill (PRE-FIX ANCHOR CAPTURE) ===");
    println!();
    println!("Fixture: 50 mm × 50 mm × 50 mm watertight cube");
    println!("  - 8 vertices in [0, {SIDE}]³, 12 outward-CCW triangles");
    println!(
        "  - signed_volume = {:.3} mm³ (= {SIDE}³)",
        SIDE * SIDE * SIDE
    );
    println!();
    println!(
        "Smoke path: generate_infill(&fixture, &for_fdm.with_cell_size({CELL_SIZE_OVERRIDE}))"
    );
    println!("  - actual_density   = {:.6}", result.actual_density);
    println!("  - interior_volume  = {:.6} mm³", result.interior_volume);
    println!(
        "  - shell_volume     = {:.6} mm³  [bbox-heuristic at v0.7; gap d post-fix → signed-volume integral]",
        result.shell_volume
    );
    println!(
        "  - lattice_volume   = {:.6} mm³  [bbox-heuristic at v0.7; gap d post-fix → signed-volume integral]",
        result.lattice_volume
    );
    println!("  - total_volume     = {:.6} mm³", result.total_volume());
    println!("  - vertex_count     = {}", result.vertex_count());
    println!("  - triangle_count   = {}", result.triangle_count());
    println!();
    println!("Output written: {OUTPUT_PATH} (50 mm cube fixture, PLY binary)");
    println!();
    println!(
        "v0.7 baseline gap-a/b/c/d/e witnesses captured by:  cargo run --bin pre_fix_check --release"
    );
}
