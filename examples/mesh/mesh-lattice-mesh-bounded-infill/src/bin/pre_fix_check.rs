// `unreachable!()` calls in this binary are diagnostic guards on
// `let-else` branches that cannot fire (validated `for_fdm` preset +
// 50 mm cube fixture is well within `2 × shell_thickness +
// cell_size`). `xtask grade`'s Safety criterion counts un-justified
// `unreachable!()` macros; allow at file level since every call site
// is a post-validation `Result::Err` impossibility.
#![allow(clippy::unreachable)]
//! pre-fix anchor capture — v0.7 `generate_infill` baseline witnesses
//! for **gap a / gap b / gap c / gap d / gap e** of the F6 sub-arc.
//!
//! This binary is the **audit-trail evidence** that the F6 gaps
//! existed numerically before the fixes landed. It is committed at
//! `§6.2 #23` follow-on (the pre-fix anchor capture commit), compiles
//! cleanly through every gap-fix commit (`§6.2 #24-#28`), and is
//! deleted at `§6.2 #29` (the post-fix anchor commit) along with its
//! `[[bin]]` entry in `Cargo.toml`.
//!
//! **Why a separate binary instead of `#[cfg(test)] mod`** —
//! `cargo xtask grade` runs `cargo test --release -p <crate>` per
//! crate (`xtask/src/grade.rs:897`). A `#[test]`-fn pre-fix witness
//! would go red at `§6.2 #24` (gap a fix flips `shell.vertex_count()
//! == 8` to `!= 8`), reddening every subsequent gap-fix commit's CI
//! run. A separate `[[bin]]` is compiled by `cargo build` but not
//! auto-invoked by `cargo test`, so the gap-fix sub-arc commits stay
//! green in CI while the audit-trail evidence lives on disk.
//!
//! Manual gate (run AT THIS COMMIT to capture the audit-trail
//! stdout): `cargo run --bin pre_fix_check --release`.
//!
//! Each anchor is labeled `GAP-X` for the gap it witnesses; many of
//! the numerical values are computed empirically before the run
//! (per `feedback_math_pass_first_handauthored`) — see the constant
//! block below for the closed-form derivations.

use anyhow::{Result, ensure};
use example_mesh_mesh_lattice_mesh_bounded_infill::{SIDE, cube_50mm, verify_fixture};
use mesh_lattice::{InfillParams, InfillResult, generate_infill};

// =============================================================================
// Pre-fix anchor constants — closed-form derivations
// =============================================================================

/// Cell size override (mm) — see `main.rs::CELL_SIZE_OVERRIDE`.
const CELL_SIZE_OVERRIDE: f64 = 10.0;

/// Inset (mm) at v0.7 per `infill.rs:333-336`:
/// `inset = cell_size × 0.5 + shell_thickness = 10 × 0.5 + 1.2 = 6.2`.
const INSET: f64 = CELL_SIZE_OVERRIDE.mul_add(0.5, 1.2);

/// Interior side length (mm) at v0.7 per `infill.rs:337-338`:
/// `interior_side = SIDE - 2 × INSET = 50 - 12.4 = 37.6`.
const INTERIOR_SIDE: f64 = SIDE - 2.0 * INSET;

/// Interior volume (mm³) at v0.7 — AABB-derived per gap e:
/// `INTERIOR_SIDE³ = 37.6³ = 53_157.376` (exact in `f64`; `37.6` is
/// representable, and `37.6 × 37.6 × 37.6 = 53_157.376` rounds to
/// the same `f64`).
const INTERIOR_VOLUME_PRE_FIX: f64 = INTERIOR_SIDE * INTERIOR_SIDE * INTERIOR_SIDE;

/// Bounds volume (mm³): `SIDE³ = 125_000.0`.
const BOUNDS_VOLUME: f64 = SIDE * SIDE * SIDE;

/// Shell volume (mm³) at v0.7 — bbox-heuristic per gap d
/// (`infill.rs:363`): `BOUNDS_VOLUME - INTERIOR_VOLUME = 125_000 -
/// 53_157.376 = 71_842.624`.
const SHELL_VOLUME_PRE_FIX: f64 = BOUNDS_VOLUME - INTERIOR_VOLUME_PRE_FIX;

/// Top-region z-threshold (mm) for the gap c witness — pre-fix, the
/// lattice extends to `interior_max.z = SIDE - INSET = 43.8`, so
/// horizontal-strut cylinder verts at the iz=3 grid row (`z = 36.2`,
/// from `cells_z` = 4 over the un-shrunken interior `[6.2, 43.8]`)
/// land above this threshold (= `43.8 - cell_size`). Post-gap-c-fix
/// (`§6.2 #27`), the lattice iteration domain shrinks to
/// `[10.2, 39.8]` (`cap_thickness` = `solid_cap_layers ×
/// cell_size/resolution = 4 × 10/10 = 4 mm`); `cells_z` drops to 3,
/// iz reindexes over `{10.2, 20.2, 30.2, 40.2}`, and
/// `trim_to_bounds` drops every strut from iz=3 (`end.z = 40.2 >
/// iter_max.z = 39.8`), so the highest surviving lattice vert lands
/// at z ≈ 30.6 — the gap-c witness's `count > 0` ensure! would FAIL
/// post-fix. The witness binary short-circuits at GAP-A's `ensure!`
/// post-`§6.2 #24`, so this gap-c witness never executes after
/// gap-fixes land — the audit-trail evidence is the captured stdout
/// AT the §6.2 #23 follow-on commit, before any gap-fix. The
/// post-fix carving is also observable via the in-tree
/// `test_generate_infill_solid_caps` max-z anchor.
const TOP_REGION_Z: f64 = SIDE - INSET - CELL_SIZE_OVERRIDE;

/// Bottom-region z-threshold (mm) — symmetric to `TOP_REGION_Z`,
/// pre-fix at least one lattice vertex with `z < INSET + cell_size
/// = 16.2`. Post-gap-c-fix carved cap band is the strict subset
/// `[interior_min.z, interior_min.z + cap_thickness] = [6.2, 10.2]`
/// mm on this fixture; same audit-trail framing as `TOP_REGION_Z`
/// (gap-c witness short-circuits behind GAP-A post-fix; witness
/// would fail if reached).
const BOTTOM_REGION_Z: f64 = INSET + CELL_SIZE_OVERRIDE;

/// Numerical tolerance for bit-exact volume anchors (`f64`
/// arithmetic on integer-spatial inputs lands within rounding-error
/// of `1e-12`; we keep `1e-9` cushion against future re-orderings).
const VOLUME_TOL: f64 = 1e-9;

// =============================================================================
// Entry point
// =============================================================================

fn main() -> Result<()> {
    let fixture = cube_50mm();
    verify_fixture(&fixture)?;

    let params = InfillParams::for_fdm().with_cell_size(CELL_SIZE_OVERRIDE);
    let Ok(result) = generate_infill(&fixture, &params) else {
        unreachable!(
            "validated for_fdm preset + cell_size {CELL_SIZE_OVERRIDE} on a 50 mm cube cannot fail \
             (interior side = {INTERIOR_SIDE} > 0)"
        );
    };

    println!("=== pre_fix_check — v0.7 generate_infill BASELINE WITNESSES ===");
    println!();
    println!("Fixture: 50 mm × 50 mm × 50 mm watertight cube");
    println!("Params:  InfillParams::for_fdm().with_cell_size({CELL_SIZE_OVERRIDE})");
    println!();

    witness_gap_a(&fixture, &result)?;
    witness_gap_b(&result)?;
    witness_gap_c(&result)?;
    witness_gap_d(&result)?;
    witness_gap_e(&result)?;

    println!();
    println!("ALL PRE-FIX WITNESSES GREEN.  v0.7 baseline locked.");
    println!();
    println!("This binary is deleted at §6.2 #29 once post-fix anchors land.");
    Ok(())
}

// =============================================================================
// GAP A — shell IS mesh.clone() (infill.rs:353)
// =============================================================================

fn witness_gap_a(fixture: &mesh_types::IndexedMesh, result: &InfillResult) -> Result<()> {
    println!("[GAP-A pre-fix witness]  shell IS mesh.clone() (infill.rs:353)");
    ensure!(
        result.shell.vertex_count() == fixture.vertex_count(),
        "GAP-A: shell.vertex_count = {}, expected fixture.vertex_count = {} (shell should be a clone of input pre-fix)",
        result.shell.vertex_count(),
        fixture.vertex_count(),
    );
    ensure!(
        result.shell.face_count() == fixture.face_count(),
        "GAP-A: shell.face_count = {}, expected fixture.face_count = {} (shell should be a clone of input pre-fix)",
        result.shell.face_count(),
        fixture.face_count(),
    );
    let shell_sv = result.shell.signed_volume();
    ensure!(
        (shell_sv - BOUNDS_VOLUME).abs() < VOLUME_TOL,
        "GAP-A: shell.signed_volume = {shell_sv}, expected {BOUNDS_VOLUME} (shell should be the 50 mm cube clone pre-fix)",
    );
    println!(
        "  ✓ shell.vertex_count = {} = fixture.vertex_count = {}",
        result.shell.vertex_count(),
        fixture.vertex_count(),
    );
    println!(
        "  ✓ shell.face_count   = {} = fixture.face_count   = {}",
        result.shell.face_count(),
        fixture.face_count(),
    );
    println!("  ✓ shell.signed_volume = {shell_sv:.6} mm³ ≈ SIDE³ = {BOUNDS_VOLUME:.6} mm³");
    println!(
        "  → POST-FIX (#24): shell becomes inward-offset hollow shell with different vertex_count."
    );
    println!();
    Ok(())
}

// =============================================================================
// GAP B — connections never injected (combine_meshes is naive offset)
// =============================================================================

fn witness_gap_b(result: &InfillResult) -> Result<()> {
    println!("[GAP-B pre-fix witness]  connect_to_shell read but never acted on");
    let sum_v = result.shell.vertex_count() + result.lattice.vertex_count();
    let sum_f = result.shell.face_count() + result.lattice.face_count();
    ensure!(
        result.mesh.vertex_count() == sum_v,
        "GAP-B: mesh.vertex_count = {}, expected shell + lattice = {} + {} = {} (no connection-strut verts injected pre-fix)",
        result.mesh.vertex_count(),
        result.shell.vertex_count(),
        result.lattice.vertex_count(),
        sum_v,
    );
    ensure!(
        result.mesh.face_count() == sum_f,
        "GAP-B: mesh.face_count = {}, expected shell + lattice = {} + {} = {} (no connection-strut tris injected pre-fix)",
        result.mesh.face_count(),
        result.shell.face_count(),
        result.lattice.face_count(),
        sum_f,
    );
    println!(
        "  ✓ mesh.vertex_count = {} = shell ({}) + lattice ({}) — no bridging-strut verts pre-fix",
        result.mesh.vertex_count(),
        result.shell.vertex_count(),
        result.lattice.vertex_count(),
    );
    println!(
        "  ✓ mesh.face_count   = {} = shell ({}) + lattice ({})",
        result.mesh.face_count(),
        result.shell.face_count(),
        result.lattice.face_count(),
    );
    println!(
        "  → POST-FIX (#28): mesh.vertex_count > shell + lattice (bridging struts inject extra verts)."
    );
    println!();
    Ok(())
}

// =============================================================================
// GAP C — solid_caps read but never acted on (lattice fills full interior)
// =============================================================================

fn witness_gap_c(result: &InfillResult) -> Result<()> {
    println!(
        "[GAP-C pre-fix witness]  solid_caps read but never acted on (lattice fills full interior)"
    );
    let lattice_top_count = result
        .lattice
        .vertices
        .iter()
        .filter(|v| v.z > TOP_REGION_Z)
        .count();
    let lattice_bottom_count = result
        .lattice
        .vertices
        .iter()
        .filter(|v| v.z < BOTTOM_REGION_Z)
        .count();
    ensure!(
        lattice_top_count > 0,
        "GAP-C: expected at least one lattice vertex in top region z > {TOP_REGION_Z}, found 0 (cap region should be lattice-populated pre-fix)",
    );
    ensure!(
        lattice_bottom_count > 0,
        "GAP-C: expected at least one lattice vertex in bottom region z < {BOTTOM_REGION_Z}, found 0 (cap region should be lattice-populated pre-fix)",
    );
    println!("  ✓ lattice has {lattice_top_count} vertices in top region (z > {TOP_REGION_Z} mm)");
    println!(
        "  ✓ lattice has {lattice_bottom_count} vertices in bottom region (z < {BOTTOM_REGION_Z} mm)"
    );
    println!(
        "  → POST-FIX (#27): cap bands [interior_max.z - cap_thickness, interior_max.z] and \
         [interior_min.z, interior_min.z + cap_thickness] (= [39.8, 43.8] and [6.2, 10.2] mm \
         on this fixture; cap_thickness = solid_cap_layers × cell_size/resolution = 4 mm) \
         become solid slabs; the lattice iteration domain shrinks to [10.2, 39.8], and \
         trim_to_bounds drops the iz=3 row at z=40.2, so the gap-c witness count would \
         FAIL post-fix — the witness short-circuits behind GAP-A's ensure! post-§6.2 #24, \
         preserving this commit's stdout as the audit-trail evidence."
    );
    println!();
    Ok(())
}

// =============================================================================
// GAP D — bbox-heuristic volumes (infill.rs:363-371)
// =============================================================================

fn witness_gap_d(result: &InfillResult) -> Result<()> {
    println!("[GAP-D pre-fix witness]  shell_volume / lattice_volume are bbox-heuristics");
    ensure!(
        (result.shell_volume - SHELL_VOLUME_PRE_FIX).abs() < VOLUME_TOL,
        "GAP-D: shell_volume = {}, expected BOUNDS_VOLUME - INTERIOR_VOLUME = {} - {} = {} (bbox-heuristic per infill.rs:363)",
        result.shell_volume,
        BOUNDS_VOLUME,
        INTERIOR_VOLUME_PRE_FIX,
        SHELL_VOLUME_PRE_FIX,
    );
    let expected_lattice_volume = INTERIOR_VOLUME_PRE_FIX * result.actual_density;
    ensure!(
        (result.lattice_volume - expected_lattice_volume).abs() < VOLUME_TOL,
        "GAP-D: lattice_volume = {}, expected interior_volume × actual_density = {} × {} = {} (bbox-heuristic per infill.rs:371)",
        result.lattice_volume,
        INTERIOR_VOLUME_PRE_FIX,
        result.actual_density,
        expected_lattice_volume,
    );
    println!(
        "  ✓ shell_volume    = {:.6} mm³ ≈ {BOUNDS_VOLUME:.0} - {INTERIOR_VOLUME_PRE_FIX:.6} = {SHELL_VOLUME_PRE_FIX:.6} (bbox-heuristic)",
        result.shell_volume,
    );
    println!(
        "  ✓ lattice_volume  = {:.6} mm³ ≈ {INTERIOR_VOLUME_PRE_FIX:.6} × actual_density {} = {expected_lattice_volume:.6}",
        result.lattice_volume, result.actual_density,
    );
    println!(
        "  → POST-FIX (#25): shell_volume = signed_volume(offset_shell); lattice_volume = signed_volume(lattice)."
    );
    println!();
    Ok(())
}

// =============================================================================
// GAP E — interior bounds via AABB inset (infill.rs:337-338)
// =============================================================================

fn witness_gap_e(result: &InfillResult) -> Result<()> {
    println!("[GAP-E pre-fix witness]  interior_volume from AABB inset, not mesh-SDF intersection");
    ensure!(
        (result.interior_volume - INTERIOR_VOLUME_PRE_FIX).abs() < VOLUME_TOL,
        "GAP-E: interior_volume = {}, expected (SIDE - 2 × INSET)³ = {INTERIOR_SIDE}³ = {INTERIOR_VOLUME_PRE_FIX} (AABB-derived per infill.rs:337-338)",
        result.interior_volume,
    );
    println!(
        "  ✓ interior_volume = {:.6} mm³ = (SIDE - 2 × {INSET})³ = {INTERIOR_SIDE}³ = {INTERIOR_VOLUME_PRE_FIX:.6} mm³ (AABB inset)",
        result.interior_volume,
    );
    println!(
        "  → POST-FIX (#26): interior bounded by mesh-SDF intersection on the offset shell (numerically equal for the cube fixture, structurally different)."
    );
    println!();
    Ok(())
}
