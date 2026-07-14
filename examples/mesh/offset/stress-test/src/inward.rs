//! Inward offset (erosion) of a unit cube via SDF + marching cubes.
//!
//! Offsets `mesh_types::unit_cube()` inward by `-0.1` and checks the
//! result against the exact **polytope** volume. Eroding a convex
//! polytope has no convex features to round (the level set retreats into
//! the body and stays sharp), so — unlike the outward [`crate::outward`]
//! Steiner-Minkowski case — the offset body is just a smaller cube:
//!
//! ```text
//!   V_d = (1 + 2d)³      (exact; d < 0 for erosion)
//!   A_d = 6 (1 + 2d)²
//! ```
//!
//! For `d = -0.1`: `V = 0.8³ = 0.512`, `A = 6 × 0.64 = 3.84`.
//!
//! ## The grid-alignment topology pitfall
//!
//! Marching cubes classifies a cell by the SDF signs at its 8 corners.
//! When the eroded cube's flat faces land **exactly on sample planes**
//! (e.g. faces at `x ∈ {0.1, 0.9}` with a cell size that divides `0.1`),
//! the SDF is zero across a whole face and the sign test is ambiguous.
//! MC then stitches spurious handles through those degenerate cells: the
//! mesh stays closed and its volume stays correct, but its **genus jumps
//! above 0** (`χ < 2`). Nudging the resolution off-grid so the level set
//! falls *between* samples removes the degeneracy and restores a clean
//! genus-0 manifold.
//!
//! This module asserts the clean off-grid guarantee and *demonstrates*
//! the on-grid degeneracy as diagnostic output — it does not assert the
//! spurious genus (that is a marching-cubes limitation, not a contract
//! to lock in; a future MC improvement should be free to fix it).
//! Outward offset is immune to the same alignment — see
//! [`crate::outward`] for that contrast.

use anyhow::Result;
use mesh_offset::{OffsetConfig, offset_mesh};
use mesh_repair::validate_mesh;
use mesh_types::{IndexedMesh, unit_cube};

use crate::diag::{
    assert_clean_offset_manifold, assert_offset_aabb, genus_str, print_diagnostics, save_to_out,
    verify_round_trip, verify_unit_cube_input,
};

/// Offset distance in mesh units. Negative = contraction (erosion).
/// Magnitude must satisfy `|distance| < 0.5` for a unit cube; larger
/// values shrink the inner level set to zero volume and `offset_mesh`
/// returns `MarchingCubesFailed` because no triangles are generated.
const OFFSET_DISTANCE: f64 = -0.1;

/// Marching-cubes cell size for the primary (asserted) run. `0.03` does
/// NOT divide the offset magnitude `0.1`, so the eroded faces at
/// `x ∈ {0.1, 0.9}` fall *between* sample planes — the level set is
/// bilinearly interpolated and the output is a clean genus-0 manifold.
const GRID_RESOLUTION: f64 = 0.03;

/// A grid-aligned cell size (`0.1 / 0.025 = 4` cells per offset), which
/// places the eroded faces exactly on sample planes. Used only to
/// demonstrate the topology pitfall — see the module docs.
const GRID_ALIGNED_RESOLUTION: f64 = 0.025;

pub fn run() -> Result<()> {
    let before = unit_cube();
    verify_unit_cube_input(&before);
    let config = OffsetConfig::default().with_resolution(GRID_RESOLUTION);
    let after = offset_mesh(&before, OFFSET_DISTANCE, &config)?;
    let after_path = save_to_out("inward.ply", &after)?;

    println!("==== offset-inward (erosion) ====");
    println!();
    println!("input  : unit_cube() — 8v 12f, AABB [0,1]^3");
    println!("config : OffsetConfig::default().with_resolution({GRID_RESOLUTION})");
    println!("offset : {OFFSET_DISTANCE} (inward, erosion)");
    println!();

    print_diagnostics("before (unit_cube)", &before);
    println!();
    print_diagnostics("after  (offset_mesh)", &after);
    println!();

    verify_after(&after);
    demonstrate_grid_alignment_pitfall(&before)?;
    verify_round_trip("inward", &after, &after_path)?;

    println!("OK — inward offset verified (polytope preservation, genus 0)");
    println!();
    Ok(())
}

/// Post-offset checks: clean genus-0 manifold, level set in the right
/// place, and volume matching the exact polytope formula.
fn verify_after(mesh: &IndexedMesh) {
    // ── Topology: single clean, closed, outward-wound genus-0 manifold ──
    assert_clean_offset_manifold(mesh, "inward");

    // ── Geometric: level set at the analytical position ─────────────────
    assert_offset_aabb(mesh, OFFSET_DISTANCE, GRID_RESOLUTION);

    // ── Exact polytope volume ──────────────────────────────────────────
    //   V_d = (1 + 2d)³   (exact; erosion preserves the sharp polytope)
    // Only MC discretization contributes error — there are no rounded
    // features to approximate — so the tolerance is tighter than the
    // outward Steiner case (2% vs 5%; measured drift here is ~0.1%).
    let d = OFFSET_DISTANCE;
    let expected_vol = 2.0f64.mul_add(d, 1.0).powi(3);
    let vol = mesh.signed_volume();
    assert!(
        ((vol - expected_vol) / expected_vol).abs() < 0.02,
        "signed_volume should match exact polytope formula ({expected_vol:.4}) within 2%; got {vol:.4}",
    );
}

/// Demonstrate the grid-alignment topology pitfall: at a resolution that
/// aligns the eroded faces onto sample planes, marching cubes stitches
/// spurious handles and the genus jumps above 0.
///
/// **Print-only, deliberately un-asserted.** The pitfall is a
/// marching-cubes limitation left open to a future fix (dual contouring,
/// sample perturbation, …), not a contract to lock in — so nothing about
/// the degenerate output is gated. Asserting its genus, closedness, or
/// volume would couple `xtask run-validators` (CI) to an un-contracted
/// degeneracy and turn red on a benign MC improvement or float drift,
/// which is exactly the anti-pattern this re-narration retired (the old
/// examples asserted an inside-out quirk that a fix later invalidated).
/// The real contract is the off-grid clean path in [`verify_after`].
fn demonstrate_grid_alignment_pitfall(before: &IndexedMesh) -> Result<()> {
    let config = OffsetConfig::default().with_resolution(GRID_ALIGNED_RESOLUTION);
    let aligned = offset_mesh(before, OFFSET_DISTANCE, &config)?;
    let report = validate_mesh(&aligned);
    println!(
        "grid-aligned (res {GRID_ALIGNED_RESOLUTION}) : genus {}, \
         watertight={}, |V|={:.4} — spurious handles from on-plane MC ambiguity",
        genus_str(&report),
        report.is_watertight,
        aligned.signed_volume().abs(),
    );
    Ok(())
}
