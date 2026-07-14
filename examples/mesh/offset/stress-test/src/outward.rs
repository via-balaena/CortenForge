//! Outward offset (dilation) of a unit cube via SDF + marching cubes.
//!
//! Offsets `mesh_types::unit_cube()` outward by `+0.1` and checks the
//! result against the **Steiner-Minkowski** volume formula. Dilating a
//! convex polytope adds rounded features — a slab over each face, a
//! quarter-cylinder along each edge, a sphere octant at each corner — so
//! the offset body is NOT a scaled cube:
//!
//! ```text
//!   V_d = 1 + 6d + 3πd² + (4π/3)d³
//!         └┬┘ └┬┘  └──┬─┘  └───┬──┘
//!    unit cube  6 face   12 edge   8 corner octants
//!               slabs    quarter-  = one full sphere
//!                        cylinders
//! ```
//!
//! This is the geometric counterpart to [`crate::inward`], which stays a
//! sharp polytope (`V_d = (1 + 2d)³`) because erosion has no convex
//! features to round.
//!
//! `offset_mesh` returns a clean, watertight, outward-wound genus-0
//! manifold directly (no winding flip / weld follow-up). Outward offset
//! is also **robust to grid alignment**: unlike [`crate::inward`], its
//! rounded Steiner surface never coincides with a full marching-cubes
//! sample plane, so it stays genus 0 even at a grid-aligned resolution.
//! This module asserts that robustness as the flip side of the inward
//! pitfall.

use anyhow::Result;
use mesh_offset::{OffsetConfig, offset_mesh};
use mesh_repair::validate_mesh;
use mesh_types::{IndexedMesh, unit_cube};

use crate::diag::{
    assert_clean_offset_manifold, assert_offset_aabb, genus_str, print_diagnostics, save_to_out,
    verify_round_trip, verify_unit_cube_input,
};

/// Offset distance in mesh units. Positive = expansion (dilation).
const OFFSET_DISTANCE: f64 = 0.1;

/// Marching-cubes cell size for the primary (asserted) run. Off the
/// cube's grid alignment, so the level set is bilinearly interpolated
/// between samples and the output is a clean genus-0 manifold. See
/// [`crate::inward`] for what happens when the cell size aligns the
/// eroded faces onto sample planes.
const GRID_RESOLUTION: f64 = 0.03;

/// A grid-aligned cell size (`0.1 / 0.025 = 4` cells per offset). Used
/// only to demonstrate that outward offset stays genus 0 here, in
/// contrast to inward offset which degenerates at the same alignment.
const GRID_ALIGNED_RESOLUTION: f64 = 0.025;

pub fn run() -> Result<()> {
    let before = unit_cube();
    verify_unit_cube_input(&before);
    let config = OffsetConfig::default().with_resolution(GRID_RESOLUTION);
    let after = offset_mesh(&before, OFFSET_DISTANCE, &config)?;
    let after_path = save_to_out("outward.ply", &after)?;

    println!("==== offset-outward (dilation) ====");
    println!();
    println!("input  : unit_cube() — 8v 12f, AABB [0,1]^3");
    println!("config : OffsetConfig::default().with_resolution({GRID_RESOLUTION})");
    println!("offset : +{OFFSET_DISTANCE} (outward, dilation)");
    println!();

    print_diagnostics("before (unit_cube)", &before);
    println!();
    print_diagnostics("after  (offset_mesh)", &after);
    println!();

    verify_after(&after);
    verify_grid_alignment_robustness(&before)?;
    verify_round_trip("outward", &after, &after_path)?;

    println!("OK — outward offset verified (Steiner-Minkowski, genus 0)");
    println!();
    Ok(())
}

/// Post-offset checks: clean genus-0 manifold, level set in the right
/// place, and volume matching the Steiner-Minkowski formula.
fn verify_after(mesh: &IndexedMesh) {
    // ── Topology: single clean, closed, outward-wound genus-0 manifold ──
    // The §Q-5 winding fix + MC edge-vertex cache mean offset_mesh emits
    // this directly — no per-face flip, no weld_vertices follow-up.
    assert_clean_offset_manifold(mesh, "outward");

    // ── Geometric: level set at the analytical position ─────────────────
    assert_offset_aabb(mesh, OFFSET_DISTANCE, GRID_RESOLUTION);

    // ── Steiner-Minkowski volume ───────────────────────────────────────
    //   V_d = 1 + 6d + 3πd² + (4π/3)d³   (cube + faces + edges + corners)
    // Marching-cubes discretization of the rounded features introduces a
    // few-percent error; 5% is comfortable headroom at this resolution
    // (measured drift here is ~0.13%). The rounded terms are what make
    // this exceed the inward polytope volume — see the module docs.
    let d = OFFSET_DISTANCE;
    let pi = std::f64::consts::PI;
    let expected_vol = 1.0 + 6.0 * d + 3.0 * pi * d.powi(2) + (4.0 * pi / 3.0) * d.powi(3);
    let vol = mesh.signed_volume();
    assert!(
        ((vol - expected_vol) / expected_vol).abs() < 0.05,
        "signed_volume should match Steiner formula ({expected_vol:.4}) within 5%; got {vol:.4}",
    );
    // Sanity that dilation genuinely ROUNDS — gating the MEASURED volume,
    // not the analytic constant. The rounded Steiner body grows past the
    // original cube (adds mass) but stays strictly under the naive
    // uniformly-expanded box of side (1 + 2d): rounding the 12 edges and 8
    // corners carves that corner/edge material back off. If offset_mesh
    // ever regressed to an un-rounded box (vol → (1 + 2d)³ = 1.728), this
    // catches it where the ±5% band above (which tolerates up to ~1.783)
    // would not. This is the asymmetry with [`crate::inward`], where
    // erosion IS exactly the scaled polytope (1 + 2d)³ (nothing to round).
    let expanded_box = 2.0f64.mul_add(d, 1.0).powi(3);
    assert!(
        1.0 < vol && vol < expanded_box,
        "measured outward volume must sit strictly between the cube (1.0) and the expanded box ({expanded_box:.4}); got {vol:.4}",
    );
}

/// Demonstrate the flip side of the inward grid-alignment pitfall:
/// outward offset stays a clean genus-0 manifold at a grid-aligned
/// resolution, because its rounded Steiner surface never coincides with
/// a full marching-cubes sample plane.
fn verify_grid_alignment_robustness(before: &IndexedMesh) -> Result<()> {
    let config = OffsetConfig::default().with_resolution(GRID_ALIGNED_RESOLUTION);
    let aligned = offset_mesh(before, OFFSET_DISTANCE, &config)?;
    let report = validate_mesh(&aligned);
    println!(
        "grid-aligned (res {GRID_ALIGNED_RESOLUTION}) : genus {} — outward is robust to alignment",
        genus_str(&report),
    );
    // Outward at grid alignment is genuinely a clean single-shell genus-0
    // manifold (not degenerate like inward), so the full clean-manifold
    // gate applies — this is the assertable flip side of the inward pitfall.
    assert_clean_offset_manifold(&aligned, "outward (grid-aligned)");
    Ok(())
}
