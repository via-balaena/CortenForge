// `unreachable!()` calls in this binary are diagnostic guards on
// `let-else` branches that cannot fire (validated `for_fdm` preset +
// 50 mm cube fixture is well within `2 × shell_thickness +
// cell_size`, so post-validation `generate_infill` cannot return
// `Err`). `xtask grade`'s Safety criterion counts un-justified
// `unreachable!()`; allow at file level since every call site is a
// post-validation `Result::Err` impossibility, not a real panic.
#![allow(clippy::unreachable)]
//! mesh-bounded-infill — FDM-style shell + lattice
//! composite via `generate_infill` on a hand-authored 50 mm
//! watertight cube.
//!
//! Mesh-bounded counterpart to `shape_bounded`
//! (analytical-SDF trim); both ship in v1.0.
//!
//! Demonstrates the canonical FDM-infill workflow on a watertight
//! input mesh: outer shell from inward offset, lattice in the
//! mesh-bounded interior, optional bridging connections from lattice
//! nodes to the inner shell, optional solid caps near the bbox top
//! and bottom. All five F6 gap-fixes (a/b/c/d/e) are landed; the
//! anchors below lock the post-fix numerical contract on the
//! `for_fdm + cell_size 10` configuration over the 50 mm cube.
//!
//! Anchors (default config, `solid_caps = true`). The shell is a
//! **welded, watertight** hollow shell (two nested closed manifolds);
//! its structure and volume are anchored as **invariants**, not exact
//! marching-cubes counts, so they survive mesher tessellation changes.
//! (The earlier exact `224 672 / 74 900` soup anchors broke silently
//! when `mesh-offset` began welding + the §Q-5 winding fix flipped the
//! inner cavity wall — see `verify_gap_a_shell` / `verify_gap_d_volumes`
//! and the library guard
//! `mesh_lattice::infill::tests::shell_signed_volume_is_wall_not_sum`.)
//!
//! - shell: welded 2-component closed manifold, `2V − F = 8` at any
//!   resolution (currently 40 844 V / 81 680 F on this fixture).
//! - `shell_volume` ≈ 17 170 mm³ — the hollow WALL volume `SIDE³ −
//!   47.6³ = 17 149.8` mm³ + a sub-percent MC chamfer, anchored within
//!   1%; strictly `< SIDE³` (a shell inside the cube cannot exceed it).
//! - lattice: 1 456 verts / 2 496 tris (104 cubic struts × 14/24 per
//!   cylinder; deterministic hand-authored geometry).
//! - caps: 16 verts / 24 tris (2 boxes × 8/12; gap c post-fix).
//! - connections: bridging struts from near-shell lattice nodes to the
//!   inner shell (gap b), ~30 whole 14/24 cylinders on this fixture —
//!   anchored to a plausible range (≥10 non-trivial bridges, and fewer
//!   than the lattice strut count) + well-formed, not an exact count (the
//!   number follows the resolution-dependent closest-point queries).
//! - mesh: shell + lattice + caps + connections, asserted as an
//!   internally-consistent decomposition (whole cylinders, vert/tri
//!   counts agree — catches cross-part welding) rather than an exact
//!   total.
//! - `lattice_volume` = 28.821 mm³ (signed-volume integral; gap d).
//! - `interior_volume` = 53 157.376 mm³ = 37.6³ (mesh-SDF intersection
//!   on offset shell; gap e — bit-equal to AABB inset on the cube,
//!   structurally different on non-cube fixtures).
//! - `actual_density` = 0.010 402.
//!
//! Comparison anchor: a second `generate_infill` run with
//! `solid_caps = false` differentiates the gap-c carving — the wide
//! top region (above `SIDE − inset − cell_size = 33.8 mm`) holds 0
//! lattice verts WITH caps and 448 verts WITHOUT.

mod fixture;

use std::path::PathBuf;

use self::fixture::{SIDE, cube_50mm, cube_at_origin, verify_fixture};
use anyhow::{Result, ensure};
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
// Numerical anchors — for_fdm + cell_size 10, solid_caps = true.
// Shell structure/volume are resolution-independent INVARIANTS (not
// exact MC counts); lattice/cap/volume values are deterministic.
// =============================================================================

/// Shell wall thickness in mm (the `for_fdm` preset default). Sets the
/// inward-offset distance, so the hollow-shell cavity is a `SIDE − 2 ×
/// SHELL_THICKNESS = 47.6` mm cube.
const SHELL_THICKNESS: f64 = 1.2;

/// The shell is a **welded, watertight** hollow shell: two nested
/// genus-0 closed manifold surfaces (outer + inner cavity). Euler's
/// formula makes `F = 2V − 2χ` with `χ = 2 + 2 = 4`, so `2V − F = 8`
/// — a TOPOLOGICAL invariant independent of marching-cubes resolution.
/// This anchors the shell's structure without locking the exact vertex
/// count (currently 40 844 V / 81 680 F on this fixture), which the old
/// `224 672 / 74 900` soup anchors did — and which broke silently when
/// `mesh-offset` began welding its MC output. Vertex-soup (`V = 3F`)
/// would give `2V − F = 5F`, so this also witnesses the weld. Checked
/// as `2V = F + 8` in `usize` (always `2V ≥ F`, no underflow).
const SHELL_EULER_2V_MINUS_F: usize = 8;

/// Number of cubic struts `generate_infill` emits into the 50 mm cube's
/// mesh-bounded interior at `for_fdm` + `cell_size 10` — a deterministic
/// topological count for this fixture, independent of per-cylinder
/// tessellation.
const LATTICE_STRUT_COUNT: usize = 104;

/// `result.lattice.vertex_count()` — one 6-segment cylinder per strut.
/// Derived from the per-cylinder const (not a raw literal) so it tracks
/// `STRUT_VERTS_PER_CYLINDER` if the strut tessellation changes; the
/// lattice is un-welded, so verts scale linearly with strut count
/// (`104 × 14 = 1456`).
const EXPECTED_LATTICE_VC: usize = LATTICE_STRUT_COUNT * STRUT_VERTS_PER_CYLINDER;

/// `result.lattice.face_count()` — 24 tris per strut cylinder, derived
/// (`104 × 24 = 2496`).
const EXPECTED_LATTICE_FC: usize = LATTICE_STRUT_COUNT * STRUT_TRIS_PER_CYLINDER;

/// Cap geometry per gap-c fix: two extruded boxes (`combine_meshes`
/// of two cube sub-meshes) at top + bottom of the inset interior, 8
/// verts and 12 tris each.
const EXPECTED_CAP_VERTS: usize = 16;
const EXPECTED_CAP_TRIS: usize = 24;

/// Cylinder geometry: `mesh-lattice` strut cylinders generate 6
/// axial segments + 2 cap rings = 14 verts, 24 tris.
const STRUT_VERTS_PER_CYLINDER: usize = 14;
const STRUT_TRIS_PER_CYLINDER: usize = 24;

/// Relative tolerance for the shell wall volume vs the analytical
/// `SIDE³ − 47.6³`. The marching-cubes chamfer on the inner offset is
/// ~0.12% (measured 17 169.7 vs analytical 17 149.8), so 1% is ~8×
/// headroom over tessellation drift while still catching a real
/// few-percent volume regression. Replaces the old 17-significant-figure
/// lock, which was fragile to any mesher change; mirrors the library
/// guard `shell_signed_volume_is_wall_not_sum`.
const SHELL_VOLUME_REL_TOL: f64 = 0.01;

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
// PLY output paths — relative to the crate out/ (resolved via CARGO_MANIFEST_DIR)
// =============================================================================

const INPUT_PLY: &str = "out/input.ply";
const SHELL_PLY: &str = "out/shell.ply";
const LATTICE_PLY: &str = "out/lattice.ply";
const COMPOSITE_PLY: &str = "out/composite.ply";

// =============================================================================
// Entry point
// =============================================================================

pub fn run() -> Result<()> {
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
    // Structure via a topological invariant, NOT an exact MC count: the
    // welded hollow shell is two nested genus-0 closed manifolds, so
    // `2V − F = 8` at any resolution (see SHELL_EULER_2V_MINUS_F). This
    // survives mesher tessellation changes; the old exact 224 672 /
    // 74 900 soup anchors did not.
    let (vc, fc) = (result.shell.vertex_count(), result.shell.face_count());
    ensure!(
        2 * vc == fc + SHELL_EULER_2V_MINUS_F,
        "shell should be a welded 2-component closed manifold (2V = F + {SHELL_EULER_2V_MINUS_F}); got V={vc}, F={fc}, 2V={}, F+{SHELL_EULER_2V_MINUS_F}={} (vertex-soup would give 2V = 6F)",
        2 * vc,
        fc + SHELL_EULER_2V_MINUS_F,
    );
    ensure!(
        fc > fixture.face_count(),
        "offset shell ({fc} tris) should be finer than the {}-tri input cube",
        fixture.face_count(),
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
    // The composite is a plain concatenation of shell + lattice + caps +
    // connection struts (no cross-part welding). Rather than lock an
    // exact total — which would rot with the shell tessellation (the
    // shell's exact count is intentionally not anchored; see
    // verify_gap_a_shell) — assert the decomposition is INTERNALLY
    // CONSISTENT: the connection geometry the composite carries beyond
    // shell + lattice + caps must be WHOLE cylinders, and the vert- and
    // tri-derived cylinder counts must AGREE. A cross-part weld or a
    // dropped face would desynchronize them, and a wrong cap count would
    // break divisibility. This holds at any shell resolution.
    let (conn_verts, conn_tris) = connection_geometry(result);
    ensure!(
        conn_verts % STRUT_VERTS_PER_CYLINDER == 0 && conn_tris % STRUT_TRIS_PER_CYLINDER == 0,
        "connection geometry should be whole cylinders: {conn_verts} verts (÷{STRUT_VERTS_PER_CYLINDER}), {conn_tris} tris (÷{STRUT_TRIS_PER_CYLINDER})",
    );
    ensure!(
        conn_verts / STRUT_VERTS_PER_CYLINDER == conn_tris / STRUT_TRIS_PER_CYLINDER,
        "decomposition inconsistent: {conn_verts} verts ⇒ {} cylinders, {conn_tris} tris ⇒ {} cylinders (cross-part weld?)",
        conn_verts / STRUT_VERTS_PER_CYLINDER,
        conn_tris / STRUT_TRIS_PER_CYLINDER,
    );
    Ok(())
}

/// Connection-strut geometry the composite carries beyond
/// shell + lattice + caps, derived from the decomposition
/// `mesh − shell − lattice − caps`.
const fn connection_geometry(result: &InfillResult) -> (usize, usize) {
    // `saturating_sub` so a hypothetical future weld/dedup regression (the
    // composite carrying fewer verts than shell + lattice + caps) yields 0
    // — caught cleanly by the whole-cylinder / agreement checks and the
    // gap-b presence guard — rather than a `usize` underflow panic.
    let conn_verts = result
        .vertex_count()
        .saturating_sub(result.shell.vertex_count())
        .saturating_sub(result.lattice.vertex_count())
        .saturating_sub(EXPECTED_CAP_VERTS);
    let conn_tris = result
        .triangle_count()
        .saturating_sub(result.shell.face_count())
        .saturating_sub(result.lattice.face_count())
        .saturating_sub(EXPECTED_CAP_TRIS);
    (conn_verts, conn_tris)
}

fn verify_gap_b_connections(result: &InfillResult) -> Result<()> {
    // gap-b: near-shell lattice nodes each emit one bridging cylinder to
    // their closest point on the inner shell, so the composite carries
    // MORE than shell + lattice + caps. The exact count (30 on this
    // fixture) follows the resolution-dependent closest-point queries on
    // the welded inner shell, so anchor a plausible RANGE rather than the
    // exact number (same anti-rot rationale as the shell Euler invariant):
    //   - lower bound MIN_BRIDGES: a near-empty count signals broken
    //     near-shell detection (a real structural weakness — few
    //     lattice-to-wall bridges), not tessellation drift.
    //   - upper bound `< lattice strut count`: each bridge originates at
    //     a lattice grid NODE, and the cubic lattice has strictly more
    //     struts than nodes, so the strut count is a valid (if loose,
    //     ~2×) ceiling — the example cannot see the internal node count.
    //     A residual at/above the strut count means non-connection
    //     geometry (e.g. a duplicated lattice, which adds exactly one
    //     strut count to the residual) was silently absorbed into the
    //     `mesh − shell − lattice − caps` decomposition.
    const MIN_BRIDGES: usize = 10;
    let base = result.shell.vertex_count() + result.lattice.vertex_count() + EXPECTED_CAP_VERTS;
    ensure!(
        result.vertex_count() > base,
        "gap-b: mesh.vertex_count ({}) should exceed shell + lattice + caps ({base}); no bridging struts injected pre-fix",
        result.vertex_count(),
    );
    let (conn_verts, _) = connection_geometry(result);
    let connections = conn_verts / STRUT_VERTS_PER_CYLINDER;
    let lattice_struts = result.lattice.vertex_count() / STRUT_VERTS_PER_CYLINDER;
    ensure!(
        connections >= MIN_BRIDGES && connections < lattice_struts,
        "gap-b: bridging strut count ({connections}) should be in [{MIN_BRIDGES}, {lattice_struts}) — at least {MIN_BRIDGES} non-trivial bridges, and fewer than the lattice strut count ({lattice_struts}, a loose ceiling since bridges ≤ nodes < struts; a residual at/above it signals duplicated geometry absorbed as fake connections)",
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
    // bbox-heuristic. The hollow-shell WALL volume is the analytical
    // `SIDE³ − 47.6³ = 17 149.824` mm³ (outer minus inner cavity) plus a
    // sub-percent marching-cubes chamfer on the inner offset. Anchored
    // as an invariant band, NOT the old 17-sig-fig exact lock:
    //   1. `0 < shell_volume < SIDE³` — a hollow shell inside the cube
    //      cannot enclose more than the cube. LOAD-BEARING: the §Q-5
    //      inner-wall winding regression made `shell_volume = SIDE³ +
    //      47.6³ = 232 850` mm³ (outer + inner instead of outer −
    //      inner), which this catches. Mirrors the library guard
    //      `mesh_lattice::infill::tests::shell_signed_volume_is_wall_not_sum`.
    //   2. within SHELL_VOLUME_REL_TOL of the analytical wall volume.
    let cube_volume = SIDE * SIDE * SIDE;
    let cavity_edge = SIDE - 2.0 * SHELL_THICKNESS;
    let wall_volume = cube_volume - cavity_edge * cavity_edge * cavity_edge;
    ensure!(
        result.shell_volume > 0.0 && result.shell_volume < cube_volume,
        "gap-d: shell_volume ({}) must be in (0, {cube_volume}) — a hollow shell within the cube cannot exceed it (> cube ⇒ inner-wall winding flipped)",
        result.shell_volume,
    );
    ensure!(
        ((result.shell_volume - wall_volume) / wall_volume).abs() < SHELL_VOLUME_REL_TOL,
        "gap-d: shell_volume ({}) should match the analytical wall SIDE³ − 47.6³ = {wall_volume} within {SHELL_VOLUME_REL_TOL} (rel)",
        result.shell_volume,
    );
    ensure!(
        (result.lattice_volume - EXPECTED_LATTICE_VOLUME).abs() < VOLUME_TOL,
        "lattice_volume = {:.17}, expected {EXPECTED_LATTICE_VOLUME} ± {VOLUME_TOL}",
        result.lattice_volume,
    );

    // total_volume() is DEFINED as shell_volume + lattice_volume
    // (interior_volume is reported separately and is NOT part of it per
    // `InfillResult::total_volume`). Assert the identity against the
    // runtime parts rather than hardcoded constants.
    let expected_total = result.shell_volume + result.lattice_volume;
    ensure!(
        (result.total_volume() - expected_total).abs() < TOTAL_VOLUME_TOL,
        "total_volume = {:.17}, expected shell_volume + lattice_volume = {expected_total:.17}",
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
// PLY outputs — write the four artifacts to the crate `out/`
// =============================================================================

fn write_plys(fixture: &IndexedMesh, result: &InfillResult) -> Result<()> {
    write_one(fixture, INPUT_PLY)?;
    write_one(&result.shell, SHELL_PLY)?;
    write_one(&result.lattice, LATTICE_PLY)?;
    write_one(&result.mesh, COMPOSITE_PLY)?;
    Ok(())
}

fn write_one(mesh: &IndexedMesh, path: &str) -> Result<()> {
    // Resolve under the crate's own directory so artifacts land in
    // `examples/mesh/lattice/stress-test/out/` (covered by the mesh
    // `**/out/` gitignore), consistent with the other four modules —
    // not a CWD-relative `out/` at the workspace root.
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(path);
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
    println!("=== mesh_bounded_infill (post-fix anchors) ===");
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
    let (conn_verts, conn_tris) = connection_geometry(result);
    println!(
        "  - connections       = {} struts × {STRUT_VERTS_PER_CYLINDER}/{STRUT_TRIS_PER_CYLINDER} per cylinder = {conn_verts} verts / {conn_tris} tris",
        conn_verts / STRUT_VERTS_PER_CYLINDER,
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
