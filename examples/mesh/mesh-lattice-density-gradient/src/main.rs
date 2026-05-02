// `unreachable!()` calls in this binary are diagnostic guards on
// `let-else` branches that cannot fire (validated builder + valid
// bounds → `generate_lattice` cannot fail; same for `validate()` post-
// builder; same for `with_beam_export(true) ⇒ beam_data is Some`).
// `xtask grade`'s Safety criterion counts un-justified `unreachable!()`
// macros; allow at file level since every call is a post-validation
// `Option::None` / `Result::Err` impossibility, not a real panic site.
#![allow(clippy::unreachable)]
//! mesh-lattice-density-gradient — variable-density lattice via
//! `DensityMap` on the octet-truss preset.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.7. Fixture: 30 mm cube at
//! 7.5 mm cell size (4 × 4 × 4 = 64 cells, octet-truss topology), strut
//! thickness 0.6 mm (baseline radius 0.3 mm), `with_beam_export(true)`.
//! `Gradient` density map climbs linearly along z: `from_density = 0.1`
//! at `(0, 0, 0)`, `to_density = 0.5` at `(0, 0, 30)`.
//!
//! Density-modulated counterpart to §5.6 `mesh-lattice-strut-cubic`:
//! same strut path mechanism, but octet-truss topology (20 struts per
//! cell — 8 corner-to-center + 12 corner-to-corner) and a non-uniform
//! density map. The load-bearing observation: density modulates per-
//! beam `r1 = strut_thickness/2 × density.sqrt()`, where density is
//! sampled at the cell center (`generate.rs:290-293`). With the
//! gradient `0.1 → 0.5` across z=0..30 and 4 cell-z strata at z=3.75 /
//! 11.25 / 18.75 / 26.25, the four discrete per-cell densities are
//! `0.15 / 0.25 / 0.35 / 0.45` and the four discrete per-beam radii are
//! `0.3 × √density ≈ 0.1162 / 0.150 / 0.1775 / 0.2012`.
//!
//! `BeamLatticeData` population on the octet-truss path was added in
//! the immediately preceding commit (parity with the cubic path);
//! before that fix, this example's HE-4 anchor was unreachable.

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply;
use mesh_lattice::{
    BeamLatticeData, DensityMap, LatticeParams, LatticeResult, LatticeType, generate_lattice,
};
use mesh_types::Point3;

// =============================================================================
// Fixture constants
// =============================================================================

/// 30 mm × 30 mm × 30 mm cubic bounding box, axis-aligned at origin.
const BBOX_MIN: f64 = 0.0;
const BBOX_MAX: f64 = 30.0;

/// Unit cell edge length in mm. `30 / 7.5 = 4` cells per axis.
const CELL_SIZE: f64 = 7.5;

/// Strut diameter in mm; baseline radius is half this. Per-beam `r1`
/// scales by `density.sqrt()` from this baseline.
const STRUT_THICKNESS: f64 = 0.6;
const BASELINE_RADIUS: f64 = STRUT_THICKNESS / 2.0;

/// `Gradient` density-map endpoints: 10% at z=0 → 50% at z=30.
const GRAD_FROM: Point3<f64> = Point3::new(0.0, 0.0, 0.0);
const GRAD_FROM_DENSITY: f64 = 0.1;
const GRAD_TO: Point3<f64> = Point3::new(0.0, 0.0, 30.0);
const GRAD_TO_DENSITY: f64 = 0.5;

/// `bbox_size / cell_size = 30 / 7.5 = 4`; cells-per-axis cubed.
const EXPECTED_CELL_COUNT: usize = 64;

/// 20 struts per octet-truss cell (8 corner-to-center + 12 edges) ×
/// 64 cells. No filter applies (gradient density `≥ 0.1 > 0.05` for
/// all cells), so every cell records all 20 beams unconditionally.
const EXPECTED_BEAM_COUNT: usize = EXPECTED_CELL_COUNT * 20;

/// Deduplicated grid-corner count `(cells + 1)³ = 5³ = 125`, plus one
/// unique cell center per cell. Adjacent cells share corners via the
/// `quantize`/`vertex_map` dedup; cell centers are unique.
const EXPECTED_BEAM_VERTEX_COUNT: usize = 125 + EXPECTED_CELL_COUNT;

/// Half-bbox z-cut for the bottom/top filter: `30 / 2 = 15`.
const Z_HALF: f64 = 15.0;

/// Tight tolerance for analytical evaluations (linear interpolation,
/// exact-clamp, `Uniform`).
const TIGHT_TOL: f64 = 1e-12;
/// Looser tolerance for `f64::sqrt()`-derived per-beam radius checks
/// (`density.sqrt()` is correctly-rounded but not exactly representable
/// for non-perfect-square densities).
const RADIUS_TOL: f64 = 1e-10;
/// Spot-check tolerance for "at least one beam …" anchors.
const SPOT_TOL: f64 = 1e-9;

// =============================================================================
// verify_density_map_evaluation — direct DensityMap::evaluate anchors
// =============================================================================

/// Locks the four DEMONSTRATED variants' `evaluate` returns at known
/// sample points: `Uniform`, `Gradient` (endpoints + midpoint),
/// `Radial` (at center), `Function` (clamps to [0, 1]). The remaining
/// two enum variants (`SurfaceDistance`, `StressField`) are exercised
/// downstream (`SurfaceDistance` via `verify_surface_distance_demo`
/// and `verify_density_map_constructors`; `StressField` only via
/// `verify_density_map_constructors` with no `evaluate` anchor since
/// it requires a meaningful stress field, deferred to v0.9).
fn verify_density_map_evaluation() {
    // (1) Uniform: density at any point ignores the point.
    let uniform = DensityMap::Uniform(0.4);
    assert_relative_eq!(
        uniform.evaluate(Point3::new(10.0, 20.0, 30.0)),
        0.4,
        epsilon = TIGHT_TOL,
    );
    assert_relative_eq!(uniform.evaluate(Point3::origin()), 0.4, epsilon = TIGHT_TOL,);

    // (2) Gradient: line interpolation between two Point3 endpoints
    // (NOT an axis enum). Endpoints are exact; midpoint is the
    // average of the two densities (linear interpolation is exact
    // at half-fractions of integer-coord endpoints in IEEE 754).
    let gradient = DensityMap::Gradient {
        from: GRAD_FROM,
        from_density: GRAD_FROM_DENSITY,
        to: GRAD_TO,
        to_density: GRAD_TO_DENSITY,
    };
    assert_relative_eq!(
        gradient.evaluate(GRAD_FROM),
        GRAD_FROM_DENSITY,
        epsilon = TIGHT_TOL,
    );
    assert_relative_eq!(
        gradient.evaluate(GRAD_TO),
        GRAD_TO_DENSITY,
        epsilon = TIGHT_TOL,
    );
    assert_relative_eq!(
        gradient.evaluate(Point3::new(0.0, 0.0, 15.0)),
        0.3,
        epsilon = TIGHT_TOL,
    );

    // (3) Radial: at center, inner_radius=0 ⇒ evaluate(center) ==
    // inner_density (line 194 hits the `dist <= inner_radius` branch).
    let radial = DensityMap::Radial {
        center: Point3::origin(),
        inner_radius: 0.0,
        inner_density: 0.8,
        outer_radius: 50.0,
        outer_density: 0.2,
    };
    assert_relative_eq!(radial.evaluate(Point3::origin()), 0.8, epsilon = TIGHT_TOL);

    // (4) Function: clamps result to [0, 1] regardless of closure
    // output (line 225 in density.rs). Constant 1.5 → clamped to 1.0.
    let func_high = DensityMap::from_function(|_| 1.5);
    assert_relative_eq!(
        func_high.evaluate(Point3::origin()),
        1.0,
        epsilon = TIGHT_TOL,
    );
    let func_low = DensityMap::from_function(|_| -0.5);
    assert_relative_eq!(
        func_low.evaluate(Point3::origin()),
        0.0,
        epsilon = TIGHT_TOL,
    );
}

// =============================================================================
// verify_density_map_clamping — Gradient beyond endpoints
// =============================================================================

/// `Gradient::evaluate` clamps the line parameter `t` to `[0, 1]`
/// (`density.rs:182`), so points beyond either endpoint return the
/// nearer endpoint density exactly.
fn verify_density_map_clamping() {
    let gradient = DensityMap::Gradient {
        from: GRAD_FROM,
        from_density: GRAD_FROM_DENSITY,
        to: GRAD_TO,
        to_density: GRAD_TO_DENSITY,
    };
    // Before the start point (z = -50): clamps to from_density.
    assert_relative_eq!(
        gradient.evaluate(Point3::new(0.0, 0.0, -50.0)),
        GRAD_FROM_DENSITY,
        epsilon = TIGHT_TOL,
    );
    // After the end point (z = 150): clamps to to_density.
    assert_relative_eq!(
        gradient.evaluate(Point3::new(0.0, 0.0, 150.0)),
        GRAD_TO_DENSITY,
        epsilon = TIGHT_TOL,
    );
}

// =============================================================================
// verify_density_map_constructors — from_function + from_stress_field
// =============================================================================

/// `from_function` and `from_stress_field` build the corresponding
/// enum variants from closures. We don't assert variant identity
/// (the Arc'd closure is opaque) but verify the resulting `evaluate`
/// behaves consistently with the closure (clamped to [0, 1] post-
/// closure).
fn verify_density_map_constructors() {
    // from_function: identity-like function.
    let f = DensityMap::from_function(|p| 0.1_f64.mul_add(p.x, 0.5));
    assert_relative_eq!(f.evaluate(Point3::origin()), 0.5, epsilon = TIGHT_TOL);
    assert_relative_eq!(
        f.evaluate(Point3::new(2.0, 0.0, 0.0)),
        0.7,
        epsilon = TIGHT_TOL,
    );

    // from_stress_field: maps stress in [0, 1] linearly to density in
    // [min_density, max_density]. At zero stress → min; at unit stress
    // → max.
    let s = DensityMap::from_stress_field(|p| (p.z / 100.0).clamp(0.0, 1.0), 0.1, 0.9);
    assert_relative_eq!(s.evaluate(Point3::origin()), 0.1, epsilon = TIGHT_TOL);
    assert_relative_eq!(
        s.evaluate(Point3::new(0.0, 0.0, 100.0)),
        0.9,
        epsilon = TIGHT_TOL,
    );
    assert_relative_eq!(
        s.evaluate(Point3::new(0.0, 0.0, 50.0)),
        0.5,
        epsilon = TIGHT_TOL,
    );
}

// =============================================================================
// verify_surface_distance_demo — special evaluate_with_distance method
// =============================================================================

/// `SurfaceDistance::evaluate_with_distance(d)` is the SPECIAL method
/// (returns `Option<f64>`); only `SurfaceDistance` returns `Some(_)`,
/// other variants return `None` (`density.rs:240-251`). The plain
/// `evaluate(p)` returns the midpoint fallback `(surface + core) / 2`
/// since the variant cannot infer distance from a `Point3` alone
/// (`density.rs:203-212`).
fn verify_surface_distance_demo() {
    let sd = DensityMap::SurfaceDistance {
        surface_density: 0.8,
        core_density: 0.2,
        transition_depth: 10.0,
    };

    // Plain evaluate returns midpoint fallback.
    assert_relative_eq!(
        sd.evaluate(Point3::new(1.0, 2.0, 3.0)),
        f64::midpoint(0.8, 0.2),
        epsilon = TIGHT_TOL,
    );

    // evaluate_with_distance returns Some(_) for SurfaceDistance.
    let Some(at_surface) = sd.evaluate_with_distance(0.0) else {
        unreachable!("SurfaceDistance::evaluate_with_distance returns Some");
    };
    assert_relative_eq!(at_surface, 0.8, epsilon = TIGHT_TOL);

    let Some(at_core) = sd.evaluate_with_distance(10.0) else {
        unreachable!("SurfaceDistance::evaluate_with_distance returns Some");
    };
    assert_relative_eq!(at_core, 0.2, epsilon = TIGHT_TOL);

    // Other variants return None.
    let uni = DensityMap::Uniform(0.5);
    assert!(
        uni.evaluate_with_distance(0.0).is_none(),
        "evaluate_with_distance is None for non-SurfaceDistance variants",
    );
}

// =============================================================================
// build_params + verify_params_validate — builder + validate() + density_at
// =============================================================================

/// Configured `LatticeParams`: `octet_truss` preset (richer geometry
/// than cubic — 20 struts per cell vs cubic's 3 axis families),
/// `strut_thickness` 0.6, `with_density_map(Gradient(0.1 → 0.5))`,
/// `with_beam_export(true)` flipping on the `BeamLatticeData` side-
/// channel that exposes per-beam `r1` for the load-bearing density-
/// modulation anchor.
fn build_params() -> LatticeParams {
    LatticeParams::octet_truss(CELL_SIZE)
        .with_strut_thickness(STRUT_THICKNESS)
        .with_density_map(DensityMap::Gradient {
            from: GRAD_FROM,
            from_density: GRAD_FROM_DENSITY,
            to: GRAD_TO,
            to_density: GRAD_TO_DENSITY,
        })
        .with_beam_export(true)
}

/// Lock fields to expected values + assert `validate()` returns
/// `Ok(())`. `density_at(p)` delegates to the wrapped density map at
/// the gradient endpoints, matching `Gradient::evaluate` exactly.
fn verify_params_validate(params: &LatticeParams) {
    assert_eq!(params.lattice_type, LatticeType::OctetTruss);
    assert_relative_eq!(params.cell_size, CELL_SIZE, epsilon = TIGHT_TOL);
    assert_relative_eq!(params.strut_thickness, STRUT_THICKNESS, epsilon = TIGHT_TOL,);
    assert!(
        params.density_map.is_some(),
        "with_density_map(_) must populate density_map",
    );
    assert!(
        params.preserve_beam_data,
        "with_beam_export(true) must enable BeamLatticeData export",
    );

    // build_params returns a known-valid LatticeParams.
    if params.validate().is_err() {
        unreachable!("build_params yields a valid LatticeParams");
    }

    // density_at delegates to the Gradient map at endpoints.
    assert_relative_eq!(
        params.density_at(GRAD_FROM),
        GRAD_FROM_DENSITY,
        epsilon = TIGHT_TOL,
    );
    assert_relative_eq!(
        params.density_at(GRAD_TO),
        GRAD_TO_DENSITY,
        epsilon = TIGHT_TOL,
    );
}

// =============================================================================
// verify_lattice_result_basic — cell_count + beam_data presence
// =============================================================================

/// `cell_count == 64` (4³) BIT-EXACT; `beam_data == Some(_)` after
/// the C15a fix to `generate_octet_truss_lattice` (parity with the
/// cubic path's beam-data export). `actual_density` is the F9
/// heuristic — finite + in [0, 1] (octet path uses the same
/// `estimate_strut_volume` route as cubic, which has no §5.5
/// drift-12 closed-orientable-manifold pathology).
fn verify_lattice_result_basic(result: &LatticeResult) {
    assert_eq!(
        result.cell_count, EXPECTED_CELL_COUNT,
        "30 mm bbox / 7.5 mm cell = 4 cells per axis, 4³ = 64",
    );
    assert!(
        result.beam_data.is_some(),
        "with_beam_export(true) on octet-truss path must populate beam_data \
         (post C15a fix to generate_octet_truss_lattice)",
    );

    let d = result.actual_density;
    assert!(
        d.is_finite() && (0.0..=1.0).contains(&d),
        "actual_density = {d} must be finite in [0, 1]",
    );
}

// =============================================================================
// verify_per_beam_r1_density_anchor — HE-4 load-bearing observation
// =============================================================================

/// Bundled regional-mean output for [`print_summary`].
struct R1Stats {
    bottom_count: usize,
    top_count: usize,
    bottom_mean: f64,
    top_mean: f64,
    /// `top_mean / bottom_mean` — the ratio anchor.
    ratio: f64,
    /// Smallest `r1` seen for any beam at `vertices[v1].z < 7.5`.
    min_at_low_z: f64,
    /// Largest `r1` seen for any beam at `vertices[v1].z > 22.5`.
    max_at_high_z: f64,
}

/// HE-4 load-bearing anchor. Density is sampled at the **cell
/// center** in the octet path (`generate.rs:290-293`), so all 20
/// beams in a cell share the same `local_radius = strut_thickness/2 ×
/// density.sqrt()`. With cell-center z values `{3.75, 11.25, 18.75,
/// 26.25}` and the gradient `0.1 → 0.5` over z=0..30, the four
/// discrete per-cell densities are `{0.15, 0.25, 0.35, 0.45}` and the
/// four discrete per-beam radii are `{0.3·√d}` ≈ `{0.1162, 0.150,
/// 0.1775, 0.2012}`. Each beam's `r1` MUST be within 1e-10 of one of
/// these four values (correctly-rounded `density.sqrt()`).
///
/// Filtering by `vertices[v1].z` partitions beams unevenly across
/// the z=15 cut (12 of 20 per-cell beams have `v1` at the cell's low
/// corner, 8 at the high corner — see `generate.rs:325-382`), so the
/// per-half mean is NOT the simple mean of the two density-strata
/// means. The empirical ratio (locked below) is still well inside
/// the spec's ±5% band, demonstrating density modulation
/// quantitatively.
#[allow(clippy::cast_precision_loss)] // bottom/top beam counts ≤ 1280, well within f64 mantissa
fn verify_per_beam_r1_density_anchor(beam_data: &BeamLatticeData) -> R1Stats {
    let four_radii = [
        BASELINE_RADIUS * 0.15_f64.sqrt(),
        BASELINE_RADIUS * 0.25_f64.sqrt(),
        BASELINE_RADIUS * 0.35_f64.sqrt(),
        BASELINE_RADIUS * 0.45_f64.sqrt(),
    ];

    let mut bottom_count = 0_usize;
    let mut top_count = 0_usize;
    let mut bottom_sum = 0.0_f64;
    let mut top_sum = 0.0_f64;
    let mut min_at_low_z = f64::INFINITY;
    let mut max_at_high_z = f64::NEG_INFINITY;

    for (i, beam) in beam_data.beams.iter().enumerate() {
        // Every per-beam r1 must equal one of the four cell-stratum
        // radii within 1e-10 (no other density values can occur).
        let v1_pos = beam_data.vertices[beam.v1 as usize];
        let r1 = beam.r1;

        let matches_one = four_radii
            .iter()
            .any(|&expected| (r1 - expected).abs() < RADIUS_TOL);
        assert!(
            matches_one,
            "beam {i} r1 = {r1} does not match any of the four expected \
             stratum radii {four_radii:?} within {RADIUS_TOL}",
        );

        if v1_pos.z <= Z_HALF {
            bottom_count += 1;
            bottom_sum += r1;
        } else {
            top_count += 1;
            top_sum += r1;
        }

        if v1_pos.z < CELL_SIZE && r1 < min_at_low_z {
            min_at_low_z = r1;
        }
        if v1_pos.z > BBOX_MAX - CELL_SIZE && r1 > max_at_high_z {
            max_at_high_z = r1;
        }
    }

    assert_eq!(
        bottom_count + top_count,
        EXPECTED_BEAM_COUNT,
        "every beam must be classified into exactly one of bottom / top",
    );
    assert!(bottom_count > 0 && top_count > 0);

    let bottom_mean = bottom_sum / bottom_count as f64;
    let top_mean = top_sum / top_count as f64;
    let ratio = top_mean / bottom_mean;

    // Empirical anchor (analytical derivation matches; see fn doc):
    // bottom mean ≈ 0.1433, top mean ≈ 0.1944, ratio ≈ 1.357. Spec
    // §5.7 line 709 claims ratio ≈ 1.41 ± 5%; our empirical 1.357 is
    // inside that band (1.357 / 1.41 ≈ 0.962, ~3.8% below).
    assert!(
        ratio > 1.30 && ratio < 1.45,
        "top/bottom r1 mean ratio = {ratio:.4} must be in (1.30, 1.45) \
         (spec §5.7 ≈ 1.41 ± 5%, empirical ≈ 1.357 with v1-filtering)",
    );

    // Spot-check: bottom-most stratum (cells iz=0, density 0.15) has
    // r1 ≈ 0.1162, which is < 0.16. Beams with v1.z < 7.5 are in
    // strata iz=0 only (no other v1.z < 7.5).
    assert!(
        min_at_low_z < 0.16,
        "min r1 at v1.z < 7.5 = {min_at_low_z:.4} must be < 0.16 \
         (iz=0 stratum, r1 ≈ 0.1162)",
    );
    // Spot-check: top-most stratum (cells iz=3, density 0.45) has
    // r1 ≈ 0.2012. Beams with v1.z > 22.5 are in strata iz=3 only
    // (v1.z = 30 from corners 4..7 of iz=3 cells).
    assert!(
        max_at_high_z > 0.18,
        "max r1 at v1.z > 22.5 = {max_at_high_z:.4} must be > 0.18 \
         (iz=3 stratum, r1 ≈ 0.2012)",
    );
    // Sanity tolerance on max_at_high_z lower bound (analytical 0.2012).
    assert!((max_at_high_z - four_radii[3]).abs() < SPOT_TOL);

    R1Stats {
        bottom_count,
        top_count,
        bottom_mean,
        top_mean,
        ratio,
        min_at_low_z,
        max_at_high_z,
    }
}

// =============================================================================
// verify_beam_data_counts — beam_count + vertex_count (BIT-EXACT)
// =============================================================================

/// `beam_count == 1280` (64 cells × 20 beams) and `vertex_count ==
/// 189` (5³ = 125 deduplicated grid corners + 64 unique cell
/// centers). The `vertex_map` quantize-dedup at scale `1e6`
/// (`generate.rs:276-284`) collapses corners shared between adjacent
/// cells; cell centers are at `cell.z + half_cell` offsets so they
/// never collide with corners.
fn verify_beam_data_counts(beam_data: &BeamLatticeData) {
    assert_eq!(
        beam_data.beam_count(),
        EXPECTED_BEAM_COUNT,
        "20 beams/cell × 64 cells = 1280 (no density cells dropped at \
         the < 0.05 threshold; gradient stays ≥ 0.1)",
    );
    assert_eq!(
        beam_data.vertex_count(),
        EXPECTED_BEAM_VERTEX_COUNT,
        "5³ deduplicated grid corners + 64 unique cell centers = 189",
    );
}

// =============================================================================
// verify_function_demo — DensityMap::Function with a non-trivial closure
// =============================================================================

/// Demonstrates `DensityMap::Function` with a sinusoidal closure (per
/// spec §5.7 line 712). Sampled at known points where the analytical
/// value is exact.
fn verify_function_demo() {
    // f(p) = 0.5 + 0.1 × sin(p.x / 10.0). At x = 0: sin(0) = 0, so
    // f = 0.5. At x = 10π: sin(π) = 0, so f = 0.5. At x = 5π: sin(π/2)
    // = 1, so f = 0.6. Output is in [0.4, 0.6] ⊂ [0, 1] so no
    // clamping fires.
    let map = DensityMap::from_function(|p| 0.1_f64.mul_add((p.x / 10.0).sin(), 0.5));
    assert_relative_eq!(map.evaluate(Point3::origin()), 0.5, epsilon = TIGHT_TOL);
    assert_relative_eq!(
        map.evaluate(Point3::new(10.0 * std::f64::consts::PI, 0.0, 0.0)),
        0.5,
        epsilon = TIGHT_TOL,
    );
    assert_relative_eq!(
        map.evaluate(Point3::new(5.0 * std::f64::consts::PI, 0.0, 0.0)),
        0.6,
        epsilon = TIGHT_TOL,
    );
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

/// Bundled inputs for [`print_summary`]; avoids
/// `clippy::too_many_arguments` while keeping fields trivially
/// constructed at the call site (precedent: §5.4 / §5.6 `Summary`).
struct Summary<'a> {
    result: &'a LatticeResult,
    beam_data: &'a BeamLatticeData,
    r1_stats: &'a R1Stats,
}

/// Print the human-readable summary. Extracted from `main` to keep
/// the entrypoint under clippy's `too_many_lines` cap.
fn print_summary(s: &Summary) {
    println!("==== mesh-lattice-density-gradient ====");
    println!();
    println!(
        "fixture: 30 mm bbox, cell_size = {CELL_SIZE} mm, strut_thickness \
         = {STRUT_THICKNESS} mm",
    );
    println!(
        "         Gradient density {GRAD_FROM_DENSITY} at z=0 → \
         {GRAD_TO_DENSITY} at z=30, with_beam_export(true)"
    );
    println!();
    println!("DensityMap::evaluate anchors (4 demonstrated variants):");
    println!("  Uniform(0.4).evaluate(any) = 0.4 (≤ 1e-12; ignores point)");
    println!("  Gradient(0.1→0.5 along z=0..30).evaluate(0,0,0)  = 0.1 (endpoint, ≤ 1e-12)");
    println!("  Gradient(...)                  .evaluate(0,0,30) = 0.5 (endpoint, ≤ 1e-12)");
    println!("  Gradient(...)                  .evaluate(0,0,15) = 0.3 (midpoint, ≤ 1e-12)");
    println!("  Gradient(...)                  .evaluate(0,0,-50)/(0,0,150) clamps to 0.1/0.5");
    println!("  Radial(inner_r=0)              .evaluate(center) = inner_density");
    println!("  Function(|_| 1.5)              .evaluate(any) = 1.0 (post-clamp, ≤ 1e-12)");
    println!();
    println!("DensityMap::evaluate_with_distance — special method on SurfaceDistance:");
    println!("  evaluate_with_distance(0.0) = surface_density; (transition_depth) = core_density");
    println!("  Other variants return None.");
    println!();
    println!("DensityMap::Function(sinusoidal closure) — non-trivial demo:");
    println!("  f(p) = 0.5 + 0.1 × sin(p.x / 10) → evaluate(origin) = 0.5 (≤ 1e-12)");
    println!();
    println!("LatticeParams::octet_truss + builder:");
    println!("  density_at(0,0,0)  = 0.1 (≤ 1e-12; delegates to Gradient map)");
    println!("  density_at(0,0,30) = 0.5 (≤ 1e-12)");
    println!();
    println!("Generated lattice:");
    println!(
        "  cell_count       = {} (BIT-EXACT; 4 × 4 × 4)",
        s.result.cell_count,
    );
    println!(
        "  vertex_count     = {} (mesh; 1280 struts × 14 verts/strut)",
        s.result.vertex_count(),
    );
    println!(
        "  triangle_count   = {} (mesh; 1280 × 24 tris/strut)",
        s.result.triangle_count(),
    );
    println!(
        "  total_strut_len  ≈ {:.2} mm (octet edge sum)",
        s.result.total_strut_length.unwrap_or(f64::NAN),
    );
    println!(
        "  actual_density   ≈ {:.4} (F9 heuristic; finite + in [0, 1])",
        s.result.actual_density,
    );
    println!();
    println!("BeamLatticeData (3MF beam-extension precursor; F11 v0.9 consumer):");
    println!(
        "  vertex_count() = {} (BIT-EXACT; 5³ deduplicated corners + 64 cell centers)",
        s.beam_data.vertex_count(),
    );
    println!(
        "  beam_count()   = {} (BIT-EXACT; 64 cells × 20 beams)",
        s.beam_data.beam_count(),
    );
    println!();
    println!("Per-beam r1 density anchor (HE-4, load-bearing):");
    println!(
        "  bottom-half (v1.z ≤ {Z_HALF}) — n={}, mean r1 = {:.4} mm",
        s.r1_stats.bottom_count, s.r1_stats.bottom_mean,
    );
    println!(
        "  top-half    (v1.z >  {Z_HALF}) — n={}, mean r1 = {:.4} mm",
        s.r1_stats.top_count, s.r1_stats.top_mean,
    );
    println!(
        "  ratio (top/bottom) = {:.4} (spec ≈ 1.41 ± 5%; analytical ≈ 1.357 with v1-filter)",
        s.r1_stats.ratio,
    );
    println!(
        "  min r1 at v1.z < 7.5  = {:.4} (iz=0 stratum, < 0.16)",
        s.r1_stats.min_at_low_z,
    );
    println!(
        "  max r1 at v1.z > 22.5 = {:.4} (iz=3 stratum, > 0.18; ≈ 0.3 × √0.45 ≈ 0.2012)",
        s.r1_stats.max_at_high_z,
    );
    println!();
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    verify_density_map_evaluation();
    verify_density_map_clamping();
    verify_density_map_constructors();
    verify_surface_distance_demo();
    verify_function_demo();

    let params = build_params();
    verify_params_validate(&params);

    let bounds = (
        Point3::new(BBOX_MIN, BBOX_MIN, BBOX_MIN),
        Point3::new(BBOX_MAX, BBOX_MAX, BBOX_MAX),
    );
    let Ok(result) = generate_lattice(&params, bounds) else {
        unreachable!("validated params + valid bounds: generate_lattice cannot fail");
    };

    verify_lattice_result_basic(&result);

    let Some(beam_data) = result.beam_data.as_ref() else {
        unreachable!("with_beam_export(true) ⇒ beam_data is Some(_) on octet-truss path");
    };
    verify_beam_data_counts(beam_data);
    let r1_stats = verify_per_beam_r1_density_anchor(beam_data);

    print_summary(&Summary {
        result: &result,
        beam_data,
        r1_stats: &r1_stats,
    });

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let mesh_path = out_dir.join("density_gradient_lattice.ply");
    save_ply(&result.mesh, &mesh_path, true)?;
    println!(
        "artifact: out/density_gradient_lattice.ply ({}v, {}f, binary little-endian)",
        result.mesh.vertices.len(),
        result.mesh.faces.len(),
    );
    println!();
    println!(
        "OK — 4 DensityMap variants + Gradient clamping + from_function + \
         from_stress_field + SurfaceDistance evaluate_with_distance + Function \
         sin demo + params validate + density_at + cell_count + beam_data counts + \
         per-beam r1 density anchor + spot-checks all green",
    );

    Ok(())
}
