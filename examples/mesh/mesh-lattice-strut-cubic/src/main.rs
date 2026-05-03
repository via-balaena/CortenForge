// `unreachable!()` calls in this binary are diagnostic guards on
// `let-else` branches that cannot fire (validated builder + valid
// bounds → `generate_lattice` cannot fail; same for `validate()` post-
// builder; same for non-degenerate strut/beam emission paths).
// `xtask grade`'s Safety criterion counts un-justified `unreachable!()`
// macros; allow at file level since every call is a post-validation
// `Option::None` / `Result::Err` impossibility, not a real panic site.
#![allow(clippy::unreachable)]
//! mesh-lattice-strut-cubic — cubic strut lattice generation +
//! the 3MF beam-data export precursor.
//!
//! Fixture: 25 mm cube at
//! 5 mm cell size (5 × 5 × 5 = 125 cells), strut thickness 1.0 mm
//! (radius 0.5 mm), uniform density 1.0 (so `density.sqrt() == 1.0`
//! and per-beam `r1 == r2 == 0.5` exactly), `with_beam_export(true)`.
//!
//! Strut counterpart to `mesh-lattice-tpms-gyroid`: cylindrical
//! beams between integer-spaced grid nodes — NOT a TPMS isosurface.
//! That topology unlocks combinatorial + bit-exact anchors that the
//! marching-cubes path cannot offer:
//!
//! - `STRUT_SEGMENTS = 6` per `mesh-lattice/src/strut.rs:14` ⇒ each
//!   strut emits exactly 14 vertices (2 cap centers + 6 × 2 ring) and
//!   24 triangles (6 sides × 2 + 12 caps).
//! - `combine_struts` does NOT weld inter-strut nodes — the 540
//!   struts of the 5 × 5 × 5 cubic lattice contribute `540 × 14 =
//!   7560` mesh vertices and `540 × 24 = 12960` mesh triangles
//!   BIT-EXACT (each grid node appears in up to 6 struts).
//! - `total_strut_length` accumulates `+= cell_size = 5.0` 540 times
//!   per `generate.rs:156/183/209`; the running sum is `[5, 10, 15,
//!   ..., 2700]`, all exact integers in f64 below 2⁵³ ⇒ final
//!   `Some(2700.0)` BIT-EXACT.
//!
//! `BeamLatticeData` (the 3MF beam-lattice extension's in-memory
//! shape) is dedup'd via the `quantize` `HashMap` pattern at scale
//! `1e6` (`generate.rs:111-119`), collapsing the 540 × 2 strut-end
//! references into the `(cells + 1)³ = 6³ = 216` unique grid nodes.
//! Each axis-aligned beam reports `length = sqrt(25) = 5.0`
//! BIT-EXACT (perfect square), so `data.total_length()` sums to
//! `540 × 5.0 = 2700.0` BIT-EXACT in any order.
//!
//! F11 (3MF beam writer) is a v0.9 candidate this example pre-stages:
//! v1.0 stops at the populated `BeamLatticeData`; the v0.9 writer
//! will emit it as `<beamlattice>` / `<beams>` / `<beam>` blocks.
//! See `mesh-io/CHANGELOG.md` for the v0.9 candidate entry.

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply;
use mesh_lattice::{
    BeamCap, BeamLatticeData, LatticeParams, LatticeResult, LatticeType, combine_struts,
    estimate_strut_volume, generate_lattice, generate_strut, generate_strut_tapered,
};
use mesh_types::Point3;

// =============================================================================
// Fixture constants
// =============================================================================

/// 25 mm × 25 mm × 25 mm cubic bounding box, axis-aligned at origin.
const BBOX_MIN: f64 = 0.0;
const BBOX_MAX: f64 = 25.0;

/// Unit cell edge length in mm. `25 / 5 = 5` cells per axis.
const CELL_SIZE: f64 = 5.0;

/// Cylindrical strut diameter in mm; per-beam radius is half this.
const STRUT_THICKNESS: f64 = 1.0;
const RADIUS: f64 = 0.5;

/// Uniform full-density grid; `density.sqrt() == 1.0`, so per-beam
/// `r1 == r2 == STRUT_THICKNESS / 2 == 0.5` exactly.
const DENSITY: f64 = 1.0;

/// `bbox_size / cell_size = 25 / 5 = 5`; cells-per-axis cubed.
const EXPECTED_CELL_COUNT: usize = 125;

/// Strut count: 3 axis-families × 5 in-axis × 6 × 6 perpendicular.
const EXPECTED_STRUT_COUNT: usize = 540;
/// Per-strut vertex count from `STRUT_SEGMENTS = 6`.
const VERTS_PER_STRUT: usize = 14;
/// Per-strut triangle count: `6 sides × 2 + 12 caps`.
const TRIS_PER_STRUT: usize = 24;
/// `combine_struts` does NOT weld; mesh-vertex count is per-strut × strut count.
const EXPECTED_VERTEX_COUNT: usize = EXPECTED_STRUT_COUNT * VERTS_PER_STRUT;
/// Same for triangles — per-strut × strut count.
const EXPECTED_TRIANGLE_COUNT: usize = EXPECTED_STRUT_COUNT * TRIS_PER_STRUT;

/// Deduplicated grid-node count: `(cells + 1)³ = 6³`.
const EXPECTED_BEAM_VERTEX_COUNT: usize = 216;
/// Edges of the cubic lattice — same as strut count.
const EXPECTED_BEAM_COUNT: usize = EXPECTED_STRUT_COUNT;
/// Sum of all 540 axis-aligned beam lengths: `540 × 5.0` BIT-EXACT.
const EXPECTED_TOTAL_LENGTH: f64 = 2700.0;

/// Free-fn `estimate_strut_volume` tolerance (analytical π formulas).
const STRUT_VOL_TOL: f64 = 1e-10;
/// Beam-radius tolerance: `density = 1.0` ⇒ `density.sqrt() = 1.0` exactly.
const TIGHT_TOL: f64 = 1e-12;
/// `data.total_length()` and per-beam length tolerance (sums of exact
/// integer-spaced norms; bit-exact in practice but spec asks 1e-9).
const LENGTH_TOL: f64 = 1e-9;
/// `data.estimate_volume()` cylinder-sum check is approximate
/// (tapered-cone formula on uniform-radius beams ⇒ same as cylinder).
const VOLUME_LOWER: f64 = 2100.0;
const VOLUME_UPPER: f64 = 2150.0;

// =============================================================================
// verify_strut_free_fns — direct generate_strut / combine_struts anchors
// =============================================================================

/// Locks the per-strut combinatorial counts BEFORE invoking
/// `generate_lattice`, so any drift in `STRUT_SEGMENTS` or the
/// cap-fan layout fails here at the unit level rather than via the
/// 540-strut aggregate. Also locks the zero-length `None` branch
/// and `combine_struts` linearity (no welding).
fn verify_strut_free_fns() {
    // (1) generate_strut: 14 verts + 24 tris.
    let Some(strut) = generate_strut(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 10.0),
        RADIUS,
    ) else {
        unreachable!("non-degenerate strut must produce a mesh");
    };
    assert_eq!(
        strut.vertex_count(),
        VERTS_PER_STRUT,
        "STRUT_SEGMENTS=6 ⇒ 2 cap centers + 6 × 2 ring = 14 verts",
    );
    assert_eq!(
        strut.face_count(),
        TRIS_PER_STRUT,
        "STRUT_SEGMENTS=6 ⇒ 6 sides × 2 + 12 caps = 24 tris",
    );

    // (2) generate_strut_tapered: same vert/tri count for a tapered strut.
    let Some(tapered) = generate_strut_tapered(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 10.0),
        RADIUS,
        RADIUS / 2.0,
    ) else {
        unreachable!("non-degenerate tapered strut must produce a mesh");
    };
    assert_eq!(tapered.vertex_count(), VERTS_PER_STRUT);
    assert_eq!(tapered.face_count(), TRIS_PER_STRUT);

    // (3) Zero-length strut → None.
    let degenerate = generate_strut(
        Point3::new(1.0, 2.0, 3.0),
        Point3::new(1.0, 2.0, 3.0),
        RADIUS,
    );
    assert!(
        degenerate.is_none(),
        "zero-length strut must return None (length < f64::EPSILON branch)",
    );

    // (4) combine_struts linearity: no inter-strut welding.
    let n = 3;
    let struts = vec![
        generate_strut(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), 0.1),
        generate_strut(Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0), 0.1),
        generate_strut(Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 1.0), 0.1),
    ];
    let combined = combine_struts(struts.into_iter().flatten());
    assert_eq!(
        combined.vertex_count(),
        n * VERTS_PER_STRUT,
        "combine_struts must NOT weld; verts = N × {VERTS_PER_STRUT}",
    );
    assert_eq!(
        combined.face_count(),
        n * TRIS_PER_STRUT,
        "combine_struts must NOT weld; tris = N × {TRIS_PER_STRUT}",
    );
}

// =============================================================================
// verify_estimate_strut_volume — cylinder + cone analytical anchors
// =============================================================================

/// `estimate_strut_volume(length, r1, r2)` per `strut.rs:219-234`:
/// cylinder branch when `|r1 − r2| < f64::EPSILON` returns
/// `π · r1² · length`; otherwise the truncated-cone formula
/// `(π/3) · length · (r1² + r1·r2 + r2²)`. With `r2 = 0` the cone
/// degenerates to `(π/3) · length · r1²`.
fn verify_estimate_strut_volume() {
    use std::f64::consts::PI;

    // (1) Cylinder: r1 == r2 == 1, length 10 ⇒ π · 1² · 10 = 10π.
    let cyl = estimate_strut_volume(10.0, 1.0, 1.0);
    assert_relative_eq!(cyl, PI * 10.0, epsilon = STRUT_VOL_TOL);

    // (2) Full cone: r1 = 1, r2 = 0, length 10 ⇒ (π/3) · 10 · 1 = 10π/3.
    let cone = estimate_strut_volume(10.0, 1.0, 0.0);
    assert_relative_eq!(cone, (PI / 3.0) * 10.0, epsilon = STRUT_VOL_TOL);
}

// =============================================================================
// verify_lattice_type_traits — public-surface coverage on LatticeType
// =============================================================================

/// `LatticeType::Cubic` is strut-based (NOT TPMS); `recommended_resolution`
/// for strut paths is the constant 10 (TPMS path returns 15).
fn verify_lattice_type_traits() {
    assert!(
        LatticeType::Cubic.is_strut_based(),
        "Cubic uses cylindrical beams between grid nodes",
    );
    assert!(
        !LatticeType::Cubic.is_tpms(),
        "Cubic is NOT a TPMS (no implicit surface)",
    );
    assert_eq!(LatticeType::Cubic.recommended_resolution(), 10);
    assert_eq!(LatticeType::Cubic.name(), "Cubic");
}

// =============================================================================
// build_params + verify_params_validate — builder + validate()
// =============================================================================

/// Configured `LatticeParams`: cubic preset, strut thickness 1.0,
/// density 1.0 (uniform full-density grid), `with_beam_export(true)`,
/// `with_trim_to_bounds(true)` (the default; reaffirmed for the
/// public-surface anchor).
fn build_params() -> LatticeParams {
    LatticeParams::cubic(CELL_SIZE)
        .with_strut_thickness(STRUT_THICKNESS)
        .with_density(DENSITY)
        .with_beam_export(true)
        .with_trim_to_bounds(true)
}

/// Lock fields to expected values + assert `validate()` returns
/// `Ok(())`. `with_density` clamps to `[0, 1]` so `1.0` stays exact.
fn verify_params_validate(params: &LatticeParams) {
    assert_eq!(params.lattice_type, LatticeType::Cubic);
    assert_relative_eq!(params.cell_size, CELL_SIZE, epsilon = TIGHT_TOL);
    assert_relative_eq!(params.strut_thickness, STRUT_THICKNESS, epsilon = TIGHT_TOL);
    assert_eq!(
        params.density.to_bits(),
        DENSITY.to_bits(),
        "density 1.0 is exactly representable in f64",
    );
    assert!(
        params.preserve_beam_data,
        "with_beam_export(true) must enable BeamLatticeData export",
    );
    assert!(
        params.trim_to_bounds,
        "with_trim_to_bounds(true) reaffirmed"
    );

    if params.validate().is_err() {
        unreachable!("build_params yields a valid LatticeParams");
    }
}

// =============================================================================
// verify_lattice_result_geometry — cell_count + mesh counts (BIT-EXACT)
// =============================================================================

/// Locks `cell_count == 125` (5³) and the per-strut combinatorial
/// mesh counts (`540 × 14 = 7560` verts + `540 × 24 = 12960` tris)
/// BIT-EXACT — `assert_eq!`, NOT `assert_relative_eq!`. None of the
/// 540 struts is degenerate at integer-spaced grid points; all pass
/// the trim check (max grid-node coord = `BBOX_MAX = 25.0`, never
/// strictly greater).
fn verify_lattice_result_geometry(result: &LatticeResult) {
    assert_eq!(
        result.cell_count, EXPECTED_CELL_COUNT,
        "25 mm bbox / 5 mm cell = 5 cells per axis, 5³ = 125",
    );
    assert_eq!(
        result.vertex_count(),
        EXPECTED_VERTEX_COUNT,
        "540 struts × 14 verts = 7560 (combine_struts has no weld)",
    );
    assert_eq!(
        result.triangle_count(),
        EXPECTED_TRIANGLE_COUNT,
        "540 struts × 24 tris = 12960",
    );
}

// =============================================================================
// verify_total_strut_length — Some(2700.0) BIT-EXACT
// =============================================================================

/// `result.total_strut_length` accumulates `+= cell_size = 5.0`
/// inside `generate_cubic_lattice` (`generate.rs:156/183/209`). The
/// running partial sum stays an integer multiple of 5.0 throughout,
/// well below 2⁵³, so the final `Some(2700.0)` is BIT-EXACT (`to_bits`
/// equality, NOT `assert_relative_eq!`).
fn verify_total_strut_length(result: &LatticeResult) {
    let Some(len) = result.total_strut_length else {
        unreachable!("strut-based lattice path always sets with_strut_length");
    };
    assert_eq!(
        len.to_bits(),
        EXPECTED_TOTAL_LENGTH.to_bits(),
        "total_strut_length must be BIT-EXACT 2700.0 (540 × 5.0); got {len}",
    );
}

// =============================================================================
// verify_actual_density — F9 heuristic finite + in [0, 1]
// =============================================================================

/// `LatticeResult::actual_density` for the cubic path uses the
/// `estimate_strut_volume(&mesh, radius)` heuristic
/// (`generate.rs:519-549`) — `triangle_count / 24` for an
/// approximate strut count, mesh-bbox diagonal divided by
/// `cbrt(strut_count) + 1` for an approximate average length, then
/// `n · π · r² · L_avg`. The heuristic doesn't have the closed-
/// orientable-manifold pathology that breaks `actual_density` on
/// un-welded TPMS shells (signed-tet volume integration is not used
/// on the strut path), but the `+ 1` term in
/// the denominator and the diagonal-as-length proxy still bias the
/// estimate. Anchor here is the F9 declared range — finite and in
/// `[0, 1]` — with the empirical `≈ 0.13` reported in the summary.
fn verify_actual_density(result: &LatticeResult) {
    let d = result.actual_density;
    assert!(
        d.is_finite() && (0.0..=1.0).contains(&d),
        "actual_density = {d} must be finite in [0, 1]",
    );
}

// =============================================================================
// verify_beam_data — independent-path verification of the 3MF precursor
// =============================================================================

/// Every beam-data anchor for the F11 v0.9 3MF beam writer's input
/// shape. Returns `data.estimate_volume()` for inclusion in the
/// summary print.
///
/// The dedup is independent from the mesh: `BeamLatticeData.vertices`
/// is keyed by the `quantize` `HashMap` pattern at scale `1e6`
/// (`generate.rs:111-119`), so all 540 beam endpoints collapse onto
/// the `(cells + 1)³ = 6³ = 216` unique grid nodes. Each beam
/// references those nodes by index.
fn verify_beam_data(data: &BeamLatticeData) -> f64 {
    // Counts.
    assert_eq!(
        data.vertex_count(),
        EXPECTED_BEAM_VERTEX_COUNT,
        "(cells + 1)³ = 6³ = 216 unique grid nodes after quantize-dedup",
    );
    assert_eq!(
        data.beam_count(),
        EXPECTED_BEAM_COUNT,
        "3 axes × 5 in-axis × 6 × 6 perpendicular = 540 cubic-lattice edges",
    );

    // Total length: every edge is exactly cell_size = 5.0 (axis-aligned,
    // perfect-square sqrt(25) = 5.0); 540 × 5.0 = 2700.0.
    let beam_total = data.total_length();
    assert_relative_eq!(beam_total, EXPECTED_TOTAL_LENGTH, epsilon = LENGTH_TOL);

    // Per-beam invariants.
    for (i, beam) in data.beams.iter().enumerate() {
        assert_relative_eq!(beam.r1, RADIUS, epsilon = TIGHT_TOL);
        assert_relative_eq!(beam.r2, RADIUS, epsilon = TIGHT_TOL);
        assert_eq!(
            beam.cap1,
            BeamCap::Sphere,
            "beam {i} cap1 default must be Sphere",
        );
        assert_eq!(
            beam.cap2,
            BeamCap::Sphere,
            "beam {i} cap2 default must be Sphere",
        );

        let Some(len) = beam.length(&data.vertices) else {
            unreachable!("beam {i} references valid vertex indices");
        };
        assert_relative_eq!(len, CELL_SIZE, epsilon = LENGTH_TOL);
    }

    // estimate_volume: 540 cylinders, π · 0.5² · 5.0 · 540 = 675π ≈ 2120.6.
    let vol = data.estimate_volume();
    assert!(
        (VOLUME_LOWER..=VOLUME_UPPER).contains(&vol),
        "data.estimate_volume() = {vol:.2} must be in [{VOLUME_LOWER}, {VOLUME_UPPER}] (≈ 675π)",
    );
    vol
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

/// Bundled inputs for [`print_summary`]; avoids
/// `clippy::too_many_arguments` while keeping fields trivially
/// constructed at the call site (precedent: `Summary` in
/// `mesh-sdf-distance-query` + `mesh-lattice-tpms-gyroid`).
struct Summary<'a> {
    result: &'a LatticeResult,
    beam_data: &'a BeamLatticeData,
    beam_estimate_volume: f64,
}

/// Print the human-readable summary. Extracted from `main` to keep
/// the entrypoint under clippy's `too_many_lines` cap.
fn print_summary(s: &Summary) {
    println!("==== mesh-lattice-strut-cubic ====");
    println!();
    println!(
        "fixture: 25 mm bbox, cell_size = {CELL_SIZE} mm, strut_thickness = \
         {STRUT_THICKNESS} mm",
    );
    println!("         density = {DENSITY}, with_beam_export(true)");
    println!();
    println!("Free-fn anchors:");
    println!("  generate_strut → {VERTS_PER_STRUT} verts + {TRIS_PER_STRUT} tris (BIT-EXACT)");
    println!("  generate_strut_tapered → same {VERTS_PER_STRUT} / {TRIS_PER_STRUT} counts");
    println!("  generate_strut(p, p, 0.5) = None (zero-length branch)");
    println!(
        "  combine_struts(N=3) → {} verts + {} tris (no inter-strut welding)",
        3 * VERTS_PER_STRUT,
        3 * TRIS_PER_STRUT,
    );
    println!("  estimate_strut_volume(10, 1, 1) ≈ 10π   = 31.4159 (cylinder, ≤ 1e-10)");
    println!("  estimate_strut_volume(10, 1, 0) ≈ 10π/3 = 10.4720 (cone,     ≤ 1e-10)");
    println!();
    println!("LatticeType::Cubic traits:");
    println!("  is_strut_based = true, is_tpms = false, recommended_resolution = 10");
    println!();
    println!("Generated lattice:");
    println!(
        "  cell_count       = {} (BIT-EXACT; 5 × 5 × 5)",
        s.result.cell_count,
    );
    println!(
        "  vertex_count     = {} (BIT-EXACT; 540 struts × 14)",
        s.result.vertex_count(),
    );
    println!(
        "  triangle_count   = {} (BIT-EXACT; 540 struts × 24)",
        s.result.triangle_count(),
    );
    println!(
        "  total_strut_len  = {:.1} (BIT-EXACT Some(2700.0); 540 × 5.0)",
        s.result.total_strut_length.unwrap_or(f64::NAN),
    );
    println!(
        "  actual_density   ≈ {:.4} (F9 heuristic; finite + in [0, 1])",
        s.result.actual_density,
    );
    println!();
    println!("BeamLatticeData (3MF beam-extension precursor; F11 v0.9 consumer):");
    println!(
        "  vertex_count() = {} (BIT-EXACT; 6³ unique grid nodes after dedup)",
        s.beam_data.vertex_count(),
    );
    println!(
        "  beam_count()   = {} (BIT-EXACT; 3 × 5 × 6 × 6)",
        s.beam_data.beam_count(),
    );
    println!(
        "  total_length() = {:.6} mm (≤ 1e-9; sum of 540 axis-aligned 5.0)",
        s.beam_data.total_length(),
    );
    println!("  per-beam r1 == r2 == {RADIUS} mm (≤ 1e-12; density.sqrt() = 1.0)");
    println!("  per-beam cap1 == cap2 == BeamCap::Sphere (default)");
    println!(
        "  estimate_volume() = {:.2} mm³ (in [{VOLUME_LOWER}, {VOLUME_UPPER}]; ≈ 675π)",
        s.beam_estimate_volume,
    );
    println!("  per-beam length(&vertices) = Some(5.0) for all 540 beams (≤ 1e-9)");
    println!();
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    verify_strut_free_fns();
    verify_estimate_strut_volume();
    verify_lattice_type_traits();

    let params = build_params();
    verify_params_validate(&params);

    let bounds = (
        Point3::new(BBOX_MIN, BBOX_MIN, BBOX_MIN),
        Point3::new(BBOX_MAX, BBOX_MAX, BBOX_MAX),
    );
    let Ok(result) = generate_lattice(&params, bounds) else {
        unreachable!("validated params + valid bounds: generate_lattice cannot fail");
    };

    verify_lattice_result_geometry(&result);
    verify_total_strut_length(&result);
    verify_actual_density(&result);

    let Some(beam_data) = result.beam_data.as_ref() else {
        unreachable!("with_beam_export(true) ⇒ beam_data is Some(_)");
    };
    let beam_estimate_volume = verify_beam_data(beam_data);

    print_summary(&Summary {
        result: &result,
        beam_data,
        beam_estimate_volume,
    });

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let mesh_path = out_dir.join("cubic_lattice.ply");
    save_ply(&result.mesh, &mesh_path, true)?;
    println!(
        "artifact: out/cubic_lattice.ply ({}v, {}f, binary little-endian)",
        result.mesh.vertices.len(),
        result.mesh.faces.len(),
    );
    println!();
    println!(
        "OK — 4 strut + 2 estimate_strut_volume + 4 LatticeType + params validate + \
         cell_count + mesh counts + total_strut_length + actual_density + \
         beam_data (counts + total_length + per-beam r1/r2/caps/length + estimate_volume) \
         all green",
    );

    Ok(())
}
