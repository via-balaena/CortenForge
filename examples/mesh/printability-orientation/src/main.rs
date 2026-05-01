//! Visual demo of Gap L (`PrinterConfig::build_up_direction` parametrization).
//!
//! Hand-authors a single watertight cylinder (radius 5 mm, length 15 mm,
//! 32-segment lateral tessellation) whose axis points along
//! `(sin(60°), 0, cos(60°))` — a column leaning 60° from vertical toward
//! `+X`. The example then validates the same physical setup three ways:
//!
//! 1. **Default `+Z` up** with the original mesh — substantial overhang
//!    flagged on the lateral downhill arc (60° lean past 45° threshold).
//! 2. **Manual exact rotation** `R_Y(-60°)` applied via
//!    `apply_orientation` — cylinder stands perfectly vertical; bottom
//!    cap flat on the build plate; no overhang.
//! 3. **`with_build_up_direction(axis)`** on the original mesh — the
//!    validator's `up` aligns with the cylinder's axis; no overhang.
//!
//! Runs 2 and 3 are mathematically equivalent expressions of the same
//! physical scenario (rotating the mesh ≡ rotating the up-vector when
//! both are exact). The example asserts this equivalence as the
//! load-bearing **Gap L invariant**:
//! `assert_relative_eq!(run2_area, run3_area, ε = 1e-6)`.
//!
//! ## Why not `find_optimal_orientation` for Run 2?
//!
//! `find_optimal_orientation`'s 12-sample default is the discrete set
//! `{identity, ±90°/180° X, ±90° Y, ±45°/±135° X, +45° Y}` — none of
//! which is `R_Y(-60°)`, the exact compensating rotation needed for this
//! fixture. For the R=5 / L=15 cylinder, the search picks **identity**
//! (no rotation): every non-identity sample either preserves or worsens
//! the overhang area for this aspect ratio. The closest candidate
//! `R_Y(-90°)` drives lateral overhang to 0 but tilts the bottom cap
//! 30°, exposing ~30 of 32 cap-fan triangles for a comparable
//! post-rotation overhang. Mesh-rotation ≡ up-vector-rotation only when
//! both rotations are EXACT; a discrete search heuristic does not
//! deliver exact equivalence in general. `find_optimal_orientation` is
//! exercised here as a **stdout diagnostic** — its output is printed
//! but NOT asserted.
//!
//! Logged as a v0.9 candidate: enrich `find_optimal_orientation`'s
//! sampler (e.g., 1° angle bins around primary axes, or a
//! gradient-descent refinement step) so it can reach arbitrary axis-
//! aligned rotations like `R_Y(-60°)`.
//!
//! ## Length deviation from spec (30 mm → 15 mm)
//!
//! `V08_FIX_ARC_SPEC.md` §7.6 originally specified `LENGTH = 30 mm`. At
//! that length, mesh-repair's `detect_self_intersections` (called from
//! `check_self_intersecting`) fires 8 false-positive `SelfIntersecting`
//! `Critical` pairs between diametrically-opposite lateral triangles
//! (independent of segment count: empirically tested at SEGMENTS ∈
//! {8, 12, 16, 32}). The threshold is around L ≈ 18 mm regardless of
//! segment count. Reducing `L` to 15 mm drops the lateral-triangle
//! aspect ratio below the false-positive threshold and yields a clean
//! `is_printable() == true` Run 1 with 2 Info-only `OverhangRegion`s.
//! The spec's original `[100, 250]` mm² area band (sized for L=30) is
//! reduced to `[50, 100]` for the L=15 fixture. Logged as v0.9
//! candidate per CHANGELOG.md "v0.9 candidates" section.
//!
//! ## Numerical anchors (asserted in `main`)
//!
//! Anchors per `V08_FIX_ARC_SPEC.md` §7.6 (corrected from spec's original
//! `find_optimal_orientation`-based Run 2 to the manual exact rotation):
//!
//! 1. Run 1: `overhangs.len() >= 1`; total overhang area ∈ `[50, 100]`
//!    mm² (analytical "downhill arc" minus build-plate-filtered cluster
//!    ≈ 66 mm² at R=5 / L=15 / SEG=32).
//! 2. Run 1: at least one `ExcessiveOverhang` issue with `Info` severity
//!    (per §4.3 boundary: `45° < observed ≤ 60°` band; max observed
//!    angle 59.526° due to 32-segment chord discretization).
//! 3. Run 1: `is_printable()` printed but NOT asserted (cylinder has no
//!    thin walls / cavity / self-intersection / small features at this
//!    scale, and `ExcessiveOverhang` at Info severity does NOT block; the
//!    value is `true` but not load-bearing).
//! 4. Run 2: `overhangs.len() == 0`; total overhang area = 0 within
//!    `1e-6` mm² (exact rotation makes cylinder perfectly vertical;
//!    bottom cap flat on plate → all build-plate-filtered).
//! 5. Run 3: `overhangs.len() == 0`; total overhang area = 0 within
//!    `1e-6` mm² (up-vector aligned with axis → all bottom-cap rim
//!    vertices share `axis-projection = mesh_min_along_up` → all
//!    build-plate-filtered).
//! 6. **Gap L invariant**: `assert_relative_eq!(run2_area, run3_area,
//!    epsilon = 1e-6)` — semantically the load-bearing equivalence
//!    (mesh-rotation ≡ up-vector-rotation when both are exact);
//!    trivially `0 == 0` at the FP level.
//! 7. Run 2 area STRICTLY LESS than Run 1 area — the exact rotation
//!    eliminates support need.
//! 8. All three Runs print `overhangs.len()` + total area to stdout.
//! 9. Manual rotation correctness: `debug_assert!` that `R_Y(-60°) ×
//!    axis_pre - (0, 0, 1)` has norm < `1e-12` (locks the construction).
//! 10. Diagnostic `find_optimal_orientation` print — picked rotation +
//!     resulting `overhang_area`; no value-level assertion (the picked
//!     sample depends on sample-set ordering, which may shift in v0.9
//!     enrichment).
//! 11. **Geometric — centroid on lateral surface**: each Run 1
//!     `OverhangRegion.center` has radial distance from the cylinder's
//!     central axis line within `[RADIAL_BAND_LO, RADIAL_BAND_HI]` =
//!     `[4.5, 5.0]` mm (chord-shrinkage envelope for a 4–5 face cluster).
//!     Catches regressions where the detector emits centroids off the
//!     geometry (e.g., interior-point bug in Gap D partition).
//! 12. **Geometric — centroid in downhill arc**: each Run 1
//!     `OverhangRegion.center`'s azimuth in the cylinder's perpendicular
//!     plane lies within `±33.75°` of the downhill direction (`270°`).
//!     Catches regressions where the detector flags the wrong side of
//!     the cylinder (Gap M predicate inversion or build-plate filter
//!     bug).
//!
//! ## How to run
//!
//! ```text
//! cargo run -p example-mesh-printability-orientation --release
//! ```
//!
//! Output written to `examples/mesh/printability-orientation/out/`. Open
//! `mesh_original.ply` and `mesh_rotated.ply` side-by-side in `MeshLab`,
//! `ParaView`, or `f3d` for the visuals pass. See the README for the
//! `f3d --up +Z` recommendation (the cylinder's axis-up convention is
//! load-bearing for the visual reading; `f3d`'s default `+Y` up renders
//! the rotated cylinder lying on its side, which is misleading).

use std::path::Path;

use anyhow::Result;
use mesh_io::save_ply;
use mesh_printability::{
    IssueSeverity, OrientationResult, PrintIssueType, PrintValidation, PrinterConfig,
    apply_orientation, find_optimal_orientation, place_on_build_plate, validate_for_printing,
};
use mesh_types::{IndexedMesh, Point3, Vector3};
use nalgebra::{Unit, UnitQuaternion};

// -- Geometry constants (mm, deg) -------------------------------------------

/// Cylinder radius. Lateral inward ray-cast distance ≈ diameter = 10 mm,
/// well above FDM `min_wall_thickness/2 = 0.5 mm`, so the `ThinWall`
/// detector does NOT co-flag (lesson from row #19's hex-prism burr).
const RADIUS: f64 = 5.0;

/// Cylinder length along its leaning axis. End-cap-to-end-cap span =
/// 15 mm. **Deviation from `V08_FIX_ARC_SPEC.md` §7.6's original 30 mm**:
/// at LENGTH=18+ mm, mesh-repair `detect_self_intersections` produces 8
/// false-positive `SelfIntersecting` Critical pairs between
/// diametrically-opposite lateral triangles (independent of segment
/// count: empirically tested at SEGMENTS ∈ {8, 12, 16, 32}); the
/// 30:1 lateral-triangle aspect ratio at R=5/L=30/SEG=32 trips an FP
/// edge case in mesh-repair's BVH triangle-pair test. LENGTH=15 keeps
/// the cylinder aspect ratio at 1.5:1 (column still visually leans),
/// stays under the false-positive threshold, and preserves a clean
/// `is_printable() == true` Run 1 (Info-only). Logged as v0.9
/// candidate: investigate mesh-repair sensitivity to thin-aspect-ratio
/// cylinders.
const LENGTH: f64 = 15.0;

/// Number of azimuthal segments for the lateral tessellation. 32 is
/// the §7.6 spec's resolution; visual rendering shows a chamfered
/// (NOT smooth) lateral surface at this segment count per
/// `feedback_chamfered_not_rounded`. With LENGTH=15, each lateral
/// triangle has area ≈ (2π · 5 · 15) / 64 ≈ 7.36 mm² — narrower
/// in azimuth than the original spec's L=30 baseline but the same
/// aspect-ratio per triangle.
const SEGMENTS: u32 = 32;

/// Tilt of the cylinder's axis from vertical (degrees). Locks the
/// problem statement: the cylinder leans 60° from vertical toward `+X`,
/// so the lateral downhill arc has `overhang_angle` reaching 60°
/// (boundary of `Info`/`Warning` per §4.3 — strict-greater-than at the
/// threshold edge keeps the cluster in `Info`).
const TILT_DEG: f64 = 60.0;

/// `find_optimal_orientation` sample count for the diagnostic call. 12
/// is the default sample count cited by the §7.6 spec body. With 12,
/// the fixed-orientation set `{identity, ±90°/180° X, ±90° Y, ±45°/
/// ±135° X, +45° Y}` is exhausted; the Fibonacci sphere branch does
/// not engage. Exhausting the fixed set is the load-bearing case for
/// the diagnostic — the sample set demonstrably does NOT include
/// `R_Y(-60°)`.
const SEARCH_SAMPLES: usize = 12;

/// Unit axis vector of the leaning cylinder, pointing along
/// `(sin(60°), 0, cos(60°))`. Computed at call sites via `tilt_rad =
/// TILT_DEG.to_radians()` so the constant value is FP-derived once and
/// reused (no precomputed-magic-number drift across call sites).
fn axis_unit() -> Vector3<f64> {
    let tilt_rad = TILT_DEG.to_radians();
    Vector3::new(tilt_rad.sin(), 0.0, tilt_rad.cos())
}

/// First perpendicular basis vector for the cylinder construction:
/// `+Y`. Trivially perpendicular to both `axis_unit` (which has `y = 0`)
/// and `perp_v_unit` (also `y = 0`).
const fn perp_u_unit() -> Vector3<f64> {
    Vector3::new(0.0, 1.0, 0.0)
}

/// Second perpendicular basis vector for the cylinder construction:
/// `(-cos(60°), 0, sin(60°))`. Verifies the right-handed basis:
/// `perp_u × perp_v = (1·sin60° - 0·0, 0·(-cos60°) - 0·sin60°,
/// 0·0 - 1·(-cos60°)) = (sin60°, 0, cos60°) = axis_unit` ✓. Magnitude:
/// `sqrt(cos²60° + sin²60°) = 1` ✓. Dot with axis: `(-cos60°)(sin60°) +
/// 0 + (sin60°)(cos60°) = 0` ✓.
fn perp_v_unit() -> Vector3<f64> {
    let tilt_rad = TILT_DEG.to_radians();
    Vector3::new(-tilt_rad.cos(), 0.0, tilt_rad.sin())
}

/// Manually-constructed exact compensating rotation for Gap L Run 2.
/// `R_Y(-60°)` maps `(sin(60°), 0, cos(60°))` onto `(0, 0, 1)` — i.e.
/// rotates the cylinder's leaning axis exactly onto the world `+Z`,
/// making the validator see the cylinder as perfectly vertical without
/// any up-vector parametrization. Verified at construction time via
/// `debug_assert` (anchor #9).
fn exact_compensating_rotation() -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(
        &Unit::new_normalize(Vector3::new(0.0, 1.0, 0.0)),
        -TILT_DEG.to_radians(),
    )
}

/// Z-shift applied by `place_on_build_plate` to the leaning cylinder
/// (= the negative of the pre-place mesh's min z). The pre-place
/// cylinder has `min z = neg_centre.z + perp_v.z · R · sin(-π/2) =
/// -cos(60°)·LENGTH/2 - sin(60°)·R`; `lift_z` is the absolute value.
/// Derived analytically rather than from the placed mesh's bbox so the
/// geometric anchors (#11 / #12) reference a closed-form value rather
/// than an `O(n_vertex)` min-reduction (which would carry per-vertex
/// FP drift).
fn place_lift_z() -> f64 {
    let tilt_rad = TILT_DEG.to_radians();
    // FP-bit preserved: the analytical lift is two independent terms
    // (axial half-length projection + radial sin-projection); rewriting
    // as `(LENGTH / 2.0).mul_add(tilt_rad.cos(), RADIUS * tilt_rad.sin())`
    // would subtly shift the cap-rim z values used by anchors #11 / #12,
    // making cross-platform FP drift hard to reason about. Mirror of the
    // mesh-printability/orientation FP-semantics convention; see
    // CHANGELOG.md `[Unreleased] / v0.9 candidates / FP bit-exactness`.
    #[allow(clippy::suboptimal_flops)]
    let lift = LENGTH / 2.0 * tilt_rad.cos() + RADIUS * tilt_rad.sin();
    lift
}

/// Radial distance from the cylinder's central axis line for a point
/// in post-place_on_build_plate coordinates. Translates the point back
/// to the pre-place frame (`z -= lift_z`), projects onto the axis line
/// passing through origin, and returns the perpendicular distance.
///
/// Anchor #11 asserts each `OverhangRegion` centroid is within
/// `[RADIAL_BAND_LO, RADIAL_BAND_HI]` of the lateral surface (= chord-
/// shrinkage envelope for a multi-face cluster spanning ±`arc_half_width`
/// in azimuth). For the §7.6 fixture's 4-5 face clusters spanning ~45°
/// of azimuth, the chord shrinkage is `R · (1 - cos(arc_half_width))` ≈
/// `R · (1 - cos(22.5°))` ≈ `0.38 mm`, so the band is `[R · cos(22.5°),
/// R] ≈ [4.62, 5.0]` mm.
fn centroid_radial_distance_from_axis(centroid: Point3<f64>) -> f64 {
    let axis = axis_unit();
    let lift_z = place_lift_z();
    // Translate centroid back to pre-place frame (cylinder centred at origin).
    let c_pre = Vector3::new(centroid.x, centroid.y, centroid.z - lift_z);
    // Project onto axis line passing through origin: t = c_pre · axis.
    let t = c_pre.dot(&axis);
    let p_on_axis = axis * t;
    (c_pre - p_on_axis).norm()
}

/// Azimuth (in degrees, range `[0, 360)`) of a point in the cylinder's
/// perpendicular plane, measured in the `(perp_u, perp_v)` frame.
/// `0°` is `+perp_u` (= `+Y` in world coords); `90°` is `+perp_v`
/// (= `(-cos60°, 0, sin60°)` in world coords); `270°` is `-perp_v`
/// (= the downhill direction in the cylinder's tilted frame).
///
/// Anchor #12 asserts each `OverhangRegion` centroid's azimuth lies
/// within `±DOWNHILL_HALF_WIDTH_DEG` of `DOWNHILL_AZIMUTH_DEG = 270°`
/// — i.e. within the flagged downhill arc per §7.6's expected output.
fn centroid_azimuth_deg(centroid: Point3<f64>) -> f64 {
    let axis = axis_unit();
    let perp_u = perp_u_unit();
    let perp_v = perp_v_unit();
    let lift_z = place_lift_z();
    let c_pre = Vector3::new(centroid.x, centroid.y, centroid.z - lift_z);
    let t = c_pre.dot(&axis);
    let p_on_axis = axis * t;
    let diff = c_pre - p_on_axis;
    let proj_u = diff.dot(&perp_u);
    let proj_v = diff.dot(&perp_v);
    let az_deg = proj_v.atan2(proj_u).to_degrees();
    if az_deg < 0.0 { az_deg + 360.0 } else { az_deg }
}

// -- Geometric anchor bands (anchors #11 + #12) ----------------------------

/// Lower bound for the centroid's radial distance from the cylinder's
/// axis line. Multi-face cluster centroid sits inside the rim circle
/// by chord-shrinkage proportional to the cluster's azimuthal half-
/// width. For the §7.6 fixture's 4-5 face clusters spanning ~45° of
/// azimuth, the bound is `R · cos(22.5°)` ≈ `4.6194`. Padded down to
/// `4.50` to absorb FP drift across mesh-printability detector
/// implementations (Gap D split heuristic is per-component edge-walk;
/// per-cluster face count can vary by ±1 cross-platform).
const RADIAL_BAND_LO: f64 = 4.50;

/// Upper bound for the centroid's radial distance: exactly `R`. A
/// single-face cluster's centroid sits at `R · cos(π/(2·SEG))` ≈
/// `R · 0.999` for `SEG = 32` (each lateral face's vertices are on
/// the rim circle of radius `R`; the face centroid is the mean of 3
/// such points). Multi-face clusters CANNOT exceed `R` (the convex
/// hull of rim verts is bounded by the cylinder's circumscribed
/// cylinder of radius `R`).
const RADIAL_BAND_HI: f64 = RADIUS;

/// Center of the flagged downhill arc in the `(perp_u, perp_v)` frame.
/// `270°` corresponds to `-perp_v` direction = the cylinder's downhill
/// side in the tilted frame.
const DOWNHILL_AZIMUTH_DEG: f64 = 270.0;

/// Half-width of the flagged downhill arc per §7.6 expected output:
/// `±33.75°` (= the strict-greater-than `45°` threshold maps to
/// lateral-normal `dot < -sin(45°)` = `dot < -0.707`; for cylinder axis
/// at `α = 60°`, max-downward dot is `-sin(60°) = -0.866`; the chord
/// of normals satisfying `dot < -0.707` spans `±33.75°` from straight
/// downhill in the perpendicular-plane azimuth).
const DOWNHILL_HALF_WIDTH_DEG: f64 = 33.75;

// `clippy::too_many_lines`: the 3-runs-+-diagnostic-+-5-PLY-writes structure
// is the example's pedagogical surface — splitting per-Run into helpers would
// hide the run-by-run flow that the visuals-pass reader walks down. Each Run
// block stays inline (validate → print → assert) for top-to-bottom readability.
#[allow(clippy::too_many_lines)]
fn main() -> Result<()> {
    // ─── Fixture: leaning cylinder, post-`place_on_build_plate` ──────────
    let cylinder_pre_place = build_leaning_cylinder();
    let mesh = place_on_build_plate(&cylinder_pre_place);

    println!("==== mesh-printability-orientation ====");
    println!();
    let axis = axis_unit();
    println!(
        "input  : {}-vertex, {}-triangle leaning cylinder ({SEGMENTS}-segment lateral, \
         radius {RADIUS} mm × length {LENGTH} mm)",
        mesh.vertices.len(),
        mesh.faces.len(),
    );
    println!(
        "         axis = ({:+.4}, {:+.4}, {:+.4}) — leaning {TILT_DEG}° from vertical toward +X",
        axis.x, axis.y, axis.z,
    );
    let bbox = bounding_box(&mesh);
    println!(
        "         bbox post-place_on_build_plate: \
         x ∈ [{:+.3}, {:+.3}], y ∈ [{:+.3}, {:+.3}], z ∈ [{:+.3}, {:+.3}]",
        bbox.0.x, bbox.1.x, bbox.0.y, bbox.1.y, bbox.0.z, bbox.1.z,
    );
    println!();

    // Anchor #9 — manual rotation correctness. R_Y(-60°) × axis must equal +Z
    // within 1e-12; this locks the rotation construction at impl time. The
    // rotation matrix path (vs direct quat application) matches what
    // `apply_orientation` does internally.
    let r_exact = exact_compensating_rotation();
    let z_unit = Vector3::new(0.0, 0.0, 1.0);
    let axis_post = r_exact.to_rotation_matrix() * axis;
    debug_assert!(
        (axis_post - z_unit).norm() < 1e-12,
        "anchor #9: R_Y(-60°) × axis_unit must equal +Z within 1e-12; got ({:+.6e}, {:+.6e}, \
         {:+.6e}) (distance {:.3e})",
        axis_post.x,
        axis_post.y,
        axis_post.z,
        (axis_post - z_unit).norm(),
    );

    // ─── Run 1: original mesh + default `+Z up` config ───────────────────
    println!("---- Run 1: original mesh + default `+Z up` config ----");
    let config_default = PrinterConfig::fdm_default();
    let validation_run1 = validate_for_printing(&mesh, &config_default)?;
    print_run_summary("Run 1", &validation_run1);
    let run1_area = total_overhang_area(&validation_run1);
    let run1_max_angle = max_overhang_angle(&validation_run1);
    println!(
        "  Run 1 overhangs: {} regions, total area {run1_area:.3} mm², max angle {run1_max_angle:.3}°",
        validation_run1.overhangs.len(),
    );
    // Anchor #3 — print is_printable() but do NOT assert (a cylinder has no
    // ThinWall / TrappedVolume / SelfIntersecting / SmallFeature at this
    // scale, and ExcessiveOverhang at Info severity does NOT block; expected
    // true). The print captures Run 1's printability for the visuals-pass
    // narrative but is not the load-bearing claim.
    println!(
        "  Run 1 is_printable() = {}",
        validation_run1.is_printable()
    );
    println!();

    // Anchor #1 — Run 1 overhangs.len() >= 1 and total area in [50, 100].
    // Empirical area at R=5 / L=15 / SEG=32 is ≈ 66.16 mm² (2 OverhangRegions
    // per Gap D split — the downhill arc partitions into two edge-adjacent
    // clusters). Band brackets ±50% to absorb tessellation drift / FP noise.
    // Reduced from spec's [100, 250] (sized for L=30) per the LENGTH
    // constant's mesh-repair-false-positive deviation rationale.
    assert!(
        !validation_run1.overhangs.is_empty(),
        "anchor #1: Run 1 must produce at least one OverhangRegion (60° lean > 45° threshold); \
         got 0 — check fixture construction or detector regression",
    );
    assert!(
        (50.0..=100.0).contains(&run1_area),
        "anchor #1: Run 1 total overhang area {run1_area:.3} mm² outside expected band [50, 100] \
         (analytical downhill arc minus build-plate-filtered cluster ≈ 66 mm² at R=5 / L=15 / \
         SEG=32) — investigate tessellation count or build-plate filter behavior",
    );

    // Anchor #2 — at least one ExcessiveOverhang issue with Info severity.
    let run1_overhang_info = validation_run1.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::ExcessiveOverhang && i.severity == IssueSeverity::Info
    });
    assert!(
        run1_overhang_info,
        "anchor #2: Run 1 must surface an ExcessiveOverhang issue with Info severity (60° max \
         angle ≤ threshold + 15° = 60° per §4.3 strict-greater-than boundary)",
    );

    // Anchors #11 + #12 — geometric correctness of OverhangRegion centroids
    // against the leaning cylinder's analytical lateral surface + downhill
    // arc azimuth band. Catches regressions where the detector emits
    // centroids off the geometry (e.g., interior-point bug in Gap D
    // partition) or flags the wrong side of the cylinder. Visuals-pass
    // counterpart of the area/severity-only anchors above.
    for (i, region) in validation_run1.overhangs.iter().enumerate() {
        // Anchor #11 — centroid lies on cylinder's lateral surface.
        let r = centroid_radial_distance_from_axis(region.center);
        assert!(
            (RADIAL_BAND_LO..=RADIAL_BAND_HI).contains(&r),
            "anchor #11: Run 1 centroid[{i}] radial distance from axis line = {r:.6} mm, \
             outside expected band [{RADIAL_BAND_LO}, {RADIAL_BAND_HI}] (chord-shrinkage envelope \
             for a 4–5 face cluster spanning ±22.5° of azimuth at R={RADIUS}). Centroid is NOT \
             on the cylinder's lateral surface — investigate detector regression that emits \
             interior or off-mesh points.",
        );

        // Anchor #12 — centroid in the downhill arc.
        let az = centroid_azimuth_deg(region.center);
        let raw_off = (az - DOWNHILL_AZIMUTH_DEG).abs();
        // Wrap into [0, 180] for the off-from-downhill comparison.
        let off = if raw_off > 180.0 {
            360.0 - raw_off
        } else {
            raw_off
        };
        assert!(
            off <= DOWNHILL_HALF_WIDTH_DEG,
            "anchor #12: Run 1 centroid[{i}] azimuth = {az:.3}° is {off:.3}° from downhill \
             ({DOWNHILL_AZIMUTH_DEG}°), outside expected ±{DOWNHILL_HALF_WIDTH_DEG}° band. \
             Cluster is on wrong side of cylinder — investigate Gap M overhang predicate \
             regression or build-plate filter inversion.",
        );
    }

    // ─── Run 2: manual exact R_Y(-60°) rotation + default config ─────────
    println!("---- Run 2: manual exact R_Y(-60°) rotation + default `+Z up` config ----");
    let exact_orientation = OrientationResult::new(r_exact, 0.0, 0.0);
    let mesh_rotated = place_on_build_plate(&apply_orientation(&mesh, &exact_orientation));
    let bbox_rotated = bounding_box(&mesh_rotated);
    println!(
        "         bbox post-rotation+place_on_build_plate: \
         x ∈ [{:+.3}, {:+.3}], y ∈ [{:+.3}, {:+.3}], z ∈ [{:+.3}, {:+.3}]",
        bbox_rotated.0.x,
        bbox_rotated.1.x,
        bbox_rotated.0.y,
        bbox_rotated.1.y,
        bbox_rotated.0.z,
        bbox_rotated.1.z,
    );
    let validation_run2 = validate_for_printing(&mesh_rotated, &config_default)?;
    print_run_summary("Run 2", &validation_run2);
    let run2_area = total_overhang_area(&validation_run2);
    println!(
        "  Run 2 overhangs: {} regions, total area {run2_area:.6} mm²",
        validation_run2.overhangs.len(),
    );
    println!();

    // Anchor #4 — Run 2 overhangs.len() == 0 + area = 0 within 1e-6.
    assert_eq!(
        validation_run2.overhangs.len(),
        0,
        "anchor #4: Run 2 must produce zero OverhangRegions (cylinder perfectly vertical \
         post-exact-rotation; bottom cap flat on plate → all build-plate-filtered); got {}",
        validation_run2.overhangs.len(),
    );
    assert!(
        run2_area.abs() < 1e-6,
        "anchor #4: Run 2 total overhang area must be 0 within 1e-6 mm²; got {run2_area:.3e}",
    );

    // ─── Run 3: original mesh + with_build_up_direction(axis) ────────────
    println!("---- Run 3: original mesh + with_build_up_direction(axis) config ----");
    let config_axis_up = PrinterConfig::fdm_default().with_build_up_direction(axis);
    println!(
        "         config.build_up_direction = ({:+.4}, {:+.4}, {:+.4}) — aligned with cylinder axis",
        config_axis_up.build_up_direction.x,
        config_axis_up.build_up_direction.y,
        config_axis_up.build_up_direction.z,
    );
    let validation_run3 = validate_for_printing(&mesh, &config_axis_up)?;
    print_run_summary("Run 3", &validation_run3);
    let run3_area = total_overhang_area(&validation_run3);
    println!(
        "  Run 3 overhangs: {} regions, total area {run3_area:.6} mm²",
        validation_run3.overhangs.len(),
    );
    println!();

    // Anchor #5 — Run 3 overhangs.len() == 0 + area = 0 within 1e-6.
    assert_eq!(
        validation_run3.overhangs.len(),
        0,
        "anchor #5: Run 3 must produce zero OverhangRegions (up-vector aligned with axis → all \
         bottom-cap rim verts share axis-projection = mesh_min_along_up → all build-plate-\
         filtered); got {}",
        validation_run3.overhangs.len(),
    );
    assert!(
        run3_area.abs() < 1e-6,
        "anchor #5: Run 3 total overhang area must be 0 within 1e-6 mm²; got {run3_area:.3e}",
    );

    // Anchor #6 — Gap L invariant: Run 2 area ≡ Run 3 area within 1e-6.
    // Trivially 0 == 0 at the FP level; semantically the load-bearing claim
    // that mesh-rotation ≡ up-vector-rotation when both are exact.
    approx::assert_relative_eq!(run2_area, run3_area, epsilon = 1e-6);
    println!(
        "Gap L invariant ✓ : Run 2 area ({run2_area:.9} mm²) ≡ Run 3 area ({run3_area:.9} mm²) \
         within 1e-6"
    );

    // Anchor #7 — Run 2 area strictly less than Run 1 area.
    assert!(
        run2_area < run1_area,
        "anchor #7: Run 2 area ({run2_area:.6} mm²) must be strictly less than Run 1 area \
         ({run1_area:.3} mm²) — the exact rotation eliminates support need",
    );
    println!("Run 2 < Run 1 ✓  : {run2_area:.6} mm² < {run1_area:.3} mm²");
    println!();

    // ─── Diagnostic: find_optimal_orientation (printed, NOT asserted) ────
    println!(
        "---- Diagnostic: find_optimal_orientation(mesh, default_config, samples = {SEARCH_SAMPLES}) ----"
    );
    let search_result = find_optimal_orientation(&mesh, &config_default, SEARCH_SAMPLES);
    print_search_diagnostic(&search_result, run1_area);
    println!();

    // ─── Save 5 PLYs (anchor support for visuals-pass) ───────────────────
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    save_ply(&mesh, out_dir.join("mesh_original.ply"), false)?;
    save_ply(&mesh_rotated, out_dir.join("mesh_rotated.ply"), false)?;
    save_overhang_centroids(&validation_run1, &out_dir.join("issues_run1.ply"))?;
    save_overhang_centroids(&validation_run2, &out_dir.join("issues_run2.ply"))?;
    save_overhang_centroids(&validation_run3, &out_dir.join("issues_run3.ply"))?;

    println!("artifacts:");
    println!(
        "  out/mesh_original.ply : {}v, {}f (ASCII; leaning cylinder, post-place_on_build_plate)",
        mesh.vertices.len(),
        mesh.faces.len(),
    );
    println!(
        "  out/mesh_rotated.ply  : {}v, {}f (ASCII; manual R_Y(-60°) rotation + place_on_build_plate)",
        mesh_rotated.vertices.len(),
        mesh_rotated.faces.len(),
    );
    println!(
        "  out/issues_run1.ply   : {} centroid point(s) (ASCII, vertex-only)",
        validation_run1.overhangs.len(),
    );
    println!(
        "  out/issues_run2.ply   : {} centroid point(s) (ASCII, vertex-only; expected empty)",
        validation_run2.overhangs.len(),
    );
    println!(
        "  out/issues_run3.ply   : {} centroid point(s) (ASCII, vertex-only; expected empty)",
        validation_run3.overhangs.len(),
    );
    println!();
    println!("OK — Gap L orientation parametrization equivalence verified");

    Ok(())
}

/// Hand-author a CCW-from-outside cylinder along the `(axis, perp_u,
/// perp_v)` right-handed basis (`perp_u × perp_v = axis`). Pattern
/// duplicated from row #17 (`printability-self-intersecting`) per
/// `feedback_simplify_examples` (no shared-fixture-helpers; per-example
/// duplication).
///
/// **Vertex layout** (`2 + 2 × SEGMENTS = 66` total at SEGMENTS=32):
/// - `0`         negative-end cap centre (`-axis × LENGTH/2`)
/// - `1`         positive-end cap centre (`+axis × LENGTH/2`)
/// - `2..2+SEG`  negative rim ring (rim vertex `s` at azimuth
///   `θ_s = s × 2π / SEG`; position
///   `neg_centre + RADIUS × (perp_u·cos θ + perp_v·sin θ)`)
/// - `2+SEG..`   positive rim ring (same azimuth pattern at the
///   positive end)
///
/// **Face layout** (`4 × SEGMENTS = 128` total at SEGMENTS=32, OUTWARD
/// winding):
/// - **Negative cap fan** (SEG tris, normal `-axis`): per `s`,
///   `[neg_centre, neg_rim(s+1), neg_rim(s)]` — reversed-pair winding
///   so the cross product `(neg_rim(s+1) - neg_centre) × (neg_rim(s) -
///   neg_centre)` points along `-axis` (outward from the cap).
/// - **Positive cap fan** (SEG tris, normal `+axis`): per `s`,
///   `[pos_centre, pos_rim(s), pos_rim(s+1)]` — forward winding (rim
///   advances CCW when viewed from `+axis`).
/// - **Lateral strip** (2·SEG tris, normal radial-outward): per
///   segment `s`, two triangles `[neg_rim(s), neg_rim(s+1), pos_rim(s)]`
///   + `[neg_rim(s+1), pos_rim(s+1), pos_rim(s)]` — radial-outward
///     normals at azimuth `(s + 0.5) × 2π / SEG`.
///
/// Combined CCW-rim / reversed-neg-fan / forward-pos-fan / CCW-lateral
/// produces a watertight + consistently-wound shell: each undirected
/// edge is incident on exactly two faces, and each directed edge appears
/// in at most one face.
fn build_leaning_cylinder() -> IndexedMesh {
    make_cylinder(
        Point3::origin(),
        axis_unit(),
        perp_u_unit(),
        perp_v_unit(),
        LENGTH,
        RADIUS,
        SEGMENTS,
    )
}

fn make_cylinder(
    center: Point3<f64>,
    axis: Vector3<f64>,
    perp_u: Vector3<f64>,
    perp_v: Vector3<f64>,
    length: f64,
    radius: f64,
    segments: u32,
) -> IndexedMesh {
    let half_len = length / 2.0;
    let n = segments;
    let n_usize = n as usize;

    let mut vertices: Vec<Point3<f64>> = Vec::with_capacity(2 + 2 * n_usize);

    // Cap centres (indices 0, 1).
    let neg_centre = center + axis * (-half_len);
    let pos_centre = center + axis * half_len;
    vertices.push(neg_centre);
    vertices.push(pos_centre);

    // Rim vertices: negative ring (indices 2..2+SEG), then positive ring
    // (indices 2+SEG..2+2·SEG). Authored sequentially so the index
    // arithmetic in `neg_rim` / `pos_rim` below stays trivial.
    for end_offset in [-half_len, half_len] {
        let end_centre = center + axis * end_offset;
        for s in 0..n {
            let theta = f64::from(s) * std::f64::consts::TAU / f64::from(n);
            let rim_pt =
                end_centre + perp_u * (radius * theta.cos()) + perp_v * (radius * theta.sin());
            vertices.push(rim_pt);
        }
    }

    let mut faces: Vec<[u32; 3]> = Vec::with_capacity(4 * n_usize);

    let neg_centre_idx: u32 = 0;
    let pos_centre_idx: u32 = 1;
    let neg_rim = |s: u32| -> u32 { 2 + (s % n) };
    let pos_rim = |s: u32| -> u32 { 2 + n + (s % n) };

    for s in 0..n {
        let s_next = (s + 1) % n;
        // Negative cap (outward = -axis): reversed-pair winding.
        faces.push([neg_centre_idx, neg_rim(s_next), neg_rim(s)]);
        // Positive cap (outward = +axis): forward winding.
        faces.push([pos_centre_idx, pos_rim(s), pos_rim(s_next)]);
        // Lateral strip (outward = radial): two triangles per segment.
        faces.push([neg_rim(s), neg_rim(s_next), pos_rim(s)]);
        faces.push([neg_rim(s_next), pos_rim(s_next), pos_rim(s)]);
    }

    IndexedMesh::from_parts(vertices, faces)
}

/// Sum total overhang area across all `OverhangRegion`s. Used for the
/// per-run area-band assertions (anchor #1) and the Gap L invariant
/// (anchor #6). FP semantics: ordered sum (insertion order); per-region
/// area precision is the dominant FP error source, not the sum itself.
fn total_overhang_area(v: &PrintValidation) -> f64 {
    v.overhangs.iter().map(|r| r.area).sum()
}

/// Maximum overhang angle (degrees) across all regions. Used for Run 1's
/// stdout diagnostic surface; severity classification (anchor #2) goes
/// through `IssueSeverity` directly, not this aggregate.
fn max_overhang_angle(v: &PrintValidation) -> f64 {
    v.overhangs.iter().map(|r| r.angle).fold(0.0_f64, f64::max)
}

/// Compute axis-aligned bounding box of a mesh's vertex set. Helper for
/// the per-Run stdout diagnostic that surfaces the post-rotation /
/// post-placement bbox (lets the visuals-pass reader cross-check the
/// spatial layout without opening the PLY).
fn bounding_box(mesh: &IndexedMesh) -> (Point3<f64>, Point3<f64>) {
    let mut mn = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut mx = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for v in &mesh.vertices {
        mn.x = mn.x.min(v.x);
        mn.y = mn.y.min(v.y);
        mn.z = mn.z.min(v.z);
        mx.x = mx.x.max(v.x);
        mx.y = mx.y.max(v.y);
        mx.z = mx.z.max(v.z);
    }
    (mn, mx)
}

/// Print a per-Run validation summary block. Mirrors the prior examples'
/// `validation.summary()` + per-region listing pattern but trimmed to
/// the overhang-only surface this example needs (no `ThinWall` / cavity /
/// self-intersection / small-feature regions are expected here).
fn print_run_summary(label: &str, v: &PrintValidation) {
    println!("  {label} summary:");
    println!("    {}", v.summary().trim_end().replace('\n', "\n    "));
    if !v.overhangs.is_empty() {
        println!("  {label} OverhangRegions ({}):", v.overhangs.len());
        for (i, region) in v.overhangs.iter().enumerate() {
            println!(
                "    [{i}] center=({:+.4}, {:+.4}, {:+.4})  angle={:.3}°  area={:.3} mm²  faces={}",
                region.center.x,
                region.center.y,
                region.center.z,
                region.angle,
                region.area,
                region.faces.len(),
            );
        }
    }
    if !v.issues.is_empty() {
        println!("  {label} PrintIssues ({}):", v.issues.len());
        for (i, issue) in v.issues.iter().enumerate() {
            println!(
                "    [{i}] {:?} / {:?}  {}",
                issue.severity, issue.issue_type, issue.description,
            );
        }
    }
}

/// Print the diagnostic `find_optimal_orientation` output and the
/// pedagogical narrative around it. The picked rotation is printed as
/// (axis, angle in degrees) for human readability; the score components
/// are surfaced verbatim. The "(reduced … but does NOT reach 0)" call-
/// out lets the visuals-pass reader connect the diagnostic to the
/// "Why not `find_optimal_orientation` for Run 2?" README pitfall.
fn print_search_diagnostic(result: &OrientationResult, run1_area: f64) {
    let axis_opt = result.rotation.axis();
    let angle_deg = result.rotation.angle().to_degrees();
    if let Some(axis) = axis_opt {
        println!(
            "  picked rotation : axis ({:+.4}, {:+.4}, {:+.4}), angle {angle_deg:+.3}°",
            axis.x, axis.y, axis.z,
        );
    } else {
        println!("  picked rotation : identity (no axis; angle {angle_deg:+.3}°)");
    }
    println!(
        "  score           : {:.6}  (= support_volume {:.3} + overhang_area {:.3} × 0.1)",
        result.score, result.support_volume, result.overhang_area,
    );
    if result.overhang_area >= run1_area {
        println!(
            "  search did NOT improve on identity ({:.3} mm² ≥ Run 1 {:.3} mm²);",
            result.overhang_area, run1_area,
        );
        println!(
            "  the 12-sample default does not include R_Y(-60°) (the exact compensating \
             rotation),"
        );
        println!(
            "  and every rotation IT does include either preserves or worsens the overhang area \
             for this fixture"
        );
        println!("  (e.g., R_Y(-90°) drives lateral overhang to 0 but tilts the bottom cap 30°,");
        println!(
            "  flagging ~30 of 32 cap-fan triangles for a comparable post-rotation overhang)."
        );
    } else {
        println!(
            "  search reduced overhang from {run1_area:.3} mm² (Run 1) to {:.3} mm² but does NOT \
             reach 0;",
            result.overhang_area,
        );
        println!(
            "  the 12-sample default does not include R_Y(-60°) (the exact compensating \
             rotation),"
        );
        println!(
            "  so the picked rotation leaves residual overhang from the bottom cap or \
             non-perpendicular lateral arc."
        );
    }
    println!("  → see README \"Why isn't `find_optimal_orientation` Run 2?\" pitfall.");
}

/// Write per-Run overhang-region centroids as a vertex-only ASCII PLY.
/// Each `OverhangRegion`'s `center` (component-mean of per-face
/// centroids per `emit_overhang_component`) becomes one vertex. Empty
/// regions vector produces a vertex-only PLY with zero vertices and zero
/// faces — readable by `MeshLab` / `ParaView` / `Blender` / `f3d` as an
/// empty point cloud.
fn save_overhang_centroids(v: &PrintValidation, path: &Path) -> Result<()> {
    let centroids: Vec<Point3<f64>> = v.overhangs.iter().map(|r| r.center).collect();
    let mesh = IndexedMesh::from_parts(centroids, vec![]);
    save_ply(&mesh, path, false)?;
    Ok(())
}
