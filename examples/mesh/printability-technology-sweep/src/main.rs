//! Visual demo of cross-technology severity divergence on a single
//! fixture (§7.7 of the v0.8 fix arc spec, row #22).
//!
//! Hand-authors the same 24-triangle hollow box used at §7.1 (top wall
//! deliberately thinned to 0.4 mm; sealed inner cavity), scaled to
//! `25 × 20 × 15 mm` outer / `22 × 17 × 13.1 mm` inner. Validates the
//! mesh under all four `PrinterConfig::*_default()` technologies. The
//! load-bearing pedagogical claim is that the **same physical part
//! produces four different printability verdicts** — for four different
//! reasons.
//!
//! ## Per-tech severity matrix (asserted in `verify_<tech>`)
//!
//! | Tech | `min_wall` | `max_overhang` | `ThinWall` (wall = 0.4) | `TrappedVolume` | Cavity-ceiling overhang | `is_printable()` |
//! |------|-----------|----------------|-------------------------|-----------------|-------------------------|------------------|
//! | FDM  | 1.0 mm    | 45°            | 2× `Critical` (0.4 < 0.5)        | `Info`          | `Critical` (90 > 75)    | `false`          |
//! | SLA  | 0.4 mm    | 30°            | 0 flagged (0.4 ≥ 0.4 strict)     | `Critical`      | `Critical` (90 > 60)    | `false`          |
//! | SLS  | 0.7 mm    | 90°            | 2× `Warning` (0.35 ≤ 0.4 < 0.7)  | `Critical`      | NOT flagged (max = 90°) | `false`          |
//! | MJF  | 0.5 mm    | 90°            | 2× `Warning` (0.25 ≤ 0.4 < 0.5)  | `Critical`      | NOT flagged (max = 90°) | `false`          |
//!
//! Each tech fails `is_printable()` for a *different* reason: FDM is
//! blocked by `Critical` thin walls + cavity-ceiling overhang;
//! SLA by `Critical` cavity + cavity-ceiling overhang; SLS / MJF by
//! `Critical` cavity (overhang silent-skips because
//! `requires_supports() == false`).
//!
//! ## Why the 0.4 mm wall?
//!
//! The wall thickness is locked at exactly 0.4 mm to land each
//! technology's `min_wall_thickness` threshold on a *different* severity
//! band per §4.3 / `classify_thin_wall_severity`:
//!
//! - FDM (`min_wall = 1.0`): `0.4 < 1.0 / 2 = 0.5` → `Critical`.
//! - SLA (`min_wall = 0.4`): `0.4 < 0.4` is *false* (strict-less-than) →
//!   not flagged at all.
//! - SLS (`min_wall = 0.7`): `0.4 < 0.7` → flag; `0.4 < 0.7 / 2 = 0.35`
//!   is *false* → `Warning` (not `Critical`).
//! - MJF (`min_wall = 0.5`): `0.4 < 0.5` → flag; `0.4 < 0.5 / 2 = 0.25`
//!   is *false* → `Warning`.
//!
//! Picking 0.4 mm — exactly at SLA's threshold — makes the
//! strict-less-than convention visible to the reader: a wall *exactly*
//! at `min_wall` does NOT flag. A common pitfall when comparing
//! validators across pipelines.
//!
//! ## Why the sealed cavity?
//!
//! `classify_trapped_volume_severity` (`validation.rs:1435`) is
//! technology-aware: a sealed cavity is fine on FDM (extrusion doesn't
//! trap material) but a hard failure on SLA (uncured resin trap), SLS,
//! and MJF (unsintered powder trap). The `~ 4899.4 mm³` cavity volume
//! (f64 representation of `22 × 17 × 13.1`; `14.6` is not bit-exact in
//! IEEE-754) is comfortably above every tech's `min_feature_size³`
//! resolution floor
//! (FDM 0.512, SLA 0.001, SLS 0.027, MJF 0.008 mm³), so the resolution
//! short-circuit at `validation.rs:1441` never fires; the per-tech arm
//! determines the severity outright.
//!
//! ## Why the cavity-ceiling overhang co-flag?
//!
//! The inner-top face's normal points DOWN into the cavity — the
//! REVERSED winding rule for cavity shells (cavity normals point INTO
//! the cavity, away from the surrounding solid). The detector reads its
//! `overhang_angle` as 90° (face pointing straight down). FDM and SLA
//! `requires_supports()` is `true`, so `check_overhangs` runs and the
//! 90° face flags Critical (`90 > 45 + 30 = 75` for FDM,
//! `90 > 30 + 30 = 60` for SLA). SLS / MJF `requires_supports()` is
//! `false`, so `check_overhangs` early-returns at `validation.rs:304`
//! before the per-face loop runs — silent skip, no `DetectorSkipped`
//! issue.
//!
//! Validators see surface geometry, not interior intent — sealed-cavity
//! ceilings flag as overhang regardless of designer intent.
//!
//! ## Numerical anchors (asserted in `verify_<tech>`)
//!
//! Per tech in `[FDM, SLA, SLS, MJF]`:
//!
//! - `trapped_volumes.len() == 1`; voxel-discretized volume within
//!   `± 10 %` of analytical `22 × 17 × 13.1 ≈ 4899.4 mm³` (f64);
//!   centroid within per-tech `voxel_size` of `(12.5, 10, 8.05)`.
//! - No `DetectorSkipped` issue mentioning `TrappedVolume` (mesh is
//!   watertight; voxel-grid memory budget below the 1 GB cap on every
//!   tech).
//!
//! Per-tech specific:
//!
//! - **FDM**: `thin_walls.len() == 2` with both clusters Critical;
//!   `TrappedVolume` severity `Info`; `overhangs.len() == 1` (cavity
//!   ceiling) Critical; `!is_printable()`.
//! - **SLA**: `thin_walls.len() == 0` (strict-less-than boundary at
//!   0.4 == 0.4); `TrappedVolume` severity `Critical`;
//!   `overhangs.len() == 1` Critical; `!is_printable()`.
//! - **SLS**: `thin_walls.len() == 2` with both clusters Warning;
//!   `TrappedVolume` severity `Critical`; `overhangs.len() == 0`
//!   (silent skip); `!is_printable()`.
//! - **MJF**: same as SLS but with MJF thresholds.
//!
//! `LongBridge` may co-flag on FDM (`max_bridge_span = 10 mm`; cavity
//! ceiling is 22 mm × 17 mm, both > 10 mm) and SLA
//! (`max_bridge_span = 5 mm`); the example surfaces the count via stdout
//! diagnostics but does NOT assert against it (not load-bearing for the
//! per-tech severity story; SLS / MJF have `max_bridge_span = ∞`).
//!
//! ## Fixture-geometry anchors (closes the visuals-pass loop)
//!
//! `verify_fixture_geometry` (called once at the top of `main`) locks
//! every visible property of the hand-authored fixture as a numerical
//! invariant: per-vertex coordinates of all 16 vertices, per-face
//! cross-product unit normal of all 24 faces, and the overall mesh
//! bounding box `[0, OUTER_X] × [0, OUTER_Y] × [0, OUTER_Z]`. With
//! these in place, a successful `cargo run --release` exit-0 is
//! equivalent to a clean visual inspection — the visuals-pass becomes
//! a user-optional sanity check, not a workflow gate. This sets the
//! precedent for hand-authored printability example fixtures.
//!
//! ## How to run
//!
//! ```text
//! cargo run -p example-mesh-printability-technology-sweep --release
//! ```
//!
//! `--release` is required: SLA's voxel grid is `~ 1000 × 800 × 600 ≈
//! 480 MB` (well under the §6.3 1 GB cap, but heavy in debug). Output
//! is written to `examples/mesh/printability-technology-sweep/out/`:
//!
//! - `out/mesh.ply` — 16-vertex, 24-triangle ASCII PLY of the fixture.
//! - `out/issues_fdm.ply`, `out/issues_sla.ply`, `out/issues_sls.ply`,
//!   `out/issues_mjf.ply` — vertex-only ASCII PLYs of per-tech region
//!   centroids. Same mesh, four different point clouds — the visual
//!   payoff.
//!
//! With `verify_fixture_geometry` active, the visuals-pass is optional.
//! If you do want to eyeball the artifacts, run `f3d` on each PLY
//! separately (`f3d --up=+Z out/mesh.ply` then `f3d --up=+Z
//! out/issues_<tech>.ply`) — `f3d --multi-file-mode=all` mixed with a
//! point-cloud falls back to all-points rendering. See `README.md`.

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply;
use mesh_printability::{
    IssueSeverity, PrintIssueType, PrintValidation, PrinterConfig, validate_for_printing,
};
use mesh_types::{IndexedMesh, Point3};

// -- Geometry constants (mm) ------------------------------------------------

/// Outer box extents. The mesh sits at `x ∈ [0, OUTER_X]`,
/// `y ∈ [0, OUTER_Y]`, `z ∈ [0, OUTER_Z]`. The build plate is at
/// `z = 0`, so the outer bottom face is build-plate-filtered out of
/// `check_overhangs` per Gap M.2.
const OUTER_X: f64 = 25.0;
const OUTER_Y: f64 = 20.0;
const OUTER_Z: f64 = 15.0;

/// Inner cavity extents. Side and bottom walls are 1.5 mm thick;
/// the top wall is thinned to 0.4 mm so `INNER_Z_MAX = 14.6`. The
/// 0.4 mm wall is the load-bearing detail — it lands on each tech's
/// `min_wall_thickness` threshold at a different severity band.
const INNER_X_MIN: f64 = 1.5;
const INNER_X_MAX: f64 = 23.5;
const INNER_Y_MIN: f64 = 1.5;
const INNER_Y_MAX: f64 = 18.5;
const INNER_Z_MIN: f64 = 1.5;
const INNER_Z_MAX: f64 = 14.6;

/// Top-wall thickness — the load-bearing fixture parameter. Each tech's
/// `min_wall_thickness` lands this 0.4 mm value on a different severity
/// band per `classify_thin_wall_severity` (validation.rs:750).
const TOP_WALL_THICKNESS: f64 = OUTER_Z - INNER_Z_MAX; // = 0.4 mm

/// Analytical cavity volume. Mathematically `22 × 17 × 13.1 = 4900.4`,
/// but `14.6` is not bit-exact in IEEE-754 f64 (`14.6 - 1.5` evaluates
/// to `13.0999...`), so this constant is `≈ 4899.4 mm³`. The voxel-
/// discretized region must lie within `± TRAPPED_VOLUME_REL_TOL` of
/// the f64 value; for axis-aligned cavities aligned to voxel edges the
/// discretization is bit-exact at this f64 value.
const ANALYTICAL_CAVITY_VOLUME: f64 =
    (INNER_X_MAX - INNER_X_MIN) * (INNER_Y_MAX - INNER_Y_MIN) * (INNER_Z_MAX - INNER_Z_MIN);

// -- Fixture-geometry expected anchors --------------------------------------

/// Expected per-vertex coordinates (mm). Locks the fixture's 16-vertex
/// hand-authored construction: 8 outer cube corners (indices `0..=7`)
/// and 8 inner cavity corners (indices `8..=15`, vertex-disjoint from
/// outer). Each entry must match `mesh.vertices[i]` within `VERTEX_TOL`.
/// Independent description vs `build_hollow_box` implementation —
/// catches typos in the vertex array.
const EXPECTED_VERTICES: [[f64; 3]; 16] = [
    [0.0, 0.0, 0.0],                         //  0  outer bottom-front-left
    [OUTER_X, 0.0, 0.0],                     //  1  outer bottom-front-right
    [OUTER_X, OUTER_Y, 0.0],                 //  2  outer bottom-back-right
    [0.0, OUTER_Y, 0.0],                     //  3  outer bottom-back-left
    [0.0, 0.0, OUTER_Z],                     //  4  outer top-front-left
    [OUTER_X, 0.0, OUTER_Z],                 //  5  outer top-front-right
    [OUTER_X, OUTER_Y, OUTER_Z],             //  6  outer top-back-right
    [0.0, OUTER_Y, OUTER_Z],                 //  7  outer top-back-left
    [INNER_X_MIN, INNER_Y_MIN, INNER_Z_MIN], //  8  inner bottom-front-left
    [INNER_X_MAX, INNER_Y_MIN, INNER_Z_MIN], //  9  inner bottom-front-right
    [INNER_X_MAX, INNER_Y_MAX, INNER_Z_MIN], // 10  inner bottom-back-right
    [INNER_X_MIN, INNER_Y_MAX, INNER_Z_MIN], // 11  inner bottom-back-left
    [INNER_X_MIN, INNER_Y_MIN, INNER_Z_MAX], // 12  inner top-front-left
    [INNER_X_MAX, INNER_Y_MIN, INNER_Z_MAX], // 13  inner top-front-right
    [INNER_X_MAX, INNER_Y_MAX, INNER_Z_MAX], // 14  inner top-back-right
    [INNER_X_MIN, INNER_Y_MAX, INNER_Z_MAX], // 15  inner top-back-left
];

/// Expected per-face unit normal directions. Outer shell wound CCW-
/// from-outside (normals OUTWARD); inner shell REVERSED (normals point
/// INTO cavity, AWAY from surrounding solid). Each entry must match the
/// cross-product unit normal of `mesh.faces[i]` within `NORMAL_TOL`.
/// Catches winding bugs (e.g. face wound clockwise) that detector-level
/// assertions wouldn't surface — only visual inspection or geometric
/// anchors would.
const EXPECTED_FACE_NORMALS: [[f64; 3]; 24] = [
    // Outer shell, OUTWARD
    [0.0, 0.0, -1.0], //  0  outer bottom  [0, 3, 2]   normal -z
    [0.0, 0.0, -1.0], //  1  outer bottom  [0, 2, 1]
    [0.0, 0.0, 1.0],  //  2  outer top     [4, 5, 6]   normal +z
    [0.0, 0.0, 1.0],  //  3  outer top     [4, 6, 7]
    [0.0, -1.0, 0.0], //  4  outer front   [0, 1, 5]   normal -y
    [0.0, -1.0, 0.0], //  5  outer front   [0, 5, 4]
    [0.0, 1.0, 0.0],  //  6  outer back    [3, 7, 6]   normal +y
    [0.0, 1.0, 0.0],  //  7  outer back    [3, 6, 2]
    [-1.0, 0.0, 0.0], //  8  outer left    [0, 4, 7]   normal -x
    [-1.0, 0.0, 0.0], //  9  outer left    [0, 7, 3]
    [1.0, 0.0, 0.0],  // 10  outer right   [1, 2, 6]   normal +x
    [1.0, 0.0, 0.0],  // 11  outer right   [1, 6, 5]
    // Inner shell, INTO cavity (REVERSED)
    [0.0, 0.0, 1.0],  // 12  inner bottom  [8, 9, 10]   +z (cavity above)
    [0.0, 0.0, 1.0],  // 13  inner bottom  [8, 10, 11]
    [0.0, 0.0, -1.0], // 14  inner top     [12, 14, 13] -z (cavity below)
    [0.0, 0.0, -1.0], // 15  inner top     [12, 15, 14]
    [0.0, 1.0, 0.0],  // 16  inner front   [8, 12, 13]  +y (cavity behind)
    [0.0, 1.0, 0.0],  // 17  inner front   [8, 13, 9]
    [0.0, -1.0, 0.0], // 18  inner back    [11, 10, 14] -y (cavity in front)
    [0.0, -1.0, 0.0], // 19  inner back    [11, 14, 15]
    [1.0, 0.0, 0.0],  // 20  inner left    [8, 11, 15]  +x (cavity to right)
    [1.0, 0.0, 0.0],  // 21  inner left    [8, 15, 12]
    [-1.0, 0.0, 0.0], // 22  inner right   [9, 13, 14]  -x (cavity to left)
    [-1.0, 0.0, 0.0], // 23  inner right   [9, 14, 10]
];

// -- Numerical-anchor tolerances --------------------------------------------

/// Tolerance on `ThinWallRegion.thickness` (mm). The detector reports
/// `min_dist + EPS_RAY_OFFSET` where `EPS_RAY_OFFSET = 1e-6` (the inward
/// ray-start offset is added back so a wall *exactly* at `min_wall`
/// does NOT flag — strict-less-than). 1e-5 leaves an order of magnitude
/// of headroom for cross-platform IEEE-754 add ordering.
const THICKNESS_TOL: f64 = 1.0e-5;

/// Tolerance on cluster centroid coordinates (mm). Fixture vertex
/// coordinates are exact-representable in f64 and the unweighted face-
/// centroid mean is exact arithmetic on those representations, so 1e-9
/// is comfortably tight while leaving headroom for IEEE-754 add ordering.
const CENTROID_TOL: f64 = 1.0e-9;

/// Tolerance on cluster area (mm²). Per-face area is `‖edge1 × edge2‖ / 2`
/// for axis-aligned right triangles — geometrically exact, no chord error.
const AREA_TOL: f64 = 1.0e-9;

/// Relative tolerance for the voxel-discretized cavity volume against
/// the analytical `22 × 17 × 13.1`. The 10 % band absorbs cross-platform
/// FP drift in the §6.3 voxel inside-test (parity-flip on row offsets —
/// the `ROW_JITTER_Y / ROW_JITTER_Z` constants — could shift voxel
/// classifications by `± 1` voxel on platform boundaries) per §9.6.
const TRAPPED_VOLUME_REL_TOL: f64 = 0.10;

/// Tolerance on per-vertex coordinate equality (mm) for fixture-
/// geometry assertions. All 16 vertex coordinates are exact-representable
/// in f64 except `INNER_Z_MAX = 14.6`, which has IEEE-754 drift of
/// `~ 1e-16`; `1e-12` is comfortably tight.
const VERTEX_TOL: f64 = 1.0e-12;

/// Tolerance on per-face unit normal equality for fixture-geometry
/// assertions. Cross-product on axis-aligned-vertex coordinates produces
/// bit-exact `±1.0` / `0.0` components after normalization for axis-
/// aligned right-triangle faces; `1e-12` is comfortably tight.
const NORMAL_TOL: f64 = 1.0e-12;

// -- Helpers ----------------------------------------------------------------

/// Per-tech `voxel_size = min(min_feature_size, layer_height) / 2` per
/// §6.3. The `TrappedVolumeRegion` centroid is the mean of voxel
/// centres, which sits within `voxel_size` of the analytical centre by
/// symmetry + half-voxel discretization headroom.
fn voxel_size(config: &PrinterConfig) -> f64 {
    config.min_feature_size.min(config.layer_height) / 2.0
}

// `clippy::similar_names`: `fdm_validation` / `sla_validation` /
// `sls_validation` / `mjf_validation` differ only by the per-tech tag,
// but the per-tech verb IS the load-bearing distinction; renaming to
// e.g. `fdm_v` / `sla_v` would obscure the per-tech intent and trade
// one similar-pair for another (row #15 precedent).
//
// `clippy::too_many_lines`: per-tech runs + 4-way verification + PLY
// emission gives main() its length; the pedagogical flow is clearest
// when it reads top-to-bottom in one function (row #21 precedent).
#[allow(clippy::similar_names, clippy::too_many_lines)]
fn main() -> Result<()> {
    let mesh = build_hollow_box();
    verify_fixture_geometry(&mesh);

    println!("==== mesh-printability-technology-sweep ====");
    println!();
    println!(
        "input  : {}-vertex, {}-triangle hollow box",
        mesh.vertices.len(),
        mesh.faces.len(),
    );
    println!(
        "         outer {OUTER_X}×{OUTER_Y}×{OUTER_Z} mm; inner cavity x ∈ [{INNER_X_MIN}, {INNER_X_MAX}], y ∈ [{INNER_Y_MIN}, {INNER_Y_MAX}], z ∈ [{INNER_Z_MIN}, {INNER_Z_MAX}]",
    );
    println!(
        "         top wall thinned to {TOP_WALL_THICKNESS:.1} mm; side and bottom walls 1.5 mm",
    );
    println!("         analytical cavity volume = {ANALYTICAL_CAVITY_VOLUME:.4} mm³");
    println!();

    let fdm = PrinterConfig::fdm_default();
    let sla = PrinterConfig::sla_default();
    let sls = PrinterConfig::sls_default();
    let mjf = PrinterConfig::mjf_default();

    let fdm_validation = run_tech("FDM", &mesh, &fdm)?;
    verify_shared_anchors(&fdm_validation, &fdm, "FDM");
    verify_fdm(&fdm_validation);

    let sla_validation = run_tech("SLA", &mesh, &sla)?;
    verify_shared_anchors(&sla_validation, &sla, "SLA");
    verify_sla(&sla_validation);

    let sls_validation = run_tech("SLS", &mesh, &sls)?;
    verify_shared_anchors(&sls_validation, &sls, "SLS");
    verify_sls(&sls_validation);

    let mjf_validation = run_tech("MJF", &mesh, &mjf)?;
    verify_shared_anchors(&mjf_validation, &mjf, "MJF");
    verify_mjf(&mjf_validation);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let mesh_path = out_dir.join("mesh.ply");
    save_ply(&mesh, &mesh_path, false)?;

    save_issue_centroids(&fdm_validation, &out_dir.join("issues_fdm.ply"))?;
    save_issue_centroids(&sla_validation, &out_dir.join("issues_sla.ply"))?;
    save_issue_centroids(&sls_validation, &out_dir.join("issues_sls.ply"))?;
    save_issue_centroids(&mjf_validation, &out_dir.join("issues_mjf.ply"))?;

    println!();
    println!("artifacts:");
    println!(
        "  out/mesh.ply        : {}v, {}f (ASCII)",
        mesh.vertices.len(),
        mesh.faces.len(),
    );
    println!(
        "  out/issues_fdm.ply  : {} centroid point(s) (ASCII, vertex-only)",
        issue_centroid_count(&fdm_validation),
    );
    println!(
        "  out/issues_sla.ply  : {} centroid point(s) (ASCII, vertex-only)",
        issue_centroid_count(&sla_validation),
    );
    println!(
        "  out/issues_sls.ply  : {} centroid point(s) (ASCII, vertex-only)",
        issue_centroid_count(&sls_validation),
    );
    println!(
        "  out/issues_mjf.ply  : {} centroid point(s) (ASCII, vertex-only)",
        issue_centroid_count(&mjf_validation),
    );
    println!();
    println!("OK — cross-tech severity divergence verified");

    Ok(())
}

/// Hand-author the 24-triangle hollow box (mirrors §7.1's pattern,
/// scaled to `25 × 20 × 15` outer / `22 × 17 × 13.1` inner).
///
/// Two vertex-disjoint shells (outer 0..=7, inner 8..=15). Each shell is
/// independently watertight; the union has every undirected edge in
/// exactly two faces (always intra-outer or intra-inner). Outer winding
/// is CCW-from-outside (normals OUTWARD); inner winding is REVERSED
/// (normals point INTO the cavity, AWAY from the surrounding solid).
/// Both shells together pass the watertight + consistent-winding
/// preconditions §6.1 and §6.3 require.
fn build_hollow_box() -> IndexedMesh {
    let vertices = vec![
        // Outer cube (8 corners)
        Point3::new(0.0, 0.0, 0.0),             // 0  bottom-front-left
        Point3::new(OUTER_X, 0.0, 0.0),         // 1  bottom-front-right
        Point3::new(OUTER_X, OUTER_Y, 0.0),     // 2  bottom-back-right
        Point3::new(0.0, OUTER_Y, 0.0),         // 3  bottom-back-left
        Point3::new(0.0, 0.0, OUTER_Z),         // 4  top-front-left
        Point3::new(OUTER_X, 0.0, OUTER_Z),     // 5  top-front-right
        Point3::new(OUTER_X, OUTER_Y, OUTER_Z), // 6  top-back-right
        Point3::new(0.0, OUTER_Y, OUTER_Z),     // 7  top-back-left
        // Inner cavity (8 corners, vertex-disjoint at indices 8..=15)
        Point3::new(INNER_X_MIN, INNER_Y_MIN, INNER_Z_MIN), // 8
        Point3::new(INNER_X_MAX, INNER_Y_MIN, INNER_Z_MIN), // 9
        Point3::new(INNER_X_MAX, INNER_Y_MAX, INNER_Z_MIN), // 10
        Point3::new(INNER_X_MIN, INNER_Y_MAX, INNER_Z_MIN), // 11
        Point3::new(INNER_X_MIN, INNER_Y_MIN, INNER_Z_MAX), // 12
        Point3::new(INNER_X_MAX, INNER_Y_MIN, INNER_Z_MAX), // 13
        Point3::new(INNER_X_MAX, INNER_Y_MAX, INNER_Z_MAX), // 14
        Point3::new(INNER_X_MIN, INNER_Y_MAX, INNER_Z_MAX), // 15
    ];

    let faces: Vec<[u32; 3]> = vec![
        // ─── Outer shell, CCW-from-outside (normals OUTWARD) ──────────
        // Bottom (z = 0, normal −z) — build-plate-filtered in check_overhangs
        [0, 3, 2],
        [0, 2, 1],
        // Top (z = OUTER_Z, normal +z) — load-bearing thin face #1
        [4, 5, 6],
        [4, 6, 7],
        // Front (y = 0, normal −y)
        [0, 1, 5],
        [0, 5, 4],
        // Back (y = OUTER_Y, normal +y)
        [3, 7, 6],
        [3, 6, 2],
        // Left (x = 0, normal −x)
        [0, 4, 7],
        [0, 7, 3],
        // Right (x = OUTER_X, normal +x)
        [1, 2, 6],
        [1, 6, 5],
        // ─── Inner shell, REVERSED winding (normals point INTO cavity) ─
        // Inner bottom (z = INNER_Z_MIN, normal +z, into cavity above)
        [8, 9, 10],
        [8, 10, 11],
        // Inner top (z = INNER_Z_MAX, normal −z, into cavity below) —
        // load-bearing thin face #2; cavity-ceiling overhang co-flag.
        [12, 14, 13],
        [12, 15, 14],
        // Inner front (y = INNER_Y_MIN, normal +y, into cavity behind)
        [8, 12, 13],
        [8, 13, 9],
        // Inner back (y = INNER_Y_MAX, normal −y, into cavity in front)
        [11, 10, 14],
        [11, 14, 15],
        // Inner left (x = INNER_X_MIN, normal +x, into cavity to right)
        [8, 11, 15],
        [8, 15, 12],
        // Inner right (x = INNER_X_MAX, normal −x, into cavity to left)
        [9, 13, 14],
        [9, 14, 10],
    ];

    IndexedMesh::from_parts(vertices, faces)
}

/// Verify the fixture's hand-authored geometry — locks the per-vertex
/// coordinates, per-face winding, and overall bounding box. Catches
/// regressions that detector-level assertions wouldn't surface (typos
/// in the vertex array, swapped winding on a face, scale-factor bug).
/// With this verifier active, the visual inspection step is no longer
/// load-bearing: every visible property of the fixture is encoded as a
/// numerical invariant, so a successful `cargo run --release` exit-0 is
/// equivalent to a clean visuals-pass.
///
/// Three logical groups:
/// 1. **Per-vertex coordinates** — all 16 vertices match
///    `EXPECTED_VERTICES`.
/// 2. **Per-face winding** — all 24 cross-product unit normals match
///    `EXPECTED_FACE_NORMALS` (outer shell OUTWARD; inner shell INTO
///    cavity).
/// 3. **Mesh bounding box** — `[0, OUTER_X] × [0, OUTER_Y] × [0,
///    OUTER_Z]`.
//
// `clippy::suboptimal_flops`: the cross-product `a*b - c*d` form is
// preferred here over `a.mul_add(b, -(c*d))` — the operands are integer
// vertex coordinates so both forms produce bit-exact results, but the
// straight-line form reads as the textbook cross-product definition.
#[allow(clippy::suboptimal_flops)]
fn verify_fixture_geometry(mesh: &IndexedMesh) {
    // (1) Vertex count + per-vertex coordinates.
    assert_eq!(
        mesh.vertices.len(),
        EXPECTED_VERTICES.len(),
        "fixture must have {} vertices",
        EXPECTED_VERTICES.len(),
    );
    for (i, expected) in EXPECTED_VERTICES.iter().enumerate() {
        let v = &mesh.vertices[i];
        assert_relative_eq!(v.x, expected[0], epsilon = VERTEX_TOL);
        assert_relative_eq!(v.y, expected[1], epsilon = VERTEX_TOL);
        assert_relative_eq!(v.z, expected[2], epsilon = VERTEX_TOL);
    }

    // (2) Face count + per-face cross-product unit normal direction.
    assert_eq!(
        mesh.faces.len(),
        EXPECTED_FACE_NORMALS.len(),
        "fixture must have {} faces",
        EXPECTED_FACE_NORMALS.len(),
    );
    for (i, expected) in EXPECTED_FACE_NORMALS.iter().enumerate() {
        let face = mesh.faces[i];
        let p0 = mesh.vertices[face[0] as usize];
        let p1 = mesh.vertices[face[1] as usize];
        let p2 = mesh.vertices[face[2] as usize];
        let edge1 = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z];
        let edge2 = [p2.x - p0.x, p2.y - p0.y, p2.z - p0.z];
        let normal = [
            edge1[1] * edge2[2] - edge1[2] * edge2[1],
            edge1[2] * edge2[0] - edge1[0] * edge2[2],
            edge1[0] * edge2[1] - edge1[1] * edge2[0],
        ];
        let len = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
        assert!(
            len > 0.0,
            "face {i} has degenerate cross product (zero-area triangle)",
        );
        let unit = [normal[0] / len, normal[1] / len, normal[2] / len];
        assert_relative_eq!(unit[0], expected[0], epsilon = NORMAL_TOL);
        assert_relative_eq!(unit[1], expected[1], epsilon = NORMAL_TOL);
        assert_relative_eq!(unit[2], expected[2], epsilon = NORMAL_TOL);
    }

    // (3) Mesh bounding box: full outer extent.
    let min_x = mesh
        .vertices
        .iter()
        .map(|v| v.x)
        .fold(f64::INFINITY, f64::min);
    let min_y = mesh
        .vertices
        .iter()
        .map(|v| v.y)
        .fold(f64::INFINITY, f64::min);
    let min_z = mesh
        .vertices
        .iter()
        .map(|v| v.z)
        .fold(f64::INFINITY, f64::min);
    let max_x = mesh
        .vertices
        .iter()
        .map(|v| v.x)
        .fold(f64::NEG_INFINITY, f64::max);
    let max_y = mesh
        .vertices
        .iter()
        .map(|v| v.y)
        .fold(f64::NEG_INFINITY, f64::max);
    let max_z = mesh
        .vertices
        .iter()
        .map(|v| v.z)
        .fold(f64::NEG_INFINITY, f64::max);
    assert_relative_eq!(min_x, 0.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(min_y, 0.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(min_z, 0.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(max_x, OUTER_X, epsilon = VERTEX_TOL);
    assert_relative_eq!(max_y, OUTER_Y, epsilon = VERTEX_TOL);
    assert_relative_eq!(max_z, OUTER_Z, epsilon = VERTEX_TOL);
}

/// Run `validate_for_printing` for one technology and surface
/// diagnostics to stdout. Mirrors the trapped-volume crate's per-tech
/// print-and-verify pattern.
fn run_tech(label: &str, mesh: &IndexedMesh, config: &PrinterConfig) -> Result<PrintValidation> {
    let validation = validate_for_printing(mesh, config)?;
    println!(
        "---- {label} (min_wall = {:.2} mm, max_overhang = {:.0}°, min_feature = {:.2} mm, voxel = {:.4} mm) ----",
        config.min_wall_thickness,
        config.max_overhang_angle,
        config.min_feature_size,
        voxel_size(config),
    );
    println!("{}", validation.summary());
    print_diagnostics(&validation);
    println!();
    Ok(validation)
}

/// Print region/issue diagnostics to stdout. Surfaces the trapped
/// region's centroid + volume, the per-thin-wall cluster centroid +
/// thickness + area, the per-overhang cluster centroid + angle + area,
/// and the issues list (which includes any `LongBridge` co-flags).
fn print_diagnostics(v: &PrintValidation) {
    println!("ThinWall regions ({}):", v.thin_walls.len());
    for (i, region) in v.thin_walls.iter().enumerate() {
        println!(
            "  [{i}] center=({:+.4}, {:+.4}, {:+.4})  thickness={:.6} mm  area={:.3} mm²  faces={}",
            region.center.x,
            region.center.y,
            region.center.z,
            region.thickness,
            region.area,
            region.faces.len(),
        );
    }
    println!("TrappedVolume regions ({}):", v.trapped_volumes.len());
    for (i, region) in v.trapped_volumes.iter().enumerate() {
        let (bb_min, bb_max) = region.bounding_box;
        println!(
            "  [{i}] center=({:+.4}, {:+.4}, {:+.4})  volume={:.3} mm³  voxels={}  bbox=[{:+.3}..{:+.3}, {:+.3}..{:+.3}, {:+.3}..{:+.3}]",
            region.center.x,
            region.center.y,
            region.center.z,
            region.volume,
            region.voxel_count,
            bb_min.x,
            bb_max.x,
            bb_min.y,
            bb_max.y,
            bb_min.z,
            bb_max.z,
        );
    }
    println!("Overhang regions ({}):", v.overhangs.len());
    for (i, region) in v.overhangs.iter().enumerate() {
        println!(
            "  [{i}] center=({:+.4}, {:+.4}, {:+.4})  angle={:.3}°  area={:.3} mm²  faces={}",
            region.center.x,
            region.center.y,
            region.center.z,
            region.angle,
            region.area,
            region.faces.len(),
        );
    }
    println!("Issues ({}):", v.issues.len());
    for issue in &v.issues {
        println!(
            "  [{:?} / {:?}] {}",
            issue.severity, issue.issue_type, issue.description,
        );
    }
}

/// Verify the §7.7 anchors that hold across all four technologies — the
/// trapped region exists, has the right volume, and has the right
/// centroid. The detector runs with a different `voxel_size` per tech
/// so the centroid tolerance is per-tech.
fn verify_shared_anchors(v: &PrintValidation, config: &PrinterConfig, label: &str) {
    // (1) Exactly one trapped region — the sealed cavity.
    assert_eq!(
        v.trapped_volumes.len(),
        1,
        "{label}: sealed inner cavity must produce exactly one TrappedVolume region",
    );

    let region = &v.trapped_volumes[0];

    // (2) Voxel-discretized cavity volume within ± 10 % of analytical
    // `22 × 17 × 13.1 ≈ 4899.4 mm³` (f64; mathematical 4900.4 — see
    // `ANALYTICAL_CAVITY_VOLUME`'s doc for the IEEE-754 drift). For
    // axis-aligned cube cavities aligned to voxel edges the
    // discretization is bit-exact; the 10 % band is cross-platform
    // headroom per §9.6.
    assert_relative_eq!(
        region.volume,
        ANALYTICAL_CAVITY_VOLUME,
        max_relative = TRAPPED_VOLUME_REL_TOL,
    );

    // (3) Cavity centroid at the analytical midpoint
    // `(OUTER_X / 2, OUTER_Y / 2, midpoint(INNER_Z_MIN, INNER_Z_MAX))
    // = (12.5, 10, 8.05)` within per-tech `voxel_size`.
    let tol = voxel_size(config);
    assert_relative_eq!(region.center.x, OUTER_X / 2.0, epsilon = tol);
    assert_relative_eq!(region.center.y, OUTER_Y / 2.0, epsilon = tol);
    let expected_centroid_z = f64::midpoint(INNER_Z_MIN, INNER_Z_MAX);
    assert_relative_eq!(region.center.z, expected_centroid_z, epsilon = tol);

    // (4) `voxel_count > 0` — sanity guard that the detector ran (no
    // `DetectorSkipped` short-circuit).
    assert!(
        region.voxel_count > 0,
        "{label}: detector must have populated voxel_count",
    );

    // (5) No `DetectorSkipped` issue mentioning `TrappedVolume` — the
    // mesh is watertight and the voxel-grid memory budget sits below
    // the 1 GB cap on every tech (FDM ~ 8 MB, SLA ~ 480 MB,
    // SLS ~ 60 MB, MJF ~ 117 MB at the 25×20×15 outer extents).
    let trapped_skipped = v
        .issues
        .iter()
        .filter(|i| {
            i.issue_type == PrintIssueType::DetectorSkipped
                && i.description.contains("TrappedVolume")
        })
        .count();
    assert_eq!(
        trapped_skipped, 0,
        "{label}: TrappedVolume detector must not skip on a watertight 25×20×15 hollow box",
    );

    // (6) `is_printable() == false` on every tech — the load-bearing
    // claim of §7.7 is *each* tech fails, but for *different* reasons.
    assert!(
        !v.is_printable(),
        "{label}: every tech must fail is_printable() — the cross-tech divergence claim",
    );
}

/// FDM verification — `ThinWall` × 2 Critical (`0.4 < 1.0 / 2 = 0.5`),
/// `TrappedVolume` `Info` (sealed cavities print fine on extrusion),
/// cavity-ceiling overhang × 1 Critical (`90 > 45 + 30 = 75`).
/// `is_printable() == false` driven by Critical thin walls + overhang.
fn verify_fdm(v: &PrintValidation) {
    verify_thin_wall_clusters(v, IssueSeverity::Critical, "FDM");
    assert_trapped_severity(v, IssueSeverity::Info, "FDM");
    verify_cavity_ceiling_overhang(v, "FDM");
}

/// SLA verification — `ThinWall` not flagged at all (`0.4 < 0.4` is
/// false; strict-less-than boundary), `TrappedVolume` `Critical`
/// (uncured-resin trap), cavity-ceiling overhang × 1 Critical
/// (`90 > 30 + 30 = 60`). `is_printable() == false` driven by Critical
/// `TrappedVolume` + overhang.
fn verify_sla(v: &PrintValidation) {
    // The 0.4 mm wall is exactly at SLA's `min_wall_thickness = 0.4`;
    // the §6.1 precondition (`thickness < min_wall_thickness`) is
    // strict-less-than, so neither cluster flags. This is the
    // load-bearing strict-less-than-boundary demonstration.
    assert_eq!(
        v.thin_walls.len(),
        0,
        "SLA: 0.4 mm wall at exactly min_wall_thickness = 0.4 must NOT flag (strict-less-than boundary)",
    );

    assert_trapped_severity(v, IssueSeverity::Critical, "SLA");
    verify_cavity_ceiling_overhang(v, "SLA");
}

/// SLS verification — `ThinWall` × 2 Warning (`0.4 < 0.7` flag;
/// `0.4 < 0.7 / 2 = 0.35` false → Warning), `TrappedVolume` `Critical`
/// (unsintered-powder trap), cavity-ceiling overhang skipped silently
/// (`SLS::requires_supports() == false` ⇒ `check_overhangs` early-
/// returns at `validation.rs:304` BEFORE the per-face loop).
/// `is_printable() == false` driven solely by Critical `TrappedVolume`.
fn verify_sls(v: &PrintValidation) {
    verify_thin_wall_clusters(v, IssueSeverity::Warning, "SLS");
    assert_trapped_severity(v, IssueSeverity::Critical, "SLS");

    assert_eq!(
        v.overhangs.len(),
        0,
        "SLS: requires_supports() == false ⇒ check_overhangs silent-skips (no DetectorSkipped issue)",
    );
}

/// MJF verification — same severity bands as SLS but with MJF
/// thresholds (`min_wall = 0.5` so `0.4 < 0.25` false → Warning;
/// `requires_supports() == false` ⇒ overhang silent-skip).
fn verify_mjf(v: &PrintValidation) {
    verify_thin_wall_clusters(v, IssueSeverity::Warning, "MJF");
    assert_trapped_severity(v, IssueSeverity::Critical, "MJF");

    assert_eq!(
        v.overhangs.len(),
        0,
        "MJF: requires_supports() == false ⇒ check_overhangs silent-skips",
    );
}

/// Assert exactly two `ThinWall` clusters — outer-top + inner-top,
/// edge-adjacency partitions into exactly two components because the
/// shells are vertex-disjoint (no shared edges). Both clusters share
/// the same physical thickness (`OUTER_Z - INNER_Z_MAX = 0.4 mm`); both
/// surface at the expected severity. Centroids are bit-exact at the
/// analytical face-mid:
///
/// - Outer cluster: `(OUTER_X / 2, OUTER_Y / 2, OUTER_Z) = (12.5, 10, 15)`.
/// - Inner cluster: `(OUTER_X / 2, OUTER_Y / 2, INNER_Z_MAX) =
///   (12.5, 10, 14.6)`.
///
/// Areas are exact for axis-aligned right triangles:
///
/// - Outer cluster: `OUTER_X × OUTER_Y = 500 mm²`.
/// - Inner cluster: `(INNER_X_MAX - INNER_X_MIN) × (INNER_Y_MAX -
///   INNER_Y_MIN) = 22 × 17 = 374 mm²`.
fn verify_thin_wall_clusters(v: &PrintValidation, expected_sev: IssueSeverity, label: &str) {
    assert_eq!(
        v.thin_walls.len(),
        2,
        "{label}: outer-top + inner-top ThinWall clusters; edge-adjacency partitions into 2 via disjoint shells",
    );

    // Sort ascending by `center.z`: inner cluster (z = 14.6) first,
    // outer cluster (z = 15) second.
    let mut sorted: Vec<&_> = v.thin_walls.iter().collect();
    sorted.sort_by(|a, b| a.center.z.total_cmp(&b.center.z));
    let inner = sorted[0];
    let outer = sorted[1];

    assert_relative_eq!(inner.center.x, OUTER_X / 2.0, epsilon = CENTROID_TOL);
    assert_relative_eq!(inner.center.y, OUTER_Y / 2.0, epsilon = CENTROID_TOL);
    assert_relative_eq!(inner.center.z, INNER_Z_MAX, epsilon = CENTROID_TOL);
    assert_relative_eq!(outer.center.x, OUTER_X / 2.0, epsilon = CENTROID_TOL);
    assert_relative_eq!(outer.center.y, OUTER_Y / 2.0, epsilon = CENTROID_TOL);
    assert_relative_eq!(outer.center.z, OUTER_Z, epsilon = CENTROID_TOL);

    assert_relative_eq!(outer.area, OUTER_X * OUTER_Y, epsilon = AREA_TOL);
    let inner_area_expected = (INNER_X_MAX - INNER_X_MIN) * (INNER_Y_MAX - INNER_Y_MIN);
    assert_relative_eq!(inner.area, inner_area_expected, epsilon = AREA_TOL);

    for region in &v.thin_walls {
        assert_relative_eq!(
            region.thickness,
            TOP_WALL_THICKNESS,
            epsilon = THICKNESS_TOL,
        );
    }

    let matching_severity = v
        .issues
        .iter()
        .filter(|i| i.issue_type == PrintIssueType::ThinWall && i.severity == expected_sev)
        .count();
    assert_eq!(
        matching_severity, 2,
        "{label}: expected exactly two {expected_sev:?} ThinWall issues, one per cluster",
    );
}

/// Assert exactly one `TrappedVolume` issue surfaces with the given
/// severity. Used by every per-tech verifier.
fn assert_trapped_severity(v: &PrintValidation, expected: IssueSeverity, label: &str) {
    let count = v
        .issues
        .iter()
        .filter(|i| i.issue_type == PrintIssueType::TrappedVolume && i.severity == expected)
        .count();
    assert_eq!(
        count, 1,
        "{label}: expected exactly one {expected:?} TrappedVolume issue",
    );
}

/// Assert the cavity-ceiling overhang fires exactly once at Critical
/// severity. The inner-top face (REVERSED winding ⇒ normal `−z`,
/// `overhang_angle = 90°`) clusters its 2 edge-adjacent triangles into
/// one `OverhangRegion`. FDM (`max = 45°`, `90 > 45 + 30 = 75`) and SLA
/// (`max = 30°`, `90 > 30 + 30 = 60`) both classify Critical.
fn verify_cavity_ceiling_overhang(v: &PrintValidation, label: &str) {
    assert_eq!(
        v.overhangs.len(),
        1,
        "{label}: cavity ceiling (inner top, normal −z) must flag exactly one OverhangRegion (2 tris edge-adjacent within inner shell)",
    );

    let region = &v.overhangs[0];
    assert_relative_eq!(region.center.x, OUTER_X / 2.0, epsilon = CENTROID_TOL);
    assert_relative_eq!(region.center.y, OUTER_Y / 2.0, epsilon = CENTROID_TOL);
    assert_relative_eq!(region.center.z, INNER_Z_MAX, epsilon = CENTROID_TOL);

    let critical_overhangs = v
        .issues
        .iter()
        .filter(|i| {
            i.issue_type == PrintIssueType::ExcessiveOverhang
                && i.severity == IssueSeverity::Critical
        })
        .count();
    assert_eq!(
        critical_overhangs, 1,
        "{label}: cavity-ceiling overhang must surface as exactly one Critical ExcessiveOverhang issue",
    );
}

/// Write region centroids as a vertex-only ASCII PLY. Per §7.0's
/// per-example helper template; duplicated per-example (not factored
/// out) per `feedback_simplify_examples`.
///
/// Aggregates `thin_walls` + `overhangs` + `support_regions` +
/// `trapped_volumes` — the populated region collections after the v0.8
/// detector arc. `LongBridge` is intentionally OMITTED (per §7.1
/// thin-wall + §7.3 trapped-volume precedent): its centroid is the
/// cluster-bbox midpoint, not a per-region "issue location" point in
/// the same sense as the other detectors.
fn save_issue_centroids(v: &PrintValidation, path: &Path) -> Result<()> {
    let mut centroids: Vec<Point3<f64>> = Vec::new();
    centroids.extend(v.thin_walls.iter().map(|r| r.center));
    centroids.extend(v.overhangs.iter().map(|r| r.center));
    centroids.extend(v.support_regions.iter().map(|r| r.center));
    centroids.extend(v.trapped_volumes.iter().map(|r| r.center));
    let mesh = IndexedMesh::from_parts(centroids, vec![]);
    save_ply(&mesh, path, false)?;
    Ok(())
}

/// Number of region centroids written to `out/issues_<tech>.ply`.
const fn issue_centroid_count(v: &PrintValidation) -> usize {
    v.thin_walls.len() + v.overhangs.len() + v.support_regions.len() + v.trapped_volumes.len()
}
