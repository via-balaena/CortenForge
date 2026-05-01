//! Visual demo of the §6.3 `TrappedVolume` detector (Gap H).
//!
//! Hand-authors a 20×20×20 mm solid cube with a sealed `SPHERE_RADIUS`-mm
//! spherical cavity at its centre — outer cube (12 outward-wound tris)
//! plus an interior UV-tessellated sphere shell (32 segs × 16 stacks =
//! 960 inward-wound tris) — and validates it under all four
//! `PrinterConfig::*_default` technologies. Saves `out/mesh.ply` (the
//! input fixture) and `out/issues.ply` (centroids from the FDM
//! iteration, the richest cluster of points).
//!
//! ## Why a sealed sphere cavity inside a solid cube
//!
//! The §6.3 `TrappedVolume` detector flood-fills voxels from the padded
//! grid corner; voxels classified as inside the mesh (per-row scanline
//! ray-tri parity) but unreachable from the corner are "trapped". A
//! sealed cavity is the cleanest fixture: the cube's outer shell shows
//! up as one bounding surface; the cavity's inner shell surrounds an
//! interior region the flood-fill can't reach. The tessellated sphere
//! demonstrates the detector working on a non-axis-aligned cavity
//! (unlike the axis-aligned thin-wall fixture's box cavity at §7.1).
//!
//! ## Inner-shell winding rule
//!
//! Each face's normal must point AWAY from the surrounding solid:
//!
//! - Outer cube faces: normals point OUTWARD (CCW-from-outside).
//! - Inner sphere faces: normals point INTO the cavity (toward the
//!   sphere centre), i.e. REVERSED relative to a standalone outward-
//!   wound sphere.
//!
//! Both shells are individually watertight + consistently wound, and
//! they are vertex-disjoint, so the combined mesh's edge-incidence is
//! count = 2 everywhere (the manifold + watertight preconditions §6.3
//! requires).
//!
//! ## Multi-technology severity sweep — the load-bearing payload
//!
//! `classify_trapped_volume_severity` (`validation.rs:1415`) is
//! technology-aware: a sealed cavity prints fine on FDM (extrusion
//! doesn't trap material) but is a hard failure on SLA (uncured resin
//! traps), SLS, and MJF (unsintered powder traps). The detector reports
//! the same region on all four techs but classifies severity per the
//! `PrintTechnology` arm.
//!
//! | Tech | `TrappedVolume` | Cavity-ceiling overhang | `is_printable()` |
//! |------|-----------------|-------------------------|------------------|
//! | FDM  | `Info`          | `Critical` (90 > 75)    | `false` (overhang) |
//! | SLA  | `Critical`      | `Critical` (90 > 60)    | `false` (both)     |
//! | SLS  | `Critical`      | NOT flagged (max=90°)   | `false` (trapped)  |
//! | MJF  | `Critical`      | NOT flagged (max=90°)   | `false` (trapped)  |
//!
//! Each tech fails `is_printable()` but for different reasons — the
//! pedagogical point of the example.
//!
//! ## Multi-detector co-flag — cavity-ceiling overhang
//!
//! A sealed sphere cavity's upper hemisphere has faces whose inward
//! (cavity-facing) normal points DOWNWARD (toward the cube floor). For
//! FDM (`max_overhang_angle = 45°`) and SLA (`30°`), the upper-cap
//! cluster of faces flags as Critical `ExcessiveOverhang` — peak
//! observed `overhang_angle ≈ 84°` on a 32 × 16 UV-tessellated sphere
//! (the polar tri's centroid normal is ~6° off vertical due to chord
//! shrinkage; v0.8 spec §7.3 line 1639 quoted ~90° as a continuum
//! limit). For SLS / MJF (`max=90°`), the strict `>` predicate at
//! `validation.rs:381` means even 84° does NOT flag → 0 overhangs.
//!
//! Validators see surface geometry, not interior intent — sealed-cavity
//! ceilings flag as overhang regardless of designer intent. The README
//! documents this as a design pattern.
//!
//! ## Numerical anchors (asserted in `main`)
//!
//! - Per tech in `[FDM, SLA, SLS, MJF]`:
//!   - `trapped_volumes.len() == 1`
//!   - Voxel-discretized cavity volume within `±10 %` of analytical
//!     `(4/3) π r³ ≈ 523.6 mm³` (§9.6 cross-platform-headroom band)
//!   - Cavity centroid within per-tech `voxel_size` of `(10, 10, 10)`
//! - FDM: `TrappedVolume` severity `Info`; ≥ 1 Critical `ExcessiveOverhang`;
//!   `!is_printable()` (driven by overhang)
//! - SLA: `TrappedVolume` severity `Critical`; ≥ 1 Critical `ExcessiveOverhang`;
//!   `!is_printable()` (both Critical)
//! - SLS: `TrappedVolume` severity `Critical`; `overhangs.len() == 0`;
//!   `!is_printable()` (trapped)
//! - MJF: `TrappedVolume` severity `Critical`; `overhangs.len() == 0`;
//!   `!is_printable()` (trapped)
//!
//! ## How to run
//!
//! ```text
//! cargo run -p example-mesh-printability-trapped-volume --release
//! ```
//!
//! Output written to `examples/mesh/printability-trapped-volume/out/`.
//! Open `mesh.ply` and `issues.ply` in `MeshLab` or `ParaView` for the
//! visuals pass — see the README's f3d-winding callout for viewer-
//! specific notes on the inner sphere's REVERSED winding (f3d back-
//! face-culls the cavity by default).
//!
//! ## v0.8 spec deviation: `out/voxels.ply` deferred
//!
//! v0.8 spec §7.3 line 1665 calls for a third PLY artifact — a point-
//! cloud of the trapped voxel centres for visualizing the discretized
//! cavity shape. v0.8's `TrappedVolumeRegion` (regions.rs:153) exposes
//! only `center / volume / bounding_box / voxel_count` — the individual
//! voxel centres live in the detector's internal `VoxelGrid::states`
//! (validation.rs:1442) and are not surfaced. Extending the public API
//! to expose them is v0.9 candidate scope; row #15's mandate is
//! example-only (no source change to mesh-printability), so v0.8 ships
//! the cavity centroid in `issues.ply` and defers the per-voxel point-
//! cloud.

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply;
use mesh_printability::{
    IssueSeverity, PrintIssueType, PrintValidation, PrinterConfig, validate_for_printing,
};
use mesh_types::{IndexedMesh, Point3};

// -- Geometry constants (mm) ------------------------------------------------

/// Outer cube edge length. Cube spans `[0, OUTER_SIZE]` on every axis.
const OUTER_SIZE: f64 = 20.0;

/// Cavity sphere radius. Centred at `(OUTER_SIZE / 2, OUTER_SIZE / 2,
/// OUTER_SIZE / 2)` so the cavity is interior on all axes (5 mm wall to
/// every cube face — well above every tech's `min_wall_thickness` so
/// `ThinWall` does not co-flag).
const SPHERE_RADIUS: f64 = 5.0;

/// Cavity sphere centre — geometric mid-point of the outer cube.
const SPHERE_CX: f64 = OUTER_SIZE / 2.0;
const SPHERE_CY: f64 = OUTER_SIZE / 2.0;
const SPHERE_CZ: f64 = OUTER_SIZE / 2.0;

/// Number of longitudinal segments around each ring (32). A reasonable
/// resolution: chord-vs-arc shrinkage `cos(π/32)³ ≈ 0.985` gives a
/// tessellated volume ≈ `0.985 × (4/3) π r³ ≈ 516 mm³` — within the
/// `± 10 %` analytical-volume tolerance below.
const SPHERE_SEGS: u32 = 32;

/// Number of latitudinal bands between the poles (16). With
/// `SPHERE_SEGS × SPHERE_STACKS = 32 × 16` the sphere has 482 vertices
/// and 960 triangles (32 north-pole fans + 14 × 64 middle-band quads +
/// 32 south-pole fans). Combined with the 8 cube vertices + 12 cube
/// triangles the fixture totals 490 vertices + 972 triangles.
const SPHERE_STACKS: u32 = 16;

// -- Numerical-anchor tolerances --------------------------------------------

/// Relative tolerance for the voxel-discretized cavity volume against
/// the analytical sphere `(4/3) π r³`. The 10 % band absorbs (a) UV-
/// tessellation chord shrinkage (~ 1.5 %) and (b) cross-platform FP
/// drift in the §6.3 voxel inside-test parity flips (the `ROW_JITTER_Y
/// / ROW_JITTER_Z` constants could shift voxel classifications by
/// ± 1 voxel on platform boundaries) per §9.6.
const TRAPPED_VOLUME_REL_TOL: f64 = 0.10;

/// Per-tech `voxel_size` from §6.3:
/// `voxel_size = min(min_feature_size, layer_height) / 2`.
///
/// The `TrappedVolumeRegion` centroid is the mean of voxel centres,
/// which sits within `voxel_size` of the analytical centre by symmetry
/// + half-voxel discretization headroom.
fn voxel_size(config: &PrinterConfig) -> f64 {
    config.min_feature_size.min(config.layer_height) / 2.0
}

/// Analytical cavity volume `(4/3) π r³ ≈ 523.598 mm³`. Inlined per
/// tech-iteration to avoid float-const-eval edge cases on older toolchains.
fn analytical_sphere_volume() -> f64 {
    (4.0 / 3.0) * std::f64::consts::PI * SPHERE_RADIUS.powi(3)
}

// `clippy::similar_names`: `sla_validation` and `sls_validation` differ by a
// single character but the per-tech verb is the load-bearing distinction;
// renaming to e.g. `sla_v` / `sls_v` would obscure the per-tech intent and
// trade one similar-pair for another.
#[allow(clippy::similar_names)]
fn main() -> Result<()> {
    let mesh = build_cube_with_sphere_cavity();

    println!("==== mesh-printability-trapped-volume ====");
    println!();
    println!(
        "input  : {}-vertex, {}-triangle solid cube with sealed sphere cavity",
        mesh.vertices.len(),
        mesh.faces.len()
    );
    println!(
        "         outer cube {OUTER_SIZE}×{OUTER_SIZE}×{OUTER_SIZE} mm; cavity sphere r = {SPHERE_RADIUS} mm at ({SPHERE_CX}, {SPHERE_CY}, {SPHERE_CZ})"
    );
    println!(
        "         sphere tessellation: {SPHERE_SEGS} segs × {SPHERE_STACKS} stacks (REVERSED winding ⇒ normals point INTO cavity)"
    );
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
    let issues_path = out_dir.join("issues.ply");
    save_ply(&mesh, &mesh_path, false)?;
    // Save the FDM iteration's centroids — richest co-flag (TrappedVolume +
    // ExcessiveOverhang + SupportRegion) of the four runs.
    save_issue_centroids(&fdm_validation, &issues_path)?;

    println!();
    println!("artifacts:");
    println!(
        "  out/mesh.ply   : {}v, {}f (ASCII)",
        mesh.vertices.len(),
        mesh.faces.len(),
    );
    println!(
        "  out/issues.ply : {} centroid point(s) (ASCII, vertex-only; FDM iteration)",
        issue_centroid_count(&fdm_validation),
    );
    println!();
    println!("OK — TrappedVolume 4-tech severity demonstration verified");

    Ok(())
}

/// Run `validate_for_printing` for one technology and surface diagnostics
/// to stdout. Mirrors the per-tech print-and-verify pattern.
fn run_tech(label: &str, mesh: &IndexedMesh, config: &PrinterConfig) -> Result<PrintValidation> {
    let validation = validate_for_printing(mesh, config)?;
    println!(
        "---- {label} (max_overhang = {:.0}°, min_feature = {:.2} mm, voxel = {:.4} mm) ----",
        config.max_overhang_angle,
        config.min_feature_size,
        voxel_size(config)
    );
    println!("{}", validation.summary());
    print_diagnostics(&validation);
    println!();
    Ok(validation)
}

/// Hand-author the cube + sphere-cavity fixture.
///
/// **Vertex layout** (490 total):
/// - `0..=7`   outer cube corners (axis-aligned, vertices at
///   `{0, OUTER_SIZE}³`)
/// - `8`       sphere north pole at `(SPHERE_CX, SPHERE_CY, SPHERE_CZ +
///   SPHERE_RADIUS)`
/// - `9`       sphere south pole at `(SPHERE_CX, SPHERE_CY, SPHERE_CZ -
///   SPHERE_RADIUS)`
/// - `10..490` sphere ring vertices; `ring(stack, seg)` lives at index
///   `10 + (stack - 1) * SPHERE_SEGS + seg` for `stack in
///   1..SPHERE_STACKS` and `seg in 0..SPHERE_SEGS`. 15 rings × 32 segs =
///   480 ring vertices.
///
/// **Face layout** (972 total):
/// - 12 outer cube tris (CCW-from-outside, normals OUTWARD)
/// - 32 north-pole fans (REVERSED winding, normals point INTO cavity)
/// - 14 × 64 = 896 middle-band tris (REVERSED)
/// - 32 south-pole fans (REVERSED)
///
/// Sphere total: 32 + 896 + 32 = 960 tris. The shells are vertex-
/// disjoint (cube uses 0..=7, sphere uses 8..490), so each shell's edge-
/// incidence is independent. Each shell is individually watertight +
/// consistently wound; their union is therefore watertight + manifold
/// with every edge in exactly two faces.
fn build_cube_with_sphere_cavity() -> IndexedMesh {
    let mut vertices: Vec<Point3<f64>> = Vec::with_capacity(490);

    // ─── Outer cube vertices (indices 0..=7) ─────────────────────────────
    vertices.push(Point3::new(0.0, 0.0, 0.0)); //          0  bottom-front-left
    vertices.push(Point3::new(OUTER_SIZE, 0.0, 0.0)); //   1  bottom-front-right
    vertices.push(Point3::new(OUTER_SIZE, OUTER_SIZE, 0.0)); // 2  bottom-back-right
    vertices.push(Point3::new(0.0, OUTER_SIZE, 0.0)); //   3  bottom-back-left
    vertices.push(Point3::new(0.0, 0.0, OUTER_SIZE)); //   4  top-front-left
    vertices.push(Point3::new(OUTER_SIZE, 0.0, OUTER_SIZE)); // 5  top-front-right
    vertices.push(Point3::new(OUTER_SIZE, OUTER_SIZE, OUTER_SIZE)); // 6  top-back-right
    vertices.push(Point3::new(0.0, OUTER_SIZE, OUTER_SIZE)); // 7  top-back-left

    // ─── Sphere poles (indices 8, 9) ────────────────────────────────────
    vertices.push(Point3::new(SPHERE_CX, SPHERE_CY, SPHERE_CZ + SPHERE_RADIUS)); // 8  north pole
    vertices.push(Point3::new(SPHERE_CX, SPHERE_CY, SPHERE_CZ - SPHERE_RADIUS)); // 9  south pole

    // ─── Sphere rings (indices 10..490) ─────────────────────────────────
    // Ring `stack` (1..SPHERE_STACKS) sits at colatitude
    // `θ_stack = stack × π / SPHERE_STACKS` (0 = north, π = south).
    // Segment `seg` (0..SPHERE_SEGS) sits at azimuth
    // `φ_seg = seg × 2π / SPHERE_SEGS` (CCW around +z axis, +x at φ=0).
    for stack in 1..SPHERE_STACKS {
        let theta = f64::from(stack) * std::f64::consts::PI / f64::from(SPHERE_STACKS);
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();
        for seg in 0..SPHERE_SEGS {
            let phi = f64::from(seg) * std::f64::consts::TAU / f64::from(SPHERE_SEGS);
            vertices.push(Point3::new(
                SPHERE_RADIUS.mul_add(sin_theta * phi.cos(), SPHERE_CX),
                SPHERE_RADIUS.mul_add(sin_theta * phi.sin(), SPHERE_CY),
                SPHERE_RADIUS.mul_add(cos_theta, SPHERE_CZ),
            ));
        }
    }

    // ─── Outer cube faces (CCW-from-outside, normals OUTWARD) ──────────
    let mut faces: Vec<[u32; 3]> = vec![
        // Bottom (z = 0, normal -z)
        [0, 3, 2],
        [0, 2, 1],
        // Top (z = OUTER_SIZE, normal +z)
        [4, 5, 6],
        [4, 6, 7],
        // Front (y = 0, normal -y)
        [0, 1, 5],
        [0, 5, 4],
        // Back (y = OUTER_SIZE, normal +y)
        [3, 7, 6],
        [3, 6, 2],
        // Left (x = 0, normal -x)
        [0, 4, 7],
        [0, 7, 3],
        // Right (x = OUTER_SIZE, normal +x)
        [1, 2, 6],
        [1, 6, 5],
    ];

    // ─── Sphere faces (REVERSED winding ⇒ normals point INTO cavity) ───
    let north_pole: u32 = 8;
    let south_pole: u32 = 9;
    let ring = |stack: u32, seg: u32| -> u32 {
        // Caller passes `stack in 1..SPHERE_STACKS` and `seg in
        // 0..SPHERE_SEGS`; both fit in u32 well below the 480-vertex bound.
        10 + (stack - 1) * SPHERE_SEGS + seg
    };

    // North-pole fan (32 tris). REVERSED winding for an outward-pointing
    // OUTWARD-from-pole tri would be `[NP, j, j+1]`; flipping the last
    // two vertex indices ⇒ `[NP, j+1, j]` gives an inward-into-cavity
    // normal (verified by hand cross-product at construction time).
    for seg in 0..SPHERE_SEGS {
        let next = (seg + 1) % SPHERE_SEGS;
        faces.push([north_pole, ring(1, next), ring(1, seg)]);
    }

    // Middle bands (14 bands × 64 tris = 896 tris). For each quad with
    // corners `a = (stack, seg)`, `b = (stack, seg+1)`, `c = (stack+1,
    // seg)`, `d = (stack+1, seg+1)`, the OUTWARD-wound triangulation is
    // `[a, c, b]` + `[b, c, d]`; REVERSED winding swaps to `[a, b, c]` +
    // `[b, d, c]`, giving inward-into-cavity normals.
    for stack in 1..(SPHERE_STACKS - 1) {
        for seg in 0..SPHERE_SEGS {
            let next = (seg + 1) % SPHERE_SEGS;
            let a = ring(stack, seg);
            let b = ring(stack, next);
            let c = ring(stack + 1, seg);
            let d = ring(stack + 1, next);
            faces.push([a, b, c]);
            faces.push([b, d, c]);
        }
    }

    // South-pole fan (32 tris). OUTWARD-wound would be `[SP, j+1, j]`;
    // REVERSED ⇒ `[SP, j, j+1]` gives inward-into-cavity normal.
    let last_ring = SPHERE_STACKS - 1;
    for seg in 0..SPHERE_SEGS {
        let next = (seg + 1) % SPHERE_SEGS;
        faces.push([south_pole, ring(last_ring, seg), ring(last_ring, next)]);
    }

    IndexedMesh::from_parts(vertices, faces)
}

/// Print region/issue diagnostics to stdout. Surfaces the trapped
/// region's centroid + volume + voxel count + bbox; the per-overhang
/// centroid + angle + area; and the issues themselves.
fn print_diagnostics(v: &PrintValidation) {
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

/// Verify the §7.3 numerical anchors that hold across all four
/// technologies — the trapped region exists, has the right volume, and
/// has the right centroid. Each tech runs the §6.3 detector with a
/// different `voxel_size` (per `voxel_size(config)`) so the centroid
/// tolerance is per-tech.
fn verify_shared_anchors(v: &PrintValidation, config: &PrinterConfig, label: &str) {
    // (1) Exactly one trapped region — the sealed sphere cavity.
    assert_eq!(
        v.trapped_volumes.len(),
        1,
        "{label}: sealed sphere cavity must produce exactly one TrappedVolume region",
    );

    let region = &v.trapped_volumes[0];

    // (2) Voxel-discretized volume within ± 10 % of analytical
    // `(4/3) π r³`. UV-tessellation chord shrinkage drops the actual
    // tessellated volume by ~ 1.5 %; voxel discretization drops it
    // another fraction; the band absorbs both plus cross-platform FP
    // drift.
    let analytical = analytical_sphere_volume();
    assert_relative_eq!(
        region.volume,
        analytical,
        max_relative = TRAPPED_VOLUME_REL_TOL,
    );

    // (3) Cavity centroid within `voxel_size` of `(SPHERE_CX, SPHERE_CY,
    // SPHERE_CZ)`. By symmetry the mean of trapped voxel centres equals
    // the analytical centre up to half-voxel discretization headroom +
    // tessellation asymmetry; `voxel_size` is the spec band (§7.3
    // assertion #1).
    let tol = voxel_size(config);
    assert_relative_eq!(region.center.x, SPHERE_CX, epsilon = tol);
    assert_relative_eq!(region.center.y, SPHERE_CY, epsilon = tol);
    assert_relative_eq!(region.center.z, SPHERE_CZ, epsilon = tol);

    // (4) `voxel_count > 0` — sanity guard that the detector ran (no
    // `DetectorSkipped` short-circuit). The `trapped_volumes.len() == 1`
    // check above already implies this, but the skip path could in
    // principle still emit a region if a future detector revision
    // changed the contract; surface it as a separate invariant.
    assert!(
        region.voxel_count > 0,
        "{label}: detector must have populated voxel_count",
    );

    // (5) No `DetectorSkipped` issue mentioning `TrappedVolume` — the
    // mesh is watertight and the voxel-grid memory budget is well below
    // the 1 GB cap (FDM ~ 8 MB, SLA ~ 515 MB, SLS ~ 65 MB, MJF ~ 125 MB).
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
        "{label}: TrappedVolume detector must not skip on a watertight cube + sphere cavity",
    );
}

/// FDM: `TrappedVolume` `Info` (sealed cavities print fine on extrusion);
/// cavity ceiling fires Critical `ExcessiveOverhang` (`90 > 45 + 30 =
/// 75` Critical band); `is_printable() == false` driven by overhang.
fn verify_fdm(v: &PrintValidation) {
    assert_trapped_severity(v, IssueSeverity::Info, "FDM");

    // Cavity-ceiling overhang fires Critical. The detector's edge-
    // adjacency cluster split (Gap D) groups the upper-cap flagged
    // faces; assert ≥ 1 cluster (the exact count depends on the chord-
    // error pattern at the polar tris and is not load-bearing).
    assert!(
        !v.overhangs.is_empty(),
        "FDM: cavity ceiling (upper hemisphere of REVERSED-wound sphere) must flag ≥ 1 ExcessiveOverhang cluster",
    );
    let critical_overhangs = v
        .issues
        .iter()
        .filter(|i| {
            i.issue_type == PrintIssueType::ExcessiveOverhang
                && i.severity == IssueSeverity::Critical
        })
        .count();
    assert!(
        critical_overhangs >= 1,
        "FDM: cavity-ceiling overhang must surface as ≥ 1 Critical ExcessiveOverhang issue (90 > 45 + 30 = 75 Critical band)",
    );

    assert!(
        !v.is_printable(),
        "FDM: Critical ExcessiveOverhang must block is_printable() (TrappedVolume is Info on FDM)",
    );
}

/// SLA: `TrappedVolume` `Critical` (uncured-resin trap); cavity ceiling
/// also Critical (`90 > 30 + 30 = 60` Critical band); `is_printable()
/// == false` driven by both.
fn verify_sla(v: &PrintValidation) {
    assert_trapped_severity(v, IssueSeverity::Critical, "SLA");

    assert!(
        !v.overhangs.is_empty(),
        "SLA: cavity ceiling must flag ≥ 1 ExcessiveOverhang cluster (max=30°)",
    );
    let critical_overhangs = v
        .issues
        .iter()
        .filter(|i| {
            i.issue_type == PrintIssueType::ExcessiveOverhang
                && i.severity == IssueSeverity::Critical
        })
        .count();
    assert!(
        critical_overhangs >= 1,
        "SLA: cavity-ceiling overhang must surface as ≥ 1 Critical issue (90 > 30 + 30 = 60 Critical band)",
    );

    assert!(
        !v.is_printable(),
        "SLA: Critical TrappedVolume + Critical ExcessiveOverhang each independently block is_printable()",
    );
}

/// SLS: `TrappedVolume` `Critical` (unsintered-powder trap); cavity
/// ceiling does NOT flag (SLS `requires_supports() == false` ⇒
/// `check_overhangs` early-returns at `validation.rs:284` BEFORE the
/// per-face predicate; even if it ran, `max=90°` + strict `>` would
/// reject the polar tris' ~ 84° peak overhang); `is_printable() ==
/// false` driven solely by `TrappedVolume`.
fn verify_sls(v: &PrintValidation) {
    assert_trapped_severity(v, IssueSeverity::Critical, "SLS");

    assert_eq!(
        v.overhangs.len(),
        0,
        "SLS: requires_supports() == false ⇒ check_overhangs silent-skips (no DetectorSkipped issue)",
    );

    assert!(
        !v.is_printable(),
        "SLS: Critical TrappedVolume must block is_printable() (overhangs silent-skip)",
    );
}

/// MJF: `TrappedVolume` `Critical` (unsintered-powder trap); cavity
/// ceiling silent-skips like SLS. `is_printable() == false` driven by
/// `TrappedVolume`.
fn verify_mjf(v: &PrintValidation) {
    assert_trapped_severity(v, IssueSeverity::Critical, "MJF");

    assert_eq!(
        v.overhangs.len(),
        0,
        "MJF: requires_supports() == false ⇒ check_overhangs silent-skips",
    );

    assert!(
        !v.is_printable(),
        "MJF: Critical TrappedVolume must block is_printable() (overhangs silent-skip)",
    );
}

/// Assert that exactly one `TrappedVolume` issue surfaces with the given
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

/// Write region centroids as a vertex-only ASCII PLY. Per §7.0's
/// per-example helper template; duplicated per-example (not factored
/// out) per `feedback_simplify_examples`.
///
/// Aggregates `thin_walls` + `overhangs` + `support_regions` +
/// `trapped_volumes` — the populated region collections after the v0.8
/// detector arc. `LongBridge` is intentionally OMITTED (per row #14b's
/// printability-thin-wall precedent): its centroid is the cluster-bbox
/// midpoint, not a per-region "issue location" point in the same sense
/// as the other detectors. For this fixture (sealed cavity, no
/// downward-facing faces with extent ≥ `max_bridge_span`), `LongBridge`
/// would never fire on any tech anyway.
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

/// Number of region centroids written to `out/issues.ply`.
const fn issue_centroid_count(v: &PrintValidation) -> usize {
    v.thin_walls.len() + v.overhangs.len() + v.support_regions.len() + v.trapped_volumes.len()
}
