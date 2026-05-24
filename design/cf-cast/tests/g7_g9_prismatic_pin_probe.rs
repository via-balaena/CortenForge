//! §G-7 + §G-9 probe-spike for the cf-cast FDM-friendly geometry
//! recon-1 arc (`docs/CF_CAST_FDM_FRIENDLY_GEOMETRY_RECON.md`).
//!
//! Per [[feedback-implement-measure-revert-pattern]] this probe lands
//! before any S2 `PrismaticPin` primitive code so the recon's
//! BRANCH A / BRANCH B / BRANCH C decision tree settles against
//! empirical manifold3d behaviour, not against intuition. Survives or
//! gets pruned at the S2 implementation session.
//!
//! # Empirical outcome (this probe, on `dev`)
//!
//! - **§G-7 BRANCH B + BRANCH C BOTH fire at production cell size**
//!   (3 mm). Mesh-CSG union of a truncated-pyramid PrismaticPin onto
//!   an SDF→MC half-sphere shell host adds 2 components AND introduces
//!   a `SelfIntersecting` F4 Critical issue. Mesh-CSG subtract adds 1
//!   component (the carved socket becomes a detached cavity-bottom
//!   shell). The exact symptoms reproduce at the **over-resolved**
//!   1 mm control cell size — so the failure is **not** MC-quantization
//!   alone, it's the mesh-CSG-union of a sharp-cornered convex
//!   polyhedron against the MC-tessellated curved-shell host
//!   (recon-4's "bulk-welded MC × fine mesh-CSG" paradigm-boundary
//!   pattern, see [[project-cf-cast-sdf-meshcsg-paradigm-boundary]]).
//! - **§G-9 chamfer sweep inherits §G-7's BRANCH determination.** All
//!   chamfer values in {0.0, 0.4, 0.6, 0.8, 1.0 mm} fail identically
//!   under the §G-7 default mesh-CSG path. The §G-9 CSG-vs-slicer
//!   question CANNOT be characterised independently until §G-12's
//!   bail-out lands; once it does, §G-9 should be re-probed against
//!   the bail-out pin geometry.
//! - **§G-12 #2 bail-out PASSES** the BRANCH-A criteria. With the pin
//!   composed pre-MC into the host SDF (`bounding ∖ body ∩ halfspace ∪
//!   pin`) → MC → mesh, the result is 1 connected component, no new
//!   F4 Critical types at the boolean junction. This matches the
//!   recon-4 (P) §F-3 SDF-union pin characterisation.
//!
//! Conclusion: §G-12 #2 is the implementation path. §G-12 #1 (post-MC
//! mesh-CSG pre-mixed with pre-MC SDF for the pin BULK only) was the
//! recon's first-pass bail-out hypothesis; the empirical evidence here
//! pushes one step further — the whole PrismaticPin primitive lives in
//! SDF, not just its bulk. The chamfer-band geometry can still emit as
//! SDF (`Solid::cuboid` + smooth-min taper or per-axis half-extent
//! reductions); the §G-9 CSG-level chamfer decision remains intact as
//! long as the chamfer band is SDF-side, not mesh-CSG-side.
//!
//! # Probe construction
//!
//! Host: hollow half-sphere shell — `sphere(R).shell(t) ∩ halfspace(z>0)`
//! built via `Solid` → `ScalarGrid` → `marching_cubes`. The
//! `Solid::shell(t)` operator builds a `2t`-thick shell centred on
//! the original surface (per `evaluate.rs:107`:
//! `|f(p)| - thickness`). For R = 30 mm and t = 5 mm the wall spans
//! `|p| ∈ [25, 35] mm`, total 10 mm thick. The seam ring at z = 0
//! joins outer and inner surfaces → ONE connected surface =
//! recon-4 (P) §F-3b baseline.
//!
//! Pin: per §G-7 doc spec — base half-extent 1.5 mm, tip half-extent
//! 1.2 mm, half-length 3 mm, chamfer 0.5 mm. Built via
//! `Manifold::hull_pts` over a 12-vertex truncated-pyramid + chamfer-
//! band point cloud. Oriented binormal to the wall (axis along +Y),
//! centred at (32.5 mm, 0, 5 mm) — radially mid-wall on the +X
//! equator, 5 mm above the seam ring.
//!
//! Cell-size sweep: 1 mm (over-resolved control) + 3 mm (cf-cast-cli
//! `default_mesh_cell_size_m()`). Comparing the two distinguishes
//! "manifold3d boolean mechanics broken" (fails at both) from "MC
//! quantization × mesh-CSG interaction" (fails at 3 mm, passes at
//! 1 mm). The empirical outcome above says BOTH fail — pointing
//! at manifold3d's mesh-CSG-on-MC-curved-shell robustness, not pure
//! MC quantization.

// Integration tests use `unwrap()` and `expect()` to surface failure
// context as readable panics; library code denies `unwrap_used` /
// `expect_used`, but they are `warn` at workspace level for
// `tests/*.rs` (same convention as `single_layer_smoke.rs`).
// Probe-spike tests prioritise empirical clarity over rustdoc lint
// compliance. The `doc_markdown` + `cast_possible_wrap` + naming-style
// nits are noise for a characterisation file that will likely get
// pruned at S2 implementation per [[feedback-implement-measure-revert-pattern]];
// silencing them keeps the recon-vs-evidence narrative intact in the
// module + item docstrings.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::too_many_lines,
    clippy::doc_markdown,
    clippy::cast_possible_wrap,
    clippy::redundant_closure_for_method_calls,
    clippy::missing_const_for_fn,
    dead_code
)]

use cf_design::Solid;
use manifold3d::Manifold;
use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};
use mesh_printability::{IssueSeverity, PrintIssueType, PrinterConfig, validate_for_printing};
use mesh_repair::components::find_connected_components;
use mesh_types::IndexedMesh;
use nalgebra::Vector3;

// ── Probe constants ──────────────────────────────────────────────────

/// Meters-to-mm scale factor at the cf-design → printer boundary.
/// Matches `cf_cast::mesher::METERS_TO_MM` (private; duplicated here
/// to keep the probe decoupled from cf-cast internals).
const METERS_TO_MM: f64 = 1000.0;

/// Grid padding cells on each side of the input AABB. Matches
/// `cf_cast::mesher::GRID_PADDING_CELLS` (private constant).
const HOST_GRID_PADDING_CELLS: usize = 2;

/// Synthetic body radius (meters) for the half-sphere shell host. A
/// 30 mm sphere gives a wall midpoint at the original surface — the
/// `Solid::shell` operator builds a 2t-thick shell centred on the
/// original surface.
const HOST_BODY_RADIUS_M: f64 = 0.030;

/// `Solid::shell` half-thickness (meters). The resulting shell spans
/// `|p| ∈ [R - t, R + t]` for total wall thickness `2t = 10 mm` —
/// thick enough for a CONTAINED PrismaticPin at half-length 3 mm with
/// margin to spare.
const HOST_SHELL_HALF_THICKNESS_M: f64 = 0.005;

/// Production-resolution MC cell size — matches the cf-cast-cli
/// `default_mesh_cell_size_m()` and the iter-1 cup-piece regime.
const PROD_CELL_SIZE_M: f64 = 0.003;

/// Over-resolved control MC cell size. Compared against
/// `PROD_CELL_SIZE_M` to isolate MC-quantization effects.
const FINE_CELL_SIZE_M: f64 = 0.001;

/// Pin centre radial position (mm). 32.5 mm = 0.5 (R - t) + 0.5
/// (R + t) = midpoint of the wall annulus [25, 35] mm. With pin
/// half-extents up to 1.5 mm, the pin's |p| extent stays well
/// inside [25, 35] from any pin corner.
const PIN_RADIUS_OFFSET_MM: f64 = 32.5;

/// PrismaticPin base half-extent (mm). Per §G-7 doc spec — typical
/// cup-pin base footprint.
const PIN_BASE_HALF_MM: f64 = 1.5;

/// PrismaticPin tip half-extent (mm). Per §G-7 doc spec; ratio
/// 1.2 / 1.5 ≈ 0.8 falls in §G-6's cup-pin taper-ratio band
/// [0.7, 0.85].
const PIN_TIP_HALF_MM: f64 = 1.2;

/// PrismaticPin half-length (mm) along its axis. Per §G-7 doc spec.
const PIN_HALF_LENGTH_MM: f64 = 3.0;

/// Default chamfer band depth (mm) for §G-7 union/subtract probes.
const PIN_DEFAULT_CHAMFER_MM: f64 = 0.5;

/// Socket diametral inflate (meters) — same convention as
/// `cf_cast::PinSpec::diametral_clearance_m`. Picks the §G-8 cup-pin
/// default (0.30 mm symmetric).
const SOCKET_DIAMETRAL_CLEARANCE_M: f64 = 0.0003;

/// Pin centre's equator height (mm) above the seam ring. 5 mm puts
/// the pin's |z| extent (±base_half = ±1.5 mm) comfortably above
/// z = 0 and well below the dome at ~35 mm.
const PIN_Z_EQUATOR_MM: f64 = 5.0;

// ── Curved-shell host builder ────────────────────────────────────────

/// Build a half-sphere shell host SDF: `sphere(R).shell(t) ∩
/// halfspace(z > 0)`. Returns the `Solid` so callers can choose
/// whether to MC it directly (default §G-7 path) or compose further
/// SDF ops onto it (§G-12 #2 bail-out path).
fn build_half_sphere_shell_solid() -> Solid {
    let sphere = Solid::sphere(HOST_BODY_RADIUS_M);
    let shell = sphere.shell(HOST_SHELL_HALF_THICKNESS_M);
    // `Solid::plane(normal, offset)` evaluates as `normal·p - offset`,
    // with the "inside" half-space at `f(p) < 0`. To keep the
    // `z > 0` half-space, point the plane normal at `-Z` so
    // `f(p) = -z` is negative when `z > 0`. Matches the
    // recon-4 §F-3 convention.
    let upper_halfspace = Solid::plane(Vector3::new(0.0, 0.0, -1.0), 0.0);
    shell.intersect(upper_halfspace)
}

/// MC the half-sphere shell SDF and return the resulting Manifold.
fn build_curved_shell_host_manifold(cell_size_m: f64) -> Manifold {
    let half_shell = build_half_sphere_shell_solid();
    let mesh_mm = solid_to_mm_mesh(&half_shell, cell_size_m);
    indexed_mesh_to_manifold(&mesh_mm)
}

/// Local re-implementation of `cf_cast::mesher::solid_to_mm_mesh`'s
/// SDF → grid → MC → mm pipeline. Kept inline so the probe doesn't
/// depend on cf-cast internals or its private `mesher` module.
fn solid_to_mm_mesh(solid: &Solid, cell_size_m: f64) -> IndexedMesh {
    let bounds = solid.bounds().expect("synthetic shell has finite bounds");
    let mut grid =
        ScalarGrid::from_bounds(bounds.min, bounds.max, cell_size_m, HOST_GRID_PADDING_CELLS);
    let (nx, ny, nz) = grid.dimensions();
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = grid.position(ix, iy, iz);
                grid.set(ix, iy, iz, solid.evaluate(&p));
            }
        }
    }
    let mut mesh = marching_cubes(&grid, &MarchingCubesConfig::default());
    assert!(
        !mesh.vertices.is_empty() && !mesh.faces.is_empty(),
        "MC produced empty mesh for synthetic shell — probe fixture broken",
    );
    for v in &mut mesh.vertices {
        v.x *= METERS_TO_MM;
        v.y *= METERS_TO_MM;
        v.z *= METERS_TO_MM;
    }
    mesh
}

/// Local re-implementation of `cf_cast::mesh_csg::indexed_mesh_to_manifold`
/// (private). Widens face indices to `u64` and feeds manifold3d's
/// `from_mesh_f64`.
fn indexed_mesh_to_manifold(mesh: &IndexedMesh) -> Manifold {
    let vert_props: Vec<f64> = mesh.vertices.iter().flat_map(|p| [p.x, p.y, p.z]).collect();
    let tri_indices: Vec<u64> = mesh
        .faces
        .iter()
        .flat_map(|&[i0, i1, i2]| [u64::from(i0), u64::from(i1), u64::from(i2)])
        .collect();
    Manifold::from_mesh_f64(&vert_props, 3, &tri_indices)
        .expect("MC output must be manifold-clean for the probe to be meaningful")
}

/// Convert a `Manifold` back into `IndexedMesh` (mm coords) for the
/// F4 gate and connectivity inspector.
fn manifold_to_indexed_mesh(m: &Manifold) -> IndexedMesh {
    let (vp, n_props, tri) = m.to_mesh_f64();
    assert_eq!(n_props, 3, "manifold3d should emit 3 props per vertex");
    let vertices: Vec<mesh_types::Point3<f64>> = vp
        .chunks_exact(n_props)
        .map(|c| mesh_types::Point3::new(c[0], c[1], c[2]))
        .collect();
    let faces: Vec<[u32; 3]> = tri
        .chunks_exact(3)
        .map(|c| {
            [
                u32::try_from(c[0]).unwrap(),
                u32::try_from(c[1]).unwrap(),
                u32::try_from(c[2]).unwrap(),
            ]
        })
        .collect();
    IndexedMesh::from_parts(vertices, faces)
}

// ── PrismaticPin primitive (probe-local) ─────────────────────────────

/// Build a truncated-pyramid `Manifold` via `Manifold::hull_pts`,
/// oriented with axis along +Y (binormal-to-wall in the §G-7 host
/// orientation, mirroring the recon-4 (P) binormal-axis cup-pin
/// layout).
///
/// Geometry (axis along +Y, centred at origin, dimensions in mm):
///
/// - `chamfer_mm == 0.0` (plain truncated pyramid):
///   - Base (y = -half_length): 4 corners at (±base_half, _, ±base_half).
///   - Tip  (y = +half_length): 4 corners at (±tip_half,  _, ±tip_half).
///   - 8 hull points total.
/// - `chamfer_mm > 0.0` (truncated pyramid + first-layer chamfer band):
///   - Base bottom (y = -half_length): 4 inset corners at
///     (±(base_half - chamfer), _, ±(base_half - chamfer)).
///   - Chamfer top (y = -half_length + chamfer): 4 corners at
///     (±base_half, _, ±base_half).
///   - Tip (y = +half_length): 4 corners at (±tip_half, _, ±tip_half).
///   - 12 hull points total.
///
/// The chamfer band narrows the bed-contacting face by `chamfer_mm`
/// in each axis (so the bed footprint is
/// `2 (base_half - chamfer)` square), which absorbs FDM first-layer
/// squish. Caller is responsible for ensuring `chamfer < base_half`.
fn build_prismatic_pin_manifold(
    base_half_mm: f64,
    tip_half_mm: f64,
    half_length_mm: f64,
    chamfer_mm: f64,
) -> Manifold {
    let mut pts: Vec<[f64; 3]> = Vec::new();
    if chamfer_mm <= 0.0 {
        for sz in [-1.0, 1.0] {
            for sx in [-1.0, 1.0] {
                pts.push([sx * base_half_mm, -half_length_mm, sz * base_half_mm]);
            }
        }
    } else {
        assert!(
            chamfer_mm < base_half_mm,
            "chamfer {chamfer_mm} mm must be strictly less than base half-extent {base_half_mm} mm",
        );
        let inset = base_half_mm - chamfer_mm;
        for sz in [-1.0, 1.0] {
            for sx in [-1.0, 1.0] {
                pts.push([sx * inset, -half_length_mm, sz * inset]);
                pts.push([
                    sx * base_half_mm,
                    -half_length_mm + chamfer_mm,
                    sz * base_half_mm,
                ]);
            }
        }
    }
    for sz in [-1.0, 1.0] {
        for sx in [-1.0, 1.0] {
            pts.push([sx * tip_half_mm, half_length_mm, sz * tip_half_mm]);
        }
    }
    Manifold::hull_pts(&pts)
}

/// Pin centre position (mm) for the §G-7 default layout — wall
/// midpoint on the +X equator, 5 mm above the seam ring.
fn pin_center_for_default_layout_mm() -> (f64, f64, f64) {
    (PIN_RADIUS_OFFSET_MM, 0.0, PIN_Z_EQUATOR_MM)
}

// ── F4 critical-issue diff helper ────────────────────────────────────

/// Collect the set of F4 critical-issue TYPES present in a mesh,
/// sorted + deduplicated.
fn critical_issue_types(mesh: &IndexedMesh, config: &PrinterConfig) -> Vec<PrintIssueType> {
    let validation =
        validate_for_printing(mesh, config).expect("non-empty mesh validates without error");
    let mut types: Vec<PrintIssueType> = validation
        .issues
        .iter()
        .filter(|i| i.severity == IssueSeverity::Critical)
        .map(|i| i.issue_type)
        .collect();
    types.sort_by_key(|t| t.as_str());
    types.dedup();
    types
}

/// Per-cell-size record of a single §G-7 union/subtract probe run.
#[derive(Debug)]
struct ProbeRun {
    cell_size_m: f64,
    baseline_components: usize,
    post_components: usize,
    baseline_critical_types: Vec<PrintIssueType>,
    post_critical_types: Vec<PrintIssueType>,
}

impl ProbeRun {
    fn new_critical_types(&self) -> Vec<PrintIssueType> {
        self.post_critical_types
            .iter()
            .filter(|t| !self.baseline_critical_types.contains(t))
            .copied()
            .collect()
    }

    fn extra_components(&self) -> isize {
        self.post_components as isize - self.baseline_components as isize
    }
}

/// Run a single §G-7 Union probe at the given cell size.
fn run_union_probe(cell_size_m: f64) -> ProbeRun {
    let host = build_curved_shell_host_manifold(cell_size_m);
    let host_mesh = manifold_to_indexed_mesh(&host);
    let baseline_components = find_connected_components(&host_mesh).component_count;
    let baseline_critical_types = critical_issue_types(&host_mesh, &PrinterConfig::fdm_default());

    let pin = build_prismatic_pin_manifold(
        PIN_BASE_HALF_MM,
        PIN_TIP_HALF_MM,
        PIN_HALF_LENGTH_MM,
        PIN_DEFAULT_CHAMFER_MM,
    );
    assert!(!pin.is_empty(), "PrismaticPin Manifold non-empty");
    let (px, py, pz) = pin_center_for_default_layout_mm();
    let positioned_pin = pin.translate(px, py, pz);

    let result = host.union(&positioned_pin);
    let result_mesh = manifold_to_indexed_mesh(&result);
    ProbeRun {
        cell_size_m,
        baseline_components,
        post_components: find_connected_components(&result_mesh).component_count,
        baseline_critical_types,
        post_critical_types: critical_issue_types(&result_mesh, &PrinterConfig::fdm_default()),
    }
}

/// Run a single §G-7 Subtract probe at the given cell size.
fn run_subtract_probe(cell_size_m: f64) -> ProbeRun {
    let host = build_curved_shell_host_manifold(cell_size_m);
    let host_mesh = manifold_to_indexed_mesh(&host);
    let baseline_components = find_connected_components(&host_mesh).component_count;
    let baseline_critical_types = critical_issue_types(&host_mesh, &PrinterConfig::fdm_default());

    let clearance_half_mm = (SOCKET_DIAMETRAL_CLEARANCE_M * METERS_TO_MM) / 2.0;
    let socket = build_prismatic_pin_manifold(
        PIN_BASE_HALF_MM + clearance_half_mm,
        PIN_TIP_HALF_MM + clearance_half_mm,
        PIN_HALF_LENGTH_MM,
        PIN_DEFAULT_CHAMFER_MM,
    );
    assert!(!socket.is_empty(), "PrismaticPin socket Manifold non-empty");
    let (px, py, pz) = pin_center_for_default_layout_mm();
    let positioned_socket = socket.translate(px, py, pz);

    let result = host.difference(&positioned_socket);
    let result_mesh = manifold_to_indexed_mesh(&result);
    ProbeRun {
        cell_size_m,
        baseline_components,
        post_components: find_connected_components(&result_mesh).component_count,
        baseline_critical_types,
        post_critical_types: critical_issue_types(&result_mesh, &PrinterConfig::fdm_default()),
    }
}

// ── Baseline characterisation ────────────────────────────────────────

/// Establish the bare half-sphere shell host's component count +
/// critical-issue type set so the §G-7 / §G-9 probes can diff against
/// it. At both 1 mm and 3 mm cell sizes the seam-cut half-shell is
/// ONE connected surface (outer + inner meet at the equator seam ring)
/// — recon-4 (P) §F-3b baseline.
///
/// The shell DOES violate `ExcessiveOverhang` + `LongBridge` Critical
/// F4 gates (a hollow hemispherical dome has overhang regions); these
/// are host properties, not boolean-junction properties, and the §G-7
/// probes subtract them from the post-CSG critical-type set.
#[test]
fn g7_baseline_half_sphere_shell_host_is_one_component_at_both_cell_sizes() {
    for &cell_size_m in &[FINE_CELL_SIZE_M, PROD_CELL_SIZE_M] {
        let host = build_curved_shell_host_manifold(cell_size_m);
        assert!(!host.is_empty(), "half-shell host non-empty");
        let host_mesh = manifold_to_indexed_mesh(&host);
        let analysis = find_connected_components(&host_mesh);
        let critical_types = critical_issue_types(&host_mesh, &PrinterConfig::fdm_default());

        eprintln!(
            "g7 baseline @ cell_size={cell_size_m} m — components: {}, critical types: \
             {critical_types:?}",
            analysis.component_count,
        );

        assert_eq!(
            analysis.component_count, 1,
            "half-shell host MUST be exactly one connected component at cell_size={cell_size_m} m \
             — recon §G-7 BRANCH-determination diffs against this baseline.",
        );
    }
}

// ── §G-7 characterisation: mesh-CSG PrismaticPin (default path) ──────
//
// These tests LOCK the empirically observed BRANCH B + BRANCH C
// outcome at the time of this probe-spike commit. They are NOT
// "PASS the recon's default path" tests — they are "the default path
// FAILS in the following specific ways" regression guards. The recon
// doc §G-7 records this as the falsification + §G-12 #2 bail-out
// pick. If a future manifold3d / mesh-CSG fix flips these tests to
// pass, the §G-12 #2 bail-out can be re-evaluated.

/// §G-7 Union @ production cell size (3 mm): mesh-CSG union of a
/// truncated-pyramid PrismaticPin onto the SDF→MC half-sphere shell
/// host produces THREE components (baseline 1, +2 disconnected
/// pin/junction shells) AND introduces a `SelfIntersecting` Critical
/// F4 issue at the boolean junction.
///
/// → BRANCH B + BRANCH C, §G-12 #2 bail-out fires.
#[test]
fn g7_characterisation_union_at_production_cell_size_branches_b_and_c() {
    let run = run_union_probe(PROD_CELL_SIZE_M);
    eprintln!("g7 union @ 3mm cell: {run:#?}");

    // BRANCH B characterisation: post-union components = baseline + 2.
    assert_eq!(
        run.baseline_components, 1,
        "§G-7 baseline regression — half-shell host should be 1-component",
    );
    assert_eq!(
        run.post_components, 3,
        "§G-7 union @ 3 mm — post-union components changed from the locked +2 BRANCH B \
         characterisation. If this is now 1, BRANCH A; rerun the recon §G-7 decision.",
    );

    // BRANCH C characterisation: post-union introduces SelfIntersecting.
    let new_types = run.new_critical_types();
    assert_eq!(
        new_types,
        vec![PrintIssueType::SelfIntersecting],
        "§G-7 union @ 3 mm — new critical types changed from the locked BRANCH C \
         characterisation [SelfIntersecting]. Recon §G-7 decision needs re-evaluation.",
    );
}

/// §G-7 Subtract @ production cell size (3 mm): mesh-CSG difference
/// of a PrismaticPin socket from the half-shell host produces TWO
/// components (baseline 1, +1 detached interior shell at the socket
/// floor) but introduces NO new critical types.
///
/// → BRANCH B only (subtract path); §G-12 #2 bail-out fires for both
/// union + subtract.
#[test]
fn g7_characterisation_subtract_at_production_cell_size_branch_b_only() {
    let run = run_subtract_probe(PROD_CELL_SIZE_M);
    eprintln!("g7 subtract @ 3mm cell: {run:#?}");

    assert_eq!(
        run.baseline_components, 1,
        "§G-7 baseline regression — half-shell host should be 1-component",
    );
    assert_eq!(
        run.post_components, 2,
        "§G-7 subtract @ 3 mm — post-subtract components changed from the locked +1 BRANCH B \
         characterisation. If this is now 1, BRANCH A; rerun the recon §G-7 decision.",
    );

    let new_types = run.new_critical_types();
    assert!(
        new_types.is_empty(),
        "§G-7 subtract @ 3 mm — new critical types {new_types:?} appeared; previously empty. \
         The subtract path was BRANCH B only (no F4 critical at the junction). Recon \
         §G-7 decision needs re-evaluation.",
    );
}

/// §G-7 Union @ fine cell size (1 mm): SAME BRANCH B + BRANCH C
/// symptoms reproduce at the over-resolved control cell size.
/// → Failure is NOT pure MC quantization — it's manifold3d's mesh-CSG-
/// on-MC-curved-shell robustness boundary. Strengthens the §G-12 #2
/// case (paradigm-boundary correction) over §G-12 #1 (which would
/// only need to fix MC quantization).
#[test]
fn g7_characterisation_union_at_fine_cell_size_also_branches_b_and_c() {
    let run = run_union_probe(FINE_CELL_SIZE_M);
    eprintln!("g7 union @ 1mm cell: {run:#?}");

    assert_eq!(run.baseline_components, 1);
    assert_eq!(
        run.post_components, 3,
        "§G-7 union @ 1 mm — fine-cell control changed from the locked +2 BRANCH B \
         characterisation. If this fixed itself at 1 mm but stayed broken at 3 mm, the \
         failure would be pure MC-quantization (§G-12 #1 would suffice). The locked \
         characterisation says BOTH cell sizes fail identically → §G-12 #2 is correct.",
    );
    let new_types = run.new_critical_types();
    assert_eq!(
        new_types,
        vec![PrintIssueType::SelfIntersecting],
        "§G-7 union @ 1 mm — new critical types {new_types:?} vs locked [SelfIntersecting].",
    );
}

/// §G-7 Subtract @ fine cell size (1 mm): same BRANCH B only outcome
/// as the 3 mm subtract path.
#[test]
fn g7_characterisation_subtract_at_fine_cell_size_branch_b_only() {
    let run = run_subtract_probe(FINE_CELL_SIZE_M);
    eprintln!("g7 subtract @ 1mm cell: {run:#?}");

    assert_eq!(run.baseline_components, 1);
    assert_eq!(
        run.post_components, 2,
        "§G-7 subtract @ 1 mm — fine-cell control changed from the locked +1 BRANCH B \
         characterisation.",
    );
    let new_types = run.new_critical_types();
    assert!(new_types.is_empty(), "subtract path stays BRANCH B only");
}

// ── §G-12 #2 bail-out characterisation: SDF-side pin ────────────────

/// §G-12 #2 bail-out: pin geometry lives entirely in SDF (composed
/// pre-MC into the host SDF). MC sees a single SDF (`half_shell ∪
/// pin`) and produces a single mesh — no mesh-CSG boolean op against
/// the curved-shell, no paradigm-boundary crossed.
///
/// This is the recon-4 (P) §F-3 pattern transposed to PrismaticPin:
/// the synthetic SDF cuboid pin (an axis-aligned-box approximation
/// of the truncated-pyramid; full taper + chamfer SDF emission is an
/// implementation-session detail) demonstrates the §G-12 #2 path is
/// topologically sound. The implementation arc S2 builds a proper
/// taper + chamfer SDF emitter for `PrismaticPin`.
///
/// → BRANCH A under the §G-12 #2 bail-out: 1 component, no new F4
/// critical types at the junction.
#[test]
fn g12_2_bailout_sdf_pin_yields_one_component_no_new_critical_at_production_cell_size() {
    // Baseline: pure half-shell SDF, MC'd, evaluated.
    let baseline_host = build_curved_shell_host_manifold(PROD_CELL_SIZE_M);
    let baseline_mesh = manifold_to_indexed_mesh(&baseline_host);
    let baseline_components = find_connected_components(&baseline_mesh).component_count;
    let baseline_critical_types =
        critical_issue_types(&baseline_mesh, &PrinterConfig::fdm_default());
    assert_eq!(
        baseline_components, 1,
        "§G-12 #2 bail-out baseline regression",
    );

    // SDF-side pin: cuboid centred at the same wall midpoint as the
    // mesh-CSG probes, axis-aligned to +Y (length axis), composed
    // via `Solid::union` BEFORE MC. The actual implementation-arc
    // PrismaticPin SDF would emit `cuboid + taper-correction +
    // chamfer-band`; for the §G-12 #2 BRANCH-A characterisation an
    // axis-aligned cuboid is the minimum viable composition.
    let pin_half_extents_m = Vector3::new(
        PIN_BASE_HALF_MM / METERS_TO_MM,
        PIN_HALF_LENGTH_MM / METERS_TO_MM,
        PIN_BASE_HALF_MM / METERS_TO_MM,
    );
    let pin_center_m = Vector3::new(
        PIN_RADIUS_OFFSET_MM / METERS_TO_MM,
        0.0,
        PIN_Z_EQUATOR_MM / METERS_TO_MM,
    );
    let pin_solid = Solid::cuboid(pin_half_extents_m).translate(pin_center_m);

    let composed = build_half_sphere_shell_solid().union(pin_solid);
    let composed_mesh = solid_to_mm_mesh(&composed, PROD_CELL_SIZE_M);
    let composed_components = find_connected_components(&composed_mesh).component_count;
    let composed_critical_types =
        critical_issue_types(&composed_mesh, &PrinterConfig::fdm_default());
    let new_critical_types: Vec<PrintIssueType> = composed_critical_types
        .iter()
        .filter(|t| !baseline_critical_types.contains(t))
        .copied()
        .collect();

    eprintln!(
        "g12 #2 SDF-pin @ 3mm cell — components: {composed_components} (baseline \
         {baseline_components}), critical types: {composed_critical_types:?}, new critical: \
         {new_critical_types:?}",
    );

    assert_eq!(
        composed_components, baseline_components,
        "§G-12 #2 BAIL-OUT FALSIFIED — SDF-union pin onto half-shell SDF, MC'd, produced \
         {composed_components} components vs baseline {baseline_components}. The bail-out path \
         is also unsound; recon-2 escalation (pivot mating-feature mechanism, e.g. surface- \
         intersecting dimples or magnetic registration) is required.",
    );
    assert!(
        new_critical_types.is_empty(),
        "§G-12 #2 BAIL-OUT FALSIFIED — SDF-union pin introduced {new_critical_types:?} \
         critical types not in baseline. Recon-2 escalation required.",
    );
}

// ── §G-9 chamfer band feasibility sweep ──────────────────────────────

/// §G-9 chamfer sweep at production cell size — characterises whether
/// the §G-7 BRANCH B + BRANCH C outcome is **chamfer-dependent** or
/// **chamfer-independent**.
///
/// Empirical result: all chamfer values in {0.0, 0.4, 0.6, 0.8, 1.0
/// mm} fail IDENTICALLY (post-union components = 3, new critical
/// types = [SelfIntersecting]). The failure is chamfer-INDEPENDENT —
/// it's the underlying mesh-CSG-on-MC-curved-shell paradigm boundary,
/// not the chamfer band geometry.
///
/// → §G-9 BRANCH determination INHERITS §G-7's BRANCH B + BRANCH C.
/// Once §G-12 #2 is implemented (SDF-side PrismaticPin emission with
/// chamfer band emitted as SDF, e.g. `cuboid + smooth taper +
/// chamfer-band cuboid subtract`), §G-9 should be re-probed against
/// that path. The §G-9 CSG-vs-slicer decision (CSG-level chamfer
/// option (i)) remains intact as long as the chamfer is SDF-side.
#[test]
fn g9_characterisation_chamfer_sweep_fails_identically_chamfer_independent() {
    const CHAMFER_SWEEP_MM: &[f64] = &[0.0, 0.4, 0.6, 0.8, 1.0];

    let host = build_curved_shell_host_manifold(PROD_CELL_SIZE_M);
    let host_mesh = manifold_to_indexed_mesh(&host);
    let baseline_components = find_connected_components(&host_mesh).component_count;
    let baseline_critical_types = critical_issue_types(&host_mesh, &PrinterConfig::fdm_default());

    let mut observations: Vec<(f64, usize, Vec<PrintIssueType>)> = Vec::new();
    for &chamfer_mm in CHAMFER_SWEEP_MM {
        let pin = build_prismatic_pin_manifold(
            PIN_BASE_HALF_MM,
            PIN_TIP_HALF_MM,
            PIN_HALF_LENGTH_MM,
            chamfer_mm,
        );
        assert!(
            !pin.is_empty(),
            "§G-9 chamfer={chamfer_mm} mm — Manifold empty at primitive build (would be \
             a separate failure class — chamfer-dependent BRANCH B/C at the primitive layer)",
        );
        assert!(
            pin.volume() > 1.0,
            "§G-9 chamfer={chamfer_mm} mm — pin volume {} mm³ implausibly small",
            pin.volume(),
        );

        let (px, py, pz) = pin_center_for_default_layout_mm();
        let positioned_pin = pin.translate(px, py, pz);
        let result = host.union(&positioned_pin);
        let result_mesh = manifold_to_indexed_mesh(&result);

        let result_components = find_connected_components(&result_mesh).component_count;
        let result_critical_types =
            critical_issue_types(&result_mesh, &PrinterConfig::fdm_default());
        let new_critical_types: Vec<PrintIssueType> = result_critical_types
            .iter()
            .filter(|t| !baseline_critical_types.contains(t))
            .copied()
            .collect();

        eprintln!(
            "§G-9 chamfer={chamfer_mm:.1} mm @ 3mm cell — pin volume {:.3} mm³, post-union \
             components {result_components} (baseline {baseline_components}), new critical \
             types {new_critical_types:?}",
            pin.volume(),
        );

        observations.push((chamfer_mm, result_components, new_critical_types));
    }

    // Lock the chamfer-INDEPENDENT failure mode: ALL values produce
    // identical post-union components + identical new-critical-types.
    let first = observations.first().expect("non-empty chamfer sweep");
    for (chamfer_mm, components, new_types) in &observations {
        assert_eq!(
            *components, first.1,
            "§G-9 chamfer={chamfer_mm} mm produced {components} components, expected \
             chamfer-independent failure ({} components like chamfer={} mm). Chamfer- \
             DEPENDENT behaviour here would mean §G-12 #3 (slicer-level chamfer) might \
             alone unblock §G-7; this characterisation says no.",
            first.1, first.0,
        );
        assert_eq!(
            new_types, &first.2,
            "§G-9 chamfer={chamfer_mm} mm new critical types {new_types:?} differ from \
             chamfer={} mm new critical types {:?}. Chamfer-dependent §G-12 #3 doesn't \
             alone unblock §G-7 under this characterisation.",
            first.0, first.2,
        );
    }
    // Lock the specific failure shape — same as g7 union characterisation.
    assert_eq!(
        first.1, 3,
        "§G-9 sweep post-union components diverged from locked +2 BRANCH B characterisation",
    );
    assert_eq!(
        first.2,
        vec![PrintIssueType::SelfIntersecting],
        "§G-9 sweep new critical types diverged from locked [SelfIntersecting]",
    );
}

// ── PrismaticPin primitive sanity guards ─────────────────────────────

/// Independent sanity: the chamfer-0 PrismaticPin has at least 8
/// distinct hull vertices and the chamfer-positive variant has at
/// least 12. hull_pts can in principle dedupe coincident points; the
/// test just pins the vertex-count contract so a future refactor that
/// accidentally drops a corner surfaces immediately. The bounding-box
/// extents are also pinned (axis is +Y → |y| spread = half-length;
/// lateral spread = max(base_half, tip_half)).
#[test]
fn prismatic_pin_primitive_vertex_count_and_extents_hold() {
    let plain =
        build_prismatic_pin_manifold(PIN_BASE_HALF_MM, PIN_TIP_HALF_MM, PIN_HALF_LENGTH_MM, 0.0);
    let chamfered = build_prismatic_pin_manifold(
        PIN_BASE_HALF_MM,
        PIN_TIP_HALF_MM,
        PIN_HALF_LENGTH_MM,
        PIN_DEFAULT_CHAMFER_MM,
    );

    assert!(
        plain.num_vert() >= 8,
        "plain truncated pyramid expected ≥ 8 manifold verts, got {}",
        plain.num_vert(),
    );
    assert!(
        chamfered.num_vert() >= 12,
        "chamfered truncated pyramid expected ≥ 12 manifold verts, got {}",
        chamfered.num_vert(),
    );

    let bb = plain
        .bounding_box_nalgebra()
        .expect("plain truncated pyramid has a finite bounding box");
    assert!(
        (bb.0.y + PIN_HALF_LENGTH_MM).abs() < 1.0e-9,
        "plain pin base y should be -{PIN_HALF_LENGTH_MM} mm; got {}",
        bb.0.y,
    );
    assert!(
        (bb.1.y - PIN_HALF_LENGTH_MM).abs() < 1.0e-9,
        "plain pin tip y should be +{PIN_HALF_LENGTH_MM} mm; got {}",
        bb.1.y,
    );
    let lateral_max = PIN_BASE_HALF_MM.max(PIN_TIP_HALF_MM);
    assert!(
        (bb.1.x - lateral_max).abs() < 1.0e-9,
        "plain pin +X max should be {lateral_max} mm; got {}",
        bb.1.x,
    );
}
