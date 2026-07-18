// Localized `expect(...)` allowance — `TriMeshDistance::new` on a
// programmatic 12-tri cube fixture cannot return `Err(EmptyMesh)` by
// construction (12 faces, non-empty), so the expect is a diagnostic
// guard on a `Result::Err` impossibility. Same precedent as PR3 row 15
// `mesh-scan-as-solid` and `examples/mesh/mesh-sdf-distance-query`.
#![allow(clippy::expect_used)]
// `usize as f64` casts on referenced-vertex / per-shell tet counts for
// mean computations. Counts ≤ ~6500 here, well within f64 mantissa
// exact range. Same allowance as rows 6+8+9+10+11+16+18.
#![allow(clippy::cast_precision_loss)]
// `cast_possible_wrap` on `usize → VertexId` (u32) packing for vertex
// indices in the BCC + stuffing pipeline. Vertex counts are bounded by
// the BCC-lattice's i32-safe `n_lattice` cap inherited from
// `BccLattice::new`. Same allowance as rows 8+9+10+11+15+16.
#![allow(clippy::cast_possible_wrap)]
// `cast_possible_truncation` on `pid as u32` packing for primitive
// indices in the per-pair-readout walk (one rigid primitive in this
// row). Same allowance as row 18.
#![allow(clippy::cast_possible_truncation)]
// PLY field-data is single-precision on disk; the per-vertex
// `as f32` narrowing on f64 displacement / material_id values is
// intrinsic to the format. The `cast_possible_truncation` allow
// added above also covers the f64 → f32 narrowing case.
// `print_summary` is a single museum-plaque stdout writer; splitting
// fragments the visual format without information gain. Same allowance
// as rows 4+5+6+9+10+11+15+16+19.
#![allow(clippy::too_many_lines)]
// Domain-meaningful naming pairs (`outer_solid` / `cavity_solid`,
// `pos_pinned` / `pos_active`) distinguish operand-vs-receiver across
// the heterogeneous-CSG and contact-readout sites. Same allowance as
// rows 6+10+11+16.
#![allow(clippy::similar_names)]

//! layered-silicone-device — the PR3 Tier 6 synthesis row, closing
//! the entire sim-soft examples arc by composing every PR3 foundation
//! piece into one end-to-end relative-comparison sim of the
//! [layered silicone device][mem]'s cavity-fit tightness behaviour.
//!
//! [mem]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_layered_silicone_device.md
//!
//! # Pipeline
//!
//! 1. **Scan-derived rigid indenter** → `mesh_sdf::Signed<TriMeshDistance,
//!    PseudoNormalSign>` over a programmatic 12-triangle cube fixture (`R_SCAN = 0.025 m`,
//!    translated by `SCAN_OFFSET_X = 0.015 m` along `+x` to make the
//!    carved cavity asymmetric and visibly non-spherical). No checked-in scan
//!    asset per the device memo's sanitization directive — the cube
//!    stand-in IS the workflow demonstration; production scans flow
//!    through the same `TriMeshDistance::new(loaded_mesh)` + sign-oracle
//!    composition via row 15 `mesh-scan-as-solid`'s STL-import precedent.
//!    `Signed<D, S>` impls `cf_design::Sdf` (the blanket adapter at
//!    `mesh-sdf/src/sdf_adapter.rs`) so it is a first-class
//!    cf-design SDF the moment it is constructed.
//!
//! 2. **Heterogeneous CSG composition** via PR3 F5 (`commit 93f4bfaa`):
//!    `Solid::from_sdf(scan_sdf, scan_bounds)` lifts the scan into a
//!    typed `cf_design::Solid` leaf, so the body is one expression in
//!    the typed-Solid kernel:
//!    ```ignore
//!    let body = Solid::sphere(R_OUTER)
//!        .subtract(Solid::from_sdf(scan_sdf.clone(), scan_bounds));
//!    ```
//!    This is the architectural payload of PR3 — the heterogeneous-CSG
//!    bridge between parametric typed bodies (typed `Solid` with
//!    sphere / cuboid / etc. primitives) and scan-derived SDFs (mesh-
//!    sdf-backed) lives in cf-design's typed kernel as F5 sugar over
//!    the existing `Solid::user_fn` escape hatch, not in sim-soft as a
//!    `DifferenceSdf<Box<dyn Sdf>>` heterogeneous escape hatch.
//!    `feedback_boolean_then_sdf` + the SDF-first design philosophy
//!    drive this choice; F5's contract tests (`from_sdf_composes_with_
//!    subtract`) cover exactly this row-20 case.
//!
//! 3. **`SdfMeshedTetMesh` build** via PR3 F1+F3:
//!    `SdfMeshedTetMesh::from_sdf(&body, &hints)` accepts `&dyn
//!    cf_design::Sdf` (since `Solid: Sdf` per F1, re-exported as
//!    `sim_soft::Sdf` per F3) — one trait-object coercion bridges the
//!    typed-Solid surface into the BCC + Labelle-Shewchuk Isosurface
//!    Stuffing tet pipeline. `MeshingHints::material_field` carries
//!    the 3-shell `LayeredScalarField` partition (parametric, NOT
//!    scan-shaped — the partition is by `‖p‖` radial-bin from origin
//!    so the cavity-asymmetry doesn't mix into the material-id classes).
//!
//! 4. **3-shell `MaterialField`** via PR3 F4 (`silicone_table.rs`):
//!    outer + inner shells = `ECOFLEX_00_30` (μ = 23 kPa); middle =
//!    `DRAGON_SKIN_10A` (μ = 51 kPa, ~2.22× outer μ). The middle
//!    layer's real-device counterpart carries copper mesh + carbon
//!    black inclusions; F4 models the silicone matrix only, and
//!    `DRAGON_SKIN_10A` is the closest pure-silicone proxy for the
//!    composite's effective stiffness. The composite mechanical uplift
//!    (Cu mesh + carbon black contributions) is deferred to a Fork-B
//!    post-cast modulus calibration that absorbs the uplift into the
//!    effective μ at calibration time; the calibrated μ then flows into
//!    F4's table for sim consumption. Each `SiliconeMaterial::to_neo_hookean()` call is
//!    a `const fn` over F4's table entries, so the per-shell
//!    `(μ, λ)` provenance survives bit-equally from F4 down to the
//!    per-tet `Material::sample` returned by `MaterialField`.
//!
//! 5. **Static fit pose** — outer-surface Dirichlet pin (`‖p‖ ≈
//!    R_OUTER` band, half-cell tolerance) + `PenaltyRigidContact::new(
//!    vec![scan_sdf.clone()])` with the SAME scan SDF as the indenter
//!    (post-PR2 trait unification: any `impl cf_design::Sdf` is a
//!    valid rigid primitive directly; row 20 is the first non-plane
//!    consumer of this surface). At rest the cavity walls already
//!    kiss the indenter (the cavity IS scan-shaped, so the body
//!    rest-config has cavity-wall vertices at the scan's zero
//!    isosurface) — the `PenaltyRigidContact`'s `sd < d̂` band gate
//!    therefore activates immediately and the rest-state preload IS
//!    the quantitative "fit-tightness" force readout per the device
//!    memo's "feels right" subjective tightness target.
//!
//! 6. **Readouts** —
//!    - JSON `out/layered_silicone_device.json`: scalars (radii +
//!      `cell_size` + counts + force-total + `n_active_pairs` +
//!      mean/max/min `force_magnitude`) + per-shell tet counts +
//!      material provenance block + per-active-pair detail array
//!      (mirrors row 18's two-section schema).
//!    - PLY `out/device_boundary.ply`: full 3D body via
//!      [`sim_soft::viz::boundary_surface`] (F1.1 lift), with
//!      sequential `displacement_magnitude` + categorical
//!      `material_id` per-vertex (volume-weighted averaged from
//!      per-tet). Replaces the pre-F1.5 z-slab centroid cloud's
//!      reduce-to-2D framing with the canonical FEM-viz convention.
//!    - PLY `out/device_slab_cut_z0.ply`: equatorial cross-section
//!      at `z = 0` via [`sim_soft::viz::slab_cut`] (F1.1 lift).
//!      Marching-tetrahedra intersection; per-vertex scalars
//!      linearly interpolated along cross-edges. Exposes both the
//!      radial material-shell partition AND the cavity-wall
//!      displacement at the equator from one cut — the same
//!      view the pre-F1.5 z-slab centroid cloud expressed, but as
//!      a proper triangulated surface.
//!    - PLY `out/device_design_slab_cut_z0.ply`: same equatorial
//!      cut at `z = 0` via [`sim_soft::viz::design_slab_cut`]
//!      (F2.0 lift). Marching-squares-filled on the design SDF
//!      (`Solid::sphere(R_OUTER).subtract(scan_cube)`) — produces a
//!      smooth circle minus a sharp axis-aligned square cross-section,
//!      decoupled from the analysis-mesh discretization. Per-vertex
//!      scalars come from barycentric interp against the analysis
//!      tet mesh, so the same three scalars surface but rendered on
//!      the design intent rather than the polyhedral approximation.
//!      Pair with the F1 `slab_cut` artifact for the analysis-mesh-
//!      vs-design-mesh side-by-side that motivated the F2 arc.
//!    - PLY `out/device_design_surface.ply`: full 3D body via
//!      [`sim_soft::viz::design_surface`] (F2.1 lift). Marching cubes
//!      on the design SDF — produces a smooth sphere boundary
//!      (without the BCC sliver-tet zigzag the F1 `boundary_surface`
//!      artifact shows on curved surfaces). Same scalar-transfer
//!      convention as `design_slab_cut`. Pair with the F1
//!      `boundary_surface` artifact for the analysis-mesh-vs-design-
//!      mesh boundary side-by-side. On organic 3D-scanned bodies (the
//!      production target), `design_surface` follows the scan's actual
//!      shape rather than the BCC lattice's approximation — the
//!      load-bearing payoff of the F2 arc.
//!    - PLY `out/device_design_surface_deformed.ply`: same body via
//!      [`sim_soft::viz::design_surface_deformed`] (F2.3a) but with
//!      each surface vertex offset by the per-vertex displacement
//!      field interpolated from the analysis-mesh deformation,
//!      amplified by `amplify=10` to make row 20's mm-scale
//!      deformation visible on the cm-scale body. Cavity walls show
//!      clear inward squashing where the rigid scan-cube indenter
//!      presses; the outer sphere boundary deforms much less
//!      (silicone bulk absorbs the contact force).
//!    - PLY `out/device_design_scene.ply`: body + scan-derived rigid
//!      indenter merged into one mesh via [`sim_soft::viz::design_scene`]
//!      (F2.3b), with a categorical `primitive_id` scalar (0 = body,
//!      1 = indenter). cf-view's Scalar dropdown toggles between
//!      `primitive_id` (categorical body-vs-indenter color split) and
//!      the body's continuous scalars (deformation / strain / `material_id`;
//!      uniform 0 on the rigid indenter).
//!    - `verify_*` runtime gates — pipeline-emergent structural and
//!      physics invariants (see "Numerical anchors" in `README.md`).
//!      This row is a Rule-B `validator`: its oracles are read from the
//!      real solve (positive tet volume, converged Newton, per-shell
//!      material routing, non-empty shell populations, engaged contact).
//!      The pre-Rule-B captured-bit self-pins (per-shell and
//!      contact-pair-count freezes, force/displacement `to_bits()` pins,
//!      the `to_neo_hookean()` provenance mirror) were stripped, since
//!      constitutive and mesher correctness is lib-owned. See the
//!      de-frag note over the `Verifications` section.
//!
//! # Why both boundary-surface and slab-cut
//!
//! Pattern (aa) banked at row 16 N+3: hollow / interior-cavity /
//! partial-occlusion bodies (the doubly-hollow layered silicone
//! device — scan-shaped cavity AND three concentric material
//! shells — qualifies on both counts) need an axis-aligned cut to
//! expose interior structure. The F1.5 retrofit (replacing the
//! pre-F1.5 z-slab centroid cloud) emits BOTH primitives: the
//! boundary-surface gives the 3D outer shape with the contact-zone
//! displacement glow, while the slab-cut at `z = 0` cuts through
//! the body to expose the radial material shells + cavity wall in
//! one frame. Together they cover the visualization story the
//! pre-F1.5 z-slab + amplified-displacement convention compressed
//! into a single artifact.
//!
//! [mem16]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_row_16_patterns.md
//!
//! # Sanitization
//!
//! Per the [device memo][mem]'s sanitization directive: the scanned
//! reference geometry is referred to as "scanned reference geometry"
//! or "scan-derived rigid indenter" throughout this crate's prose.
//! No anatomical references appear in any tracked surface. The 12-tri
//! cube placeholder is a programmatic synthetic stand-in — the
//! pipeline demonstration is the workflow ("scan → `Signed<TriMeshDistance, _>`
//! → cf-design `Solid` → `SdfMeshedTetMesh` → 3-material FEM → contact"),
//! not the cube's specific geometry; production runs swap the cube
//! fixture for a real scan via row 15's STL-import path without any
//! other code change.
//!
//! # Run
//!
//! ```sh
//! cargo run -p example-sim-soft-layered-silicone-device --release
//! ```
//!
//! Per `feedback_release_mode_heavy_tests` — release mode is required
//! for the FEM solve at this mesh resolution (~51 k tets through
//! faer's sparse Cholesky); debug mode would take many minutes for
//! what runs in seconds release.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use anyhow::Result;
use cf_design::Solid;
use mesh_io::save_ply_attributed;
use mesh_sdf::{PseudoNormalSign, Signed, TriMeshDistance};

/// Local alias — the canonical parry-pseudo-normal composition,
/// matching the sister `mesh-scan-as-solid` example's `ScanSdf` for
/// cross-file consistency. [`PseudoNormalSign`] is the cheap parry
/// pseudo-normal path; cleaned body-part scans should prefer
/// `mesh_sdf::flood_filled_sdf` per
/// `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md`.
type ScanSdf = Signed<TriMeshDistance, PseudoNormalSign>;
use mesh_types::{IndexedMesh, Point3};
use nalgebra::Matrix3;
use serde_json::{Value, json};
use sim_ml_chassis::Tensor;
use sim_soft::material::silicone_table::{DRAGON_SKIN_10A, ECOFLEX_00_30};
use sim_soft::{
    Aabb3, BoundaryConditions, CpuNewtonSolver, Field, LayeredScalarField, Material, MaterialField,
    Mesh, MeshingHints, NewtonStep, PenaltyRigidContact, PenaltyRigidContactSolver, Plane,
    SceneInitial, Sdf, SdfMeshedTetMesh, Solver, SolverConfig, SphereSdf, Tet4, Vec3, VertexId,
    boundary_surface, design_scene, design_slab_cut, design_surface, design_surface_deformed,
    pick_vertices_by_predicate, referenced_vertices, slab_cut,
};

// =============================================================================
// Constants — body geometry
// =============================================================================

/// Outer body radius (m) — matches PR1 row 11 + PR3 row 16's
/// `LAYERED_SPHERE_R_OUTER` value visually but is row-20-specific (no
/// cross-row bit-pin, since this row's materials diverge from the
/// `(μ, λ) = (1e5, 4e5)` baseline rows 11 + 16 share — see pattern (y)
/// caveat at row 19's banking memo).
const R_OUTER: f64 = 0.10;

/// Outer/middle interface radius (m). Uniform `0.025 m` shell
/// thickness across all three layers — row-20-native partition.
const R_MIDDLE_OUTER: f64 = 0.075;

/// Middle/inner interface radius (m). Uniform `0.025 m` shell
/// thickness.
const R_INNER_OUTER: f64 = 0.05;

/// Programmatic 12-tri cube scan-fixture half-extent (m). Sized so the
/// cube's max-extent from origin is `|SCAN_OFFSET_X| + R_SCAN = 0.040
/// m`, matching rows 11 + 16's `R_CAVITY = 0.04 m` visually but the
/// cube IS NOT a sphere — the asymmetric offset makes the carved
/// cavity demonstrably non-spherical, which is the load-bearing
/// pedagogy of this row.
const R_SCAN: f64 = 0.025;

/// Scan-fixture centre offset along the +x axis (m). Asymmetric so
/// the carved cavity is offset from the body origin, demonstrating
/// non-spherical heterogeneous-CSG. The cavity max-extent from origin
/// is `|SCAN_OFFSET_X| + R_SCAN = 0.040 m`; min-extent is
/// `|SCAN_OFFSET_X| - R_SCAN = -0.010 m` (the cube does cross the
/// origin along x).
const SCAN_OFFSET_X: f64 = 0.015;

/// Bbox half-extent for the `MeshingHints` evaluation domain (m).
/// Matches rows 11 + 16's `LAYERED_SPHERE_BBOX_HALF_EXTENT` verbatim;
/// `R_OUTER + 2 * cell_size` headroom over the body's outer radius
/// keeps every BCC lattice corner that touches the body inside the
/// bbox.
const BBOX_HALF_EXTENT: f64 = 0.12;

// =============================================================================
// Constants — meshing + solver
// =============================================================================

/// BCC lattice spacing (m). 2× finer than the original row-20 0.02 m
/// (which was inherited from rows 11 + 16's IV-3/IV-4/IV-5
/// mid-refinement choice; `~5–7 k tets` after the scan-shaped cavity
/// carve). The original coarse mesh approximated the spherical outer
/// surface as a low-poly envelope (only ~5 lattice cells across the
/// `R_OUTER = 0.10 m` radius), which the F1.5-lifted
/// `boundary_surface` + `slab_cut` PLY emits faithfully exposed —
/// the pre-F1.5 z-slab centroid PLY hid the surface approximation
/// because centroids are interior-only. Refined to 0.01 m (~10
/// lattice cells across the radius, ~51 k tets) so the spherical
/// envelope is visibly smoother in the slab cut.
const CELL_SIZE: f64 = 0.01;

/// Static-regime time step (s). Mirrors rows 11 + 14 + 16 + 18's
/// `STATIC_DT` verbatim: at large `dt`, the backward-Euler residual's
/// inertial term `M / dt² · (x − x_prev)` collapses ~4 orders below
/// the stiffness contribution, so a single `replay_step` from rest
/// converges to the static equilibrium far below `tol = 1e-10`.
const STATIC_DT: f64 = 1.0;

/// Newton iter cap. Bumped from 50 → 200 alongside the
/// `CELL_SIZE = 0.02 → 0.01` refinement: at the finer mesh the
/// initial overlap between the cavity walls (which sit on the scan
/// SDF zero-set by construction) and the indenter spans more
/// vertices, so the contact-onset Newton iteration's residual takes
/// more steps to drive below `tol = 1e-10`. Empirical convergence
/// at the refined mesh is ~70-100 iters at first call from rest;
/// 200 leaves comfortable headroom for future load perturbations.
const MAX_NEWTON_ITER: usize = 200;

// =============================================================================
// Programmatic 12-tri cube scan fixture
// =============================================================================

/// Build the 12-triangle axis-aligned cube scan-fixture, half-extent
/// `R_SCAN` along each axis, translated by `SCAN_OFFSET_X` along `+x`
/// (so the cube is centred at `(SCAN_OFFSET_X, 0, 0)`).
/// Outward face winding produces the 6 cube-face outward normals
/// `±x̂, ±ŷ, ±ẑ` from the 12 cross products. Mirrors row 15
/// `mesh-scan-as-solid::unit_cube` verbatim, with translation applied
/// to the 8 corner vertices.
///
/// The "scan" framing is the workflow, not the geometry — production
/// runs replace this fixture with `mesh_io::load_mesh(scan.stl)` then
/// the explicit `TriMeshDistance::new(loaded)` + `PseudoNormalSign::from_distance`
/// composition, and every downstream code path
/// (`Solid::from_sdf` + `SdfMeshedTetMesh::from_sdf` + the contact
/// `PenaltyRigidContact::new(scan.clone())` line) stays unchanged.
fn build_scan_fixture() -> ScanSdf {
    let mut mesh = IndexedMesh::new();
    let cx = SCAN_OFFSET_X;

    // 8 corners of the axis-aligned cube `[cx-R, cx+R] × [-R, +R]^2`.
    for &(sx, sy, sz) in &[
        (-1.0_f64, -1.0_f64, -1.0_f64),
        (1.0, -1.0, -1.0),
        (1.0, 1.0, -1.0),
        (-1.0, 1.0, -1.0),
        (-1.0, -1.0, 1.0),
        (1.0, -1.0, 1.0),
        (1.0, 1.0, 1.0),
        (-1.0, 1.0, 1.0),
    ] {
        mesh.vertices.push(Point3::new(
            sx.mul_add(R_SCAN, cx),
            sy * R_SCAN,
            sz * R_SCAN,
        ));
    }

    // 12 outward-CCW triangles (2 per face × 6 faces). Face indices
    // mirror row 15: -z (3 -> 1 -> 0; 3 -> 2 -> 1), +z (4 -> 5 -> 6;
    // 4 -> 6 -> 7), -x (0 -> 4 -> 7; 0 -> 7 -> 3), +x (2 -> 6 -> 5;
    // 2 -> 5 -> 1), -y (0 -> 1 -> 5; 0 -> 5 -> 4), +y (3 -> 7 -> 6;
    // 3 -> 6 -> 2).
    for tri in [
        [3, 1, 0],
        [3, 2, 1],
        [4, 5, 6],
        [4, 6, 7],
        [0, 4, 7],
        [0, 7, 3],
        [2, 6, 5],
        [2, 5, 1],
        [0, 1, 5],
        [0, 5, 4],
        [3, 7, 6],
        [3, 6, 2],
    ] {
        mesh.faces.push(tri);
    }

    // Synthetic CCW cube — both PseudoNormalSign and FloodFillSign
    // produce correct signs here. PseudoNormalSign is the cheap parry
    // pseudo-normal path; no flood-fill grid build cost. Production
    // cleaned-scan code paths should prefer `mesh_sdf::flood_filled_sdf`
    // — see [[project-mesh-sdf-oracle-decomposition-spec]].
    let distance =
        TriMeshDistance::new(mesh).expect("12-tri cube fixture is non-empty by construction");
    let sign = PseudoNormalSign::from_distance(&distance);
    Signed { distance, sign }
}

/// Conservative bbox for the scan fixture — `[-R_SCAN + cx, R_SCAN + cx]`
/// along x with one-cell-edge slack on every axis to keep the SDF
/// evaluation domain comfortably inside the meshing bbox.
fn scan_bounds() -> cf_design::Aabb {
    let pad = CELL_SIZE;
    cf_design::Aabb::new(
        Point3::new(SCAN_OFFSET_X - R_SCAN - pad, -R_SCAN - pad, -R_SCAN - pad),
        Point3::new(SCAN_OFFSET_X + R_SCAN + pad, R_SCAN + pad, R_SCAN + pad),
    )
}

// =============================================================================
// Body builder — heterogeneous CSG via PR3 F5
// =============================================================================

/// Compose the layered-silicone-device body — outer parametric sphere
/// MINUS the scan-derived cavity. The scan SDF is bridged into the
/// typed-Solid kernel via `Solid::from_sdf` (PR3 F5); the resulting
/// `Solid` composes with parametric primitives via the standard
/// `subtract` / `union` / `intersect` API.
fn build_body(scan_sdf: ScanSdf) -> Solid {
    let cavity = Solid::from_sdf(scan_sdf, scan_bounds());
    Solid::sphere(R_OUTER).subtract(cavity)
}

// =============================================================================
// 3-shell MaterialField — outer ECOFLEX_00_30 / middle DRAGON_SKIN_10A
// / inner ECOFLEX_00_30 via F4 const entries
// =============================================================================

/// Three-shell `MaterialField` per the layered-silicone-device memo:
/// outer + inner = `ECOFLEX_00_30`; middle = `DRAGON_SKIN_10A` as the
/// closest pure-silicone proxy for the conductive-composite middle
/// layer (silicone matrix only — Cu mesh + carbon black contributions
/// are deferred to a Fork-B post-cast modulus calibration). Partition
/// is by `phi = ‖p‖ - R_OUTER` radial-bin (NOT scan-shaped) — the
/// material classes are concentric shells with parametric thresholds
/// independent of the cavity asymmetry.
fn build_material_field() -> MaterialField {
    // The SDF driving the partition is a pure parametric sphere at
    // `R_OUTER` — `phi = ‖p‖ - R_OUTER` is the LayeredScalarField's
    // threshold variable. The cavity-asymmetry has no role here.
    let body_sdf = || Box::new(SphereSdf { radius: R_OUTER }) as Box<dyn Sdf>;
    let phi_inner = R_INNER_OUTER - R_OUTER; // ~ -0.05
    let phi_middle = R_MIDDLE_OUTER - R_OUTER; // ~ -0.025

    // F4 const Lamé pairs (Pa). `to_neo_hookean()` round-trips the
    // pair bit-equally per F4's `to_neo_hookean_round_trips_lame_pair`
    // unit test; any drift here means F4 drifted.
    let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        body_sdf(),
        vec![phi_inner, phi_middle],
        vec![ECOFLEX_00_30.mu, DRAGON_SKIN_10A.mu, ECOFLEX_00_30.mu],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        body_sdf(),
        vec![phi_inner, phi_middle],
        vec![
            ECOFLEX_00_30.lambda,
            DRAGON_SKIN_10A.lambda,
            ECOFLEX_00_30.lambda,
        ],
    ));
    MaterialField::from_fields(mu_field, lambda_field)
}

// =============================================================================
// Per-tet shell classification (mirrors row 11's centroid-bin idiom)
// =============================================================================

/// Shell index of a per-tet centroid: 0 = inner / 1 = middle / 2 =
/// outer, by radial `phi = ‖p‖ − R_OUTER` bin. The boundary convention
/// is **open-left, closed-right** (strict `<`): a centroid at the inner
/// threshold belongs to the middle shell, and one at the middle
/// threshold belongs to the outer shell. This matches
/// `LayeredScalarField::sample`'s `partition_point(|&t| t <= phi)` idiom
/// (the predicate is true at threshold equality, so `partition_point`
/// steps PAST it — equivalent to the strict-`<` chain here), so the
/// per-tet classification agrees with the `MaterialField`'s sample at
/// the centroid bit-equally — the invariant `verify_material_routing`
/// gates. (Row 20's sphere geometry + BCC centroids don't land on the
/// exact thresholds at any meaningful fraction of tets, so `<` vs `<=`
/// is behaviourally identical here; strict `<` is the convention that
/// stays correct if a future scan geometry does produce boundary hits,
/// mirroring row 21's `shell_at_phi`.)
fn shell_at(centroid: Vec3) -> usize {
    let phi = centroid.norm() - R_OUTER;
    if phi < R_INNER_OUTER - R_OUTER {
        0 // inner
    } else if phi < R_MIDDLE_OUTER - R_OUTER {
        1 // middle
    } else {
        2 // outer
    }
}

// =============================================================================
// Mesh build via PR3 F1+F3+F5
// =============================================================================

fn build_hints(material_field: MaterialField) -> MeshingHints {
    MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(-BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT),
            Vec3::new(BBOX_HALF_EXTENT, BBOX_HALF_EXTENT, BBOX_HALF_EXTENT),
        ),
        cell_size: CELL_SIZE,
        material_field: Some(material_field),
    }
}

// =============================================================================
// BoundaryConditions — outer-surface Dirichlet pin only
// =============================================================================

/// Build the BC: outer-surface Dirichlet pin (`‖p‖ ≈ R_OUTER` band) +
/// empty `loaded_vertices` (no distributed pressure — the contact
/// penalty handles all loading at the cavity wall). Mirrors row 14
/// `compressive-block`'s contact-only loading idiom.
fn build_boundary_conditions(
    mesh: &SdfMeshedTetMesh,
    referenced: &[VertexId],
) -> BoundaryConditions {
    let referenced_set: BTreeSet<VertexId> = referenced.iter().copied().collect();
    let band_tol = 0.5 * CELL_SIZE;
    let pinned: Vec<VertexId> =
        pick_vertices_by_predicate(mesh, |p| (p.norm() - R_OUTER).abs() < band_tol)
            .into_iter()
            .filter(|v| referenced_set.contains(v))
            .collect();
    assert!(
        !pinned.is_empty(),
        "outer-surface band turned up empty at cell_size = {CELL_SIZE}",
    );

    BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: Vec::new(),
    }
}

// =============================================================================
// Solver run — one backward-Euler replay_step with PenaltyRigidContact
// =============================================================================

struct SolveResult {
    rest_positions: Vec<Vec3>,
    step: NewtonStep<sim_soft::CpuTape>,
}

fn solve_static(
    mesh: SdfMeshedTetMesh,
    contact: PenaltyRigidContact,
    bc: BoundaryConditions,
) -> SolveResult {
    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();
    let n_dof = 3 * rest_positions.len();
    let mut x_prev_flat = vec![0.0; n_dof];
    for (v, pos) in rest_positions.iter().enumerate() {
        x_prev_flat[3 * v] = pos.x;
        x_prev_flat[3 * v + 1] = pos.y;
        x_prev_flat[3 * v + 2] = pos.z;
    }
    let initial = SceneInitial {
        x_prev: Tensor::from_slice(&x_prev_flat, &[n_dof]),
        v_prev: Tensor::zeros(&[n_dof]),
    };

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;

    let solver: PenaltyRigidContactSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    // No external traction — contact-only loading. Empty θ matches
    // `assemble_external_force`'s empty-loaded ⇒ length-0-θ assert.
    let empty_theta: [f64; 0] = [];
    let theta_tensor = Tensor::from_slice(&empty_theta, &[0]);
    let step = solver.replay_step(&initial.x_prev, &initial.v_prev, &theta_tensor, cfg.dt);

    SolveResult {
        rest_positions,
        step,
    }
}

// =============================================================================
// Per-tet deformation gradient (mirrors row 22 / row 23 / row 24's
// `deformation_gradient` helper; private to this row, used only for
// the F1.5-follow-up per-tet psi computation feeding the viz emits).
// =============================================================================

fn deformation_gradient(verts: [VertexId; 4], rest: &[Vec3], curr: &[Vec3]) -> Matrix3<f64> {
    let r0 = rest[verts[0] as usize];
    let r1 = rest[verts[1] as usize];
    let r2 = rest[verts[2] as usize];
    let r3 = rest[verts[3] as usize];
    let c0 = curr[verts[0] as usize];
    let c1 = curr[verts[1] as usize];
    let c2 = curr[verts[2] as usize];
    let c3 = curr[verts[3] as usize];
    let d_rest = Matrix3::from_columns(&[r1 - r0, r2 - r0, r3 - r0]);
    let d_curr = Matrix3::from_columns(&[c1 - c0, c2 - c0, c3 - c0]);
    let d_rest_inv = d_rest
        .try_inverse()
        .expect("D_rest is invertible by verify_quality_floors signed-volume gate");
    d_curr * d_rest_inv
}

// =============================================================================
// Verifications — structural + physics gates
// =============================================================================
//
// Rule-B de-frag: the pre-Rule-B captured-bit self-pins (per-shell + z-slab
// tet-count freezes, contact-pair-count freeze, force/displacement `to_bits()`
// pins, `to_neo_hookean()` provenance mirror, and the `CF_CAPTURE_BITS` capture
// scaffold) were STRIPPED — they pinned this run's exact FP trajectory + mesher
// version on one toolchain, strictly more fragile than the pipeline-emergent
// invariants they redundantly implied. Constitutive correctness (`NeoHookean`
// closed form + the `to_neo_hookean()` Lamé round-trip) is lib-owned
// (`neo_hookean.rs` tests + `silicone_table.rs::tests::
// to_neo_hookean_round_trips_lame_pair`); per-shell material routing by radial
// bin is lib-owned (`sdf_material_tagging.rs` IV-4). What survives here is the
// scene's own emergent physics: a valid mesh, a converged solve, correct
// per-shell material assignment, non-empty shell populations, and engaged
// contact — read from the real solve, robust to FP drift.

/// Structural mesh invariants — resolution-robust, no exact-count freeze.
/// Every tet is classified into exactly one radial shell, so the three
/// per-shell counts partition the whole mesh; the referenced set is a subset of
/// all vertices; and the pinned band is a non-empty proper subset of the
/// referenced set. (The exact counts these once pinned were a mesher-version
/// artifact — see the module de-frag note.)
fn verify_mesh_structure(
    mesh: &SdfMeshedTetMesh,
    referenced: &[VertexId],
    pinned: &[VertexId],
    inner_count: usize,
    middle_count: usize,
    outer_count: usize,
) {
    assert!(mesh.n_tets() > 0, "mesh has no tets");
    assert!(
        referenced.len() <= mesh.n_vertices(),
        "referenced vertices ({}) exceed total vertices ({})",
        referenced.len(),
        mesh.n_vertices(),
    );
    assert!(!referenced.is_empty(), "no referenced vertices");
    assert!(
        !pinned.is_empty() && pinned.len() < referenced.len(),
        "pinned band ({}) must be a non-empty proper subset of referenced ({})",
        pinned.len(),
        referenced.len(),
    );
    assert!(inner_count > 0, "inner shell is empty");
    assert!(middle_count > 0, "middle shell is empty");
    assert!(outer_count > 0, "outer shell is empty");
    // (No `inner + middle + outer == n_tets` assert: the caller derives the
    // three counts by iterating every tet through a total match, so the sum
    // equals `n_tets` by construction — it would guard nothing. The per-shell
    // non-empty checks above + `verify_material_routing`'s independent per-shell
    // classification carry the real routing content.)
}

/// z-slab populations — each shell contributes at least one tet to the
/// `|centroid.z| < CELL_SIZE / 2` cut, so the cf-view PLY artifacts show all
/// three material bands. Non-empty, not exact-count (mesher-version robust).
fn verify_zslab_populations(inner_zslab: usize, middle_zslab: usize, outer_zslab: usize) {
    assert!(inner_zslab > 0, "z-slab inner shell empty");
    assert!(middle_zslab > 0, "z-slab middle shell empty");
    assert!(outer_zslab > 0, "z-slab outer shell empty");
}

fn verify_quality_floors(mesh: &SdfMeshedTetMesh) {
    let positions = mesh.positions();
    let n_tets = mesh.n_tets();
    for tet_idx in 0..n_tets {
        let [v0, v1, v2, v3] = mesh.tet_vertices(tet_idx as u32);
        let p0 = positions[v0 as usize];
        let p1 = positions[v1 as usize];
        let p2 = positions[v2 as usize];
        let p3 = positions[v3 as usize];
        let signed_volume = (p1 - p0).cross(&(p2 - p0)).dot(&(p3 - p0)) / 6.0;
        assert!(
            signed_volume > 0.0,
            "tet {tet_idx}: signed_volume = {signed_volume:e} (expected strictly positive — \
             D-10 detector)",
        );
    }
}

fn verify_solver_converges<T>(step: &NewtonStep<T>) {
    assert!(
        step.iter_count < MAX_NEWTON_ITER,
        "Newton iter_count = {} ≥ MAX_NEWTON_ITER = {}",
        step.iter_count,
        MAX_NEWTON_ITER,
    );
    assert!(
        step.final_residual_norm < 1.0e-10,
        "Newton final_residual_norm = {:e} ≥ 1e-10",
        step.final_residual_norm,
    );
}

/// Per-tet material routing — the `MaterialField` assigned every tet the
/// `(μ, λ)` of the radial shell its centroid falls in. Reads the real per-tet
/// `NeoHookean` from `mesh.materials()` via the public `.mu()` / `.lambda()`
/// accessors and compares to the shell's F4 table entry (same const source →
/// bit-equal, so exact `==`, not a tolerance). This is the scene's routing
/// invariant — NOT a constitutive-arithmetic mirror; the closed-form
/// `NeoHookean` energy and the `to_neo_hookean()` Lamé round-trip are lib-owned
/// (`neo_hookean.rs` tests + `silicone_table.rs::tests::
/// to_neo_hookean_round_trips_lame_pair`), and the generic routing mechanism is
/// lib-owned (`sdf_material_tagging.rs` IV-4). Each shell is verified
/// non-vacuously (≥ 1 tet). Shell → material: inner + outer = `ECOFLEX_00_30`,
/// middle = `DRAGON_SKIN_10A`.
fn verify_material_routing(mesh: &SdfMeshedTetMesh, shell_idx_per_tet: &[usize]) {
    let materials = mesh.materials();
    assert_eq!(
        materials.len(),
        shell_idx_per_tet.len(),
        "materials() length does not match per-tet shell-classification length",
    );
    let expected_nh = [
        ECOFLEX_00_30.to_neo_hookean(),   // 0 = inner
        DRAGON_SKIN_10A.to_neo_hookean(), // 1 = middle
        ECOFLEX_00_30.to_neo_hookean(),   // 2 = outer
    ];
    let mut checked = [0usize; 3];
    for (t, &shell_idx) in shell_idx_per_tet.iter().enumerate() {
        let observed = &materials[t];
        let expected = &expected_nh[shell_idx];
        // Bit-equal `==` on purpose: both `(μ, λ)` come from the SAME F4 table
        // entry through the SAME `to_neo_hookean()` const fn, so a correctly
        // routed tet is bit-identical. `to_bits()` makes the exact comparison
        // explicit and clippy-clean (no `float_cmp`).
        assert!(
            observed.mu().to_bits() == expected.mu().to_bits()
                && observed.lambda().to_bits() == expected.lambda().to_bits(),
            "tet {t} shell {shell_idx}: routed (μ, λ) = ({}, {}) != table ({}, {}) \
             (boundary-convention drift between shell_at and LayeredScalarField::sample?)",
            observed.mu(),
            observed.lambda(),
            expected.mu(),
            expected.lambda(),
        );
        checked[shell_idx] += 1;
    }
    assert!(
        checked.iter().all(|&c| c > 0),
        "material routing not exercised on every shell (per-shell tet counts {checked:?})",
    );
}

/// The static fit pose actually engaged contact — at least one active
/// contact pair. (The exact pair count this once froze was a mesher/
/// discretization artifact; non-empty is the invariant that matters.)
fn verify_contact_engaged(n_pairs: usize) {
    assert!(
        n_pairs > 0,
        "no active contact pairs at the static fit pose — cavity walls did not \
         engage the scan-derived indenter",
    );
}

// =============================================================================
// JSON readout
// =============================================================================

#[allow(clippy::too_many_arguments)]
fn write_json_readout(
    path: &Path,
    n_tets: usize,
    n_vertices: usize,
    n_referenced: usize,
    n_pinned: usize,
    inner_count: usize,
    middle_count: usize,
    outer_count: usize,
    n_pairs: usize,
    force_total_z: f64,
    mean_force_magnitude: f64,
    max_force_magnitude: f64,
    mean_disp_magnitude: f64,
    max_disp_magnitude: f64,
    pair_records: &[Value],
) -> Result<()> {
    let scalars = json!({
        "r_outer_m": R_OUTER,
        "r_middle_outer_m": R_MIDDLE_OUTER,
        "r_inner_outer_m": R_INNER_OUTER,
        "r_scan_m": R_SCAN,
        "scan_offset_x_m": SCAN_OFFSET_X,
        "cell_size_m": CELL_SIZE,
        "n_tets": n_tets,
        "n_vertices": n_vertices,
        "n_referenced": n_referenced,
        "n_pinned": n_pinned,
        "n_inner_tets": inner_count,
        "n_middle_tets": middle_count,
        "n_outer_tets": outer_count,
        "n_contact_pairs": n_pairs,
        "force_total_z_n": force_total_z,
        "mean_force_magnitude_n": mean_force_magnitude,
        "max_force_magnitude_n": max_force_magnitude,
        "mean_displacement_magnitude_m": mean_disp_magnitude,
        "max_displacement_magnitude_m": max_disp_magnitude,
    });
    let materials = json!([
        {
            "name": "ECOFLEX_00_30",
            "shell": "outer",
            "n_tets": outer_count,
            "mu_pa": ECOFLEX_00_30.mu,
            "lambda_pa": ECOFLEX_00_30.lambda,
            "density_kg_m3": ECOFLEX_00_30.density,
        },
        {
            "name": "DRAGON_SKIN_10A",
            "shell": "middle",
            "n_tets": middle_count,
            "mu_pa": DRAGON_SKIN_10A.mu,
            "lambda_pa": DRAGON_SKIN_10A.lambda,
            "density_kg_m3": DRAGON_SKIN_10A.density,
        },
        {
            "name": "ECOFLEX_00_30",
            "shell": "inner",
            "n_tets": inner_count,
            "mu_pa": ECOFLEX_00_30.mu,
            "lambda_pa": ECOFLEX_00_30.lambda,
            "density_kg_m3": ECOFLEX_00_30.density,
        },
    ]);
    let document = json!({
        "scalars": scalars,
        "material_layers": materials,
        "contact_pairs": pair_records,
    });
    std::fs::write(path, serde_json::to_string_pretty(&document)?)?;
    Ok(())
}

// =============================================================================
// (PLY z-slab inline scratch helpers retired at F1.5)
// =============================================================================
//
// The pre-F1.5 inline `emit_zslab_ply` + `ZslabRecord` (with
// DISPLACEMENT_SCALE = 50.0 amplification) emitted a z-slab per-tet
// centroid cloud. Lifted to [`sim_soft::viz::boundary_surface`] +
// [`sim_soft::viz::slab_cut`] at F1.1 / F1.5 retrofit; row 20 now
// emits the full 3D body + the proper triangulated cross-section
// rather than an amplified centroid cloud. Z-slab per-shell
// population gate (`verify_zslab_populations`) survives — cheap
// centroid filter, no PLY data.

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    println!("layered-silicone-device — PR3 row 20 (Tier 6 synthesis)");
    println!();

    // 1. Build the scan-derived rigid indenter (programmatic 12-tri
    //    cube fixture; production swaps for an STL-loaded scan).
    let scan_sdf = build_scan_fixture();

    // 2. Heterogeneous-CSG body via PR3 F5 sugar.
    let body = build_body(scan_sdf.clone());

    // 3. 3-shell MaterialField via F4 const entries.
    let material_field = build_material_field();

    // 4. Mesh through SdfMeshedTetMesh::from_sdf — F1+F3 trait
    //    dispatch on `&dyn cf_design::Sdf`.
    let hints = build_hints(material_field);
    let mesh = SdfMeshedTetMesh::from_sdf(&body, &hints).map_err(|e| anyhow::anyhow!("{e:?}"))?;

    // Snapshot pre-solver views of the mesh (positions, tets, n_tets,
    // n_vertices) for the verify_* + zslab pipelines BEFORE the mesh
    // is moved into the solver. `Mesh::tet_vertices(t)` is the public
    // accessor for the per-tet `[VertexId; 4]` (the field itself is
    // private).
    let n_tets = mesh.n_tets();
    let n_vertices = mesh.n_vertices();
    let positions: Vec<Vec3> = mesh.positions().to_vec();
    let tets: Vec<[VertexId; 4]> = (0..n_tets as u32).map(|t| mesh.tet_vertices(t)).collect();
    let referenced = referenced_vertices(&mesh);

    // 5. BC + per-shell counts at rest from centroid bins.
    let bc = build_boundary_conditions(&mesh, &referenced);
    // Snapshot the pinned count before bc is moved into the solver.
    let n_pinned = bc.pinned_vertices.len();

    // Per-shell tet counts at rest centroid bins.
    let mut n_inner = 0usize;
    let mut n_middle = 0usize;
    let mut n_outer = 0usize;
    let mut shell_idx_per_tet: Vec<usize> = Vec::with_capacity(n_tets);
    for &[v0, v1, v2, v3] in &tets {
        let centroid = (positions[v0 as usize]
            + positions[v1 as usize]
            + positions[v2 as usize]
            + positions[v3 as usize])
            / 4.0;
        let s = shell_at(centroid);
        shell_idx_per_tet.push(s);
        match s {
            0 => n_inner += 1,
            1 => n_middle += 1,
            _ => n_outer += 1,
        }
    }

    // 6. Quality + structure gates BEFORE moving the mesh into the
    //    solver. (`mesh.n_tets()` etc. work on `&mesh`; the
    //    `verify_quality_floors` walk is read-only.)
    verify_quality_floors(&mesh);
    verify_mesh_structure(
        &mesh,
        &referenced,
        &bc.pinned_vertices,
        n_inner,
        n_middle,
        n_outer,
    );

    // 7. Material routing gate — every tet was assigned its radial
    //    shell's (μ, λ) by the `MaterialField`. Reads real
    //    `mesh.materials()` via `.mu()` / `.lambda()`; the constitutive
    //    math + `to_neo_hookean()` round-trip are lib-owned.
    verify_material_routing(&mesh, &shell_idx_per_tet);

    // 8. Solver run — penalty contact with the scan as rigid
    //    indenter (post-PR2 trait unification: any `impl Sdf` is a
    //    valid rigid primitive directly via `PenaltyRigidContact::
    //    new`; row 20 is the first non-plane consumer).
    let contact = PenaltyRigidContact::new(vec![scan_sdf.clone()]);
    let SolveResult {
        rest_positions,
        step,
    } = solve_static(mesh, contact, bc);

    verify_solver_converges(&step);

    // 9. Per-pair contact readout — build a fresh inspection mesh +
    //    contact (the originals were moved into the solver) and call
    //    `per_pair_readout` against the deformed positions. Mirrors
    //    row 18's two-scene idiom.
    let inspection_mesh = {
        let body_again = build_body(scan_sdf.clone());
        let inspection_hints = build_hints(build_material_field());
        SdfMeshedTetMesh::from_sdf(&body_again, &inspection_hints)
            .map_err(|e| anyhow::anyhow!("{e:?}"))?
    };
    let inspection_contact = PenaltyRigidContact::new(vec![scan_sdf]);

    let positions_vec3: Vec<Vec3> = (0..n_vertices)
        .map(|i| {
            Vec3::new(
                step.x_final[3 * i],
                step.x_final[3 * i + 1],
                step.x_final[3 * i + 2],
            )
        })
        .collect();
    let readouts = inspection_contact.per_pair_readout(&inspection_mesh, &positions_vec3);
    let n_pairs = readouts.len();
    verify_contact_engaged(n_pairs);

    let force_total_z: f64 = readouts.iter().map(|r| r.force_on_soft.z).sum();
    let force_magnitudes: Vec<f64> = readouts.iter().map(|r| r.force_on_soft.norm()).collect();
    let mean_force_magnitude = force_magnitudes.iter().sum::<f64>() / n_pairs as f64;
    let max_force_magnitude = force_magnitudes
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);

    // Cavity-wall displacement statistics — vertices on the cavity
    // surface (the active-contact-pair vertex set is a tight subset).
    let cavity_vertex_ids: BTreeSet<VertexId> = readouts
        .iter()
        .map(|r| match r.pair {
            sim_soft::ContactPair::Vertex { vertex_id, .. } => vertex_id,
        })
        .collect();
    let disp_magnitudes: Vec<f64> = cavity_vertex_ids
        .iter()
        .map(|&v| (positions_vec3[v as usize] - rest_positions[v as usize]).norm())
        .collect();
    let mean_disp_magnitude = disp_magnitudes.iter().sum::<f64>() / disp_magnitudes.len() as f64;
    let max_disp_magnitude = disp_magnitudes
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);

    // 10. JSON + PLY readouts.
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;

    // Per-pair JSON records — mirrors row 18's per-active-pair detail
    // block. Bounded by `n_pairs` (~440) so the JSON stays under a
    // few hundred KB.
    let pair_records: Vec<Value> = readouts
        .iter()
        .map(|r| {
            let sim_soft::ContactPair::Vertex {
                vertex_id,
                primitive_id,
            } = r.pair;
            json!({
                "vertex_id": vertex_id,
                "primitive_id": primitive_id,
                "position_x_m": r.position.x,
                "position_y_m": r.position.y,
                "position_z_m": r.position.z,
                "sd_m": r.sd,
                "force_x_n": r.force_on_soft.x,
                "force_y_n": r.force_on_soft.y,
                "force_z_n": r.force_on_soft.z,
            })
        })
        .collect();
    let json_path = out_dir.join("layered_silicone_device.json");
    write_json_readout(
        &json_path,
        n_tets,
        n_vertices,
        referenced.len(),
        n_pinned,
        n_inner,
        n_middle,
        n_outer,
        n_pairs,
        force_total_z,
        mean_force_magnitude,
        max_force_magnitude,
        mean_disp_magnitude,
        max_disp_magnitude,
        &pair_records,
    )?;

    // PLY emits via `sim_soft::viz` public API (F1.5 retrofit; see
    // `sim/L0/soft/src/viz/mod.rs` + `project_sim_soft_viz_arc.md`).
    // The `inspection_mesh` (built post-solver from the same SDF +
    // hints, deterministic BCC + IS → bit-equal mesh) carries the
    // `&dyn Mesh<NeoHookean>` access the public viz API needs; the
    // original `mesh` was moved into the solver above.
    let half_cell = 0.5 * CELL_SIZE;

    // z-slab per-shell population gate (cheap centroid filter — no
    // PLY data accumulation; pre-F1.5 PLY-emit path retired). Each
    // shell must be non-empty in the cut so the cross-section artifacts
    // show all three material bands.
    let mut n_inner_z = 0usize;
    let mut n_middle_z = 0usize;
    let mut n_outer_z = 0usize;
    for (tet_idx, &[v0, v1, v2, v3]) in tets.iter().enumerate() {
        let rest_centroid = (rest_positions[v0 as usize]
            + rest_positions[v1 as usize]
            + rest_positions[v2 as usize]
            + rest_positions[v3 as usize])
            / 4.0;
        if rest_centroid.z.abs() >= half_cell {
            continue;
        }
        match shell_idx_per_tet[tet_idx] {
            0 => n_inner_z += 1,
            1 => n_middle_z += 1,
            _ => n_outer_z += 1,
        }
    }
    verify_zslab_populations(n_inner_z, n_middle_z, n_outer_z);

    // Per-tet displacement magnitude across the full mesh + per-tet
    // material id (radial shell index). Both feed boundary-surface
    // and slab-cut emits via the public viz API.
    let displacement_per_tet: Vec<f64> = tets
        .iter()
        .map(|&[v0, v1, v2, v3]| {
            let rest = (rest_positions[v0 as usize]
                + rest_positions[v1 as usize]
                + rest_positions[v2 as usize]
                + rest_positions[v3 as usize])
                / 4.0;
            let deformed = (positions_vec3[v0 as usize]
                + positions_vec3[v1 as usize]
                + positions_vec3[v2 as usize]
                + positions_vec3[v3 as usize])
                / 4.0;
            (deformed - rest).norm()
        })
        .collect();
    let material_id_per_tet: Vec<f64> = shell_idx_per_tet.iter().map(|&s| s as f64).collect();

    // Per-tet strain-energy density (`psi_j_per_m3`) at the static
    // fit pose. Same `Material::energy(F_t)` per-tet computation
    // rows 22/23/24 use; row 20 didn't track this in pre-F1.5 prose
    // because the device's headline JSON readouts were force +
    // displacement (cavity-fit tightness story). The viz emits get
    // it now as a 3rd scalar so cf-view's dropdown surfaces the
    // strain-energy-density heatmap on the boundary surface + cut.
    let materials_for_psi = inspection_mesh.materials();
    let psi_per_tet: Vec<f64> = tets
        .iter()
        .enumerate()
        .map(|(t, &verts)| {
            let f = deformation_gradient(verts, &rest_positions, &positions_vec3);
            materials_for_psi[t].energy(&f)
        })
        .collect();

    let mut per_tet_scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    per_tet_scalars.insert("displacement_magnitude", &displacement_per_tet);
    per_tet_scalars.insert("material_id", &material_id_per_tet);
    per_tet_scalars.insert("psi_j_per_m3", &psi_per_tet);

    let bd_ply_path = out_dir.join("device_boundary.ply");
    let bd_attr =
        boundary_surface(&inspection_mesh, &per_tet_scalars).map_err(|e| anyhow::anyhow!("{e}"))?;
    save_ply_attributed(&bd_attr, &bd_ply_path, true)?;

    let slab_ply_path = out_dir.join("device_slab_cut_z0.ply");
    let slab_attr = slab_cut(
        &inspection_mesh,
        Plane {
            axis: 2,
            value: 0.0,
        },
        &per_tet_scalars,
    )
    .map_err(|e| anyhow::anyhow!("{e}"))?;
    save_ply_attributed(&slab_attr, &slab_ply_path, true)?;

    // F2.0 design-mesh slab cut at z = 0. Marching-squares on the design
    // SDF (sphere - scan_cube) + barycentric scalar interpolation from
    // the analysis tet mesh — decouples display from sim mesh per the
    // F2 viz arc, so the cross-section is a clean smooth circle minus
    // sharp axis-aligned square regardless of the analysis-mesh
    // discretization. Emits alongside the F1 tet-based artifact so
    // both visualization conventions stay available.
    let body_for_viz = build_body(build_scan_fixture());
    let design_slab_bounds = Aabb3::new(
        Vec3::new(-BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT),
        Vec3::new(BBOX_HALF_EXTENT, BBOX_HALF_EXTENT, BBOX_HALF_EXTENT),
    );
    let design_slab_resolution = CELL_SIZE / 4.0;
    let design_slab_attr = design_slab_cut(
        &body_for_viz,
        &inspection_mesh,
        Plane {
            axis: 2,
            value: 0.0,
        },
        &design_slab_bounds,
        design_slab_resolution,
        &per_tet_scalars,
    )
    .map_err(|e| anyhow::anyhow!("{e}"))?;
    let design_slab_path = out_dir.join("device_design_slab_cut_z0.ply");
    save_ply_attributed(&design_slab_attr, &design_slab_path, true)?;

    // F2.1 design-mesh boundary surface. Marching cubes on the design
    // SDF (sphere - scan_cube) + barycentric scalar interp from the
    // analysis tet mesh — produces a clean smooth sphere boundary
    // (no BCC sliver-tet zigzag like the F1 boundary_surface). Emits
    // alongside the F1 tet-based artifact so both visualization
    // conventions stay available.
    let design_surface_resolution = CELL_SIZE / 4.0;
    let design_surface_attr = design_surface(
        &body_for_viz,
        &inspection_mesh,
        &design_slab_bounds,
        design_surface_resolution,
        &per_tet_scalars,
    )
    .map_err(|e| anyhow::anyhow!("{e}"))?;
    let design_surface_path = out_dir.join("device_design_surface.ply");
    save_ply_attributed(&design_surface_attr, &design_surface_path, true)?;

    // F2.3a deformed design surface — apply per-vertex displacement
    // (interpolated from the analysis-mesh deformation field via
    // barycentric) to the rest-config design_surface vertices,
    // amplified for visibility on row 20's mm-scale deformation.
    // The cavity walls show clear inward squashing where the rigid
    // scan-cube indenter presses from inside; the outer sphere
    // boundary deforms much less (silicone bulk absorbs the force).
    let displacement_per_vertex: Vec<Vec3> = (0..n_vertices)
        .map(|i| positions_vec3[i] - rest_positions[i])
        .collect();
    let amplify = 10.0_f64;
    let deformed_attr = design_surface_deformed(
        &body_for_viz,
        &inspection_mesh,
        &design_slab_bounds,
        design_surface_resolution,
        &per_tet_scalars,
        &displacement_per_vertex,
        amplify,
    )
    .map_err(|e| anyhow::anyhow!("{e}"))?;
    let deformed_path = out_dir.join("device_design_surface_deformed.ply");
    save_ply_attributed(&deformed_attr, &deformed_path, true)?;

    // F2.3b design scene: body + contact primitives merged into one
    // PLY with a categorical primitive_id scalar. cf-view toggles
    // between primitive_id (categorical body-vs-indenter color split)
    // and the body's continuous scalars (displacement_magnitude /
    // psi_j_per_m3 / material_id; uniform 0 on the rigid indenter).
    let scan_for_overlay = build_scan_fixture();
    let scene_attr = design_scene(
        &body_for_viz,
        &[&scan_for_overlay as &dyn Sdf],
        &inspection_mesh,
        &design_slab_bounds,
        design_surface_resolution,
        &per_tet_scalars,
    )
    .map_err(|e| anyhow::anyhow!("{e}"))?;
    let scene_path = out_dir.join("device_design_scene.ply");
    save_ply_attributed(&scene_attr, &scene_path, true)?;

    // 11. Museum-plaque summary.
    print_summary(
        n_tets,
        n_vertices,
        referenced.len(),
        n_pinned,
        n_inner,
        n_middle,
        n_outer,
        n_pairs,
        force_total_z,
        mean_force_magnitude,
        max_force_magnitude,
        mean_disp_magnitude,
        max_disp_magnitude,
        step.iter_count,
        step.final_residual_norm,
    );

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn print_summary(
    n_tets: usize,
    n_vertices: usize,
    n_referenced: usize,
    n_pinned: usize,
    n_inner: usize,
    n_middle: usize,
    n_outer: usize,
    n_pairs: usize,
    force_total_z: f64,
    mean_force_magnitude: f64,
    max_force_magnitude: f64,
    mean_disp_magnitude: f64,
    max_disp_magnitude: f64,
    iter_count: usize,
    final_residual: f64,
) {
    println!("Body geometry:");
    println!("  outer-shell radius           : R_OUTER         = {R_OUTER} m");
    println!("  middle/outer interface       : R_MIDDLE_OUTER  = {R_MIDDLE_OUTER} m");
    println!("  inner/middle interface       : R_INNER_OUTER   = {R_INNER_OUTER} m");
    println!("  scan-fixture half-extent     : R_SCAN          = {R_SCAN} m");
    println!("  scan-fixture x-axis offset   : SCAN_OFFSET_X   = {SCAN_OFFSET_X} m");
    println!("  BCC cell size                : CELL_SIZE       = {CELL_SIZE} m");
    println!();
    println!("Mesh (post BCC + Isosurface Stuffing):");
    println!("  n_tets        : {n_tets}");
    println!("  n_vertices    : {n_vertices}");
    println!("  n_referenced  : {n_referenced}");
    println!("  n_pinned (R_OUTER band) : {n_pinned}");
    println!("  per-shell tet counts:");
    println!("    inner  (ECOFLEX_00_30)  : {n_inner}");
    println!("    middle (DRAGON_SKIN_10A): {n_middle}");
    println!("    outer  (ECOFLEX_00_30)  : {n_outer}");
    println!();
    println!("Contact (PenaltyRigidContact, scan-derived rigid indenter):");
    println!("  n_active_pairs        : {n_pairs}");
    println!("  force_total_z (N)     : {force_total_z:e}");
    println!("  mean force magnitude  : {mean_force_magnitude:e}");
    println!("  max force magnitude   : {max_force_magnitude:e}");
    println!();
    println!("Cavity-wall displacement (active-contact-pair vertices):");
    println!("  mean magnitude (m)    : {mean_disp_magnitude:e}");
    println!("  max magnitude  (m)    : {max_disp_magnitude:e}");
    println!();
    println!("Solver:");
    println!("  iter_count            : {iter_count}");
    println!("  final residual norm   : {final_residual:e}");
    println!();
    println!("Outputs:");
    println!("  out/layered_silicone_device.json  (scalars + materials + per-pair)");
    println!(
        "  out/device_boundary.ply           (full 3D body via sim_soft::viz::boundary_surface)"
    );
    println!(
        "  out/device_slab_cut_z0.ply        (cross-section at z = 0 via sim_soft::viz::slab_cut)"
    );
    println!(
        "  out/device_design_slab_cut_z0.ply (cross-section at z = 0 via sim_soft::viz::design_slab_cut, marching-squares-filled on design SDF + barycentric scalar interp)"
    );
    println!(
        "  out/device_design_surface.ply     (full 3D body via sim_soft::viz::design_surface, marching-cubes on design SDF + barycentric scalar interp)"
    );
    println!(
        "  out/device_design_surface_deformed.ply (deformed body via sim_soft::viz::design_surface_deformed at amplify=10; deformed-shape rendering with stress contours)"
    );
    println!(
        "  out/device_design_scene.ply       (body + scan indenter merged via sim_soft::viz::design_scene; primitive_id scalar distinguishes body=0 vs indenter=1)"
    );
    println!();
    println!("View with cf-view (workspace's unified visual-review viewer):");
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/layered-silicone-device/out/device_boundary.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/layered-silicone-device/out/device_slab_cut_z0.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/layered-silicone-device/out/device_design_slab_cut_z0.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/layered-silicone-device/out/device_design_surface.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/layered-silicone-device/out/device_design_surface_deformed.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/layered-silicone-device/out/device_design_scene.ply"
    );
}

// =============================================================================
// Compile-time assertions on geometric invariants
// =============================================================================

const _: () = {
    assert!(R_INNER_OUTER < R_MIDDLE_OUTER);
    assert!(R_MIDDLE_OUTER < R_OUTER);
    assert!(R_SCAN > 0.0);
    assert!(SCAN_OFFSET_X.abs() + R_SCAN < R_INNER_OUTER);
    assert!(BBOX_HALF_EXTENT > R_OUTER);
    assert!(CELL_SIZE > 0.0);
    assert!(STATIC_DT > 0.0);
};
