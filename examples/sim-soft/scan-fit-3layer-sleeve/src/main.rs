// `usize as f64` casts on referenced-vertex / per-shell tet counts for
// mean computations. Counts ≤ ~150 k here, well within f64 mantissa
// exact range. Same allowance as rows 6+8+9+10+11+16+18+20.
#![allow(clippy::cast_precision_loss)]
// `cast_possible_wrap` on `usize → VertexId` (u32) packing for vertex
// indices in the BCC + stuffing pipeline. Vertex counts are bounded by
// the BCC-lattice's i32-safe `n_lattice` cap inherited from
// `BccLattice::new`. Same allowance as rows 8+9+10+11+15+16+20.
#![allow(clippy::cast_possible_wrap)]
// `cast_possible_truncation` on `pid as u32` packing for primitive
// indices in the per-pair-readout walk (one rigid primitive in this
// row). Same allowance as rows 18 + 20.
#![allow(clippy::cast_possible_truncation)]
// `expect()` on `Matrix3::try_inverse()` for the per-tet `D_rest`
// reference shape derivative. `D_rest` is invertible iff the tet has
// positive signed volume — `verify_quality_floors` is the
// corresponding pre-condition gate. The `expect` is therefore a
// diagnostic guard on a `None` impossibility, not a real failure
// path. Same precedent as row 20's `expect(...)` allowance.
#![allow(clippy::expect_used)]
// `print_summary` is a single museum-plaque stdout writer; splitting
// fragments the visual format without information gain. Same allowance
// as rows 4+5+6+9+10+11+15+16+19+20.
#![allow(clippy::too_many_lines)]
// Domain-meaningful naming pairs (`outer_envelope` / `sleeve_body`,
// `pos_pinned` / `pos_active`, `mu_field` / `lambda_field`) distinguish
// operand-vs-receiver across the heterogeneous-CSG and multi-field
// composition sites. Same allowance as rows 6+10+11+16+20.
#![allow(clippy::similar_names)]

//! scan-fit-3layer-sleeve — the row 21 Tier 6 synthesis row, the second
//! end-to-end relative-comparison sim of the [layered silicone device][mem]
//! cavity-fit workflow, after row 20. Where row 20 carved a 12-tri-cube
//! cavity from a parametric outer sphere, this row models the
//! manufacturing target's geometry: a thin-walled 3-layer sleeve
//! wrapping a cuboid scan stand-in, exercised by a spherical rigid
//! intrusion probe under a static overlap pose.
//!
//! [mem]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_layered_silicone_device.md
//!
//! # Pipeline
//!
//! 1. **Scan stand-in via `Solid::cuboid`** — `(SCAN_HX, SCAN_HY,
//!    SCAN_HZ) = (0.020, 0.015, 0.040) m` half-extents. An axis-aligned
//!    rectangular box models a scanned reference geometry the sleeve is
//!    custom-fit around — the cuboid is an *exact* SDF (per cf-design's
//!    `Solid::cuboid` doc-comment), so `offset(WRAP_THICKNESS)` produces
//!    a uniformly 14 mm-thick wrap everywhere on the boundary (the
//!    Minkowski sum with a 14 mm-radius sphere — rounded at the
//!    cuboid's edges and corners, flat-faced everywhere else). The
//!    box stand-in is openly synthetic — the framing emphasises the
//!    workflow, not the geometry. Production runs swap this analytic
//!    stand-in for a `mesh_sdf::SignedDistanceField` lifted via PR3 F2
//!    (`impl Sdf for SignedDistanceField`); row 15
//!    `mesh-scan-as-solid` is the canonical STL-import precedent.
//!    Non-exact SDFs (`Solid::superellipsoid`, `Solid::ellipsoid`) are
//!    rejected here on the same Q1 grounds — `offset` on a non-exact
//!    SDF shifts the level set in non-distance units, producing a
//!    paper-thin or empty wrap shell at meshing time.
//!
//! 2. **Outer envelope via `Solid::offset`** — the scan offset outward
//!    by `WRAP_THICKNESS = 0.014 m` (14 mm). The sleeve's outer
//!    boundary is the offset solid's zero isosurface; the offset
//!    operation is exact in the typed-Solid kernel (no Minkowski-sum
//!    approximation).
//!
//! 3. **Sleeve body via `Solid::subtract`** — `outer_envelope.subtract(
//!    scan)` carves the scan-shaped cavity from the wrapped envelope,
//!    yielding the 3-layer-thick sleeve. Same `subtract` operator as
//!    row 16 `solid-to-sim-soft`'s parametric hollow body, here applied
//!    to two typed-Solid operands (homogeneous typed CSG; F5
//!    heterogeneous bridge is NOT exercised — row 20 already covers F5
//!    explicitly).
//!
//! 4. **`SdfMeshedTetMesh` build** via PR3 F1+F3:
//!    `SdfMeshedTetMesh::from_sdf(&sleeve_body, &hints)` accepts `&dyn
//!    cf_design::Sdf` (since `Solid: Sdf` per F1, re-exported as
//!    `sim_soft::Sdf` per F3) — one trait-object coercion bridges the
//!    typed-Solid surface into the BCC + Labelle-Shewchuk Isosurface
//!    Stuffing tet pipeline. `MeshingHints::material_field` carries
//!    the 3-layer `LayeredScalarField` partition by **distance from
//!    the scan**, so the layers wrap the scan at constant offsets
//!    matching the manufacturing build sequence (inner cast first,
//!    middle wrapped over inner, outer wrapped over middle).
//!
//! 5. **3-layer `MaterialField`** via PR3 F4 `silicone_table.rs`:
//!    inner = `ECOFLEX_00_20` (μ = 18 kPa) as proxy for Slacker-softened
//!    Ecoflex 00-30 at effective Shore 00-20; middle = `DRAGON_SKIN_10A`
//!    (μ = 51 kPa) as proxy for the conductive composite at effective
//!    Shore 15-18A (matches row 20's CB+mesh proxy precedent verbatim;
//!    Cu mesh + carbon black uplift is deferred to a Fork-B post-cast
//!    modulus calibration that absorbs uplift into the effective μ at
//!    calibration time); outer = `DRAGON_SKIN_20A` (μ = 113 kPa)
//!    direct match. Each `SiliconeMaterial::to_neo_hookean()` call is
//!    a `const fn` over F4's table entries, so the per-shell `(μ, λ)`
//!    provenance survives bit-equally from F4 down to the per-tet
//!    `Material::sample` returned by `MaterialField`. The layer
//!    partition is by `phi = scan.eval(p)` distance-from-scan: `[0,
//!    LAYER_INNER) → inner`, `[LAYER_INNER, LAYER_MIDDLE_OUTER) →
//!    middle`, `[LAYER_MIDDLE_OUTER, WRAP_THICKNESS] → outer`.
//!
//! 6. **Static fit pose under rigid intrusion** — outer-envelope-
//!    Dirichlet pin (`|outer_envelope.eval(p)| < CELL_SIZE / 2` band)
//!    plus `PenaltyRigidContact::new(vec![probe_solid])` where
//!    `probe_solid = Solid::sphere(PROBE_RADIUS).translate(...)` is
//!    positioned to penetrate `PROBE_PENETRATION_DEPTH = 0.001 m`
//!    (1 mm) past the scan's `+z` extent into the sleeve cavity. The
//!    static overlap pose drives the inner-layer cavity wall to
//!    deform around the probe; the cavity-side cavity-wall vertices
//!    in the penetration zone start INSIDE the contact band at rest,
//!    and the static fit pose finds the equilibrium where the
//!    penalty-elastic balance settles. v1 keeps the overlap gentle
//!    (1 mm) so the iter-0 penalty gradient stays inside Newton's
//!    basin of convergence on the stiffer 18-113 kPa silicone stack;
//!    deeper penetration (the user-target 8 mm physical intrusion)
//!    flows through v2's quasi-static multi-step ramp. Headline
//!    "fit-tightness" readout is the peak normal force at the
//!    penetration band per pattern-row-20 idiom.
//!
//! 7. **Per-tet strain energy density** — first sim-soft user-facing
//!    row to demonstrate post-solve per-tet `Ψ` extraction. The
//!    `deformation_gradient` helper reconstructs `F = D_curr · D_rest⁻¹`
//!    inline from `mesh.tet_vertices(t)`, `mesh.positions()` (rest),
//!    and `step.x_final` (deformed); per-tet `Ψ_t = mesh.materials()[t]
//!    .energy(&F)` aggregates per-layer to `Ψ̄_inner / Ψ̄_middle /
//!    Ψ̄_outer`. The aggregation drives anchor 5 (compliance ordering)
//!    and anchor 6 (outer-layer durability bound).
//!
//! 8. **Readouts** —
//!    - JSON `out/scan_fit_3layer_sleeve.json`: scalars (geometry
//!      constants + counts + force-total + `n_active_pairs` +
//!      mean/max force/displacement + per-layer `Ψ̄`) + per-shell tet
//!      counts + 3-material provenance block + per-active-pair detail
//!      array (mirrors row 18 + 20's two-section schema).
//!    - PLY `out/sleeve_zslab.ply`: z-slab per-tet centroid cloud
//!      (`|centroid.z| < CELL_SIZE / 2`) with two scalars —
//!      `material_id` categorical (0 = inner / 1 = middle / 2 = outer;
//!      cf-view auto-picks categorical palette per pattern (u)) and
//!      `displacement_magnitude` continuous unipolar (sequential
//!      viridis per pattern (u)).
//!    - `verify_*` runtime gates (9 anchor groups, see "Numerical
//!      anchors" in `README.md`).
//!
//! # Why z-slab over full-boundary-surface
//!
//! Per pattern (aa) banked at row 16 N+3 ([memo][mem16]): hollow /
//! interior-cavity / partial-occlusion bodies → z-slab per-tet centroid
//! cloud (rows 11 + 16 + 20 precedent). The 3-layer sleeve has BOTH
//! the scan-shaped cavity AND three concentric material shells — a
//! full-boundary-surface PLY would 360°-occlude the cavity and the
//! inner/middle interfaces from every cf-view orbit angle. The z-slab
//! projects centroids onto a 2-D annulus cut on `z = 0`, exposing both
//! the radial material-shell partition (via `material_id` categorical)
//! and the cavity-wall displacement response under probe intrusion
//! (via `displacement_magnitude` sequential).
//!
//! [mem16]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_row_16_patterns.md
//!
//! # Sanitization
//!
//! Per the [device memo][mem]'s sanitization directive: the scanned
//! reference geometry is referred to as "scanned reference geometry"
//! or "scan stand-in" throughout this crate's prose. No anatomical
//! references appear in any tracked surface. The cuboid placeholder
//! is a parametric synthetic stand-in — the pipeline demonstration is
//! the workflow ("scan-shaped body → wrap by offset → carve cavity
//! → 3-material FEM → rigid intrusion contact"), not the cuboid's
//! specific geometry; production runs swap the cuboid for a real
//! scan via row 15's STL-import path without any other code change.
//!
//! # Run
//!
//! ```sh
//! cargo run -p example-sim-soft-scan-fit-3layer-sleeve --release
//! ```
//!
//! Per `feedback_release_mode_heavy_tests` — release mode is required
//! for the FEM solve at this mesh resolution (~75 k tets through
//! faer's sparse Cholesky + 9 Newton iters); debug mode would take
//! many minutes for what runs in seconds release. The `CELL_SIZE =
//! 0.004 m` (4 mm) is sized so each of the 6/4/4 mm layers carries at
//! least one BCC cell across thickness — coarsening further would
//! erase the middle and outer layers, and finer cells (e.g.
//! `0.002 m`) push the per-cell penalty gradient too high under the
//! static-overlap pose, inverting tets in the first Newton step.

use std::collections::BTreeSet;
use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use cf_design::Solid;
use mesh_io::save_ply_attributed;
use mesh_types::{AttributedMesh, IndexedMesh, Point3, Vector3};
use nalgebra::Matrix3;
use serde_json::{Value, json};
use sim_ml_chassis::Tensor;
use sim_soft::material::silicone_table::{DRAGON_SKIN_10A, DRAGON_SKIN_20A, ECOFLEX_00_20};
use sim_soft::{
    Aabb3, BoundaryConditions, CpuNewtonSolver, Field, LayeredScalarField, Material, MaterialField,
    Mesh, MeshingHints, NewtonStep, PenaltyRigidContact, PenaltyRigidContactSolver, SceneInitial,
    Sdf, SdfMeshedTetMesh, Solver, SolverConfig, Tet4, Vec3, VertexId, pick_vertices_by_predicate,
    referenced_vertices,
};

// =============================================================================
// Constants — scan stand-in geometry (axis-aligned cuboid)
// =============================================================================

/// Cuboid half-extents (m). The 40 × 30 × 80 mm rectangular box
/// (full-extent) models a scanned reference geometry the sleeve is
/// custom-fit around; the long axis is `+z` (penetrated by the rigid
/// probe). `Solid::cuboid` is an *exact* SDF — `offset(d)` then
/// produces a true uniform `d`-thick Minkowski-sum shell, which is
/// the load-bearing requirement for the sleeve's wrap geometry to
/// have non-trivial physical thickness at meshing time. Production
/// runs replace this analytic stand-in with a
/// `mesh_sdf::SignedDistanceField` lifted via PR3 F2.
const SCAN_HX: f64 = 0.020;
const SCAN_HY: f64 = 0.015;
const SCAN_HZ: f64 = 0.040;

// =============================================================================
// Constants — sleeve wrap geometry
// =============================================================================

/// Total wrap thickness (m) — the radial offset from the scan's zero
/// isosurface to the outer envelope's zero isosurface. `0.014 m` is a
/// 14 mm 3-layer build budget. The v1 split is `6 + 4 + 4` mm
/// (inner + middle + outer) so each layer carries at least one BCC
/// cell at the v1's `CELL_SIZE = 0.004 m` (the user-target hardware
/// build is `6 + 5 + 3` mm; v1's 4 mm middle + 4 mm outer is a
/// numerical proxy for 5 + 3, both layers still meaningfully
/// resolved). v2's quasi-static ramp + finer cells will recover the
/// 6/5/3 split.
const WRAP_THICKNESS: f64 = 0.014;

/// Inner-layer outer threshold (m) — `phi = 0` (scan boundary) up to
/// `phi = LAYER_INNER` is the inner-layer band. The 6 mm inner layer
/// carries the skin-contact softness story (Ecoflex 00-30 + 75%
/// Slacker, effective Shore 00-20 — proxied here by `ECOFLEX_00_20`
/// from F4's silicone table).
const LAYER_INNER: f64 = 0.006;

/// Middle-layer outer threshold (m) — `phi = LAYER_INNER` up to
/// `phi = LAYER_MIDDLE_OUTER` is the middle-layer band. The 4 mm
/// middle layer carries the conductive-composite mechanical role
/// (DS10A + Cu mesh + carbon black, effective Shore 15-18A —
/// proxied here by `DRAGON_SKIN_10A` per row 20's precedent).
const LAYER_MIDDLE_OUTER: f64 = 0.010;

/// Outer-layer outer threshold (m) — `phi = LAYER_MIDDLE_OUTER` up to
/// `phi = WRAP_THICKNESS` is the outer-layer band. The 4 mm outer
/// layer carries the structural-stiffness role (DS20A direct, no
/// modifier).
const LAYER_OUTER: f64 = WRAP_THICKNESS;

// =============================================================================
// Constants — meshing bbox + cell size
// =============================================================================

/// Bbox half-extent along `x` (m). Outer envelope max-extent along x
/// is `SCAN_HX + WRAP_THICKNESS = 0.034 m`; one-cell-edge slack keeps
/// every BCC lattice corner that touches the sleeve inside the bbox.
const BBOX_HALF_X: f64 = SCAN_HX + WRAP_THICKNESS + CELL_SIZE;

/// Bbox half-extent along `y` (m). Outer envelope max-extent along y
/// is `SCAN_HY + WRAP_THICKNESS = 0.029 m`; one-cell-edge slack.
const BBOX_HALF_Y: f64 = SCAN_HY + WRAP_THICKNESS + CELL_SIZE;

/// Bbox half-extent along `z` (m). Outer envelope max-extent along z
/// is `SCAN_HZ + WRAP_THICKNESS = 0.054 m`; one-cell-edge slack.
const BBOX_HALF_Z: f64 = SCAN_HZ + WRAP_THICKNESS + CELL_SIZE;

/// BCC lattice spacing (m). `0.004 m` (4 mm) is sized so each of the
/// three 6/4/4 mm layers carries at least one BCC cell across
/// thickness. The resulting tet count is ~75 k — ~10× row 20's
/// ~7 k baseline (the body is z-elongated and the wrap shell occupies
/// most of the bbox), runs release-mode through faer sparse Cholesky
/// in seconds. v2's quasi-static ramp will tolerate finer cells +
/// the user-target 6/5/3 mm layer split.
const CELL_SIZE: f64 = 0.004;

// =============================================================================
// Constants — rigid intrusion probe
// =============================================================================

/// Spherical probe radius (m). 12 mm probe centred at
/// `(0, 0, PROBE_CENTER_Z)` reaches z ∈ `[PROBE_CENTER_Z - 12 mm,
/// PROBE_CENTER_Z + 12 mm]`; at v1's 1 mm penetration the probe-wrap
/// overlap is a thin annulus near the scan's `+z` cap, ~294 active
/// contact pairs.
const PROBE_RADIUS: f64 = 0.012;

/// Probe penetration depth (m) past the scan's `+z` extent. The
/// static overlap pose creates a contact-band gradient at iter 0; if
/// the gradient is too aggressive (deep penetration + stiff outer
/// layer), the first Newton step can invert tets and the condensed
/// tangent matrix loses positive-definiteness. 1 mm overlap is
/// gentle enough for the 18-113 kPa silicone stack to converge in
/// fewer than `MAX_NEWTON_ITER` iters; the headline "fit-tightness"
/// readout (peak normal force at the penetration band) is still
/// quantitatively meaningful at this overlap. v2's quasi-static
/// ramp will recover deeper penetration through a multi-step solve.
const PROBE_PENETRATION_DEPTH: f64 = 0.001;

/// Probe centre `+z` coordinate (m). At rest the probe centre sits at
/// `SCAN_HZ - PROBE_RADIUS + PROBE_PENETRATION_DEPTH = 0.040 - 0.012 +
/// 0.001 = 0.029 m`, so the probe surface penetrates the cavity by
/// `PROBE_PENETRATION_DEPTH` along the scan's long axis.
const PROBE_CENTER_Z: f64 = SCAN_HZ - PROBE_RADIUS + PROBE_PENETRATION_DEPTH;

// =============================================================================
// Constants — solver
// =============================================================================

/// Static-regime time step (s). Mirrors rows 11 + 14 + 16 + 18 + 20's
/// `STATIC_DT` verbatim: at large `dt`, the backward-Euler residual's
/// inertial term `M / dt² · (x − x_prev)` collapses ~4 orders below
/// the stiffness contribution, so a single `replay_step` from rest
/// converges to the static equilibrium far below `tol = 1e-10`.
const STATIC_DT: f64 = 1.0;

/// Newton iter cap. Mirrors rows 11 + 16 + 20's `MAX_NEWTON_ITER`
/// verbatim; empirical convergence on similar geometries is ≤ 7 iters.
const MAX_NEWTON_ITER: usize = 50;

// =============================================================================
// Constants — tolerances
// =============================================================================

/// IV-1 sparse-tier rel-tol for captured force / displacement / per-
/// layer Ψ̄ bits. ~75 k tets through faer's sparse Cholesky lives at
/// the IV-1 sparse-at-scale tier (3-ULP cross-platform drift);
/// `1e-12` admits sparse-solver SIMD/FMA noise while catching any
/// real regression. Same precedent as rows 6+10+11+16+20.
const SPARSE_REL_TOL: f64 = 1.0e-12;

/// Absolute floor for the captured-bits comparison; below the cavity-
/// wall displacement-magnitude scale (`< 1e-3 m`) by 9 orders. Same
/// precedent as rows 6+10+11+16+20.
const SPARSE_EPS_ABS: f64 = 1.0e-12;

/// Bit-exact tolerance for the F4 const-fn `to_neo_hookean()` Lamé-
/// pair round-trip — the F4 unit test at `silicone_table.rs::tests::
/// to_neo_hookean_round_trips_lame_pair` asserts the same identity at
/// `epsilon = 0.0`, and we re-assert it here so a regression in F4
/// trips this row directly. Same precedent as rows 19 + 20.
const F4_PROVENANCE_EXACT_TOL: f64 = 0.0;

/// Probe `F = diag(1.01, 1, 1)` material-assignment-probe tolerance.
/// `EXACT_TOL = 0.0` per row 8 pattern (a) — both sides run identical
/// NH arithmetic on identical `(μ, λ)` and identical `F`, bit-equal by
/// construction on a fixed toolchain.
const MATERIAL_PROBE_EXACT_TOL: f64 = 0.0;

// =============================================================================
// Constants — captured first-run anchor bits
// =============================================================================
//
// **Capture provenance** — captured 2026-05-08 at sim-soft `dev`
// (post-Q7 tip `13e46dad`, PR3 + post-PR3 cleanup + Q7 cf-viewer
// retrofit shipped), rustc 1.95.0 (`59807616e` 2026-04-14) on macOS
// arm64 — same toolchain + platform as IV-1's reference capture. Re-
// bake protocol per IV-1: if a rel-tol assertion fails, do NOT re-
// bake; rule out toolchain drift first; if same toolchain, real
// regression in cf-design's `cuboid` / `offset` plumbing OR in
// sim-soft's BCC + IS + faer hot path OR in the inline
// `deformation_gradient` arithmetic.
//
// First-run capture is bootstrapped via `CF_CAPTURE_BITS=1` (pattern
// (cc) banked at row 19): when the env var is set, the capture-print
// helper emits `CAPTURED_*_REF_BITS = 0x...` blocks ready to paste
// into the constants below; when unset (default), every captured-bits
// gate runs the strict `to_bits()` self-pin against these constants.
// Identity-row pattern: every `*_EXACT` count is filled at first run
// (initial value `0` triggers a deliberate-fail diagnostic), every
// `*_REF_BITS` is filled at first run after that.

/// Total tet count after the BCC + Isosurface Stuffing pipeline carves
/// the scan-shaped cavity from the offset-wrapped cuboid. Captured at
/// first run (`CF_CAPTURE_BITS=1`, post-Q7 tip `13e46dad`, 2026-05-08,
/// rustc 1.95.0, macOS arm64).
const N_TETS_EXACT: usize = 74_628;

/// Total mesh vertex count, including BCC lattice corners not
/// referenced by any tet.
const N_VERTICES_EXACT: usize = 31_966;

/// Vertices referenced by at least one tet. `N_VERTICES_EXACT -
/// N_REFERENCED_EXACT = 14_582` orphan BCC lattice corners excluded
/// from solver participation.
const N_REFERENCED_EXACT: usize = 17_384;

/// Outer-envelope-surface Dirichlet-pinned vertex count (every vertex
/// with `|outer_envelope.eval(p)| < CELL_SIZE / 2`, filtered to
/// referenced set).
const N_PINNED_EXACT: usize = 7_046;

/// Per-shell tet counts at first capture. `INNER + MIDDLE + OUTER ==
/// N_TETS_EXACT` by construction (every tet centroid sits in exactly
/// one of the three distance-from-scan bins).
const N_INNER_TETS_EXACT: usize = 25_892;
const N_MIDDLE_TETS_EXACT: usize = 16_656;
const N_OUTER_TETS_EXACT: usize = 32_080;

/// Per-shell tet counts in the `|centroid.z| < CELL_SIZE / 2 = 0.002`
/// z-slab cut for the cf-view PLY artifact.
const N_INNER_TETS_ZSLAB_EXACT: usize = 768;
const N_MIDDLE_TETS_ZSLAB_EXACT: usize = 432;
const N_OUTER_TETS_ZSLAB_EXACT: usize = 892;

/// Active contact-pair count at the static fit pose, filtered to
/// REFERENCED vertices (v2.5-equivalent cleanup, 2026-05-08; row 21
/// anchor cleanup mirroring row 22 v2.5). 13 real physical contacts
/// at the 1 mm static-overlap pose. Pre-cleanup the unfiltered count
/// was 294 (95-97 % orphan-driven — orphan BCC corners inside the
/// empty cavity, with no FEM stiffness, ignored by the solver).
/// Cross-row continuity to row 22 v2.5 step 2 (also at 1 mm depth):
/// bit-equal at 13 pairs.
const N_CONTACT_PAIRS_EXACT: usize = 13;

/// Total `+z`-component of the static-pose contact reaction force (N),
/// summed over REFERENCED vertices only (v2.5-equivalent cleanup).
/// `force_on_soft = +κ · (d̂ - sd) · normal`; for the row's static-
/// overlap pose the wrap-cap material above the scan's `+z` cap is
/// pushed UP (`+z` direction), so the physical sum is positive.
/// Bits represent ~+1.87 N. Pre-cleanup the unfiltered sum was
/// -130.5 N — that NEGATIVE sign was orphan-driven (orphans below
/// the probe equator have `-z` normals and dominated the sum);
/// post-cleanup the physically correct `+z` push surfaces.
const FORCE_TOTAL_Z_REF_BITS: u64 = 0x3ffd_e776_ddb0_0a18;

/// Cavity-wall mean displacement-magnitude bits (m) over the FILTERED
/// (referenced-only) active-contact-pair vertex set (v2.5-equivalent
/// cleanup). ~1.24 mm — the 13 real cavity-wall vertices in the
/// active contact band each move ~1-2 mm under the 1 mm probe
/// penetration (penalty-equilibrium amplification, see anchor 8's
/// `max_disp / depth ≈ 2-3×` at shallow penetration prose). Pre-
/// cleanup the mean was 55 µm — diluted by ~280 orphan vertices
/// with zero displacement (orphans don't move; the FEM solver skips
/// them); post-cleanup the mean reflects only physically displaced
/// cavity-wall material.
const CAVITY_WALL_MEAN_DISP_REF_BITS: u64 = 0x3f54_791c_dd65_0589;

/// Cavity-wall max displacement-magnitude bits (m) over the FILTERED
/// (referenced-only) active-contact-pair vertex set. ~1.97 mm —
/// UNCHANGED from the pre-cleanup value (max is over real movements;
/// orphans contribute zero by construction), confirming the orphan-
/// pollution effect on the mean was a dilution-by-zero artifact.
const CAVITY_WALL_MAX_DISP_REF_BITS: u64 = 0x3f60_2b04_a1ce_3f13;

/// Per-layer mean strain-energy-density bits (J/m³) — the headline
/// architectural payload of row 21. Inner is softest (μ = 18 kPa) AND
/// closest to probe → highest mean Ψ. Outer is stiffest (μ = 113 kPa)
/// AND outer-Dirichlet-pinned → lowest mean Ψ. The strict-monotone
/// ordering `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` is encoded as anchor 5;
/// the bit-pinned values catch FMA / faer drift in the per-tet F-from-
/// positions + `Material::energy` pipeline. Approximate values:
/// ~9.21 / ~1.53 / ~0.34 J/m³.
const MEAN_PSI_INNER_REF_BITS: u64 = 0x4022_6b7f_4bef_57af;
const MEAN_PSI_MIDDLE_REF_BITS: u64 = 0x3ff8_675a_701a_9886;
const MEAN_PSI_OUTER_REF_BITS: u64 = 0x3fd5_e5a3_dc72_1c2a;

/// Outer-layer max strain-energy-density bits (J/m³) — durability
/// proxy. Self-pinned at first capture; catches regressions where the
/// outer-layer peak Ψ drifts under cf-design / sim-soft hot-path
/// changes. ~175.8 J/m³ (~ 514× the outer-layer mean — peak is
/// localised in tets adjacent to the contact band where the probe
/// loads transmit through the radial chain inner → middle → outer).
const OUTER_PSI_MAX_REF_BITS: u64 = 0x4065_f999_5fa6_15bd;

// =============================================================================
// SDF builders
// =============================================================================

/// The scan stand-in — axis-aligned cuboid at origin with half-extents
/// `(SCAN_HX, SCAN_HY, SCAN_HZ)`. `cf_design::Solid::cuboid` is an
/// *exact* SDF (per its doc-comment), so the downstream
/// `Solid::offset(WRAP_THICKNESS)` produces a true uniform 14 mm-thick
/// shell — the load-bearing requirement for the sleeve's wrap to
/// carry non-trivial physical thickness through the BCC + Isosurface
/// Stuffing pipeline.
///
/// `Solid: Sdf` per PR3 F1 (re-exported as `sim_soft::Sdf` per F3).
fn build_scan_solid() -> Solid {
    Solid::cuboid(Vector3::new(SCAN_HX, SCAN_HY, SCAN_HZ))
}

/// The outer envelope — the scan offset outward by `WRAP_THICKNESS`.
/// `Solid::offset(d)` is exact in the typed-Solid kernel (no
/// Minkowski-sum approximation); the resulting leaf has zero
/// isosurface at `phi_scan = -WRAP_THICKNESS` (the scan's distance
/// field shifted by `+WRAP_THICKNESS`).
fn build_outer_envelope(scan: Solid) -> Solid {
    scan.offset(WRAP_THICKNESS)
}

/// The sleeve body — outer envelope MINUS the scan-shaped cavity.
/// Same `Solid::subtract` operator as parametric-only CSG; no
/// per-shape glue. F5 heterogeneous bridge is NOT exercised here
/// (both operands are typed `Solid`); row 20 `layered-silicone-device`
/// is the canonical F5 demonstration.
fn build_sleeve_body(outer: Solid, scan: Solid) -> Solid {
    outer.subtract(scan)
}

/// Probe `Solid` — sphere at `(0, 0, PROBE_CENTER_Z)`, penetrating
/// the scan cavity by `PROBE_PENETRATION_DEPTH` along `+z`. Static
/// overlap pose: cavity-wall vertices in the penetration zone start
/// INSIDE the contact band at rest; the static fit pose drives them
/// outward, settling at the penalty-elastic equilibrium.
fn build_probe_solid() -> Solid {
    Solid::sphere(PROBE_RADIUS).translate(Vector3::new(0.0, 0.0, PROBE_CENTER_Z))
}

// =============================================================================
// 3-layer MaterialField — partition by distance-from-scan
// =============================================================================

/// Three-layer `MaterialField` per the manufacturing build sequence:
/// inner = `ECOFLEX_00_20` (proxy for Slacker-softened Ecoflex 00-30
/// at effective Shore 00-20); middle = `DRAGON_SKIN_10A` (proxy for
/// the conductive composite at effective Shore 15-18A — same
/// precedent as row 20's CB+mesh proxy; mesh+CB uplift deferred to
/// Fork-B post-cast modulus calibration); outer = `DRAGON_SKIN_20A`
/// direct match. Partition is by `phi = scan.eval(p)` distance-from-
/// scan: `[0, LAYER_INNER) → inner`, `[LAYER_INNER,
/// LAYER_MIDDLE_OUTER) → middle`, `[LAYER_MIDDLE_OUTER,
/// WRAP_THICKNESS] → outer`. The thresholds are positive (outside
/// the scan) because every sleeve point has `phi > 0` by
/// construction (the sleeve body is `outer ⊖ scan`, so its support
/// is the wrap shell).
fn build_material_field() -> MaterialField {
    // Partition Sdf is the scan itself — every cloned `Box<dyn Sdf>`
    // wraps a fresh copy of the typed-Solid cuboid, evaluated at the
    // per-tet centroid in `LayeredScalarField::sample` to look up
    // `phi = scan.eval(p)` distance-from-scan.
    let scan_for_partition = || Box::new(build_scan_solid()) as Box<dyn Sdf>;

    let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![ECOFLEX_00_20.mu, DRAGON_SKIN_10A.mu, DRAGON_SKIN_20A.mu],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![
            ECOFLEX_00_20.lambda,
            DRAGON_SKIN_10A.lambda,
            DRAGON_SKIN_20A.lambda,
        ],
    ));
    MaterialField::from_fields(mu_field, lambda_field)
}

// =============================================================================
// Per-tet shell classification — by distance-from-scan at centroid
// =============================================================================

/// Shell index of a per-tet centroid: 0 = inner / 1 = middle / 2 =
/// outer. The boundary convention is **open-left, closed-right** —
/// at `phi == LAYER_INNER` the point belongs to the middle shell, at
/// `phi == LAYER_MIDDLE_OUTER` to the outer shell. This mirrors
/// `LayeredScalarField::sample`'s `partition_point(|&t| t <= phi)`
/// idiom verbatim (predicate `t <= phi` returns true at threshold
/// equality, so `partition_point` steps PAST it; equivalent direct
/// chain is strict-`<` test). Rows 8 + 20 use `<=` because their
/// sphere geometry + BCC tet centroids don't generate exact-threshold
/// `phi` values at any meaningful fraction of tets; row 21's axis-
/// aligned cuboid SDF + axis-aligned BCC grid + face-aligned tet
/// centroids inside the wrap region DO generate exact phi-at-threshold
/// matches at a regular fraction of tets (e.g. centroids on the +x
/// face annulus see `phi = cx - SCAN_HX` with `cx` at half-integer
/// multiples of `CELL_SIZE / 2`, hitting `LAYER_INNER = 0.006` and
/// `LAYER_MIDDLE_OUTER = 0.010` exactly), so the strict-`<` convention
/// is required for the per-tet classifier to agree with the
/// `MaterialField`'s sample bit-equally.
fn shell_at_phi(phi: f64) -> usize {
    if phi < LAYER_INNER {
        0 // inner
    } else if phi < LAYER_MIDDLE_OUTER {
        1 // middle
    } else {
        2 // outer
    }
}

// =============================================================================
// Mesh build via PR3 F1+F3
// =============================================================================

fn build_hints(material_field: MaterialField) -> MeshingHints {
    MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(-BBOX_HALF_X, -BBOX_HALF_Y, -BBOX_HALF_Z),
            Vec3::new(BBOX_HALF_X, BBOX_HALF_Y, BBOX_HALF_Z),
        ),
        cell_size: CELL_SIZE,
        material_field: Some(material_field),
    }
}

// =============================================================================
// BoundaryConditions — outer-envelope-surface Dirichlet pin only
// =============================================================================

/// Build the BC: outer-envelope-surface Dirichlet pin (`|outer_envelope.
/// eval(p)| < CELL_SIZE / 2` band) + empty `loaded_vertices` (no
/// distributed pressure — the contact penalty handles all loading at
/// the inner cavity wall via the rigid intrusion probe). Mirrors row
/// 20's contact-only loading idiom, with the band predicate re-shaped
/// from a sphere-norm to the outer-envelope SDF.
fn build_boundary_conditions(
    mesh: &SdfMeshedTetMesh,
    referenced: &[VertexId],
    outer_envelope: &Solid,
) -> BoundaryConditions {
    let referenced_set: BTreeSet<VertexId> = referenced.iter().copied().collect();
    let band_tol = 0.5 * CELL_SIZE;
    let pinned: Vec<VertexId> = pick_vertices_by_predicate(mesh, |p| {
        outer_envelope.eval(nalgebra::Point3::from(*p)).abs() < band_tol
    })
    .into_iter()
    .filter(|v| referenced_set.contains(v))
    .collect();
    assert!(
        !pinned.is_empty(),
        "outer-envelope band turned up empty at cell_size = {CELL_SIZE}",
    );

    BoundaryConditions {
        pinned_vertices: pinned,
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

    let empty_theta: [f64; 0] = [];
    let theta_tensor = Tensor::from_slice(&empty_theta, &[0]);
    let step = solver.replay_step(&initial.x_prev, &initial.v_prev, &theta_tensor, cfg.dt);

    SolveResult {
        rest_positions,
        step,
    }
}

// =============================================================================
// Per-tet deformation gradient (NEW capability — first sim-soft user-
// facing row to demonstrate post-solve per-tet F extraction)
// =============================================================================

/// Reconstruct the per-tet deformation gradient `F = D_curr · D_rest⁻¹`
/// from rest + deformed positions. `D_∗` is the 3×3 column-matrix
/// `[v1 - v0, v2 - v0, v3 - v0]` of edge vectors emanating from the
/// tet's first vertex, evaluated at rest (subscript `rest`) and at
/// the post-solve deformed state (subscript `curr`). The reference
/// shape derivative `D_rest⁻¹` is the standard FEM mapping from
/// reference-frame edge coordinates to barycentric basis functions;
/// `F = D_curr · D_rest⁻¹` then maps reference vectors to spatial
/// vectors at the tet's interior.
///
/// `D_rest` is invertible iff the tet has positive signed volume —
/// `verify_quality_floors` is the corresponding gate. A regression
/// where the BCC + IS pipeline emits a degenerate tet would surface
/// here as a `try_inverse() -> None`; per the `expect` message, that
/// is structurally impossible after `verify_quality_floors` passes.
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
// Verifications — 9 anchor groups
// =============================================================================

/// `true` when the `CF_CAPTURE_BITS` env var is set (any value). Per
/// pattern (cc) banked at row 19: env-var-gated bypass of every
/// captured-anchor check + a single print pass that emits every
/// `*_EXACT` count and every `*_REF_BITS` line ready to paste into
/// the constants block. IV-1 protocol forbids re-baking to silence a
/// drift assertion — `CF_CAPTURE_BITS` is for first-time capture
/// (initial-author-bake) and intentional re-bake (e.g., F4 const
/// value updated to a new data-sheet revision), not for failure
/// silencing.
fn capturing_bits() -> bool {
    std::env::var("CF_CAPTURE_BITS").is_ok()
}

fn verify_counts_exact(
    mesh: &SdfMeshedTetMesh,
    referenced: &[VertexId],
    pinned: &[VertexId],
    inner_count: usize,
    middle_count: usize,
    outer_count: usize,
) {
    if capturing_bits() {
        eprintln!("=== CAPTURED COUNTS (paste into source) ===");
        eprintln!("const N_TETS_EXACT: usize = {};", mesh.n_tets());
        eprintln!("const N_VERTICES_EXACT: usize = {};", mesh.n_vertices());
        eprintln!("const N_REFERENCED_EXACT: usize = {};", referenced.len());
        eprintln!("const N_PINNED_EXACT: usize = {};", pinned.len());
        eprintln!("const N_INNER_TETS_EXACT: usize = {inner_count};");
        eprintln!("const N_MIDDLE_TETS_EXACT: usize = {middle_count};");
        eprintln!("const N_OUTER_TETS_EXACT: usize = {outer_count};");
        return;
    }
    assert_eq!(mesh.n_tets(), N_TETS_EXACT, "n_tets");
    assert_eq!(mesh.n_vertices(), N_VERTICES_EXACT, "n_vertices");
    assert_eq!(referenced.len(), N_REFERENCED_EXACT, "n_referenced");
    assert_eq!(pinned.len(), N_PINNED_EXACT, "n_pinned");
    assert_eq!(inner_count, N_INNER_TETS_EXACT, "n_inner_tets");
    assert_eq!(middle_count, N_MIDDLE_TETS_EXACT, "n_middle_tets");
    assert_eq!(outer_count, N_OUTER_TETS_EXACT, "n_outer_tets");
    assert_eq!(
        inner_count + middle_count + outer_count,
        N_TETS_EXACT,
        "shell-count partition sums to N_TETS_EXACT",
    );
}

fn verify_zslab_counts_exact(inner_zslab: usize, middle_zslab: usize, outer_zslab: usize) {
    if capturing_bits() {
        eprintln!("const N_INNER_TETS_ZSLAB_EXACT: usize = {inner_zslab};");
        eprintln!("const N_MIDDLE_TETS_ZSLAB_EXACT: usize = {middle_zslab};");
        eprintln!("const N_OUTER_TETS_ZSLAB_EXACT: usize = {outer_zslab};");
        return;
    }
    assert_eq!(inner_zslab, N_INNER_TETS_ZSLAB_EXACT, "n_inner_tets_zslab");
    assert_eq!(
        middle_zslab, N_MIDDLE_TETS_ZSLAB_EXACT,
        "n_middle_tets_zslab",
    );
    assert_eq!(outer_zslab, N_OUTER_TETS_ZSLAB_EXACT, "n_outer_tets_zslab");
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

fn verify_material_provenance() {
    // F4 const-fn `to_neo_hookean()` returns a `NeoHookean` whose
    // energy at `F = I` is bit-equally zero AND whose energy at a
    // small uniaxial stretch `F = diag(1.01, 1, 1)` matches the
    // closed-form `(λ/2)(ln J)² + ((μ/2)(I₁ − 3) − μ ln J)`. Mirrors
    // F4's own unit test verbatim — a regression here surfaces as
    // drift between F4's table and `NeoHookean::from_lame`'s FMA
    // chain. Same precedent as rows 19 + 20.
    for mat in [&ECOFLEX_00_20, &DRAGON_SKIN_10A, &DRAGON_SKIN_20A] {
        let nh = mat.to_neo_hookean();
        let id = Matrix3::<f64>::identity();
        assert_relative_eq!(nh.energy(&id), 0.0, epsilon = F4_PROVENANCE_EXACT_TOL);

        let mut f = Matrix3::<f64>::identity();
        f[(0, 0)] = 1.01;
        let i1 = 1.01_f64.mul_add(1.01, 2.0);
        let j_ln = 1.01_f64.ln();
        let half_mu = 0.5 * mat.mu;
        let half_lambda = 0.5 * mat.lambda;
        let expected = half_lambda.mul_add(j_ln * j_ln, half_mu.mul_add(i1 - 3.0, -mat.mu * j_ln));
        assert_relative_eq!(nh.energy(&f), expected, epsilon = F4_PROVENANCE_EXACT_TOL,);
    }
}

/// Per-tet material assignment matches the `LayeredScalarField`
/// partition by distance-from-scan at the centroid. Probe via the
/// `Material::energy` trait at `F = diag(1.01, 1, 1)` (row 8 pattern
/// (a)): both sides run identical NH arithmetic on identical
/// `(μ, λ)`, bit-equal by construction on a fixed toolchain.
///
/// Headline gate of the row's multi-material correctness — verifies
/// that every per-tet `NeoHookean` cached by `MaterialField::sample`
/// at mesh-build time matches the `(μ, λ)` pair the centroid's
/// shell would assign by direct lookup at `silicone_table.rs`.
fn verify_material_assignment_partition(mesh: &SdfMeshedTetMesh, shell_idx_per_tet: &[usize]) {
    let materials = mesh.materials();
    assert_eq!(
        materials.len(),
        shell_idx_per_tet.len(),
        "materials() length does not match per-tet shell-classification length",
    );
    let mut f = Matrix3::<f64>::identity();
    f[(0, 0)] = 1.01;

    let expected_nh = [
        ECOFLEX_00_20.to_neo_hookean(),
        DRAGON_SKIN_10A.to_neo_hookean(),
        DRAGON_SKIN_20A.to_neo_hookean(),
    ];
    for (t, &shell_idx) in shell_idx_per_tet.iter().enumerate() {
        let observed = materials[t].energy(&f);
        let expected = expected_nh[shell_idx].energy(&f);
        assert!(
            (observed - expected).abs() <= MATERIAL_PROBE_EXACT_TOL,
            "tet {t} shell {shell_idx}: observed energy {observed} != expected {expected} \
             (boundary-convention drift between shell_at_phi and LayeredScalarField::sample?)",
        );
    }
}

/// Per-layer mean strain-energy density `Ψ̄_inner / Ψ̄_middle /
/// Ψ̄_outer` — the row's headline mechanical readout. Strict-monotone
/// ordering `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` reflects:
///
/// 1. **Compliance**: inner is softest (μ = 18 kPa), outer is stiffest
///    (μ = 113 kPa). Under the same probe-driven displacement field
///    (continuous through the bonded multi-material body), strain
///    concentrates in the softer layers.
/// 2. **Distance to load**: inner is at the cavity wall directly
///    contacting the probe; the strain field decays radially outward.
/// 3. **Distance to constraint**: outer is pinned at the Dirichlet
///    band on the outer envelope; the displacement field is forced
///    to zero at the boundary, so outer-layer F stays close to I and
///    Ψ stays small.
///
/// All three effects compound in the same direction — the ordering is
/// robust to small drift in the per-tet F arithmetic.
fn verify_strain_energy_ordering(mean_psi_inner: f64, mean_psi_middle: f64, mean_psi_outer: f64) {
    assert!(
        mean_psi_inner > mean_psi_middle,
        "mean_psi_inner ({mean_psi_inner:e}) ≯ mean_psi_middle ({mean_psi_middle:e}) — \
         softest+closest-to-load layer should carry the highest mean strain energy density",
    );
    assert!(
        mean_psi_middle > mean_psi_outer,
        "mean_psi_middle ({mean_psi_middle:e}) ≯ mean_psi_outer ({mean_psi_outer:e}) — \
         middle layer should carry more strain than outer (stiffer + Dirichlet-pinned) layer",
    );
}

/// Outer-layer max strain-energy-density bound (durability proxy).
/// The captured `OUTER_PSI_MAX_REF_BITS` is the peak over outer-shell
/// tets at first capture; this anchor catches a regression where the
/// outer-layer's peak Ψ exceeds that value under a hot-path drift in
/// cf-design / sim-soft. Compared via `assert_relative_eq!` at
/// `SPARSE_REL_TOL` + `SPARSE_EPS_ABS` — same self-pin protocol as
/// the per-layer means.
fn verify_outer_layer_max_psi(max_psi_outer: f64) {
    if capturing_bits() {
        eprintln!(
            "const OUTER_PSI_MAX_REF_BITS: u64 = 0x{:016x};",
            max_psi_outer.to_bits(),
        );
        return;
    }
    assert_relative_eq!(
        max_psi_outer,
        f64::from_bits(OUTER_PSI_MAX_REF_BITS),
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
}

/// Peak vertex displacement < `WRAP_THICKNESS`. Hard geometric upper
/// bound: no cavity-wall vertex can move farther than the wrap is
/// thick, since the outer envelope at exactly `phi = WRAP_THICKNESS`
/// is Dirichlet-pinned (the cavity wall reaching the outer envelope
/// would mean the wrap collapsed to zero thickness). Soft penalty
/// contact's elastic equilibrium under static overlap can push the
/// cavity wall farther than the rigid penetration depth — that's
/// expected behaviour, not a violation; only displacement that
/// exceeds the wrap thickness signals a real geometric failure.
fn verify_peak_displacement_bounded(max_disp: f64) {
    assert!(
        max_disp < WRAP_THICKNESS,
        "max cavity-wall displacement {max_disp:e} m ≥ WRAP_THICKNESS {WRAP_THICKNESS} m — \
         geometric sanity violated (wrap should not collapse to zero thickness)",
    );
}

fn verify_n_contact_pairs_exact(n_pairs: usize) {
    if capturing_bits() {
        eprintln!("const N_CONTACT_PAIRS_EXACT: usize = {n_pairs};");
        return;
    }
    assert_eq!(
        n_pairs, N_CONTACT_PAIRS_EXACT,
        "n_contact_pairs at static fit pose",
    );
}

#[allow(clippy::too_many_arguments)]
fn verify_captured_bits(
    force_total_z: f64,
    mean_disp: f64,
    max_disp: f64,
    mean_psi_inner: f64,
    mean_psi_middle: f64,
    mean_psi_outer: f64,
) {
    if capturing_bits() {
        eprintln!(
            "const FORCE_TOTAL_Z_REF_BITS: u64 = 0x{:016x};",
            force_total_z.to_bits(),
        );
        eprintln!(
            "const CAVITY_WALL_MEAN_DISP_REF_BITS: u64 = 0x{:016x};",
            mean_disp.to_bits(),
        );
        eprintln!(
            "const CAVITY_WALL_MAX_DISP_REF_BITS: u64 = 0x{:016x};",
            max_disp.to_bits(),
        );
        eprintln!(
            "const MEAN_PSI_INNER_REF_BITS: u64 = 0x{:016x};",
            mean_psi_inner.to_bits(),
        );
        eprintln!(
            "const MEAN_PSI_MIDDLE_REF_BITS: u64 = 0x{:016x};",
            mean_psi_middle.to_bits(),
        );
        eprintln!(
            "const MEAN_PSI_OUTER_REF_BITS: u64 = 0x{:016x};",
            mean_psi_outer.to_bits(),
        );
        eprintln!("=== END CAPTURED BITS ===");
        return;
    }

    assert_relative_eq!(
        force_total_z,
        f64::from_bits(FORCE_TOTAL_Z_REF_BITS),
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        mean_disp,
        f64::from_bits(CAVITY_WALL_MEAN_DISP_REF_BITS),
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        max_disp,
        f64::from_bits(CAVITY_WALL_MAX_DISP_REF_BITS),
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        mean_psi_inner,
        f64::from_bits(MEAN_PSI_INNER_REF_BITS),
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        mean_psi_middle,
        f64::from_bits(MEAN_PSI_MIDDLE_REF_BITS),
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        mean_psi_outer,
        f64::from_bits(MEAN_PSI_OUTER_REF_BITS),
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
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
    mean_psi_inner: f64,
    mean_psi_middle: f64,
    mean_psi_outer: f64,
    max_psi_outer: f64,
    pair_records: &[Value],
) -> Result<()> {
    let scalars = json!({
        "scan_hx_m": SCAN_HX,
        "scan_hy_m": SCAN_HY,
        "scan_hz_m": SCAN_HZ,
        "wrap_thickness_m": WRAP_THICKNESS,
        "layer_inner_m": LAYER_INNER,
        "layer_middle_outer_m": LAYER_MIDDLE_OUTER,
        "layer_outer_m": LAYER_OUTER,
        "cell_size_m": CELL_SIZE,
        "probe_radius_m": PROBE_RADIUS,
        "probe_penetration_depth_m": PROBE_PENETRATION_DEPTH,
        "probe_center_z_m": PROBE_CENTER_Z,
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
        "mean_psi_inner_j_per_m3": mean_psi_inner,
        "mean_psi_middle_j_per_m3": mean_psi_middle,
        "mean_psi_outer_j_per_m3": mean_psi_outer,
        "max_psi_outer_j_per_m3": max_psi_outer,
    });
    let materials = json!([
        {
            "name": "ECOFLEX_00_20",
            "shell": "inner",
            "n_tets": inner_count,
            "mu_pa": ECOFLEX_00_20.mu,
            "lambda_pa": ECOFLEX_00_20.lambda,
            "density_kg_m3": ECOFLEX_00_20.density,
            "proxy_for": "Ecoflex 00-30 + 75% Slacker (effective Shore 00-20)",
        },
        {
            "name": "DRAGON_SKIN_10A",
            "shell": "middle",
            "n_tets": middle_count,
            "mu_pa": DRAGON_SKIN_10A.mu,
            "lambda_pa": DRAGON_SKIN_10A.lambda,
            "density_kg_m3": DRAGON_SKIN_10A.density,
            "proxy_for": "DS10A + Cu mesh + carbon black (effective Shore 15-18A)",
        },
        {
            "name": "DRAGON_SKIN_20A",
            "shell": "outer",
            "n_tets": outer_count,
            "mu_pa": DRAGON_SKIN_20A.mu,
            "lambda_pa": DRAGON_SKIN_20A.lambda,
            "density_kg_m3": DRAGON_SKIN_20A.density,
            "proxy_for": "DS20A direct (no modifier)",
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
// PLY z-slab artifact emit
// =============================================================================

#[derive(Clone, Copy)]
struct ZslabRecord {
    centroid: Vec3,
    deformed_centroid: Vec3,
}

fn emit_zslab_ply(
    path: &Path,
    records: &[ZslabRecord],
    displacement_magnitudes: &[f64],
    material_ids: &[f64],
) -> Result<()> {
    assert_eq!(records.len(), displacement_magnitudes.len());
    assert_eq!(records.len(), material_ids.len());

    let mut geometry = IndexedMesh::new();
    for r in records {
        // Geometric amplification scales the displacement for visual
        // clarity; the `displacement_magnitude` extra carries the
        // unscaled physical magnitude for quantitative cross-readout.
        // Mirrors rows 11 + 16 + 20's z-slab pattern.
        const DISPLACEMENT_SCALE: f64 = 50.0;
        let amplified = r.centroid + DISPLACEMENT_SCALE * (r.deformed_centroid - r.centroid);
        geometry
            .vertices
            .push(Point3::new(amplified.x, amplified.y, amplified.z));
    }
    // No faces — z-slab is a per-tet centroid cloud, point-only PLY.

    let mut mesh = AttributedMesh::new(geometry);
    let disp_f32: Vec<f32> = displacement_magnitudes.iter().map(|&v| v as f32).collect();
    let mat_f32: Vec<f32> = material_ids.iter().map(|&v| v as f32).collect();
    mesh.insert_extra("displacement_magnitude", disp_f32)?;
    mesh.insert_extra("material_id", mat_f32)?;
    save_ply_attributed(&mesh, path, true)?;
    Ok(())
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    println!("scan-fit-3layer-sleeve — row 21 (Tier 6 synthesis #2)");
    println!();

    // 1. Scan stand-in (typed-Solid cuboid).
    let scan_solid = build_scan_solid();

    // 2. Outer envelope = scan offset by WRAP_THICKNESS.
    let outer_envelope = build_outer_envelope(scan_solid.clone());

    // 3. Sleeve body = outer envelope ⊖ scan (homogeneous typed CSG).
    let sleeve_body = build_sleeve_body(outer_envelope.clone(), scan_solid.clone());

    // 4. 3-layer MaterialField via F4 const entries; partition by
    //    distance-from-scan.
    let material_field = build_material_field();

    // 5. Mesh through SdfMeshedTetMesh::from_sdf — F1+F3 trait
    //    dispatch on `&dyn cf_design::Sdf`.
    let hints = build_hints(material_field);
    let mesh =
        SdfMeshedTetMesh::from_sdf(&sleeve_body, &hints).map_err(|e| anyhow::anyhow!("{e:?}"))?;

    // Snapshot pre-solver views of the mesh (positions, tets, n_tets,
    // n_vertices) for the verify_* + zslab pipelines BEFORE the mesh
    // is moved into the solver.
    let n_tets = mesh.n_tets();
    let n_vertices = mesh.n_vertices();
    let positions: Vec<Vec3> = mesh.positions().to_vec();
    let tets: Vec<[VertexId; 4]> = (0..n_tets as u32).map(|t| mesh.tet_vertices(t)).collect();
    let referenced = referenced_vertices(&mesh);

    // 6. BC: outer-envelope-band Dirichlet pin (probe overlap drives
    //    the cavity from inside; outer envelope is fixed in space).
    let bc = build_boundary_conditions(&mesh, &referenced, &outer_envelope);
    let n_pinned = bc.pinned_vertices.len();

    // Per-shell tet counts at rest centroids — bin by `phi =
    // scan.eval(centroid)` distance-from-scan. Mirrors row 20's
    // shell_at + classification idiom, with the radial-bin replaced
    // by the scan-distance bin.
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
        let phi = scan_solid.eval(nalgebra::Point3::from(centroid));
        let s = shell_at_phi(phi);
        shell_idx_per_tet.push(s);
        match s {
            0 => n_inner += 1,
            1 => n_middle += 1,
            _ => n_outer += 1,
        }
    }

    // 7. Quality + counts gates BEFORE moving the mesh into the
    //    solver. (`mesh.n_tets()` etc. work on `&mesh`; the
    //    `verify_quality_floors` walk is read-only.)
    verify_quality_floors(&mesh);
    verify_counts_exact(
        &mesh,
        &referenced,
        &bc.pinned_vertices,
        n_inner,
        n_middle,
        n_outer,
    );

    // 8. Material assignment partition gate — per-tet `Material::
    //    energy(F_probe)` matches expected[shell_at(centroid)] bit-
    //    equally. Headline gate of the row's multi-material correctness.
    verify_material_assignment_partition(&mesh, &shell_idx_per_tet);

    // 9. Material provenance — F4 const Lamé pair survives
    //    `to_neo_hookean()` round-trip bit-equally per F4's contract
    //    test. Re-asserted here so a regression in F4 trips this row
    //    directly.
    verify_material_provenance();

    // 10. Solver run — penalty contact with the spherical probe as
    //     rigid intrusion primitive (post-PR2 trait unification: any
    //     `impl Sdf` is a valid rigid primitive directly via
    //     `PenaltyRigidContact::new`; row 20 was the first non-plane
    //     consumer, and this row exercises a translated typed-Solid
    //     primitive).
    let probe_solid = build_probe_solid();
    let contact = PenaltyRigidContact::new(vec![probe_solid]);
    let SolveResult {
        rest_positions,
        step,
    } = solve_static(mesh, contact, bc);

    verify_solver_converges(&step);

    // 11. Per-pair contact readout — build a fresh inspection mesh +
    //     contact (the originals were moved into the solver) and call
    //     `per_pair_readout` against the deformed positions. Mirrors
    //     row 18 + 20's two-scene idiom.
    let inspection_mesh = {
        let scan_again = build_scan_solid();
        let outer_again = build_outer_envelope(scan_again.clone());
        let body_again = build_sleeve_body(outer_again, scan_again);
        let inspection_hints = build_hints(build_material_field());
        SdfMeshedTetMesh::from_sdf(&body_again, &inspection_hints)
            .map_err(|e| anyhow::anyhow!("{e:?}"))?
    };
    let inspection_contact = PenaltyRigidContact::new(vec![build_probe_solid()]);

    let positions_vec3: Vec<Vec3> = (0..n_vertices)
        .map(|i| {
            Vec3::new(
                step.x_final[3 * i],
                step.x_final[3 * i + 1],
                step.x_final[3 * i + 2],
            )
        })
        .collect();
    let raw_readouts = inspection_contact.per_pair_readout(&inspection_mesh, &positions_vec3);
    // v2.5-equivalent anchor cleanup (2026-05-08, post-row-22 mirror):
    // drop ORPHAN BCC lattice corners (not in any tet → no FEM
    // stiffness, solver ignores) from the readout. For row 21's
    // geometry (probe inside cavity), orphans dominated raw readouts
    // at ~95-97 % (286/295 at 1 mm penetration) — they polluted
    // `n_pairs` + `force_total_z` and inverted the anchored sign.
    // See pattern (xx) at row 22 patterns memo.
    let readouts = sim_soft::filter_pair_readouts_to_referenced(raw_readouts, &referenced);
    let n_pairs = readouts.len();
    verify_n_contact_pairs_exact(n_pairs);

    let force_total_z: f64 = readouts.iter().map(|r| r.force_on_soft.z).sum();
    let force_magnitudes: Vec<f64> = readouts.iter().map(|r| r.force_on_soft.norm()).collect();
    let mean_force_magnitude = if n_pairs == 0 {
        0.0
    } else {
        force_magnitudes.iter().sum::<f64>() / n_pairs as f64
    };
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
    let mean_disp_magnitude = if disp_magnitudes.is_empty() {
        0.0
    } else {
        disp_magnitudes.iter().sum::<f64>() / disp_magnitudes.len() as f64
    };
    let max_disp_magnitude = disp_magnitudes
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);

    verify_peak_displacement_bounded(max_disp_magnitude);

    // 12. Per-tet strain energy density — first sim-soft user-facing
    //     row to demonstrate post-solve per-tet `Ψ` extraction. The
    //     deformed `positions_vec3` + the `rest_positions` snapshot +
    //     the per-tet vertex indices in `tets` are all that's needed;
    //     the inspection_mesh.materials() carries the per-tet
    //     `NeoHookean` cached at mesh-build time (bit-equal to the
    //     solver-internal materials by construction).
    let materials_inspection = inspection_mesh.materials();
    let mut psi_per_tet: Vec<f64> = Vec::with_capacity(n_tets);
    for (t, &verts) in tets.iter().enumerate() {
        let f = deformation_gradient(verts, &rest_positions, &positions_vec3);
        let psi_t = materials_inspection[t].energy(&f);
        psi_per_tet.push(psi_t);
    }
    let mut sum_psi_inner = 0.0;
    let mut sum_psi_middle = 0.0;
    let mut sum_psi_outer = 0.0;
    let mut max_psi_outer = f64::NEG_INFINITY;
    for (t, &shell) in shell_idx_per_tet.iter().enumerate() {
        let psi = psi_per_tet[t];
        match shell {
            0 => sum_psi_inner += psi,
            1 => sum_psi_middle += psi,
            _ => {
                sum_psi_outer += psi;
                if psi > max_psi_outer {
                    max_psi_outer = psi;
                }
            }
        }
    }
    let mean_psi_inner = if n_inner == 0 {
        0.0
    } else {
        sum_psi_inner / n_inner as f64
    };
    let mean_psi_middle = if n_middle == 0 {
        0.0
    } else {
        sum_psi_middle / n_middle as f64
    };
    let mean_psi_outer = if n_outer == 0 {
        0.0
    } else {
        sum_psi_outer / n_outer as f64
    };

    verify_strain_energy_ordering(mean_psi_inner, mean_psi_middle, mean_psi_outer);
    verify_outer_layer_max_psi(max_psi_outer);
    verify_captured_bits(
        force_total_z,
        mean_disp_magnitude,
        max_disp_magnitude,
        mean_psi_inner,
        mean_psi_middle,
        mean_psi_outer,
    );

    // 13. JSON + PLY readouts.
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;

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
    let json_path = out_dir.join("scan_fit_3layer_sleeve.json");
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
        mean_psi_inner,
        mean_psi_middle,
        mean_psi_outer,
        max_psi_outer,
        &pair_records,
    )?;

    // PLY z-slab — per-tet centroid cloud filtered to `|cz| < CELL/2`,
    // with categorical `material_id` + sequential
    // `displacement_magnitude` per the cf-view artifact-shape
    // decision rule (pattern (aa)).
    let half_cell = 0.5 * CELL_SIZE;
    let mut zslab_records: Vec<ZslabRecord> = Vec::new();
    let mut zslab_disp: Vec<f64> = Vec::new();
    let mut zslab_mat: Vec<f64> = Vec::new();
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
        let deformed_centroid = (positions_vec3[v0 as usize]
            + positions_vec3[v1 as usize]
            + positions_vec3[v2 as usize]
            + positions_vec3[v3 as usize])
            / 4.0;
        let mat_id = shell_idx_per_tet[tet_idx];
        match mat_id {
            0 => n_inner_z += 1,
            1 => n_middle_z += 1,
            _ => n_outer_z += 1,
        }
        zslab_records.push(ZslabRecord {
            centroid: rest_centroid,
            deformed_centroid,
        });
        zslab_disp.push((deformed_centroid - rest_centroid).norm());
        zslab_mat.push(mat_id as f64);
    }
    verify_zslab_counts_exact(n_inner_z, n_middle_z, n_outer_z);

    let ply_path = out_dir.join("sleeve_zslab.ply");
    emit_zslab_ply(&ply_path, &zslab_records, &zslab_disp, &zslab_mat)?;

    // 14. Museum-plaque summary.
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
        mean_psi_inner,
        mean_psi_middle,
        mean_psi_outer,
        max_psi_outer,
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
    mean_psi_inner: f64,
    mean_psi_middle: f64,
    mean_psi_outer: f64,
    max_psi_outer: f64,
    iter_count: usize,
    final_residual: f64,
) {
    println!("Scan stand-in (cf-design Solid::cuboid):");
    println!("  half-extents (m)         : ({SCAN_HX}, {SCAN_HY}, {SCAN_HZ})");
    println!();
    println!("Sleeve wrap geometry:");
    println!("  total wrap thickness     : WRAP_THICKNESS     = {WRAP_THICKNESS} m");
    println!("  inner-layer outer        : LAYER_INNER        = {LAYER_INNER} m");
    println!("  middle-layer outer       : LAYER_MIDDLE_OUTER = {LAYER_MIDDLE_OUTER} m");
    println!("  outer-layer outer        : LAYER_OUTER        = {LAYER_OUTER} m");
    println!("  BCC cell size            : CELL_SIZE          = {CELL_SIZE} m");
    println!();
    println!("Rigid intrusion probe:");
    println!("  PROBE_RADIUS              : {PROBE_RADIUS} m");
    println!("  PROBE_PENETRATION_DEPTH   : {PROBE_PENETRATION_DEPTH} m");
    println!("  PROBE_CENTER_Z            : {PROBE_CENTER_Z} m");
    println!();
    println!("Mesh (post BCC + Isosurface Stuffing):");
    println!("  n_tets        : {n_tets}");
    println!("  n_vertices    : {n_vertices}");
    println!("  n_referenced  : {n_referenced}");
    println!("  n_pinned (outer-envelope band) : {n_pinned}");
    println!("  per-shell tet counts:");
    println!("    inner  (ECOFLEX_00_20)   : {n_inner}");
    println!("    middle (DRAGON_SKIN_10A) : {n_middle}");
    println!("    outer  (DRAGON_SKIN_20A) : {n_outer}");
    println!();
    println!("Contact (PenaltyRigidContact, spherical rigid probe):");
    println!("  n_active_pairs        : {n_pairs}");
    println!("  force_total_z (N)     : {force_total_z:e}");
    println!("  mean force magnitude  : {mean_force_magnitude:e}");
    println!("  max force magnitude   : {max_force_magnitude:e}");
    println!();
    println!("Cavity-wall displacement (active-contact-pair vertices):");
    println!("  mean magnitude (m)    : {mean_disp_magnitude:e}");
    println!("  max magnitude  (m)    : {max_disp_magnitude:e}");
    println!();
    println!("Per-layer mean strain-energy density (J/m³):");
    println!("  Ψ̄_inner  : {mean_psi_inner:e}");
    println!("  Ψ̄_middle : {mean_psi_middle:e}");
    println!("  Ψ̄_outer  : {mean_psi_outer:e}");
    println!("  max Ψ_outer : {max_psi_outer:e}");
    println!();
    println!("Solver:");
    println!("  iter_count            : {iter_count}");
    println!("  final residual norm   : {final_residual:e}");
    println!();
    println!("Outputs:");
    println!("  out/scan_fit_3layer_sleeve.json (scalars + materials + per-pair + per-layer Ψ̄)");
    println!("  out/sleeve_zslab.ply            (z-slab centroid cloud, two scalars)");
    println!();
    println!("View with cf-view (workspace's unified visual-review viewer):");
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve/out/sleeve_zslab.ply"
    );
}

// =============================================================================
// Compile-time assertions on geometric invariants
// =============================================================================

const _: () = {
    assert!(SCAN_HX > 0.0);
    assert!(SCAN_HY > 0.0);
    assert!(SCAN_HZ > 0.0);
    assert!(LAYER_INNER > 0.0);
    assert!(LAYER_INNER < LAYER_MIDDLE_OUTER);
    assert!(LAYER_MIDDLE_OUTER < LAYER_OUTER);
    // LAYER_OUTER == WRAP_THICKNESS by definition
    assert!(WRAP_THICKNESS > 0.0);
    assert!(CELL_SIZE > 0.0);
    assert!(STATIC_DT > 0.0);
    assert!(PROBE_RADIUS > 0.0);
    assert!(PROBE_PENETRATION_DEPTH > 0.0);
    assert!(PROBE_PENETRATION_DEPTH < PROBE_RADIUS);
    // The thinnest layer (outer at LAYER_OUTER - LAYER_MIDDLE_OUTER =
    // 4 mm at v1) carries at least one BCC cell across thickness.
    assert!(LAYER_OUTER - LAYER_MIDDLE_OUTER >= CELL_SIZE);
    // Bbox covers the outer envelope with one-cell-edge slack on
    // every axis.
    assert!(BBOX_HALF_X > SCAN_HX + WRAP_THICKNESS);
    assert!(BBOX_HALF_Y > SCAN_HY + WRAP_THICKNESS);
    assert!(BBOX_HALF_Z > SCAN_HZ + WRAP_THICKNESS);
};
