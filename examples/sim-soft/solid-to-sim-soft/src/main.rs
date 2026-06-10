//! solid-to-sim-soft — `cf_design::Solid` is a first-class SDF for
//! `SdfMeshedTetMesh::from_sdf`, shipped at PR3 F1+F3.
//!
//! A typed boolean-difference body composed via cf-design's CSG kernel
//! (`Solid::sphere(R_OUTER).subtract(Solid::sphere(R_CAVITY))`) flows
//! into sim-soft's BCC + Labelle-Shewchuk Isosurface Stuffing pipeline
//! through a single trait-object coercion. No per-shape glue: F1 ships
//! `impl Sdf for Solid` and F3 re-exports `cf_design::Sdf` as
//! `sim_soft::Sdf`, so `&solid: &dyn cf_design::Sdf` is what
//! `SdfMeshedTetMesh::from_sdf(sdf: &dyn Sdf, ...)` already accepts —
//! load-bearing demonstration of the bridge.
//!
//! Companion to row 15 `mesh-scan-as-solid` (cf-design-side, scan-derived
//! `mesh_sdf::Signed<TriMeshDistance, _>` consumed via the same trait): row 15
//! validates the mesh-derived SDF impl against a closed-form cube anchor;
//! row 16 validates the typed-`Solid` consumer against the production
//! sim-soft `DifferenceSdf<SphereSdf>` baseline (semantic-equivalence
//! anchor). The two rows together close the bridge story PR3 was set up
//! for: typed CSG bodies AND scanned bodies are both first-class
//! `cf_design::Sdf` primitives in sim-soft.
//!
//! ## Geometry
//!
//! Hollow shell with `R_OUTER = 0.10 m`, `R_CAVITY = 0.04 m` —
//! geometry-identical to the canonical IV-5 / row 11 hollow silicone
//! sphere fixture, so the resulting mesh's tet/vertex/boundary counts
//! are bit-equal to row 11's at the same `cell_size = h/2 = 0.02 m`
//! (mesh topology is a function of geometry + `cell_size` only, not of
//! material partitioning). Cross-row continuity: the captured cavity-
//! wall mean bits below match row 11's `CAVITY_WALL_UNIFORM_1X_REF_BITS`
//! exactly, since `material_field: None` → `MaterialField::skeleton_default
//! = uniform(1e5, 4e5)` is bit-equal to row 11's `MU_INNER`/`LAMBDA_INNER`.
//!
//! ## Single-material Lamé closed-form
//!
//! Hollow sphere with internal pressure `p` on `R_a = R_CAVITY` and
//! fixed outer surface `u_r(R_b) = 0` at `R_b = R_OUTER`, single
//! material `(μ, λ) = (1e5, 4e5)`:
//!
//! ```text
//! u_r(r) = A·r + B/r²
//! σ_rr(r) = K·A − 4μ·B/r³,   K = 3λ + 2μ
//! ```
//!
//! BCs:
//! ```text
//! u_r(R_b) = 0           ⇒  B = -A·R_b³
//! σ_rr(R_a) = -p         ⇒  K·A − 4μ·B/R_a³ = -p
//!                        ⇒  A·[K + 4μ·R_b³/R_a³] = -p
//! ```
//!
//! Then:
//! ```text
//! A = -p / [K + 4μ·R_b³/R_a³]
//! B = -A·R_b³
//! u_r(R_a) = A·R_a + B/R_a² = A·(R_a − R_b³/R_a²)
//! ```
//!
//! At the row's constants `(p, μ, λ, R_a, R_b) = (5e3, 1e5, 4e5, 0.04,
//! 0.10)`, `K = 1.4e6`, `(R_b/R_a)³ = 15.625`, `4μ·R_b³/R_a³ = 6.25e6`,
//! denominator `= 7.65e6`, `A ≈ -6.536e-4`, `B ≈ 6.536e-7`,
//! `u_r(R_a) ≈ -6.536e-4 · 0.04 + 6.536e-7 / 1.6e-3 = -2.614e-5 +
//! 4.085e-4 = 3.823e-4 m`. Cavity wall expands ≈ 3.823e-4 m radially
//! outward — about `0.96 %` of `R_CAVITY = 0.04 m`, well inside the
//! Neo-Hookean small-strain band where Lamé is the leading-order
//! approximation.
//!
//! ## Anchor groups (all assertions exit-0 on success)
//!
//! - **`bridge_equivalence`** — HEADLINE A. The typed-`Solid` mesh
//!   `equals_structurally` the sim-soft `DifferenceSdf<SphereSdf>`
//!   baseline mesh AND positions match bit-exact at every vertex.
//!   Underlying arithmetic is identical: cf-design's `Sphere` evaluates
//!   `p.coords.norm() - radius`, `Subtract` is `a.max(-b)` —
//!   bit-equal to sim-soft's `SphereSdf::eval` and `DifferenceSdf::eval`,
//!   so the BCC + warp + stuffing pipeline samples bit-identical SDF
//!   values and produces bit-identical meshes. `EXACT_TOL = 0.0`
//!   anchors the bridge as semantic-preserving.
//! - **`counts_exact`** — `n_tets`, `n_vertices`, `n_referenced`,
//!   `n_pinned`, `n_loaded` exact-pinned per the III-1 determinism
//!   contract. Counts captured from row 11's identical-geometry fixture
//!   (cross-row sanity check + zero-recapture cycle).
//! - **`quality_floors`** — Theorem-1 sanity floors per row 3:
//!   per-tet `signed_volume > 0`, `aspect_ratio ≥ 0.05`,
//!   `dihedral ∈ [5°, 175°]`.
//! - **`solver_converges`** — `iter_count < cfg.max_newton_iter = 50`,
//!   `final_residual_norm < cfg.tol = 1e-10`.
//! - **`cavity_wall_lame`** — HEADLINE B. Cavity-wall mean radial
//!   displacement (mean over `bc.loaded_vertices` of `|x_final[v]| -
//!   |rest[v]|`) vs single-material Lamé closed-form within
//!   `max_relative = 0.30, epsilon = 5e-6 m` — same gate row 11 uses
//!   for its three-shell cavity readout. At h/2, observed `≈ 3.245e-4 m`
//!   (`~15 %` rel-err vs analytic `≈ 3.823e-4 m`); IV-5 documents
//!   super-quadratic convergence at the fine end.
//! - **`captured_cavity_wall_mean_bits`** — observed cavity-wall mean
//!   bits captured under the IV-1 sparse-tier rel-tol contract
//!   (`assert_relative_eq!` at `1e-12` rel, NOT strict `to_bits`
//!   equality). Bit-equal to row 11's `CAVITY_WALL_UNIFORM_1X_REF_BITS`
//!   by construction — same geometry, same Lamé pair, same solver
//!   path. Cross-row continuity guard: any drift here surfaces a
//!   regression in the typed-`Solid` → sim-soft bridge OR a regression
//!   in the shared FEM hot path that row 11 would also catch.
//!
//! ## cf-view artifact
//!
//! Per-tet centroid point cloud filtered to the
//! `|rest_centroid.z| < CELL_SIZE/2 = 0.01 m` z-slab, with
//! `DISPLACEMENT_SCALE = 50.0` geometric amplification on positions
//! (`amplified = rest_centroid + SCALE * (deformed_centroid -
//! rest_centroid)`). 616 centroids in the slab — bit-equal to row
//! 11's per-shell z-slab sum at the same geometry + `cell_size`. The
//! `radial_displacement` per-vertex scalar carries the TRUE physical
//! FEM displacement (`|deformed_centroid| - |rest_centroid|`); every
//! `verify_*` operates on the unscaled solver outputs.
//!
//! `DISPLACEMENT_SCALE = 50.0` and the z-slab pattern both inherit
//! from row 11's cf-view artifact convention verbatim — the
//! single-material variant just drops row 11's `material_id`
//! categorical scalar. The slab projects centroids onto a 2-D annulus
//! at z=0; centroids are interior to the wall by ~half-cell-edge by
//! construction (a centroid is the mean of 4 tet vertex positions,
//! so wall-incident tets have centroids one-quarter-edge inside the
//! body). Empirically `|p_xy|` spans `[0.054, 0.097] m` — cavity-side
//! ring just outside `R_CAVITY = 0.04 m` (innermost centroids at
//! `~0.054 m`), outer-side ring just inside `R_OUTER = 0.10 m`
//! (outermost centroids at `~0.097 m`); interior centroids between
//! the two rings show the radial-decay profile through the shell
//! body's middle (no occlusion — every centroid is visible from
//! cf-view's default oblique orbit angle).
//!
//! cf-view's auto-detect picks sequential viridis (unipolar continuous
//! per pattern (u)); colormap range `[0, ~3.0e-4]` empirically (the
//! per-centroid peak is `~3.005e-4 m`, smaller than the per-vertex
//! peak `~4.3e-4 m` because centroid displacement averages 4 vertex
//! displacements). The cavity-side ring reads bright yellow/green
//! (peak `~3.0e-4 m`); the outermost ring reads deep purple (`0`,
//! Dirichlet pin); the radial decay between them is continuous. For
//! reference: HEADLINE B's `bc.loaded_vertices` per-vertex cavity-wall
//! mean is `~3.245e-4 m`; analytic single-material Lamé predicts
//! `~3.823e-4 m` at `R_CAVITY` — the `~15 %` rel-err vs analytic
//! lives in the Tet4 Newton + half-cell convergence band per IV-5.
//!
//! Why z-slab over full-boundary-surface (the row 3 sphere precedent
//! the initial row-16 commit `82aaca4b` adopted): a hollow body's
//! full boundary surface 360°-occludes the inner cavity from every
//! orbit angle, and cf-view does not expose section-cut UI; the inner
//! cavity's displacement signal becomes invisible. Row 16 adopts row
//! 11's z-slab + per-tet centroid pattern for hollow-body
//! visualization. Note row 11's z-slab choice was driven by visual
//! density management (cf-view's instanced-sphere radius bug at the
//! time made dense centroid clouds visually overcrowded) rather than
//! occlusion; row 16 inherits the SAME PATTERN for a DIFFERENT REASON
//! (occlusion). Banked at iter-15 N+3 visuals-pass.

// PLY field-data is single-precision on disk; converting f64
// `radial_displacement` to f32 for the AttributedMesh extras is
// intrinsic to the PLY format. Same precedent as rows 1+2+3+8+9+10+11+15.
#![allow(clippy::cast_possible_truncation)]
// `usize as u32` casts on `mesh.n_tets()` (max ~7k here, ≪ u32::MAX) — the
// standard Mesh-trait API tax mirrored across the workspace (rows 8+9+
// 10+11+15).
#![allow(clippy::cast_possible_wrap)]
// `expect(...)` on the canonical scene's `from_sdf` result + the
// boundary-PLY remap lookup: same precedent as row 3 + row 11.
#![allow(clippy::expect_used)]
// `usize as f64` casts on loaded-vertex counts for mean computations.
// Counts ≤ ~200 here, well within f64 mantissa exact range. Same
// allowance as rows 6+8+9+10+11.
#![allow(clippy::cast_precision_loss)]
// Domain-meaningful naming pairs (`mesh_typed`/`mesh_baseline`,
// `pos_typed`/`pos_baseline`) distinguish the two bridge meshes in the
// equivalence anchor. Same precedent as rows 6+10+11.
#![allow(clippy::similar_names)]
// `print_summary` is a single museum-plaque stdout writer; splitting
// fragments the visual format without information gain. Same allowance
// as rows 4+5+6+9+10+11+15.
#![allow(clippy::too_many_lines)]
// `print_summary` aggregates many scalars into the museum-plaque format;
// threading them through a struct adds indirection without information
// gain. Same allowance as rows 10+11.
#![allow(clippy::too_many_arguments)]

use std::collections::BTreeSet;
use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use cf_design::Solid;
use mesh_io::save_ply_attributed;
use mesh_types::{AttributedMesh, IndexedMesh, Point3};
use sim_ml_chassis::Tensor;
use sim_soft::{
    Aabb3, BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, DifferenceSdf,
    LAYERED_SPHERE_BBOX_HALF_EXTENT, LAYERED_SPHERE_R_CAVITY, LAYERED_SPHERE_R_OUTER, LoadAxis,
    Mesh, MeshingHints, NewtonStep, NullContact, SceneInitial, Sdf, SdfMeshedTetMesh, Solver,
    SolverConfig, SphereSdf, Tet4, Vec3, VertexId, pick_vertices_by_predicate, referenced_vertices,
};

// =============================================================================
// Constants — material parameters (single-material variant of row 11's
// uniform-1× baseline; bit-equal to MaterialField::skeleton_default)
// =============================================================================

/// Single-material `μ` (Pa). Pinned to row 11's `MU_INNER` so this row's
/// FEM solve produces a cavity-wall mean bit-equal to row 11's
/// `CAVITY_WALL_UNIFORM_1X_REF_BITS` — cross-row continuity anchor.
/// Equivalent to `MaterialField::skeleton_default = uniform(1e5, 4e5)`.
const MU: f64 = 1.0e5;

/// Single-material `λ` (Pa). `λ = 4 μ` ⇒ `ν = 0.4` compressible regime
/// per IV-3 + IV-5 deviation from Decision J's near-incompressible
/// Ecoflex (recovered at Phase H Tet10 + F-bar).
const LAMBDA: f64 = 4.0e5;

// =============================================================================
// Constants — pressure + refinement + solver config
// =============================================================================

/// Internal pressure on the cavity wall (Pa). Mirrors row 11's `PRESSURE`
/// verbatim — engineered to land cavity-wall inflation in the small-
/// strain band (`< ~1 %`) where Neo-Hookean reduces to Lamé. At the
/// chosen radii the single-material closed-form predicts cavity-wall
/// `u_r(R_CAVITY) ≈ 3.823e-4 m` ≈ `~0.96 %` of `R_CAVITY = 0.04 m`
/// (FEM observed at h/2 = `~3.245e-4 m` ≈ `~0.81 %`, `~15 %` rel-err vs
/// analytic).
const PRESSURE: f64 = 5.0e3;

/// BCC lattice spacing (m). `cell_size = h/2 = 0.02` — the canonical
/// Phase 3 / IV-4 mid-refinement and row 11's `CELL_SIZE` verbatim, so
/// mesh topology agrees bit-exactly with row 11's at the same geometry.
const CELL_SIZE: f64 = 0.02;

/// Static-regime time step (s). Mirrors row 11's `STATIC_DT` and IV-3 /
/// IV-5's `STATIC_DT` verbatim: at large `dt`, the backward-Euler
/// residual's inertial term `M / dt² · (x − x_prev)` collapses to
/// negligible vs the stiffness contribution, so a single `replay_step`
/// from rest converges to the static equilibrium far below `tol = 1e-10`.
const STATIC_DT: f64 = 1.0;

/// Newton iter cap. Mirrors row 11 verbatim: empirically Newton
/// converges in `~3` iters under the radially-symmetric distributed-
/// pressure load with fixed-outer-pin BC. `50` leaves headroom against
/// future load / material perturbations.
const MAX_NEWTON_ITER: usize = 50;

// =============================================================================
// Constants — Lamé closed-form + tolerance
// =============================================================================

/// Bit-exact tolerance for the bridge-equivalence position-by-position
/// match (HEADLINE A). The cf-design `Sphere` SDF evaluates
/// `p.coords.norm() - radius` and `Subtract` is `a.max(-b)`; sim-soft's
/// `SphereSdf::eval` is `p.coords.norm() - radius` and
/// `DifferenceSdf::eval` is `phi_a.max(-phi_b)` — bit-identical
/// arithmetic on the FP path. The BCC + warp + stuffing pipeline
/// samples bit-equal SDF values and produces bit-equal meshes.
/// `epsilon = 0.0` satisfies clippy's `float_cmp` lint without actually
/// relaxing the bound (mirrors rows 8+9+10+11+15's `EXACT_TOL = 0.0`).
const EXACT_TOL: f64 = 0.0;

/// IV-1 sparse-tier rel-tol for captured cavity-wall mean bits. ~6.5k
/// tets through faer's sparse Cholesky lives between IV-1's dense
/// bit-equal tier (12-24 DOFs) and IV-1's sparse-at-scale tier
/// (~3k tets, 3-ULP cross-platform drift on faer's per-column FMA-
/// fusion path). `1e-12` admits sparse-solver SIMD/FMA noise while
/// catching any real regression. Same precedent as rows 6+10+11.
const SPARSE_REL_TOL: f64 = 1.0e-12;

/// Absolute floor for the captured-bits comparison; below the
/// cavity-wall mean magnitude (`~3.245e-4 m`) by 8 orders. Same
/// precedent as rows 6+10+11.
const SPARSE_EPS_ABS: f64 = 1.0e-12;

/// Cavity-wall rel-err tolerance for the Lamé closed-form comparison
/// (HEADLINE B). Mirrors row 11's `RADIAL_REL_TOL = 0.30` cavity gate
/// verbatim. Observed `~15 %` (well under gate); the `< 0.30`
/// envelope absorbs FEM Tet4 quadratic-convergence headroom at h/2
/// (IV-5 documents super-quadratic empirical rate at the fine end).
const RADIAL_REL_TOL: f64 = 0.30;

/// Absolute floor for the cavity-wall Lamé gate. `5e-6 m` is below the
/// cavity-wall observed magnitude (`~3.245e-4 m`) by 2 orders, but
/// retained for consistency with row 11's gate signature; not load-
/// bearing here (rel-tol branch is binding at single-material h/2).
const RADIAL_EPS_ABS_FLOOR: f64 = 5.0e-6;

// =============================================================================
// Constants — geometry derived (re-exports from sim-soft + display scale)
// =============================================================================

/// Visualisation-only geometric displacement scale factor — applied to
/// PLY vertex positions ONLY (`vertex = rest + SCALE * (deformed -
/// rest)`), NOT to the `radial_displacement` per-vertex scalar (which
/// continues to carry the TRUE physical displacement) or to any
/// numerical assertion (every `verify_*` operates on the unscaled
/// solver outputs). Same `50.0` amplifier as row 11 — small-strain
/// spherical-symmetry regime where `50×` puts cavity-wall inflation at
/// `~36 %` of `R_CAVITY`, dramatically visible without distorting
/// spherical-symmetry readability.
const DISPLACEMENT_SCALE: f64 = 50.0;

// =============================================================================
// Exact-pinned counts (III-1 determinism contract; cross-row continuity
// with row 11's identical-geometry fixture)
// =============================================================================

/// Total tet count at `cell_size = h/2 = 0.02` on the hollow-shell body
/// `(R_OUTER=0.10, R_CAVITY=0.04)`. Bit-exact match with row 11's
/// `N_TETS_EXACT` and IV-5's three-shell convergence test at h/2 — same
/// geometry, same `cell_size` ⇒ same BCC + warp + stuffing output (the
/// material partitioning is a per-tet attribute on identical topology).
const N_TETS_EXACT: usize = 6456;

/// Total mesh vertex count, including BCC lattice corners not
/// referenced by any tet (orphans). Bit-exact match with row 11.
const N_VERTICES_EXACT: usize = 4682;

/// Vertices referenced by at least one tet. `N_VERTICES_EXACT -
/// N_REFERENCED_EXACT = 3202` orphan BCC lattice corners excluded from
/// solver participation. Bit-exact match with row 11.
const N_REFERENCED_EXACT: usize = 1480;

/// Outer-surface-band pinned vertex count (every vertex with
/// `(‖p‖ - R_OUTER).abs() < cell_size/2 = 0.01` filtered to referenced
/// set). Bit-exact match with row 11's `N_PINNED_EXACT`.
const N_PINNED_EXACT: usize = 734;

/// Cavity-surface-band loaded vertex count (every vertex with
/// `(‖p‖ - R_CAVITY).abs() < cell_size/2 = 0.01` filtered to referenced
/// set). Bit-exact match with row 11's `N_LOADED_EXACT`.
const N_LOADED_EXACT: usize = 134;

/// Per-tet centroid count in the `|rest_centroid.z| < cell_size/2 = 0.01`
/// z-slab cut for the cf-view PLY artifact (mirrors row 11's z-slab
/// pattern, single-material variant). Captured at row 16 N+3 visuals-
/// pass-driven pivot from full-boundary-surface to z-slab centroid cloud
/// (the boundary-surface artifact failed cf-view occlusion review — outer
/// shell occluded inner cavity 360°). Bit-equal to row 11's per-shell
/// z-slab sum (`184 + 176 + 256 = 616`); single-material row 16 doesn't
/// partition by shell but the slab-mask is identical.
const N_ZSLAB_TETS_EXACT: usize = 616;

// ── Captured cavity-wall mean bits (cross-row continuity vs row 11) ─────
//
// Bit-exact match with row 11's `CAVITY_WALL_UNIFORM_1X_REF_BITS` by
// construction — same geometry, same Lamé pair (`MU = 1e5, LAMBDA =
// 4e5`), same solver path through faer's sparse Cholesky. Captured at
// row 11's land time (sim-soft `dev` post-row-10 tip `6f84c9cf`,
// rustc 1.95.0 on macOS arm64). See row 11's
// `CAVITY_WALL_UNIFORM_1X_REF_BITS` docstring for the IV-1 sparse-tier
// failure-mode protocol — relative tolerance, NEVER re-bake.

/// Cavity-wall mean radial displacement bits (m). Mean over the
/// cavity-band loaded vertices of `(|x_final[v]| - |rest[v]|)`.
/// `f64::from_bits(0x3f35_43f1_0b56_98c8) ≈ 3.244_842_040_096_671_161e-4`.
/// Rel-err vs single-material analytic `≈ 3.823e-4 m` is `~15 %`
/// (well under `RADIAL_REL_TOL = 0.30`).
const CAVITY_WALL_MEAN_REF_BITS: u64 = 0x3f35_43f1_0b56_98c8;

// =============================================================================
// 0. verify_geometry_invariants — compile-time const asserts
// =============================================================================

const fn verify_geometry_invariants() {
    // Row 11's geometry-invariant subset that applies to the single-
    // material variant. Reaching runtime here is a no-op; the const-
    // block panics surface at compile time on a regression and the
    // source location anchors the dependency on the geometry constants
    // loud and user-facing.
    const { assert!(LAYERED_SPHERE_R_CAVITY > 0.0) };
    const { assert!(LAYERED_SPHERE_R_CAVITY < LAYERED_SPHERE_R_OUTER) };
    const { assert!(LAYERED_SPHERE_BBOX_HALF_EXTENT > LAYERED_SPHERE_R_OUTER) };
    const { assert!(PRESSURE > 0.0) };
    const { assert!(MU > 0.0) };
    const { assert!(LAMBDA == 4.0 * MU) };
    const { assert!(CELL_SIZE > 0.0) };
    // BCC + stuffing canonical bbox-margin ratio per Phase 3 / IV-4.
    const { assert!(LAYERED_SPHERE_BBOX_HALF_EXTENT / CELL_SIZE >= 6.0) };
    // Single-material analytic well-conditioning: denominator
    // `K + 4μ·R_b³/R_a³` is positive at any positive (μ, λ, R_a, R_b)
    // with R_b > R_a, so no further conditioning gate is needed
    // (vs row 11's 6×6 `κ < 1e12` gate for the three-shell case).
    const { assert!(MAX_NEWTON_ITER > 0) };
    const { assert!(STATIC_DT > 0.0) };
}

// =============================================================================
// Single-material Lamé closed-form
// =============================================================================

/// Closed-form cavity-wall radial displacement `u_r(R_a)` for a hollow
/// sphere with internal pressure `p` on `R_a`, fixed outer at `R_b`,
/// and single material `(μ, λ)`. Derivation in module docstring.
fn cavity_wall_lame_analytic(mu: f64, lambda: f64, p: f64, r_a: f64, r_b: f64) -> f64 {
    let k = 3.0_f64.mul_add(lambda, 2.0 * mu);
    let denom = (4.0 * mu).mul_add(r_b.powi(3) / r_a.powi(3), k);
    let a_coeff = -p / denom;
    let b_coeff = -a_coeff * r_b.powi(3);
    a_coeff.mul_add(r_a, b_coeff / (r_a * r_a))
}

// =============================================================================
// 1. Bridge — typed Solid + sim-soft DifferenceSdf baseline
// =============================================================================

/// Build the typed cf-design body: hollow shell via the CSG kernel.
fn build_typed_solid() -> Solid {
    Solid::sphere(LAYERED_SPHERE_R_OUTER).subtract(Solid::sphere(LAYERED_SPHERE_R_CAVITY))
}

/// Build the sim-soft `DifferenceSdf` baseline — same hollow shell via
/// PR1-era `SphereSdf` primitives. The bridge-equivalence reference: any
/// drift between the typed-Solid and `DifferenceSdf`-meshed outputs surfaces
/// at HEADLINE A.
fn build_baseline_difference_sdf() -> DifferenceSdf {
    DifferenceSdf::new(
        Box::new(SphereSdf {
            radius: LAYERED_SPHERE_R_OUTER,
        }),
        Box::new(SphereSdf {
            radius: LAYERED_SPHERE_R_CAVITY,
        }),
    )
}

/// Canonical meshing hints — same `bbox` + `cell_size` for both meshes.
/// `material_field: None` selects `MaterialField::skeleton_default`
/// (`uniform(1e5, 4e5)`); single-material body, no per-tet partitioning.
fn canonical_hints() -> MeshingHints {
    let half = LAYERED_SPHERE_BBOX_HALF_EXTENT;
    MeshingHints {
        bbox: Aabb3::new(Vec3::new(-half, -half, -half), Vec3::new(half, half, half)),
        cell_size: CELL_SIZE,
        material_field: None,
    }
}

/// Mesh the body via `SdfMeshedTetMesh::from_sdf` taking `&dyn Sdf` —
/// load-bearing demonstration that `&Solid` AND `&DifferenceSdf` both
/// coerce to `&dyn Sdf` (post-F1+F3 unification).
fn mesh_via(sdf: &dyn Sdf, hints: &MeshingHints) -> SdfMeshedTetMesh {
    SdfMeshedTetMesh::from_sdf(sdf, hints)
        .expect("canonical hollow-shell scene meshes successfully — see III-1")
}

// =============================================================================
// 2. verify_bridge_equivalence — HEADLINE A
// =============================================================================

/// HEADLINE A — typed-`Solid` mesh agrees with sim-soft `DifferenceSdf`
/// baseline `equals_structurally` AND positions match bit-exact.
fn verify_bridge_equivalence(typed: &SdfMeshedTetMesh, baseline: &SdfMeshedTetMesh) {
    assert!(
        typed.equals_structurally(baseline),
        "bridge equivalence: typed-Solid mesh and DifferenceSdf<SphereSdf> baseline \
         disagree on (n_vertices, n_tets, per-tet vertex IDs) — F1 impl Sdf for Solid \
         OR cf-design Sphere/Subtract evaluation drifted from sim-soft SphereSdf/DifferenceSdf",
    );

    let pos_typed = typed.positions();
    let pos_baseline = baseline.positions();
    assert_eq!(
        pos_typed.len(),
        pos_baseline.len(),
        "bridge equivalence: position vector length differs ({} vs {})",
        pos_typed.len(),
        pos_baseline.len(),
    );
    for (pt, pb) in pos_typed.iter().zip(pos_baseline.iter()) {
        assert_relative_eq!(pt.x, pb.x, epsilon = EXACT_TOL);
        assert_relative_eq!(pt.y, pb.y, epsilon = EXACT_TOL);
        assert_relative_eq!(pt.z, pb.z, epsilon = EXACT_TOL);
    }
}

// =============================================================================
// 3. verify_counts_exact — III-1 determinism + cross-row continuity
// =============================================================================

fn verify_counts_exact(
    mesh: &SdfMeshedTetMesh,
    referenced: &[VertexId],
    pinned: &[VertexId],
    loaded: &[VertexId],
) {
    let n_tets = mesh.n_tets();
    let n_vertices = mesh.n_vertices();
    let n_referenced = referenced.len();
    assert_eq!(
        n_tets, N_TETS_EXACT,
        "n_tets drift: got {n_tets}, expected {N_TETS_EXACT} \
         (cross-row continuity vs row 11's identical-geometry fixture)",
    );
    assert_eq!(
        n_vertices, N_VERTICES_EXACT,
        "n_vertices drift: got {n_vertices}, expected {N_VERTICES_EXACT}"
    );
    assert_eq!(
        n_referenced, N_REFERENCED_EXACT,
        "referenced vertex count drift: got {n_referenced}, expected {N_REFERENCED_EXACT}"
    );
    assert!(
        n_referenced < n_vertices,
        "orphan-rejection invariant vacuous: referenced ({n_referenced}) == n_vertices \
         ({n_vertices})"
    );
    assert_eq!(
        pinned.len(),
        N_PINNED_EXACT,
        "pinned (outer-surface band) cardinality drift: got {}, expected {N_PINNED_EXACT}",
        pinned.len(),
    );
    assert_eq!(
        loaded.len(),
        N_LOADED_EXACT,
        "loaded (cavity-surface band) cardinality drift: got {}, expected {N_LOADED_EXACT}",
        loaded.len(),
    );
}

// =============================================================================
// 4. verify_quality_floors — Theorem 1 sanity floors
// =============================================================================

/// Per-tet `signed_volume > 0` strict (D-10 detector); `aspect_ratio ≥
/// 0.05`; `dihedral_min ≥ 5°`; `dihedral_max ≤ 175°`. Mirrors row 3's
/// quality-floor anchors verbatim — single-material variant uses the
/// same Theorem 1 bounds because the BCC + stuffing pipeline is
/// material-blind for tet quality.
fn verify_quality_floors(mesh: &SdfMeshedTetMesh) {
    let q = mesh.quality();
    let alpha_lo = 5.0_f64.to_radians();
    let alpha_hi = 175.0_f64.to_radians();
    for (tet_id, &v) in q.signed_volume.iter().enumerate() {
        assert!(
            v > 0.0,
            "tet {tet_id}: signed_volume {v:e} ≤ 0 — D-10 filter or post-hoc \
             orientation swap regressed",
        );
    }
    for (tet_id, &a) in q.aspect_ratio.iter().enumerate() {
        assert!(
            a >= 0.05,
            "tet {tet_id}: aspect_ratio {a:.4} < RHO_MIN 0.05",
        );
    }
    for tet_id in 0..q.dihedral_min.len() {
        let lo = q.dihedral_min[tet_id];
        let hi = q.dihedral_max[tet_id];
        assert!(
            lo >= alpha_lo,
            "tet {tet_id}: dihedral_min {:.2}° below 5°",
            lo.to_degrees(),
        );
        assert!(
            hi <= alpha_hi,
            "tet {tet_id}: dihedral_max {:.2}° above 175°",
            hi.to_degrees(),
        );
    }
}

// =============================================================================
// 5. Build BoundaryConditions — cavity-loaded + outer-pinned
// =============================================================================

/// Build the BC set: cavity-wall (`R_CAVITY` band) gets `LoadAxis::FullVector`
/// radially-outward force; outer-wall (`R_OUTER` band) gets full Dirichlet
/// pin. Mirrors `SoftScene::layered_silicone_sphere`'s BC construction
/// verbatim — half-cell `band_tol` brackets the BCC mesher's cut-points
/// (which lie on the linear-interp secant zero, ≤ `O(cell_size²)` off the
/// analytic surface) plus any warp-snapped lattice vertices.
fn build_boundary_conditions(
    mesh: &SdfMeshedTetMesh,
    referenced: &[VertexId],
) -> (BoundaryConditions, Vec<VertexId>, Tensor<f64>) {
    let referenced_set: BTreeSet<VertexId> = referenced.iter().copied().collect();
    let band_tol = 0.5 * CELL_SIZE;
    let pinned: Vec<VertexId> = pick_vertices_by_predicate(mesh, |p| {
        (p.norm() - LAYERED_SPHERE_R_OUTER).abs() < band_tol
    })
    .into_iter()
    .filter(|v| referenced_set.contains(v))
    .collect();
    let loaded: Vec<VertexId> = pick_vertices_by_predicate(mesh, |p| {
        (p.norm() - LAYERED_SPHERE_R_CAVITY).abs() < band_tol
    })
    .into_iter()
    .filter(|v| referenced_set.contains(v))
    .collect();
    assert!(
        !pinned.is_empty(),
        "outer-surface band turned up empty at cell_size = {CELL_SIZE}",
    );
    assert!(
        !loaded.is_empty(),
        "cavity-surface band turned up empty at cell_size = {CELL_SIZE}",
    );

    // Per-vertex tributary area under uniform Saint-Venant
    // distribution: total cavity surface area divided over every
    // loaded vertex. Pressure-times-area-times-normal force at each
    // loaded vertex sums (vectorially) to zero for a closed sphere by
    // symmetry, but its radial component sums to the total internal-
    // pressure thrust 4π R_CAVITY² · p — the analytic load magnitude.
    let area_per_vertex =
        4.0 * std::f64::consts::PI * LAYERED_SPHERE_R_CAVITY * LAYERED_SPHERE_R_CAVITY
            / loaded.len() as f64;
    let positions = mesh.positions();
    let mut theta_data = vec![0.0; 3 * loaded.len()];
    for (i, &v) in loaded.iter().enumerate() {
        let p = positions[v as usize];
        let radius = p.norm();
        let n_hat = p / radius;
        let force = PRESSURE * area_per_vertex;
        theta_data[3 * i] = force * n_hat.x;
        theta_data[3 * i + 1] = force * n_hat.y;
        theta_data[3 * i + 2] = force * n_hat.z;
    }
    let theta = Tensor::from_slice(&theta_data, &[3 * loaded.len()]);

    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::FullVector)).collect(),
    };
    (bc, loaded, theta)
}

// =============================================================================
// 6. Solver run — one backward-Euler replay_step
// =============================================================================

struct SolveResult {
    rest_positions: Vec<Vec3>,
    step: NewtonStep<sim_soft::CpuTape>,
}

fn solve_static(
    mesh: SdfMeshedTetMesh,
    bc: BoundaryConditions,
    theta: &Tensor<f64>,
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

    let solver: CpuTet4NHSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
    let step = solver.replay_step(&initial.x_prev, &initial.v_prev, theta, cfg.dt);

    SolveResult {
        rest_positions,
        step,
    }
}

// =============================================================================
// 7. verify_solver_converges
// =============================================================================

fn verify_solver_converges(result: &SolveResult, cfg: &SolverConfig) {
    assert!(
        result.step.iter_count < cfg.max_newton_iter,
        "Newton hit iter cap: iter_count = {}, max_newton_iter = {}",
        result.step.iter_count,
        cfg.max_newton_iter,
    );
    assert!(
        result.step.final_residual_norm < cfg.tol,
        "final_residual_norm {} >= tol {} — Newton did not converge",
        result.step.final_residual_norm,
        cfg.tol,
    );
}

// =============================================================================
// 8. verify_cavity_wall_lame — HEADLINE B
// =============================================================================

/// Saint-Venant-averaged cavity-wall mean radial displacement: mean
/// over `loaded_vertices` of `(|x_final[v]| - |rest[v]|)`. Mirrors
/// row 11's `cavity_wall_mean_radial_displacement` verbatim.
fn cavity_wall_mean(result: &SolveResult, loaded: &[VertexId]) -> f64 {
    let sum: f64 = loaded
        .iter()
        .map(|&v| {
            let rest = result.rest_positions[v as usize];
            let final_pos = Vec3::new(
                result.step.x_final[3 * v as usize],
                result.step.x_final[3 * v as usize + 1],
                result.step.x_final[3 * v as usize + 2],
            );
            final_pos.norm() - rest.norm()
        })
        .sum();
    sum / loaded.len() as f64
}

/// HEADLINE B — observed cavity-wall mean within `RADIAL_REL_TOL` of
/// single-material Lamé closed-form. Single-material gate signature
/// matches row 11's three-shell cavity gate verbatim.
fn verify_cavity_wall_lame(observed: f64, analytic: f64) {
    assert_relative_eq!(
        observed,
        analytic,
        max_relative = RADIAL_REL_TOL,
        epsilon = RADIAL_EPS_ABS_FLOOR,
    );
}

// =============================================================================
// 9. verify_captured_cavity_wall_mean_bits — cross-row continuity
// =============================================================================

/// Cross-row continuity guard: observed cavity-wall mean within IV-1
/// sparse-tier rel-tol of the row 11 captured bits. Bit-equal by
/// construction — same geometry, same `(μ, λ)`, same solver path —
/// but compared via `assert_relative_eq!` at `1e-12` to admit any
/// future cross-platform 3-ULP drift on faer's per-column FMA-fusion.
fn verify_captured_cavity_wall_mean_bits(observed: f64) {
    let expected = f64::from_bits(CAVITY_WALL_MEAN_REF_BITS);
    assert_relative_eq!(
        observed,
        expected,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
}

// =============================================================================
// 10. PLY emit — z-slab per-tet centroid cloud + radial_displacement scalar
// =============================================================================

/// Per-tet record: rest centroid (for z-slab filter and radial reference) +
/// deformed centroid (for amplified PLY position) + `radial_displacement`
/// scalar (TRUE physical magnitude).
struct TetRecord {
    rest_centroid: Vec3,
    deformed_centroid: Vec3,
    radial_displacement: f64,
}

/// Walk every tet and compute its (rest, deformed) centroids + radial
/// displacement. Mirrors row 11's `build_tet_records` verbatim, dropping
/// the multi-shell `shell_id` and `f_at_x_final` fields the single-
/// material variant doesn't need.
fn build_tet_records(mesh: &SdfMeshedTetMesh, result: &SolveResult) -> Vec<TetRecord> {
    let n_tets = mesh.n_tets();
    (0..n_tets as u32)
        .map(|tet_id| {
            let verts = mesh.tet_vertices(tet_id);
            let v = |i: usize| result.rest_positions[verts[i] as usize];
            let rest_centroid = (v(0) + v(1) + v(2) + v(3)) * 0.25;
            let cur = |i: usize| {
                let idx = verts[i] as usize;
                Vec3::new(
                    result.step.x_final[3 * idx],
                    result.step.x_final[3 * idx + 1],
                    result.step.x_final[3 * idx + 2],
                )
            };
            let deformed_centroid = (cur(0) + cur(1) + cur(2) + cur(3)) * 0.25;
            let radial_displacement = deformed_centroid.norm() - rest_centroid.norm();
            TetRecord {
                rest_centroid,
                deformed_centroid,
                radial_displacement,
            }
        })
        .collect()
}

/// Filter records to the `|rest_centroid.z| < CELL_SIZE / 2` z-slab —
/// the canonical row-8/9/11 visualization slab through the body's middle.
/// For a hollow shell at the row's geometry, the slab projects centroids
/// onto a 2-D annulus on z=0 with cavity-wall ring at `|p_xy| ≈ R_CAVITY`
/// and outer-wall ring at `|p_xy| ≈ R_OUTER`; cf-view reads as a clean
/// continuous radial-decay gradient from the cavity inward to outward.
fn zslab_records(records: &[TetRecord]) -> Vec<&TetRecord> {
    let half_slab = 0.5 * CELL_SIZE;
    records
        .iter()
        .filter(|r| r.rest_centroid.z.abs() < half_slab)
        .collect()
}

/// Emit z-slab per-tet centroid cloud as a vertices-only PLY with
/// `radial_displacement` per-vertex scalar (TRUE physical magnitude;
/// `verify_*` operates on unscaled solver outputs). Vertex positions
/// are amplified by `DISPLACEMENT_SCALE = 50.0` for cf-view readability.
/// Mirrors row 11's z-slab cf-view artifact pattern verbatim, single-
/// material variant (no `material_id` categorical scalar).
fn save_zslab_centroid_ply(zslab: &[&TetRecord], path: &Path) -> Result<()> {
    let mut vertices: Vec<Point3<f64>> = Vec::with_capacity(zslab.len());
    let mut radial_displacement: Vec<f32> = Vec::with_capacity(zslab.len());
    for record in zslab {
        let amplified = record.rest_centroid
            + DISPLACEMENT_SCALE * (record.deformed_centroid - record.rest_centroid);
        vertices.push(Point3::new(amplified.x, amplified.y, amplified.z));
        radial_displacement.push(record.radial_displacement as f32);
    }

    let faces: Vec<[u32; 3]> = Vec::new();
    let geometry = IndexedMesh::from_parts(vertices, faces);
    let mut attributed = AttributedMesh::new(geometry);
    attributed.insert_extra("radial_displacement", radial_displacement)?;
    save_ply_attributed(&attributed, path, true)?;
    Ok(())
}

// =============================================================================
// 11. print_summary — museum-plaque stdout
// =============================================================================

fn print_summary(
    n_tets: usize,
    n_vertices: usize,
    n_referenced: usize,
    n_pinned: usize,
    n_loaded: usize,
    n_zslab: usize,
    iter_count: usize,
    final_residual: f64,
    observed_cavity: f64,
    analytic_cavity: f64,
    path: &Path,
) {
    let rel_err = (observed_cavity - analytic_cavity).abs() / analytic_cavity.abs();
    let r_outer = LAYERED_SPHERE_R_OUTER;
    let r_cavity = LAYERED_SPHERE_R_CAVITY;
    let half = LAYERED_SPHERE_BBOX_HALF_EXTENT;
    println!("==== solid-to-sim-soft ====");
    println!();
    println!("input  : Solid::sphere({r_outer}).subtract(Solid::sphere({r_cavity}))");
    println!("         (typed cf_design::Solid CSG body — F1 impl Sdf for Solid)");
    println!("         dispatched via cf_design::Sdf trait into");
    println!("         SdfMeshedTetMesh::from_sdf(&dyn Sdf, &MeshingHints) — F3 re-export");
    println!("         hints: bbox = [-{half}, {half}]³, cell_size = {CELL_SIZE},");
    println!("                material_field = None  // skeleton_default = uniform(1e5, 4e5)");
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!("  bridge_equivalence         : equals_structurally + EXACT_TOL positions vs");
    println!("                               sim-soft DifferenceSdf<SphereSdf> baseline");
    println!("  counts_exact               : n_tets / n_vertices / n_referenced / pinned / loaded");
    println!("                               (cross-row continuity vs row 11)");
    println!("  quality_floors             : Theorem-1 signed_volume / aspect_ratio / dihedral");
    println!("  solver_converges           : iters < max_newton_iter, residual < tol");
    println!("  cavity_wall_lame           : observed within {RADIAL_REL_TOL} rel-err of analytic");
    println!(
        "  captured_cavity_wall_bits  : within {SPARSE_REL_TOL:e} of row 11's uniform-1× capture"
    );
    println!();
    println!("Mesh:");
    println!("  n_tets                     : {n_tets:>7}");
    println!("  n_vertices                 : {n_vertices:>7}");
    println!("  referenced (≤ n_vertices)  : {n_referenced:>7}");
    println!("  pinned   (outer-band)      : {n_pinned:>7}");
    println!("  loaded   (cavity-band)     : {n_loaded:>7}");
    println!();
    println!("Solver:");
    println!("  Newton iters               : {iter_count:>7}");
    println!("  final_residual_norm        : {final_residual:.3e}");
    println!();
    println!("Cavity-wall radial displacement (mean over loaded vertices):");
    println!("  observed (FEM, h/2)        : {observed_cavity:.6e} m");
    println!("  analytic (single-mat Lamé) : {analytic_cavity:.6e} m");
    println!("  rel-err |Δ| / |analytic|   : {rel_err:.4}");
    println!();
    println!("PLY    : {}", path.display());
    println!(
        "         z-slab per-tet centroid cloud ({n_zslab} pts; |centroid.z| < {half:.3} m;",
        half = 0.5 * CELL_SIZE,
    );
    println!("         DISPLACEMENT_SCALE = {DISPLACEMENT_SCALE}× amplification on positions)");
    println!("         + 1 per-vertex scalar:");
    println!(
        "           extras[\"radial_displacement\"] — TRUE physical magnitude (m), sequential"
    );
    println!("         open in cf-view, the workspace's unified visual-review viewer:");
    println!("           cargo run -p cf-viewer --release -- <path>");
    println!("         default-picks radial_displacement (sequential viridis, unipolar);");
    println!(
        "         z-slab projects centroids onto z=0 annulus — cavity ring (|p_xy| ~ R_CAVITY)"
    );
    println!("         to outer ring (|p_xy| ~ R_OUTER) with continuous radial-decay gradient.");
    println!(
        "         HEADLINE B loaded mean: {observed_cavity:.3e} m; analytic: {analytic_cavity:.3e} m."
    );
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    verify_geometry_invariants();

    // Bridge equivalence (HEADLINE A): same hints, two SDF surfaces.
    let hints = canonical_hints();
    let typed_solid = build_typed_solid();
    let baseline_diff = build_baseline_difference_sdf();
    let mesh_typed = mesh_via(&typed_solid, &hints);
    let mesh_baseline = mesh_via(&baseline_diff, &hints);
    verify_bridge_equivalence(&mesh_typed, &mesh_baseline);

    // Counts + quality floors on the typed mesh (the bridge-equivalence
    // anchor proves both meshes carry identical structure, so checking
    // one suffices).
    let referenced = referenced_vertices(&mesh_typed);
    let (bc, loaded, theta) = build_boundary_conditions(&mesh_typed, &referenced);
    verify_counts_exact(&mesh_typed, &referenced, &bc.pinned_vertices, &loaded);
    verify_quality_floors(&mesh_typed);

    // Capture the mesh handle for PLY emit before consuming into solver.
    let mesh_for_ply = mesh_typed.clone();

    // Solver run + convergence gate.
    let cfg = {
        let mut c = SolverConfig::skeleton();
        c.dt = STATIC_DT;
        c.max_newton_iter = MAX_NEWTON_ITER;
        c
    };
    let result = solve_static(mesh_typed, bc, &theta);
    verify_solver_converges(&result, &cfg);

    // HEADLINE B + cross-row continuity.
    let observed = cavity_wall_mean(&result, &loaded);
    let analytic = cavity_wall_lame_analytic(
        MU,
        LAMBDA,
        PRESSURE,
        LAYERED_SPHERE_R_CAVITY,
        LAYERED_SPHERE_R_OUTER,
    );
    verify_cavity_wall_lame(observed, analytic);
    verify_captured_cavity_wall_mean_bits(observed);

    // Build per-tet records (rest + deformed centroids + radial
    // displacement); filter to z-slab; emit cf-view PLY.
    let records = build_tet_records(&mesh_for_ply, &result);
    let zslab = zslab_records(&records);
    assert_eq!(
        zslab.len(),
        N_ZSLAB_TETS_EXACT,
        "z-slab tet count drift: got {}, expected {N_ZSLAB_TETS_EXACT}",
        zslab.len(),
    );

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let out_path = out_dir.join("shell_zslab.ply");
    save_zslab_centroid_ply(&zslab, &out_path)?;

    print_summary(
        mesh_for_ply.n_tets(),
        mesh_for_ply.n_vertices(),
        referenced.len(),
        N_PINNED_EXACT,
        loaded.len(),
        zslab.len(),
        result.step.iter_count,
        result.step.final_residual_norm,
        observed,
        analytic,
        &out_path,
    );

    Ok(())
}
