//! concentric-lame-shells — Phase 4 IV-5 user-facing wrap: three-shell
//! concentric hollow silicone sphere meshed via the BCC + Labelle-Shewchuk
//! pipeline, internal pressure on the cavity wall, fixed Dirichlet pin on
//! the outer surface, with closed-form piecewise-Lamé radial-displacement
//! comparison per shell.
//!
//! `SoftScene::layered_silicone_sphere(material_field, cell_size, pressure)`
//! is the production scene constructor (sim-soft `c3729d4a` per Phase 4
//! scope memo §1 IV-5 + §8 commit 11) — `DifferenceSdf` of two `SphereSdf`s
//! at radii `(R_OUTER, R_CAVITY) = (0.10, 0.04) m` (hollow shell body), with
//! a 3-shell `MaterialField` partitioning into inner Ecoflex / middle
//! carbon-black-composite / outer Ecoflex Lamé pairs per Decision J's
//! `1× / 2× / 1×` stiffness pattern (compressible regime, all in
//! `λ = 4 μ` ⇒ `ν = 0.4` per IV-3 + IV-5's `## Compressible Neo-Hookean
//! regime` deviation; near-incompressible Ecoflex `ν ≈ 0.49` is recovered
//! at Phase H Tet10 + F-bar). The cavity surface receives a per-vertex
//! radially outward force traction packed as `LoadAxis::FullVector`
//! (`f_v = pressure · n̂_v · A_v`, `A_v = 4π R_CAVITY² / N_loaded`); the
//! outer surface is fully Dirichlet-pinned (kills 6 rigid-body modes,
//! generalises Lamé closed-form to fixed-outer BC variant via
//! `u_r^{(3)}(R_b) = 0`). A single backward-Euler `replay_step` at
//! `cfg.dt = STATIC_DT = 1.0 s` collapses the inertial term `M / Δt²` by
//! ~4 orders of magnitude relative to stiffness — IV-5's static-regime
//! idiom verbatim — so Newton from rest converges to the static
//! equilibrium far below `tol = 1e-10`.
//!
//! The headline new capability vs row 10 is **three-shell hollow-body
//! integration** — multi-material (3 shells, not 2) + SDF-meshed (hollow
//! `DifferenceSdf` body, not hand-built) + non-vertex-force BC
//! (radially-outward distributed pressure traction, not axial tip-force) +
//! piecewise-Lamé closed-form analytic (3-shell composite, not
//! Euler-Bernoulli composite-beam). Row 11 generalises row 10's bilayer
//! cantilever-beam + EB-composite analytic to a 3-shell hollow sphere +
//! piecewise-Lamé composite analytic, AND generalises row 8's 3-shell
//! concentric scene from field-exposition (no solver) to solver-driven
//! (analytic comparison). It is the PR1 finale — the canonical IV-5
//! multi-material + SDF + hollow-body integration test, exposed as a
//! user-facing demo.
//!
//! ## Convergence gate for the per-shell Lamé comparison
//!
//! `EXAMPLE_INVENTORY.md` Tier 3 row 11 says "assert radial displacement
//! vs Lamé per shell" without a quantitative bound — unlike row 10's "to
//! 3 digits" wording, no inventory-recalibration is required here.
//! The operative gate per readout is `assert_relative_eq!(observed,
//! analytic, max_relative = 0.30, epsilon = 5.0e-6 m)` at `cell_size =
//! h/2 = 0.02` — mirroring IV-5's
//! `iv_5_uniform_passthrough_at_h2_matches_single_shell_lame` sanity-
//! band rel-tol with an absolute floor that absorbs the small-magnitude
//! outer-shell mean (cavity / inner / middle readouts pass via the rel-
//! tol branch at `~14 % / ~12 % / ~19 %` rel-err observed; outer-shell
//! readout passes via the eps-floor branch at `~36 %` rel-err but
//! absolute diff `~3.7e-6 m < 5e-6 m` floor — the outer shell's `~952`
//! referenced vertices include `734` pinned-at-zero outer-wall vertices
//! by the fixed-outer BC, so the mean's small magnitude makes BCC
//! quantization a meaningful fraction of the rel-err denominator). See
//! README §"Tet4 caveat" for the convergence story (IV-5 documents
//! super-quadratic empirical rate at the fine end with a conservative
//! `< 0.20` absolute floor at h/4; the user-facing example at h/2 lives
//! a refinement up that ladder).
//!
//! ## cf-view artifact
//!
//! Per-tet centroid point cloud of the deformed configuration with
//! `DISPLACEMENT_SCALE = 50.0` geometric amplification on vertex
//! positions (`vertex = rest + SCALE * (deformed - rest)`) —
//! visualisation-only, the `radial_displacement` per-vertex scalar
//! carries the TRUE physical displacement and every `verify_*` operates
//! on the unscaled solver outputs (mirrors row 10's `DISPLACEMENT_SCALE
//! = 20.0` precedent at smaller-strain regime — row 11's observed
//! cavity-wall inflation is `~0.71 %` of `R_CAVITY` (`~0.84 %` analytic),
//! smaller than row 10's `~2.2 %` of `L`; `50×` amplifier puts visible
//! cavity-wall inflation at `~36 %` of `R_CAVITY` — dramatically visible
//! without distorting the spherical-symmetry readability). The cloud is
//! filtered to a thin `|centroid.z| < cell_size/2` z-slab cut (mirrors
//! row 8 + 9 z-slab pattern; row 10 used y-slab adapted to the
//! cantilever-beam axial geometry — row 11 reverts to the row-8 z-slab
//! for the spherically-symmetric body since radial expansion is
//! azimuthally symmetric and projects cleanly to concentric color rings
//! on the z=0 plane). Two per-vertex scalars: the categorical
//! `material_id` (0 / 1 / 2 → tab10 binary highlight of inner / middle /
//! outer shells), and the continuous `radial_displacement` (sequential
//! viridis on `[0, ~5e-4]`, showing the radial-inflation profile, TRUE
//! physical magnitude). cf-view's alphabetical-first pick lands on
//! `material_id` — the 3 sharp shells in 3 colors is the loud first
//! impression on launch; user dropdowns to `radial_displacement` to see
//! the continuous radial-decay gradient from cavity wall to outer wall.
//!
//! ## Anchor groups (all assertions exit-0 on success)
//!
//! - **`geometry_invariants`** — compile-time `const { assert!(...) }`
//!   on `0 < R_CAVITY < R_INNER_OUTER < R_OUTER_INNER < R_OUTER`,
//!   `BBOX_HALF_EXTENT > R_OUTER`, `PRESSURE > 0`, `MU_INNER == MU_OUTER
//!   < MU_MIDDLE` (Decision J 1×/2×/1× symmetry + ordering),
//!   `LAMBDA_X = 4 · MU_X` per ν=0.4. Re-asserts the
//!   `layered_silicone_sphere` constructor's runtime panic invariants at
//!   the user-facing example layer (compile-time enforcement on geometry
//!   constants).
//! - **`mesh_topology_exact`** — `mesh.n_tets`, `mesh.n_vertices`,
//!   `referenced_vertices(&mesh).len()` exact-pinned per the III-1
//!   determinism contract at `cell_size = h/2 = 0.02` on the canonical
//!   `DifferenceSdf` hollow-shell body.
//! - **`boundary_partition`** — `bc.pinned_vertices` (outer-surface band)
//!   and `bc.loaded_vertices` (cavity-surface band) cardinality
//!   exact-pinned, ascending-order, disjoint; every `bc.loaded_vertices`
//!   entry uses `LoadAxis::FullVector` (radially-outward pressure
//!   traction is per-component, not axis-aligned).
//! - **`per_tet_material_assignment`** — for every tet,
//!   `mesh.materials()[t]` probed via `energy(F_probe) +
//!   first_piola(F_probe)` bit-equal vs `expected = NH(MU_X, LAMBDA_X)`
//!   with `X` selected by `shell_at(rest_centroid)` under the
//!   `LayeredScalarField`'s `partition_point(|&t| t <= phi)` rule
//!   (closed-left, open-right boundary convention). Per-shell tet counts
//!   exact-pinned. HEADLINE A — the canonical IV-5 cross-impl gate
//!   (per-tet material assignment matches the SDF predicate's analytic
//!   re-derivation), exposed user-facing at h/2 production-scale mesh
//!   resolution.
//! - **`solver_converges`** — `iter_count < cfg.max_newton_iter = 50`,
//!   `final_residual_norm < cfg.tol = 1e-10`, AND per-tet
//!   `max|σᵢ - 1| < 1.0` at converged `x_final` (NH validity-domain
//!   sanity at the deformed configuration; the inflation cavity at
//!   `pressure = 5e3 Pa` lands well inside `RequireOrientation` regime
//!   with global `max|σ-1| < 0.05` small-strain sanity).
//! - **`radial_displacement_per_shell_matches_lame_within_30pct`** —
//!   four Saint-Venant-averaged radial-displacement readouts. (1) The
//!   cavity-wall mean over `bc.loaded_vertices` of the per-vertex norm-
//!   difference `|x_final| - |rest|`, mirroring IV-5's
//!   `tests/concentric_lame_shells.rs:run_at_refinement` cavity-wall
//!   reading verbatim. (2-4) The inner / middle / outer shell means over
//!   all referenced vertices in each shell (`shell_at` partition) of the
//!   same scalar; the analytic mean `coeffs.u_r(shell_id, |rest|)` is
//!   computed over the SAME vertex set so quantization noise affects
//!   both sides equally. Each readout gated `assert_relative_eq!(observed,
//!   analytic, max_relative = 0.30, epsilon = 5.0e-6)`. The `5e-6` eps
//!   absolute floor absorbs the small-magnitude outer-shell mean's BCC-
//!   quantization; the `0.30` rel-tol is binding for cavity / inner /
//!   middle readouts. HEADLINE B — the inventory's named gate ("assert
//!   radial displacement vs Lamé per shell"), generalising IV-5's
//!   cavity-wall-only reading to a 3-shell profile gate while preserving
//!   the IV-5 cross-reference at the cavity-wall readout.
//! - **`cavity_wall_three_shell_strictly_between_uniform_bounds`** —
//!   IV-2 lens β analog at the cavity-wall mean (mirrors row 10 banked
//!   pattern (d) on tip displacement, here adapted to hollow-body cavity-
//!   wall mean). Three solver runs: three-shell + uniform-1× baseline +
//!   uniform-2× baseline. Strict-between inequality
//!   `u_r_uniform_2x < u_r_three_shell < u_r_uniform_1x` per the
//!   compliance-composition direction (uniform-2× = all-stiff = smallest
//!   inflation; uniform-1× = all-soft = largest; three-shell intermediate).
//!   HEADLINE C — catches mis-assigned bugs that anchor 6's `< 0.30`
//!   rel-err gate could miss in the 17-30% downward-bias range.
//! - **`radial_monotonicity_outward`** — strict ordering across the 4
//!   three-shell readouts: `cavity > inner_shell > middle_shell >
//!   outer_shell ≥ 0`. Sanity guard against load-direction or BC-sign
//!   regressions; the piecewise-Lamé closed-form predicts strict outward
//!   monotone-decay under internal-pressure-with-fixed-outer geometry
//!   (cavity wall most inflated, outer wall pinned at 0).
//! - **`captured_bits_radial_displacements`** — six radial-displacement
//!   means captured under the IV-1 sparse-tier rel-tol contract
//!   (`assert_relative_eq!` at `1e-12` rel, NOT strict `to_bits`
//!   equality): four from the three-shell run (cavity-wall plus three
//!   per-shell) and two from the HEADLINE C uniform-baseline runs
//!   (uniform-1× cavity, uniform-2× cavity). ~6.5k tets through faer's
//!   sparse Cholesky lives between IV-1's dense bit-equal tier (12-24
//!   DOFs) and IV-1's sparse-at-scale tier (~3k tets); the rel-tol bar
//!   reserves slack for cross-platform robustness while catching real
//!   regressions.
//! - **`material_populations_exact`** — per-shell tet counts (full body,
//!   not z-slab subset) exact-pinned per the III-1 determinism contract;
//!   sum partitions over `N_TETS_EXACT` exactly.
//! - **`zslab_visual_populations_exact`** — per-shell tet counts in the
//!   `|centroid.z| < cell_size/2` z-slab cut that drives the cf-view PLY
//!   artifact. Visual-pedagogy guard: each shell must have ≥ 1 z-slab
//!   centroid for the three concentric-ring cf-view rendering to work;
//!   per-shell counts also exact-pinned.

// PLY field-data is single-precision on disk; converting f64 quantities
// (material_id 0.0/1.0/2.0, radial_displacement) to f32 for the
// AttributedMesh extras is intrinsic to the PLY format. Same precedent as
// rows 1+2+3+8+9+10.
#![allow(clippy::cast_possible_truncation)]
// `usize as u32` casts on `mesh.n_tets()` (max ~7k here, ≪ u32::MAX) — the
// standard Mesh-trait API tax mirrored across the workspace (rows 8 + 9 +
// 10).
#![allow(clippy::cast_possible_wrap)]
// `try_inverse().expect(...)` on `J_0` for the canonical mesh: the BCC +
// Labelle-Shewchuk pipeline produces non-degenerate tets by construction.
// Same precedent as rows 6 + 8 + 10.
#![allow(clippy::expect_used)]
// `usize as f64` cast on `loaded.len()` / per-shell counts for mean
// computations. Counts ≤ ~7k here, well within f64 mantissa exact range.
// Same allowance as rows 6 + 8 + 9 + 10.
#![allow(clippy::cast_precision_loss)]
// `doc_markdown` flags Unicode math notation (`σᵢ`, `λ`, `μ`) as if they
// were unbacktrick-quoted code identifiers. Same allowance as rows 5 + 6
// + 8 + 9 + 10.
#![allow(clippy::doc_markdown)]
// `print_summary` is a single museum-plaque stdout writer; splitting into
// sub-helpers fragments the visual format without information gain. Same
// allowance as rows 4 + 5 + 6 + 9 + 10.
#![allow(clippy::too_many_lines)]
// `print_summary` aggregates many scalars-and-collections into the
// museum-plaque format; threading them through a struct adds indirection
// without information gain (same trade-off `print_summary` already faces
// for `too_many_lines`). Same allowance as row 10.
#![allow(clippy::too_many_arguments)]
// Domain-meaningful naming: `mu_inner`/`mu_middle`/`mu_outer` distinguish
// the per-shell Lamé pairs in `verify_per_tet_material_assignment`;
// `inner_mean`/`middle_mean`/`outer_mean` distinguish per-shell readouts.
// Same precedent as rows 6 + 10.
#![allow(clippy::similar_names)]
// `panic!(...)` calls in helper functions document malformed-mesh
// structural-invariant failures. Same precedent as row 10.
#![allow(clippy::panic)]
// Manual `as f64` casts in tet-count division for shell-mean computation.
#![allow(clippy::cast_sign_loss)]

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply_attributed;
use mesh_types::{AttributedMesh, IndexedMesh, Point3};
use nalgebra::{Matrix3, Matrix6, Vector6};
use sim_soft::{
    CpuNewtonSolver, CpuTet4NHSolver, Field, InversionHandling, LAYERED_SPHERE_BBOX_HALF_EXTENT,
    LAYERED_SPHERE_R_CAVITY, LAYERED_SPHERE_R_INNER_OUTER, LAYERED_SPHERE_R_OUTER,
    LAYERED_SPHERE_R_OUTER_INNER, LayeredScalarField, LoadAxis, Material, MaterialField, Mesh,
    NeoHookean, NewtonStep, NullContact, SceneInitial, SdfMeshedTetMesh, SoftScene, Solver,
    SolverConfig, SphereSdf, Tet4, Vec3, VertexId, referenced_vertices,
};

// =============================================================================
// Constants — material parameters (mirror IV-5 verbatim, Decision J)
// =============================================================================

/// Outer Ecoflex 00-30 baseline `μ` (Pa). Compressible Lamé pair
/// (`λ = 4 μ` ⇒ `ν = 0.4`) per the module docstring's "Compressible
/// Neo-Hookean regime" section + IV-5's `MU_OUTER`. Same baseline as IV-1
/// / IV-2 / IV-3 / row 10's region A.
const MU_OUTER: f64 = 1.0e5;
const LAMBDA_OUTER: f64 = 4.0e5;

/// Middle carbon-black composite shell `μ`: `2×` stiffness multiplier per
/// Decision J (Part 1 §04 §02). Same `ν = 0.4` (both Lamé parameters
/// scale together). Mirrors IV-5's `MU_MIDDLE`.
const MU_MIDDLE: f64 = 2.0e5;
const LAMBDA_MIDDLE: f64 = 8.0e5;

/// Inner Ecoflex 00-30 — same baseline as outer per Decision J's
/// outer / middle / inner = Ecoflex / composite / Ecoflex symmetry
/// commitment. Mirrors IV-5's `MU_INNER`.
const MU_INNER: f64 = 1.0e5;
const LAMBDA_INNER: f64 = 4.0e5;

// =============================================================================
// Constants — pressure + refinement + solver config
// =============================================================================

/// Internal pressure on the cavity wall (Pa). Mirrors IV-5's `PRESSURE`
/// verbatim — engineered to land cavity-wall inflation in the
/// small-strain band (`< ~3 %`) where Neo-Hookean reduces to Lamé. At
/// the chosen radii the piecewise-Lamé closed-form (6×6 LU) predicts
/// cavity-wall radial displacement `u_r(R_cavity) ≈ 3.342e-4 m` ≈
/// `~0.84 %` of `R_CAVITY = 0.04 m` (FEM observed at h/2 = `2.857e-4 m`
/// ≈ `~0.71 %`, `~14.5 %` rel-err vs analytic per IV-5 sanity-band
/// convergence). Note: IV-5's `PRESSURE` constant docstring at
/// `tests/concentric_lame_shells.rs:289-293` quotes `~4.6e-4 m` for the
/// cavity-wall analytic — that's a stale pre-implementation estimate;
/// the actual analytic at the row's constants is `3.342e-4 m` per the
/// LU solve.
const PRESSURE: f64 = 5.0e3;

/// BCC lattice spacing (m). `cell_size = h/2 = 0.02` — IV-5's
/// canonical Phase 3 / IV-4 mid-refinement, the same cell size row 8
/// uses (the canonical user-facing example refinement for SDF-meshed
/// scenes). IV-5's main convergence test runs three levels h, h/2, h/4;
/// row 11 ships at the middle level mirroring row 10's choice of IV-3
/// sanity-test refinement.
const CELL_SIZE: f64 = 0.02;

/// Static-regime time step (s). Mirrors IV-5's `STATIC_DT` and IV-3's
/// `STATIC_DT` verbatim: at large `dt`, the backward-Euler residual's
/// inertial term `M / dt² · (x − x_prev)` collapses to negligible vs.
/// the stiffness contribution, so a single `replay_step` from rest
/// converges to the static equilibrium far below `tol = 1e-10`.
const STATIC_DT: f64 = 1.0;

/// Newton iter cap. Mirrors IV-5 verbatim: empirically Newton converges
/// in `~3` iters per level under the radially-symmetric distributed-
/// pressure load with fixed-outer-pin BC (linearisable small-strain
/// regime from rest, no rotations, no tip-force concentration). `50`
/// leaves headroom against future load / material perturbations.
const MAX_NEWTON_ITER: usize = 50;

// =============================================================================
// Constants — Lamé closed-form + per-tet probe + tolerance
// =============================================================================

/// Probe stretch for the Material-trait value-correctness gate.
/// `F_probe = diag(PROBE_LAMBDA, 1, 1)` ties to row 6's `LAMBDA_STRETCH`
/// and rows 8 + 9 + 10's `PROBE_LAMBDA` for cross-row continuity.
const PROBE_LAMBDA: f64 = 1.20;

/// Bit-equal tolerance for the Material-trait probe. Both the test-side
/// `expected` and the mesher-side `mesh.materials()[t]` run identical
/// `NeoHookean::first_piola` / `energy` arithmetic on identical
/// `(μ, λ)` pairs and identical `F_probe` — outputs are bit-equal by
/// construction on a fixed toolchain. `epsilon = 0.0` satisfies clippy's
/// `float_cmp` lint without actually relaxing the bound (mirrors rows
/// 8 + 9 + 10's `EXACT_TOL = 0.0`).
const EXACT_TOL: f64 = 0.0;

/// IV-1 sparse-tier rel-tol for captured radial-displacement bits + the
/// per-shell radial-displacement-vs-Lamé gate. ~7k tets through faer's
/// sparse Cholesky lives between IV-1's dense bit-equal tier (12-24 DOFs)
/// and IV-1's sparse-at-scale tier (~3k tets, 3-ULP cross-platform drift
/// on faer's per-column FMA-fusion path). `1e-12` admits sparse-solver
/// SIMD/FMA noise while catching any real regression. Same precedent as
/// rows 6 + 10.
const SPARSE_REL_TOL: f64 = 1.0e-12;

/// Absolute floor for relative comparisons that touch zero. `1e-12` is
/// below typical radial-displacement magnitudes (cavity-wall ~3e-4 m,
/// outer-shell-mean ~7e-6 m) by 6-8 orders of magnitude. Same precedent
/// as rows 6 + 10.
const SPARSE_EPS_ABS: f64 = 1.0e-12;

/// Per-shell rel-err tolerance for the Lamé closed-form comparison.
/// Mirrors IV-5's `iv_5_uniform_passthrough_at_h2_matches_single_shell_lame`
/// sanity-band gate (`< 0.30` rel-err on cavity-wall reading at h/2).
/// See README §"Tet4 caveat" for the convergence story (IV-5's main
/// convergence test ships at `< 0.20` absolute floor at h/4; the
/// user-facing example at h/2 lives a refinement up that ladder).
const RADIAL_REL_TOL: f64 = 0.30;

/// Absolute floor for the per-shell radial-displacement gate — absorbs
/// the small-magnitude outer-shell mean's BCC-quantization noise. The
/// outer shell's referenced-vertex mean is `~6.7e-6 m` (small magnitude
/// because 77% of outer-shell vertices are pinned at `u_r = 0` by the
/// fixed-outer-surface BC, plus the non-pinned outer-shell-interior
/// vertices have `u_r` decaying toward 0 at `r ≈ R_OUTER`); at h/2 the
/// observed-vs-analytic abs diff is `~3.7e-6 m` (~35% rel-err). The
/// `5e-6 m` floor absorbs this case via `assert_relative_eq!`'s
/// `epsilon` branch (`abs_diff <= max(eps, rel_tol * max(|obs|, |ana|))`),
/// while the `0.30` `max_relative` gate stays binding for the three
/// larger-magnitude readouts (cavity ~3e-4 m, inner ~2e-4 m, middle
/// ~6e-5 m). Headroom over observed outer abs diff is `~1.3e-6 m`
/// (~35% margin); IV-1 sparse-tier 3-ULP cross-platform drift is far
/// below that scale at sparse-mesh magnitudes.
const RADIAL_EPS_ABS_FLOOR: f64 = 5.0e-6;

// =============================================================================
// Constants — geometry derived (re-exports, phi thresholds, bbox)
// =============================================================================

/// Inner-shell threshold on `phi = ‖p‖ − R_OUTER`. `phi < PHI_INNER_THRESHOLD`
/// (i.e., `‖p‖ < R_INNER_OUTER`) ⇒ inner shell (closed-left at R_CAVITY,
/// open-right at R_INNER_OUTER per the
/// `partition_point(|&t| t <= phi)` rule — at `phi == PHI_INNER_THRESHOLD`
/// exactly, the point lands in the MIDDLE shell relative to this
/// threshold). Mirrors row 8's + IV-5's threshold construction.
const PHI_INNER_THRESHOLD: f64 = LAYERED_SPHERE_R_INNER_OUTER - LAYERED_SPHERE_R_OUTER;

/// Middle-shell threshold on `phi`. `PHI_INNER_THRESHOLD ≤ phi <
/// PHI_MIDDLE_THRESHOLD` ⇒ middle shell (closed-left at R_INNER_OUTER,
/// open-right at R_OUTER_INNER); `phi ≥ PHI_MIDDLE_THRESHOLD` ⇒ outer
/// shell.
const PHI_MIDDLE_THRESHOLD: f64 = LAYERED_SPHERE_R_OUTER_INNER - LAYERED_SPHERE_R_OUTER;

/// Visualisation-only geometric displacement scale factor — applied to
/// PLY vertex positions ONLY (`vertex = rest + SCALE * (deformed -
/// rest)`), NOT to the `radial_displacement` per-vertex scalar (which
/// continues to carry the TRUE physical displacement) or to any
/// numerical assertion (every `verify_*` operates on the unscaled solver
/// outputs). Standard FEM-visualisation trick: at small-strain regime
/// (observed cavity-wall inflation `~0.71 %` of `R_CAVITY`, `~0.84 %`
/// analytic), the geometric inflation arc is honest-but-subtle at
/// default cf-view orbit; a `50×` amplifier puts the visible cavity-wall
/// inflation at `~36 %` of `R_CAVITY`, dramatically visible without
/// distorting the spherical-symmetry readability — the inflation is
/// exaggerated but the body still reads as a hollow shell. Same trade-off
/// framing as row 10's `DISPLACEMENT_SCALE = 20.0` (smaller-strain row 11
/// → larger amplifier).
const DISPLACEMENT_SCALE: f64 = 50.0;

// =============================================================================
// Exact-pinned counts (III-1 determinism contract)
// =============================================================================

/// Total tet count at `cell_size = h/2 = 0.02` on the `DifferenceSdf`
/// hollow-shell body. Captured 2026-05-05 at sim-soft `dev` (post-row-10
/// tip `6f84c9cf`, post-N+3 row-10 docs), rustc 1.95.0 (`59807616e`
/// 2026-04-14) on macOS arm64 — same toolchain + platform as IV-1's
/// reference capture per
/// `invariant_iv_1_uniform_passthrough.rs:138-151`. Bit-exact match with
/// IV-5's three-shell convergence test at h/2 (`6456` per IV-5
/// `eprintln!`).
const N_TETS_EXACT: usize = 6456;

/// Total mesh vertex count, including BCC lattice corners not referenced
/// by any tet (orphans).
const N_VERTICES_EXACT: usize = 4682;

/// Vertices referenced by at least one tet. `N_VERTICES_EXACT -
/// N_REFERENCED_EXACT = 3202` orphan BCC lattice corners excluded from
/// solver participation.
const N_REFERENCED_EXACT: usize = 1480;

/// Outer-surface-band pinned vertex count (every vertex with
/// `(‖p‖ - R_OUTER).abs() < cell_size/2 = 0.01` filtered to referenced
/// set). Bit-exact match with IV-5's `n_pinned = 734`.
const N_PINNED_EXACT: usize = 734;

/// Cavity-surface-band loaded vertex count (every vertex with
/// `(‖p‖ - R_CAVITY).abs() < cell_size/2 = 0.01` filtered to referenced
/// set). Bit-exact match with IV-5's `n_loaded = 134`.
const N_LOADED_EXACT: usize = 134;

/// Per-shell tet counts. `INNER + MIDDLE + OUTER == N_TETS_EXACT` by
/// construction (no tet sits outside all three buckets — the body is a
/// `DifferenceSdf` of two `SphereSdf`s with material partition covering
/// `[R_CAVITY, R_OUTER]` in three concentric shells).
const N_INNER_TETS_EXACT: usize = 1032;
const N_MIDDLE_TETS_EXACT: usize = 1800;
const N_OUTER_TETS_EXACT: usize = 3624;

/// Per-shell tet counts in the `|centroid.z| < cell_size/2 = 0.01`
/// z-slab cut for the cf-view PLY artifact (~10% of full body, mirroring
/// row 8's z-slab fraction).
const N_INNER_TETS_ZSLAB_EXACT: usize = 184;
const N_MIDDLE_TETS_ZSLAB_EXACT: usize = 176;
const N_OUTER_TETS_ZSLAB_EXACT: usize = 256;

// ── Captured radial-displacement bits (IV-1 sparse-tier contract) ───────
//
// **Capture provenance** — captured 2026-05-05 at sim-soft `dev`
// (post-row-10 tip `6f84c9cf`, post-N+3 row-10 docs), rustc 1.95.0
// (`59807616e` 2026-04-14) — the same toolchain IV-1 captured at
// sim-soft `c3729d4a` per
// `invariant_iv_1_uniform_passthrough.rs:138-151` — on macOS arm64.
//
// **IV-1 sparse-tier contract.** This row's ~6.5k-tet FEM solve through
// faer's sparse Cholesky lives between IV-1's dense bit-equal tier
// (12-24 DOFs, `nalgebra::Matrix3` scalar arithmetic, bit-equal across
// rustc minor versions AND across `(macOS arm64, Linux x86_64)`) and
// IV-1's sparse-at-scale tier (~3k tets, 3-ULP cross-platform drift on
// faer's per-column FMA-fusion path). 6456 tets is past IV-1's
// sparse-at-scale tier; the contract here is **relative tolerance, not
// strict bit-equality** — observed bits captured for regression
// detection, compared via `assert_relative_eq!` at `1e-12` rel.
//
// **Failure-mode protocol** (mirrors IV-1's): if the rel-tol comparison
// fails, do NOT re-bake. Diagnose in this order:
//   1. Rule out toolchain drift (rustc / LLVM / libm minor version delta
//      vs the rustc 1.95.0 capture).
//   2. If same toolchain, real regression — identify which sim-soft
//      commit altered the `SoftScene::layered_silicone_sphere`
//      constructor OR the SDF-meshed FEM assembly path through faer.
//   3. NEVER re-bake the reference values to make the test green.

/// Cavity-wall mean radial displacement bits (m). Mirrors IV-5's
/// `tests/concentric_lame_shells.rs:run_at_refinement` cavity-wall
/// reading verbatim — mean over `bc.loaded_vertices` of
/// `(|x_final[v]| - |rest_pos[v]|)`.
/// `f64::from_bits(0x3f32_b990_b16c_03c2) ≈ 2.857_187_513_854_408_641e-4`.
/// Bit-exact match with IV-5's three-shell-h/2 cavity-wall reading per
/// the `iv_5_three_shell_converges_to_piecewise_lame` `eprintln!`.
/// Rel-err vs piecewise-Lamé analytic `≈ 3.342e-4 m` is `~14.5 %` —
/// well under the `RADIAL_REL_TOL = 0.30` gate; mirrors IV-5's empirical
/// Tet4 + SDF-meshed three-shell convergence at h/2.
const CAVITY_WALL_MEAN_REF_BITS: u64 = 0x3f32_b990_b16c_03c2;

/// Inner-shell mean radial displacement bits (m). Mean over all 226
/// referenced vertices in inner shell (`shell_at(rest_pos[v]) == 0`) of
/// `(|x_final[v]| - |rest_pos[v]|)`.
/// `f64::from_bits(0x3f2b_ebf4_18dc_5f63) ≈ 2.130_256_147_487_809_748e-4`.
/// Rel-err vs same-vertex-set analytic mean `~12.0 %` (`< 0.30` gate).
const INNER_SHELL_MEAN_REF_BITS: u64 = 0x3f2b_ebf4_18dc_5f63;

/// Middle-shell mean radial displacement bits (m). Mean over all 302
/// referenced vertices in middle shell.
/// `f64::from_bits(0x3f0f_79e1_9a3e_5ec8) ≈ 6.003_589_376_673_900_769e-5`.
/// Rel-err vs same-vertex-set analytic mean `~18.9 %` (`< 0.30` gate).
const MIDDLE_SHELL_MEAN_REF_BITS: u64 = 0x3f0f_79e1_9a3e_5ec8;

/// Outer-shell mean radial displacement bits (m). Mean over all 952
/// referenced vertices in outer shell (734 of which are pinned at
/// `u_r = 0` by the fixed-outer-surface BC, so the mean is dominated by
/// the 218 outer-shell-interior vertices' small radial displacement).
/// `f64::from_bits(0x3edc_33bf_bc93_1078) ≈ 6.723_915_199_905_690_701e-6`.
/// Rel-err vs same-vertex-set analytic mean `~35.6 %`; the
/// `RADIAL_REL_TOL = 0.30` gate is absorbed by `RADIAL_EPS_ABS_FLOOR =
/// 5e-6` for this small-magnitude case (observed abs diff `~3.7e-6 m`,
/// floor headroom `~1.3e-6 m`).
const OUTER_SHELL_MEAN_REF_BITS: u64 = 0x3edc_33bf_bc93_1078;

/// Uniform-1× cavity-wall mean radial displacement bits (m). Captured
/// from the homogeneous-MU_INNER baseline run; the softest configuration
/// of the row 11 stiffness family. Three-shell cavity-wall mean is
/// strictly less than this per HEADLINE C between-bounds gate.
/// `f64::from_bits(0x3f35_43f1_0b56_98c8) ≈ 3.244_842_040_096_671_161e-4`.
/// Empirically `~13.6 %` larger than three-shell cavity (3.245e-4 vs
/// 2.857e-4) — the upper bound on the row-11 between-bounds gate.
const CAVITY_WALL_UNIFORM_1X_REF_BITS: u64 = 0x3f35_43f1_0b56_98c8;

/// Uniform-2× cavity-wall mean radial displacement bits (m). Captured
/// from the homogeneous-MU_MIDDLE baseline run; the stiffest
/// configuration of the row 11 stiffness family. Three-shell cavity-
/// wall mean is strictly greater than this per HEADLINE C between-bounds
/// gate. By linearity of the 6×6 closed-form in `(μ, λ)` (uniform-2× has
/// every shell at exactly 2× the uniform-1× pair), uniform-2× cavity is
/// `~half` of uniform-1× cavity at the FEM level too (observed 1.626e-4
/// vs uniform-1×'s 3.245e-4 — `~0.25 %` rel diff from exact-half, clean
/// FP noise on the linearity prediction).
/// `f64::from_bits(0x3f25_510d_7bf6_ab86) ≈ 1.626_328_430_409_351_949e-4`.
const CAVITY_WALL_UNIFORM_2X_REF_BITS: u64 = 0x3f25_510d_7bf6_ab86;

// =============================================================================
// Shell partition (mirrors `LayeredScalarField`'s
// `partition_point(|&t| t <= phi)` rule — same as IV-5's three_shell_field
// + row 8's shell_at)
// =============================================================================

/// Layer-ID for a single point under the partition rule
/// `LayeredScalarField` uses internally
/// (`partition_point(|&t| t <= phi)` ⇒ at exactly `phi == threshold[i]`
/// the point lands in the *outer* shell, `values[i + 1]`). Returns
/// `0` for inner, `1` for middle, `2` for outer — matching the `values`
/// slot indexing of the `LayeredScalarField` constructed in
/// [`three_shell_field`]. Test-side reimplementation of the partition
/// rule provides the cross-implementation gate at
/// [`verify_per_tet_material_assignment`]: any drift between this
/// if-else and the mesher's `partition_point` walk fires loud at runtime.
fn shell_at(p: Vec3) -> usize {
    let phi = p.norm() - LAYERED_SPHERE_R_OUTER;
    if phi < PHI_INNER_THRESHOLD {
        0
    } else if phi < PHI_MIDDLE_THRESHOLD {
        1
    } else {
        2
    }
}

// =============================================================================
// MaterialField builder (mirrors IV-5's three_shell_field verbatim)
// =============================================================================

/// Three-shell `MaterialField` per Decision J: outer Ecoflex (`1×`) /
/// middle composite (`2×`) / inner Ecoflex (`1×`). Mirrors IV-5's
/// `tests/concentric_lame_shells.rs:three_shell_field` verbatim — same
/// SDF reuse-as-partition trick, same
/// `partition_point(|&t| t <= phi)` boundary convention.
fn three_shell_field() -> MaterialField {
    let body = || {
        Box::new(SphereSdf {
            radius: LAYERED_SPHERE_R_OUTER,
        })
    };
    let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        body(),
        vec![PHI_INNER_THRESHOLD, PHI_MIDDLE_THRESHOLD],
        vec![MU_INNER, MU_MIDDLE, MU_OUTER],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        body(),
        vec![PHI_INNER_THRESHOLD, PHI_MIDDLE_THRESHOLD],
        vec![LAMBDA_INNER, LAMBDA_MIDDLE, LAMBDA_OUTER],
    ));
    MaterialField::from_fields(mu_field, lambda_field)
}

/// Uniform-1× `MaterialField` — every tet at the inner / outer Ecoflex
/// baseline `(MU_INNER, LAMBDA_INNER) = (1e5, 4e5) Pa`. The "softest"
/// homogeneous configuration of the row 11 stiffness family: the
/// three-shell cavity-wall mean is bounded above by uniform-1×'s per
/// the IV-2 lens β analog at HEADLINE C.
fn uniform_1x_field() -> MaterialField {
    MaterialField::uniform(MU_INNER, LAMBDA_INNER)
}

/// Uniform-2× `MaterialField` — every tet at the middle composite's
/// stiffness `(MU_MIDDLE, LAMBDA_MIDDLE) = (2e5, 8e5) Pa`. The "stiffest"
/// homogeneous configuration of the row 11 stiffness family: the
/// three-shell cavity-wall mean is bounded below by uniform-2×'s per
/// the IV-2 lens β analog at HEADLINE C.
fn uniform_2x_field() -> MaterialField {
    MaterialField::uniform(MU_MIDDLE, LAMBDA_MIDDLE)
}

// =============================================================================
// Closed-form Lamé multi-shell solver (inlined from IV-5)
// =============================================================================
//
// Mirrors IV-5's `tests/concentric_lame_shells.rs:solve_three_shell_lame`
// + `LameCoefficients::u_r` verbatim. See IV-5 module docstring's
// "Closed-form three-shell Lamé under internal pressure with fixed
// outer surface" section for the 6×6 derivation; the inlining here
// replicates the test-side helper at the user-facing example layer
// (same precedent as row 10 inlining IV-3's `eb_composite_tip_displacement`).

/// Per-shell parameter pair `(μ, λ)`. The closed-form solver below
/// indexes shells inner-to-outer.
#[derive(Clone, Copy)]
struct ShellParams {
    mu: f64,
    lambda: f64,
}

impl ShellParams {
    /// `K = 3 λ + 2 μ` — the radial-stress coefficient on `A_i` per
    /// shell.
    fn k_coefficient(self) -> f64 {
        3.0_f64.mul_add(self.lambda, 2.0 * self.mu)
    }
}

/// Closed-form multi-shell `(A_i, B_i)` coefficients for the radial-
/// displacement form `u_r^{(i)}(r) = A_i r + B_i / r²`.
///
/// Six unknowns laid out as `[A_1, B_1, A_2, B_2, A_3, B_3]`. See IV-5
/// module docstring for the 6×6 system derivation.
struct LameCoefficients {
    a: [f64; 3],
    b: [f64; 3],
}

impl LameCoefficients {
    /// Evaluate `u_r^{(i)}(r) = A_i r + B_i / r²`.
    fn u_r(&self, shell_index: usize, r: f64) -> f64 {
        self.a[shell_index].mul_add(r, self.b[shell_index] / (r * r))
    }
}

/// Solve the 6×6 closed-form system for the three-shell hollow sphere
/// under internal pressure `p` at `R_a` with **fixed outer surface**
/// (`u_r^{(3)}(R_b) = 0`) and bonded continuity at `R_1` and `R_2`.
///
/// The matrix `M` is well-conditioned at the IV-5 radii; an inline
/// SVD-based κ-bound check gates `κ(M) < 1e12` to surface any future
/// radii change that erodes invertibility. Empirical κ at the IV-5
/// radii is `O(1e8)` due to the wide A-vs-B coefficient scale gap
/// (`A_i` rows ~ `K_i` ~ `1e6`, `B_i` rows ~ `μ / r³` ~ `1e10`).
/// Mirrors IV-5's `solve_three_shell_lame` verbatim.
fn solve_three_shell_lame(
    shells: [ShellParams; 3],
    radii: [f64; 4], // [R_a, R_1, R_2, R_b]
    pressure: f64,
) -> LameCoefficients {
    let r_a = radii[0];
    let r_1 = radii[1];
    let r_2 = radii[2];
    let r_b = radii[3];
    let k = [
        shells[0].k_coefficient(),
        shells[1].k_coefficient(),
        shells[2].k_coefficient(),
    ];
    let mu = [shells[0].mu, shells[1].mu, shells[2].mu];

    // Layout: x = [A_1, B_1, A_2, B_2, A_3, B_3]ᵀ.
    let mut m = Matrix6::zeros();
    let mut b_vec = Vector6::zeros();

    // Eq 1: σ_rr^{(1)}(R_a) = -p  (cavity traction)
    m[(0, 0)] = k[0];
    m[(0, 1)] = -4.0 * mu[0] / r_a.powi(3);
    b_vec[0] = -pressure;

    // Eq 2: u_r^{(3)}(R_b) = 0  (fixed outer surface)
    m[(1, 4)] = r_b;
    m[(1, 5)] = 1.0 / (r_b * r_b);
    b_vec[1] = 0.0;

    // Eq 3: u_r^{(1)}(R_1) = u_r^{(2)}(R_1)
    m[(2, 0)] = r_1;
    m[(2, 1)] = 1.0 / (r_1 * r_1);
    m[(2, 2)] = -r_1;
    m[(2, 3)] = -1.0 / (r_1 * r_1);
    b_vec[2] = 0.0;

    // Eq 4: σ_rr^{(1)}(R_1) = σ_rr^{(2)}(R_1)
    m[(3, 0)] = k[0];
    m[(3, 1)] = -4.0 * mu[0] / r_1.powi(3);
    m[(3, 2)] = -k[1];
    m[(3, 3)] = 4.0 * mu[1] / r_1.powi(3);
    b_vec[3] = 0.0;

    // Eq 5: u_r^{(2)}(R_2) = u_r^{(3)}(R_2)
    m[(4, 2)] = r_2;
    m[(4, 3)] = 1.0 / (r_2 * r_2);
    m[(4, 4)] = -r_2;
    m[(4, 5)] = -1.0 / (r_2 * r_2);
    b_vec[4] = 0.0;

    // Eq 6: σ_rr^{(2)}(R_2) = σ_rr^{(3)}(R_2)
    m[(5, 2)] = k[1];
    m[(5, 3)] = -4.0 * mu[1] / r_2.powi(3);
    m[(5, 4)] = -k[2];
    m[(5, 5)] = 4.0 * mu[2] / r_2.powi(3);
    b_vec[5] = 0.0;

    // Conditioning gate.
    let cond = m.svd(false, false).singular_values;
    let kappa = cond[0] / cond[5];
    assert!(
        kappa < 1.0e12,
        "Lamé 6×6 conditioning κ(M) = {kappa:e} ≥ 1e12 at radii \
         (R_a, R_1, R_2, R_b) = ({r_a}, {r_1}, {r_2}, {r_b}); \
         radii ratios outside the well-conditioned band [1.2, 2.5] erode \
         invertibility — verify shell-radii choice before trusting the \
         analytic prediction"
    );

    let solution = m.lu().solve(&b_vec).expect(
        "Lamé 6×6 LU solve failed — matrix is singular or near-singular; check shell-radii \
         conditioning gate above",
    );
    LameCoefficients {
        a: [solution[0], solution[2], solution[4]],
        b: [solution[1], solution[3], solution[5]],
    }
}

/// Build the canonical three-shell Lamé coefficients at the row 11
/// constants. Returned `LameCoefficients` indexes shells inner-to-outer
/// (`shell_index = 0` → inner Ecoflex, `1` → middle composite, `2` →
/// outer Ecoflex), matching `shell_at`'s return.
fn build_canonical_lame_coefficients() -> LameCoefficients {
    let shells = [
        ShellParams {
            mu: MU_INNER,
            lambda: LAMBDA_INNER,
        },
        ShellParams {
            mu: MU_MIDDLE,
            lambda: LAMBDA_MIDDLE,
        },
        ShellParams {
            mu: MU_OUTER,
            lambda: LAMBDA_OUTER,
        },
    ];
    let radii = [
        LAYERED_SPHERE_R_CAVITY,
        LAYERED_SPHERE_R_INNER_OUTER,
        LAYERED_SPHERE_R_OUTER_INNER,
        LAYERED_SPHERE_R_OUTER,
    ];
    solve_three_shell_lame(shells, radii, PRESSURE)
}

// =============================================================================
// Solver runner — one backward-Euler `replay_step`, return SceneSnapshot
// =============================================================================

/// Bundle returned from [`run_scene`] — carries the production-scene
/// outputs + pre-consume snapshots needed by post-solve helpers (mirrors
/// row 10's `SceneSnapshot` shape, row 6 banked "snapshot before consume"
/// pattern).
struct SceneSnapshot {
    rest_positions: Vec<Vec3>,
    tet_verts: Vec<[VertexId; 4]>,
    materials: Vec<NeoHookean>,
    pinned: Vec<VertexId>,
    loaded: Vec<VertexId>,
    loaded_axes: Vec<LoadAxis>,
    referenced: Vec<VertexId>,
    step: NewtonStep<sim_soft::CpuTape>,
    cfg: SolverConfig,
}

fn run_scene(field: MaterialField) -> SceneSnapshot {
    let (mesh, bc, initial, theta) = SoftScene::layered_silicone_sphere(field, CELL_SIZE, PRESSURE)
        .expect(
            "layered_silicone_sphere should mesh successfully at the canonical h/2 cell_size + \
             PRESSURE scene per IV-5 + III-1 contracts",
        );
    let SceneInitial { x_prev, v_prev } = initial;

    // Snapshot before the mesh moves into the solver — row 6 banked
    // pattern.
    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();
    let n_tets = mesh.n_tets();
    let tet_verts: Vec<[VertexId; 4]> = (0..n_tets as u32).map(|t| mesh.tet_vertices(t)).collect();
    let materials: Vec<NeoHookean> = mesh.materials().to_vec();
    let referenced: Vec<VertexId> = referenced_vertices(&mesh);
    let pinned: Vec<VertexId> = bc.pinned_vertices.clone();
    let loaded: Vec<VertexId> = bc.loaded_vertices.iter().map(|&(v, _)| v).collect();
    let loaded_axes: Vec<LoadAxis> = bc.loaded_vertices.iter().map(|&(_, a)| a).collect();

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;

    let solver: CpuTet4NHSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
    let step = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);

    SceneSnapshot {
        rest_positions,
        tet_verts,
        materials,
        pinned,
        loaded,
        loaded_axes,
        referenced,
        step,
        cfg,
    }
}

/// Run the layered_silicone_sphere scene with the supplied (typically
/// uniform) `MaterialField` and return just the cavity-wall Saint-Venant
/// mean radial displacement. Used by the uniform-1× / uniform-2×
/// baseline runs at HEADLINE C (anchor 7 — `verify_cavity_wall_three_shell_strictly_between_uniform_bounds`).
/// Reuses [`run_scene`] for the full pipeline (snapshot + replay_step) +
/// [`cavity_wall_mean_radial_displacement`] for the readout; the per-tet
/// records and per-shell means are not needed for the between-bounds
/// gate so they're discarded.
fn run_uniform_baseline_cavity_wall_mean(field: MaterialField) -> f64 {
    let snapshot = run_scene(field);
    cavity_wall_mean_radial_displacement(&snapshot)
}

// =============================================================================
// Per-tet records — for anchors 4, 5, 10, 11 + PLY emit
// =============================================================================

#[derive(Debug, Clone, Copy)]
struct TetRecord {
    tet_id: u32,
    rest_centroid: Vec3,
    deformed_centroid: Vec3,
    /// Shell index per `shell_at(rest_centroid)`: 0 = inner, 1 = middle,
    /// 2 = outer. Mirrors `LayeredScalarField`'s
    /// `partition_point(|&t| t <= phi)` rule.
    shell_id: usize,
    /// Norm-difference radial displacement `|deformed_centroid| -
    /// |rest_centroid|` — the load-bearing scalar for the cf-view
    /// inflation visualisation. Mirrors IV-5's vertex-level idiom
    /// applied to centroids for cf-view emit.
    radial_displacement: f64,
    /// Per-tet `F = J · J_0^-1` at `x_final` — for the
    /// `solver_converges` validity-domain sanity gate.
    f_at_x_final: Matrix3<f64>,
}

fn build_tet_records(snapshot: &SceneSnapshot) -> Vec<TetRecord> {
    snapshot
        .tet_verts
        .iter()
        .enumerate()
        .map(|(t, verts)| {
            let v0 = snapshot.rest_positions[verts[0] as usize];
            let v1 = snapshot.rest_positions[verts[1] as usize];
            let v2 = snapshot.rest_positions[verts[2] as usize];
            let v3 = snapshot.rest_positions[verts[3] as usize];
            let rest_centroid = (v0 + v1 + v2 + v3) * 0.25;
            let shell_id = shell_at(rest_centroid);

            let cur = |i: usize| {
                let idx = verts[i] as usize;
                Vec3::new(
                    snapshot.step.x_final[3 * idx],
                    snapshot.step.x_final[3 * idx + 1],
                    snapshot.step.x_final[3 * idx + 2],
                )
            };
            let c0 = cur(0);
            let c1 = cur(1);
            let c2 = cur(2);
            let c3 = cur(3);
            let deformed_centroid = (c0 + c1 + c2 + c3) * 0.25;
            let radial_displacement = deformed_centroid.norm() - rest_centroid.norm();

            let j_0 = Matrix3::from_columns(&[v1 - v0, v2 - v0, v3 - v0]);
            let j = Matrix3::from_columns(&[c1 - c0, c2 - c0, c3 - c0]);
            let j_0_inv = j_0
                .try_inverse()
                .expect("singular reference jacobian — malformed rest mesh");
            let f_at_x_final = j * j_0_inv;

            TetRecord {
                tet_id: t as u32,
                rest_centroid,
                deformed_centroid,
                shell_id,
                radial_displacement,
                f_at_x_final,
            }
        })
        .collect()
}

/// `max |σᵢ − 1|` over the singular values of `F`. Same metric the
/// solver's `check_validity_at_step_start` evaluates per tet. Mirrors
/// rows 6 + 10's `max_stretch_deviation`.
fn max_stretch_deviation(f: &Matrix3<f64>) -> f64 {
    let svd = f.svd_unordered(false, false);
    svd.singular_values
        .iter()
        .map(|s| (s - 1.0).abs())
        .fold(0.0_f64, f64::max)
}

fn probe_f() -> Matrix3<f64> {
    Matrix3::from_diagonal_element(1.0)
        .map_with_location(|i, j, x| if i == 0 && j == 0 { PROBE_LAMBDA } else { x })
}

// =============================================================================
// Per-shell vertex-level radial-displacement readouts (HEADLINE B workhorse)
// =============================================================================

/// Saint-Venant-averaged cavity-wall mean radial displacement: mean over
/// `bc.loaded_vertices` of `(|x_final[v]| - |rest_pos[v]|)`. Mirrors
/// IV-5's `tests/concentric_lame_shells.rs:run_at_refinement` cavity-wall
/// reading verbatim.
fn cavity_wall_mean_radial_displacement(snapshot: &SceneSnapshot) -> f64 {
    let sum: f64 = snapshot
        .loaded
        .iter()
        .map(|&v| {
            let rest = snapshot.rest_positions[v as usize];
            let final_pos = Vec3::new(
                snapshot.step.x_final[3 * v as usize],
                snapshot.step.x_final[3 * v as usize + 1],
                snapshot.step.x_final[3 * v as usize + 2],
            );
            final_pos.norm() - rest.norm()
        })
        .sum();
    sum / snapshot.loaded.len() as f64
}

/// Mean radial displacement over all referenced vertices in a given
/// shell. Includes the cavity-surface vertices (in inner) and outer-
/// surface pinned vertices (in outer); both observed and analytic mean
/// computed over the same vertex set so quantization noise affects both
/// sides equally.
fn shell_mean_radial_displacement(snapshot: &SceneSnapshot, shell_id: usize) -> (f64, usize) {
    let mut sum = 0.0;
    let mut count: usize = 0;
    for &v in &snapshot.referenced {
        let rest = snapshot.rest_positions[v as usize];
        if shell_at(rest) != shell_id {
            continue;
        }
        let final_pos = Vec3::new(
            snapshot.step.x_final[3 * v as usize],
            snapshot.step.x_final[3 * v as usize + 1],
            snapshot.step.x_final[3 * v as usize + 2],
        );
        sum += final_pos.norm() - rest.norm();
        count += 1;
    }
    (sum / count as f64, count)
}

/// Mean analytic `coeffs.u_r(shell_id, |rest_pos[v]|)` over the same
/// vertex set as [`shell_mean_radial_displacement`] — both means walk
/// `snapshot.referenced` filtered by `shell_at(rest_pos[v]) == shell_id`,
/// so quantization noise affects both sides equally.
fn shell_mean_analytic_radial_displacement(
    snapshot: &SceneSnapshot,
    shell_id: usize,
    coeffs: &LameCoefficients,
) -> (f64, usize) {
    let mut sum = 0.0;
    let mut count: usize = 0;
    for &v in &snapshot.referenced {
        let rest = snapshot.rest_positions[v as usize];
        if shell_at(rest) != shell_id {
            continue;
        }
        sum += coeffs.u_r(shell_id, rest.norm());
        count += 1;
    }
    (sum / count as f64, count)
}

/// Cavity-wall mean analytic — the closed-form reading at
/// `r = R_CAVITY` from the inner shell's coefficients. Mirrors IV-5's
/// `coeffs.u_r(0, LAYERED_SPHERE_R_CAVITY)` reading verbatim.
fn cavity_wall_mean_analytic(coeffs: &LameCoefficients) -> f64 {
    coeffs.u_r(0, LAYERED_SPHERE_R_CAVITY)
}

// =============================================================================
// 1. verify_geometry_invariants — compile-time const asserts
// =============================================================================

const fn verify_geometry_invariants() {
    // Re-assert the `layered_silicone_sphere` constructor's runtime
    // panic invariants at the user-facing example layer (compile-time
    // enforcement on geometry constants). Reaching runtime here is a
    // no-op; the const-block panics surface at compile time on a
    // regression and the source location anchors the dependency on the
    // geometry constants loud and user-facing.
    const { assert!(LAYERED_SPHERE_R_CAVITY > 0.0) };
    const { assert!(LAYERED_SPHERE_R_CAVITY < LAYERED_SPHERE_R_INNER_OUTER) };
    const { assert!(LAYERED_SPHERE_R_INNER_OUTER < LAYERED_SPHERE_R_OUTER_INNER) };
    const { assert!(LAYERED_SPHERE_R_OUTER_INNER < LAYERED_SPHERE_R_OUTER) };
    const { assert!(LAYERED_SPHERE_BBOX_HALF_EXTENT > LAYERED_SPHERE_R_OUTER) };
    const { assert!(PRESSURE > 0.0) };
    // Decision J 1×/2×/1× symmetry + ordering: outer = inner < middle.
    const { assert!(MU_INNER == MU_OUTER) };
    const { assert!(LAMBDA_INNER == LAMBDA_OUTER) };
    const { assert!(MU_INNER < MU_MIDDLE) };
    const { assert!(LAMBDA_INNER < LAMBDA_MIDDLE) };
    // λ = 4 μ ⇒ ν = 0.4 compressible regime per IV-3 + IV-5 deviation
    // from Decision J's near-incompressible Ecoflex (recovered at
    // Phase H Tet10 + F-bar).
    const { assert!(LAMBDA_INNER == 4.0 * MU_INNER) };
    const { assert!(LAMBDA_MIDDLE == 4.0 * MU_MIDDLE) };
    const { assert!(LAMBDA_OUTER == 4.0 * MU_OUTER) };
    // Cell size + bbox margin sanity: the BCC + stuffing pipeline
    // requires `bbox_half_extent / cell_size ≥ ~6` for stable surface
    // resolution (per Phase 3 / IV-4 canonical).
    const { assert!(CELL_SIZE > 0.0) };
    const { assert!(LAYERED_SPHERE_BBOX_HALF_EXTENT / CELL_SIZE >= 6.0) };
    // PHI threshold ordering (re-asserts the `LayeredScalarField`
    // constructor's strict-monotone-increasing thresholds invariant).
    const { assert!(PHI_INNER_THRESHOLD < PHI_MIDDLE_THRESHOLD) };
    const { assert!(PHI_MIDDLE_THRESHOLD < 0.0) };
}

// =============================================================================
// 2. verify_mesh_topology_exact
// =============================================================================

fn verify_mesh_topology_exact(snapshot: &SceneSnapshot) {
    let n_vertices = snapshot.rest_positions.len();
    let n_tets = snapshot.tet_verts.len();
    let n_referenced = snapshot.referenced.len();
    assert_eq!(
        n_tets, N_TETS_EXACT,
        "n_tets drift: got {n_tets}, expected {N_TETS_EXACT} \
         at the canonical layered_silicone_sphere h/2 scene"
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
}

// =============================================================================
// 3. verify_boundary_partition
// =============================================================================

fn verify_boundary_partition(snapshot: &SceneSnapshot) {
    assert_eq!(
        snapshot.pinned.len(),
        N_PINNED_EXACT,
        "pinned (outer-surface band) cardinality drift: got {}, expected {N_PINNED_EXACT}",
        snapshot.pinned.len(),
    );
    assert_eq!(
        snapshot.loaded.len(),
        N_LOADED_EXACT,
        "loaded (cavity-surface band) cardinality drift: got {}, expected {N_LOADED_EXACT}",
        snapshot.loaded.len(),
    );

    // Both vectors come from `pick_vertices_by_predicate` which walks
    // vertex IDs in ascending order; ascending is structural here
    // (mirrors row 10 anchor 3 framing).
    for window in snapshot.pinned.windows(2) {
        assert!(
            window[0] < window[1],
            "pinned partition not ascending: {} >= {}",
            window[0],
            window[1],
        );
    }
    for window in snapshot.loaded.windows(2) {
        assert!(
            window[0] < window[1],
            "loaded partition not ascending: {} >= {}",
            window[0],
            window[1],
        );
    }

    // pinned ∩ loaded = ∅ — outer-surface band (R_OUTER) and
    // cavity-surface band (R_CAVITY) are at opposite radii so
    // disjointness is geometric.
    for &p in &snapshot.pinned {
        assert!(
            !snapshot.loaded.contains(&p),
            "vertex {p} appears in both pinned and loaded sets — outer-surface and \
             cavity-surface bands must be disjoint",
        );
    }

    // Every loaded entry uses LoadAxis::FullVector (radially-outward
    // pressure traction is per-component, not axis-aligned). Mirrors
    // IV-5's load axis convention verbatim — the
    // `layered_silicone_sphere` constructor packs every loaded vertex
    // with `LoadAxis::FullVector`.
    for (i, axis) in snapshot.loaded_axes.iter().enumerate() {
        assert!(
            matches!(axis, LoadAxis::FullVector),
            "loaded[{i}] uses {axis:?}, expected LoadAxis::FullVector \
             (radially-outward pressure traction is per-component)"
        );
    }
}

// =============================================================================
// 4. verify_per_tet_material_assignment — HEADLINE A
// =============================================================================

/// HEADLINE A — for every tet, `mesh.materials()[t]` agrees with
/// `expected = NH(MU_X, LAMBDA_X)` where `X = shell_at(rest_centroid)`
/// under the Material-trait probe at `F_probe = diag(1.20, 1, 1)`.
///
/// Cross-implementation gate: the test-side `shell_at` reimplements
/// `LayeredScalarField`'s `partition_point(|&t| t <= phi)` rule
/// verbatim, so centroid-sampled `materials()` and re-derived
/// `expected[shell_at(...)]` MUST agree on every tet by construction.
/// Drift between mesher partition pass and test re-derivation fires
/// loud. Same `Matrix3` over-determination logic as rows 8 + 9 + 10:
/// `energy(F_probe)` alone is one linear equation in `(μ, λ)`;
/// `first_piola(F_probe)`'s `P_22 = λ ln J` directly fixes `λ`, then
/// `P_11` fixes `μ` (over-determination).
///
/// Per-shell tet counts also exact-pinned. The canonical IV-5
/// cross-impl gate exposed user-facing.
fn verify_per_tet_material_assignment(
    snapshot: &SceneSnapshot,
    records: &[TetRecord],
) -> [usize; 3] {
    let f_probe = probe_f();
    let nh_inner = NeoHookean::from_lame(MU_INNER, LAMBDA_INNER);
    let nh_middle = NeoHookean::from_lame(MU_MIDDLE, LAMBDA_MIDDLE);
    let nh_outer = NeoHookean::from_lame(MU_OUTER, LAMBDA_OUTER);
    let materials = &snapshot.materials;
    assert_eq!(
        materials.len(),
        records.len(),
        "materials() length ({}) does not match per-tet record length ({})",
        materials.len(),
        records.len(),
    );

    let mut counts = [0usize; 3];
    for rec in records {
        let observed = &materials[rec.tet_id as usize];
        let expected_nh = match rec.shell_id {
            0 => &nh_inner,
            1 => &nh_middle,
            2 => &nh_outer,
            _ => unreachable!("shell_id is 0, 1, or 2 by construction"),
        };
        // Probe via energy — scalar comparison surfaces the easiest
        // diagnostic when a regression hits.
        assert_relative_eq!(
            observed.energy(&f_probe),
            expected_nh.energy(&f_probe),
            epsilon = EXACT_TOL,
        );
        // Probe via first_piola — Matrix3 entries over-determine the
        // (μ, λ) pair (energy alone underdetermines).
        let observed_p = observed.first_piola(&f_probe);
        let expected_p = expected_nh.first_piola(&f_probe);
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(observed_p[(i, j)], expected_p[(i, j)], epsilon = EXACT_TOL,);
            }
        }
        counts[rec.shell_id] += 1;
    }
    counts
}

// =============================================================================
// 5. verify_solver_converges
// =============================================================================

/// `iter_count < cfg.max_newton_iter`, `final_residual_norm < cfg.tol`,
/// AND per-tet `max|σ-1| < 1.0` at converged `x_final` (NH validity
/// boundary at `RequireOrientation` regime). The cavity-pressure-
/// inflation scene at `pressure = 5e3 Pa` deforms by `~0.71 %` of
/// `R_CAVITY` at the cavity wall (observed; `~0.84 %` analytic) — the
/// maximum per-tet stretch deviation `max|σ-1|` peaks at `~0.024`, well
/// inside NH's `RequireOrientation` validity domain.
fn verify_solver_converges(snapshot: &SceneSnapshot, records: &[TetRecord]) {
    let step = &snapshot.step;
    let cfg = &snapshot.cfg;

    assert!(
        step.iter_count < cfg.max_newton_iter,
        "Newton did not converge within budget: iter_count = {} >= max_newton_iter = {}",
        step.iter_count,
        cfg.max_newton_iter,
    );
    assert!(
        step.final_residual_norm < cfg.tol,
        "Final residual norm {:e} not below tol {:e}",
        step.final_residual_norm,
        cfg.tol,
    );

    // Validity-domain sanity at x_final. NH's `RequireOrientation`
    // declares `max_stretch_deviation = 1.0`; the cavity-pressure
    // inflation at 5e3 Pa lands at `~0.71 %` cavity-wall strain
    // (observed; `~0.84 %` analytic) ⇒ `max|σ-1|` per tet far below 1.0.
    assert_eq!(
        snapshot.materials[0]
            .validity()
            .max_stretch_deviation
            .to_bits(),
        1.0_f64.to_bits(),
        "NH validity boundary drift: max_stretch_deviation = {} (bits {:#018x}), \
         expected 1.0",
        snapshot.materials[0].validity().max_stretch_deviation,
        snapshot.materials[0]
            .validity()
            .max_stretch_deviation
            .to_bits(),
    );
    assert!(
        matches!(
            snapshot.materials[0].validity().inversion,
            InversionHandling::RequireOrientation
        ),
        "NH inversion handler drift: expected RequireOrientation",
    );

    let mut max_dev_global = 0.0_f64;
    for rec in records {
        let dev = max_stretch_deviation(&rec.f_at_x_final);
        if dev > max_dev_global {
            max_dev_global = dev;
        }
        assert!(
            dev < 1.0,
            "tet {} at x_final: max|σ-1| = {dev} >= NH bound 1.0 — cavity-pressure \
             inflation at {} Pa has produced an out-of-domain F at convergence",
            rec.tet_id,
            PRESSURE,
        );
    }
    assert!(
        max_dev_global < 0.05,
        "global max|σ-1| at x_final = {max_dev_global} > 0.05 — small-strain \
         regime sanity violated. Row 11 at h/2 ships with peak per-tet `max|σ-1|` \
         (3D peak SVD over F) at `~0.024` against `~0.84 %` cavity-wall analytic strain \
         (radial scalar; the SVD-peak / radial-strain factor is `~3×` from local \
         intensification at BCC mesh tets near the cavity surface — normal FEM behavior). \
         A peak `max|σ-1|` past 0.05 signals the converged field has shifted out of the \
         small-strain band into NH's nonlinear regime, OR that the inflation regime has \
         changed (PRESSURE / radii / stiffnesses)"
    );
}

// =============================================================================
// 6. verify_radial_displacement_per_shell_matches_lame_within_30pct — HEADLINE B
// =============================================================================

/// HEADLINE B — four Saint-Venant-averaged radial-displacement readouts
/// matched against the piecewise-Lamé closed-form at the row 11
/// constants:
///
/// 1. `cavity_wall_mean` — mean over `bc.loaded_vertices` (mirrors IV-5's
///    cavity-wall reading); analytic = `coeffs.u_r(0, R_CAVITY)`.
/// 2. `inner_shell_mean` — mean over all referenced vertices in inner
///    shell; analytic = `mean over same set of coeffs.u_r(0, |rest|)`.
/// 3. `middle_shell_mean` — same shape, shell 1.
/// 4. `outer_shell_mean` — same shape, shell 2.
///
/// Both observed and analytic means walk the SAME vertex set so
/// BCC-quantization noise affects both sides equally; the comparison is
/// apples-to-apples without needing a band-tolerance choice. Inventory's
/// named gate ("assert radial displacement vs Lamé per shell"),
/// generalising IV-5's cavity-wall-only reading to a 3-shell profile
/// gate while preserving the IV-5 cross-reference at the cavity-wall
/// readout.
///
/// Returns `[cavity_wall_mean, inner_shell_mean, middle_shell_mean,
/// outer_shell_mean]` for downstream anchors 7 + 8.
fn verify_radial_displacement_per_shell_matches_lame_within_30pct(
    snapshot: &SceneSnapshot,
    coeffs: &LameCoefficients,
) -> [f64; 4] {
    let cavity_observed = cavity_wall_mean_radial_displacement(snapshot);
    let cavity_analytic = cavity_wall_mean_analytic(coeffs);

    assert!(
        cavity_observed > 0.0,
        "cavity-wall mean radial displacement must be positive (cavity inflates outward \
         under internal pressure); got {cavity_observed:e}"
    );
    assert!(
        cavity_analytic > 0.0,
        "cavity-wall analytic radial displacement must be positive at internal pressure; \
         got {cavity_analytic:e} — possible sign error in the closed-form 6×6 setup"
    );
    assert_relative_eq!(
        cavity_observed,
        cavity_analytic,
        max_relative = RADIAL_REL_TOL,
        epsilon = RADIAL_EPS_ABS_FLOOR,
    );

    let mut means = [cavity_observed, 0.0, 0.0, 0.0];
    for shell_id in 0..3 {
        let (observed, n_obs) = shell_mean_radial_displacement(snapshot, shell_id);
        let (analytic, n_ana) = shell_mean_analytic_radial_displacement(snapshot, shell_id, coeffs);
        assert_eq!(
            n_obs, n_ana,
            "shell {shell_id} vertex-count mismatch between observed mean ({n_obs}) and \
             analytic mean ({n_ana}) — both walk the SAME vertex set by construction"
        );
        assert!(
            n_obs > 0,
            "shell {shell_id} has zero referenced vertices — mean is undefined"
        );
        assert!(
            observed > 0.0,
            "shell {shell_id} observed mean must be positive (radial expansion outward); \
             got {observed:e}"
        );
        assert!(
            analytic > 0.0,
            "shell {shell_id} analytic mean must be positive; got {analytic:e}"
        );
        assert_relative_eq!(
            observed,
            analytic,
            max_relative = RADIAL_REL_TOL,
            epsilon = RADIAL_EPS_ABS_FLOOR,
        );
        means[shell_id + 1] = observed;
    }
    means
}

// =============================================================================
// 7. verify_cavity_wall_three_shell_strictly_between_uniform_bounds — HEADLINE C
// =============================================================================

/// HEADLINE C — IV-2 lens β analog at the cavity-wall mean (mirrors row
/// 10's banked pattern (d) on tip displacement, here adapted to the
/// hollow-body cavity-wall mean). Three solver runs total — the
/// three-shell scene plus the uniform-1× baseline (every shell at
/// `MU_INNER, LAMBDA_INNER`) plus the uniform-2× baseline (every shell
/// at `MU_MIDDLE, LAMBDA_MIDDLE`). The cavity-wall mean radial
/// displacement satisfies a strict-between inequality:
///
/// ```text
/// u_r_uniform_2x_cavity < u_r_three_shell_cavity < u_r_uniform_1x_cavity
/// ```
///
/// The directionality follows from compliance composition under fixed-
/// outer pressure-vessel mechanics: uniform-2× has every shell at `2×`
/// the IV-1 baseline → stiffest configuration → smallest cavity inflation
/// (by linearity of the Lamé 6×6 system in `(μ, λ)`, uniform-2× cavity
/// is exactly half of uniform-1× cavity); uniform-1× has every shell at
/// `1×` → softest → largest inflation; three-shell `1×/2×/1×` sits
/// between because two of its three shells are at `1×` (more compliant
/// than uniform-2×) and one is at `2×` (less compliant than uniform-1×).
///
/// Catches dropped-tet-contribution / swapped-materials / mis-assigned
/// bugs that would push `u_r_three_shell_cavity` to one of the bounds
/// (or outside) without flipping any per-tet material-probe (anchor 4)
/// or per-shell rel-err gate (anchor 6). The strict-between inequality
/// is structurally tighter than anchor 6's `< 0.30` rel-err gate at
/// downward-bias FEM-assembly regressions in the 17-30% range — anchor
/// 6's cavity-wall reading observed at 14.5% rel-err is 15-percentage-
/// points below the gate; anchor 7 catches a 17-percentage-point
/// downward shift that would push three-shell below uniform-1×.
fn verify_cavity_wall_three_shell_strictly_between_uniform_bounds(
    cavity_three_shell: f64,
    cavity_uniform_1x: f64,
    cavity_uniform_2x: f64,
) {
    assert!(
        cavity_three_shell > 0.0,
        "three-shell cavity-wall mean must be positive (cavity inflates outward); got {cavity_three_shell:e}"
    );
    assert!(
        cavity_uniform_1x > 0.0,
        "uniform-1× cavity-wall mean must be positive; got {cavity_uniform_1x:e}"
    );
    assert!(
        cavity_uniform_2x > 0.0,
        "uniform-2× cavity-wall mean must be positive; got {cavity_uniform_2x:e}"
    );
    // Sanity: uniform-2× (all-2× stiff) deflects strictly less than
    // uniform-1× (all-1× soft). Without this the "between bounds" claim
    // is vacuous.
    assert!(
        cavity_uniform_2x < cavity_uniform_1x,
        "uniform-2× cavity-wall mean ({cavity_uniform_2x:e}) must be less than uniform-1× \
         ({cavity_uniform_1x:e}) — all-stiff configuration deflects strictly less than all-soft \
         (linearity of the Lamé 6×6 in `(μ, λ)`)"
    );
    // Three-shell strictly between: lower bound (uniform-2×).
    assert!(
        cavity_uniform_2x < cavity_three_shell,
        "three-shell cavity-wall mean ({cavity_three_shell:e}) must exceed uniform-2× \
         ({cavity_uniform_2x:e}) — two of three shells (inner + outer) are at 1× (softer than \
         uniform-2×'s all-2× configuration), so three-shell is more compliant"
    );
    // Three-shell strictly between: upper bound (uniform-1×).
    assert!(
        cavity_three_shell < cavity_uniform_1x,
        "three-shell cavity-wall mean ({cavity_three_shell:e}) must be less than uniform-1× \
         ({cavity_uniform_1x:e}) — the middle shell is at 2× (stiffer than uniform-1×'s all-1× \
         configuration), so three-shell is less compliant"
    );
}

// =============================================================================
// 8. verify_radial_monotonicity_outward
// =============================================================================

/// Strict outward-decreasing monotonicity across the four Saint-Venant-
/// averaged readouts: `cavity_wall_mean > inner_shell_mean >
/// middle_shell_mean > outer_shell_mean ≥ 0`. The piecewise-Lamé
/// closed-form predicts strict outward monotone-decay under the
/// internal-pressure-with-fixed-outer geometry (cavity wall most
/// inflated, outer wall pinned at 0). Sanity guard against
/// load-direction or BC-sign regressions.
fn verify_radial_monotonicity_outward(means: &[f64; 4]) {
    let [cavity, inner, middle, outer] = *means;
    assert!(
        cavity > inner,
        "monotonicity drift: cavity_wall_mean ({cavity:e}) must exceed \
         inner_shell_mean ({inner:e}) — cavity-wall vertices are at the inner \
         boundary where u_r is maximal under internal-pressure inflation"
    );
    assert!(
        inner > middle,
        "monotonicity drift: inner_shell_mean ({inner:e}) must exceed \
         middle_shell_mean ({middle:e}) — radial displacement decays outward \
         under fixed-outer BC"
    );
    assert!(
        middle > outer,
        "monotonicity drift: middle_shell_mean ({middle:e}) must exceed \
         outer_shell_mean ({outer:e}) — outer wall is pinned at u_r = 0"
    );
    assert!(
        outer >= 0.0,
        "monotonicity drift: outer_shell_mean ({outer:e}) must be non-negative \
         (radial expansion is outward, never inward, under positive internal \
         pressure)"
    );
}

// =============================================================================
// 9. verify_captured_bits_radial_displacements
// =============================================================================

/// 6 captured radial-displacement means under the IV-1 sparse-tier
/// rel-tol contract: 4 from the three-shell run (cavity-wall + 3
/// per-shell) + 2 from the HEADLINE C uniform-baseline runs
/// (uniform-1× cavity-wall, uniform-2× cavity-wall).
fn verify_captured_bits_radial_displacements(
    means: &[f64; 4],
    cavity_uniform_1x: f64,
    cavity_uniform_2x: f64,
) {
    let [cavity, inner, middle, outer] = *means;
    let cavity_ref = f64::from_bits(CAVITY_WALL_MEAN_REF_BITS);
    let inner_ref = f64::from_bits(INNER_SHELL_MEAN_REF_BITS);
    let middle_ref = f64::from_bits(MIDDLE_SHELL_MEAN_REF_BITS);
    let outer_ref = f64::from_bits(OUTER_SHELL_MEAN_REF_BITS);
    let uniform_1x_ref = f64::from_bits(CAVITY_WALL_UNIFORM_1X_REF_BITS);
    let uniform_2x_ref = f64::from_bits(CAVITY_WALL_UNIFORM_2X_REF_BITS);
    assert_relative_eq!(
        cavity,
        cavity_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        inner,
        inner_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        middle,
        middle_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        outer,
        outer_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        cavity_uniform_1x,
        uniform_1x_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        cavity_uniform_2x,
        uniform_2x_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
}

// =============================================================================
// 10. verify_material_populations_exact
// =============================================================================

fn verify_material_populations_exact(counts: [usize; 3]) {
    let [n_inner, n_middle, n_outer] = counts;
    assert_eq!(
        n_inner, N_INNER_TETS_EXACT,
        "inner-shell tet count drift: got {n_inner}, expected {N_INNER_TETS_EXACT}"
    );
    assert_eq!(
        n_middle, N_MIDDLE_TETS_EXACT,
        "middle-shell tet count drift: got {n_middle}, expected {N_MIDDLE_TETS_EXACT}"
    );
    assert_eq!(
        n_outer, N_OUTER_TETS_EXACT,
        "outer-shell tet count drift: got {n_outer}, expected {N_OUTER_TETS_EXACT}"
    );
    assert_eq!(
        n_inner + n_middle + n_outer,
        N_TETS_EXACT,
        "per-shell tet counts do not partition: sum != N_TETS_EXACT"
    );
}

// =============================================================================
// 11. verify_zslab_visual_populations_exact
// =============================================================================

/// Per-shell tet counts in the `|centroid.z| < cell_size/2` z-slab cut
/// for the cf-view PLY artifact. Visual-pedagogy guard: each shell must
/// have ≥ 1 z-slab centroid for the three concentric-ring cf-view
/// rendering to work; per-shell counts also exact-pinned per the III-1
/// determinism contract.
fn verify_zslab_visual_populations_exact(records: &[TetRecord]) -> [usize; 3] {
    let zslab_half = CELL_SIZE / 2.0;
    let mut counts = [0usize; 3];
    for rec in records {
        if rec.rest_centroid.z.abs() < zslab_half {
            counts[rec.shell_id] += 1;
        }
    }
    let [n_inner, n_middle, n_outer] = counts;
    assert!(
        n_inner > 0,
        "z-slab inner-shell bucket empty — cf-view visual would lose the inner-shell color"
    );
    assert!(
        n_middle > 0,
        "z-slab middle-shell bucket empty — cf-view visual would lose the middle-shell color"
    );
    assert!(
        n_outer > 0,
        "z-slab outer-shell bucket empty — cf-view visual would lose the outer-shell color"
    );
    assert_eq!(
        n_inner, N_INNER_TETS_ZSLAB_EXACT,
        "z-slab inner-shell count drift: got {n_inner}, expected {N_INNER_TETS_ZSLAB_EXACT}"
    );
    assert_eq!(
        n_middle, N_MIDDLE_TETS_ZSLAB_EXACT,
        "z-slab middle-shell count drift: got {n_middle}, expected {N_MIDDLE_TETS_ZSLAB_EXACT}"
    );
    assert_eq!(
        n_outer, N_OUTER_TETS_ZSLAB_EXACT,
        "z-slab outer-shell count drift: got {n_outer}, expected {N_OUTER_TETS_ZSLAB_EXACT}"
    );
    counts
}

// =============================================================================
// PLY emit — z-slab per-tet centroid point cloud + 2 per-vertex scalars
// =============================================================================

/// Vertices = per-tet centroids in the `|centroid.z| < cell_size/2`
/// z-slab cut, with `vertex = rest + DISPLACEMENT_SCALE * (deformed -
/// rest)` — visualisation-only geometric amplification of the deformed
/// configuration (see `DISPLACEMENT_SCALE` doc above). Faces empty
/// (point cloud); two per-vertex scalars cast as f32:
///
/// - `material_id` (categorical 0.0 / 1.0 / 2.0; cf-view auto-detects
///   integer-valued and 3 unique values < 16 → tab10 categorical
///   highlight of the three concentric shells)
/// - `radial_displacement` (continuous; cf-view auto-detects all-positive
///   continuous → viridis sequential — the **TRUE physical** radial-
///   inflation profile, NOT scaled)
///
/// cf-view's Q5 colormap heuristic detects each scalar's distribution
/// independently. Default-pick is alphabetical-first → `material_id`
/// shows on launch (3 sharp shells in 3 colors is the loudest first
/// impression); user dropdowns to `radial_displacement` to see the
/// continuous radial-decay gradient from cavity wall to outer wall.
///
/// z-slab filter applied on `rest_centroid.z` (not deformed_centroid.z
/// — the z-component of the deformation is small at this configuration
/// but the rest position determines what's "in the equatorial plane").
fn save_concentric_lame_shells_ply(records: &[TetRecord], path: &Path) -> Result<()> {
    let zslab_half = CELL_SIZE / 2.0;
    let kept: Vec<&TetRecord> = records
        .iter()
        .filter(|r| r.rest_centroid.z.abs() < zslab_half)
        .collect();
    let vertices: Vec<Point3<f64>> = kept
        .iter()
        .map(|r| {
            let amplified =
                r.rest_centroid + DISPLACEMENT_SCALE * (r.deformed_centroid - r.rest_centroid);
            Point3::new(amplified.x, amplified.y, amplified.z)
        })
        .collect();
    let faces: Vec<[u32; 3]> = Vec::new();
    let geometry = IndexedMesh::from_parts(vertices, faces);
    let mut attributed = AttributedMesh::new(geometry);
    let material_id: Vec<f32> = kept.iter().map(|r| r.shell_id as f32).collect();
    let radial_displacement: Vec<f32> = kept.iter().map(|r| r.radial_displacement as f32).collect();
    attributed.insert_extra("material_id", material_id)?;
    attributed.insert_extra("radial_displacement", radial_displacement)?;
    save_ply_attributed(&attributed, path, true)?;
    Ok(())
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

#[allow(clippy::similar_names)]
fn print_summary(
    snapshot: &SceneSnapshot,
    records: &[TetRecord],
    pop_counts: [usize; 3],
    zslab_counts: [usize; 3],
    means: &[f64; 4],
    cavity_uniform_1x: f64,
    cavity_uniform_2x: f64,
    coeffs: &LameCoefficients,
    path: &Path,
) {
    let [n_inner, n_middle, n_outer] = pop_counts;
    let [n_inner_zslab, n_middle_zslab, n_outer_zslab] = zslab_counts;
    let [cavity_obs, inner_obs, middle_obs, outer_obs] = *means;
    let cavity_ana = cavity_wall_mean_analytic(coeffs);
    let (inner_ana, _) = shell_mean_analytic_radial_displacement(snapshot, 0, coeffs);
    let (middle_ana, _) = shell_mean_analytic_radial_displacement(snapshot, 1, coeffs);
    let (outer_ana, _) = shell_mean_analytic_radial_displacement(snapshot, 2, coeffs);

    let zslab_total = n_inner_zslab + n_middle_zslab + n_outer_zslab;
    let max_dev_global = records
        .iter()
        .map(|r| max_stretch_deviation(&r.f_at_x_final))
        .fold(0.0_f64, f64::max);

    let cavity_rel_err = (cavity_obs - cavity_ana).abs() / cavity_ana.abs();
    let inner_rel_err = (inner_obs - inner_ana).abs() / inner_ana.abs();
    let middle_rel_err = (middle_obs - middle_ana).abs() / middle_ana.abs();
    let outer_rel_err = (outer_obs - outer_ana).abs() / outer_ana.abs();

    println!("==== concentric-lame-shells ====");
    println!();
    println!("Scene: SoftScene::layered_silicone_sphere");
    println!(
        "  geometry             : DifferenceSdf hollow shell, R_OUTER = {LAYERED_SPHERE_R_OUTER}, R_CAVITY = {LAYERED_SPHERE_R_CAVITY}"
    );
    println!(
        "  shell radii          : R_CAVITY = {LAYERED_SPHERE_R_CAVITY} → R_INNER_OUTER = {LAYERED_SPHERE_R_INNER_OUTER} → R_OUTER_INNER = {LAYERED_SPHERE_R_OUTER_INNER} → R_OUTER = {LAYERED_SPHERE_R_OUTER}"
    );
    println!("  cell_size            : {CELL_SIZE} (h/2 — IV-5 / III-1 canonical)");
    println!(
        "                         = {N_VERTICES_EXACT} verts ({N_REFERENCED_EXACT} referenced), {N_TETS_EXACT} tets"
    );
    println!(
        "  inner shell           : NH(MU_INNER = {MU_INNER:e}, LAMBDA_INNER = {LAMBDA_INNER:e})  Ecoflex 1×"
    );
    println!(
        "  middle shell          : NH(MU_MIDDLE = {MU_MIDDLE:e}, LAMBDA_MIDDLE = {LAMBDA_MIDDLE:e})  composite 2×"
    );
    println!(
        "  outer shell           : NH(MU_OUTER = {MU_OUTER:e}, LAMBDA_OUTER = {LAMBDA_OUTER:e})  Ecoflex 1×"
    );
    println!(
        "  internal pressure     : {PRESSURE} Pa, distributed across {N_LOADED_EXACT} cavity-surface vertices via LoadAxis::FullVector"
    );
    println!(
        "  outer surface         : Dirichlet-pinned ({N_PINNED_EXACT} vertices in R_OUTER band)"
    );
    println!(
        "  solver config         : Δt = {} s (static-regime), tol = {:e} N, max_newton_iter = {}",
        snapshot.cfg.dt, snapshot.cfg.tol, snapshot.cfg.max_newton_iter
    );
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!(
        "  geometry_invariants                                              : compile-time const asserts on geometry constants"
    );
    println!(
        "  mesh_topology_exact                                              : n_tets = {N_TETS_EXACT}, n_vertices = {N_VERTICES_EXACT}, referenced = {N_REFERENCED_EXACT}"
    );
    println!(
        "  boundary_partition                                               : {N_PINNED_EXACT} pinned + {N_LOADED_EXACT} loaded; ascending; disjoint; LoadAxis::FullVector"
    );
    println!(
        "  per_tet_material_assignment                                      : every tet probed bit-equal vs (NH(inner) | NH(middle) | NH(outer)) (HEADLINE A)"
    );
    println!(
        "  solver_converges                                                 : iter < {MAX_NEWTON_ITER}; residual < tol; max|σ-1| < 1.0 at x_final"
    );
    println!(
        "  radial_displacement_per_shell_matches_lame_within_30pct          : 4 readouts pass assert_relative_eq! (rel < {RADIAL_REL_TOL} OR abs < {RADIAL_EPS_ABS_FLOOR:e} m) (HEADLINE B)"
    );
    println!(
        "  cavity_wall_three_shell_strictly_between_uniform_bounds          : u_r_uniform_2x < u_r_three_shell < u_r_uniform_1x (HEADLINE C)"
    );
    println!(
        "  radial_monotonicity_outward                                      : cavity > inner > middle > outer ≥ 0"
    );
    println!(
        "  captured_bits_radial_displacements                               : 6 means (4 three-shell + 2 uniform) within IV-1 sparse-tier rel-tol"
    );
    println!(
        "  material_populations_exact                                       : per-shell tet counts exact-pinned (full body)"
    );
    println!(
        "  zslab_visual_populations_exact                                   : per-shell z-slab counts exact-pinned (cf-view artifact)"
    );
    println!();
    println!("Per-shell tet counts (full body):");
    println!("  inner  (R_CAVITY ≤ ‖p‖ < R_INNER_OUTER) : {n_inner:>5}");
    println!("  middle (R_INNER_OUTER ≤ ‖p‖ < R_OUTER_INNER) : {n_middle:>5}");
    println!("  outer  (R_OUTER_INNER ≤ ‖p‖ ≤ R_OUTER)  : {n_outer:>5}");
    println!(
        "  total                                : {:>5}  (== n_tets)",
        n_inner + n_middle + n_outer
    );
    println!();
    println!("Solver result:");
    println!("  iter_count            : {:>5}", snapshot.step.iter_count);
    println!(
        "  final_residual_norm   : {:>13e} N  (tol {:e})",
        snapshot.step.final_residual_norm, snapshot.cfg.tol
    );
    println!(
        "  max|σ-1| at x_final   : {max_dev_global:>13.6}     (< 1.0 strict in-domain; < 0.05 small-strain sanity)"
    );
    println!();
    println!("Saint-Venant-averaged radial displacements (HEADLINE B):");
    println!("                          observed             Lamé analytic        rel-err");
    println!(
        "  cavity_wall_mean      : {cavity_obs:>13.6e} m    {cavity_ana:>13.6e} m    {cavity_rel_err:>8.4}"
    );
    println!(
        "  inner_shell_mean      : {inner_obs:>13.6e} m    {inner_ana:>13.6e} m    {inner_rel_err:>8.4}"
    );
    println!(
        "  middle_shell_mean     : {middle_obs:>13.6e} m    {middle_ana:>13.6e} m    {middle_rel_err:>8.4}"
    );
    println!(
        "  outer_shell_mean      : {outer_obs:>13.6e} m    {outer_ana:>13.6e} m    {outer_rel_err:>8.4}"
    );
    println!(
        "  gate: assert_relative_eq! at max_relative = {RADIAL_REL_TOL}, epsilon = {RADIAL_EPS_ABS_FLOOR:e} m"
    );
    println!("        (rel-tol binding for cavity / inner / middle; eps-floor absorbs outer-shell");
    println!(
        "         small-magnitude case — abs diff ~3.7e-6 m < {RADIAL_EPS_ABS_FLOOR:e} m floor)"
    );

    let three_shell_cavity = means[0];
    println!();
    println!("Cavity-wall mean uniform-baseline comparison (HEADLINE C — IV-2 lens β analog):");
    println!(
        "  uniform_2× cavity     : {cavity_uniform_2x:>13.6e} m  (every shell at MU_MIDDLE = 2× — stiffest)"
    );
    println!("  three-shell cavity    : {three_shell_cavity:>13.6e} m  (1× / 2× / 1× layered)");
    println!(
        "  uniform_1× cavity     : {cavity_uniform_1x:>13.6e} m  (every shell at MU_INNER = 1× — softest)"
    );
    println!(
        "  inequality            : {cavity_uniform_2x:.4e} < {three_shell_cavity:.4e} < {cavity_uniform_1x:.4e}  ✓"
    );

    println!();
    println!("Z-slab cut PLY (cf-view artifact, |centroid.z| < cell_size/2 equatorial plane):");
    println!("  inner shell           : {n_inner_zslab:>5}");
    println!("  middle shell          : {n_middle_zslab:>5}");
    println!("  outer shell           : {n_outer_zslab:>5}");
    println!("  total                 : {zslab_total:>5}");
    println!();
    println!("PLY    : {}", path.display());
    println!(
        "         vertices-only point cloud ({zslab_total} centroids — z-slab of the {N_TETS_EXACT}"
    );
    println!(
        "         body tets, vertex positions amplified `rest + {DISPLACEMENT_SCALE}× (deformed - rest)` — vis only)"
    );
    println!("         + 2 per-vertex scalars:");
    println!(
        "           extras[\"material_id\"]          — categorical 0.0/1.0/2.0 (inner/middle/outer)"
    );
    println!(
        "           extras[\"radial_displacement\"]  — continuous m       (TRUE physical inflation profile)"
    );
    println!("         open in cf-view, the workspace's unified visual-review viewer:");
    println!("           cargo run -p cf-viewer --release -- <path>");
    println!(
        "         alphabetical-first scalar pick lands on `material_id` (categorical → tab10);"
    );
    println!(
        "         scalar dropdown switches to `radial_displacement` (continuous → viridis) for the"
    );
    println!(
        "         radial-inflation gradient. The thin z-slab projects to a 2D x-y disk in cf-view,"
    );
    println!(
        "         reading as three concentric color rings + a smooth radial gradient unmistakably."
    );
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    verify_geometry_invariants();

    let snapshot = run_scene(three_shell_field());
    let records = build_tet_records(&snapshot);
    let coeffs = build_canonical_lame_coefficients();

    verify_mesh_topology_exact(&snapshot);
    verify_boundary_partition(&snapshot);

    let pop_counts = verify_per_tet_material_assignment(&snapshot, &records);
    verify_solver_converges(&snapshot, &records);

    let means = verify_radial_displacement_per_shell_matches_lame_within_30pct(&snapshot, &coeffs);

    // HEADLINE C uniform-baseline runs (anchor 7).
    let cavity_uniform_1x = run_uniform_baseline_cavity_wall_mean(uniform_1x_field());
    let cavity_uniform_2x = run_uniform_baseline_cavity_wall_mean(uniform_2x_field());

    verify_cavity_wall_three_shell_strictly_between_uniform_bounds(
        means[0],
        cavity_uniform_1x,
        cavity_uniform_2x,
    );
    verify_radial_monotonicity_outward(&means);
    verify_captured_bits_radial_displacements(&means, cavity_uniform_1x, cavity_uniform_2x);

    verify_material_populations_exact(pop_counts);
    let zslab_counts = verify_zslab_visual_populations_exact(&records);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let out_path = out_dir.join("concentric_lame_shells.ply");
    save_concentric_lame_shells_ply(&records, &out_path)?;

    print_summary(
        &snapshot,
        &records,
        pop_counts,
        zslab_counts,
        &means,
        cavity_uniform_1x,
        cavity_uniform_2x,
        &coeffs,
        &out_path,
    );

    Ok(())
}
