//! compressive-block — Compressive-block user-facing wrap: soft cube
//! quasi-statically compressed by a descending rigid plane against a
//! BC-pinned bottom face; per-refinement reaction force compared
//! against the two pure-BC analytic limits + Cauchy convergence.
//!
//! `SoftScene::compressive_block_on_plane(edge_len, cell_size,
//! displacement, &material_field)` (the compressive-block fixture's Phase 5 commit-6
//! helper at `sim/L0/soft/src/readout/scene.rs:377`) builds the
//! production scene — a `HandBuiltTetMesh::uniform_block` cube of edge
//! `L = 1 cm` at three refinement levels (`n ∈ {2, 4, 8}` →
//! `48 / 384 / 3072` tets), full-pin BC on every bottom-face vertex
//! (`z ≈ 0` band, half-cell tolerance), no external traction. A single
//! `RigidPlane(n = -ẑ, offset = δ - L)` represents the descended top
//! plate, penetrated by exactly `δ = 5 × 10⁻⁵ m` at rest; one-way
//! `PenaltyRigidContact` with **fixture-local `(d̂, δ)` override**
//! (`d̂ = 1e-5 m`, default `κ = 1e4 N/m`) drives the cube into static
//! equilibrium under the descended plate.
//!
//! `STATIC_DT = 1.0 s` collapses the inertial Tikhonov regulariser
//! `M / dt²` so a single `replay_step` from rest converges to static
//! equilibrium (mirrors the compressive block's `cfg.dt = 1.0`). Three refinements run
//! sequentially; the row's headline gate is the **per-refinement
//! two-bound bracket `F_us ≤ F_R_FEM ≤ F_strain`** plus **Cauchy ratio
//! `< 1`** on the `F_R_FEM` sequence.
//!
//! ## Why `F_R ∈ [F_us, F_strain]` is the headline
//!
//! the compressive-block fixture's "Deviation 1" lays out the full derivation; mirrored
//! here verbatim:
//!
//! The original spec named the gate as `< 5 %` rel-err vs uniaxial-
//! stress small-strain `F_us = E · A · ε`, presupposing pure
//! uniaxial-stress BC (z-only pin on the bottom face, x/y free). The
//! commit-6 helper full-pins every bottom-face vertex (Phase 5
//! `BoundaryConditions` only models full-vertex Dirichlet — see
//! `scene.rs:414-423`), giving a **mixed BC**: bottom full-pinned
//! (constrained-modulus regime locally), sides free (uniaxial-stress
//! regime), top z-contacted. The deformation field is non-uniform;
//! **no clean closed-form exists** for this BC at general aspect
//! ratio.
//!
//! This row therefore brackets the FEM response by **two pure-BC bounds**
//! at the equilibrium strain:
//!
//! - **Lower bound** — uniaxial-stress small-strain `F_us = E · A · ε`
//!   (everywhere lateral free; achievable only with z-only-pin BC).
//!   The mixed BC must give `F ≥ F_us` because adding lateral
//!   constraints (the bottom full-pin) makes the system stiffer, not
//!   softer.
//! - **Upper bound** — uniaxial-strain small-strain
//!   `F_strain = M_c · A · ε` where
//!   `M_c = E · (1 - ν) / ((1 + ν)(1 - 2 ν)) = λ + 2 μ` (everywhere
//!   lateral pin; achievable only with full-pin throughout). The
//!   mixed BC must give `F ≤ F_strain` because removing lateral
//!   constraints (sides free, top free) makes the system softer.
//!
//! With `μ = 1e5`, `λ = 4e5` ⇒ `ν = 0.4`, `E = 2 μ (1 + ν) = 2.8e5
//! Pa`, `M_c = λ + 2 μ = 6.0e5 Pa` (note `M_c / E ≈ 2.14`); the bounds
//! at `ε ≈ 0.6 %` give `F_us ≈ 0.16 N` and `F_strain ≈ 0.36 N`. The
//! FEM at this scene's aspect ratio (cube) sits close to `F_us` — the
//! bottom-pin's lateral constraint is geometrically confined to a
//! thin Saint-Venant boundary layer, with most of the interior in
//! uniaxial-stress regime.
//!
//! The "NH-derived stiffness slope" framing in the [`EXAMPLE_INVENTORY`]
//! row 14 spec is **mathematically equivalent** to the two-bound
//! bracket: `F_R / (A · ε) ∈ [E, M_c]` ⟺ `F_R ∈ [E·A·ε, M_c·A·ε]` =
//! `[F_us, F_strain]`. The example reports
//! `effective_modulus_n8 = F_R_n8 / (A · ε_n8)` and
//! `rel_pos_in_bounds_n8 = (effective_modulus − E) / (M_c − E)` ∈
//! `[0, 1]` in stdout + JSON as derived diagnostics; only the F_R
//! bracket is asserted (the modulus framing is a redundant view of
//! the same constraint).
//!
//! [`EXAMPLE_INVENTORY`]: ../../sim/L0/soft/EXAMPLE_INVENTORY.md
//!
//! Reference: Sokolnikoff, *Mathematical Theory of Elasticity*, 2nd ed.,
//! Ch 4 (Saint-Venant principle for boundary-layer effects); Bonet &
//! Wood, *Nonlinear Continuum Mechanics for Finite Element Analysis*,
//! 2nd ed., Ch 5 (small-strain regime).
//!
//! ## Why the fixture-local `(d̂, δ)` override
//!
//! the compressive-block fixture's "Deviation 2" lays out the full derivation; mirrored
//! here verbatim:
//!
//! At the original spec parameters
//! (`L = 1 cm, δ = 0.5 mm, κ = 1e4 N/m, d̂ = 1 mm, ν = 0.4`), the
//! cold-start penalty residual is `~κ · d̂ ≈ 10 N` per top-face
//! vertex. The raw Newton step is `~residual / κ ≈ 1 mm` per vertex —
//! 10 % of the cube edge, past the tet-inversion threshold. Armijo's
//! residual-norm sufficient-decrease condition does NOT check element
//! invertibility; line-search accepts trial steps that push elements
//! into NeoHookean's compressive nonlinearity regime, and the next
//! Newton iter's tangent fails Cholesky.
//!
//! This row takes a fixture-local override:
//! `d̂ = 1e-5 m` (100× smaller than default) and `δ = 5e-5 m` (10×
//! smaller than scope memo §9). At the override parameters: cold-start
//! residual `κ · (d̂ + δ) ≈ 0.6 N` per vertex, raw Newton step `~6 ×
//! 10⁻⁵ m ≈ 0.6 %` of edge — safely below tet-inversion threshold.
//! Equilibrium strain `ε ≈ 0.6 %` — deep into small-strain regime
//! where the two pure-BC bounds cleanly bracket the FEM response.
//!
//! Production scenes (the passthrough / Hertzian / non-interpenetration / drop-and-rest / grad-hook fixtures + rows 12+13)
//! continue to use the default `(κ, d̂)`; this row's override is local to
//! this example only, never propagated upstream. `KAPPA` stays at the
//! `PENALTY_KAPPA_DEFAULT = 1e4` value (no κ change, only `d̂`).
//!
//! ## Why three refinements
//!
//! The convergence story needs three points: per-level two-bound
//! bracket holds + Cauchy ratio `|Δ_fine| / |Δ_coarse| < 1`. Mirrors
//! The compressive-block fixture's `n_per_edge ∈ {2, 4, 8}` choice. Sub-second
//! release-mode runtime per refinement at the `(d̂, δ)` override
//! (Newton typically converges in `3-5` iters per level; the
//! `MAX_NEWTON_ITER = 50` cap exists as headroom against material /
//! load perturbations rather than as a tight working budget).
//!
//! ## Why `cargo run --release` only
//!
//! Mirrors row 13 + row 12 + the compressive-block fixture precedent. The IV-1
//! captured-bits contract is platform + build-mode-locked; matching
//! row 13's `--release` invocation removes one variable from the
//! determinism contract. The compressive block is fast enough at finest `n=8` that
//! debug-mode is also viable, but consistency with sister rows wins
//! over the cold-build-time savings.
//!
//! ## Visual-mode opt-in: `CF_VISUAL=1`
//!
//! Headless asserts + JSON + PLY ALWAYS run. Setting `CF_VISUAL=1`
//! (any non-empty value) additionally spawns a Bevy app rendering the
//! **finest-refinement settled deformed cube** between **two visual
//! plates** (bottom + top) — a static (single-step) scene with the
//! HUD reading per-refinement ε, F_R_FEM, two-bound bracket, Cauchy
//! ratio, effective modulus, `rel_pos_in_bounds_n8`, and the
//! VIZ_AMPLIFY disclosure line. Pressing `R` is a no-op (no animation
//! to reset); orbit / zoom / pan via mouse work as in rows 12 + 13.
//!
//! **The bottom plate is purely visual** — physics has no penalty
//! contact at the bottom face; the bottom face is BC-pinned in
//! `BoundaryConditions::pinned_vertices`. The visualization shows two
//! plates because the `EXAMPLE_INVENTORY` row 14 framing is "soft cube
//! between two RigidPlanes," and the BC-pin is the nearest-
//! representable analog of a perfectly bonded rigid bottom plate; the
//! second plate carries no penalty force in the physics.
//!
//! **Visualization is amplified** by `VIZ_AMPLIFY = 50×` per banked
//! pattern (b) — the compressive block's small-strain regime (`ε ≈ 0.6 %`) is below
//! human visual acuity at typical viewing distance, so cube
//! deformations from rest are multiplied by `VIZ_AMPLIFY` so the
//! squish becomes visually perceptible (`~30 %` apparent
//! z-compression at finest). The **plates are positioned flush
//! against the (amplified) cube faces** with a sub-mm z-fight offset,
//! NOT at the kinematic `δ`-offset positions the physics solver uses
//! — the penalty model's `sd ≈ 9.78 μm` contact-band would inflate to
//! ~5 cm Bevy under amplification and dominate the visual story; the
//! band is an FEM no-penetration-enforcement implementation detail
//! tangential to the row's "two plates squishing a cube" pedagogy, so
//! it's absorbed into the visualization. **Headless asserts + JSON +
//! PLY are NOT amplified** — they read the true `x_final` from the
//! solver. HUD numbers + JSON + plot.py F-vs-ε scatter still carry
//! the penalty-band physics. The HUD's last line declares the
//! amplification factor so the visual-vs-numerical contract is
//! explicit.
//!
//! ### Why the rendered scene is `100×` simulation scale
//!
//! Mirrors rows 12 + 13's `RENDER_SCALE = 100×` policy verbatim —
//! Bevy 0.18's pipeline defaults (near plane `0.1 m`, OrbitCamera
//! `min_distance = 0.1 m`, AmbientLight brightness, depth precision)
//! were tuned for human-scale (1 m+) scenes; lifting cm-scale physics
//! to meter scale puts everything safely past the defaults. Headless
//! asserts + JSON + PLY are scale-invariant — they operate on
//! unscaled physics positions, so this is visualization-only.
//!
//! ## JSON artifact
//!
//! `out/compressive_block.json` — three sections:
//!
//! 1. `scalars` — `mu`, `lambda`, `nu`, `e_young`, `m_constrained`,
//!    `edge_len`, `displacement`, `kappa`, `d_hat_override`,
//!    `static_dt`, plus per-refinement `(cell_size, lambda_z_avg, eps,
//!    f_r_fem, f_us, f_strain, n_active_pairs, iter_count,
//!    residual_norm)`, plus derived `(cauchy_step_coarse,
//!    cauchy_step_fine, cauchy_ratio, effective_modulus_n8,
//!    rel_pos_in_bounds_n8)`.
//! 2. `vertices` — at finest n=8 only, one entry per active top-face
//!    vertex: `(v, x, y, sd, force_z)` where `force_z = κ · (d̂ − sd)
//!    > 0`.
//! 3. `analytic` — 11-point sample of the two linear bound functions
//!    `F_us(ε) = E · A · ε` and `F_strain(ε) = M_c · A · ε` over
//!    `ε ∈ [0, 1.5 · ε_n8]` for plot.py F-vs-ε overlay.
//!
//! Run `uv run examples/sim-soft/compressive-block/plot.py` to overlay
//! the three FEM scatter points against the analytic bound lines.
//!
//! ## PLY artifact
//!
//! `out/compressive_block.ply` — finest n=8 deformed boundary mesh
//! with per-vertex `contact_force_z` extra (zero for inactive
//! vertices, `κ · (d̂ − sd)` for active top-face vertices). cf-view
//! auto-colormaps the sequential viridis on positive scalar — the
//! contact patch reads as a uniform bright disk on the deformed
//! cube's top face. Open in cf-view: `cargo run -p cf-viewer
//! --release -- <path>`.
//!
//! ## Anchor groups (all assertions exit-0 on success)
//!
//! - **`geometry_invariants`** — compile-time `const { assert!(...) }`
//!   on `EDGE_LEN > 0`, `DISPLACEMENT > 0`, `MU > 0`, `LAMBDA > 0`,
//!   `KAPPA > 0`, `D_HAT_OVERRIDE > 0`, `STATIC_DT > 0`, refinement
//!   ordering (`n2 < n4 < n8`; `h_n2 > h_n4 > h_n8 > 0`),
//!   `DISPLACEMENT < EDGE_LEN` (plate descends INTO not THROUGH the
//!   cube), `MAX_NEWTON_ITER > 0`, `NEWTON_ITER_SANITY_CAP <
//!   MAX_NEWTON_ITER`, `0 < SMALL_STRAIN_CEILING < 1`,
//!   `0 < LAMBDA_Z_FLOOR < 1`.
//! - **`mesh_topology_exact`** — per-refinement exact-pin
//!   `(n_tets, n_vertices, n_loaded, n_pinned)` per the III-1
//!   determinism contract. `n_loaded == 0` exact (compressive-block-specific — no
//!   external traction, load is penalty-mediated).
//! - **`boundary_partition`** — per-refinement `n_pinned > 0` (bottom
//!   face full-pin) and `n_loaded == 0` exact.
//! - **`solver_per_step_invariants`** — per-refinement: no NaN in
//!   `x_final`; `iter_count < NEWTON_ITER_SANITY_CAP = 40` per the compressive-block fixture
//!   iter-margin policy; finite residual norm.
//! - **`contact_engagement`** — `n_active_pairs == (n+1)²` exact per
//!   refinement (every top-face vertex inside the `d̂`-band at
//!   equilibrium per the compressive-block fixture's docstring; stronger than row 13's
//!   `> 0` since the compressive block's geometry guarantees uniform top-face
//!   penetration).
//! - **`small_strain_validity`** — `0 < ε < SMALL_STRAIN_CEILING =
//!   0.10` per refinement (the compressive block expects `ε ≈ 0.6 %` — well into
//!   small-strain).
//! - **`gross_physics_per_level`** — `λ_z ∈ (0.5, 1.0)` (cube
//!   compresses, not extends, and < 50 % strain) + `F_R > 0` (soft
//!   body pushes UP on rigid plane — Newton's 3rd-law partner of
//!   penalty's DOWN force on top face). Catches sign-flip regressions
//!   at gross-physics level before the bound asserts surface them
//!   numerically.
//! - **`two_bound_per_level`** — `F_us ≤ F_R_FEM ≤ F_strain` per
//!   refinement at that level's equilibrium ε. Headline gate.
//! - **`cauchy_f_r_convergence`** — `|F_R_n4 − F_R_n8| <
//!   |F_R_n2 − F_R_n4|` (Cauchy ratio `< 1`; geometric convergence).
//! - **`captured_bits_compressive_metrics`** — IV-1 sparse-tier
//!   rel-tol contract on `lambda_z_avg`, `eps`, `f_r_fem` per
//!   refinement (9 f64), `cauchy_ratio` + `effective_modulus_n8` (2
//!   f64), `iter_count` + `n_active_pairs` per refinement (6 usize).

// `serde_json::json!` macro recurses internally per key-value pair; at
// the 46-key `scalars` block in `save_compressive_json`, the default
// `128` ceiling overflows. `256` doubles headroom; verified via cold
// build with no other consequences on compile time.
#![recursion_limit = "256"]
// PLY field-data is single-precision on disk; converting f64 quantities
// to f32 for the AttributedMesh emit is intrinsic to the PLY format.
// Same precedent as PR1 rows 1+2+3+8+9+10+11 + rows 12+13.
#![allow(clippy::cast_possible_truncation)]
// `usize as u32` casts on `mesh.n_tets()` (~few thousand here, ≪
// u32::MAX) — the standard Mesh-trait API tax.
#![allow(clippy::cast_possible_wrap)]
// `usize as f64` casts for averaging / indexing. Counts ≤ ~thousands
// here, well within f64 mantissa exact range.
#![allow(clippy::cast_precision_loss)]
// `print_summary` and `print_capture_block` are single museum-plaque
// stdout writers; splitting into sub-helpers fragments the visual
// format without information gain. Same allowance as rows 12+13.
#![allow(clippy::too_many_lines)]
// `print_summary` aggregates many scalars-and-collections; threading
// them through a struct adds indirection without information gain.
#![allow(clippy::too_many_arguments)]
// `doc_markdown` flags Unicode math notation (`σ`, `κ`, `λ`, `μ`, `δ`,
// `ν`, `π`) as if they were unbacktick-quoted code identifiers. Same
// allowance as rows 12+13.
#![allow(clippy::doc_markdown)]
// Bevy systems take Resources by value or by `Res<T>`, both flagged as
// pass-by-value. Same allowance as rows 12+13 +
// `sim/L1/sim-bevy-soft/src/trajectory.rs`.
#![allow(clippy::needless_pass_by_value)]
// Wrapped module-docstring paragraphs with continuation lines starting
// with characters like `+ Ecoflex stiffness` or `~219 μm` (Unicode math
// notation in prose) are not Markdown lists; clippy parses them that
// way and requests 4-space indentation. Same allowance as rows 12+13.
#![allow(clippy::doc_lazy_continuation)]
// Per-refinement bindings (`f_r_fem_n2_ref` etc.) carry the same role
// at different resolutions; renaming to avoid the lint would obscure
// the parallel structure with corresponding `*_REF_BITS` constants.
#![allow(clippy::similar_names)]

use std::path::Path;

use anyhow::{Context, Result};
use approx::assert_relative_eq;
// `bevy::prelude::Vec3` (f32 3-vec, Bevy math) vs `sim_soft::Vec3`
// (nalgebra f64 3-vec) collide; same for `bevy::prelude::Mesh` (asset
// struct) vs `sim_soft::Mesh` (tet-mesh trait). Alias both Bevy items
// so the physics-side imports below stay ergonomic for solver inputs.
// Mirror of rows 12+13.
use bevy::prelude::Mesh as BevyMesh;
use bevy::prelude::Vec3 as BevyVec3;
use bevy::prelude::*;
use cf_bevy_common::axis::UpAxis;
use cf_bevy_common::camera::{OrbitCamera, OrbitCameraPlugin};
use mesh_io::save_ply_attributed;
use mesh_types::{AttributedMesh, IndexedMesh, Point3};
use serde_json::{Value, json};
use sim_bevy_soft::mesh::build_soft_mesh;
use sim_bevy_soft::plugin::SoftBodyVisualPlugin;
use sim_bevy_soft::trajectory::Trajectory;
use sim_ml_chassis::Tensor;
use sim_soft::{
    CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh, NewtonStep, PenaltyRigidContact,
    PenaltyRigidContactSolver, RigidPlane, SceneInitial, SoftScene, Solver, SolverConfig, Tet4,
    Vec3, VertexId, pick_vertices_by_predicate,
};

// =============================================================================
// Scene constants — mirror the compressive-block fixture (`sim/L0/soft/tests/penalty_compressive_block.rs`)
// verbatim. Re-deriving here keeps the example self-contained AND
// captures-platform-locked; any regression in the compressive-block helper that
// shifts these implicitly would surface at first row 14 visual review.
// =============================================================================

/// Cube edge length (1 cm). Mirror the compressive block's `EDGE_LEN`.
const EDGE_LEN: f64 = 1.0e-2;

/// Rigid-plane axial displacement (50 μm). Mirror the compressive-block fixture's
/// override of scope memo §9's recommended 0.5 mm — see module
/// docstring "Why the fixture-local (d̂, δ) override" section.
const DISPLACEMENT: f64 = 5.0e-5;

/// Lamé pair `(μ, λ)` — Phase 4 IV-3 / IV-5 / compressive-block default Ecoflex-
/// class compressible NeoHookean (`λ = 4 μ` ⇒ `ν = 0.4`). Pins this row to
/// the rest of the regression net.
const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0e5;

/// Cube cross-sectional area (m²) at rest config. `EDGE_LEN²` = 1 cm²
/// = 1e-4 m². Used in both `F_us` and `F_strain` analytic bounds.
const A_CROSS: f64 = EDGE_LEN * EDGE_LEN;

/// Refinement levels — `n ∈ {2, 4, 8}` cells per cube edge. Mirrors
/// the compressive-block fixture verbatim. Tet counts: `6 · n³ = {48, 384, 3072}`
/// (uniform_block decomposes each unit cell into 6 tets). Locked
/// exact via the III-1 captured-bits block below.
const N_PER_EDGE_N2: usize = 2;
const N_PER_EDGE_N4: usize = 4;
const N_PER_EDGE_N8: usize = 8;

/// Cell size at each refinement (m). `EDGE_LEN / n_per_edge`.
const CELL_SIZE_N2: f64 = EDGE_LEN / 2.0;
const CELL_SIZE_N4: f64 = EDGE_LEN / 4.0;
const CELL_SIZE_N8: f64 = EDGE_LEN / 8.0;

/// Fixture-local penalty stiffness. Pinned at the
/// `sim_soft::contact::penalty::PENALTY_KAPPA_DEFAULT` value
/// (`penalty.rs:57`) — this row's override is
/// scoped to `d̂` (and `δ`); `κ` stays at default. The `pub(crate)`
/// visibility on the upstream constant forces re-pinning here.
const KAPPA: f64 = 1.0e4;

/// Fixture-local penalty contact band (m). **Override of**
/// `PENALTY_DHAT_DEFAULT = 1e-3` (`penalty.rs:65`) per scope memo
/// — see module docstring "Why
/// the fixture-local (d̂, δ) override" section. 100× smaller than default
/// to bring cold-start penalty residual `κ · (d̂ + δ) ≈ 0.6 N` per
/// top-face vertex below the tet-inversion threshold. Production
/// scenes (the passthrough / Hertzian / non-interpenetration / drop-and-rest / grad-hook fixtures + rows 12+13) continue to use
/// the default; this constant only enters via
/// [`PenaltyRigidContact::with_params`] in [`run_at_refinement`] and
/// must NOT be propagated upstream.
const D_HAT_OVERRIDE: f64 = 1.0e-5;

/// Static-equilibrium time-step — large `dt` damps the inertial
/// Tikhonov regulariser `M / dt²` to negligible relative magnitude,
/// yielding pure-static root-find. Mirrors the compressive block verbatim.
const STATIC_DT: f64 = 1.0;

/// Newton iter cap — mirror the compressive block (also matches IV-3 + Hertzian-fixture precedents).
/// Static-equilibrium from rest under the `(d̂, δ)` override
/// typically completes in `3-5` iters per refinement; cap leaves
/// `45+` iters of margin against material / load perturbations.
const MAX_NEWTON_ITER: usize = 50;

/// Per-refinement Newton-iter sanity cap (10-iter margin under
/// `MAX_NEWTON_ITER`). Mirror the compressive block `< 40`. If any refinement spends
/// more than this, surface as a regression before bumping the cap.
const NEWTON_ITER_SANITY_CAP: usize = 40;

/// Small-strain validity ceiling on the equilibrium compressive strain
/// `ε = 1 − λ_z`. The compressive block expects `ε ≈ 0.6 %` — well below the textbook
/// `ε ≲ 5 %` linear-elastic threshold. Cap at `10 %` gives ~17×
/// headroom over expected; failure here means the `(d̂, δ)`
/// override is no longer producing a small-strain regime — investigate
/// before relaxing.
const SMALL_STRAIN_CEILING: f64 = 0.10;

/// Lower-bound on `λ_z` — physical sanity for "less than 50 %
/// compression." Catches gross-physics regressions where the cube has
/// collapsed entirely. The compressive block expects `λ_z ≈ 0.994`.
const LAMBDA_Z_FLOOR: f64 = 0.5;

/// IV-1 sparse-tier rel-tol for captured bits. `~few thousand tets`
/// through faer's sparse Cholesky lives in IV-1's sparse-at-scale
/// tier; static quasi-step layers another arithmetic stage. `1e-12`
/// admits sparse-solver SIMD/FMA noise while catching any real
/// regression. Same precedent as PR1 rows 6+10+11 + rows 12+13.
const SPARSE_REL_TOL: f64 = 1.0e-12;

/// Absolute floor for relative comparisons that touch zero. Below
/// typical compressive-metric magnitudes by 8+ orders of magnitude.
/// Same precedent as PR1 rows 6+10+11 + rows 12+13.
const SPARSE_EPS_ABS: f64 = 1.0e-12;

/// Per-refinement labels for diagnostic stdout + JSON keys.
const LABEL_N2: &str = "n=2";
const LABEL_N4: &str = "n=4";
const LABEL_N8: &str = "n=8";

// =============================================================================
// Exact-pinned mesh counts (III-1 determinism contract) — per refinement.
// Captured 2026-05-06 on macOS arm64, Darwin 25.4.0, stable rustc.
// `HandBuiltTetMesh::uniform_block` decomposes each unit cell into 6
// tets (so `n_tets = 6 · n³`); vertex count is the (n+1)³ grid.
//
// To re-bake: run `cargo run -p example-sim-soft-compressive-block
// --release` and copy the values from the `## Capture-bit reference
// values` block printed at the top of stdout (always runs, even on
// assertion failure).
// =============================================================================

/// Tet count at n=2. `6 · n³ = 48`.
const N_TETS_N2: usize = 48;
/// Tet count at n=4. `6 · n³ = 384`.
const N_TETS_N4: usize = 384;
/// Tet count at n=8. `6 · n³ = 3072`.
const N_TETS_N8: usize = 3072;

/// Vertex count at n=2. `(n+1)³ = 27`.
const N_VERTICES_N2: usize = 27;
/// Vertex count at n=4. `(n+1)³ = 125`.
const N_VERTICES_N4: usize = 125;
/// Vertex count at n=8. `(n+1)³ = 729`.
const N_VERTICES_N8: usize = 729;

/// Loaded vertex count at n=2. **Compressive-block-specific: 0 exact** —
/// `compressive_block_on_plane` helper sets
/// `BoundaryConditions { loaded_vertices: Vec::new(), ... }` because
/// load is penalty-mediated (no external traction).
const N_LOADED_N2: usize = 0;
/// Loaded vertex count at n=4. **0 exact**.
const N_LOADED_N4: usize = 0;
/// Loaded vertex count at n=8. **0 exact**.
const N_LOADED_N8: usize = 0;

/// Bottom-face pinned vertex count at n=2. `(n+1)² = 9` (helper picks
/// all bottom-face vertices, half-cell tolerance).
const N_PINNED_N2: usize = 9;
/// Pin count at n=4. `(n+1)² = 25`.
const N_PINNED_N4: usize = 25;
/// Pin count at n=8. `(n+1)² = 81`.
const N_PINNED_N8: usize = 81;

// =============================================================================
// Captured compressive-metric bits (IV-1 sparse-tier contract).
// Captured 2026-05-06 on macOS arm64, Darwin 25.4.0, stable rustc, in
// `--release` build.
//
// **Failure-mode protocol** (mirrors row 13's): if the rel-tol
// comparison fails, do NOT re-bake. Diagnose in this order:
//   1. Rule out toolchain drift (rustc / LLVM / libm minor version
//      delta vs the capture).
//   2. If same toolchain, real regression — identify which sim-soft
//      commit altered the `SoftScene::compressive_block_on_plane`
//      constructor, the `PenaltyRigidContact::with_params` defaults,
//      the FEM assembly path through faer, OR `HandBuiltTetMesh::
//      uniform_block`'s tet decomposition.
//   3. NEVER re-bake the reference values to make the test green.
// =============================================================================

/// `λ_z_avg` at n=2 — mean axial stretch ratio over top-face vertices,
/// dimensionless. `f64::from_bits(0x3fefd09dfdaa6122) ≈ 0.99422`
/// (`ε ≈ 0.578 %`).
const LAMBDA_Z_AVG_N2_REF_BITS: u64 = 0x3fef_d09d_fdaa_6122;
/// `λ_z_avg` at n=4. `≈ 0.99407` (`ε ≈ 0.593 %`).
const LAMBDA_Z_AVG_N4_REF_BITS: u64 = 0x3fef_cf74_cf2f_8dde;
/// `λ_z_avg` at n=8. `≈ 0.99402` (`ε ≈ 0.598 %`).
const LAMBDA_Z_AVG_N8_REF_BITS: u64 = 0x3fef_cf08_2dd5_ddff;

/// Compressive strain `ε = 1 − λ_z_avg` at n=2 — `≈ 5.784e-3`.
const EPS_N2_REF_BITS: u64 = 0x3f77_b101_2acf_6f00;
/// `ε` at n=4 — `≈ 5.926e-3`.
const EPS_N4_REF_BITS: u64 = 0x3f78_4598_6839_1100;
/// `ε` at n=8 — `≈ 5.978e-3`. Headline equilibrium strain.
const EPS_N8_REF_BITS: u64 = 0x3f78_7be9_1511_0080;

/// FEM-integrated reaction force `F_R_FEM` at n=2 (N) — `≈ 0.1944`.
const F_R_FEM_N2_REF_BITS: u64 = 0x3fc8_e0f2_5f29_1ade;
/// `F_R_FEM` at n=4 — `≈ 0.1856`.
const F_R_FEM_N4_REF_BITS: u64 = 0x3fc7_c2dd_3195_969b;
/// `F_R_FEM` at n=8 — `≈ 0.1819`. Sits in `[F_us(ε_n8) ≈ 0.167 N,
/// F_strain(ε_n8) ≈ 0.359 N]` (close to F_us per Saint-Venant
/// boundary-layer argument: bottom-pin lateral constraint confined to
/// thin layer; cube interior in uniaxial-stress regime).
const F_R_FEM_N8_REF_BITS: u64 = 0x3fc7_47f5_d85c_59cb;

/// Cauchy ratio `|F_R_n4 − F_R_n8| / |F_R_n2 − F_R_n4|` — `≈ 0.4296`.
/// Comfortably below `1.0`; geometric convergence demonstrated.
const CAUCHY_RATIO_REF_BITS: u64 = 0x3fdb_7eb6_9dba_5bd5;

/// Effective modulus at n=8: `F_R_FEM_n8 / (A · ε_n8)` — `≈ 3.043e5
/// Pa`. Sits in `[E = 2.8e5, M_c = 6.0e5]` close to E
/// (uniaxial-stress regime for the cube interior); `rel_pos_in_bounds
/// ≈ 0.076`.
const EFFECTIVE_MODULUS_N8_REF_BITS: u64 = 0x4112_9258_06e1_6c93;

/// Active-pair count at n=2. **`(n+1)² = 9` exact** — every top-face
/// vertex inside the `d̂`-band at equilibrium per the compressive-block fixture's
/// docstring.
const N_ACTIVE_N2_REF: usize = 9;
/// Active-pair count at n=4. **`(n+1)² = 25` exact**.
const N_ACTIVE_N4_REF: usize = 25;
/// Active-pair count at n=8. **`(n+1)² = 81` exact**.
const N_ACTIVE_N8_REF: usize = 81;

/// Newton iter count at n=2 — `3` iters at the `(d̂, δ)`
/// override regime (cold-start residual `~0.6 N` per top-face vertex,
/// raw step ≪ tet-inversion threshold).
const ITER_COUNT_N2_REF: usize = 3;
/// Newton iter count at n=4 — `3` iters.
const ITER_COUNT_N4_REF: usize = 3;
/// Newton iter count at n=8 — `3` iters. Flat across refinements at
/// the `(d̂, δ)` override (active-set size grows but per-iter
/// residual decreases at the same Newton rate).
const ITER_COUNT_N8_REF: usize = 3;

// =============================================================================
// Helpers — math (NeoHookean → small-strain bound functions)
// =============================================================================

/// Young's modulus from Lamé pair: `E = μ (3 λ + 2 μ) / (λ + μ)`.
/// At canonical `(1e5, 4e5)`: `E = 2.8e5 Pa = 2 μ (1 + ν) =
/// 2 · 1e5 · 1.4`.
const fn young_modulus(mu: f64, lambda: f64) -> f64 {
    // const fn doesn't allow `mul_add`; expand explicitly. The formula
    // is exact for compressible isotropic linear elastic.
    mu * (3.0 * lambda + 2.0 * mu) / (lambda + mu)
}

/// Constrained modulus (1D dilatational stiffness):
/// `M_c = E · (1 - ν) / ((1 + ν)(1 - 2 ν)) = λ + 2 μ`. The second
/// equality is exact for isotropic linear elastic — so this is just
/// `λ + 2 μ = 6.0e5 Pa` at the canonical pair.
const fn constrained_modulus(mu: f64, lambda: f64) -> f64 {
    lambda + 2.0 * mu
}

/// Poisson's ratio from Lamé pair: `ν = λ / (2 (λ + μ))`. At canonical
/// `(1e5, 4e5)`: `ν = 0.4`.
const fn poisson_ratio(mu: f64, lambda: f64) -> f64 {
    lambda / (2.0 * (lambda + mu))
}

/// Uniaxial-stress small-strain reaction force at compressive strain
/// `ε` (positive in compression): `F_us = E · A · ε`. Lower bound for
/// the mixed-BC FEM response — see module docstring "Why
/// `F_R ∈ [F_us, F_strain]` is the headline" section.
fn uniaxial_stress_reaction(epsilon: f64) -> f64 {
    young_modulus(MU, LAMBDA) * A_CROSS * epsilon
}

/// Uniaxial-strain small-strain reaction force at compressive strain
/// `ε`: `F_strain = M_c · A · ε`. Upper bound for the mixed-BC FEM
/// response.
fn uniaxial_strain_reaction(epsilon: f64) -> f64 {
    constrained_modulus(MU, LAMBDA) * A_CROSS * epsilon
}

fn material_field() -> MaterialField {
    MaterialField::uniform(MU, LAMBDA)
}

// =============================================================================
// Snapshot — captured solver outputs at one refinement level
// =============================================================================

/// One active top-face contact-vertex sample at the finest refinement,
/// used by the JSON + PLY emit paths.
#[derive(Clone, Debug)]
struct ContactVertex {
    /// Vertex id in the FEM mesh.
    v: VertexId,
    /// Final x-coordinate at converged `x_final` (m).
    x: f64,
    /// Final y-coordinate (m).
    y: f64,
    /// Plane signed distance: `sd = EDGE_LEN − DISPLACEMENT − z_v`.
    /// Active when `sd < D_HAT_OVERRIDE`. The compressive block expects
    /// `sd ≈ 9.8 μm` for the average top-face vertex at finest
    /// equilibrium (just under `D_HAT_OVERRIDE = 10 μm`); per-vertex
    /// values vary by sub-micron amounts (~0.25 μm) around the mean
    /// depending on corner-vs-interior position (interior vertices
    /// have lower sd / deeper d̂-band penetration than corners — see
    /// row 18 `contact-force-readout` for the per-pair distribution
    /// pattern). Banked from row 18's `b3614811` sub-micron fix.
    sd: f64,
    /// Penalty force z-component on the vertex: `force_z = κ ·
    /// (d̂_override − sd)`. Positive (rigid pushes DOWN on soft;
    /// Newton's 3rd-law partner is soft pushing UP on rigid).
    force_z: f64,
}

/// Per-refinement run output. Carries FEM-measured compressive
/// quantities + Newton diagnostics + mesh stats for the diagnostic
/// stdout + JSON + PLY paths. Mirrors the compressive-block fixture's `StepReport` plus
/// the extras the user-facing example wants (`x_final`,
/// `rest_positions`, `boundary_faces`, `contact_vertices`).
struct RefinementSnapshot {
    /// Cell size at this refinement (m).
    cell_size: f64,
    /// Diagnostic label (`n=2` / `n=4` / `n=8`).
    label: &'static str,
    /// Refinement parameter `n`. Used by the
    /// `n_active_pairs == (n+1)²` exact-engagement check.
    n_per_edge: usize,
    /// Mean axial stretch ratio: `(z_eq_avg over top face) / EDGE_LEN`.
    /// `λ_z < 1` for compression.
    lambda_z_avg: f64,
    /// Compressive strain `ε = 1 − λ_z_avg`. Positive in compression.
    eps: f64,
    /// FEM-integrated reaction force on the rigid plane (Newton's
    /// 3rd-law partner of the penalty force on the top face).
    /// Positive when the soft body pushes UP on the rigid plane.
    f_r_fem: f64,
    /// Uniaxial-stress small-strain bound at this `ε`: `E · A · ε`.
    /// Lower bound on `f_r_fem`.
    f_us: f64,
    /// Uniaxial-strain small-strain bound at this `ε`: `M_c · A · ε`.
    /// Upper bound on `f_r_fem`.
    f_strain: f64,
    /// Active-pair count at converged `x_final`. Exact `(n+1)²` per
    /// the compressive-block fixture's docstring (every top-face vertex inside band).
    n_active_pairs: usize,
    /// Newton iter count at convergence.
    iter_count: usize,
    /// Free-DOF residual norm at convergence.
    residual_norm: f64,
    /// Total tet count.
    n_tets: usize,
    /// Total mesh vertex count. `HandBuiltTetMesh::uniform_block` has
    /// no orphan-bbox-margin (unlike `SdfMeshedTetMesh`); every vertex
    /// is referenced by ≥ 1 tet.
    n_vertices: usize,
    /// External-traction loaded vertex count. **0 exact** for the compressive block (no
    /// external load).
    n_loaded: usize,
    /// Bottom-face pinned vertex count. `(n+1)²` per `helper`.
    n_pinned: usize,
    /// Converged `x_final` (vertex-major + xyz-inner DOF layout).
    x_final: Vec<f64>,
    /// Rest-configuration positions (mesh.positions() snapshot — used
    /// for the Bevy build_soft_mesh's initial pose at finest).
    rest_positions: Vec<Vec3>,
    /// Boundary-face triangulation (cached by `Mesh::boundary_faces`).
    boundary_faces: Vec<[VertexId; 3]>,
    /// Active top-face contact vertices at finest. Empty for non-finest
    /// refinements (we don't emit per-vertex JSON / PLY at n=2 or n=4).
    contact_vertices: Vec<ContactVertex>,
}

/// Aggregate snapshot across all three refinements + NH-derived
/// scalars + Cauchy convergence + effective-modulus diagnostics.
struct CompressiveSnapshot {
    /// Coarsest refinement (n=2).
    n2: RefinementSnapshot,
    /// Mid refinement (n=4).
    n4: RefinementSnapshot,
    /// Finest refinement (n=8) — has populated `contact_vertices`.
    n8: RefinementSnapshot,
    /// Young's modulus E at canonical (μ, λ) (Pa).
    e_young: f64,
    /// Constrained modulus M_c (Pa).
    m_constrained: f64,
    /// Poisson's ratio ν (dimensionless).
    nu: f64,
    /// `|F_R_n2 − F_R_n4|` (N).
    cauchy_step_coarse: f64,
    /// `|F_R_n4 − F_R_n8|` (N).
    cauchy_step_fine: f64,
    /// Cauchy ratio `cauchy_step_fine / cauchy_step_coarse`. Asserted
    /// `< 1`.
    cauchy_ratio: f64,
    /// Effective modulus at finest: `F_R_n8 / (A · ε_n8)` (Pa). Should
    /// sit in `[E, M_c]`.
    effective_modulus_n8: f64,
    /// Relative position of effective modulus in `[E, M_c]`:
    /// `(effective_modulus_n8 - E) / (M_c - E)`. Closer to 0 means
    /// closer to uniaxial-stress (lateral free), closer to 1 means
    /// closer to uniaxial-strain (lateral pinned).
    rel_pos_in_bounds_n8: f64,
}

// =============================================================================
// Run — single refinement
// =============================================================================

/// Build the compressive-block scene at `n_per_edge`, replace the
/// helper's default-`d̂` contact with a fixture-local `D_HAT_OVERRIDE`
/// override, run a single static `replay_step`, and capture the
/// [`RefinementSnapshot`].
///
/// `populate_contact_vertices` is `true` only at the finest refinement
/// — the per-vertex sample list is consumed by the JSON + PLY emit
/// paths, both of which only run at n=8.
fn run_at_refinement(
    n_per_edge: usize,
    label: &'static str,
    populate_contact_vertices: bool,
) -> RefinementSnapshot {
    // Cast safe: `n_per_edge` is `{2, 4, 8}`; conversion to f64 is
    // loss-free.
    let cell_size = EDGE_LEN / (n_per_edge as f64);

    // Helper builds mesh + BC + initial via commit-6 scaffolding; its
    // returned `default_contact` (default `κ`, `d̂`) is discarded and
    // replaced with a `with_params` override per the module docstring's
    // "Why the fixture-local (d̂, δ) override" section. The plane is
    // reconstructed identically to the helper's `RigidPlane::new(
    // Vec3::new(0.0, 0.0, -1.0), DISPLACEMENT - EDGE_LEN)`
    // (`scene.rs:450`).
    let (mesh, bc, initial, _default_contact) =
        SoftScene::compressive_block_on_plane(EDGE_LEN, cell_size, DISPLACEMENT, &material_field());
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), DISPLACEMENT - EDGE_LEN);
    let contact = PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT_OVERRIDE);

    let n_tets = mesh.n_tets();
    let n_vertices = mesh.n_vertices();
    let n_loaded = bc.loaded_vertices.len();
    let n_pinned = bc.pinned_vertices.len();

    // Snapshot top-face vertex IDs (rest config `z = EDGE_LEN`,
    // half-cell tolerance) BEFORE moving the mesh into the solver.
    // Mirror the compressive-block fixture's idiom — only top-face vertices can enter
    // the contact band (`sd < D_HAT_OVERRIDE`), so iterating just the
    // top face captures every active pair.
    let band_tol = 0.5 * cell_size;
    let top_face_vertices: Vec<VertexId> =
        pick_vertices_by_predicate(&mesh, |p| (p.z - EDGE_LEN).abs() < band_tol);
    assert!(
        !top_face_vertices.is_empty(),
        "{label}: top face must contain at least one vertex at refinement n = {n_per_edge}",
    );

    // Snapshot boundary-faces + rest_positions BEFORE the solver moves
    // the mesh. Both are used by the finest-only PLY + Bevy paths.
    let boundary_faces: Vec<[VertexId; 3]> = mesh.boundary_faces().to_vec();
    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();

    let SceneInitial { x_prev, v_prev } = initial;

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;

    let solver: PenaltyRigidContactSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    // No external traction (helper's `loaded_vertices` is empty); θ is
    // length-0 to match BC per `assemble_external_force`'s
    // empty-loaded-vs-θ-length assert. The contact load enters via the
    // penalty model; `step` is driven by penalty alone.
    let empty_theta: [f64; 0] = [];
    let theta_tensor = Tensor::from_slice(&empty_theta, &[0]);
    let step: NewtonStep<_> = solver.replay_step(&x_prev, &v_prev, &theta_tensor, cfg.dt);

    // λ_z = (mean z_eq over top face) / L. Top face was at z = L at
    // rest; compression drops them to z_eq < L → λ_z < 1.
    let z_sum: f64 = top_face_vertices
        .iter()
        .map(|&v| step.x_final[3 * v as usize + 2])
        .sum();
    let lambda_z_avg = (z_sum / top_face_vertices.len() as f64) / EDGE_LEN;
    let eps = 1.0 - lambda_z_avg;
    let f_us = uniaxial_stress_reaction(eps);
    let f_strain = uniaxial_strain_reaction(eps);

    // FEM reaction force = sum over active top-face pairs of penalty
    // gradient.z. Manual reconstruction from the known plane geometry
    // (`normal = -ẑ`, `offset = DISPLACEMENT - EDGE_LEN`); see the compressive-block
    // fixture's "Reaction-force extraction" section. The bottom-face
    // pinned vertices stay at `z = 0`, giving `sd = EDGE_LEN -
    // DISPLACEMENT ≈ 1e-2 m ≫ D_HAT_OVERRIDE = 1e-5 m`, never active —
    // so iterating only top-face vertices captures every active pair.
    let mut f_r_fem = 0.0;
    let mut n_active_pairs = 0;
    let mut contact_vertices: Vec<ContactVertex> = Vec::new();
    for &v in &top_face_vertices {
        let v_idx = v as usize;
        let x_v = step.x_final[3 * v_idx];
        let y_v = step.x_final[3 * v_idx + 1];
        let z_v = step.x_final[3 * v_idx + 2];
        let sd = EDGE_LEN - DISPLACEMENT - z_v;
        if sd < D_HAT_OVERRIDE {
            let force_z = KAPPA * (D_HAT_OVERRIDE - sd);
            f_r_fem += force_z;
            n_active_pairs += 1;
            if populate_contact_vertices {
                contact_vertices.push(ContactVertex {
                    v,
                    x: x_v,
                    y: y_v,
                    sd,
                    force_z,
                });
            }
        }
    }

    RefinementSnapshot {
        cell_size,
        label,
        n_per_edge,
        lambda_z_avg,
        eps,
        f_r_fem,
        f_us,
        f_strain,
        n_active_pairs,
        iter_count: step.iter_count,
        residual_norm: step.final_residual_norm,
        n_tets,
        n_vertices,
        n_loaded,
        n_pinned,
        x_final: step.x_final,
        rest_positions,
        boundary_faces,
        contact_vertices,
    }
}

/// Run all three refinements and assemble the [`CompressiveSnapshot`].
fn run_compressive_sweep() -> CompressiveSnapshot {
    let n2 = run_at_refinement(N_PER_EDGE_N2, LABEL_N2, false);
    let n4 = run_at_refinement(N_PER_EDGE_N4, LABEL_N4, false);
    let n8 = run_at_refinement(N_PER_EDGE_N8, LABEL_N8, true);

    let e_young = young_modulus(MU, LAMBDA);
    let m_constrained = constrained_modulus(MU, LAMBDA);
    let nu = poisson_ratio(MU, LAMBDA);

    let cauchy_step_coarse = (n2.f_r_fem - n4.f_r_fem).abs();
    let cauchy_step_fine = (n4.f_r_fem - n8.f_r_fem).abs();
    let cauchy_ratio = if cauchy_step_coarse > 0.0 {
        cauchy_step_fine / cauchy_step_coarse
    } else {
        f64::INFINITY
    };

    let effective_modulus_n8 = n8.f_r_fem / (A_CROSS * n8.eps);
    let rel_pos_in_bounds_n8 = (effective_modulus_n8 - e_young) / (m_constrained - e_young);

    CompressiveSnapshot {
        n2,
        n4,
        n8,
        e_young,
        m_constrained,
        nu,
        cauchy_step_coarse,
        cauchy_step_fine,
        cauchy_ratio,
        effective_modulus_n8,
        rel_pos_in_bounds_n8,
    }
}

// =============================================================================
// Capture-bit dump — always runs first so failure-mode protocol has the
// actuals visible even when later assertions panic.
// =============================================================================

fn print_capture_block(snapshot: &CompressiveSnapshot) {
    eprintln!();
    eprintln!("==== Capture-bit reference values (paste into source on first bake) ====");
    eprintln!();
    eprintln!("// Topology exact-pins");
    eprintln!("const N_TETS_N2: usize       = {:>8};", snapshot.n2.n_tets);
    eprintln!("const N_TETS_N4: usize       = {:>8};", snapshot.n4.n_tets);
    eprintln!("const N_TETS_N8: usize       = {:>8};", snapshot.n8.n_tets);
    eprintln!(
        "const N_VERTICES_N2: usize   = {:>8};",
        snapshot.n2.n_vertices
    );
    eprintln!(
        "const N_VERTICES_N4: usize   = {:>8};",
        snapshot.n4.n_vertices
    );
    eprintln!(
        "const N_VERTICES_N8: usize   = {:>8};",
        snapshot.n8.n_vertices
    );
    eprintln!(
        "const N_LOADED_N2: usize     = {:>8};",
        snapshot.n2.n_loaded
    );
    eprintln!(
        "const N_LOADED_N4: usize     = {:>8};",
        snapshot.n4.n_loaded
    );
    eprintln!(
        "const N_LOADED_N8: usize     = {:>8};",
        snapshot.n8.n_loaded
    );
    eprintln!(
        "const N_PINNED_N2: usize     = {:>8};",
        snapshot.n2.n_pinned
    );
    eprintln!(
        "const N_PINNED_N4: usize     = {:>8};",
        snapshot.n4.n_pinned
    );
    eprintln!(
        "const N_PINNED_N8: usize     = {:>8};",
        snapshot.n8.n_pinned
    );
    eprintln!();
    eprintln!("// Compressive-metric captured bits");
    eprintln!(
        "const LAMBDA_Z_AVG_N2_REF_BITS: u64    = 0x{:016x}; // λ_z_avg n=2 = {:e}",
        snapshot.n2.lambda_z_avg.to_bits(),
        snapshot.n2.lambda_z_avg,
    );
    eprintln!(
        "const LAMBDA_Z_AVG_N4_REF_BITS: u64    = 0x{:016x}; // λ_z_avg n=4 = {:e}",
        snapshot.n4.lambda_z_avg.to_bits(),
        snapshot.n4.lambda_z_avg,
    );
    eprintln!(
        "const LAMBDA_Z_AVG_N8_REF_BITS: u64    = 0x{:016x}; // λ_z_avg n=8 = {:e}",
        snapshot.n8.lambda_z_avg.to_bits(),
        snapshot.n8.lambda_z_avg,
    );
    eprintln!(
        "const EPS_N2_REF_BITS: u64             = 0x{:016x}; // ε n=2       = {:e}",
        snapshot.n2.eps.to_bits(),
        snapshot.n2.eps,
    );
    eprintln!(
        "const EPS_N4_REF_BITS: u64             = 0x{:016x}; // ε n=4       = {:e}",
        snapshot.n4.eps.to_bits(),
        snapshot.n4.eps,
    );
    eprintln!(
        "const EPS_N8_REF_BITS: u64             = 0x{:016x}; // ε n=8       = {:e}",
        snapshot.n8.eps.to_bits(),
        snapshot.n8.eps,
    );
    eprintln!(
        "const F_R_FEM_N2_REF_BITS: u64         = 0x{:016x}; // F_R n=2     = {:e} N",
        snapshot.n2.f_r_fem.to_bits(),
        snapshot.n2.f_r_fem,
    );
    eprintln!(
        "const F_R_FEM_N4_REF_BITS: u64         = 0x{:016x}; // F_R n=4     = {:e} N",
        snapshot.n4.f_r_fem.to_bits(),
        snapshot.n4.f_r_fem,
    );
    eprintln!(
        "const F_R_FEM_N8_REF_BITS: u64         = 0x{:016x}; // F_R n=8     = {:e} N",
        snapshot.n8.f_r_fem.to_bits(),
        snapshot.n8.f_r_fem,
    );
    eprintln!(
        "const CAUCHY_RATIO_REF_BITS: u64       = 0x{:016x}; // Cauchy      = {:e}",
        snapshot.cauchy_ratio.to_bits(),
        snapshot.cauchy_ratio,
    );
    eprintln!(
        "const EFFECTIVE_MODULUS_N8_REF_BITS: u64 = 0x{:016x}; // E_eff n=8   = {:e} Pa",
        snapshot.effective_modulus_n8.to_bits(),
        snapshot.effective_modulus_n8,
    );
    eprintln!(
        "const N_ACTIVE_N2_REF: usize           = {:>8};                       // n_active n=2",
        snapshot.n2.n_active_pairs,
    );
    eprintln!(
        "const N_ACTIVE_N4_REF: usize           = {:>8};                       // n_active n=4",
        snapshot.n4.n_active_pairs,
    );
    eprintln!(
        "const N_ACTIVE_N8_REF: usize           = {:>8};                       // n_active n=8",
        snapshot.n8.n_active_pairs,
    );
    eprintln!(
        "const ITER_COUNT_N2_REF: usize         = {:>8};                       // iter n=2",
        snapshot.n2.iter_count,
    );
    eprintln!(
        "const ITER_COUNT_N4_REF: usize         = {:>8};                       // iter n=4",
        snapshot.n4.iter_count,
    );
    eprintln!(
        "const ITER_COUNT_N8_REF: usize         = {:>8};                       // iter n=8",
        snapshot.n8.iter_count,
    );
    eprintln!();
    eprintln!("==== End capture-bit block ====");
    eprintln!();
}

// =============================================================================
// Anchor 1 — geometry_invariants (compile-time)
// =============================================================================

const fn verify_geometry_invariants() {
    const { assert!(EDGE_LEN > 0.0, "EDGE_LEN must be positive") };
    const { assert!(DISPLACEMENT > 0.0, "DISPLACEMENT must be positive") };
    const { assert!(MU > 0.0, "MU must be positive") };
    const { assert!(LAMBDA > 0.0, "LAMBDA must be positive") };
    const { assert!(KAPPA > 0.0, "KAPPA must be positive") };
    const { assert!(D_HAT_OVERRIDE > 0.0, "D_HAT_OVERRIDE must be positive") };
    const { assert!(STATIC_DT > 0.0, "STATIC_DT must be positive") };
    const {
        assert!(
            N_PER_EDGE_N2 < N_PER_EDGE_N4 && N_PER_EDGE_N4 < N_PER_EDGE_N8,
            "Refinement levels must satisfy n2 < n4 < n8 (cell sizes decreasing)",
        );
    };
    const {
        assert!(
            CELL_SIZE_N2 > CELL_SIZE_N4 && CELL_SIZE_N4 > CELL_SIZE_N8 && CELL_SIZE_N8 > 0.0,
            "Cell sizes must satisfy h_n2 > h_n4 > h_n8 > 0",
        );
    };
    const {
        assert!(
            DISPLACEMENT < EDGE_LEN,
            "DISPLACEMENT must be less than EDGE_LEN (plate descends INTO the cube, not THROUGH)",
        );
    };
    const {
        assert!(MAX_NEWTON_ITER > 0, "MAX_NEWTON_ITER must be positive");
    };
    const {
        assert!(
            NEWTON_ITER_SANITY_CAP < MAX_NEWTON_ITER,
            "NEWTON_ITER_SANITY_CAP must leave margin under MAX_NEWTON_ITER",
        );
    };
    const {
        assert!(
            SMALL_STRAIN_CEILING > 0.0 && SMALL_STRAIN_CEILING < 1.0,
            "SMALL_STRAIN_CEILING must be in (0, 1)",
        );
    };
    const {
        assert!(
            LAMBDA_Z_FLOOR > 0.0 && LAMBDA_Z_FLOOR < 1.0,
            "LAMBDA_Z_FLOOR must be in (0, 1)",
        );
    };
}

// =============================================================================
// Anchor 2 — mesh_topology_exact (per-refinement III-1 contract)
// =============================================================================

fn verify_mesh_topology_exact(snapshot: &CompressiveSnapshot) {
    let triples: [(&RefinementSnapshot, usize, usize, usize, usize); 3] = [
        (
            &snapshot.n2,
            N_TETS_N2,
            N_VERTICES_N2,
            N_LOADED_N2,
            N_PINNED_N2,
        ),
        (
            &snapshot.n4,
            N_TETS_N4,
            N_VERTICES_N4,
            N_LOADED_N4,
            N_PINNED_N4,
        ),
        (
            &snapshot.n8,
            N_TETS_N8,
            N_VERTICES_N8,
            N_LOADED_N8,
            N_PINNED_N8,
        ),
    ];
    for (rs, n_tets_ref, n_vertices_ref, n_loaded_ref, n_pinned_ref) in triples {
        assert_eq!(
            rs.n_tets, n_tets_ref,
            "{}: n_tets drift — expected {n_tets_ref}, got {} (III-1 determinism contract violated)",
            rs.label, rs.n_tets,
        );
        assert_eq!(
            rs.n_vertices, n_vertices_ref,
            "{}: n_vertices drift — expected {n_vertices_ref}, got {}",
            rs.label, rs.n_vertices,
        );
        assert_eq!(
            rs.n_loaded, n_loaded_ref,
            "{}: n_loaded drift — expected {n_loaded_ref}, got {} (the compressive block expects 0 exact — \
             load is penalty-mediated, no external traction)",
            rs.label, rs.n_loaded,
        );
        assert_eq!(
            rs.n_pinned, n_pinned_ref,
            "{}: n_pinned drift — expected {n_pinned_ref}, got {}",
            rs.label, rs.n_pinned,
        );
    }
}

// =============================================================================
// Anchor 3 — boundary_partition (per-refinement)
// =============================================================================

fn verify_boundary_partition(snapshot: &CompressiveSnapshot) {
    for rs in [&snapshot.n2, &snapshot.n4, &snapshot.n8] {
        assert!(
            rs.n_pinned > 0,
            "{}: bottom-face pin set is empty — `compressive_block_on_plane` helper picks all \
             bottom-face vertices via `pick_vertices_by_predicate(|p| p.z.abs() < band_tol)`; \
             empty set means the helper failed to enumerate any vertex at `z ≈ 0`",
            rs.label,
        );
        assert_eq!(
            rs.n_loaded, 0,
            "{}: the compressive block expects no external traction — load is penalty-mediated; \
             `BoundaryConditions::loaded_vertices` must be empty (got {})",
            rs.label, rs.n_loaded,
        );
    }
}

// =============================================================================
// Anchor 4 — solver_per_step_invariants (per-refinement)
// =============================================================================

fn verify_solver_per_step_invariants(snapshot: &CompressiveSnapshot) {
    for rs in [&snapshot.n2, &snapshot.n4, &snapshot.n8] {
        for (i, &x) in rs.x_final.iter().enumerate() {
            assert!(
                x.is_finite(),
                "{}: x_final[{i}] = {x} is not finite — Newton diverged",
                rs.label,
            );
        }
        assert!(
            rs.iter_count < NEWTON_ITER_SANITY_CAP,
            "{}: Newton ran {} iters, ≥ sanity cap {NEWTON_ITER_SANITY_CAP} \
             (= MAX_NEWTON_ITER {MAX_NEWTON_ITER} − 10) — investigate solver / penalty regime \
             regression before bumping the cap",
            rs.label,
            rs.iter_count,
        );
        assert!(
            rs.residual_norm.is_finite(),
            "{}: residual norm = {} is not finite — Newton diverged",
            rs.label,
            rs.residual_norm,
        );
    }
}

// =============================================================================
// Anchor 5 — contact_engagement (n_active == (n+1)² exact per refinement)
// =============================================================================

fn verify_contact_engagement(snapshot: &CompressiveSnapshot) {
    for rs in [&snapshot.n2, &snapshot.n4, &snapshot.n8] {
        let expected = (rs.n_per_edge + 1) * (rs.n_per_edge + 1);
        assert_eq!(
            rs.n_active_pairs, expected,
            "{}: n_active_pairs = {} should equal (n+1)² = {expected} (every top-face vertex \
             inside the d̂-band at equilibrium per the compressive-block fixture's docstring)",
            rs.label, rs.n_active_pairs,
        );
    }
}

// =============================================================================
// Anchor 6 — small_strain_validity (ε < SMALL_STRAIN_CEILING per refinement)
// =============================================================================

fn verify_small_strain_validity(snapshot: &CompressiveSnapshot) {
    for rs in [&snapshot.n2, &snapshot.n4, &snapshot.n8] {
        assert!(
            rs.eps > 0.0,
            "{}: ε = {} should be positive (compression); negative indicates extension or \
             sign flip",
            rs.label,
            rs.eps,
        );
        assert!(
            rs.eps < SMALL_STRAIN_CEILING,
            "{}: ε = {:.4} ≥ small-strain ceiling {SMALL_STRAIN_CEILING:.2} — the `(d̂, δ)` \
             override no longer producing small-strain regime; investigate before relaxing",
            rs.label,
            rs.eps,
        );
    }
}

// =============================================================================
// Anchor 7 — gross_physics_per_level (λ_z plausible + F_R sign)
// =============================================================================

fn verify_gross_physics_per_level(snapshot: &CompressiveSnapshot) {
    for rs in [&snapshot.n2, &snapshot.n4, &snapshot.n8] {
        assert!(
            rs.lambda_z_avg < 1.0 && rs.lambda_z_avg > LAMBDA_Z_FLOOR,
            "{}: λ_z = {:.6} out of plausible compression range ({LAMBDA_Z_FLOOR:.2}, 1.0) — \
             sign-flip or contact-machinery regression",
            rs.label,
            rs.lambda_z_avg,
        );
        assert!(
            rs.f_r_fem > 0.0,
            "{}: F_R = {:e} N should be positive (soft body pushes UP on rigid plane); \
             negative reaction is a sign-convention regression",
            rs.label,
            rs.f_r_fem,
        );
    }
}

// =============================================================================
// Anchor 8 — two_bound_per_level (F_us ≤ F_R ≤ F_strain at each ε)
// =============================================================================

fn verify_two_bound_per_level(snapshot: &CompressiveSnapshot) {
    for rs in [&snapshot.n2, &snapshot.n4, &snapshot.n8] {
        assert!(
            rs.f_r_fem >= rs.f_us,
            "{}: F_R = {:.4e} N below uniaxial-stress lower bound F_us = {:.4e} N \
             (ε = {:.4e}); mixed BC should be at least as stiff as pure uniaxial-stress \
             (adding the bottom-pin lateral constraint can only stiffen the response)",
            rs.label,
            rs.f_r_fem,
            rs.f_us,
            rs.eps,
        );
        assert!(
            rs.f_r_fem <= rs.f_strain,
            "{}: F_R = {:.4e} N above uniaxial-strain upper bound F_strain = {:.4e} N \
             (ε = {:.4e}); mixed BC should be at most as stiff as pure uniaxial-strain \
             (removing lateral pin from sides can only soften the response)",
            rs.label,
            rs.f_r_fem,
            rs.f_strain,
            rs.eps,
        );
    }
}

// =============================================================================
// Anchor 9 — cauchy_f_r_convergence (Cauchy ratio < 1 on F_R sequence)
// =============================================================================

fn verify_cauchy_f_r_convergence(snapshot: &CompressiveSnapshot) {
    assert!(
        snapshot.cauchy_ratio < 1.0,
        "Cauchy ratio (F_R) {:.4} (|Δ_fine| / |Δ_coarse|) ≥ 1.0 — F_R sequence is not \
         geometrically converging across (n=2, n=4, n=8). |F_R_n2 - F_R_n4| = {:.4e} N, \
         |F_R_n4 - F_R_n8| = {:.4e} N",
        snapshot.cauchy_ratio,
        snapshot.cauchy_step_coarse,
        snapshot.cauchy_step_fine,
    );
}

// =============================================================================
// Anchor 10 — captured_bits_compressive_metrics (IV-1 sparse-tier rel-tol)
// =============================================================================

fn verify_captured_bits_compressive_metrics(snapshot: &CompressiveSnapshot) {
    let lambda_z_avg_n2_ref = f64::from_bits(LAMBDA_Z_AVG_N2_REF_BITS);
    let lambda_z_avg_n4_ref = f64::from_bits(LAMBDA_Z_AVG_N4_REF_BITS);
    let lambda_z_avg_n8_ref = f64::from_bits(LAMBDA_Z_AVG_N8_REF_BITS);
    let eps_n2_ref = f64::from_bits(EPS_N2_REF_BITS);
    let eps_n4_ref = f64::from_bits(EPS_N4_REF_BITS);
    let eps_n8_ref = f64::from_bits(EPS_N8_REF_BITS);
    let f_r_fem_n2_ref = f64::from_bits(F_R_FEM_N2_REF_BITS);
    let f_r_fem_n4_ref = f64::from_bits(F_R_FEM_N4_REF_BITS);
    let f_r_fem_n8_ref = f64::from_bits(F_R_FEM_N8_REF_BITS);
    let cauchy_ratio_ref = f64::from_bits(CAUCHY_RATIO_REF_BITS);
    let effective_modulus_n8_ref = f64::from_bits(EFFECTIVE_MODULUS_N8_REF_BITS);

    assert_relative_eq!(
        snapshot.n2.lambda_z_avg,
        lambda_z_avg_n2_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.n4.lambda_z_avg,
        lambda_z_avg_n4_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.n8.lambda_z_avg,
        lambda_z_avg_n8_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.n2.eps,
        eps_n2_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.n4.eps,
        eps_n4_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.n8.eps,
        eps_n8_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.n2.f_r_fem,
        f_r_fem_n2_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.n4.f_r_fem,
        f_r_fem_n4_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.n8.f_r_fem,
        f_r_fem_n8_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.cauchy_ratio,
        cauchy_ratio_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.effective_modulus_n8,
        effective_modulus_n8_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_eq!(
        snapshot.n2.n_active_pairs, N_ACTIVE_N2_REF,
        "n_active drift at n=2: expected {N_ACTIVE_N2_REF}, got {}",
        snapshot.n2.n_active_pairs,
    );
    assert_eq!(
        snapshot.n4.n_active_pairs, N_ACTIVE_N4_REF,
        "n_active drift at n=4: expected {N_ACTIVE_N4_REF}, got {}",
        snapshot.n4.n_active_pairs,
    );
    assert_eq!(
        snapshot.n8.n_active_pairs, N_ACTIVE_N8_REF,
        "n_active drift at n=8: expected {N_ACTIVE_N8_REF}, got {}",
        snapshot.n8.n_active_pairs,
    );
    assert_eq!(
        snapshot.n2.iter_count, ITER_COUNT_N2_REF,
        "iter_count drift at n=2: expected {ITER_COUNT_N2_REF}, got {}",
        snapshot.n2.iter_count,
    );
    assert_eq!(
        snapshot.n4.iter_count, ITER_COUNT_N4_REF,
        "iter_count drift at n=4: expected {ITER_COUNT_N4_REF}, got {}",
        snapshot.n4.iter_count,
    );
    assert_eq!(
        snapshot.n8.iter_count, ITER_COUNT_N8_REF,
        "iter_count drift at n=8: expected {ITER_COUNT_N8_REF}, got {}",
        snapshot.n8.iter_count,
    );
}

// =============================================================================
// JSON emit — three sections (scalars + finest top-face vertices + 11pt analytic)
// =============================================================================

const ANALYTIC_SAMPLES: usize = 11;

fn save_compressive_json(snapshot: &CompressiveSnapshot, path: &Path) -> Result<()> {
    let scalars = json!({
        "edge_len":             EDGE_LEN,
        "displacement":         DISPLACEMENT,
        "kappa":                KAPPA,
        "d_hat_override":       D_HAT_OVERRIDE,
        "static_dt":            STATIC_DT,
        "max_newton_iter":      MAX_NEWTON_ITER,
        "mu":                   MU,
        "lambda":               LAMBDA,
        "nu":                   snapshot.nu,
        "e_young":              snapshot.e_young,
        "m_constrained":        snapshot.m_constrained,
        "a_cross":              A_CROSS,
        "cell_size_n2":         CELL_SIZE_N2,
        "cell_size_n4":         CELL_SIZE_N4,
        "cell_size_n8":         CELL_SIZE_N8,
        "lambda_z_avg_n2":      snapshot.n2.lambda_z_avg,
        "lambda_z_avg_n4":      snapshot.n4.lambda_z_avg,
        "lambda_z_avg_n8":      snapshot.n8.lambda_z_avg,
        "eps_n2":               snapshot.n2.eps,
        "eps_n4":               snapshot.n4.eps,
        "eps_n8":               snapshot.n8.eps,
        "f_r_fem_n2":           snapshot.n2.f_r_fem,
        "f_r_fem_n4":           snapshot.n4.f_r_fem,
        "f_r_fem_n8":           snapshot.n8.f_r_fem,
        "f_us_n2":              snapshot.n2.f_us,
        "f_us_n4":              snapshot.n4.f_us,
        "f_us_n8":              snapshot.n8.f_us,
        "f_strain_n2":          snapshot.n2.f_strain,
        "f_strain_n4":          snapshot.n4.f_strain,
        "f_strain_n8":          snapshot.n8.f_strain,
        "n_active_pairs_n2":    snapshot.n2.n_active_pairs,
        "n_active_pairs_n4":    snapshot.n4.n_active_pairs,
        "n_active_pairs_n8":    snapshot.n8.n_active_pairs,
        "iter_count_n2":        snapshot.n2.iter_count,
        "iter_count_n4":        snapshot.n4.iter_count,
        "iter_count_n8":        snapshot.n8.iter_count,
        "residual_norm_n2":     snapshot.n2.residual_norm,
        "residual_norm_n4":     snapshot.n4.residual_norm,
        "residual_norm_n8":     snapshot.n8.residual_norm,
        "cauchy_step_coarse":   snapshot.cauchy_step_coarse,
        "cauchy_step_fine":     snapshot.cauchy_step_fine,
        "cauchy_ratio":         snapshot.cauchy_ratio,
        "effective_modulus_n8": snapshot.effective_modulus_n8,
        "rel_pos_in_bounds_n8": snapshot.rel_pos_in_bounds_n8,
        "small_strain_ceiling": SMALL_STRAIN_CEILING,
    });

    let vertices: Vec<Value> = snapshot
        .n8
        .contact_vertices
        .iter()
        .map(|c| {
            json!({
                "v":       c.v,
                "x":       c.x,
                "y":       c.y,
                "sd":      c.sd,
                "force_z": c.force_z,
            })
        })
        .collect();

    // 11-point sample of the two linear bound functions over
    // ε ∈ [0, 1.5 · ε_n8]. F_us = E · A · ε; F_strain = M_c · A · ε.
    // Both are linear so endpoints would suffice mathematically, but
    // 11 sample points give plot.py a clean line render without
    // depending on matplotlib's auto-extension to the FEM scatter.
    let eps_max = 1.5 * snapshot.n8.eps;
    let mut analytic: Vec<Value> = Vec::with_capacity(ANALYTIC_SAMPLES);
    for i in 0..ANALYTIC_SAMPLES {
        let t = i as f64 / (ANALYTIC_SAMPLES - 1) as f64;
        let eps = t * eps_max;
        let f_us = snapshot.e_young * A_CROSS * eps;
        let f_strain = snapshot.m_constrained * A_CROSS * eps;
        analytic.push(json!({
            "eps":      eps,
            "f_us":     f_us,
            "f_strain": f_strain,
        }));
    }

    let root = json!({
        "scalars":  scalars,
        "vertices": vertices,
        "analytic": analytic,
    });

    let json_str = serde_json::to_string_pretty(&root).context("serialize compressive JSON")?;
    std::fs::write(path, json_str)
        .with_context(|| format!("write compressive JSON to {}", path.display()))?;
    Ok(())
}

// =============================================================================
// PLY emit — finest-refinement deformed boundary mesh + contact_force_z
// =============================================================================

fn save_finest_frame_ply(snapshot: &CompressiveSnapshot, path: &Path) -> Result<()> {
    let n8 = &snapshot.n8;
    let n_vertices = n8.n_vertices;

    // Build positions in physics +Z frame. Positions are unscaled
    // physics — scale-invariance is the point of decoupling
    // RENDER_SCALE from the headless paths.
    let vertices: Vec<Point3<f64>> = (0..n_vertices)
        .map(|i| {
            Point3::new(
                n8.x_final[3 * i],
                n8.x_final[3 * i + 1],
                n8.x_final[3 * i + 2],
            )
        })
        .collect();

    // Boundary-face indices as CCW-wound triangles. Outward winding
    // from the right-handed-tet `signed_volume > 0` invariant per
    // `boundary_faces_from_topology`.
    let faces: Vec<[u32; 3]> = n8
        .boundary_faces
        .iter()
        .map(|f| [f[0], f[1], f[2]])
        .collect();

    let geometry = IndexedMesh::from_parts(vertices, faces);
    let mut mesh = AttributedMesh::new(geometry);
    // Smooth normals for cf-view PBR — mirror rows 12+13.
    mesh.compute_normals();

    // Per-vertex contact-force scalar: zero for inactive vertices,
    // `κ · (d̂_override − sd)` for active top-face vertices. cf-view's
    // auto-colormap (sequential viridis on positive scalar) renders
    // the contact patch as a uniform bright disk on the deformed
    // cube's top face.
    let mut contact_force_z: Vec<f32> = vec![0.0; n_vertices];
    for c in &n8.contact_vertices {
        contact_force_z[c.v as usize] = c.force_z as f32;
    }
    mesh.insert_extra("contact_force_z", contact_force_z)
        .context("insert contact_force_z extra (length must equal n_vertices)")?;

    save_ply_attributed(&mesh, path, true)?;
    Ok(())
}

// =============================================================================
// Bevy visual mode — opt-in via `CF_VISUAL=1` (static finest + two plates)
// =============================================================================

/// Render-side scale factor on visual entities. Mirror rows 12+13
/// verbatim — Bevy 0.18's pipeline defaults are tuned for human-scale
/// (1 m+) scenes, and at sim-soft's cm-scale rendering the camera
/// approaches the near plane on any zoom-in. RENDER_SCALE = 100 lifts
/// the scene past the defaults. Headless asserts + JSON + PLY are
/// scale-invariant.
const RENDER_SCALE: f32 = 100.0;

/// Visualization-only displacement amplifier (banked pattern (b) at
/// row 10 — `feedback_visual_review_is_the_test`). The compressive block's small-strain
/// regime (`ε ≈ 0.6 %` finest-equilibrium) is well below human visual
/// acuity at typical viewing distance: 60 μm cube compression on a
/// 1 m visual cube subtends ~2 mrad ≈ 0.1° from a 3 m camera, below
/// the ~17 mrad ≈ 1 arc-min threshold. **Multiplies `(deformed −
/// rest)` by `VIZ_AMPLIFY` before passing to the Bevy trajectory
/// replay**, so the cube's compression + lateral Poisson bulge become
/// visually perceptible (`50× → ~30 %` apparent z-squish + `~12 %`
/// lateral expansion at `ε = 0.6 %`, `ν = 0.4`).
///
/// **Plates are positioned flush against the (amplified) cube faces**
/// rather than at the kinematic `δ`-offset positions, so the visual
/// reads as "two plates squishing the cube" without an inflated-by-
/// `VIZ_AMPLIFY` contact-band gap (the penalty model's finite-band
/// compliance produces `sd ≈ 9.78 μm` at finest equilibrium, which
/// amplifies to ~5 cm Bevy and dominates the visual story; the band
/// is an FEM no-penetration-enforcement implementation detail
/// tangential to the row's physics pedagogy). The contact-band gap
/// is absorbed into the visualization layer; HUD numbers + JSON +
/// plot.py F-vs-ε scatter still carry the penalty-band physics for
/// anyone wanting to inspect it.
///
/// **Headless asserts + JSON + PLY are NOT amplified** — they operate
/// on the true `x_final` produced by the solver. The HUD declares the
/// amplification factor on the last line so the visual-vs-numerical
/// contract is explicit.
const VIZ_AMPLIFY: f32 = 50.0;

/// Sub-mm Bevy offset between plate face and cube face to prevent
/// depth-buffer z-fight while reading visually as "in contact."
/// At RENDER_SCALE 100×, 0.002 Bevy = 20 μm physics-equivalent
/// (negligible vs the cube's 1 cm physics edge). Sufficient depth
/// resolution at the default Bevy 0.18 near/far clip bracket.
const PLATE_ZFIGHT_OFFSET: f32 = 0.002;

/// Resource carrying the headless-harness outputs the Bevy `Startup`
/// system needs to spawn the soft-mesh + plates + camera entities.
/// Mirrors rows 12+13's `VisualSetup` shape — wrapped in `Option` so
/// `setup_visual_scene` `.take()`s it once.
#[derive(Resource)]
struct VisualSetup(Option<VisualSetupInner>);

struct VisualSetupInner {
    /// Single-frame trajectory carrying the finest x_final. With one
    /// frame, `step_replay` writes the deformed positions once and
    /// `frame_index_at` clamps at end (no animation, no looping).
    trajectory: Trajectory,
    rest_positions: Vec<Vec3>,
    boundary_faces: Vec<[VertexId; 3]>,
    /// Bevy y-coordinate of the amplified cube top face (rest top
    /// minus `VIZ_AMPLIFY × compression`, then × RENDER_SCALE).
    /// Top plate's bottom face is positioned at this y plus
    /// `PLATE_ZFIGHT_OFFSET`.
    cube_top_amplified_y: f32,
    /// HUD-line text — pre-formatted on the headless side so the Bevy
    /// system reads ready-to-render strings.
    hud: String,
}

fn run_visual_mode(snapshot: &CompressiveSnapshot) {
    // Amplify deformations from rest for visualization-only — see
    // `VIZ_AMPLIFY` constant docstring + pattern (b) memo. Each vertex
    // gets `viz_pos = rest + VIZ_AMPLIFY · (x_final − rest)` per
    // dimension; rest positions are unchanged in the trajectory's
    // first-tick replay payload (Bevy mesh starts at rest, then
    // step_replay overwrites with amplified positions). Headless
    // asserts + JSON + PLY paths read `snapshot.n8.x_final` directly
    // and remain unamplified.
    let n_vertices = snapshot.n8.rest_positions.len();
    let mut amplified_x_final = vec![0.0_f64; 3 * n_vertices];
    let amp = f64::from(VIZ_AMPLIFY);
    for i in 0..n_vertices {
        let rest = &snapshot.n8.rest_positions[i];
        let dx = snapshot.n8.x_final[3 * i] - rest.x;
        let dy = snapshot.n8.x_final[3 * i + 1] - rest.y;
        let dz = snapshot.n8.x_final[3 * i + 2] - rest.z;
        amplified_x_final[3 * i] = amp.mul_add(dx, rest.x);
        amplified_x_final[3 * i + 1] = amp.mul_add(dy, rest.y);
        amplified_x_final[3 * i + 2] = amp.mul_add(dz, rest.z);
    }

    let trajectory = Trajectory {
        frames: vec![amplified_x_final],
        // dt is irrelevant at frames.len() == 1 (frame_index_at clamps
        // to 0); set to STATIC_DT so the math is well-defined.
        dt: STATIC_DT,
    };

    // Amplified cube top y (Bevy units) for top-plate face placement.
    // At rest, cube top is at z = EDGE_LEN; under VIZ_AMPLIFY the top
    // face descends by VIZ × (compression) where compression = ε ·
    // EDGE_LEN. Final amplified top z = EDGE_LEN · (1 − VIZ × ε);
    // multiply by RENDER_SCALE for Bevy y.
    #[allow(clippy::cast_possible_truncation)]
    let cube_top_amplified_y = (EDGE_LEN
        * f64::from(VIZ_AMPLIFY).mul_add(-snapshot.n8.eps, 1.0)
        * f64::from(RENDER_SCALE)) as f32;

    // ASCII-only HUD per row 12's banked precedent (`886e6ba3` —
    // Bevy 0.18's default font lacks Unicode glyphs like ε / ×,
    // which render as missing-glyph boxes). Use ASCII identifiers
    // (`eps`, `x` instead of the Unicode `×` multiplication sign).
    // Banked from row 18's `4bea4c2f` visual-review fixup.
    let hud = format!(
        "n=2  eps = {:.4}%  F_R = {:.4} N  [F_us {:.4}, F_strain {:.4}]\n\
         n=4  eps = {:.4}%  F_R = {:.4} N  [F_us {:.4}, F_strain {:.4}]\n\
         n=8  eps = {:.4}%  F_R = {:.4} N  [F_us {:.4}, F_strain {:.4}]\n\
         Cauchy ratio (F_R) = {:.4} (gate < 1)\n\
         E_eff (n=8) = {:.4e} Pa  in [E={:.4e}, M_c={:.4e}]\n\
         rel_pos_in_bounds = {:.4} (0=uniaxial-stress, 1=uniaxial-strain)\n\
         VIZ_AMPLIFY = {:.0}x (cube + plates amplified for visibility; plates flush against cube; numbers above are TRUE physics)",
        snapshot.n2.eps * 100.0,
        snapshot.n2.f_r_fem,
        snapshot.n2.f_us,
        snapshot.n2.f_strain,
        snapshot.n4.eps * 100.0,
        snapshot.n4.f_r_fem,
        snapshot.n4.f_us,
        snapshot.n4.f_strain,
        snapshot.n8.eps * 100.0,
        snapshot.n8.f_r_fem,
        snapshot.n8.f_us,
        snapshot.n8.f_strain,
        snapshot.cauchy_ratio,
        snapshot.effective_modulus_n8,
        snapshot.e_young,
        snapshot.m_constrained,
        snapshot.rel_pos_in_bounds_n8,
        VIZ_AMPLIFY,
    );
    let visual_setup = VisualSetupInner {
        trajectory,
        rest_positions: snapshot.n8.rest_positions.clone(),
        boundary_faces: snapshot.n8.boundary_faces.clone(),
        cube_top_amplified_y,
        hud,
    };

    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(OrbitCameraPlugin)
        .add_plugins(SoftBodyVisualPlugin)
        .insert_resource(UpAxis::PlusZ)
        .insert_resource(VisualSetup(Some(visual_setup)))
        .add_systems(Startup, setup_visual_scene)
        .run();
}

fn setup_visual_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<BevyMesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut visual_setup: ResMut<VisualSetup>,
    up: Res<UpAxis>,
) {
    let Some(VisualSetupInner {
        trajectory,
        rest_positions,
        boundary_faces,
        cube_top_amplified_y,
        hud,
    }) = visual_setup.0.take()
    else {
        return;
    };

    // Soft-mesh entity: built from rest config, animated to the single
    // captured frame on first step_replay tick. Coral PBR matches rows
    // 12+13 for cross-row visual consistency.
    let soft_mesh_handle = meshes.add(build_soft_mesh(&rest_positions, &boundary_faces, *up));
    let soft_material = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.55, 0.4),
        perceptual_roughness: 0.5,
        metallic: 0.05,
        ..default()
    });
    commands.spawn((
        Mesh3d(soft_mesh_handle),
        MeshMaterial3d(soft_material),
        Transform::from_scale(BevyVec3::splat(RENDER_SCALE)),
        trajectory,
    ));

    // Plate visualizations — gray PBR cuboids (thick rectangular
    // slabs that read as compression-test rig press plates). Sized
    // 1.5× cube edge in the lateral (xz) directions so they're
    // clearly larger than the cube without dominating the frame, and
    // 8% of cube edge in thickness (y) so they have visible substance
    // — Plane3d quads (rows 12+13's choice for a sphere-on-plane
    // scene) read as paper-thin under perspective and obscure the
    // "two plates squishing the cube" pedagogy this row needs. Both
    // plates share the same mesh + material.
    let plate_lateral = 1.5 * EDGE_LEN as f32 * RENDER_SCALE;
    let plate_thickness = 0.08 * EDGE_LEN as f32 * RENDER_SCALE;
    let plate_mesh = meshes.add(Cuboid::new(plate_lateral, plate_thickness, plate_lateral));
    let plate_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.7, 0.7, 0.7),
        perceptual_roughness: 0.9,
        ..default()
    });

    // Cube lateral center (Bevy units). The compressive-block helper builds the
    // mesh spanning physics `[0, EDGE_LEN]` in each axis, NOT
    // centered at origin. Under `UpAxis::PlusZ` swap (physics
    // `(x, y, z)` → Bevy `(x, z, y)`) and `Transform::from_scale(
    // RENDER_SCALE)`, the Bevy cube spans `(0, RENDER_SCALE *
    // EDGE_LEN)` in each axis, so its lateral center sits at
    // `(0.5, *, 0.5)` Bevy meters (= 0.5 m at L=1cm × 100×). Plates
    // centered at `(0, *, 0)` would extend asymmetrically past the
    // cube on one side; align them to the cube's actual lateral
    // center instead. Camera target also shifted to match (camera
    // orbits around the cube's true COM, not the world origin).
    // Banked from row 18's `4bea4c2f` visual-review fixup.
    let cube_lateral_x = 0.5 * EDGE_LEN as f32 * RENDER_SCALE;
    let cube_lateral_z = 0.5 * EDGE_LEN as f32 * RENDER_SCALE;

    // Bottom plate — purely visual; physics has no penalty contact
    // here (bottom face is BC-pinned, doesn't move from z=0). Plate's
    // TOP face sits flush against cube bottom at Bevy y=0, with a
    // sub-mm `PLATE_ZFIGHT_OFFSET` to prevent depth-buffer z-fight.
    // Cuboid is centered at the entity transform, so transform y =
    // face_y − thickness/2. Centered at cube lateral COM.
    let bottom_plate_face_y = -PLATE_ZFIGHT_OFFSET;
    let bottom_plate_y = (-0.5_f32).mul_add(plate_thickness, bottom_plate_face_y);
    commands.spawn((
        Mesh3d(plate_mesh.clone()),
        MeshMaterial3d(plate_material.clone()),
        Transform::from_xyz(cube_lateral_x, bottom_plate_y, cube_lateral_z),
    ));

    // Top plate — the physical `RigidPlane`. Plate's BOTTOM face
    // positioned flush against the AMPLIFIED cube top surface (sub-mm
    // `PLATE_ZFIGHT_OFFSET` above to prevent depth-buffer z-fight),
    // NOT at the kinematic `δ`-offset position the physics solver
    // uses. The penalty model's finite-band compliance produces
    // `sd ≈ 9.78 μm` at finest equilibrium — under VIZ_AMPLIFY=50
    // this would inflate to ~5 cm Bevy gap and dominate the visual
    // story. The band is an FEM no-penetration-enforcement
    // implementation detail tangential to the row's "two plates
    // squishing a cube" pedagogy; absorbing it into the visualization
    // gives the rigid-contact look users expect from a compression
    // test rig. HUD numbers + JSON + plot.py F-vs-ε scatter still
    // carry the penalty-band physics for inspection. Cuboid transform
    // y = face_y + thickness/2. Centered at cube lateral COM.
    let top_plate_face_y = cube_top_amplified_y + PLATE_ZFIGHT_OFFSET;
    let top_plate_y = 0.5_f32.mul_add(plate_thickness, top_plate_face_y);
    commands.spawn((
        Mesh3d(plate_mesh),
        MeshMaterial3d(plate_material),
        Transform::from_xyz(cube_lateral_x, top_plate_y, cube_lateral_z),
    ));

    // Camera framing — full-scene default per pattern (l): camera
    // distance ≈ 3 × EDGE_LEN × RENDER_SCALE (3 m), target at the
    // **rest** cube COM `(cube_lateral_x, EDGE_LEN/2 · RENDER_SCALE,
    // cube_lateral_z) = (0.5, 0.5, 0.5)` Bevy m at L=1cm × 100×;
    // angles (0.4, 0.5) rad mirroring row 13's angle pair. The cube
    // IS the orbit-rotation center, so mouse-orbit feels natural (no
    // drift around a phantom origin). Note: the target is at the
    // **rest** COM, ~0.15 Bevy ABOVE the amplified-deformed cube COM
    // (≈ 0.35 m at VIZ_AMPLIFY=50 with `ε ≈ 0.6 %`); the cube
    // appears slightly low in frame, but the full cube + both plates
    // fit comfortably and the HUD readout makes the bound-bracket
    // numbers visible regardless of camera position. Tracking the
    // amplified COM would require recomputing on every VIZ_AMPLIFY
    // change without visual-quality dividend.
    let cube_com_y = 0.5 * EDGE_LEN as f32 * RENDER_SCALE;
    let camera_distance = 3.0 * EDGE_LEN as f32 * RENDER_SCALE;
    commands.spawn((
        Camera3d::default(),
        Transform::default(),
        OrbitCamera::new()
            .with_target(BevyVec3::new(cube_lateral_x, cube_com_y, cube_lateral_z))
            .with_distance(camera_distance)
            .with_angles(0.4, 0.5),
        AmbientLight {
            color: Color::WHITE,
            brightness: 80.0,
            ..default()
        },
    ));

    // Directional light from upper-front-right — mirror rows 12+13.
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(0.5, 1.0, 0.5).looking_at(BevyVec3::ZERO, BevyVec3::Y),
    ));

    // HUD — top-left for asserted scalars (read alongside the visible
    // deformed cube + two plates); controls bottom-left (mirror rows
    // 12+13).
    commands.spawn((
        Text::new(hud),
        TextFont {
            font_size: 13.0,
            ..default()
        },
        TextColor(Color::srgb(0.95, 0.95, 0.7)),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));
    commands.spawn((
        Text::new("Mouse: drag orbit | scroll zoom | right-drag pan | close window to exit"),
        TextFont {
            font_size: 14.0,
            ..default()
        },
        TextColor(Color::srgb(0.95, 0.95, 0.7)),
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));
}

// =============================================================================
// Stdout museum-plaque
// =============================================================================

fn print_summary(snapshot: &CompressiveSnapshot, ply_path: &Path, json_path: &Path) {
    println!("==== compressive-block ====");
    println!();
    println!("Scene: SoftScene::compressive_block_on_plane (compressive-block mirror)");
    println!(
        "  geometry      : HandBuiltTetMesh::uniform_block cube, EDGE_LEN = {EDGE_LEN} m, \
         A = {A_CROSS:.4e} m²"
    );
    println!(
        "  refinements   : n=2 (h={CELL_SIZE_N2} m), n=4 (h={CELL_SIZE_N4} m), \
         n=8 (h={CELL_SIZE_N8} m)"
    );
    println!(
        "  material      : NH(MU = {MU:e}, LAMBDA = {LAMBDA:e})  ν = {nu:.3}, \
         E = {e:.4e} Pa, M_c = {mc:.4e} Pa",
        nu = snapshot.nu,
        e = snapshot.e_young,
        mc = snapshot.m_constrained,
    );
    println!(
        "  rigid plane   : RigidPlane(normal = -ẑ, offset = δ - L = {:e}) (kinematic, top plate)",
        DISPLACEMENT - EDGE_LEN,
    );
    println!(
        "  contact       : PenaltyRigidContact fixture-local (κ = {KAPPA} N/m, \
         d̂_override = {D_HAT_OVERRIDE} m)"
    );
    println!(
        "  bottom face   : BC-pinned (n_pinned = (n+1)² per refinement; helper full-pins \
         all bottom-face vertices)"
    );
    println!("  solver config : Δt = {STATIC_DT} s (static), max_newton_iter = {MAX_NEWTON_ITER}");
    println!();
    println!("NH-derived analytic bounds (Sokolnikoff Ch 4 / Bonet-Wood Ch 5):");
    println!(
        "  E             = {:>13.6e} Pa  (Young's modulus from Lamé pair)",
        snapshot.e_young,
    );
    println!(
        "  M_c           = {:>13.6e} Pa  (constrained modulus = λ + 2μ)",
        snapshot.m_constrained,
    );
    println!(
        "  ν             = {:>13.4}      (Poisson's ratio)",
        snapshot.nu,
    );
    println!(
        "  M_c / E       = {:>13.4}      (uniaxial-strain stiffer by this factor)",
        snapshot.m_constrained / snapshot.e_young,
    );
    println!();
    println!("Per-refinement results:");
    println!(
        "  {:>4}  cell_size = {:>7.4e} m  n_tets = {:>5}  n_active = {:>3}/{:?}  iter = {:>2}  \
         ε = {:>9.4e}  F_R = {:>9.4e} N  [F_us = {:>9.4e}, F_strain = {:>9.4e}]",
        snapshot.n2.label,
        snapshot.n2.cell_size,
        snapshot.n2.n_tets,
        snapshot.n2.n_active_pairs,
        (snapshot.n2.n_per_edge + 1) * (snapshot.n2.n_per_edge + 1),
        snapshot.n2.iter_count,
        snapshot.n2.eps,
        snapshot.n2.f_r_fem,
        snapshot.n2.f_us,
        snapshot.n2.f_strain,
    );
    println!(
        "  {:>4}  cell_size = {:>7.4e} m  n_tets = {:>5}  n_active = {:>3}/{:?}  iter = {:>2}  \
         ε = {:>9.4e}  F_R = {:>9.4e} N  [F_us = {:>9.4e}, F_strain = {:>9.4e}]",
        snapshot.n4.label,
        snapshot.n4.cell_size,
        snapshot.n4.n_tets,
        snapshot.n4.n_active_pairs,
        (snapshot.n4.n_per_edge + 1) * (snapshot.n4.n_per_edge + 1),
        snapshot.n4.iter_count,
        snapshot.n4.eps,
        snapshot.n4.f_r_fem,
        snapshot.n4.f_us,
        snapshot.n4.f_strain,
    );
    println!(
        "  {:>4}  cell_size = {:>7.4e} m  n_tets = {:>5}  n_active = {:>3}/{:?}  iter = {:>2}  \
         ε = {:>9.4e}  F_R = {:>9.4e} N  [F_us = {:>9.4e}, F_strain = {:>9.4e}]",
        snapshot.n8.label,
        snapshot.n8.cell_size,
        snapshot.n8.n_tets,
        snapshot.n8.n_active_pairs,
        (snapshot.n8.n_per_edge + 1) * (snapshot.n8.n_per_edge + 1),
        snapshot.n8.iter_count,
        snapshot.n8.eps,
        snapshot.n8.f_r_fem,
        snapshot.n8.f_us,
        snapshot.n8.f_strain,
    );
    println!();
    println!("Cauchy convergence on F_R sequence:");
    println!(
        "  |F_R_n2 - F_R_n4| = {:>10.4e} N",
        snapshot.cauchy_step_coarse,
    );
    println!(
        "  |F_R_n4 - F_R_n8| = {:>10.4e} N",
        snapshot.cauchy_step_fine,
    );
    println!(
        "  Cauchy ratio      = {:>10.4}      (gate: < 1; geometric convergence)",
        snapshot.cauchy_ratio,
    );
    println!();
    println!("Effective modulus (derived diagnostic):");
    println!(
        "  E_eff(n=8)         = {:>10.4e} Pa  (= F_R_n8 / (A · ε_n8))",
        snapshot.effective_modulus_n8,
    );
    println!(
        "  rel_pos_in_bounds = {:>10.4}      (0 = uniaxial-stress E={:.2e}, \
         1 = uniaxial-strain M_c={:.2e})",
        snapshot.rel_pos_in_bounds_n8, snapshot.e_young, snapshot.m_constrained,
    );
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!("  geometry_invariants               : compile-time const asserts");
    println!("  mesh_topology_exact               : per-refinement III-1 contract");
    println!(
        "  boundary_partition                : n_pinned > 0 (bottom full-pin) + n_loaded == 0 \
         (compressive block)"
    );
    println!(
        "  solver_per_step_invariants        : no NaN, iter < {NEWTON_ITER_SANITY_CAP}, \
         residual finite"
    );
    println!("  contact_engagement                : n_active == (n+1)² exact per refinement");
    println!("  small_strain_validity             : 0 < ε < {SMALL_STRAIN_CEILING:.2}");
    println!(
        "  gross_physics_per_level           : λ_z ∈ ({LAMBDA_Z_FLOOR}, 1.0) + F_R > 0 \
         per refinement"
    );
    println!(
        "  two_bound_per_level               : F_us ≤ F_R ≤ F_strain per refinement \
         (HEADLINE)"
    );
    println!("  cauchy_f_r_convergence            : |Δ_fine| / |Δ_coarse| < 1");
    println!(
        "  captured_bits_compressive_metrics : 11 f64 bits + 6 usize within IV-1 sparse-tier \
         rel-tol = {SPARSE_REL_TOL:.0e}"
    );
    println!();
    println!("PLY    : {}", ply_path.display());
    println!(
        "         finest deformed boundary mesh (n_vertices = {}, n_faces = {} via \
         Mesh::boundary_faces);",
        snapshot.n8.n_vertices,
        snapshot.n8.boundary_faces.len(),
    );
    println!(
        "         per-vertex contact_force_z extra (zero for inactive, κ·(d̂-sd) for \
         top-face active);"
    );
    println!("         open in cf-view, the workspace's unified visual-review viewer:");
    println!("           cargo run -p cf-viewer --release -- <path>");
    println!();
    println!("JSON   : {}", json_path.display());
    println!(
        "         scalars + per-active top-face vertex (x, y, sd, force_z) at finest + \
         11-pt analytic bounds;"
    );
    println!("         render the matplotlib plot via:");
    println!("           uv run examples/sim-soft/compressive-block/plot.py");
    println!();
    println!("Bevy visualization (CF_VISUAL=1):");
    println!("           CF_VISUAL=1 cargo run -p example-sim-soft-compressive-block --release");
    println!("         spawns an OrbitCamera scene with the finest deformed cube + bottom plate");
    println!("         (purely visual; physics has BC-pin) + top plate (the physical RigidPlane).");
    println!("         Static (single-step quasi-static) — HUD reads 1:1 with asserted scalars.");
    println!("         Mouse drag to orbit, scroll to zoom, right-drag to pan, close to exit.");
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    verify_geometry_invariants();

    let snapshot = run_compressive_sweep();

    // Capture-bit dump runs FIRST so failure-mode protocol has the
    // actuals visible even when later assertions panic on placeholder
    // refs (first-bake cycle) or on real regressions (subsequent
    // re-bakes). Always runs, regardless of assertion outcome below.
    print_capture_block(&snapshot);

    verify_mesh_topology_exact(&snapshot);
    verify_boundary_partition(&snapshot);
    verify_solver_per_step_invariants(&snapshot);
    verify_contact_engagement(&snapshot);
    verify_small_strain_validity(&snapshot);
    verify_gross_physics_per_level(&snapshot);
    verify_two_bound_per_level(&snapshot);
    verify_cauchy_f_r_convergence(&snapshot);
    verify_captured_bits_compressive_metrics(&snapshot);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let ply_path = out_dir.join("compressive_block.ply");
    let json_path = out_dir.join("compressive_block.json");
    save_finest_frame_ply(&snapshot, &ply_path)?;
    save_compressive_json(&snapshot, &json_path)?;

    print_summary(&snapshot, &ply_path, &json_path);

    if std::env::var("CF_VISUAL").is_ok() {
        println!();
        println!(
            "CF_VISUAL set — spawning Bevy static-state visualization (close window to exit) ..."
        );
        run_visual_mode(&snapshot);
    }

    Ok(())
}
