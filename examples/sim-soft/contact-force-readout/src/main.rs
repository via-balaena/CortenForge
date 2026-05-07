//! contact-force-readout — Phase 5 V-3a scene + per-active-pair contact
//! readout via the [`PenaltyRigidContact::per_pair_readout`] foundation
//! patch shipped at `995fb0bf`. Single-refinement (n=8) headline:
//! **the public per-pair readout surface returns the same per-vertex
//! forces row 14's `compressive-block` reconstructs inline from known
//! plane geometry** — bit-equivalent at 1e-12 relative tolerance.
//!
//! `SoftScene::compressive_block_on_plane(edge_len, cell_size,
//! displacement, &material_field)` (V-3a fixture's Phase 5 commit-6
//! helper at `sim/L0/soft/src/readout/scene.rs:377`) builds the
//! production scene — a `HandBuiltTetMesh::uniform_block` cube of
//! edge `L = 1 cm` at `n_per_edge = 8` (3072 tets, 729 vertices),
//! full-pin BC on every bottom-face vertex (`z ≈ 0` band, half-cell
//! tolerance), no external traction. A single `RigidPlane(n = -ẑ,
//! offset = δ - L)` represents the descended top plate, penetrated
//! by exactly `δ = 5 × 10⁻⁵ m` at rest; one-way `PenaltyRigidContact`
//! with **V-3a-LOCAL `(d̂, δ)` override** (`d̂ = 1e-5 m`, default
//! `κ = 1e4 N/m`) drives the cube into static equilibrium.
//!
//! `STATIC_DT = 1.0 s` collapses the inertial Tikhonov regulariser
//! `M / dt²` so a single `replay_step` from rest converges to static
//! equilibrium (mirrors V-3a + row 14). Newton typically takes 3
//! iters at this regime.
//!
//! ## Why per-pair readout (vs. row 14's manual reconstruction)
//!
//! Row 14 reconstructs each top-face vertex's penalty force inline
//! from the known plane geometry: `sd = EDGE_LEN - DISPLACEMENT - z_v`
//! (axis-aligned plane closed-form), then `force_z = κ · (d̂ - sd)`
//! gated on `sd < d̂`. This works because row 14's plane is
//! axis-aligned and there's exactly one rigid primitive — but it
//! **duplicates `penalty.rs`'s gradient formula** at the example
//! layer, and breaks if the row ever switches to a non-axis-aligned
//! primitive (sphere, scan-derived `MeshSdf`, cf-design `Solid`).
//!
//! Row 18 lifts the reconstruction into a public surface:
//! [`PenaltyRigidContact::per_pair_readout(mesh, positions)`] returns
//! a `Vec<ContactPairReadout>` with `(pair, position, sd, normal,
//! force_on_soft)` per active pair, primitive-agnostic. The headline
//! gate asserts `Σ readouts.force_on_soft.z` is bit-equivalent at
//! 1e-12 rel to the row-14-style manual reconstruction (with sign
//! flip for the rigid-reaction-vs-soft-side convention) — the
//! structural anchor that the new public surface returns the same
//! arithmetic the inline duplication produces.
//!
//! ## Why single-refinement n=8
//!
//! Row 14's headline is convergence (Cauchy ratio across n=2/4/8);
//! row 18's headline is per-pair readout (an API/structural property
//! of a single configuration). Re-running n=2 and n=4 would add no
//! signal — the accessor-vs-manual gate works at any single
//! refinement, and n=8 gives the maximum 81 active pairs for the
//! per-pair JSON to be informative.
//!
//! ## Why uniform per-pair area approximation (vs. Voronoi)
//!
//! Pressure (Pa) requires per-vertex area to convert from per-pair
//! force (N). The exact derivation is per-vertex Voronoi-cell or
//! barycentric area on the boundary triangulation — non-trivial
//! enough that V-3 (row 13) deferred per-vertex pressure entirely,
//! emitting raw `force_z` to PLY instead.
//!
//! Row 18 uses a **uniform per-pair area approximation**:
//! `A_per_pair_uniform = A_top_face / n_active_pairs = 1e-4 m² / 81
//! ≈ 1.235e-6 m²`. Per-pair pressure is then `|force_z| /
//! A_per_pair_uniform` (unsigned magnitude — V-3a's `force_z` is
//! negative under the soft-side sign convention; pressure is reported
//! as its absolute value). The aggregate `mean_pressure =
//! sum(|force_z|) / A_top_face` is exact engineering stress σ_z
//! (independent of the per-vertex area choice); per-vertex pressure
//! varies because `force_z` varies (interior-vs-corner penetration
//! depth differs by a few μm under the mixed BC's Saint-Venant
//! boundary-layer pattern — interior vertices are laterally
//! surrounded by material on all four sides → locally uniaxial-strain
//! → stiffer → equilibrium at less compliance → vertex stays close
//! to rest z → larger penetration → ~6× larger force/pressure than
//! corners). Honest "approximate per-vertex pressure" — not "exact
//! Voronoi." Voronoi-cell area is banked as a live followup.
//!
//! ## Why `cargo run --release` only
//!
//! Mirrors row 14 + V-3a fixture precedent. The IV-1 captured-bits
//! contract is platform + build-mode-locked; matching row 14's
//! `--release` invocation removes one variable from the determinism
//! contract. V-3a is fast enough at finest `n=8` that debug-mode is
//! also viable, but consistency with sister rows wins.
//!
//! ## Visual-mode opt-in: `CF_VISUAL=1`
//!
//! Headless asserts + JSON + PLY ALWAYS run. Setting `CF_VISUAL=1`
//! (any non-empty value) additionally spawns a Bevy app rendering
//! the **finest-refinement settled deformed cube** between **two
//! visual plates** (bottom + top) — a static (single-step) scene
//! mirroring row 14's harness verbatim, with the HUD reading per-pair
//! readout statistics: max / mean / min force_z, max / mean / min
//! pressure, n_active_pairs, ε, λ_z, e_eff, plus the `VIZ_AMPLIFY`
//! disclosure line. Pressing `R` is a no-op (no animation to reset);
//! orbit / zoom / pan via mouse work as in rows 12 + 13 + 14.
//!
//! **The bottom plate is purely visual** — physics has no penalty
//! contact at the bottom face; the bottom face is BC-pinned in
//! `BoundaryConditions::pinned_vertices`. The visualization shows two
//! plates because the EXAMPLE_INVENTORY framing inherited from row 14
//! is "soft cube between two RigidPlanes," and the BC-pin is the
//! nearest-representable analog of a perfectly bonded rigid bottom
//! plate; the bottom plate carries no penalty force in the physics.
//!
//! **Visualization is amplified** by `VIZ_AMPLIFY = 50×` per banked
//! pattern (b) — V-3a's small-strain regime (`ε ≈ 0.6 %`) is below
//! human visual acuity at typical viewing distance. The plates are
//! positioned flush against the (amplified) cube faces with a sub-mm
//! z-fight offset, NOT at the kinematic `δ`-offset positions the
//! physics solver uses. **Headless asserts + JSON + PLY are NOT
//! amplified** — they read the true `x_final` from the solver. The
//! HUD's last line declares the amplification factor.
//!
//! ### Why the rendered scene is `100×` simulation scale
//!
//! Mirrors rows 12 + 13 + 14's `RENDER_SCALE = 100×` policy verbatim
//! — Bevy 0.18's pipeline defaults (near plane `0.1 m`, OrbitCamera
//! `min_distance = 0.1 m`, AmbientLight brightness, depth precision)
//! were tuned for human-scale (1 m+) scenes; lifting cm-scale physics
//! to meter scale puts everything safely past the defaults.
//!
//! ## JSON artifact
//!
//! `out/contact_force_readout.json` — two sections:
//!
//! 1. `scalars` — `mu`, `lambda`, `nu`, `e_young`, `m_constrained`,
//!    `edge_len`, `displacement`, `kappa`, `d_hat_override`,
//!    `static_dt`, `n_per_edge`, `cell_size`, `a_top_face`,
//!    `a_per_pair_uniform`, `lambda_z_avg`, `eps`, `f_r_total`,
//!    `mean_force_z`, `max_force_z`, `min_force_z`, `mean_pressure`,
//!    `max_pressure`, `min_pressure`, `f_us`, `f_strain`,
//!    `effective_modulus`, `rel_pos_in_bounds`, `n_active_pairs`,
//!    `iter_count`, `residual_norm`, `n_tets`, `n_vertices`,
//!    `n_loaded`, `n_pinned`, `small_strain_ceiling`.
//! 2. `pairs` — one entry per active pair (sorted by vertex_id):
//!    `(vertex_id, primitive_id, x, y, z, sd, force_x, force_y,
//!    force_z, pressure)`.
//!
//! Run `uv run examples/sim-soft/contact-force-readout/plot.py` to
//! render the contact-patch scatter (top-down x vs y, marker color =
//! pressure, marker size = force_z).
//!
//! ## PLY artifact
//!
//! `out/contact_force_readout.ply` — finest-refinement deformed
//! boundary mesh with per-vertex `contact_pressure` extra (zero for
//! inactive vertices, `|force_z| / A_per_pair_uniform` for active
//! top-face vertices, units Pa under the uniform-area approximation;
//! pressure is unsigned so cf-view's sequential viridis colormap
//! renders the contact patch as a bright top-face cap).
//!
//! **Top-face-only coloring is the intended physics asymmetry** —
//! the bottom face is BC-pinned (Dirichlet, not penalty-contacted),
//! so it carries no `contact_pressure` scalar. cf-view best surfaces
//! the engagement boundary (top vs not-top) and the mean pressure
//! level; the interior-vs-corner pressure gradient (the Saint-Venant
//! boundary-layer pattern: interior vertices are laterally
//! constrained so locally uniaxial-strain → stiffer → larger
//! penetration → ~6× larger pressure than corners, which are
//! free-edge-adjacent and locally uniaxial-stress) is smoothed out
//! by smooth-normal interpolation across the boundary triangulation,
//! and reads cleanly only in `plot.py`'s scatter (no smoothing —
//! each active pair is a discrete marker).
//!
//! Open in cf-view: `cargo run -p cf-viewer --release -- <path>`.
//!
//! ## Anchor groups (all assertions exit-0 on success)
//!
//! - **`geometry_invariants`** — compile-time `const { assert!(...) }`
//!   on `EDGE_LEN > 0`, `DISPLACEMENT > 0`, `MU > 0`, `LAMBDA > 0`,
//!   `KAPPA > 0`, `D_HAT_OVERRIDE > 0`, `STATIC_DT > 0`,
//!   `DISPLACEMENT < EDGE_LEN`, `MAX_NEWTON_ITER > 0`,
//!   `NEWTON_ITER_SANITY_CAP < MAX_NEWTON_ITER`,
//!   `0 < SMALL_STRAIN_CEILING < 1`, `0 < LAMBDA_Z_FLOOR < 1`,
//!   `N_PER_EDGE > 0`, `CELL_SIZE > 0`.
//! - **`mesh_topology_exact`** — `(n_tets, n_vertices, n_loaded,
//!   n_pinned) = (3072, 729, 0, 81)` per the III-1 determinism
//!   contract.
//! - **`solver_per_step_invariants`** — no NaN in `x_final`;
//!   `iter_count < NEWTON_ITER_SANITY_CAP = 40`; finite residual
//!   norm.
//! - **`contact_engagement`** — `n_active_pairs == 81 = (n+1)²`.
//! - **`small_strain_validity`** — `0 < ε < SMALL_STRAIN_CEILING =
//!   0.10`.
//! - **`gross_physics`** — `λ_z ∈ (0.5, 1.0)`; `f_r_total < 0`
//!   (V-3a soft-side convention: outward normal `n = -ẑ` so the
//!   z-component is negative; rigid reaction `−f_r_total` is
//!   positive); every `readout.force_on_soft.z < 0` (per-pair sign
//!   sanity).
//! - **`force_bound_bracket`** — `f_us ≤ |f_r_total| ≤ f_strain`
//!   at the equilibrium ε (inherited from row 14 V-3a; the
//!   absolute value flips `f_r_total` from soft-side-negative to
//!   the row-14-style positive rigid reaction for the bound check).
//! - **`accessor_vs_manual_consistency`** (HEADLINE) —
//!   `assert_relative_eq!(−sum(readouts.force_on_soft.z),
//!   manual_reconstruction_sum, max_relative=1e-12, epsilon=1e-12)`.
//!   The leading `−` is the soft-side-vs-rigid-reaction sign flip
//!   (manual reconstruction follows row 14's positive convention).
//!   The structural anchor: the new public surface returns what row
//!   14 had to reconstruct manually.
//! - **`per_pair_invariants`** — for every readout: `sd <
//!   D_HAT_OVERRIDE`, `|force_on_soft.x|` and `|force_on_soft.y|`
//!   near zero (axis-aligned plane), `normal ≈ -ẑ`, `vertex_id` in
//!   range. (Per-pair `force_on_soft.z` sign is enforced in
//!   `gross_physics`, not here.)
//! - **`captured_bits_readout_metrics`** — IV-1 sparse-tier rel-tol
//!   contract on `lambda_z_avg`, `eps`, `f_r_total`,
//!   `mean_force_z`, `max_force_z`, `min_force_z`, `mean_pressure`,
//!   `max_pressure`, `min_pressure`, `effective_modulus` (10 f64),
//!   `n_active_pairs`, `iter_count`, `n_tets`, `n_vertices`,
//!   `n_loaded`, `n_pinned` (6 usize).

// `serde_json::json!` macro recurses internally per key-value pair;
// at the 36-key `scalars` block in `save_readout_json`, the default
// `128` ceiling overflows. `256` doubles headroom; verified via cold
// build with no other consequences on compile time. Same precedent
// as row 14.
#![recursion_limit = "256"]
// PLY field-data is single-precision on disk; converting f64
// quantities to f32 for the AttributedMesh emit is intrinsic to the
// PLY format. Same precedent as PR1 rows 1+2+3+8+9+10+11 + rows
// 12+13+14.
#![allow(clippy::cast_possible_truncation)]
// `usize as u32` casts on `mesh.n_tets()` (~few thousand here, ≪
// u32::MAX) — the standard Mesh-trait API tax.
#![allow(clippy::cast_possible_wrap)]
// `usize as f64` casts for averaging / indexing. Counts ≤ ~thousands
// here, well within f64 mantissa exact range.
#![allow(clippy::cast_precision_loss)]
// `print_summary` and `print_capture_block` are single museum-plaque
// stdout writers; splitting into sub-helpers fragments the visual
// format without information gain. Same allowance as rows 12+13+14.
#![allow(clippy::too_many_lines)]
// `print_summary` aggregates many scalars-and-collections; threading
// them through a struct adds indirection without information gain.
#![allow(clippy::too_many_arguments)]
// `doc_markdown` flags Unicode math notation (`σ`, `κ`, `λ`, `μ`,
// `δ`, `ν`, `π`) as if they were unbacktick-quoted code identifiers.
// Same allowance as rows 12+13+14.
#![allow(clippy::doc_markdown)]
// Bevy systems take Resources by value or by `Res<T>`, both flagged
// as pass-by-value. Same allowance as rows 12+13+14 +
// `sim/L1/sim-bevy-soft/src/trajectory.rs`.
#![allow(clippy::needless_pass_by_value)]
// Wrapped module-docstring paragraphs with continuation lines
// starting with characters like `+ Ecoflex stiffness` or `~219 μm`
// (Unicode math notation in prose) are not Markdown lists; clippy
// parses them that way and requests 4-space indentation. Same
// allowance as rows 12+13+14.
#![allow(clippy::doc_lazy_continuation)]

use std::path::Path;

use anyhow::{Context, Result};
use approx::assert_relative_eq;
// `bevy::prelude::Vec3` (f32 3-vec, Bevy math) vs `sim_soft::Vec3`
// (nalgebra f64 3-vec) collide; same for `bevy::prelude::Mesh`
// (asset struct) vs `sim_soft::Mesh` (tet-mesh trait). Alias both
// Bevy items so the physics-side imports below stay ergonomic for
// solver inputs. Mirror of rows 12+13+14.
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
    ContactPair, ContactPairReadout, CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh,
    NewtonStep, PenaltyRigidContact, PenaltyRigidContactSolver, RigidPlane, SceneInitial,
    SoftScene, Solver, SolverConfig, Tet4, Vec3, VertexId, pick_vertices_by_predicate,
};

// =============================================================================
// Scene constants — mirror row 14 + V-3a
// (`sim/L0/soft/tests/penalty_compressive_block.rs`) verbatim.
// Re-deriving here keeps the example self-contained AND captures-
// platform-locked; any regression in the V-3a helper that shifts
// these implicitly would surface at first row 18 visual review.
// =============================================================================

/// Cube edge length (1 cm). Mirror V-3a's `EDGE_LEN` per scope memo
/// §9.
const EDGE_LEN: f64 = 1.0e-2;

/// Rigid-plane axial displacement (50 μm). Mirror row 14 / V-3a
/// fixture's override of scope memo §9's recommended 0.5 mm.
const DISPLACEMENT: f64 = 5.0e-5;

/// Lamé pair `(μ, λ)` — Phase 4 IV-3 / IV-5 / V-3a / row 14 default
/// Ecoflex-class compressible NeoHookean (`λ = 4 μ` ⇒ `ν = 0.4`).
const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0e5;

/// Cube top-face area (m²) at rest config. `EDGE_LEN²` = 1 cm² =
/// 1e-4 m². Used for the uniform per-pair pressure approximation +
/// `f_us` / `f_strain` analytic bounds (which use the same area).
const A_TOP_FACE: f64 = EDGE_LEN * EDGE_LEN;

/// Single refinement — `n_per_edge = 8`. Single-refinement choice
/// per the locked decisions: row 18's headline is per-pair readout
/// (an API/structural property), not convergence (which row 14
/// owns at n ∈ {2, 4, 8}). At n=8 the cube has 3072 tets, 729
/// vertices, 81 top-face active pairs — the maximum density for
/// per-pair JSON inspection.
const N_PER_EDGE: usize = 8;

/// Cell size (m). `EDGE_LEN / N_PER_EDGE = 1.25e-3`.
const CELL_SIZE: f64 = EDGE_LEN / 8.0;

/// V-3a-LOCAL penalty stiffness. Pinned at the
/// `sim_soft::contact::penalty::PENALTY_KAPPA_DEFAULT` value
/// (`penalty.rs:54`) per scope memo Decision J — V-3a's override
/// is scoped to `d̂` (and `δ`); `κ` stays at default. The
/// `pub(crate)` visibility on the upstream constant forces
/// re-pinning here.
const KAPPA: f64 = 1.0e4;

/// V-3a-LOCAL penalty contact band (m). **Override of**
/// `PENALTY_DHAT_DEFAULT = 1e-3` (`penalty.rs:62`) per scope memo
/// Decision J's V-3a-may-tune authority — see row 14's "Why the
/// V-3a-LOCAL (d̂, δ) override" section for the full derivation.
/// 100× smaller than default to bring cold-start penalty residual
/// `κ · (d̂ + δ) ≈ 0.6 N` per top-face vertex below the
/// tet-inversion threshold.
const D_HAT_OVERRIDE: f64 = 1.0e-5;

/// Static-equilibrium time-step — large `dt` damps the inertial
/// Tikhonov regulariser `M / dt²` to negligible relative magnitude,
/// yielding pure-static root-find. Mirrors V-3a + row 14 verbatim.
const STATIC_DT: f64 = 1.0;

/// Newton iter cap — mirror V-3a + row 14. Static-equilibrium from
/// rest under V-3a `(d̂, δ)` override typically completes in `3-5`
/// iters; cap leaves `45+` iters of margin against material / load
/// perturbations.
const MAX_NEWTON_ITER: usize = 50;

/// Per-refinement Newton-iter sanity cap (10-iter margin under
/// `MAX_NEWTON_ITER`). Mirror row 14 `< 40`. If the run spends more
/// than this, surface as a regression before bumping the cap.
const NEWTON_ITER_SANITY_CAP: usize = 40;

/// Small-strain validity ceiling on the equilibrium compressive
/// strain `ε = 1 − λ_z`. V-3a expected `ε ≈ 0.6 %` — well below the
/// textbook `ε ≲ 5 %` linear-elastic threshold. Cap at `10 %` gives
/// ~17× headroom over expected; failure here means the V-3a
/// `(d̂, δ)` override is no longer producing a small-strain regime
/// — investigate before relaxing.
const SMALL_STRAIN_CEILING: f64 = 0.10;

/// Lower-bound on `λ_z` — physical sanity for "less than 50 %
/// compression." V-3a expected `λ_z ≈ 0.994`.
const LAMBDA_Z_FLOOR: f64 = 0.5;

/// IV-1 sparse-tier rel-tol for captured bits. `~few thousand tets`
/// through faer's sparse Cholesky lives in IV-1's sparse-at-scale
/// tier; static quasi-step layers another arithmetic stage. `1e-12`
/// admits sparse-solver SIMD/FMA noise while catching any real
/// regression. Same precedent as PR1 rows 6+10+11 + rows 12+13+14.
const SPARSE_REL_TOL: f64 = 1.0e-12;

/// Absolute floor for relative comparisons that touch zero. Below
/// typical compressive-metric magnitudes by 8+ orders of magnitude.
/// Same precedent as PR1 rows 6+10+11 + rows 12+13+14.
const SPARSE_EPS_ABS: f64 = 1.0e-12;

/// Tolerance for "lateral force component near zero" check on the
/// per-pair invariants gate. The plane normal is exactly `(0, 0,
/// -1)` (constructor unit-normalises a unit-aligned input); under
/// IEEE-754 multiplication `force.x = κ · (d̂ - sd) · 0 = ±0` —
/// strictly bit-zero. The tolerance leaves headroom for any
/// floating-point quirk introduced by `RigidPlane` construction
/// without admitting a real lateral-force regression.
const LATERAL_FORCE_TOL: f64 = 1.0e-15;

// =============================================================================
// Exact-pinned mesh counts (III-1 determinism contract).
// Captured 2026-05-06 on macOS arm64, Darwin 25.4.0, stable rustc
// in `--release` build. `HandBuiltTetMesh::uniform_block`
// decomposes each unit cell into 6 tets (so `n_tets = 6 · n³`);
// vertex count is the (n+1)³ grid.
//
// To re-bake: run `cargo run -p example-sim-soft-contact-force-
// readout --release` and copy the values from the `## Capture-bit
// reference values` block printed at the top of stdout (always
// runs, even on assertion failure).
// =============================================================================

/// Tet count at n=8. `6 · n³ = 3072`.
const N_TETS_REF: usize = 3072;
/// Vertex count at n=8. `(n+1)³ = 729`.
const N_VERTICES_REF: usize = 729;
/// Loaded vertex count. **V-3a-specific: 0 exact** —
/// `compressive_block_on_plane` helper sets
/// `BoundaryConditions { loaded_vertices: Vec::new(), ... }`
/// because load is penalty-mediated (no external traction).
const N_LOADED_REF: usize = 0;
/// Bottom-face pinned vertex count. `(n+1)² = 81` (helper picks
/// all bottom-face vertices, half-cell tolerance).
const N_PINNED_REF: usize = 81;

// =============================================================================
// Captured readout-metric bits (IV-1 sparse-tier contract).
// Captured 2026-05-06 on macOS arm64, Darwin 25.4.0, stable rustc,
// in `--release` build.
//
// **Failure-mode protocol** (mirrors row 14's): if the rel-tol
// comparison fails, do NOT re-bake. Diagnose in this order:
//   1. Rule out toolchain drift (rustc / LLVM / libm minor version
//      delta vs the capture).
//   2. If same toolchain, real regression — identify which
//      sim-soft commit altered the
//      `SoftScene::compressive_block_on_plane` constructor, the
//      `PenaltyRigidContact::with_params` / `per_pair_readout`
//      surface, the FEM assembly path through faer, OR
//      `HandBuiltTetMesh::uniform_block`'s tet decomposition.
//   3. NEVER re-bake the reference values to make the test green.
//
// The `lambda_z_avg`, `eps`, and `f_r_total` bits are bit-equivalent
// to row 14's `*_N8_REF_BITS` (same scene + solver, single-refinement
// of row 14's n=8); cross-row regression would surface here AND in
// row 14 simultaneously.
// =============================================================================

/// `λ_z_avg` — mean axial stretch ratio over top-face vertices,
/// dimensionless. Bit-equivalent to row 14's
/// `LAMBDA_Z_AVG_N8_REF_BITS`. `≈ 0.99402` (`ε ≈ 0.598 %`).
const LAMBDA_Z_AVG_REF_BITS: u64 = 0x3fef_cf08_2dd5_ddff;

/// Compressive strain `ε = 1 − λ_z_avg` — `≈ 5.978e-3`.
/// Bit-equivalent to row 14's `EPS_N8_REF_BITS`.
const EPS_REF_BITS: u64 = 0x3f78_7be9_1511_0080;

/// Sum of per-pair `force_on_soft.z` over all 81 active pairs (N).
/// **NEGATIVE** — soft-side force points along outward normal
/// `n = -ẑ`, so the z-component is negative. The *rigid reaction*
/// is `-f_r_total` (positive, what row 14 captures as
/// `F_R_FEM_N8`). Row 14's `F_R_FEM_N8_REF_BITS = 0x3fc7_47f5_d85c_59cb`
/// flips the sign bit to `0xbfc7_47f5_d85c_59cb`. `≈ -0.1819 N`.
const F_R_TOTAL_REF_BITS: u64 = 0xbfc7_47f5_d85c_59cb;

/// Mean per-pair `force_on_soft.z` (N). `f_r_total / 81`.
/// `≈ -2.245e-3 N`.
const MEAN_FORCE_Z_REF_BITS: u64 = 0xbf62_651a_bdea_2758;

/// Max (least negative; closest to zero) per-pair `force_on_soft.z`
/// (N). **Corner** top-face vertices have the smallest penetration
/// — free side-faces in two directions allow lateral bulging
/// (locally uniaxial-stress regime, vertically softer), so the
/// vertex moves DOWN toward the plate level and reaches equilibrium
/// at near-zero penetration. `≈ -4.619e-4 N` — about 5× smaller
/// magnitude than the mean and about 6× smaller than the
/// largest-penetration interior vertex.
const MAX_FORCE_Z_REF_BITS: u64 = 0xbf3e_45a5_7010_4830;

/// Min (most negative; largest magnitude) per-pair `force_on_soft.z`
/// (N). **Interior** top-face vertices have the largest penetration
/// — laterally surrounded by material on all four sides, so they
/// can't bulge laterally (locally uniaxial-strain regime, vertically
/// stiffer); equilibrium reached at less vertical compliance, vertex
/// stays close to its rest z, plate penetration is largest.
/// `≈ -2.942e-3 N`.
const MIN_FORCE_Z_REF_BITS: u64 = 0xbf68_19ff_ca41_5346;

/// Mean per-pair pressure (Pa). `mean(|force_z|) /
/// A_per_pair_uniform`, equivalent to `|f_r_total| / A_top_face`
/// (engineering stress σ_z). `≈ 1.819e3 Pa`.
const MEAN_PRESSURE_REF_BITS: u64 = 0x409c_6b57_9a9c_bba8;

/// Max per-pair pressure (Pa). `|min_force_z| / A_per_pair_uniform`
/// — **interior vertex** (largest-magnitude force_z gives
/// highest-magnitude pressure since pressure is unsigned).
/// `≈ 2.383e3 Pa`.
const MAX_PRESSURE_REF_BITS: u64 = 0x40a2_9e2d_707b_c055;

/// Min per-pair pressure (Pa). `|max_force_z| / A_per_pair_uniform`
/// — **corner vertex**. `≈ 374 Pa`. About 5× smaller than mean —
/// reflects the Saint-Venant boundary-layer pattern: corners
/// (locally uniaxial-stress, free-edge-adjacent) carry less
/// vertical load than the laterally-constrained interior.
const MIN_PRESSURE_REF_BITS: u64 = 0x4077_626a_d0f6_03d3;

/// Effective modulus: `|f_r_total| / (A_top_face · ε)` — `≈ 3.043e5
/// Pa`. Bit-equivalent to row 14's `EFFECTIVE_MODULUS_N8_REF_BITS`.
/// Sits in `[E = 2.8e5, M_c = 6.0e5]` close to E (uniaxial-stress
/// regime for the cube interior); `rel_pos_in_bounds ≈ 0.076`.
const EFFECTIVE_MODULUS_REF_BITS: u64 = 0x4112_9258_06e1_6c93;

/// Active-pair count. **`(n+1)² = 81` exact** — every top-face
/// vertex inside the `d̂`-band at equilibrium per V-3a fixture's
/// docstring.
const N_ACTIVE_PAIRS_REF: usize = 81;

/// Newton iter count at n=8 — `3` iters at the V-3a `(d̂, δ)`
/// override regime. Bit-equivalent to row 14's `ITER_COUNT_N8_REF`.
const ITER_COUNT_REF: usize = 3;

// =============================================================================
// Helpers — math (NeoHookean → small-strain bound functions)
// =============================================================================

/// Young's modulus from Lamé pair: `E = μ (3 λ + 2 μ) / (λ + μ)`.
/// At canonical `(1e5, 4e5)`: `E = 2.8e5 Pa = 2 μ (1 + ν) =
/// 2 · 1e5 · 1.4`.
const fn young_modulus(mu: f64, lambda: f64) -> f64 {
    // const fn doesn't allow `mul_add`; expand explicitly. The
    // formula is exact for compressible isotropic linear elastic.
    mu * (3.0 * lambda + 2.0 * mu) / (lambda + mu)
}

/// Constrained modulus (1D dilatational stiffness):
/// `M_c = E · (1 - ν) / ((1 + ν)(1 - 2 ν)) = λ + 2 μ`. The second
/// equality is exact for isotropic linear elastic — so this is just
/// `λ + 2 μ = 6.0e5 Pa` at the canonical pair.
const fn constrained_modulus(mu: f64, lambda: f64) -> f64 {
    lambda + 2.0 * mu
}

/// Poisson's ratio from Lamé pair: `ν = λ / (2 (λ + μ))`. At
/// canonical `(1e5, 4e5)`: `ν = 0.4`.
const fn poisson_ratio(mu: f64, lambda: f64) -> f64 {
    lambda / (2.0 * (lambda + mu))
}

/// Uniaxial-stress small-strain reaction force at compressive
/// strain `ε`: `F_us = E · A · ε`. Lower bound for the mixed-BC
/// FEM response — see row 14's "Why F_R ∈ [F_us, F_strain] is the
/// headline" section.
fn uniaxial_stress_reaction(epsilon: f64) -> f64 {
    young_modulus(MU, LAMBDA) * A_TOP_FACE * epsilon
}

/// Uniaxial-strain small-strain reaction force at compressive
/// strain `ε`: `F_strain = M_c · A · ε`. Upper bound for the
/// mixed-BC FEM response.
fn uniaxial_strain_reaction(epsilon: f64) -> f64 {
    constrained_modulus(MU, LAMBDA) * A_TOP_FACE * epsilon
}

fn material_field() -> MaterialField {
    MaterialField::uniform(MU, LAMBDA)
}

// =============================================================================
// Snapshot — captured solver outputs + per-pair readouts
// =============================================================================

/// Aggregate snapshot — the single n=8 refinement plus all derived
/// per-pair statistics. Mirrors row 14's `RefinementSnapshot` +
/// `CompressiveSnapshot` collapsed (single-refinement scope).
struct ReadoutSnapshot {
    /// Cell size at n=8 (m). Equal to `CELL_SIZE`.
    cell_size: f64,
    /// Mean axial stretch ratio: `(z_eq_avg over top face) /
    /// EDGE_LEN`. `λ_z < 1` for compression.
    lambda_z_avg: f64,
    /// Compressive strain `ε = 1 − λ_z_avg`. Positive in
    /// compression.
    eps: f64,
    /// Sum of `readout.force_on_soft.z` over all active pairs (N).
    /// NEGATIVE — soft-side force points along outward normal
    /// `n = -ẑ`. Rigid reaction is `-f_r_total` (positive).
    f_r_total: f64,
    /// Mean per-pair `force_on_soft.z` (N).
    mean_force_z: f64,
    /// Max per-pair `force_on_soft.z` (N) — least-negative
    /// (closest-to-zero) value across the 81 pairs. Corresponds to
    /// the smallest-penetration top-face vertex.
    max_force_z: f64,
    /// Min per-pair `force_on_soft.z` (N) — most-negative
    /// (largest-magnitude) value. Corresponds to the
    /// largest-penetration top-face vertex.
    min_force_z: f64,
    /// Mean per-pair pressure (Pa) = `|mean_force_z| /
    /// a_per_pair_uniform`. Equivalent to `|f_r_total| /
    /// A_top_face` — engineering stress σ_z.
    mean_pressure: f64,
    /// Max per-pair pressure (Pa) — corresponds to `min_force_z`'s
    /// vertex (most penetration → most pressure).
    max_pressure: f64,
    /// Min per-pair pressure (Pa) — corresponds to `max_force_z`'s
    /// vertex.
    min_pressure: f64,
    /// Uniaxial-stress small-strain bound `F_us = E · A · ε`.
    /// Lower bound on `|f_r_total|`.
    f_us: f64,
    /// Uniaxial-strain small-strain bound `F_strain = M_c · A · ε`.
    /// Upper bound on `|f_r_total|`.
    f_strain: f64,
    /// Effective modulus: `|f_r_total| / (A · ε)` (Pa). Should sit
    /// in `[E, M_c]`.
    effective_modulus: f64,
    /// Relative position of effective modulus in `[E, M_c]`:
    /// `(effective_modulus - E) / (M_c - E)`. Closer to 0 means
    /// closer to uniaxial-stress (lateral free); closer to 1 means
    /// closer to uniaxial-strain (lateral pinned).
    rel_pos_in_bounds: f64,
    /// Active-pair count. **`(n+1)² = 81` exact** per V-3a fixture's
    /// docstring (every top-face vertex inside band).
    n_active_pairs: usize,
    /// Newton iter count at convergence.
    iter_count: usize,
    /// Free-DOF residual norm at convergence.
    residual_norm: f64,
    /// Total tet count.
    n_tets: usize,
    /// Total mesh vertex count.
    n_vertices: usize,
    /// External-traction loaded vertex count. **0 exact** for V-3a.
    n_loaded: usize,
    /// Bottom-face pinned vertex count. `(n+1)² = 81`.
    n_pinned: usize,
    /// Per-active-pair readouts via the foundation patch
    /// `PenaltyRigidContact::per_pair_readout`. Length ==
    /// `n_active_pairs == 81`. Sorted by vertex_id ascending (the
    /// underlying walk is vertices-outer × primitives-inner; with
    /// one primitive that's vertex-id-sorted).
    readouts: Vec<ContactPairReadout>,
    /// Per-pair pressure (Pa) = `|readouts[i].force_on_soft.z| /
    /// a_per_pair_uniform`. Indexed parallel to `readouts`.
    pressures: Vec<f64>,
    /// Manual reconstruction sum (positive — rigid reaction). Used
    /// by the accessor-vs-manual gate. Bit-equivalent at 1e-12 rel
    /// to `-f_r_total`.
    manual_reaction_sum: f64,
    /// Engineering quantities derived from material constants.
    e_young: f64,
    m_constrained: f64,
    nu: f64,
    /// Top-face area (m²). Equal to `A_TOP_FACE`.
    a_top_face: f64,
    /// Per-pair uniform area (m²) = `a_top_face / n_active_pairs`.
    /// Used for the per-pair pressure conversion under the uniform
    /// approximation; exact-engineering-stress when summed.
    a_per_pair_uniform: f64,
    /// Converged `x_final` (vertex-major + xyz-inner DOF layout).
    x_final: Vec<f64>,
    /// Rest-configuration positions (mesh.positions() snapshot —
    /// used for the Bevy build_soft_mesh's initial pose).
    rest_positions: Vec<Vec3>,
    /// Boundary-face triangulation (cached by
    /// `Mesh::boundary_faces`).
    boundary_faces: Vec<[VertexId; 3]>,
}

// =============================================================================
// Run — single n=8 refinement
// =============================================================================

/// Build the V-3a compressive-block scene at `N_PER_EDGE = 8`,
/// replace the helper's default-`d̂` contact with a V-3a-LOCAL
/// `D_HAT_OVERRIDE` override, run a single static `replay_step`,
/// then call the [`PenaltyRigidContact::per_pair_readout`]
/// foundation surface against a freshly-constructed inspection
/// contact + mesh (the originals were moved into the solver).
/// Aggregate per-pair statistics + the accessor-vs-manual gate's
/// manual reconstruction.
fn run_readout_sweep() -> ReadoutSnapshot {
    let materials = material_field();

    // Helper builds mesh + BC + initial via commit-6 scaffolding;
    // its returned `default_contact` (default `κ`, `d̂`) is
    // discarded and replaced with a `with_params` override per the
    // V-3a-LOCAL section in the module docstring.
    let (mesh, bc, initial, _default_contact) =
        SoftScene::compressive_block_on_plane(EDGE_LEN, CELL_SIZE, DISPLACEMENT, &materials);
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), DISPLACEMENT - EDGE_LEN);
    let contact = PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT_OVERRIDE);

    let n_tets = mesh.n_tets();
    let n_vertices = mesh.n_vertices();
    let n_loaded = bc.loaded_vertices.len();
    let n_pinned = bc.pinned_vertices.len();

    // Snapshot top-face vertex IDs (rest config `z = EDGE_LEN`,
    // half-cell tolerance) BEFORE moving the mesh into the solver.
    // Mirror row 14's idiom — only top-face vertices can enter the
    // contact band (`sd < D_HAT_OVERRIDE`), so iterating just the
    // top face captures every active pair for the manual
    // reconstruction.
    let band_tol = 0.5 * CELL_SIZE;
    let top_face_vertices: Vec<VertexId> =
        pick_vertices_by_predicate(&mesh, |p| (p.z - EDGE_LEN).abs() < band_tol);
    assert!(
        !top_face_vertices.is_empty(),
        "top face must contain at least one vertex at refinement n = {N_PER_EDGE}",
    );

    // Snapshot boundary-faces + rest_positions BEFORE the solver
    // moves the mesh. Both are used by the PLY + Bevy paths.
    let boundary_faces: Vec<[VertexId; 3]> = mesh.boundary_faces().to_vec();
    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();

    let SceneInitial { x_prev, v_prev } = initial;

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;

    let solver: PenaltyRigidContactSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    // No external traction (helper's `loaded_vertices` is empty); θ
    // is length-0 to match BC per `assemble_external_force`'s
    // empty-loaded-vs-θ-length assert. The contact load enters via
    // the penalty model; `step` is driven by penalty alone.
    let empty_theta: [f64; 0] = [];
    let theta_tensor = Tensor::from_slice(&empty_theta, &[0]);
    let step: NewtonStep<_> = solver.replay_step(&x_prev, &v_prev, &theta_tensor, cfg.dt);

    // λ_z = (mean z_eq over top face) / L. Top face was at z = L
    // at rest; compression drops them to z_eq < L → λ_z < 1.
    let z_sum: f64 = top_face_vertices
        .iter()
        .map(|&v| step.x_final[3 * v as usize + 2])
        .sum();
    let lambda_z_avg = (z_sum / top_face_vertices.len() as f64) / EDGE_LEN;
    let eps = 1.0 - lambda_z_avg;
    let f_us = uniaxial_stress_reaction(eps);
    let f_strain = uniaxial_strain_reaction(eps);

    // Manual reconstruction (mirror of row 14's F_R_FEM aggregation
    // path at `examples/sim-soft/compressive-block/src/main.rs:794-
    // 814`). Walks top-face vertices, derives `sd` from known
    // axis-aligned plane geometry, sums `+κ · (d̂ − sd)` per active
    // pair. This is the *rigid reaction* (positive); the
    // soft-side force.z is its negation.
    let mut manual_reaction_sum = 0.0;
    for &v in &top_face_vertices {
        let z_v = step.x_final[3 * v as usize + 2];
        let sd_manual = EDGE_LEN - DISPLACEMENT - z_v;
        if sd_manual < D_HAT_OVERRIDE {
            manual_reaction_sum += KAPPA * (D_HAT_OVERRIDE - sd_manual);
        }
    }

    // Public per-pair readout via the foundation patch. Build a
    // SECOND scene + contact for inspection — the originals were
    // moved into the solver and can't be borrowed back. The helper
    // is a pure function (deterministic mesh build); the cost is
    // one extra mesh allocation (~729 vertices, sub-millisecond
    // even in debug). The plane reconstruction mirrors the
    // moved-into-solver one verbatim so the readout walks the same
    // primitive geometry.
    let (inspection_mesh, _, _, _) =
        SoftScene::compressive_block_on_plane(EDGE_LEN, CELL_SIZE, DISPLACEMENT, &materials);
    let inspection_plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), DISPLACEMENT - EDGE_LEN);
    let inspection_contact =
        PenaltyRigidContact::with_params(vec![inspection_plane], KAPPA, D_HAT_OVERRIDE);

    // Convert flat `x_final` to `Vec<Vec3>` for the readout call.
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

    // Per-pair statistics — sum + min + max + mean across the
    // 81-entry readouts vec.
    let n_active_pairs = readouts.len();
    assert!(
        n_active_pairs > 0,
        "per_pair_readout returned 0 readouts at n=8 — V-3a expects 81 active pairs",
    );
    let f_r_total: f64 = readouts.iter().map(|r| r.force_on_soft.z).sum();
    let mean_force_z = f_r_total / n_active_pairs as f64;
    let max_force_z = readouts
        .iter()
        .map(|r| r.force_on_soft.z)
        .fold(f64::NEG_INFINITY, f64::max);
    let min_force_z = readouts
        .iter()
        .map(|r| r.force_on_soft.z)
        .fold(f64::INFINITY, f64::min);

    // Per-pair pressure — uniform-area approximation per the
    // module docstring's "Why uniform per-pair area approximation"
    // section. `pressure[i] = |force_on_soft[i].z| /
    // a_per_pair_uniform`; pressure is unsigned (a magnitude).
    let a_top_face = A_TOP_FACE;
    let a_per_pair_uniform = a_top_face / n_active_pairs as f64;
    let pressures: Vec<f64> = readouts
        .iter()
        .map(|r| r.force_on_soft.z.abs() / a_per_pair_uniform)
        .collect();
    let mean_pressure = pressures.iter().sum::<f64>() / n_active_pairs as f64;
    let max_pressure = pressures.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let min_pressure = pressures.iter().copied().fold(f64::INFINITY, f64::min);

    // Effective modulus (derived diagnostic). |f_r_total| because
    // f_r_total carries the soft-side sign (negative); modulus is
    // unsigned.
    let e_young = young_modulus(MU, LAMBDA);
    let m_constrained = constrained_modulus(MU, LAMBDA);
    let nu = poisson_ratio(MU, LAMBDA);
    let effective_modulus = f_r_total.abs() / (a_top_face * eps);
    let rel_pos_in_bounds = (effective_modulus - e_young) / (m_constrained - e_young);

    ReadoutSnapshot {
        cell_size: CELL_SIZE,
        lambda_z_avg,
        eps,
        f_r_total,
        mean_force_z,
        max_force_z,
        min_force_z,
        mean_pressure,
        max_pressure,
        min_pressure,
        f_us,
        f_strain,
        effective_modulus,
        rel_pos_in_bounds,
        n_active_pairs,
        iter_count: step.iter_count,
        residual_norm: step.final_residual_norm,
        n_tets,
        n_vertices,
        n_loaded,
        n_pinned,
        readouts,
        pressures,
        manual_reaction_sum,
        e_young,
        m_constrained,
        nu,
        a_top_face,
        a_per_pair_uniform,
        x_final: step.x_final,
        rest_positions,
        boundary_faces,
    }
}

// =============================================================================
// Capture-bit dump — always runs first so failure-mode protocol has
// the actuals visible even when later assertions panic.
// =============================================================================

fn print_capture_block(snapshot: &ReadoutSnapshot) {
    eprintln!();
    eprintln!("==== Capture-bit reference values (paste into source on first bake) ====");
    eprintln!();
    eprintln!("// Topology exact-pins");
    eprintln!("const N_TETS_REF: usize       = {:>8};", snapshot.n_tets);
    eprintln!(
        "const N_VERTICES_REF: usize   = {:>8};",
        snapshot.n_vertices
    );
    eprintln!("const N_LOADED_REF: usize     = {:>8};", snapshot.n_loaded);
    eprintln!("const N_PINNED_REF: usize     = {:>8};", snapshot.n_pinned);
    eprintln!();
    eprintln!("// Readout-metric captured bits");
    eprintln!(
        "const LAMBDA_Z_AVG_REF_BITS: u64       = 0x{:016x}; // λ_z_avg          = {:e}",
        snapshot.lambda_z_avg.to_bits(),
        snapshot.lambda_z_avg,
    );
    eprintln!(
        "const EPS_REF_BITS: u64                = 0x{:016x}; // ε                = {:e}",
        snapshot.eps.to_bits(),
        snapshot.eps,
    );
    eprintln!(
        "const F_R_TOTAL_REF_BITS: u64          = 0x{:016x}; // F_R_total        = {:e} N (NEG soft-side)",
        snapshot.f_r_total.to_bits(),
        snapshot.f_r_total,
    );
    eprintln!(
        "const MEAN_FORCE_Z_REF_BITS: u64       = 0x{:016x}; // mean force_z     = {:e} N",
        snapshot.mean_force_z.to_bits(),
        snapshot.mean_force_z,
    );
    eprintln!(
        "const MAX_FORCE_Z_REF_BITS: u64        = 0x{:016x}; // max force_z      = {:e} N",
        snapshot.max_force_z.to_bits(),
        snapshot.max_force_z,
    );
    eprintln!(
        "const MIN_FORCE_Z_REF_BITS: u64        = 0x{:016x}; // min force_z      = {:e} N",
        snapshot.min_force_z.to_bits(),
        snapshot.min_force_z,
    );
    eprintln!(
        "const MEAN_PRESSURE_REF_BITS: u64      = 0x{:016x}; // mean pressure    = {:e} Pa",
        snapshot.mean_pressure.to_bits(),
        snapshot.mean_pressure,
    );
    eprintln!(
        "const MAX_PRESSURE_REF_BITS: u64       = 0x{:016x}; // max pressure     = {:e} Pa",
        snapshot.max_pressure.to_bits(),
        snapshot.max_pressure,
    );
    eprintln!(
        "const MIN_PRESSURE_REF_BITS: u64       = 0x{:016x}; // min pressure     = {:e} Pa",
        snapshot.min_pressure.to_bits(),
        snapshot.min_pressure,
    );
    eprintln!(
        "const EFFECTIVE_MODULUS_REF_BITS: u64  = 0x{:016x}; // E_eff            = {:e} Pa",
        snapshot.effective_modulus.to_bits(),
        snapshot.effective_modulus,
    );
    eprintln!(
        "const N_ACTIVE_PAIRS_REF: usize        = {:>8};                       // n_active",
        snapshot.n_active_pairs,
    );
    eprintln!(
        "const ITER_COUNT_REF: usize            = {:>8};                       // iter",
        snapshot.iter_count,
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
    const { assert!(N_PER_EDGE > 0, "N_PER_EDGE must be positive") };
    const { assert!(CELL_SIZE > 0.0, "CELL_SIZE must be positive") };
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
    const {
        assert!(
            LATERAL_FORCE_TOL > 0.0,
            "LATERAL_FORCE_TOL must be positive"
        );
    };
    const {
        assert!(A_TOP_FACE > 0.0, "A_TOP_FACE = EDGE_LEN² must be positive");
    };
}

// =============================================================================
// Anchor 2 — mesh_topology_exact (III-1 contract)
// =============================================================================

fn verify_mesh_topology_exact(snapshot: &ReadoutSnapshot) {
    assert_eq!(
        snapshot.n_tets, N_TETS_REF,
        "n_tets drift — expected {N_TETS_REF}, got {} (III-1 determinism contract violated)",
        snapshot.n_tets,
    );
    assert_eq!(
        snapshot.n_vertices, N_VERTICES_REF,
        "n_vertices drift — expected {N_VERTICES_REF}, got {}",
        snapshot.n_vertices,
    );
    assert_eq!(
        snapshot.n_loaded, N_LOADED_REF,
        "n_loaded drift — expected {N_LOADED_REF}, got {} (V-3a expects 0 exact — load is \
         penalty-mediated, no external traction)",
        snapshot.n_loaded,
    );
    assert_eq!(
        snapshot.n_pinned, N_PINNED_REF,
        "n_pinned drift — expected {N_PINNED_REF}, got {}",
        snapshot.n_pinned,
    );
}

// =============================================================================
// Anchor 3 — solver_per_step_invariants
// =============================================================================

fn verify_solver_per_step_invariants(snapshot: &ReadoutSnapshot) {
    for (i, &x) in snapshot.x_final.iter().enumerate() {
        assert!(
            x.is_finite(),
            "x_final[{i}] = {x} is not finite — Newton diverged",
        );
    }
    assert!(
        snapshot.iter_count < NEWTON_ITER_SANITY_CAP,
        "Newton ran {} iters, ≥ sanity cap {NEWTON_ITER_SANITY_CAP} (= MAX_NEWTON_ITER \
         {MAX_NEWTON_ITER} − 10) — investigate solver / penalty regime regression before \
         bumping the cap",
        snapshot.iter_count,
    );
    assert!(
        snapshot.residual_norm.is_finite(),
        "residual norm = {} is not finite — Newton diverged",
        snapshot.residual_norm,
    );
}

// =============================================================================
// Anchor 4 — contact_engagement (n_active == 81 = (n+1)²)
// =============================================================================

fn verify_contact_engagement(snapshot: &ReadoutSnapshot) {
    let expected = (N_PER_EDGE + 1) * (N_PER_EDGE + 1);
    assert_eq!(
        snapshot.n_active_pairs, expected,
        "n_active_pairs = {} should equal (n+1)² = {expected} (every top-face vertex inside \
         the d̂-band at equilibrium per V-3a fixture's docstring)",
        snapshot.n_active_pairs,
    );
    assert_eq!(
        snapshot.readouts.len(),
        snapshot.n_active_pairs,
        "readouts.len() = {} should match n_active_pairs = {} (per_pair_readout walks the \
         same active set as active_pairs by construction)",
        snapshot.readouts.len(),
        snapshot.n_active_pairs,
    );
}

// =============================================================================
// Anchor 5 — small_strain_validity
// =============================================================================

fn verify_small_strain_validity(snapshot: &ReadoutSnapshot) {
    assert!(
        snapshot.eps > 0.0,
        "ε = {} should be positive (compression); negative indicates extension or sign flip",
        snapshot.eps,
    );
    assert!(
        snapshot.eps < SMALL_STRAIN_CEILING,
        "ε = {:.4} ≥ small-strain ceiling {SMALL_STRAIN_CEILING:.2} — V-3a `(d̂, δ)` override \
         no longer producing small-strain regime; investigate before relaxing",
        snapshot.eps,
    );
}

// =============================================================================
// Anchor 6 — gross_physics (λ_z plausible + per-pair sign sanity)
// =============================================================================

fn verify_gross_physics(snapshot: &ReadoutSnapshot) {
    assert!(
        snapshot.lambda_z_avg < 1.0 && snapshot.lambda_z_avg > LAMBDA_Z_FLOOR,
        "λ_z = {:.6} out of plausible compression range ({LAMBDA_Z_FLOOR:.2}, 1.0) — sign-flip \
         or contact-machinery regression",
        snapshot.lambda_z_avg,
    );
    // f_r_total is the SUM over readouts of `force_on_soft.z`,
    // which is negative per the V-3a sign convention (soft body
    // pushed DOWN by penalty along outward normal `n = -ẑ`).
    // Rigid reaction is its negation (positive). Per-pair sign
    // sanity is checked in the next gate.
    assert!(
        snapshot.f_r_total < 0.0,
        "f_r_total = {:e} N should be negative (soft-side force.z along outward normal n = -ẑ); \
         positive indicates a sign-convention regression. Rigid reaction (-f_r_total) should \
         be positive.",
        snapshot.f_r_total,
    );
    for (idx, r) in snapshot.readouts.iter().enumerate() {
        assert!(
            r.force_on_soft.z < 0.0,
            "readout[{idx}] force_on_soft.z = {:e} should be negative (soft-side force pushed \
             along outward normal n = -ẑ for V-3a's descended top plate)",
            r.force_on_soft.z,
        );
    }
}

// =============================================================================
// Anchor 7 — force_bound_bracket (F_us ≤ |f_r_total| ≤ F_strain)
// =============================================================================

fn verify_force_bound_bracket(snapshot: &ReadoutSnapshot) {
    let rigid_reaction = -snapshot.f_r_total;
    assert!(
        rigid_reaction >= snapshot.f_us,
        "|f_r_total| = {:.4e} N below uniaxial-stress lower bound F_us = {:.4e} N (ε = \
         {:.4e}); mixed BC should be at least as stiff as pure uniaxial-stress (adding the \
         bottom-pin lateral constraint can only stiffen the response)",
        rigid_reaction,
        snapshot.f_us,
        snapshot.eps,
    );
    assert!(
        rigid_reaction <= snapshot.f_strain,
        "|f_r_total| = {:.4e} N above uniaxial-strain upper bound F_strain = {:.4e} N (ε = \
         {:.4e}); mixed BC should be at most as stiff as pure uniaxial-strain (removing \
         lateral pin from sides can only soften the response)",
        rigid_reaction,
        snapshot.f_strain,
        snapshot.eps,
    );
}

// =============================================================================
// Anchor 8 — accessor_vs_manual_consistency (HEADLINE)
// =============================================================================

fn verify_accessor_vs_manual_consistency(snapshot: &ReadoutSnapshot) {
    // `f_r_total` is `Σ readouts.force_on_soft.z` (negative);
    // `manual_reaction_sum` is `Σ +κ·(d̂-sd_manual)` (positive
    // rigid reaction). They should bit-equal at 1e-12 rel after
    // the sign flip — both compute the same arithmetic at the
    // per-pair level (see `tests/penalty_pair_readout.rs`'s force-
    // vs-gradient parity gate); the only non-determinism is the
    // floating-point summation order, which is identical here
    // (both walks are top-face vertices in the same insertion
    // order from `pick_vertices_by_predicate`).
    let accessor_reaction = -snapshot.f_r_total;
    assert_relative_eq!(
        accessor_reaction,
        snapshot.manual_reaction_sum,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
}

// =============================================================================
// Anchor 9 — per_pair_invariants (geometry + plane sanity per readout)
// =============================================================================

fn verify_per_pair_invariants(snapshot: &ReadoutSnapshot) {
    for (idx, r) in snapshot.readouts.iter().enumerate() {
        // Active-band gate: every readout must satisfy `sd < d̂`.
        // The per_pair_readout impl filters on this condition;
        // this gate catches a regression in the filter logic.
        assert!(
            r.sd < D_HAT_OVERRIDE,
            "readout[{idx}] sd = {:e} should be strictly less than D_HAT_OVERRIDE = {:e} \
             (active-band gate)",
            r.sd,
            D_HAT_OVERRIDE,
        );
        // Lateral force components — V-3a's plane has normal exactly
        // `(0, 0, -1)`; under IEEE-754 multiplication
        // `force.x = κ·(d̂-sd)·0 = ±0` strictly. Tolerance leaves
        // headroom for a `RigidPlane` constructor quirk without
        // admitting a real lateral-force regression.
        assert!(
            r.force_on_soft.x.abs() < LATERAL_FORCE_TOL,
            "readout[{idx}] force_on_soft.x = {:e} should be near zero (axis-aligned plane); \
             tol = {:e}",
            r.force_on_soft.x,
            LATERAL_FORCE_TOL,
        );
        assert!(
            r.force_on_soft.y.abs() < LATERAL_FORCE_TOL,
            "readout[{idx}] force_on_soft.y = {:e} should be near zero (axis-aligned plane); \
             tol = {:e}",
            r.force_on_soft.y,
            LATERAL_FORCE_TOL,
        );
        // Plane outward normal — `RigidPlane::new(Vec3(0, 0, -1),
        // ...)` constructs a unit-z normal; `Sdf::grad(p)` returns
        // a constant `n` everywhere on the plane.
        let normal_drift = (r.normal - Vec3::new(0.0, 0.0, -1.0)).norm();
        assert!(
            normal_drift < LATERAL_FORCE_TOL,
            "readout[{idx}] normal = {:?} drifts from expected (0, 0, -1) by {:e} > tol {:e}",
            r.normal,
            normal_drift,
            LATERAL_FORCE_TOL,
        );
        // Pair variant — Phase 5 only ships `ContactPair::Vertex`;
        // future IPC will add `EdgeEdge` / `VertexFace`. This gate
        // pins the row's expectations at the variant level so a
        // future variant addition forces a row-18 review.
        let ContactPair::Vertex { vertex_id, .. } = r.pair;
        assert!(
            (vertex_id as usize) < snapshot.n_vertices,
            "readout[{idx}] pair vertex_id = {vertex_id} out of range [0, {})",
            snapshot.n_vertices,
        );
    }
}

// =============================================================================
// Anchor 10 — captured_bits_readout_metrics (IV-1 sparse-tier rel-tol)
// =============================================================================

fn verify_captured_bits_readout_metrics(snapshot: &ReadoutSnapshot) {
    let lambda_z_avg_ref = f64::from_bits(LAMBDA_Z_AVG_REF_BITS);
    let eps_ref = f64::from_bits(EPS_REF_BITS);
    let f_r_total_ref = f64::from_bits(F_R_TOTAL_REF_BITS);
    let mean_force_z_ref = f64::from_bits(MEAN_FORCE_Z_REF_BITS);
    let max_force_z_ref = f64::from_bits(MAX_FORCE_Z_REF_BITS);
    let min_force_z_ref = f64::from_bits(MIN_FORCE_Z_REF_BITS);
    let mean_pressure_ref = f64::from_bits(MEAN_PRESSURE_REF_BITS);
    let max_pressure_ref = f64::from_bits(MAX_PRESSURE_REF_BITS);
    let min_pressure_ref = f64::from_bits(MIN_PRESSURE_REF_BITS);
    let effective_modulus_ref = f64::from_bits(EFFECTIVE_MODULUS_REF_BITS);

    assert_relative_eq!(
        snapshot.lambda_z_avg,
        lambda_z_avg_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.eps,
        eps_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.f_r_total,
        f_r_total_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.mean_force_z,
        mean_force_z_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.max_force_z,
        max_force_z_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.min_force_z,
        min_force_z_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.mean_pressure,
        mean_pressure_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.max_pressure,
        max_pressure_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.min_pressure,
        min_pressure_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.effective_modulus,
        effective_modulus_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_eq!(
        snapshot.n_active_pairs, N_ACTIVE_PAIRS_REF,
        "n_active drift: expected {N_ACTIVE_PAIRS_REF}, got {}",
        snapshot.n_active_pairs,
    );
    assert_eq!(
        snapshot.iter_count, ITER_COUNT_REF,
        "iter_count drift: expected {ITER_COUNT_REF}, got {}",
        snapshot.iter_count,
    );
    assert_eq!(
        snapshot.n_tets, N_TETS_REF,
        "n_tets drift: expected {N_TETS_REF}, got {}",
        snapshot.n_tets,
    );
    assert_eq!(
        snapshot.n_vertices, N_VERTICES_REF,
        "n_vertices drift: expected {N_VERTICES_REF}, got {}",
        snapshot.n_vertices,
    );
    assert_eq!(
        snapshot.n_loaded, N_LOADED_REF,
        "n_loaded drift: expected {N_LOADED_REF}, got {}",
        snapshot.n_loaded,
    );
    assert_eq!(
        snapshot.n_pinned, N_PINNED_REF,
        "n_pinned drift: expected {N_PINNED_REF}, got {}",
        snapshot.n_pinned,
    );
}

// =============================================================================
// JSON emit — two sections (scalars + per-active-pair readouts)
// =============================================================================

fn save_readout_json(snapshot: &ReadoutSnapshot, path: &Path) -> Result<()> {
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
        "n_per_edge":           N_PER_EDGE,
        "cell_size":            snapshot.cell_size,
        "a_top_face":           snapshot.a_top_face,
        "a_per_pair_uniform":   snapshot.a_per_pair_uniform,
        "lambda_z_avg":         snapshot.lambda_z_avg,
        "eps":                  snapshot.eps,
        "f_r_total":            snapshot.f_r_total,
        "mean_force_z":         snapshot.mean_force_z,
        "max_force_z":          snapshot.max_force_z,
        "min_force_z":          snapshot.min_force_z,
        "mean_pressure":        snapshot.mean_pressure,
        "max_pressure":         snapshot.max_pressure,
        "min_pressure":         snapshot.min_pressure,
        "f_us":                 snapshot.f_us,
        "f_strain":             snapshot.f_strain,
        "effective_modulus":    snapshot.effective_modulus,
        "rel_pos_in_bounds":    snapshot.rel_pos_in_bounds,
        "n_active_pairs":       snapshot.n_active_pairs,
        "iter_count":           snapshot.iter_count,
        "residual_norm":        snapshot.residual_norm,
        "n_tets":               snapshot.n_tets,
        "n_vertices":           snapshot.n_vertices,
        "n_loaded":             snapshot.n_loaded,
        "n_pinned":             snapshot.n_pinned,
        "small_strain_ceiling": SMALL_STRAIN_CEILING,
    });

    let pairs: Vec<Value> = snapshot
        .readouts
        .iter()
        .zip(snapshot.pressures.iter())
        .map(|(r, &p)| {
            let ContactPair::Vertex {
                vertex_id,
                primitive_id,
            } = r.pair;
            json!({
                "vertex_id":    vertex_id,
                "primitive_id": primitive_id,
                "x":            r.position.x,
                "y":            r.position.y,
                "z":            r.position.z,
                "sd":           r.sd,
                "force_x":      r.force_on_soft.x,
                "force_y":      r.force_on_soft.y,
                "force_z":      r.force_on_soft.z,
                "pressure":     p,
            })
        })
        .collect();

    let root = json!({
        "scalars": scalars,
        "pairs":   pairs,
    });

    let json_str = serde_json::to_string_pretty(&root).context("serialize readout JSON")?;
    std::fs::write(path, json_str)
        .with_context(|| format!("write readout JSON to {}", path.display()))?;
    Ok(())
}

// =============================================================================
// PLY emit — finest deformed boundary mesh + contact_pressure
// =============================================================================

fn save_finest_frame_ply(snapshot: &ReadoutSnapshot, path: &Path) -> Result<()> {
    let n_vertices = snapshot.n_vertices;

    // Build positions in physics +Z frame. Positions are unscaled
    // physics — scale-invariance is the point of decoupling
    // RENDER_SCALE from the headless paths.
    let vertices: Vec<Point3<f64>> = (0..n_vertices)
        .map(|i| {
            Point3::new(
                snapshot.x_final[3 * i],
                snapshot.x_final[3 * i + 1],
                snapshot.x_final[3 * i + 2],
            )
        })
        .collect();

    // Boundary-face indices as CCW-wound triangles. Outward winding
    // from the right-handed-tet `signed_volume > 0` invariant per
    // `boundary_faces_from_topology`.
    let faces: Vec<[u32; 3]> = snapshot
        .boundary_faces
        .iter()
        .map(|f| [f[0], f[1], f[2]])
        .collect();

    let geometry = IndexedMesh::from_parts(vertices, faces);
    let mut mesh = AttributedMesh::new(geometry);
    // Smooth normals for cf-view PBR — mirror rows 12+13+14.
    mesh.compute_normals();

    // Per-vertex contact-pressure scalar (Pa, uniform-area
    // approximation): zero for inactive vertices, `|force_z| /
    // a_per_pair_uniform` for active top-face vertices. cf-view's
    // auto-colormap (sequential viridis on positive scalar)
    // renders the contact patch as a uniform bright disk on the
    // deformed cube's top face — colorbar reads in pressure units
    // (vs row 14's force_z units).
    let mut contact_pressure: Vec<f32> = vec![0.0; n_vertices];
    for (r, &p) in snapshot.readouts.iter().zip(snapshot.pressures.iter()) {
        let ContactPair::Vertex { vertex_id, .. } = r.pair;
        contact_pressure[vertex_id as usize] = p as f32;
    }
    mesh.insert_extra("contact_pressure", contact_pressure)
        .context("insert contact_pressure extra (length must equal n_vertices)")?;

    save_ply_attributed(&mesh, path, true)?;
    Ok(())
}

// =============================================================================
// Bevy visual mode — opt-in via `CF_VISUAL=1` (mirror of row 14's harness)
// =============================================================================

/// Render-side scale factor on visual entities. Mirror rows 12+13+14
/// verbatim — Bevy 0.18's pipeline defaults are tuned for human-
/// scale (1 m+) scenes, and at sim-soft's cm-scale rendering the
/// camera approaches the near plane on any zoom-in. RENDER_SCALE =
/// 100 lifts the scene past the defaults. Headless asserts + JSON +
/// PLY are scale-invariant.
const RENDER_SCALE: f32 = 100.0;

/// Visualization-only displacement amplifier (banked pattern (b)
/// at row 10 — `feedback_visual_review_is_the_test`). Mirror row
/// 14 verbatim — V-3a's small-strain regime (`ε ≈ 0.6 %`) is well
/// below human visual acuity at typical viewing distance. Plates
/// are positioned flush against the (amplified) cube faces, NOT at
/// the kinematic `δ`-offset positions. **Headless asserts + JSON +
/// PLY are NOT amplified** — they read the true `x_final` from
/// the solver.
const VIZ_AMPLIFY: f32 = 50.0;

/// Sub-mm Bevy offset between plate face and cube face to prevent
/// depth-buffer z-fight while reading visually as "in contact." At
/// RENDER_SCALE 100×, 0.002 Bevy = 20 μm physics-equivalent.
/// Sufficient depth resolution at the default Bevy 0.18 near/far
/// clip bracket. Mirror row 14.
const PLATE_ZFIGHT_OFFSET: f32 = 0.002;

#[derive(Resource)]
struct VisualSetup(Option<VisualSetupInner>);

struct VisualSetupInner {
    trajectory: Trajectory,
    rest_positions: Vec<Vec3>,
    boundary_faces: Vec<[VertexId; 3]>,
    cube_top_amplified_y: f32,
    hud: String,
}

fn run_visual_mode(snapshot: &ReadoutSnapshot) {
    // Amplify deformations from rest for visualization-only.
    // Mirror row 14's path verbatim. Headless asserts + JSON + PLY
    // remain unamplified.
    let n_vertices = snapshot.rest_positions.len();
    let mut amplified_x_final = vec![0.0_f64; 3 * n_vertices];
    let amp = f64::from(VIZ_AMPLIFY);
    for i in 0..n_vertices {
        let rest = &snapshot.rest_positions[i];
        let dx = snapshot.x_final[3 * i] - rest.x;
        let dy = snapshot.x_final[3 * i + 1] - rest.y;
        let dz = snapshot.x_final[3 * i + 2] - rest.z;
        amplified_x_final[3 * i] = amp.mul_add(dx, rest.x);
        amplified_x_final[3 * i + 1] = amp.mul_add(dy, rest.y);
        amplified_x_final[3 * i + 2] = amp.mul_add(dz, rest.z);
    }

    let trajectory = Trajectory {
        frames: vec![amplified_x_final],
        // dt is irrelevant at frames.len() == 1 (frame_index_at
        // clamps to 0); set to STATIC_DT so the math is well-defined.
        dt: STATIC_DT,
    };

    // Amplified cube top y (Bevy units) for top-plate face placement.
    #[allow(clippy::cast_possible_truncation)]
    let cube_top_amplified_y = (EDGE_LEN
        * f64::from(VIZ_AMPLIFY).mul_add(-snapshot.eps, 1.0)
        * f64::from(RENDER_SCALE)) as f32;

    // ASCII-only HUD per row 12's banked precedent (`886e6ba3` —
    // Bevy 0.18's default font lacks Unicode glyphs like ε / λ /
    // σ / × / em-dash, which render as missing-glyph boxes). Use
    // ASCII identifiers (eps, lambda_z, sigma_z, x) and `-`
    // hyphens.
    let hud = format!(
        "contact-force-readout (V-3a scene + per-pair readout)\n\
         n_active_pairs : {n_active}  (= {n_per_edge_plus_1}x{n_per_edge_plus_1} top-face grid)\n\
         F_R_total      : {fr:.4} N  (rigid reaction; soft-side total = {fr_neg:.4} N)\n\
         mean force_z   : {mfz:.4e} N  (per-pair, soft-side; negative)\n\
         max force_z    : {Mfz:.4e} N  (least negative - smallest penetration)\n\
         min force_z    : {mnfz:.4e} N  (most negative - largest penetration)\n\
         mean pressure  : {mp:.4e} Pa  (= |F_R_total| / A_top_face = engineering stress sigma_z)\n\
         max pressure   : {Mp:.4e} Pa  (uniform per-pair area approx; interior vertices)\n\
         min pressure   : {mnp:.4e} Pa  (corner vertices)\n\
         eps            : {eps_pct:.4}%\n\
         lambda_z       : {lz:.6}\n\
         e_eff          : {eeff:.4e} Pa  in [E={E:.2e}, M_c={Mc:.2e}]  rel_pos = {relp:.3}\n\
         iter_count     : {ic}\n\
         VIZ_AMPLIFY = {viz:.0}x (cube + plates amplified for visibility; plates flush against cube; numbers above are TRUE physics)",
        n_active = snapshot.n_active_pairs,
        n_per_edge_plus_1 = N_PER_EDGE + 1,
        fr = -snapshot.f_r_total,
        fr_neg = snapshot.f_r_total,
        mfz = snapshot.mean_force_z,
        Mfz = snapshot.max_force_z,
        mnfz = snapshot.min_force_z,
        mp = snapshot.mean_pressure,
        Mp = snapshot.max_pressure,
        mnp = snapshot.min_pressure,
        eps_pct = snapshot.eps * 100.0,
        lz = snapshot.lambda_z_avg,
        eeff = snapshot.effective_modulus,
        E = snapshot.e_young,
        Mc = snapshot.m_constrained,
        relp = snapshot.rel_pos_in_bounds,
        ic = snapshot.iter_count,
        viz = VIZ_AMPLIFY,
    );

    let visual_setup = VisualSetupInner {
        trajectory,
        rest_positions: snapshot.rest_positions.clone(),
        boundary_faces: snapshot.boundary_faces.clone(),
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

    // Soft-mesh entity. Coral PBR matches rows 12+13+14.
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

    // Plate visualizations — gray PBR cuboids. Both plates share
    // the same mesh + material.
    let plate_lateral = 1.5 * EDGE_LEN as f32 * RENDER_SCALE;
    let plate_thickness = 0.08 * EDGE_LEN as f32 * RENDER_SCALE;
    let plate_mesh = meshes.add(Cuboid::new(plate_lateral, plate_thickness, plate_lateral));
    let plate_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.7, 0.7, 0.7),
        perceptual_roughness: 0.9,
        ..default()
    });

    // Cube lateral center (Bevy units). The V-3a helper builds the
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
    let cube_lateral_x = 0.5 * EDGE_LEN as f32 * RENDER_SCALE;
    let cube_lateral_z = 0.5 * EDGE_LEN as f32 * RENDER_SCALE;

    // Bottom plate — purely visual (BC-pinned bottom, no penalty
    // contact). TOP face flush against cube bottom at Bevy y=0
    // with sub-mm `PLATE_ZFIGHT_OFFSET` to prevent depth-buffer
    // z-fight. Cuboid centered at lateral cube center.
    let bottom_plate_face_y = -PLATE_ZFIGHT_OFFSET;
    let bottom_plate_y = (-0.5_f32).mul_add(plate_thickness, bottom_plate_face_y);
    commands.spawn((
        Mesh3d(plate_mesh.clone()),
        MeshMaterial3d(plate_material.clone()),
        Transform::from_xyz(cube_lateral_x, bottom_plate_y, cube_lateral_z),
    ));

    // Top plate — BOTTOM face flush against the AMPLIFIED cube
    // top. Same lateral centering as bottom plate.
    let top_plate_face_y = cube_top_amplified_y + PLATE_ZFIGHT_OFFSET;
    let top_plate_y = 0.5_f32.mul_add(plate_thickness, top_plate_face_y);
    commands.spawn((
        Mesh3d(plate_mesh),
        MeshMaterial3d(plate_material),
        Transform::from_xyz(cube_lateral_x, top_plate_y, cube_lateral_z),
    ));

    // Camera framing — full-scene default per pattern (l). Target
    // at the rest cube COM (`(cube_lateral_x, EDGE_LEN/2 *
    // RENDER_SCALE, cube_lateral_z) = (0.5, 0.5, 0.5)` Bevy m at
    // L=1cm × 100×); the cube IS the orbit-rotation center, so
    // mouse-orbit feels natural (no drift around a phantom origin).
    // Note: the target is at the **rest** COM, ~0.15 Bevy ABOVE the
    // amplified-deformed cube COM (≈ 0.35 m at VIZ_AMPLIFY=50 with
    // ε ≈ 0.6 %); the cube appears slightly low in frame, but the
    // full cube + both plates fit comfortably and the HUD readout
    // makes the bound-bracket numbers visible regardless of camera
    // position. Tracking the amplified COM would require
    // recomputing on every VIZ_AMPLIFY change without
    // visual-quality dividend.
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

    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(0.5, 1.0, 0.5).looking_at(BevyVec3::ZERO, BevyVec3::Y),
    ));

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

fn print_summary(snapshot: &ReadoutSnapshot, ply_path: &Path, json_path: &Path) {
    println!("==== contact-force-readout ====");
    println!();
    println!("Scene: SoftScene::compressive_block_on_plane (V-3a mirror, single n=8)");
    println!(
        "  geometry      : HandBuiltTetMesh::uniform_block cube, EDGE_LEN = {EDGE_LEN} m, \
         A_top = {A_TOP_FACE:.4e} m²"
    );
    println!(
        "  refinement    : n = {N_PER_EDGE} (cell_size = {CELL_SIZE:.4e} m, \
         {N_TETS_REF} tets, {N_VERTICES_REF} vertices)"
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
        "  contact       : PenaltyRigidContact V-3a-LOCAL (κ = {KAPPA} N/m, \
         d̂_override = {D_HAT_OVERRIDE} m)"
    );
    println!(
        "  bottom face   : BC-pinned ({N_PINNED_REF} vertices = (n+1)²; helper full-pins all \
         bottom-face vertices)"
    );
    println!("  solver config : Δt = {STATIC_DT} s (static), max_newton_iter = {MAX_NEWTON_ITER}");
    println!();
    println!(
        "Per-pair readout (via PenaltyRigidContact::per_pair_readout — foundation patch `995fb0bf`):"
    );
    println!(
        "  n_active_pairs : {n_active}  (= (n+1)² = 81 — every top-face vertex inside d̂-band)",
        n_active = snapshot.n_active_pairs,
    );
    println!(
        "  F_R_total      : {:>12.6e} N  (rigid reaction; soft-side total = {:>12.6e} N)",
        -snapshot.f_r_total, snapshot.f_r_total,
    );
    println!("  per-pair force_z (soft-side, negative):");
    println!("    mean         : {:>12.6e} N", snapshot.mean_force_z);
    println!(
        "    max          : {:>12.6e} N  (least negative — smallest-penetration vertex)",
        snapshot.max_force_z,
    );
    println!(
        "    min          : {:>12.6e} N  (most negative — largest-penetration vertex)",
        snapshot.min_force_z,
    );
    println!(
        "  per-pair pressure (Pa, uniform-area approximation: A_per_pair = {:.4e} m²):",
        snapshot.a_per_pair_uniform,
    );
    println!(
        "    mean         : {:>12.6e} Pa  (= |F_R_total| / A_top_face = engineering stress σ_z)",
        snapshot.mean_pressure,
    );
    println!(
        "    max          : {:>12.6e} Pa  (interior vertices — locally uniaxial-strain, stiffer)",
        snapshot.max_pressure,
    );
    println!(
        "    min          : {:>12.6e} Pa  (corner vertices — locally uniaxial-stress, softer)",
        snapshot.min_pressure,
    );
    println!();
    println!("Force-bound bracket (V-3a inheritance):");
    println!(
        "  ε              = {:>12.6e}     (compressive strain)",
        snapshot.eps,
    );
    println!(
        "  F_us(ε)        = {:>12.6e} N   (uniaxial-stress lower bound; lateral free)",
        snapshot.f_us,
    );
    println!(
        "  |F_R_total|    = {:>12.6e} N   (FEM mixed BC; should sit in [F_us, F_strain])",
        -snapshot.f_r_total,
    );
    println!(
        "  F_strain(ε)    = {:>12.6e} N   (uniaxial-strain upper bound; lateral pinned)",
        snapshot.f_strain,
    );
    println!();
    println!("Effective modulus (derived diagnostic):");
    println!(
        "  E_eff           = {:>12.6e} Pa  (= |F_R_total| / (A_top_face · ε))",
        snapshot.effective_modulus,
    );
    println!(
        "  rel_pos_in_bounds = {:>10.4}      (0 = uniaxial-stress E={:.2e}, 1 = uniaxial-strain M_c={:.2e})",
        snapshot.rel_pos_in_bounds, snapshot.e_young, snapshot.m_constrained,
    );
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!("  geometry_invariants               : compile-time const asserts");
    println!(
        "  mesh_topology_exact               : III-1 contract (3072 tets / 729 vertices / 0 loaded / 81 pinned)"
    );
    println!(
        "  solver_per_step_invariants        : no NaN, iter < {NEWTON_ITER_SANITY_CAP}, \
         residual finite"
    );
    println!("  contact_engagement                : n_active == 81 = (n+1)²");
    println!("  small_strain_validity             : 0 < ε < {SMALL_STRAIN_CEILING:.2}");
    println!(
        "  gross_physics                     : λ_z ∈ ({LAMBDA_Z_FLOOR}, 1.0) + f_r_total < 0 \
         + every readout.force.z < 0"
    );
    println!("  force_bound_bracket               : F_us ≤ |F_R_total| ≤ F_strain");
    println!(
        "  accessor_vs_manual_consistency    : −Σ readouts.force.z == manual sum at 1e-12 rel \
         (HEADLINE)"
    );
    println!(
        "  per_pair_invariants               : sd < d̂, lateral force ≈ 0, normal ≈ -ẑ per readout"
    );
    println!(
        "  captured_bits_readout_metrics     : 10 f64 bits + 6 usize within IV-1 sparse-tier \
         rel-tol = {SPARSE_REL_TOL:.0e}"
    );
    println!();
    println!("PLY    : {}", ply_path.display());
    println!(
        "         finest deformed boundary mesh ({} vertices, {} faces via Mesh::boundary_faces);",
        snapshot.n_vertices,
        snapshot.boundary_faces.len(),
    );
    println!(
        "         per-vertex contact_pressure extra (zero for inactive, |force_z| / A_per_pair \
         for active);"
    );
    println!(
        "         top-face-only coloring is intended (bottom is BC-pinned, not \
         penalty-contacted);"
    );
    println!(
        "         cf-view shows the engagement boundary + mean pressure level — the \
         interior-vs-corner"
    );
    println!(
        "         pressure gradient (~6x; interior > corner per Saint-Venant) is plot.py's \
         headline"
    );
    println!("         (no smooth-normal interpolation).");
    println!("         Open in cf-view:");
    println!("           cargo run -p cf-viewer --release -- <path>");
    println!();
    println!("JSON   : {}", json_path.display());
    println!(
        "         scalars + per-active-pair (vertex_id, primitive_id, x, y, z, sd, force_x/y/z, \
         pressure);"
    );
    println!("         render the matplotlib contact-patch scatter via:");
    println!("           uv run examples/sim-soft/contact-force-readout/plot.py");
    println!();
    println!("Bevy visualization (CF_VISUAL=1):");
    println!(
        "           CF_VISUAL=1 cargo run -p example-sim-soft-contact-force-readout --release"
    );
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

    let snapshot = run_readout_sweep();

    // Capture-bit dump runs FIRST so failure-mode protocol has the
    // actuals visible even when later assertions panic on
    // placeholder refs (first-bake cycle) or on real regressions
    // (subsequent re-bakes). Always runs, regardless of assertion
    // outcome below.
    print_capture_block(&snapshot);

    verify_mesh_topology_exact(&snapshot);
    verify_solver_per_step_invariants(&snapshot);
    verify_contact_engagement(&snapshot);
    verify_small_strain_validity(&snapshot);
    verify_gross_physics(&snapshot);
    verify_force_bound_bracket(&snapshot);
    verify_accessor_vs_manual_consistency(&snapshot);
    verify_per_pair_invariants(&snapshot);
    verify_captured_bits_readout_metrics(&snapshot);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let ply_path = out_dir.join("contact_force_readout.ply");
    let json_path = out_dir.join("contact_force_readout.json");
    save_finest_frame_ply(&snapshot, &ply_path)?;
    save_readout_json(&snapshot, &json_path)?;

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
