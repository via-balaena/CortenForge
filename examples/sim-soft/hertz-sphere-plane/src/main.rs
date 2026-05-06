//! hertz-sphere-plane — Phase 5 V-3 user-facing wrap: soft sphere
//! quasi-statically pressed against a `RigidPlane` by an axial force,
//! contact-patch radius compared against the Hertzian closed-form.
//!
//! `SoftScene::sphere_on_plane(radius, cell_size, force, material_field)`
//! (sim-soft `818fa7b1` Phase 5 commit-6 helper + `b3368aa9` commit-9
//! V-3 fixture) builds the production scene — a `SphereSdf` body
//! BCC-meshed at three refinement
//! levels (`h, h/2, h/4` = `3, 1.5, 0.75 mm`) via
//! `SdfMeshedTetMesh::from_sdf`, four equator pins (cardinal-direction
//! `(±R, 0, 0)`, `(0, ±R, 0)` to remove rigid-body modes), top-of-sphere
//! band loaded with `LoadAxis::AxisZ` traction summing to `−F = −500 mN`.
//! A single `RigidPlane(+ẑ, offset = -(R + d̂))` at `z = -(R + d̂)` sits
//! just below the sphere's south pole; one-way `PenaltyRigidContact`
//! resists the press with **V-3-LOCAL κ = 1e3 N/m** (10× softer than
//! `PENALTY_KAPPA_DEFAULT = 1e4`, applied via
//! [`PenaltyRigidContact::with_params`](sim_soft::PenaltyRigidContact::with_params)).
//!
//! `STATIC_DT = 1.0 s` collapses the inertial Tikhonov regulariser
//! `M / dt²` so a single `replay_step` from rest converges to static
//! equilibrium (mirrors V-3's `cfg.dt = 1.0`). Three refinements run
//! sequentially; the row's headline gate is the **finest-level
//! Hertzian patch-radius match `rel_err_a < 20%`** plus **monotonic
//! convergence + Cauchy ratio** on `a_FEM`.
//!
//! ## Why `a_FEM` is the headline (and `δ_FEM` is diagnostic-only)
//!
//! V-3 commit 9 surfaced empirically that the rigid-plane Hertz
//! indentation `δ_Hertz` is **structurally unreachable** in the penalty
//! regime: penalty's contact-band depth `d̂ = 1 mm` provides ~1 mm of
//! "soft compliance" before saturating, and at the V-3 force `F = 500 mN`
//! + Ecoflex stiffness, the sphere reaches penalty-band equilibrium at
//! ~219 μm COM descent — far short of the rigid-limit `R + d̂ + δ_Hertz
//! ≈ 11.3 mm`. Higher κ doesn't fix it (penalty stiffer = even less
//! descent); lower κ allows more descent but elastic compliance caps at
//! ~75 μm regardless. **Penalty's compliance band fundamentally prevents
//! the `force = ∞ at sd ≤ 0` rigid-wall behavior Hertz assumes.** Phase
//! H IPC's logarithmic barrier `−κ log(d/d̂)` recovers the rigid limit
//! at `κ → ∞`; until then, the `δ` comparison fails not from FEM error
//! but from the model's stepping-stone design (BF-12 amendment).
//!
//! `a_FEM` (the contact-patch radius) is the Hertz-physical quantity
//! that DOES match in the penalty regime. Hertz `a ∝ F^{1/3} R^{1/3} /
//! E*^{1/3}` is independent of the local pressure profile that penalty
//! distorts; the FEM finds the correct contact-patch radius scaling
//! within mesh-bound discretisation error. Both `δ_FEM` and `a_FEM`
//! appear in stdout + JSON for inspection; **only `a_FEM` is asserted**.
//! Mirror of V-3 fixture's "Plan change 2" reframe verbatim.
//!
//! ## Hertzian closed-form (Johnson 1985 §3.4)
//!
//! For a soft elastic sphere of radius `R` and contact modulus
//! `E* = E / (1 - ν²)` (rigid plane case — the rigid body's compliance
//! drops out of the standard `1/E* = (1-ν₁²)/E₁ + (1-ν₂²)/E₂`
//! reduction) pressed by axial force `F`:
//!
//! ```text
//! δ_Hertz = (9 F² / (16 R E*²))^(1/3)        (indentation, eq 3.36)
//! a_Hertz = (3 F R / (4 E*))^(1/3)            (contact-patch radius, eq 3.37)
//! p₀_Hertz = 3 F / (2 π a_Hertz²)            (peak pressure at center, eq 3.41)
//! ```
//!
//! At V-3 parameters (`R = 1 cm`, `F = 500 mN`, `μ = 2e5 Pa`,
//! `λ = 8e5 Pa` ⇒ `ν = 0.4`, `E* ≈ 6.67e5 Pa`): `δ_Hertz ≈ 316 μm`,
//! `a_Hertz ≈ 1.78 mm`, `p₀_Hertz ≈ 75 kPa`. Strain `δ/R ≈ 3.2%`,
//! `a/R ≈ 17.8%` — comfortably inside Hertz small-strain validity
//! (textbook threshold `a/R ≲ 30%`) and the linear-elastic regime
//! where Neo-Hookean reduces to Lamé.
//!
//! ## Visual-mode opt-in: `CF_VISUAL=1`
//!
//! Headless asserts + JSON + PLY ALWAYS run. Setting `CF_VISUAL=1` (any
//! non-empty value) additionally spawns a Bevy app rendering the
//! **finest-refinement settled deformed sphere** + **two thin annuli
//! on the plane** — coral `a_FEM` ring + white `a_Hertz` ring — so the
//! patch-radius gap reads visually 1:1 with the asserted `rel_err_a`
//! number. The scene is static (single-step quasi-static; no trajectory
//! to replay). Pressing `R` is a no-op for this row (no animation);
//! orbit / zoom / pan via mouse work as in row 12.
//!
//! ### Why the rendered scene is `100×` simulation scale
//!
//! Mirrors row 12's `RENDER_SCALE = 100×` policy verbatim — Bevy 0.18's
//! pipeline defaults (near plane `0.1 m`, OrbitCamera `min_distance =
//! 0.1 m`, AmbientLight brightness, depth precision) were tuned for
//! human-scale (1 m+) scenes; lifting cm-scale physics to meter scale
//! puts everything safely past the defaults. Rendered sphere is `1 m`
//! radius (instead of `1 cm`), plane is `4 m` half-size (instead of
//! `4 cm`), camera distance is `3 m` (instead of `3 cm`). Headless
//! asserts + JSON + PLY are scale-invariant — they operate on
//! unscaled physics positions, so this is visualization-only. Banked
//! at inventory iter-11 pattern (b) for rows 13/14/18.
//!
//! ## JSON artifact
//!
//! `out/hertz_sphere_plane.json` — three sections:
//!
//! 1. `scalars` — `e_star`, `force_applied`, analytic `(a_hertz,
//!    delta_hertz, p0_hertz)`, FEM-measured `(a_fem, n_active_pairs,
//!    iter_count)` per refinement, derived `(rel_err_a, cauchy_ratio_a,
//!    delta_fem_h4)`.
//! 2. `vertices` — at finest refinement only, one entry per active
//!    contact vertex: `(v, r, sd, force_z)` where `v` is the FEM
//!    vertex id, `r = sqrt(x² + y²)` the horizontal radius, `sd` the
//!    plane signed distance, and `force_z = κ · (d̂ − sd)`.
//! 3. `analytic` — 200-point Hertz pressure curve `p(r) = p₀ · √(1 −
//!    (r/a)²)` sampled on `r ∈ [0, a_Hertz]` for plot.py overlay.
//!
//! Run `uv run examples/sim-soft/hertz-sphere-plane/plot.py` to overlay
//! per-vertex points against the analytic curve.
//!
//! ## PLY artifact
//!
//! `out/hertz_sphere_plane.ply` — finest-refinement deformed boundary
//! mesh with per-vertex `contact_force_z` extra (zero for inactive
//! vertices, `κ · (d̂ − sd)` for active). cf-view auto-colormaps the
//! sequential viridis on positive scalar — the contact patch reads as a
//! bright disk on the deformed sphere. Open in cf-view: `cargo run -p
//! cf-viewer --release -- <path>`. Filling the row 12 deferral
//! ("contact-band membership is a row-13 Hertz-patch concern").
//!
//! ## Anchor groups (all assertions exit-0 on success)
//!
//! - **`geometry_invariants`** — compile-time `const { assert!(...) }`
//!   on `RADIUS > 0`, `FORCE > 0`, `MU > 0`, `LAMBDA > 0`, `KAPPA > 0`,
//!   `PENALTY_DHAT > 0`, `STATIC_DT > 0`, `CELL_SIZE_H > CELL_SIZE_H2 >
//!   CELL_SIZE_H4 > 0`, `MAX_NEWTON_ITER > 0`, `0 < REL_ERR_GATE < 1`.
//! - **`mesh_topology_exact`** — per-refinement exact-pin
//!   `(n_tets, n_vertices, n_referenced, n_loaded, n_pinned)` per the
//!   III-1 determinism contract.
//! - **`boundary_partition`** — per-refinement `n_pinned ≥ 3` (the
//!   `sphere_on_plane` helper picks four cardinal-direction equator
//!   points then deduplicates; under generic BCC resolution all four
//!   hit distinct vertices, but the assertion accepts `≥ 3` to match
//!   the helper's contract) and `n_loaded > 0` (top-of-sphere band).
//! - **`solver_per_step_invariants`** — per-refinement: no NaN in
//!   `x_final`; `iter_count < MAX_NEWTON_ITER - 10 = 40` per V-3's iter
//!   margin policy; finite residual norm.
//! - **`contact_engagement`** — `n_active_pairs > 0` per refinement
//!   (sphere reached the plane, contact engaged at all).
//! - **`small_strain_validity`** — `a_fem / R << 1` per refinement
//!   (`< 0.30` per Hertz domain ceiling; expected `~0.18` at V-3
//!   parameters).
//! - **`monotonic_a_convergence`** — `rel_err_a` decreases across
//!   `(h → h/2 → h/4)` (mesh refinement progressively resolves the
//!   contact patch).
//! - **`cauchy_a_convergence`** — `|a_h2 − a_h4| < |a_h − a_h2|`
//!   (Cauchy ratio `< 1`; geometric convergence). Catches "bounded but
//!   non-converging" regressions monotonic-only would miss.
//! - **`finest_level_hertz_match`** — `rel_err_a < REL_ERR_GATE = 20%`
//!   at h/4. Mesh-bound; tightening to `<10%` is Phase H Tet10 +
//!   adaptive refinement work.
//! - **`captured_bits_hertz_metrics`** — IV-1 sparse-tier rel-tol
//!   contract on `a_fem` per refinement (3 f64), `n_active_pairs`
//!   per refinement (3 usize), `iter_count` per refinement (3 usize),
//!   `delta_fem` at finest (1 f64). All four floats asserted at
//!   `1e-12` rel-tol for cross-platform regression detection;
//!   `delta_fem`'s **value** is diagnostic vs Hertz analytic
//!   (structurally unreachable in penalty regime per Plan change 2),
//!   but the captured-bits **assertion** still pins it.

// PLY field-data is single-precision on disk; converting f64 quantities
// to f32 for the AttributedMesh emit is intrinsic to the PLY format. Same
// precedent as PR1 rows 1+2+3+8+9+10+11 + row 12.
#![allow(clippy::cast_possible_truncation)]
// `usize as u32` casts on `mesh.n_tets()` (~few thousand here, ≪
// u32::MAX) — the standard Mesh-trait API tax. Also covers `step_idx as
// f64` for any analytic comparisons.
#![allow(clippy::cast_possible_wrap)]
// `usize as f64` casts for averaging / indexing. Counts ≤ ~tens of
// thousands here, well within f64 mantissa exact range.
#![allow(clippy::cast_precision_loss)]
// `sphere_on_plane(...).expect(...)` on the helper signature. Mirror of
// V-3 fixture + concentric_lame_shells precedent.
#![allow(clippy::expect_used)]
// `print_summary` and `print_capture_block` are single museum-plaque
// stdout writers; splitting into sub-helpers fragments the visual format
// without information gain. Same allowance as PR1 rows + row 12.
#![allow(clippy::too_many_lines)]
// `print_summary` aggregates many scalars-and-collections; threading
// them through a struct adds indirection without information gain. Same
// allowance as row 12.
#![allow(clippy::too_many_arguments)]
// `doc_markdown` flags Unicode math notation (`σ`, `κ`, `λ`, `μ`, `δ`,
// `ν`, `π`) as if they were unbacktrick-quoted code identifiers. Same
// allowance as PR1 rows 5+6+8+9+10+11 + row 12.
#![allow(clippy::doc_markdown)]
// Bevy systems take Resources by value or by `Res<T>`, both flagged as
// pass-by-value. Same allowance as row 12 +
// `sim/L1/sim-bevy-soft/src/trajectory.rs`.
#![allow(clippy::needless_pass_by_value)]
// Wrapped module-docstring paragraphs whose continuation lines start
// with characters like `+ Ecoflex stiffness` or `~219 μm` (Unicode
// math notation in prose) are not Markdown lists; clippy parses them
// that way and requests 4-space indentation. The intent is prose
// continuation, not bullets — same allowance as PR1 rows 5+6+8+9+10+11
// + row 12 use for analogous wrapping inside multi-paragraph docs.
#![allow(clippy::doc_lazy_continuation)]
// `a_fem_h_ref` / `a_fem_h2_ref` / `a_fem_h4_ref` are systematically
// per-refinement bindings carrying the same role at different
// resolutions. Renaming to avoid the lint would obscure the parallel
// structure with the corresponding `A_FEM_*_REF_BITS` constants.
#![allow(clippy::similar_names)]

use std::path::Path;

use anyhow::{Context, Result};
use approx::assert_relative_eq;
// `bevy::prelude::Vec3` (f32 3-vec, Bevy math) vs `sim_soft::Vec3`
// (nalgebra f64 3-vec) collide; same for `bevy::prelude::Mesh` (asset
// struct) vs `sim_soft::Mesh` (tet-mesh trait). Alias both Bevy items so
// the physics-side imports below stay ergonomic for solver inputs.
// Mirror of row 12.
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
use sim_soft::{
    CpuNewtonSolver, MaterialField, Mesh, NewtonStep, PenaltyRigidContact,
    PenaltyRigidContactSolver, RigidPlane, SceneInitial, SdfMeshedTetMesh, SoftScene, Solver,
    SolverConfig, Tet4, Vec3, VertexId, referenced_vertices,
};

// =============================================================================
// Scene constants — mirror V-3 (`sim/L0/soft/tests/hertz_sphere_plane.rs`)
// verbatim. Re-deriving here keeps the example self-contained AND
// captures-platform-locked; any regression in the V-3 helper that shifts
// these implicitly would surface at first row 13 visual review.
// =============================================================================

/// Sphere radius (1 cm). Mirror V-3's `RADIUS`. Hertz validity requires
/// `δ / R ≪ 1`; at `F = 500 mN` and the chosen material, `δ_Hertz / R
/// ≈ 3.2 %` and `a_Hertz / R ≈ 17.8 %` — comfortably small-strain.
const RADIUS: f64 = 1.0e-2;

/// Axial downward force on top-of-sphere band (500 mN). Mirror V-3 per
/// the empirical "Material plan change 1" — at default `κ = 1e4` and
/// scope-memo F (50-200 mN), only the south pole engages and dynamics
/// reach single-vertex penalty equilibrium far short of multi-vertex
/// Hertz. F = 500 mN + V-3-LOCAL κ = 1e3 raises the multi-vertex
/// threshold `h < sqrt(2 R · F/κ)` to `≈ 3.16 mm`, engaging multi-vertex
/// contact at the mid + finest refinement levels (h sits **right at the
/// 3.16 mm threshold** and gives single-vertex regime empirically:
/// captured `n_active` = 1, 5, 45 at (h, h/2, h/4); the convergence
/// story still works because `rel_err_a` decreases monotonically
/// 100% → 40% → 16% across the sequence).
const FORCE: f64 = 0.5;

/// Lamé pair `(μ, λ)` — Ecoflex 00-30 + 15 wt% carbon-black composite,
/// Phase 4 IV-3 Region B / IV-5 middle-shell precedent + V-3 mirror.
/// `λ = 4 μ` ⇒ `ν = 0.4` (compressible Neo-Hookean — Tet4 under
/// `ν → 0.5` exhibits volumetric locking; Phase H Tet10 + F-bar
/// recovers near-incompressible Ecoflex per Part 2 §05 §02). At canonical
/// `(2e5, 8e5)`: `E = 2 μ (1 + ν) = 5.6e5 Pa` and `E* = E / (1 - ν²) ≈
/// 6.67e5 Pa`.
const MU: f64 = 2.0e5;
const LAMBDA: f64 = 8.0e5;

/// Coarsest cell size — V-3 mirror. At `R = 1 cm` BCC sphere gives
/// ~few thousand tets; in the single-vertex / transition regime
/// (`h ≈ sqrt(2R · F/κ) ≈ 3 mm`), `rel_err` is high — establishes the
/// monotonic-convergence baseline.
const CELL_SIZE_H: f64 = 3.0e-3;

/// Mid refinement — V-3 mirror. Below the multi-vertex threshold
/// (`1.5 mm < 3.16 mm`); engages multi-vertex Hertz contact (captured
/// `n_active = 5`). Error reduces vs coarse but remains substantial
/// (rel-err ~40%) — the contact patch is undersampled at this
/// resolution. (V-3 fixture's docstring says "slightly above the
/// multi-vertex threshold" using a stale 1 mm threshold from κ = 1e4
/// scope-memo arithmetic — the actual κ = 1e3 V-3-LOCAL override
/// gives threshold 3.16 mm and h/2 sits comfortably below it.)
const CELL_SIZE_H2: f64 = 1.5e-3;

/// Fine refinement — V-3 mirror. Below the multi-vertex threshold;
/// engages multi-vertex Hertz contact in the disk of radius `sqrt(2R ·
/// F/κ) ≈ 3 mm` at single-pole descent. Captured `n_active_pairs = 45`
/// at this refinement (matches V-3 docstring's empirical claim
/// exactly). **Empirical-cap finest level**: at `h/4 = 0.5 mm`
/// Newton convergence with multi-vertex active-set churn ran > 12 min
/// release-mode; `0.75 mm` is the release-mode-feasible finest at ~2
/// min total release-mode runtime across all three refinements.
const CELL_SIZE_H4: f64 = 7.5e-4;

/// Static-equilibrium time-step — large `dt` damps the inertial
/// Tikhonov regulariser `M / dt²` to negligible relative magnitude,
/// yielding pure-static root-find. Mirror V-3 verbatim.
const STATIC_DT: f64 = 1.0;

/// Newton iter cap — mirror V-3. Static-equilibrium from rest with
/// multi-vertex active-set churn typically completes in `3-10` iters
/// per refinement; cap leaves `40+` iters of margin against material
/// or load perturbations.
const MAX_NEWTON_ITER: usize = 50;

/// Per-refinement Newton-iter sanity cap (margin under
/// `MAX_NEWTON_ITER`). V-3 uses `< 40` (10-iter margin under cap). If
/// any refinement spends more than this, surface as a regression
/// before bumping the cap.
const NEWTON_ITER_SANITY_CAP: usize = 40;

/// Default contact band `d̂` (m). Pinned at upstream
/// `sim_soft::contact::penalty::PENALTY_DHAT_DEFAULT` value
/// (`penalty.rs:62`) per Phase 5 scope memo Decision J. Re-pinned here
/// because the upstream constant is `pub(crate)`. d̂ stays at default
/// since the helper bakes `-(R + d̂)` into plane construction
/// (`scene.rs:691-694`); the V-3-LOCAL override touches κ only.
const PENALTY_DHAT: f64 = 1.0e-3;

/// V-3-LOCAL penalty stiffness. **Override of**
/// `PENALTY_KAPPA_DEFAULT = 1e4` (`penalty.rs:54`) per scope memo
/// Decision J's V-3-may-tune authority — see V-3 fixture's "Material
/// plan change 1" docstring for the empirical motivation. `1e3` is 10×
/// softer than default; brings the multi-vertex contact threshold
/// `h < sqrt(2 R · F/κ)` from `≈ 0.45 mm` (default κ + F = 100 mN) to
/// `≈ 3.16 mm` (this κ + F = 500 mN), so all three refinement levels
/// engage multi-vertex Hertz contact rather than single-vertex penalty
/// equilibrium.
///
/// **Penalty-correction bias** at this κ: `pen_avg = F/(κ·N_active) ≈
/// 9 μm` at h/4 multi-vertex equilibrium — `~3%` bias relative to
/// `δ_Hertz ≈ 316 μm`, well below the 20% finest-level rel-err gate.
/// Production scenes (V-1 / V-3a / V-4 / V-5 / V-7 + row 12) continue
/// to use the default κ; this constant only enters via
/// [`PenaltyRigidContact::with_params`] in [`run_at_refinement`] and
/// must NOT be propagated upstream.
const KAPPA: f64 = 1.0e3;

/// Finest-level Hertz patch-radius rel-error gate. V-3 ships at 20%
/// (relaxed from scope memo §1 V-3's 15% on δ_FEM, since δ is
/// structurally unreachable per "Plan change 2"). Mesh-bound; tightening
/// to `<10%` is Phase H Tet10 + adaptive refinement work.
const REL_ERR_GATE: f64 = 0.20;

/// Hertz small-strain validity ceiling (`a / R`). Textbook Hertz
/// requires `a / R ≲ 30%`; expected at V-3 `(F, R, E*)` is `~17.8%`.
const SMALL_STRAIN_CEILING: f64 = 0.30;

/// IV-1 sparse-tier rel-tol for captured bits. ~few thousand tets through
/// faer's sparse Cholesky lives in IV-1's sparse-at-scale tier; static
/// quasi-step layers another arithmetic stage. `1e-12` admits sparse-
/// solver SIMD/FMA noise while catching any real regression. Same
/// precedent as PR1 rows 6+10+11 + row 12.
const SPARSE_REL_TOL: f64 = 1.0e-12;

/// Absolute floor for relative comparisons that touch zero. Below
/// typical Hertz-metric magnitudes by 8+ orders of magnitude. Same
/// precedent as PR1 rows 6+10+11 + row 12.
const SPARSE_EPS_ABS: f64 = 1.0e-12;

/// Per-refinement labels for diagnostic stdout + JSON keys.
const LABEL_H: &str = "h";
const LABEL_H2: &str = "h/2";
const LABEL_H4: &str = "h/4";

// =============================================================================
// Exact-pinned mesh counts (III-1 determinism contract) — per refinement.
// Captured 2026-05-06 at sim-soft `dev` tip `fed36c62` (post-PR2 row 12
// shipment + iter-11 banking + cold-read fixups), rustc 1.95.0
// (`59807616e` 2026-04-14) on macOS arm64.
//
// To re-bake: run `cargo run -p example-sim-soft-hertz-sphere-plane
// --release` and copy the values from the `## Capture-bit reference
// values` block printed at the top of stdout (always runs, even on
// assertion failure).
// =============================================================================

/// Tet count at coarsest refinement (h = 3 mm).
const N_TETS_H: usize = 2208;

/// Tet count at mid refinement (h/2 = 1.5 mm).
const N_TETS_H2: usize = 15384;

/// Tet count at finest refinement (h/4 = 0.75 mm).
const N_TETS_H4: usize = 124_344;

/// Vertex count at h.
const N_VERTICES_H: usize = 18696;

/// Vertex count at h/2.
const N_VERTICES_H2: usize = 39732;

/// Vertex count at h/4.
const N_VERTICES_H4: usize = 139_936;

/// Referenced (non-orphan) vertex count at h.
const N_REFERENCED_H: usize = 561;

/// Referenced vertex count at h/2.
const N_REFERENCED_H2: usize = 3155;

/// Referenced vertex count at h/4.
const N_REFERENCED_H4: usize = 23353;

/// Top-of-sphere band loaded vertex count at h. The helper computes
/// `band_tol = 0.5 * cell_size` and picks all referenced vertices with
/// `(p.z - radius).abs() < band_tol` — count varies with mesh
/// refinement.
const N_LOADED_H: usize = 22;

/// Loaded vertex count at h/2.
const N_LOADED_H2: usize = 45;

/// Loaded vertex count at h/4.
const N_LOADED_H4: usize = 118;

/// Equator pin count at h. The helper picks the four cardinal-direction
/// equator points and de-duplicates; under generic BCC resolution the
/// dedup leaves `≥ 3` distinct vertices — at all three V-3 cell sizes
/// the four cardinal points hit four distinct referenced vertices.
const N_PINNED_H: usize = 4;

/// Pin count at h/2.
const N_PINNED_H2: usize = 4;

/// Pin count at h/4.
const N_PINNED_H4: usize = 4;

// =============================================================================
// Captured Hertz-metric bits (IV-1 sparse-tier contract).
//
// **Failure-mode protocol** (mirrors row 12's): if the rel-tol
// comparison fails, do NOT re-bake. Diagnose in this order:
//   1. Rule out toolchain drift (rustc / LLVM / libm minor version
//      delta vs the rustc 1.95.0 capture).
//   2. If same toolchain, real regression — identify which sim-soft
//      commit altered the `SoftScene::sphere_on_plane` constructor,
//      the `PenaltyRigidContact::with_params` defaults, the SDF-meshed
//      FEM assembly path through faer, OR the BCC mesher's tet count
//      / orphan handling.
//   3. NEVER re-bake the reference values to make the test green.
// =============================================================================

/// `a_FEM` at h (m). Refinement-h contact-patch radius = `max sqrt(x² +
/// y²)` over active vertices at converged `x_final`.
/// `f64::from_bits(0x3e29_f1ec_0667_6821) ≈ 3.020e-9 m` — at coarse h
/// only the south-pole vertex enters the band (single-vertex penalty
/// regime per V-3 "Material plan change 1"); horizontal radius of that
/// pole's converged position is 3 nm (essentially 0). 100% rel-err vs
/// `a_Hertz` is expected and is the monotonic-baseline starting point.
const A_FEM_H_REF_BITS: u64 = 0x3e29_f1ec_0667_6821;

/// `a_FEM` at h/2 (m).
/// `f64::from_bits(0x3f51_6ea7_e77f_12ad) ≈ 1.064e-3 m`. Multi-vertex
/// regime engaged (5 active pairs); rel-err ~40%.
const A_FEM_H2_REF_BITS: u64 = 0x3f51_6ea7_e77f_12ad;

/// `a_FEM` at h/4 (m). The headline-anchor scalar — finest-level
/// rel-err vs `a_Hertz` is the row's REL_ERR_GATE = 20% gate.
/// `f64::from_bits(0x3f58_861e_638b_8a7a) ≈ 1.497e-3 m`. 45 active
/// pairs (matches V-3 docstring); rel-err ~16% (under the 20% gate).
const A_FEM_H4_REF_BITS: u64 = 0x3f58_861e_638b_8a7a;

/// `δ_FEM` at h/4 (m). **Diagnostic-only** per "Plan change 2" reframe —
/// penalty compliance dominates indentation, Hertz comparison on δ_FEM
/// is structurally unreachable in this regime. Pinned for regression
/// detection only.
/// `f64::from_bits(0xbf49_989d_b206_e410) ≈ -7.811e-4 m` — sphere is in
/// the band but hasn't reached first-contact-equivalent depth `−d̂`,
/// so δ_FEM is negative (matches V-3 docstring's `-781 μm` empirical).
const DELTA_FEM_H4_REF_BITS: u64 = 0xbf49_989d_b206_e410;

/// Active-pair count at h. Single-vertex regime — only south pole.
const N_ACTIVE_H_REF: usize = 1;

/// Active-pair count at h/2. Multi-vertex regime engaged.
const N_ACTIVE_H2_REF: usize = 5;

/// Active-pair count at h/4. Matches V-3 docstring's "~45 active pairs"
/// empirical (banked at scope memo §1 V-3 "Material plan change 2").
const N_ACTIVE_H4_REF: usize = 45;

/// Newton iter count at h.
const ITER_COUNT_H_REF: usize = 3;

/// Newton iter count at h/2.
const ITER_COUNT_H2_REF: usize = 4;

/// Newton iter count at h/4. Active-set churn at large N drives the
/// iter count up vs the coarser refinements; well below the
/// `NEWTON_ITER_SANITY_CAP = 40` margin under `MAX_NEWTON_ITER = 50`.
const ITER_COUNT_H4_REF: usize = 10;

// =============================================================================
// Helpers — math (Hertz analytic + Lamé conversion)
// =============================================================================

/// Young's modulus from Lamé pair: `E = μ (3 λ + 2 μ) / (λ + μ)`.
/// At canonical `(2e5, 8e5)`: `E = 5.6e5 Pa = 2 μ (1 + ν) = 2 · 2e5 ·
/// 1.4`.
const fn young_modulus(mu: f64, lambda: f64) -> f64 {
    // const fn doesn't allow `mul_add`; expand explicitly.
    mu * (3.0 * lambda + 2.0 * mu) / (lambda + mu)
}

/// Hertz contact modulus `E* = E / (1 - ν²)` for the soft-on-rigid
/// case. Standard Johnson 1985 §3 form `1/E* = (1-ν₁²)/E₁ +
/// (1-ν₂²)/E₂` reduces to this for a rigid second body. With Lamé
/// `(μ, λ)` ⇒ `ν = λ / (2 (λ + μ))`. At canonical `(2e5, 8e5)`:
/// `ν = 0.4`, `E* = 5.6e5 / 0.84 ≈ 6.667e5 Pa`.
fn e_star(mu: f64, lambda: f64) -> f64 {
    let e = young_modulus(mu, lambda);
    let nu = lambda / (2.0 * (lambda + mu));
    e / 1.0_f64.mul_add(-nu * nu, 1.0)
}

/// Hertzian indentation (sphere-on-rigid-plane). Johnson 1985 §3.4
/// eq 3.36 in the form `δ³ = 9 F² / (16 R E*²)`.
fn delta_hertz(force: f64, radius: f64, e_star_val: f64) -> f64 {
    (9.0 * force * force / (16.0 * radius * e_star_val * e_star_val)).cbrt()
}

/// Hertzian contact-patch radius (sphere-on-rigid-plane). Johnson 1985
/// §3.4 eq 3.37 in the form `a³ = 3 F R / (4 E*)`.
fn a_hertz(force: f64, radius: f64, e_star_val: f64) -> f64 {
    (3.0 * force * radius / (4.0 * e_star_val)).cbrt()
}

/// Hertzian peak pressure at contact center. Johnson 1985 §3.4 eq 3.41
/// in the form `p₀ = 3 F / (2 π a²)`.
fn p_zero_hertz(force: f64, a: f64) -> f64 {
    1.5 * force / (std::f64::consts::PI * a * a)
}

fn material_field() -> MaterialField {
    MaterialField::uniform(MU, LAMBDA)
}

// =============================================================================
// Snapshot — captured solver outputs at one refinement level
// =============================================================================

/// One active contact-vertex sample at the finest refinement, used by
/// the JSON + PLY emit paths.
#[derive(Clone, Debug)]
struct ContactVertex {
    /// Vertex id in the FEM mesh (referenced subset).
    v: VertexId,
    /// Horizontal radius `sqrt(x² + y²)` at converged `x_final` (m).
    r: f64,
    /// Plane signed distance at converged `x_final`: `sd = z_v -
    /// plane_offset = z_v + R + d̂`. Negative when below the plane;
    /// `< d̂` when active.
    sd: f64,
    /// Penalty force z-component on the vertex: `force_z = κ · (d̂ -
    /// sd)`. Positive (pushing back up out of the rigid half-space).
    force_z: f64,
}

/// Per-refinement run output. Carries FEM-measured Hertz quantities +
/// Newton diagnostics + mesh stats for the diagnostic stdout + JSON +
/// PLY paths. Mirrors V-3 fixture's `StepReport` plus the extras the
/// user-facing example wants (`x_final`, `rest_positions`,
/// `boundary_faces`, `contact_vertices`).
struct RefinementSnapshot {
    /// Cell size at this refinement (m).
    cell_size: f64,
    /// Diagnostic label (`h` / `h/2` / `h/4`).
    label: &'static str,
    /// FEM-measured Hertz indentation `δ_FEM = -d̂ - mean_z_disp_COM`
    /// (positive when sphere indents downward). Diagnostic-only per
    /// "Plan change 2".
    delta_fem: f64,
    /// FEM-measured contact-patch radius `a_FEM = max sqrt(x² + y²)`
    /// over active vertices. Headline asserted scalar.
    a_fem: f64,
    /// Active-pair count at converged `x_final` (sd < d̂ over
    /// referenced vertices).
    n_active_pairs: usize,
    /// Newton iter count at convergence.
    iter_count: usize,
    /// Free-DOF residual norm at convergence.
    residual_norm: f64,
    /// Total tet count.
    n_tets: usize,
    /// Total mesh vertex count (including BCC orphans).
    n_vertices: usize,
    /// Top-of-sphere band loaded vertex count.
    n_loaded: usize,
    /// Equator pin count.
    n_pinned: usize,
    /// Referenced (non-orphan) vertex count.
    n_referenced: usize,
    /// Converged `x_final` (vertex-major + xyz-inner DOF layout).
    x_final: Vec<f64>,
    /// Rest-configuration positions (mesh.positions() snapshot — used
    /// for the Bevy build_soft_mesh's initial pose at finest).
    rest_positions: Vec<Vec3>,
    /// Boundary-face triangulation (cached by `Mesh::boundary_faces`).
    boundary_faces: Vec<[VertexId; 3]>,
    /// Active contact vertices at finest. Empty for non-finest
    /// refinements (we don't emit per-vertex JSON / PLY at h or h/2).
    contact_vertices: Vec<ContactVertex>,
}

/// Aggregate snapshot across all three refinements + analytic Hertz
/// quantities + derived comparison scalars.
struct HertzSnapshot {
    /// Coarsest refinement (h = 3 mm).
    h: RefinementSnapshot,
    /// Mid refinement (h/2 = 1.5 mm).
    h2: RefinementSnapshot,
    /// Finest refinement (h/4 = 0.75 mm) — has populated
    /// `contact_vertices`.
    h4: RefinementSnapshot,
    /// Hertz contact modulus (Pa).
    e_star: f64,
    /// Hertz indentation (m). `δ_Hertz` for diagnostic comparison only.
    delta_hertz: f64,
    /// Hertz contact-patch radius (m). The asserted comparison target.
    a_hertz: f64,
    /// Hertz peak pressure at center (Pa).
    p0_hertz: f64,
    /// Per-refinement `a_FEM` rel-error vs `a_Hertz`.
    rel_err_a_h: f64,
    rel_err_a_h2: f64,
    rel_err_a_h4: f64,
    /// `δ_FEM` rel-error at finest (diagnostic-only).
    rel_err_delta_h4: f64,
    /// Cauchy-step magnitudes on `a_FEM` sequence.
    cauchy_step_coarse_a: f64,
    cauchy_step_fine_a: f64,
    /// Cauchy ratio `|Δ_fine| / |Δ_coarse|` — asserted `< 1`.
    cauchy_ratio_a: f64,
}

// =============================================================================
// Run — single refinement
// =============================================================================

/// Build the V-3 sphere-on-plane scene at `cell_size`, replace the
/// helper's default-κ contact with a V-3-LOCAL `KAPPA` override, run a
/// single static `replay_step`, and capture the [`RefinementSnapshot`].
///
/// `populate_contact_vertices` is `true` only at the finest refinement
/// — the per-vertex sample list is consumed by the JSON + PLY emit
/// paths, both of which only run at h/4.
fn run_at_refinement(
    cell_size: f64,
    label: &'static str,
    populate_contact_vertices: bool,
) -> RefinementSnapshot {
    // Helper builds mesh + BC + initial + theta + a default-κ contact;
    // we discard the default contact and replace with a `with_params`
    // override at V-3-LOCAL κ per the module docstring. The plane is
    // reconstructed identically to the helper's `RigidPlane::new(
    // Vec3::new(0.0, 0.0, 1.0), -(radius + d̂))` (`scene.rs:691-694`).
    let (mesh, bc, initial, _default_contact, theta) =
        SoftScene::sphere_on_plane(RADIUS, cell_size, FORCE, material_field())
            .expect("sphere_on_plane should mesh successfully at canonical cell sizes");
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), -(RADIUS + PENALTY_DHAT));
    let contact = PenaltyRigidContact::with_params(vec![plane], KAPPA, PENALTY_DHAT);

    let n_tets = mesh.n_tets();
    let n_vertices = mesh.n_vertices();
    let n_loaded = bc.loaded_vertices.len();
    let n_pinned = bc.pinned_vertices.len();

    // Snapshot referenced vertices (drop BCC lattice orphans) BEFORE
    // moving the mesh into the solver. Mirrors V-3 fixture's idiom +
    // row 12's pattern; the BCC mesher allocates corners for the full
    // bbox-margin region that no tet references, and orphan auto-pin
    // by `CpuNewtonSolver::new` keeps the free-DOF Hessian SPD without
    // touching them.
    let referenced: Vec<VertexId> = referenced_vertices(&mesh);
    let n_referenced = referenced.len();
    assert!(
        n_referenced > 0,
        "{label}: sphere mesh at cell_size = {cell_size} has zero referenced vertices",
    );

    // Snapshot boundary-faces + rest_positions BEFORE the solver moves
    // the mesh. Both are used by the finest-only PLY + Bevy paths.
    let boundary_faces: Vec<[VertexId; 3]> = mesh.boundary_faces().to_vec();
    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();

    // Snapshot rest-z values for COM displacement. Only z is needed —
    // x/y rest values aren't read (active-pair walk reads x_final's
    // x/y for the contact-patch radius).
    let rest_z: Vec<f64> = rest_positions.iter().map(|p| p.z).collect();

    let SceneInitial { x_prev, v_prev } = initial;

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;

    let solver: PenaltyRigidContactSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    let step: NewtonStep<_> = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);

    // COM displacement Δz_COM = mean(z_v_final - z_v_initial) over
    // referenced vertices. Mirror V-3 verbatim — referenced-only mean
    // tracks the actual sphere body rather than the static auto-pinned
    // orphan cohort.
    let z_disp_sum: f64 = referenced
        .iter()
        .map(|&v| {
            let v_idx = v as usize;
            step.x_final[3 * v_idx + 2] - rest_z[v_idx]
        })
        .sum();
    let delta_z_com = z_disp_sum / n_referenced as f64;

    // Hertz convention: δ_FEM = -d̂ - Δz_COM. At rest sphere center
    // sits at z = 0, distance R + d̂ from plane; first-contact (rigid
    // limit) has center at z = -d̂; loaded center is at z = -d̂ -
    // δ_Hertz, so Δz_COM = -d̂ - δ_Hertz ⇒ δ_FEM = -d̂ - Δz_COM.
    let delta_fem = -PENALTY_DHAT - delta_z_com;

    // Contact-patch walk. Plane: outward normal `+ẑ`, offset = -(R +
    // d̂). signed_distance(p) = p.z - offset = z_v + R + d̂. Active
    // when sd < d̂, i.e., z_v < -R.
    //
    // For each active vertex, record horizontal radius `sqrt(x² + y²)`
    // and penalty z-force `κ · (d̂ - sd)`. `a_FEM = max r` over
    // active vertices captures the contact-patch boundary directly.
    let plane_offset = -(RADIUS + PENALTY_DHAT);
    let mut a_fem: f64 = 0.0;
    let mut n_active_pairs: usize = 0;
    let mut contact_vertices: Vec<ContactVertex> = Vec::new();
    for &v in &referenced {
        let v_idx = v as usize;
        let z_final = step.x_final[3 * v_idx + 2];
        let sd = z_final - plane_offset;
        if sd < PENALTY_DHAT {
            n_active_pairs += 1;
            let x_final = step.x_final[3 * v_idx];
            let y_final = step.x_final[3 * v_idx + 1];
            let r_horiz = x_final.hypot(y_final);
            if r_horiz > a_fem {
                a_fem = r_horiz;
            }
            if populate_contact_vertices {
                let force_z = KAPPA * (PENALTY_DHAT - sd);
                contact_vertices.push(ContactVertex {
                    v,
                    r: r_horiz,
                    sd,
                    force_z,
                });
            }
        }
    }

    RefinementSnapshot {
        cell_size,
        label,
        delta_fem,
        a_fem,
        n_active_pairs,
        iter_count: step.iter_count,
        residual_norm: step.final_residual_norm,
        n_tets,
        n_vertices,
        n_loaded,
        n_pinned,
        n_referenced,
        x_final: step.x_final,
        rest_positions,
        boundary_faces,
        contact_vertices,
    }
}

/// Run all three refinements and assemble the [`HertzSnapshot`].
fn run_hertz_sweep() -> HertzSnapshot {
    let h = run_at_refinement(CELL_SIZE_H, LABEL_H, false);
    let h2 = run_at_refinement(CELL_SIZE_H2, LABEL_H2, false);
    let h4 = run_at_refinement(CELL_SIZE_H4, LABEL_H4, true);

    let e_star_val = e_star(MU, LAMBDA);
    let delta_hertz_val = delta_hertz(FORCE, RADIUS, e_star_val);
    let a_hertz_val = a_hertz(FORCE, RADIUS, e_star_val);
    let p0_hertz_val = p_zero_hertz(FORCE, a_hertz_val);

    let rel_err_a_h = (h.a_fem - a_hertz_val).abs() / a_hertz_val;
    let rel_err_a_h2 = (h2.a_fem - a_hertz_val).abs() / a_hertz_val;
    let rel_err_a_h4 = (h4.a_fem - a_hertz_val).abs() / a_hertz_val;
    let rel_err_delta_h4 = (h4.delta_fem - delta_hertz_val).abs() / delta_hertz_val;

    let cauchy_step_coarse_a = (h.a_fem - h2.a_fem).abs();
    let cauchy_step_fine_a = (h2.a_fem - h4.a_fem).abs();
    let cauchy_ratio_a = if cauchy_step_coarse_a > 0.0 {
        cauchy_step_fine_a / cauchy_step_coarse_a
    } else {
        f64::INFINITY
    };

    HertzSnapshot {
        h,
        h2,
        h4,
        e_star: e_star_val,
        delta_hertz: delta_hertz_val,
        a_hertz: a_hertz_val,
        p0_hertz: p0_hertz_val,
        rel_err_a_h,
        rel_err_a_h2,
        rel_err_a_h4,
        rel_err_delta_h4,
        cauchy_step_coarse_a,
        cauchy_step_fine_a,
        cauchy_ratio_a,
    }
}

// =============================================================================
// Capture-bit dump — always runs first so failure-mode protocol has the
// actuals visible even when later assertions panic.
// =============================================================================

fn print_capture_block(snapshot: &HertzSnapshot) {
    eprintln!();
    eprintln!("==== Capture-bit reference values (paste into source on first bake) ====");
    eprintln!();
    eprintln!("// Topology exact-pins");
    eprintln!("const N_TETS_H: usize       = {:>8};", snapshot.h.n_tets);
    eprintln!("const N_TETS_H2: usize      = {:>8};", snapshot.h2.n_tets);
    eprintln!("const N_TETS_H4: usize      = {:>8};", snapshot.h4.n_tets);
    eprintln!(
        "const N_VERTICES_H: usize   = {:>8};",
        snapshot.h.n_vertices
    );
    eprintln!(
        "const N_VERTICES_H2: usize  = {:>8};",
        snapshot.h2.n_vertices
    );
    eprintln!(
        "const N_VERTICES_H4: usize  = {:>8};",
        snapshot.h4.n_vertices
    );
    eprintln!(
        "const N_REFERENCED_H: usize  = {:>8};",
        snapshot.h.n_referenced
    );
    eprintln!(
        "const N_REFERENCED_H2: usize = {:>8};",
        snapshot.h2.n_referenced
    );
    eprintln!(
        "const N_REFERENCED_H4: usize = {:>8};",
        snapshot.h4.n_referenced
    );
    eprintln!("const N_LOADED_H: usize     = {:>8};", snapshot.h.n_loaded);
    eprintln!("const N_LOADED_H2: usize    = {:>8};", snapshot.h2.n_loaded);
    eprintln!("const N_LOADED_H4: usize    = {:>8};", snapshot.h4.n_loaded);
    eprintln!("const N_PINNED_H: usize     = {:>8};", snapshot.h.n_pinned);
    eprintln!("const N_PINNED_H2: usize    = {:>8};", snapshot.h2.n_pinned);
    eprintln!("const N_PINNED_H4: usize    = {:>8};", snapshot.h4.n_pinned);
    eprintln!();
    eprintln!("// Hertz-metric captured bits");
    eprintln!(
        "const A_FEM_H_REF_BITS: u64       = 0x{:016x}; // a_FEM at h     = {:e}",
        snapshot.h.a_fem.to_bits(),
        snapshot.h.a_fem,
    );
    eprintln!(
        "const A_FEM_H2_REF_BITS: u64      = 0x{:016x}; // a_FEM at h/2   = {:e}",
        snapshot.h2.a_fem.to_bits(),
        snapshot.h2.a_fem,
    );
    eprintln!(
        "const A_FEM_H4_REF_BITS: u64      = 0x{:016x}; // a_FEM at h/4   = {:e}",
        snapshot.h4.a_fem.to_bits(),
        snapshot.h4.a_fem,
    );
    eprintln!(
        "const DELTA_FEM_H4_REF_BITS: u64  = 0x{:016x}; // δ_FEM at h/4   = {:e}",
        snapshot.h4.delta_fem.to_bits(),
        snapshot.h4.delta_fem,
    );
    eprintln!(
        "const N_ACTIVE_H_REF: usize       = {:>8};                       // n_active at h",
        snapshot.h.n_active_pairs,
    );
    eprintln!(
        "const N_ACTIVE_H2_REF: usize      = {:>8};                       // n_active at h/2",
        snapshot.h2.n_active_pairs,
    );
    eprintln!(
        "const N_ACTIVE_H4_REF: usize      = {:>8};                       // n_active at h/4",
        snapshot.h4.n_active_pairs,
    );
    eprintln!(
        "const ITER_COUNT_H_REF: usize     = {:>8};                       // iter at h",
        snapshot.h.iter_count,
    );
    eprintln!(
        "const ITER_COUNT_H2_REF: usize    = {:>8};                       // iter at h/2",
        snapshot.h2.iter_count,
    );
    eprintln!(
        "const ITER_COUNT_H4_REF: usize    = {:>8};                       // iter at h/4",
        snapshot.h4.iter_count,
    );
    eprintln!();
    eprintln!("==== End capture-bit block ====");
    eprintln!();
}

// =============================================================================
// Anchor 1 — geometry_invariants (compile-time)
// =============================================================================

const fn verify_geometry_invariants() {
    const { assert!(RADIUS > 0.0, "RADIUS must be positive") };
    const { assert!(FORCE > 0.0, "FORCE must be positive (downward press)") };
    const { assert!(MU > 0.0, "MU must be positive") };
    const { assert!(LAMBDA > 0.0, "LAMBDA must be positive") };
    const { assert!(KAPPA > 0.0, "KAPPA must be positive") };
    const { assert!(PENALTY_DHAT > 0.0, "PENALTY_DHAT must be positive") };
    const { assert!(STATIC_DT > 0.0, "STATIC_DT must be positive") };
    const {
        assert!(
            CELL_SIZE_H > CELL_SIZE_H2 && CELL_SIZE_H2 > CELL_SIZE_H4 && CELL_SIZE_H4 > 0.0,
            "Cell sizes must satisfy h > h/2 > h/4 > 0",
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
            REL_ERR_GATE > 0.0 && REL_ERR_GATE < 1.0,
            "REL_ERR_GATE must be in (0, 1)",
        );
    };
    const {
        assert!(
            SMALL_STRAIN_CEILING > 0.0 && SMALL_STRAIN_CEILING < 1.0,
            "SMALL_STRAIN_CEILING must be in (0, 1)",
        );
    };
}

// =============================================================================
// Anchor 2 — mesh_topology_exact (per-refinement III-1 contract)
// =============================================================================

fn verify_mesh_topology_exact(snapshot: &HertzSnapshot) {
    let triples: [(&RefinementSnapshot, usize, usize, usize, usize, usize); 3] = [
        (
            &snapshot.h,
            N_TETS_H,
            N_VERTICES_H,
            N_REFERENCED_H,
            N_LOADED_H,
            N_PINNED_H,
        ),
        (
            &snapshot.h2,
            N_TETS_H2,
            N_VERTICES_H2,
            N_REFERENCED_H2,
            N_LOADED_H2,
            N_PINNED_H2,
        ),
        (
            &snapshot.h4,
            N_TETS_H4,
            N_VERTICES_H4,
            N_REFERENCED_H4,
            N_LOADED_H4,
            N_PINNED_H4,
        ),
    ];
    for (rs, n_tets_ref, n_vertices_ref, n_referenced_ref, n_loaded_ref, n_pinned_ref) in triples {
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
            rs.n_referenced, n_referenced_ref,
            "{}: n_referenced drift — expected {n_referenced_ref}, got {}",
            rs.label, rs.n_referenced,
        );
        assert_eq!(
            rs.n_loaded, n_loaded_ref,
            "{}: n_loaded drift — expected {n_loaded_ref}, got {}",
            rs.label, rs.n_loaded,
        );
        assert_eq!(
            rs.n_pinned, n_pinned_ref,
            "{}: n_pinned drift — expected {n_pinned_ref}, got {}",
            rs.label, rs.n_pinned,
        );
        assert!(
            rs.n_referenced < rs.n_vertices,
            "{}: BCC orphan-rejection invariant non-vacuous: referenced ({}) < n_vertices ({})",
            rs.label,
            rs.n_referenced,
            rs.n_vertices,
        );
    }
}

// =============================================================================
// Anchor 3 — boundary_partition (per-refinement)
// =============================================================================

fn verify_boundary_partition(snapshot: &HertzSnapshot) {
    for rs in [&snapshot.h, &snapshot.h2, &snapshot.h4] {
        assert!(
            rs.n_pinned >= 3,
            "{}: equator pin set must contain ≥ 3 distinct vertices for rigid-body mode \
             removal — got {}; mesh resolution may be too coarse to hit four distinct \
             cardinal-equator points",
            rs.label,
            rs.n_pinned,
        );
        assert!(
            rs.n_loaded > 0,
            "{}: loaded vertex set is empty — sphere top-band traction has nothing to drive \
             through; check `band_tol = 0.5 * cell_size` against mesher cut-point density at \
             cell_size = {}",
            rs.label,
            rs.cell_size,
        );
    }
}

// =============================================================================
// Anchor 4 — solver_per_step_invariants (per-refinement)
// =============================================================================

fn verify_solver_per_step_invariants(snapshot: &HertzSnapshot) {
    for rs in [&snapshot.h, &snapshot.h2, &snapshot.h4] {
        for (i, &x) in rs.x_final.iter().enumerate() {
            assert!(
                x.is_finite(),
                "{}: x_final[{i}] = {x} is not finite — Newton diverged",
                rs.label,
            );
        }
        assert!(
            rs.iter_count < NEWTON_ITER_SANITY_CAP,
            "{}: Newton ran {} iters, ≥ sanity cap {NEWTON_ITER_SANITY_CAP} (= MAX_NEWTON_ITER \
             {MAX_NEWTON_ITER} − 10) — investigate solver / penalty regime regression before \
             bumping the cap",
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
// Anchor 5 — contact_engagement (per-refinement)
// =============================================================================

fn verify_contact_engagement(snapshot: &HertzSnapshot) {
    for rs in [&snapshot.h, &snapshot.h2, &snapshot.h4] {
        assert!(
            rs.n_active_pairs > 0,
            "{}: n_active_pairs = 0 — sphere has not contacted the plane; check that the \
             loaded-band traction is driving the south pole into the contact band",
            rs.label,
        );
    }
}

// =============================================================================
// Anchor 6 — small_strain_validity (a/R << 1 per refinement)
// =============================================================================

fn verify_small_strain_validity(snapshot: &HertzSnapshot) {
    for rs in [&snapshot.h, &snapshot.h2, &snapshot.h4] {
        let a_over_r = rs.a_fem / RADIUS;
        assert!(
            rs.a_fem > 0.0,
            "{}: a_FEM = {:e} m must be strictly positive (partial contact)",
            rs.label,
            rs.a_fem,
        );
        assert!(
            a_over_r < SMALL_STRAIN_CEILING,
            "{}: a_FEM / R = {a_over_r:.4} ≥ Hertz small-strain ceiling \
             {SMALL_STRAIN_CEILING:.2} — Hertz `a/R << 1` validity domain violated; investigate \
             before relaxing",
            rs.label,
        );
    }
}

// =============================================================================
// Anchor 7 — monotonic_a_convergence (rel_err_a decreases h → h/2 → h/4)
// =============================================================================

fn verify_monotonic_a_convergence(snapshot: &HertzSnapshot) {
    assert!(
        snapshot.rel_err_a_h2 < snapshot.rel_err_a_h,
        "h/2 rel_err_a = {:.4} not below h rel_err_a = {:.4} — non-monotonic convergence \
         indicates resolution-induced scatter or a regression in contact-patch resolution \
         with mesh density",
        snapshot.rel_err_a_h2,
        snapshot.rel_err_a_h,
    );
    assert!(
        snapshot.rel_err_a_h4 < snapshot.rel_err_a_h2,
        "h/4 rel_err_a = {:.4} not below h/2 rel_err_a = {:.4} — non-monotonic convergence \
         from mid- to fine-refinement; investigate before relaxing the bound",
        snapshot.rel_err_a_h4,
        snapshot.rel_err_a_h2,
    );
}

// =============================================================================
// Anchor 8 — cauchy_a_convergence (Cauchy ratio < 1 on a_FEM)
// =============================================================================

fn verify_cauchy_a_convergence(snapshot: &HertzSnapshot) {
    assert!(
        snapshot.cauchy_ratio_a < 1.0,
        "Cauchy ratio (a_FEM) {:.4} (|Δ_fine| / |Δ_coarse|) ≥ 1.0 — a_FEM sequence is not \
         geometrically converging across (h, h/2, h/4). |a_h - a_h2| = {:.4e} m, \
         |a_h2 - a_h4| = {:.4e} m",
        snapshot.cauchy_ratio_a,
        snapshot.cauchy_step_coarse_a,
        snapshot.cauchy_step_fine_a,
    );
}

// =============================================================================
// Anchor 9 — finest_level_hertz_match (rel_err_a at h/4 < REL_ERR_GATE)
// =============================================================================

fn verify_finest_level_hertz_match(snapshot: &HertzSnapshot) {
    assert!(
        snapshot.rel_err_a_h4 < REL_ERR_GATE,
        "h/4 rel_err_a = {:.4} ≥ REL_ERR_GATE = {REL_ERR_GATE:.2} — Hertz patch-radius \
         comparison fails the finest-level gate. a_FEM = {:e} m, a_Hertz = {:e} m. Mesh-bound \
         tightening to <10% is Phase H Tet10 work; failure at 20% is a Phase 5 contact-machinery \
         regression worth investigating before relaxing.",
        snapshot.rel_err_a_h4,
        snapshot.h4.a_fem,
        snapshot.a_hertz,
    );
}

// =============================================================================
// Anchor 10 — captured_bits_hertz_metrics (IV-1 sparse-tier rel-tol)
// =============================================================================

fn verify_captured_bits_hertz_metrics(snapshot: &HertzSnapshot) {
    let a_fem_h_ref = f64::from_bits(A_FEM_H_REF_BITS);
    let a_fem_h2_ref = f64::from_bits(A_FEM_H2_REF_BITS);
    let a_fem_h4_ref = f64::from_bits(A_FEM_H4_REF_BITS);
    let delta_fem_h4_ref = f64::from_bits(DELTA_FEM_H4_REF_BITS);

    assert_relative_eq!(
        snapshot.h.a_fem,
        a_fem_h_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.h2.a_fem,
        a_fem_h2_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.h4.a_fem,
        a_fem_h4_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_relative_eq!(
        snapshot.h4.delta_fem,
        delta_fem_h4_ref,
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
    assert_eq!(
        snapshot.h.n_active_pairs, N_ACTIVE_H_REF,
        "n_active drift at h: expected {N_ACTIVE_H_REF}, got {}",
        snapshot.h.n_active_pairs,
    );
    assert_eq!(
        snapshot.h2.n_active_pairs, N_ACTIVE_H2_REF,
        "n_active drift at h/2: expected {N_ACTIVE_H2_REF}, got {}",
        snapshot.h2.n_active_pairs,
    );
    assert_eq!(
        snapshot.h4.n_active_pairs, N_ACTIVE_H4_REF,
        "n_active drift at h/4: expected {N_ACTIVE_H4_REF}, got {}",
        snapshot.h4.n_active_pairs,
    );
    assert_eq!(
        snapshot.h.iter_count, ITER_COUNT_H_REF,
        "iter_count drift at h: expected {ITER_COUNT_H_REF}, got {}",
        snapshot.h.iter_count,
    );
    assert_eq!(
        snapshot.h2.iter_count, ITER_COUNT_H2_REF,
        "iter_count drift at h/2: expected {ITER_COUNT_H2_REF}, got {}",
        snapshot.h2.iter_count,
    );
    assert_eq!(
        snapshot.h4.iter_count, ITER_COUNT_H4_REF,
        "iter_count drift at h/4: expected {ITER_COUNT_H4_REF}, got {}",
        snapshot.h4.iter_count,
    );
}

// =============================================================================
// JSON emit — three sections (scalars + finest per-vertex + 200pt analytic)
// =============================================================================

const ANALYTIC_SAMPLES: usize = 200;

fn save_hertz_json(snapshot: &HertzSnapshot, path: &Path) -> Result<()> {
    let scalars = json!({
        "e_star":           snapshot.e_star,
        "force_applied":    FORCE,
        "radius":           RADIUS,
        "mu":               MU,
        "lambda":           LAMBDA,
        "kappa":            KAPPA,
        "d_hat":            PENALTY_DHAT,
        "a_hertz":          snapshot.a_hertz,
        "delta_hertz":      snapshot.delta_hertz,
        "p0_hertz":         snapshot.p0_hertz,
        "a_fem_h":          snapshot.h.a_fem,
        "a_fem_h2":         snapshot.h2.a_fem,
        "a_fem_h4":         snapshot.h4.a_fem,
        "delta_fem_h4":     snapshot.h4.delta_fem,
        "rel_err_a_h":      snapshot.rel_err_a_h,
        "rel_err_a_h2":     snapshot.rel_err_a_h2,
        "rel_err_a_h4":     snapshot.rel_err_a_h4,
        "rel_err_delta_h4": snapshot.rel_err_delta_h4,
        "cauchy_ratio_a":   snapshot.cauchy_ratio_a,
        "n_active_h":       snapshot.h.n_active_pairs,
        "n_active_h2":      snapshot.h2.n_active_pairs,
        "n_active_h4":      snapshot.h4.n_active_pairs,
        "iter_count_h":     snapshot.h.iter_count,
        "iter_count_h2":    snapshot.h2.iter_count,
        "iter_count_h4":    snapshot.h4.iter_count,
        "cell_size_h":      CELL_SIZE_H,
        "cell_size_h2":     CELL_SIZE_H2,
        "cell_size_h4":     CELL_SIZE_H4,
        "rel_err_gate":     REL_ERR_GATE,
    });

    let vertices: Vec<Value> = snapshot
        .h4
        .contact_vertices
        .iter()
        .map(|c| {
            json!({
                "v":       c.v,
                "r":       c.r,
                "sd":      c.sd,
                "force_z": c.force_z,
            })
        })
        .collect();

    // 200-point Hertz analytic curve `p(r) = p₀ · sqrt(1 - (r/a)²)` on
    // r ∈ [0, a_Hertz]. Endpoint inclusive: at r = a_Hertz, p = 0.
    let mut analytic: Vec<Value> = Vec::with_capacity(ANALYTIC_SAMPLES);
    for i in 0..ANALYTIC_SAMPLES {
        let t = i as f64 / (ANALYTIC_SAMPLES - 1) as f64;
        let r = t * snapshot.a_hertz;
        let inside = 1.0 - t * t;
        let p = snapshot.p0_hertz * inside.max(0.0).sqrt();
        analytic.push(json!({ "r": r, "p_hertz": p }));
    }

    let root = json!({
        "scalars":  scalars,
        "vertices": vertices,
        "analytic": analytic,
    });

    let json_str = serde_json::to_string_pretty(&root).context("serialize Hertz JSON")?;
    std::fs::write(path, json_str)
        .with_context(|| format!("write Hertz JSON to {}", path.display()))?;
    Ok(())
}

// =============================================================================
// PLY emit — finest-refinement deformed boundary mesh + contact_force_z
// =============================================================================

fn save_finest_frame_ply(snapshot: &HertzSnapshot, path: &Path) -> Result<()> {
    let h4 = &snapshot.h4;
    let n_vertices = h4.n_vertices;

    // Build positions in physics +Z frame (no swap — cf-view handles
    // orientation). Positions are unscaled physics — scale-invariance
    // is the point of decoupling RENDER_SCALE from the headless paths.
    let vertices: Vec<Point3<f64>> = (0..n_vertices)
        .map(|i| {
            Point3::new(
                h4.x_final[3 * i],
                h4.x_final[3 * i + 1],
                h4.x_final[3 * i + 2],
            )
        })
        .collect();

    // Boundary-face indices as CCW-wound triangles. Outward winding
    // from the right-handed-tet `signed_volume > 0` invariant per
    // `boundary_faces_from_topology`.
    let faces: Vec<[u32; 3]> = h4
        .boundary_faces
        .iter()
        .map(|f| [f[0], f[1], f[2]])
        .collect();

    let geometry = IndexedMesh::from_parts(vertices, faces);
    let mut mesh = AttributedMesh::new(geometry);
    // Smooth normals for cf-view PBR — mirror row 12.
    mesh.compute_normals();

    // Per-vertex contact-force scalar: zero for inactive vertices,
    // `κ · (d̂ - sd)` for active. cf-view's auto-colormap (sequential
    // viridis on positive scalar) renders the contact patch as a
    // bright disk on the deformed sphere — fills row 12's "contact-band
    // membership is a row-13 Hertz-patch concern" deferral.
    let mut contact_force_z: Vec<f32> = vec![0.0; n_vertices];
    for c in &h4.contact_vertices {
        contact_force_z[c.v as usize] = c.force_z as f32;
    }
    mesh.insert_extra("contact_force_z", contact_force_z)
        .context("insert contact_force_z extra (length must equal n_vertices)")?;

    save_ply_attributed(&mesh, path, true)?;
    Ok(())
}

// =============================================================================
// Bevy visual mode — opt-in via `CF_VISUAL=1` (static finest + annuli)
// =============================================================================

/// Render-side scale factor on visual entities. Mirror row 12 verbatim
/// — Bevy 0.18's pipeline defaults are tuned for human-scale (1 m+)
/// scenes, and at sim-soft's cm-scale rendering the camera approaches
/// the near plane on any zoom-in. RENDER_SCALE = 100 lifts the scene
/// past the defaults. Headless asserts + JSON + PLY are scale-invariant.
const RENDER_SCALE: f32 = 100.0;

/// Resource carrying the headless-harness outputs the Bevy `Startup`
/// system needs to spawn the soft-mesh + plane + annuli + camera
/// entities. Mirrors row 12's `VisualSetup` shape — wrapped in
/// `Option` so `setup_visual_scene` `.take()`s it once.
#[derive(Resource)]
struct VisualSetup(Option<VisualSetupInner>);

struct VisualSetupInner {
    /// Single-frame trajectory carrying the finest x_final. With one
    /// frame, `step_replay` writes the deformed positions once and
    /// `frame_index_at` clamps at end (no animation, no looping).
    trajectory: Trajectory,
    rest_positions: Vec<Vec3>,
    boundary_faces: Vec<[VertexId; 3]>,
    /// FEM-measured patch radius (m, physics scale). Annulus inner +
    /// outer radii derive from this via
    /// `× RENDER_SCALE × (1.0 ∓ RING_THICKNESS_FRAC)` where
    /// `RING_THICKNESS_FRAC = 0.025` (`× 0.975` inner, `× 1.025`
    /// outer).
    a_fem: f32,
    /// Hertz analytic patch radius (m, physics scale).
    a_hertz: f32,
}

fn run_visual_mode(snapshot: &HertzSnapshot) {
    let trajectory = Trajectory {
        frames: vec![snapshot.h4.x_final.clone()],
        // dt is irrelevant at frames.len() == 1 (frame_index_at clamps
        // to 0); set to STATIC_DT so the math is well-defined.
        dt: STATIC_DT,
    };
    let visual_setup = VisualSetupInner {
        trajectory,
        rest_positions: snapshot.h4.rest_positions.clone(),
        boundary_faces: snapshot.h4.boundary_faces.clone(),
        a_fem: snapshot.h4.a_fem as f32,
        a_hertz: snapshot.a_hertz as f32,
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
        a_fem,
        a_hertz,
    }) = visual_setup.0.take()
    else {
        return;
    };

    // Soft-mesh entity: built from rest config, animated to the single
    // captured frame on first step_replay tick. Coral PBR matches row
    // 12 for cross-row visual consistency.
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

    // Rigid plane visualization — gray PBR quad at Bevy y = -(R+d̂) ·
    // RENDER_SCALE under PlusZ swap (physics z = -(R+d̂) is plane
    // surface). Plane mesh half_size = 4 × R × RENDER_SCALE = 4 m.
    let plane_y = -((RADIUS + PENALTY_DHAT) as f32) * RENDER_SCALE;
    let plane_half_size = 4.0 * RADIUS as f32 * RENDER_SCALE;
    let plane_mesh = meshes.add(Plane3d::new(BevyVec3::Y, Vec2::splat(plane_half_size)));
    let plane_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.7, 0.7, 0.7),
        perceptual_roughness: 0.9,
        ..default()
    });
    commands.spawn((
        Mesh3d(plane_mesh),
        MeshMaterial3d(plane_material),
        Transform::from_xyz(0.0, plane_y, 0.0),
    ));

    // Patch-radius annuli — thin rings sitting just above the plane in
    // Bevy +Y. Bevy's Annulus is a 2D primitive in its mesh-local XY
    // plane; rotate by -π/2 around X so the disc faces +Y (lays flat
    // on the floor).
    //
    // ring_y placement: critical that the ring sits CLOSE to the plane
    // (just above, well below the sphere body). The plane lives at
    // `plane_y = -(R + d̂) · RENDER_SCALE = -1.1` Bevy; the deformed
    // sphere south pole sits near `-R · RENDER_SCALE = -1.0` Bevy.
    // The 0.1-Bevy contact-band gap between them is where the rings go
    // — pin at `plane_y + 0.005` = `-1.095` Bevy (5 cm Bevy above
    // plane = 50 μm physics, well above depth-buffer z-fight floor at
    // ~3 m camera distance, well below the deformed sphere body that
    // would otherwise occlude rings placed too high).
    //
    // RING_THICKNESS_FRAC = 0.025 — at radii 0.15-0.178 Bevy and the
    // camera distance below, this is large enough to read as a clearly
    // visible ring without overlapping the FEM/Hertz radial gap (gap
    // is 16% of radius; ring half-thickness is 2.5% per ring, so 5%
    // combined coverage leaves 11% radial gap clear between them).
    let ring_y = plane_y + 0.005_f32;
    let ring_thickness_frac: f32 = 0.025;
    let ring_rotation = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);

    // Coral a_FEM annulus — matches the soft-mesh PBR for "this is the
    // body's contact radius" reading.
    let r_fem_render = a_fem * RENDER_SCALE;
    let fem_annulus = meshes.add(Annulus::new(
        r_fem_render * (1.0 - ring_thickness_frac),
        r_fem_render * (1.0 + ring_thickness_frac),
    ));
    // Unlit + double_sided + cull_mode None — annuli must render from
    // both faces (Bevy's `Annulus` 2D primitive carries a single-side
    // winding; under the -π/2 X-rotation that lays the disc flat, the
    // back-face culls when the camera is above the disc). Unlit is
    // intentional — the ring colors should read identically regardless
    // of camera angle, since they encode pure-quantity scalars (not
    // surface lighting).
    let fem_material = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.55, 0.4),
        unlit: true,
        double_sided: true,
        cull_mode: None,
        ..default()
    });
    commands.spawn((
        Mesh3d(fem_annulus),
        MeshMaterial3d(fem_material),
        Transform {
            translation: BevyVec3::new(0.0, ring_y, 0.0),
            rotation: ring_rotation,
            ..default()
        },
    ));

    // White a_Hertz annulus — analytic reference, "the truth"; high
    // contrast against the gray plane and the coral FEM ring.
    let r_hertz_render = a_hertz * RENDER_SCALE;
    let hertz_annulus = meshes.add(Annulus::new(
        r_hertz_render * (1.0 - ring_thickness_frac),
        r_hertz_render * (1.0 + ring_thickness_frac),
    ));
    let hertz_material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        unlit: true,
        double_sided: true,
        cull_mode: None,
        ..default()
    });
    // Both annuli at the same `ring_y` — they have non-overlapping
    // radii (a_FEM = 1.50 mm vs a_Hertz = 1.78 mm physics), so no
    // z-fighting between them; staggering by a sub-mm Bevy offset is
    // unnecessary and hides the radial gap visually.
    commands.spawn((
        Mesh3d(hertz_annulus),
        MeshMaterial3d(hertz_material),
        Transform {
            translation: BevyVec3::new(0.0, ring_y, 0.0),
            rotation: ring_rotation,
            ..default()
        },
    ));

    // Camera framing — full-scene default: sphere + plane both
    // visible with the patch annuli readable as tiny markers at the
    // contact zone. Distance 3R, target -0.5R (between sphere center
    // and contact zone), angles `(0.4, 0.5)` rad (~23° azimuth, ~29°
    // elevation). Inherits row 12's "full-scene framing" convention
    // but the specific values diverge from row 12's `(0.6, 0.4)` rad
    // + 15R distance + target at sphere COM (Bevy `y = (R + d̂) ·
    // RENDER_SCALE = 1.1`): row 13's contact zone is BELOW the sphere
    // center (south pole pressing), so target is shifted lower (Bevy
    // `y = -0.5 · R · RENDER_SCALE = -0.5`); row 13 closes distance
    // 15R → 3R since the row's "interesting feature" is contact-zone
    // detail rather than full freefall trajectory. The patch rings
    // (radii 0.15-0.178 Bevy on a sphere of radius 1.0 Bevy) appear
    // small from this distance — scroll-wheel zoom-in is required to
    // inspect the FEM/Hertz radial gap up close (see README "Bevy
    // visualization" section). Keeping the default at the full-scene
    // framing prioritizes spatial-context-on-startup over
    // ring-detail-on-startup; user-orbit + scroll-zoom drive the
    // detail review.
    let target_y = -0.5 * RADIUS as f32 * RENDER_SCALE;
    commands.spawn((
        Camera3d::default(),
        Transform::default(),
        OrbitCamera::new()
            .with_target(BevyVec3::new(0.0, target_y, 0.0))
            .with_distance(3.0 * RADIUS as f32 * RENDER_SCALE)
            .with_angles(0.4, 0.5),
        AmbientLight {
            color: Color::WHITE,
            brightness: 80.0,
            ..default()
        },
    ));

    // Directional light from upper-front-right so the sphere's
    // camera-facing hemisphere is the lit one — mirror row 12.
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(0.5, 1.0, 0.5).looking_at(BevyVec3::ZERO, BevyVec3::Y),
    ));

    // HUD — both panels pinned to the LEFT edge so multi-line text
    // grows rightward without truncating against the right-edge
    // viewport boundary. Asserted scalars top-left (read alongside the
    // visual rings); controls bottom-left (mirror row 12).
    commands.spawn((
        Text::new(format!(
            "a_FEM (coral)  = {:.4} mm\na_Hertz (white) = {:.4} mm\nrel_err_a       = {:.2} %",
            a_fem * 1000.0,
            a_hertz * 1000.0,
            ((a_fem - a_hertz).abs() / a_hertz) * 100.0,
        )),
        TextFont {
            font_size: 14.0,
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

fn print_summary(snapshot: &HertzSnapshot, ply_path: &Path, json_path: &Path) {
    println!("==== hertz-sphere-plane ====");
    println!();
    println!("Scene: SoftScene::sphere_on_plane (V-3 mirror)");
    println!(
        "  geometry      : SphereSdf body, RADIUS = {RADIUS} m, FORCE = {FORCE} N (axial press)"
    );
    println!(
        "  refinements   : h = {CELL_SIZE_H} m, h/2 = {CELL_SIZE_H2} m, h/4 = {CELL_SIZE_H4} m"
    );
    println!(
        "  material      : NH(MU = {MU:e}, LAMBDA = {LAMBDA:e})  Ecoflex 00-30 + 15 wt% \
         carbon-black, ν = 0.4"
    );
    println!(
        "  rigid plane   : RigidPlane(normal = +ẑ, offset = -(R + d̂) = {:e}) (kinematic, one-way)",
        -(RADIUS + PENALTY_DHAT),
    );
    println!(
        "  contact       : PenaltyRigidContact V-3-LOCAL (κ = {KAPPA} N/m, d̂ = {PENALTY_DHAT} m)"
    );
    println!("  solver config : Δt = {STATIC_DT} s (static), max_newton_iter = {MAX_NEWTON_ITER}");
    println!();
    println!("Hertzian closed-form (Johnson 1985 §3.4):");
    println!("  E*       = {:>13.6e} Pa", snapshot.e_star);
    println!(
        "  δ_Hertz  = {:>13.6e} m   ({:.4} mm)  — diagnostic-only (penalty unreachable)",
        snapshot.delta_hertz,
        snapshot.delta_hertz * 1000.0,
    );
    println!(
        "  a_Hertz  = {:>13.6e} m   ({:.4} mm)  — asserted comparison target",
        snapshot.a_hertz,
        snapshot.a_hertz * 1000.0,
    );
    println!(
        "  p₀_Hertz = {:>13.6e} Pa  ({:.4} kPa)",
        snapshot.p0_hertz,
        snapshot.p0_hertz / 1000.0,
    );
    println!();
    println!("Per-refinement results:");
    for rs in [&snapshot.h, &snapshot.h2, &snapshot.h4] {
        println!(
            "  {:>4}: cell_size = {:>7.4e} m  n_tets = {:>5}  n_active = {:>3}  iter = {:>2}  \
             a_FEM = {:>10.4e} m  δ_FEM = {:>11.4e} m",
            rs.label,
            rs.cell_size,
            rs.n_tets,
            rs.n_active_pairs,
            rs.iter_count,
            rs.a_fem,
            rs.delta_fem,
        );
    }
    println!();
    println!("Hertz patch-radius convergence:");
    println!(
        "  rel_err_a      h: {:>8.4} %",
        snapshot.rel_err_a_h * 100.0,
    );
    println!(
        "  rel_err_a    h/2: {:>8.4} %",
        snapshot.rel_err_a_h2 * 100.0,
    );
    println!(
        "  rel_err_a    h/4: {:>8.4} %  (gate: < {:.2} %)",
        snapshot.rel_err_a_h4 * 100.0,
        REL_ERR_GATE * 100.0,
    );
    println!(
        "  Cauchy ratio    : {:>8.4}  (gate: < 1.0; geometric convergence)",
        snapshot.cauchy_ratio_a,
    );
    println!(
        "  rel_err_δ    h/4: {:>8.4} %  (diagnostic-only — penalty compliance unreachable)",
        snapshot.rel_err_delta_h4 * 100.0,
    );
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!("  geometry_invariants               : compile-time const asserts");
    println!("  mesh_topology_exact               : per-refinement III-1 contract");
    println!(
        "  boundary_partition                : ≥ 3 equator pins (helper picks 4, dedups) + nonempty top-band loaded"
    );
    println!(
        "  solver_per_step_invariants        : no NaN, iter < {NEWTON_ITER_SANITY_CAP}, \
         residual finite"
    );
    println!("  contact_engagement                : n_active > 0 per refinement");
    println!(
        "  small_strain_validity             : a/R < {SMALL_STRAIN_CEILING:.2} (Hertz domain)"
    );
    println!("  monotonic_a_convergence           : rel_err_a decreases h → h/2 → h/4");
    println!("  cauchy_a_convergence              : |Δ_fine| < |Δ_coarse|");
    println!(
        "  finest_level_hertz_match          : rel_err_a at h/4 < REL_ERR_GATE = \
         {REL_ERR_GATE:.2}"
    );
    println!(
        "  captured_bits_hertz_metrics       : 4 f64 bits + 6 usize within \
         IV-1 sparse-tier rel-tol = {SPARSE_REL_TOL:.0e}"
    );
    println!();
    println!("PLY    : {}", ply_path.display());
    println!(
        "         finest deformed boundary mesh (n_vertices = {}, n_faces = {} via \
         Mesh::boundary_faces);",
        snapshot.h4.n_vertices,
        snapshot.h4.boundary_faces.len(),
    );
    println!("         per-vertex contact_force_z extra (zero for inactive, κ·(d̂-sd) for active);");
    println!("         open in cf-view, the workspace's unified visual-review viewer:");
    println!("           cargo run -p cf-viewer --release -- <path>");
    println!();
    println!("JSON   : {}", json_path.display());
    println!(
        "         scalars + per-active-vertex (r, sd, force_z) at finest + 200-pt Hertz analytic;"
    );
    println!("         render the matplotlib plot via:");
    println!("           uv run examples/sim-soft/hertz-sphere-plane/plot.py");
    println!();
    println!("Bevy visualization (CF_VISUAL=1):");
    println!("           CF_VISUAL=1 cargo run -p example-sim-soft-hertz-sphere-plane --release");
    println!("         spawns an OrbitCamera scene with the finest deformed sphere + coral a_FEM");
    println!(
        "         annulus + white a_Hertz annulus on the plane. Static (single-step \
         quasi-static)"
    );
    println!(
        "         — patch-radius gap reads visually 1:1 with the asserted rel_err_a. Mouse drag"
    );
    println!("         to orbit, scroll to zoom, right-drag to pan, close window to exit.");
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    verify_geometry_invariants();

    let snapshot = run_hertz_sweep();

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
    verify_monotonic_a_convergence(&snapshot);
    verify_cauchy_a_convergence(&snapshot);
    verify_finest_level_hertz_match(&snapshot);
    verify_captured_bits_hertz_metrics(&snapshot);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let ply_path = out_dir.join("hertz_sphere_plane.ply");
    let json_path = out_dir.join("hertz_sphere_plane.json");
    save_finest_frame_ply(&snapshot, &ply_path)?;
    save_hertz_json(&snapshot, &json_path)?;

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
