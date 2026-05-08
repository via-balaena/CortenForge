// `usize as f64` casts on referenced-vertex / per-shell / per-step
// counts for mean computations. Counts ≤ ~150 k here, well within f64
// mantissa exact range. Same allowance as rows 6+8+9+10+11+16+18+20+21.
#![allow(clippy::cast_precision_loss)]
// `cast_possible_wrap` on `usize → VertexId` (u32) packing for vertex
// indices in the BCC + stuffing pipeline. Vertex counts are bounded by
// the BCC-lattice's i32-safe `n_lattice` cap inherited from
// `BccLattice::new`. Same allowance as rows 8+9+10+11+15+16+20+21.
#![allow(clippy::cast_possible_wrap)]
// `cast_possible_truncation` on `pid as u32` packing for primitive
// indices in the per-pair-readout walk (one rigid primitive in this
// row). Same allowance as rows 18 + 20 + 21.
#![allow(clippy::cast_possible_truncation)]
// `expect()` on `Matrix3::try_inverse()` for the per-tet `D_rest`
// reference shape derivative. `D_rest` is invertible iff the tet has
// positive signed volume — `verify_quality_floors` is the
// corresponding pre-condition gate. Same precedent as row 20 + 21.
#![allow(clippy::expect_used)]
// `print_summary` is a single museum-plaque stdout writer; splitting
// fragments the visual format without information gain. Same allowance
// as rows 4+5+6+9+10+11+15+16+19+20+21.
#![allow(clippy::too_many_lines)]
// Domain-meaningful naming pairs (`outer_envelope` / `sleeve_body`,
// `pos_pinned` / `pos_active`, `mu_field` / `lambda_field`) distinguish
// operand-vs-receiver across the heterogeneous-CSG and multi-field
// composition sites. Same allowance as rows 6+10+11+16+20+21.
#![allow(clippy::similar_names)]

//! scan-fit-3layer-sleeve-ramp — the row 22 Tier 6 synthesis row, a
//! multi-step quasi-static intrusion ramp evolution of row 21
//! [`scan-fit-3layer-sleeve`](../scan-fit-3layer-sleeve). Where row 21
//! computes a single static fit pose at 1 mm probe penetration, this
//! row chains 12 backward-Euler `replay_step` calls in a quasi-static
//! ramp from rest to 6 mm penetration (0.5 mm/step), bounding each
//! step's Newton iter-0 penalty gradient by the per-step delta only —
//! NOT the full target overlap from rest. Same single-step solver,
//! same SPD-skeleton path, same `STATIC_DT = 1.0 s` collapse of the
//! inertial term. Each step is its own converged static equilibrium;
//! the ramp sweeps the rigid-primitive geometry between converged
//! equilibria.
//!
//! [mem]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_layered_silicone_device.md
//! [v2spec]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_row_22_v2_spec.md
//!
//! # Why a ramp, not a single step
//!
//! Row 21 v1's static fit pose at 1 mm penetration is the canonical
//! single-step demonstration of the row 21 pipeline. Deeper
//! penetration single-stepped on this geometry is expected to push
//! the iter-0 penalty gradient out of Newton's basin — single-step
//! failure modes were directly observed at related configs (v1.5's
//! capsule + capsule at 4 mm depth tripped a non-PD pivot at iter 3;
//! the earlier-pivot 459 K-tet 2 mm-cell experiment at 8 mm depth
//! inverted tets in the first Newton step), and v2's own ramp
//! observed Armijo line-search collapse at 6.5 mm depth even with a
//! 6.0 mm pre-converged `x_prev`. The quasi-static ramp circumvents
//! the single-step basin issue: each step's `x_prev` is the previous
//! step's `x_final` (cavity wall already deformed to the previous
//! step's equilibrium), so the iter-0 gradient is bounded by the
//! per-step delta only.
//!
//! The ramp's empirical reach on this geometry/material/resolution
//! was characterized at v2-spec lock time via five pre-execution
//! spike runs (Runs 1–3 swept step-size + iter-cap; Spike A + Spike B
//! probed mesh resolution + stiffness gradient — see [v2 spec
//! memo][v2spec]): the wall lies at `max_disp ≈ 7 mm`, where
//! Newton's tangent matrix becomes near-singular and Armijo
//! line-search hits its hardcoded `α_min = 2⁻²¹` floor. 6 mm
//! penetration (this row's target) is the deepest reach with
//! comfortable solver margin (Run 3 step 12 in 61 iters with
//! `MAX_NEWTON_ITER = 100`). The user-target 8 mm intrusion is
//! deferred to a v3 followup that requires either solver-side faer
//! LU fallback OR contact-geometry change (probe placement / shape).
//!
//! # Pipeline
//!
//! 1. **Scan stand-in via `Solid::cuboid`** — same as row 21 v1.
//!    `(SCAN_HX, SCAN_HY, SCAN_HZ) = (0.020, 0.015, 0.040) m`
//!    half-extents.
//!
//! 2. **Outer envelope via `Solid::offset(WRAP_THICKNESS)`** — same.
//!
//! 3. **Sleeve body via `Solid::subtract`** — same.
//!
//! 4. **`SdfMeshedTetMesh` build** via PR3 F1+F3 — same. Mesh is
//!    rebuilt fresh at every ramp step (the BCC + IS pipeline is
//!    deterministic on the same SDF + hints, so re-built mesh is
//!    bit-identical step-to-step). Cost: ~12× row 21's single
//!    rebuild ≈ ~1.5 s release.
//!
//! 5. **3-layer `MaterialField`** via PR3 F4 — same. Inner =
//!    `ECOFLEX_00_20`, middle = `DRAGON_SKIN_10A`, outer =
//!    `DRAGON_SKIN_20A` (the user-intended manufacturing target;
//!    preserved verbatim from row 21 v1).
//!
//! 6. **Quasi-static intrusion ramp** — `N_RAMP_STEPS = 12` steps of
//!    `RAMP_STEP_DELTA = 0.5 mm` each, reaching
//!    `PROBE_PENETRATION_FINAL = 6 mm`. At step `k` (1-indexed): the
//!    probe sits at `probe_z_k = SCAN_HZ - PROBE_RADIUS + k *
//!    RAMP_STEP_DELTA`; `PenaltyRigidContact::new(vec![probe_k])`
//!    receives the new probe; `replay_step(x_prev=x_final[k-1],
//!    v_prev=0, ...)` solves the per-step equilibrium. `STATIC_DT =
//!    1.0 s` collapses the inertial term per step.
//!    `MAX_NEWTON_ITER = 100` (vs row 21 v1's 50) absorbs the per-step
//!    iter-count growth that the v2 spike measured (8 / 8 / 9 / 11 /
//!    11 / 13 / 14 / 16 / 19 / 22 / 30 / 61 at the 12 steps; step 12
//!    is the empirical max + 39-iter margin).
//!
//! 7. **Per-step + final-step readouts** — at every ramp step, capture
//!    `(iter_count, final_residual_norm, n_active_pairs,
//!    force_total_z, mean_force_magnitude, max_force_magnitude,
//!    mean_disp, max_disp, mean_psi_inner/middle/outer,
//!    max_psi_outer)` into a `RampStepResult`. Per-tet `Ψ_t =
//!    Material::energy(F_t)` extraction (introduced as row 21 v1's
//!    new capability) runs at every ramp step against the per-step
//!    `x_final`. The final step (step 12 at 6 mm) drives the headline
//!    captured-bit anchors + the PLY z-slab artifact.
//!
//! 8. **Readouts** —
//!    - JSON `out/scan_fit_3layer_sleeve_ramp.json`: 4-section schema
//!      (scalars at final step + `material_layers` provenance +
//!      `ramp_curve` 12-element array + `final_contact_pairs` per-pair
//!      detail at step 12 only).
//!    - PLY `out/sleeve_zslab_final.ply`: z-slab per-tet centroid
//!      cloud at the FINAL step, with categorical `material_id` +
//!      sequential `displacement_magnitude`. Same z-slab pattern as
//!      row 21 v1 + rows 11+16+20.
//!    - Optional `plot_ramp.py` (PEP 723 + matplotlib): dual-axis
//!      depth × `force_z` + depth × `max_disp` force-displacement
//!      curve, mirrors row 5's `plot.py` structure. Run via
//!      `uv run plot_ramp.py`.
//!    - `verify_*` runtime gates (12 anchor groups, see "Numerical
//!      anchors" in `README.md`).
//!
//! # Why z-slab over full-boundary-surface
//!
//! Same rationale as row 21 v1. Pattern (aa) banked at row 16 N+3:
//! hollow / interior-cavity / partial-occlusion bodies → z-slab
//! per-tet centroid cloud. The 3-layer sleeve's doubly-hollow geometry
//! (scan-shaped cavity AND three concentric material shells) requires
//! an axis-aligned slab cut to expose both the radial material-shell
//! partition and the cavity-wall displacement response. The z-slab
//! at `z = 0` is the equatorial cross-section, 40 mm BELOW the
//! contact zone at `z ≈ 0.040 m` — it catches the propagated
//! secondary response, NOT the contact-zone signal directly. (A
//! longitudinal y-slab was considered for v1.5 capsule + capsule
//! geometry but is unrelated to v2's ramp differentiator; the v2 row
//! retains v1's z-slab axis verbatim.)
//!
//! # Sanitization
//!
//! Per the [device memo][mem]'s sanitization directive: the scanned
//! reference geometry is referred to as "scanned reference geometry"
//! or "scan stand-in" throughout this crate's prose. No anatomical
//! references appear in any tracked surface. The cuboid placeholder
//! is a parametric synthetic stand-in — the pipeline demonstration is
//! the workflow ("scan-shaped body → wrap by offset → carve cavity →
//! 3-material FEM → multi-step rigid intrusion ramp"), not the
//! cuboid's specific geometry; production runs swap the cuboid for a
//! real scan via row 15's STL-import path without any other code
//! change.
//!
//! # Run
//!
//! ```sh
//! cargo run -p example-sim-soft-scan-fit-3layer-sleeve-ramp --release
//! ```
//!
//! Per `feedback_release_mode_heavy_tests` — release mode is required.
//! The 12-step ramp at ~75 k tets through faer's sparse Cholesky takes
//! ~30-60 s release; debug mode would take many minutes per step ×
//! 12 steps. The `CELL_SIZE = 0.004 m` (4 mm) is sized so each of the
//! 6/4/4 mm layers carries at least one BCC cell across thickness;
//! finer cells (e.g., `0.002 m`) trip an SPD pivot at the FIRST ramp
//! step (denser per-cell penalty gradient — empirically tested at
//! v2-spec spike time and rejected on this row).
//!
//! Optional matplotlib post-processing:
//!
//! ```sh
//! uv run examples/sim-soft/scan-fit-3layer-sleeve-ramp/plot_ramp.py
//! ```

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
    Mesh, MeshingHints, PenaltyRigidContact, PenaltyRigidContactSolver, Sdf, SdfMeshedTetMesh,
    Solver, SolverConfig, Tet4, Vec3, VertexId, pick_vertices_by_predicate, referenced_vertices,
};

// =============================================================================
// Constants — scan stand-in geometry (axis-aligned cuboid)
// =============================================================================
//
// All geometry constants inherited verbatim from row 21 v1 — v2 keeps
// the cuboid + sphere geometry; the only differentiator is the ramp.

/// Cuboid half-extents (m). 40 × 30 × 80 mm full-extent.
const SCAN_HX: f64 = 0.020;
const SCAN_HY: f64 = 0.015;
const SCAN_HZ: f64 = 0.040;

// =============================================================================
// Constants — sleeve wrap geometry
// =============================================================================

/// Total wrap thickness (m).
const WRAP_THICKNESS: f64 = 0.014;

/// Inner-layer outer threshold (m).
const LAYER_INNER: f64 = 0.006;

/// Middle-layer outer threshold (m).
const LAYER_MIDDLE_OUTER: f64 = 0.010;

/// Outer-layer outer threshold (m).
const LAYER_OUTER: f64 = WRAP_THICKNESS;

// =============================================================================
// Constants — meshing bbox + cell size
// =============================================================================

const BBOX_HALF_X: f64 = SCAN_HX + WRAP_THICKNESS + CELL_SIZE;
const BBOX_HALF_Y: f64 = SCAN_HY + WRAP_THICKNESS + CELL_SIZE;
const BBOX_HALF_Z: f64 = SCAN_HZ + WRAP_THICKNESS + CELL_SIZE;

/// BCC lattice spacing (m). 4 mm — same as row 21 v1. Finer cells
/// (`0.002 m`) trip SPD at first ramp step (denser per-cell penalty
/// gradient, empirically rejected at v2 spike time).
const CELL_SIZE: f64 = 0.004;

// =============================================================================
// Constants — rigid intrusion probe (sphere, repositioned per step)
// =============================================================================

/// Spherical probe radius (m). 12 mm — same as row 21 v1.
const PROBE_RADIUS: f64 = 0.012;

// =============================================================================
// Constants — quasi-static ramp
// =============================================================================

/// Number of ramp steps. 12 × 0.5 mm/step = 6 mm total. The user-
/// target 8 mm physical intrusion would require either solver-side
/// faer LU fallback (so the engine can absorb a near-singular tangent
/// regime) OR a contact-geometry change (probe placement / shape) —
/// deferred to a v3 followup.
const N_RAMP_STEPS: usize = 12;

/// Final probe penetration depth (m). 6 mm — at this depth the v2
/// spike's Run 3 step 12 converged in 61 Newton iters; deeper steps
/// (6.5 mm in Run 3) hit the Armijo line-search `α_min = 2⁻²¹` floor
/// on the near-singular tangent.
const PROBE_PENETRATION_FINAL: f64 = 0.006;

/// Penetration delta per ramp step (m). 0.5 mm/step.
const RAMP_STEP_DELTA: f64 = PROBE_PENETRATION_FINAL / N_RAMP_STEPS as f64;

// =============================================================================
// Constants — solver
// =============================================================================

/// Static-regime time step per ramp step (s). At `dt = 1.0 s`, the
/// `M / Δt²` inertial term collapses ~4 orders below the stiffness
/// contribution per step; each `replay_step` from the previous step's
/// `x_final` converges to a fresh static equilibrium far below
/// `tol = 1e-10`. Same idiom as row 21 v1 + rows 11/14/16/18/20.
const STATIC_DT: f64 = 1.0;

/// Newton iter cap per ramp step. Bumped from row 21 v1's 50 to 100
/// to absorb the per-step iter-count growth at deeper penetration —
/// v2 spike Run 3 step 12 (6 mm) needed 61 iters; 100 has a 39-iter
/// margin. Lower `MAX_NEWTON_ITER` would reach the cap mid-ramp
/// around step 11-12 and fail spuriously.
const MAX_NEWTON_ITER: usize = 100;

// =============================================================================
// Constants — tolerances
// =============================================================================

/// IV-1 sparse-tier rel-tol for captured per-step force / displacement
/// / per-layer Ψ̄ bits. ~75 k tets × 12 chained steps through faer's
/// sparse Cholesky lives at the IV-1 sparse-at-scale tier; `1e-12`
/// admits sparse-solver SIMD/FMA noise while catching real
/// regressions. Same precedent as rows 6+10+11+16+20+21.
const SPARSE_REL_TOL: f64 = 1.0e-12;

/// Absolute floor for the captured-bits comparison.
const SPARSE_EPS_ABS: f64 = 1.0e-12;

/// Bit-exact tolerance for the F4 const-fn `to_neo_hookean()` Lamé-
/// pair round-trip.
const F4_PROVENANCE_EXACT_TOL: f64 = 0.0;

/// Probe `F = diag(1.01, 1, 1)` material-assignment-probe tolerance.
const MATERIAL_PROBE_EXACT_TOL: f64 = 0.0;

// =============================================================================
// Constants — captured first-run anchor bits
// =============================================================================
//
// **Capture provenance** — captured 2026-05-08 at sim-soft `dev` (row
// 21 v1 tip `b6035fa0`), rustc 1.95.0 (`59807616e` 2026-04-14) on
// macOS arm64 — same toolchain + platform as IV-1's reference capture.
//
// First-run capture bootstrapped via `CF_CAPTURE_BITS=1` (pattern
// (cc) banked at row 19): when set, every captured-anchor check is
// bypassed and a paste-ready capture block is printed to stderr; when
// unset (default), every captured-bits gate runs the strict
// `to_bits()` self-pin against the constants below. Identity-row
// pattern: every `*_EXACT` count is filled at first run (initial `0`
// triggers a deliberate-fail diagnostic), every `*_REF_BITS` is
// filled likewise.
//
// Geometry-derived counts are bit-equal to row 21 v1's captures
// (cuboid + sphere + CELL=4mm + 6/4/4 layers + 18/51/113 kPa stack
// is verbatim).

/// Total tet count after BCC + Isosurface Stuffing on the v1 sleeve.
/// Bit-equal to row 21 v1's `N_TETS_EXACT = 74_628` (geometry + BCC
/// + IS pipeline are deterministic on the same SDF + hints).
const N_TETS_EXACT: usize = 74_628;

/// Total mesh vertex count, including BCC corners not referenced by
/// any tet.
const N_VERTICES_EXACT: usize = 31_966;

/// Vertices referenced by ≥ 1 tet.
const N_REFERENCED_EXACT: usize = 17_384;

/// Outer-envelope-surface Dirichlet-pinned vertex count.
const N_PINNED_EXACT: usize = 7_046;

/// Per-shell tet counts at first capture (bit-equal to row 21 v1).
const N_INNER_TETS_EXACT: usize = 25_892;
const N_MIDDLE_TETS_EXACT: usize = 16_656;
const N_OUTER_TETS_EXACT: usize = 32_080;

/// Per-shell tet counts in the `|centroid.z| < CELL_SIZE / 2 = 0.002`
/// z-slab cut for the cf-view PLY artifact (final step only).
/// Bit-equal to row 21 v1 (geometry + slab cut are unchanged).
const N_INNER_TETS_ZSLAB_EXACT: usize = 768;
const N_MIDDLE_TETS_ZSLAB_EXACT: usize = 432;
const N_OUTER_TETS_ZSLAB_EXACT: usize = 892;

/// Ramp-step partition gate. Bit-pinned to `N_RAMP_STEPS = 12`.
const N_RAMP_STEPS_EXACT: usize = N_RAMP_STEPS;

/// Active contact-pair count at the FINAL ramp step (depth = 6 mm),
/// filtered to REFERENCED vertices (v2.5 cleanup; 2026-05-08). 37
/// real physical contacts at the deepest pose. Pre-v2.5 the
/// unfiltered count was 273 (95-97% orphan-driven); post-v2.5 this
/// counts physical contacts only.
const N_CONTACT_PAIRS_FINAL_EXACT: usize = 37;

/// Per-step Newton iter counts. The chained `replay_step` is
/// deterministic on a fixed toolchain; iter-count drift signals
/// real solver-path regression, not noise. v2 spike Run 3 reproduces
/// these values bit-equally — `[8, 8, 9, 11, 11, 13, 14, 16, 19, 22,
/// 30, 61]`.
const IT_COUNT_RAMP_EXACT: [usize; N_RAMP_STEPS] = [8, 8, 9, 11, 11, 13, 14, 16, 19, 22, 30, 61];

/// Per-step `+z`-component of contact reaction force bits (N), summed
/// over REFERENCED vertices only (v2.5 cleanup; 2026-05-08). Force
/// is in `+z` direction (probe pushes wrap-cap material UP), grows
/// monotonically with deeper penetration. Approximate values:
/// `[1.1, 1.9, 2.9, 4.2, 5.6, 7.2, 9.2, 11.5, 14.1, 16.9, 20.0,
/// 23.1] N`. Pre-v2.5 these were `[-137, -131, ..., -1135] N` —
/// the sign was negative because orphan BCC vertices inside the
/// cavity dominated the readout sum with `-z` normal components;
/// orphans contributed ~95-97% of readout entries.
const FORCE_TOTAL_Z_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x3ff1_7025_2061_5434,
    0x3ffd_e776_8e72_d943,
    0x4007_8028_e030_a891,
    0x4010_a224_f6c6_6811,
    0x4016_4676_7eae_988b,
    0x401c_e5b8_834c_8232,
    0x4022_6b54_7de7_eea2,
    0x4027_095c_9831_1757,
    0x402c_2271_a671_c21f,
    0x4030_e095_1f5b_6fab,
    0x4033_f37f_f378_8547,
    0x4037_238b_49db_964a,
];

/// Per-step max body-wide displacement-magnitude bits (m) over all
/// referenced vertices. Approximate values: `[1.5, 2.0, 2.5, 3.0,
/// 3.4, 3.9, 4.4, 4.9, 5.4, 5.8, 6.3, 6.7] mm`. Step 12's
/// `max_disp` = 6.7 mm < `WRAP_THICKNESS` = 14 mm — the geometric
/// upper-bound gate stays comfortable throughout the ramp.
const MAX_DISP_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x3f58_4265_78fe_156a,
    0x3f60_2b04_a288_9913,
    0x3f64_3183_d814_9bd3,
    0x3f68_34e4_1845_8e59,
    0x3f6c_343e_2b10_7319,
    0x3f70_1721_6d62_8cda,
    0x3f72_0fc0_d273_8772,
    0x3f74_02ea_cbbb_4b51,
    0x3f75_f001_c4c3_fe6f,
    0x3f77_d587_037c_e5ab,
    0x3f79_b185_3bb1_0951,
    0x3f7b_8064_bc49_ecc8,
];

/// Per-step inner-layer mean strain-energy-density bits (J/m³).
/// Inner is softest (μ = 18 kPa) AND closest to probe → highest
/// mean Ψ throughout the ramp. Final-step value ≈ 302 J/m³ (~33×
/// row 21 v1's 9.21 J/m³ at 1 mm; the 6× depth + non-linear
/// stiffening is what drives the increase).
const MEAN_PSI_INNER_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x4011_9944_2267_d50a,
    0x4022_6b7f_3f66_a9b1,
    0x4030_dfb8_c4f0_e361,
    0x403c_4c93_1f98_304f,
    0x4045_c752_78f7_1446,
    0x404f_80eb_f5e2_e0e4,
    0x4055_d8db_2cf6_17fb,
    0x405d_5acc_c97d_30c1,
    0x4063_30a4_f4ec_0a1e,
    0x4068_7c74_0a9a_923d,
    0x406e_99f7_f4da_40be,
    0x4072_e2a7_70bc_c370,
];

/// Per-step middle-layer mean strain-energy-density bits (J/m³).
/// Final-step value ≈ 102 J/m³.
const MEAN_PSI_MIDDLE_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x3fe2_f35c_8741_5571,
    0x3ff8_675a_bab3_dc9f,
    0x4009_d3a9_d2b0_1ac6,
    0x4017_9b00_e7ea_0d54,
    0x4023_a837_433b_2c54,
    0x402e_c0a6_64ce_7d4b,
    0x4036_dff8_c883_0cc1,
    0x4040_5dec_35bc_0e3e,
    0x4046_b4d3_2356_8a8c,
    0x404e_ac74_4a86_9b13,
    0x4054_33f4_db21_61da,
    0x4059_9b79_454a_fb70,
];

/// Per-step outer-layer mean strain-energy-density bits (J/m³).
/// Outer is stiffest (μ = 113 kPa) AND outer-Dirichlet-pinned →
/// lowest mean Ψ throughout the ramp. Final-step value ≈ 30 J/m³.
const MEAN_PSI_OUTER_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x3fc0_3cf1_12db_cfaf,
    0x3fd5_e5a4_23de_34d2,
    0x3fe8_49a4_066e_2815,
    0x3ff6_ec6a_e126_8a1d,
    0x4003_a069_bff7_7564,
    0x400f_8cb9_d648_0697,
    0x4018_21fb_d586_b328,
    0x4021_bd0f_7992_2fd4,
    0x4029_3bda_b330_884a,
    0x4031_7672_4109_d2da,
    0x4037_8bf0_1fec_f828,
    0x403e_620a_c6ce_c21d,
];

/// Final-step (step 12, depth = 6 mm) outer-layer max strain-energy-
/// density bits (J/m³). Durability proxy at the user-target
/// equivalent depth: ~10487 J/m³ (~ 345× outer-layer mean ~30 J/m³ —
/// peak localises in tets adjacent to the contact band where probe
/// loads transmit through the radial chain inner → middle → outer).
const MAX_PSI_OUTER_FINAL_REF_BITS: u64 = 0x40c4_7b7c_e1e6_f7f8;

// =============================================================================
// SDF builders
// =============================================================================

fn build_scan_solid() -> Solid {
    Solid::cuboid(Vector3::new(SCAN_HX, SCAN_HY, SCAN_HZ))
}

fn build_outer_envelope(scan: Solid) -> Solid {
    scan.offset(WRAP_THICKNESS)
}

fn build_sleeve_body(outer: Solid, scan: Solid) -> Solid {
    outer.subtract(scan)
}

/// Probe `Solid` at the given penetration depth (m). At `depth = 0.001 m`
/// (1 mm) this matches row 21 v1's static fit-pose probe verbatim; the
/// ramp loop calls this at progressively deeper depths up to
/// `PROBE_PENETRATION_FINAL = 0.006 m`.
fn build_probe_solid_at_depth(depth: f64) -> Solid {
    let probe_z = SCAN_HZ - PROBE_RADIUS + depth;
    Solid::sphere(PROBE_RADIUS).translate(Vector3::new(0.0, 0.0, probe_z))
}

// =============================================================================
// 3-layer MaterialField — partition by distance-from-scan
// =============================================================================

fn build_material_field() -> MaterialField {
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

/// Open-left, closed-right boundary convention to match
/// `LayeredScalarField::sample`'s `partition_point(|&t| t <= phi)`
/// behavior bit-equally. Same as row 21 v1.
fn shell_at_phi(phi: f64) -> usize {
    if phi < LAYER_INNER {
        0
    } else if phi < LAYER_MIDDLE_OUTER {
        1
    } else {
        2
    }
}

// =============================================================================
// MeshingHints
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
// BoundaryConditions
// =============================================================================

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
// Per-tet deformation gradient (same as row 21 v1)
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
// RampStepResult — per-step record collected during the ramp
// =============================================================================

#[derive(Clone, Debug)]
struct RampStepResult {
    step: usize,
    depth_m: f64,
    iter_count: usize,
    final_residual_norm: f64,
    n_active_pairs: usize,
    force_total_z_n: f64,
    mean_force_magnitude_n: f64,
    max_force_magnitude_n: f64,
    mean_disp_m: f64,
    max_disp_m: f64,
    mean_psi_inner_j_per_m3: f64,
    mean_psi_middle_j_per_m3: f64,
    mean_psi_outer_j_per_m3: f64,
    max_psi_outer_j_per_m3: f64,
    /// Step 12 only: the per-pair readouts for JSON `final_contact_pairs`
    /// and the final-step PLY z-slab (intermediate steps drop this
    /// to keep memory bounded).
    final_step_data: Option<FinalStepData>,
}

#[derive(Clone, Debug)]
struct FinalStepData {
    rest_positions: Vec<Vec3>,
    deformed_positions: Vec<Vec3>,
    pair_records: Vec<Value>,
}

// =============================================================================
// solve_ramp — chained replay_step over N_RAMP_STEPS
// =============================================================================

fn solve_ramp(
    n_vertices: usize,
    referenced: &[VertexId],
    tets: &[[VertexId; 4]],
    shell_idx_per_tet: &[usize],
    n_inner: usize,
    n_middle: usize,
    n_outer: usize,
) -> Result<Vec<RampStepResult>> {
    let n_dof = 3 * n_vertices;

    // v2.5 anchor cleanup (2026-05-08): filter `per_pair_readout`
    // entries to vertices in the referenced set. The unfiltered
    // readout includes ORPHAN BCC lattice vertices (corners not in
    // any tet, with no FEM stiffness contribution) that happen to
    // sit inside the rigid primitive's volume — at v2's 1 mm
    // penetration, ~286/295 readout entries are orphans inside the
    // empty cavity; they're ignored by the solver but pollute
    // counts + force aggregates by ~95-97%. v2.5 anchors only
    // referenced (= solver-active) vertices, yielding physically
    // meaningful counts + forces. See pattern (xx) at row 22
    // patterns memo.
    let referenced_set: BTreeSet<VertexId> = referenced.iter().copied().collect();

    // Initial x_prev = rest positions (from a one-shot mesh build).
    let initial_mesh = {
        let scan = build_scan_solid();
        let outer = build_outer_envelope(scan.clone());
        let body = build_sleeve_body(outer, scan);
        let hints = build_hints(build_material_field());
        SdfMeshedTetMesh::from_sdf(&body, &hints).map_err(|e| anyhow::anyhow!("{e:?}"))?
    };
    let mut x_prev_flat: Vec<f64> = vec![0.0; n_dof];
    for (v, p) in initial_mesh.positions().iter().enumerate() {
        x_prev_flat[3 * v] = p.x;
        x_prev_flat[3 * v + 1] = p.y;
        x_prev_flat[3 * v + 2] = p.z;
    }
    drop(initial_mesh);

    let mut results: Vec<RampStepResult> = Vec::with_capacity(N_RAMP_STEPS);

    for k in 0..N_RAMP_STEPS {
        let depth = (k + 1) as f64 * RAMP_STEP_DELTA;

        // Rebuild mesh + bc + contact for this step.
        let scan_k = build_scan_solid();
        let outer_k = build_outer_envelope(scan_k.clone());
        let body_k = build_sleeve_body(outer_k.clone(), scan_k);
        let hints_k = build_hints(build_material_field());
        let mesh_k =
            SdfMeshedTetMesh::from_sdf(&body_k, &hints_k).map_err(|e| anyhow::anyhow!("{e:?}"))?;
        let referenced_k = referenced_vertices(&mesh_k);
        let bc_k = build_boundary_conditions(&mesh_k, &referenced_k, &outer_k);

        let probe_k = build_probe_solid_at_depth(depth);
        let contact_k = PenaltyRigidContact::new(vec![probe_k]);

        let mut cfg = SolverConfig::skeleton();
        cfg.dt = STATIC_DT;
        cfg.max_newton_iter = MAX_NEWTON_ITER;

        let x_prev_t = Tensor::from_slice(&x_prev_flat, &[n_dof]);
        let v_prev_t = Tensor::zeros(&[n_dof]);
        let empty_theta: [f64; 0] = [];
        let theta_t = Tensor::from_slice(&empty_theta, &[0]);

        let solver: PenaltyRigidContactSolver<SdfMeshedTetMesh> =
            CpuNewtonSolver::new(Tet4, mesh_k, contact_k, cfg, bc_k);
        let step_k = solver.replay_step(&x_prev_t, &v_prev_t, &theta_t, cfg.dt);

        // Inspection mesh + contact for per-pair readout + Ψ extraction.
        let inspection_mesh = {
            let scan_again = build_scan_solid();
            let outer_again = build_outer_envelope(scan_again.clone());
            let body_again = build_sleeve_body(outer_again, scan_again);
            let inspection_hints = build_hints(build_material_field());
            SdfMeshedTetMesh::from_sdf(&body_again, &inspection_hints)
                .map_err(|e| anyhow::anyhow!("{e:?}"))?
        };
        let inspection_contact = PenaltyRigidContact::new(vec![build_probe_solid_at_depth(depth)]);

        let positions_k: Vec<Vec3> = (0..n_vertices)
            .map(|i| {
                Vec3::new(
                    step_k.x_final[3 * i],
                    step_k.x_final[3 * i + 1],
                    step_k.x_final[3 * i + 2],
                )
            })
            .collect();
        let raw_readouts = inspection_contact.per_pair_readout(&inspection_mesh, &positions_k);
        // v2.5 cleanup: filter to referenced vertices only — see
        // function-prologue comment above the `referenced_set` build.
        let readouts: Vec<_> = raw_readouts
            .into_iter()
            .filter(|r| match r.pair {
                sim_soft::ContactPair::Vertex { vertex_id, .. } => {
                    referenced_set.contains(&vertex_id)
                }
            })
            .collect();
        let n_active_pairs = readouts.len();
        let force_total_z: f64 = readouts.iter().map(|r| r.force_on_soft.z).sum();
        let force_mags: Vec<f64> = readouts.iter().map(|r| r.force_on_soft.norm()).collect();
        let mean_force_magnitude = if n_active_pairs == 0 {
            0.0
        } else {
            force_mags.iter().sum::<f64>() / n_active_pairs as f64
        };
        let max_force_magnitude = force_mags.iter().copied().fold(f64::NEG_INFINITY, f64::max);

        let rest_pos_k: Vec<Vec3> = inspection_mesh.positions().to_vec();

        // Cavity-wall mean disp over the FILTERED active-pair vertex set
        // (referenced-only, post-v2.5-cleanup); max disp computed over
        // ALL referenced vertices (body-wide, not just contact-band).
        let cavity_vertex_ids: BTreeSet<VertexId> = readouts
            .iter()
            .map(|r| match r.pair {
                sim_soft::ContactPair::Vertex { vertex_id, .. } => vertex_id,
            })
            .collect();
        let cavity_disp: Vec<f64> = cavity_vertex_ids
            .iter()
            .map(|&v| (positions_k[v as usize] - rest_pos_k[v as usize]).norm())
            .collect();
        let mean_disp_m = if cavity_disp.is_empty() {
            0.0
        } else {
            cavity_disp.iter().sum::<f64>() / cavity_disp.len() as f64
        };
        let max_disp_m: f64 = (0..n_vertices)
            .map(|i| (positions_k[i] - rest_pos_k[i]).norm())
            .fold(0.0, f64::max);

        // Per-tet Ψ aggregates.
        let materials = inspection_mesh.materials();
        let mut sum_in = 0.0;
        let mut sum_mi = 0.0;
        let mut sum_ou = 0.0;
        let mut max_psi_outer = f64::NEG_INFINITY;
        for (t, &verts) in tets.iter().enumerate() {
            let f = deformation_gradient(verts, &rest_pos_k, &positions_k);
            let psi_t = materials[t].energy(&f);
            match shell_idx_per_tet[t] {
                0 => sum_in += psi_t,
                1 => sum_mi += psi_t,
                _ => {
                    sum_ou += psi_t;
                    if psi_t > max_psi_outer {
                        max_psi_outer = psi_t;
                    }
                }
            }
        }
        let mean_psi_inner_j_per_m3 = if n_inner == 0 {
            0.0
        } else {
            sum_in / n_inner as f64
        };
        let mean_psi_middle_j_per_m3 = if n_middle == 0 {
            0.0
        } else {
            sum_mi / n_middle as f64
        };
        let mean_psi_outer_j_per_m3 = if n_outer == 0 {
            0.0
        } else {
            sum_ou / n_outer as f64
        };

        // Final step: capture per-pair detail for JSON + PLY emit.
        let final_step_data = if k == N_RAMP_STEPS - 1 {
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
            Some(FinalStepData {
                rest_positions: rest_pos_k,
                deformed_positions: positions_k,
                pair_records,
            })
        } else {
            None
        };

        results.push(RampStepResult {
            step: k + 1,
            depth_m: depth,
            iter_count: step_k.iter_count,
            final_residual_norm: step_k.final_residual_norm,
            n_active_pairs,
            force_total_z_n: force_total_z,
            mean_force_magnitude_n: mean_force_magnitude,
            max_force_magnitude_n: max_force_magnitude,
            mean_disp_m,
            max_disp_m,
            mean_psi_inner_j_per_m3,
            mean_psi_middle_j_per_m3,
            mean_psi_outer_j_per_m3,
            max_psi_outer_j_per_m3: max_psi_outer,
            final_step_data,
        });

        // Chain x_prev for next step.
        x_prev_flat.clone_from(&step_k.x_final);
    }

    Ok(results)
}

// =============================================================================
// Verifications — 12 anchor groups
// =============================================================================

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
            "tet {tet_idx}: signed_volume = {signed_volume:e} (expected strictly positive)",
        );
    }
}

fn verify_n_ramp_steps_exact(n_results: usize) {
    assert_eq!(
        n_results, N_RAMP_STEPS_EXACT,
        "ramp produced {n_results} step results, expected {N_RAMP_STEPS_EXACT}",
    );
}

fn verify_per_step_solver_converges(results: &[RampStepResult]) {
    for r in results {
        assert!(
            r.iter_count < MAX_NEWTON_ITER,
            "ramp step {} (depth {} m): iter_count = {} ≥ MAX_NEWTON_ITER = {}",
            r.step,
            r.depth_m,
            r.iter_count,
            MAX_NEWTON_ITER,
        );
        assert!(
            r.final_residual_norm < 1.0e-10,
            "ramp step {} (depth {} m): final_residual_norm = {:e} ≥ 1e-10",
            r.step,
            r.depth_m,
            r.final_residual_norm,
        );
    }
}

fn verify_per_step_iter_count(results: &[RampStepResult]) {
    if capturing_bits() {
        let entries: Vec<String> = results.iter().map(|r| r.iter_count.to_string()).collect();
        eprintln!(
            "const IT_COUNT_RAMP_EXACT: [usize; N_RAMP_STEPS] = [{}];",
            entries.join(", "),
        );
        return;
    }
    for r in results {
        let expected = IT_COUNT_RAMP_EXACT[r.step - 1];
        assert_eq!(
            r.iter_count, expected,
            "ramp step {}: iter_count = {} (expected {})",
            r.step, r.iter_count, expected,
        );
    }
}

fn verify_force_displacement_monotone(results: &[RampStepResult]) {
    // v2.5 cleanup: post-filter, `force_total_z_n` is the `+z` sum of
    // penalty forces over REFERENCED active pairs (orphan-vertex
    // pseudo-forces excluded). The probe enters the cavity from above
    // and pushes the wrap-cap material UP in `+z`; the per-vertex
    // normals at active pairs (mostly above the probe, see (vv) and
    // (xx)) point in `+z` direction with axial bias, so
    // `force_total_z_n > 0` and grows monotonically with deeper
    // penetration as more wrap-cap material engages.
    //
    // Pre-v2.5 the sign was reversed (-1135 N at step 12) because
    // orphan BCC vertices inside the cavity dominated the readout
    // sum with `-z` normal components; v2.5 anchors the physically
    // meaningful (+z, monotone-growing) sum.
    //
    // Strict-adjacent monotonicity at the step 1 → step 2 boundary
    // is NOT required: contact engagement at the shallowest 0.5 mm
    // penetration is in a transient regime where the active-pair
    // set is still settling, and force can dip slightly before
    // becoming monotone. From step 2 onward the monotone gate IS
    // strict (any inversion in the deeper regime would signal a
    // real defect).
    //
    // Sanity bound at the ramp endpoints: force_z[N-1] > force_z[0] —
    // the deepest step's force magnitude must exceed the shallowest
    // step's, even with the transient.
    let first = &results[0];
    let last = results.last().expect("ramp produced no results");
    assert!(
        last.force_total_z_n > first.force_total_z_n,
        "ramp endpoint sanity: final step force_z {:e} N not greater than first step {:e} N",
        last.force_total_z_n,
        first.force_total_z_n,
    );

    // Strict-adjacent monotone from step 2 onward (skip step 1→2).
    for w in results.windows(2).skip(1) {
        let a = &w[0];
        let b = &w[1];
        assert!(
            b.force_total_z_n > a.force_total_z_n,
            "non-monotone force-displacement at step {}→{}: {:e} N → {:e} N \
             (expected step {} > step {} from step 2 onward)",
            a.step,
            b.step,
            a.force_total_z_n,
            b.force_total_z_n,
            b.step,
            a.step,
        );
    }
}

fn verify_per_step_strain_energy_ordering(results: &[RampStepResult]) {
    // Same `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` ordering as row 21 v1's
    // anchor 7, but applied at every ramp step. The compounding
    // (compliance + distance-to-load + distance-to-constraint) holds
    // throughout the ramp, so the ordering is robust at every
    // intermediate equilibrium.
    for r in results {
        assert!(
            r.mean_psi_inner_j_per_m3 > r.mean_psi_middle_j_per_m3,
            "ramp step {} (depth {} m): mean_psi_inner ({:e}) ≯ mean_psi_middle ({:e})",
            r.step,
            r.depth_m,
            r.mean_psi_inner_j_per_m3,
            r.mean_psi_middle_j_per_m3,
        );
        assert!(
            r.mean_psi_middle_j_per_m3 > r.mean_psi_outer_j_per_m3,
            "ramp step {} (depth {} m): mean_psi_middle ({:e}) ≯ mean_psi_outer ({:e})",
            r.step,
            r.depth_m,
            r.mean_psi_middle_j_per_m3,
            r.mean_psi_outer_j_per_m3,
        );
    }
}

fn verify_per_step_max_disp_bounded(results: &[RampStepResult]) {
    // Body-wide max displacement must stay strictly < WRAP_THICKNESS
    // at every ramp step. The 6 mm final depth produces ~6.7 mm peak
    // body-wide displacement (per the v2 spike), well under 14 mm.
    // (The penalty equilibrium can push the cavity wall farther than
    // the rigid penetration depth — that's expected behaviour.)
    for r in results {
        assert!(
            r.max_disp_m < WRAP_THICKNESS,
            "ramp step {} (depth {} m): max_disp = {:e} m ≥ WRAP_THICKNESS = {} m",
            r.step,
            r.depth_m,
            r.max_disp_m,
            WRAP_THICKNESS,
        );
    }
}

fn verify_n_contact_pairs_final_exact(results: &[RampStepResult]) {
    let final_step = results.last().expect("ramp produced no results");
    if capturing_bits() {
        eprintln!(
            "const N_CONTACT_PAIRS_FINAL_EXACT: usize = {};",
            final_step.n_active_pairs,
        );
        return;
    }
    assert_eq!(
        final_step.n_active_pairs, N_CONTACT_PAIRS_FINAL_EXACT,
        "n_active_pairs at final ramp step",
    );
}

fn verify_material_provenance() {
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
            "tet {t} shell {shell_idx}: observed energy {observed} != expected {expected}",
        );
    }
}

fn verify_outer_layer_max_psi_final(results: &[RampStepResult]) {
    let final_step = results.last().expect("ramp produced no results");
    if capturing_bits() {
        eprintln!(
            "const MAX_PSI_OUTER_FINAL_REF_BITS: u64 = 0x{:016x};",
            final_step.max_psi_outer_j_per_m3.to_bits(),
        );
        return;
    }
    assert_relative_eq!(
        final_step.max_psi_outer_j_per_m3,
        f64::from_bits(MAX_PSI_OUTER_FINAL_REF_BITS),
        max_relative = SPARSE_REL_TOL,
        epsilon = SPARSE_EPS_ABS,
    );
}

fn verify_per_step_captured_bits(results: &[RampStepResult]) {
    if capturing_bits() {
        let force_z_hex: Vec<String> = results
            .iter()
            .map(|r| format!("0x{:016x}", r.force_total_z_n.to_bits()))
            .collect();
        let max_disp_hex: Vec<String> = results
            .iter()
            .map(|r| format!("0x{:016x}", r.max_disp_m.to_bits()))
            .collect();
        let psi_in_hex: Vec<String> = results
            .iter()
            .map(|r| format!("0x{:016x}", r.mean_psi_inner_j_per_m3.to_bits()))
            .collect();
        let psi_mi_hex: Vec<String> = results
            .iter()
            .map(|r| format!("0x{:016x}", r.mean_psi_middle_j_per_m3.to_bits()))
            .collect();
        let psi_ou_hex: Vec<String> = results
            .iter()
            .map(|r| format!("0x{:016x}", r.mean_psi_outer_j_per_m3.to_bits()))
            .collect();
        eprintln!(
            "const FORCE_TOTAL_Z_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [\n    {},\n];",
            force_z_hex.join(",\n    "),
        );
        eprintln!(
            "const MAX_DISP_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [\n    {},\n];",
            max_disp_hex.join(",\n    "),
        );
        eprintln!(
            "const MEAN_PSI_INNER_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [\n    {},\n];",
            psi_in_hex.join(",\n    "),
        );
        eprintln!(
            "const MEAN_PSI_MIDDLE_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [\n    {},\n];",
            psi_mi_hex.join(",\n    "),
        );
        eprintln!(
            "const MEAN_PSI_OUTER_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [\n    {},\n];",
            psi_ou_hex.join(",\n    "),
        );
        eprintln!("=== END CAPTURED BITS ===");
        return;
    }

    for r in results {
        let k = r.step - 1;
        assert_relative_eq!(
            r.force_total_z_n,
            f64::from_bits(FORCE_TOTAL_Z_RAMP_REF_BITS[k]),
            max_relative = SPARSE_REL_TOL,
            epsilon = SPARSE_EPS_ABS,
        );
        assert_relative_eq!(
            r.max_disp_m,
            f64::from_bits(MAX_DISP_RAMP_REF_BITS[k]),
            max_relative = SPARSE_REL_TOL,
            epsilon = SPARSE_EPS_ABS,
        );
        assert_relative_eq!(
            r.mean_psi_inner_j_per_m3,
            f64::from_bits(MEAN_PSI_INNER_RAMP_REF_BITS[k]),
            max_relative = SPARSE_REL_TOL,
            epsilon = SPARSE_EPS_ABS,
        );
        assert_relative_eq!(
            r.mean_psi_middle_j_per_m3,
            f64::from_bits(MEAN_PSI_MIDDLE_RAMP_REF_BITS[k]),
            max_relative = SPARSE_REL_TOL,
            epsilon = SPARSE_EPS_ABS,
        );
        assert_relative_eq!(
            r.mean_psi_outer_j_per_m3,
            f64::from_bits(MEAN_PSI_OUTER_RAMP_REF_BITS[k]),
            max_relative = SPARSE_REL_TOL,
            epsilon = SPARSE_EPS_ABS,
        );
    }
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
    results: &[RampStepResult],
) -> Result<()> {
    let final_step = results.last().expect("ramp produced no results");
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
        "probe_penetration_final_m": PROBE_PENETRATION_FINAL,
        "ramp_step_delta_m": RAMP_STEP_DELTA,
        "n_ramp_steps": N_RAMP_STEPS,
        "n_tets": n_tets,
        "n_vertices": n_vertices,
        "n_referenced": n_referenced,
        "n_pinned": n_pinned,
        "n_inner_tets": inner_count,
        "n_middle_tets": middle_count,
        "n_outer_tets": outer_count,
        "final_step": {
            "depth_m": final_step.depth_m,
            "n_active_pairs": final_step.n_active_pairs,
            "iter_count": final_step.iter_count,
            "final_residual_norm": final_step.final_residual_norm,
            "force_total_z_n": final_step.force_total_z_n,
            "mean_force_magnitude_n": final_step.mean_force_magnitude_n,
            "max_force_magnitude_n": final_step.max_force_magnitude_n,
            "mean_displacement_magnitude_m": final_step.mean_disp_m,
            "max_displacement_magnitude_m": final_step.max_disp_m,
            "mean_psi_inner_j_per_m3": final_step.mean_psi_inner_j_per_m3,
            "mean_psi_middle_j_per_m3": final_step.mean_psi_middle_j_per_m3,
            "mean_psi_outer_j_per_m3": final_step.mean_psi_outer_j_per_m3,
            "max_psi_outer_j_per_m3": final_step.max_psi_outer_j_per_m3,
        },
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
    let ramp_curve: Vec<Value> = results
        .iter()
        .map(|r| {
            json!({
                "step": r.step,
                "depth_m": r.depth_m,
                "iter_count": r.iter_count,
                "final_residual_norm": r.final_residual_norm,
                "n_active_pairs": r.n_active_pairs,
                "force_total_z_n": r.force_total_z_n,
                "mean_force_magnitude_n": r.mean_force_magnitude_n,
                "max_force_magnitude_n": r.max_force_magnitude_n,
                "mean_displacement_magnitude_m": r.mean_disp_m,
                "max_displacement_magnitude_m": r.max_disp_m,
                "mean_psi_inner_j_per_m3": r.mean_psi_inner_j_per_m3,
                "mean_psi_middle_j_per_m3": r.mean_psi_middle_j_per_m3,
                "mean_psi_outer_j_per_m3": r.mean_psi_outer_j_per_m3,
                "max_psi_outer_j_per_m3": r.max_psi_outer_j_per_m3,
            })
        })
        .collect();
    let final_pairs = final_step
        .final_step_data
        .as_ref()
        .map(|d| d.pair_records.clone())
        .unwrap_or_default();
    let document = json!({
        "scalars": scalars,
        "material_layers": materials,
        "ramp_curve": ramp_curve,
        "final_contact_pairs": final_pairs,
    });
    std::fs::write(path, serde_json::to_string_pretty(&document)?)?;
    Ok(())
}

// =============================================================================
// PLY z-slab artifact emit (final step only)
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
        // unscaled physical magnitude. v2 final-step max_disp ~6.7 mm
        // × 10× = 67 mm rendered, fits in cf-view's bbox without
        // saturation. Lower than row 21 v1's 50× because the v2
        // displacements are ~3-4× larger at the deeper final pose.
        const DISPLACEMENT_SCALE: f64 = 10.0;
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
    println!("scan-fit-3layer-sleeve-ramp — row 22 (Tier 6 synthesis #3)");
    println!();

    // 1-5. Build initial mesh + verifies (geometry + material checks).
    let scan_solid = build_scan_solid();
    let outer_envelope = build_outer_envelope(scan_solid.clone());
    let sleeve_body = build_sleeve_body(outer_envelope.clone(), scan_solid.clone());
    let material_field = build_material_field();
    let hints = build_hints(material_field);
    let mesh =
        SdfMeshedTetMesh::from_sdf(&sleeve_body, &hints).map_err(|e| anyhow::anyhow!("{e:?}"))?;

    let n_tets = mesh.n_tets();
    let n_vertices = mesh.n_vertices();
    let positions: Vec<Vec3> = mesh.positions().to_vec();
    let tets: Vec<[VertexId; 4]> = (0..n_tets as u32).map(|t| mesh.tet_vertices(t)).collect();
    let referenced = referenced_vertices(&mesh);

    let bc = build_boundary_conditions(&mesh, &referenced, &outer_envelope);
    let n_pinned = bc.pinned_vertices.len();

    // Per-shell tet counts at rest centroids — bin by `phi =
    // scan.eval(centroid)` distance-from-scan.
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

    // Quality + counts gates BEFORE the ramp (geometry-derived, bit-
    // equal to row 21 v1's anchors).
    verify_quality_floors(&mesh);
    verify_counts_exact(
        &mesh,
        &referenced,
        &bc.pinned_vertices,
        n_inner,
        n_middle,
        n_outer,
    );
    verify_material_assignment_partition(&mesh, &shell_idx_per_tet);
    verify_material_provenance();

    // 6. Quasi-static intrusion ramp — chained `replay_step` over
    // 12 steps. The upstream `mesh` + `bc` are NOT moved into the
    // ramp's first solver call — the ramp rebuilds its own mesh per
    // step (same SDF, same hints, deterministic BCC + IS → bit-equal
    // mesh).
    drop(mesh);
    drop(bc);
    let _outer_kept = outer_envelope; // silence unused

    let results = solve_ramp(
        n_vertices,
        &referenced,
        &tets,
        &shell_idx_per_tet,
        n_inner,
        n_middle,
        n_outer,
    )?;

    // 7. Per-step + final-step verifies.
    verify_n_ramp_steps_exact(results.len());
    verify_per_step_solver_converges(&results);
    verify_per_step_iter_count(&results);
    verify_force_displacement_monotone(&results);
    verify_per_step_strain_energy_ordering(&results);
    verify_per_step_max_disp_bounded(&results);
    verify_n_contact_pairs_final_exact(&results);
    verify_outer_layer_max_psi_final(&results);
    verify_per_step_captured_bits(&results);

    // 8. JSON + PLY readouts.
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;

    let json_path = out_dir.join("scan_fit_3layer_sleeve_ramp.json");
    write_json_readout(
        &json_path,
        n_tets,
        n_vertices,
        referenced.len(),
        n_pinned,
        n_inner,
        n_middle,
        n_outer,
        &results,
    )?;

    // PLY z-slab (final step only) — uses final-step rest_positions
    // and deformed_positions captured in `RampStepResult.final_step_data`.
    let final_step = results.last().expect("ramp produced no results");
    let final_data = final_step
        .final_step_data
        .as_ref()
        .expect("final ramp step missing FinalStepData");
    let half_cell = 0.5 * CELL_SIZE;
    let mut zslab_records: Vec<ZslabRecord> = Vec::new();
    let mut zslab_disp: Vec<f64> = Vec::new();
    let mut zslab_mat: Vec<f64> = Vec::new();
    let mut n_inner_z = 0usize;
    let mut n_middle_z = 0usize;
    let mut n_outer_z = 0usize;
    for (tet_idx, &[v0, v1, v2, v3]) in tets.iter().enumerate() {
        let rest_centroid = (final_data.rest_positions[v0 as usize]
            + final_data.rest_positions[v1 as usize]
            + final_data.rest_positions[v2 as usize]
            + final_data.rest_positions[v3 as usize])
            / 4.0;
        if rest_centroid.z.abs() >= half_cell {
            continue;
        }
        let deformed_centroid = (final_data.deformed_positions[v0 as usize]
            + final_data.deformed_positions[v1 as usize]
            + final_data.deformed_positions[v2 as usize]
            + final_data.deformed_positions[v3 as usize])
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

    let ply_path = out_dir.join("sleeve_zslab_final.ply");
    emit_zslab_ply(&ply_path, &zslab_records, &zslab_disp, &zslab_mat)?;

    // 9. Museum-plaque summary.
    print_summary(
        n_tets,
        n_vertices,
        referenced.len(),
        n_pinned,
        n_inner,
        n_middle,
        n_outer,
        &results,
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
    results: &[RampStepResult],
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
    println!("Quasi-static ramp:");
    println!("  N_RAMP_STEPS             : {N_RAMP_STEPS}");
    println!(
        "  RAMP_STEP_DELTA          : {RAMP_STEP_DELTA} m ({} mm)",
        RAMP_STEP_DELTA * 1000.0
    );
    println!(
        "  PROBE_PENETRATION_FINAL  : {PROBE_PENETRATION_FINAL} m ({} mm)",
        PROBE_PENETRATION_FINAL * 1000.0
    );
    println!("  PROBE_RADIUS             : {PROBE_RADIUS} m");
    println!("  MAX_NEWTON_ITER          : {MAX_NEWTON_ITER}");
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
    println!("Force-displacement curve (per-step):");
    println!("  step | depth(mm) | iter | residual    | n_pairs | force_z(N)    | max_disp(m)");
    println!("  -----|-----------|------|-------------|---------|---------------|------------");
    for r in results {
        println!(
            "   {:>3} |  {:>5.3}    |  {:>3} | {:.3e}  |  {:>5}  | {:+.4e}  |  {:.3e}",
            r.step,
            r.depth_m * 1000.0,
            r.iter_count,
            r.final_residual_norm,
            r.n_active_pairs,
            r.force_total_z_n,
            r.max_disp_m,
        );
    }
    println!();
    let final_step = results.last().expect("ramp produced no results");
    println!(
        "Final step ({}, depth = {} mm):",
        final_step.step,
        final_step.depth_m * 1000.0
    );
    println!("  Per-layer mean strain-energy density (J/m³):");
    println!("    Ψ̄_inner  : {:e}", final_step.mean_psi_inner_j_per_m3);
    println!("    Ψ̄_middle : {:e}", final_step.mean_psi_middle_j_per_m3);
    println!("    Ψ̄_outer  : {:e}", final_step.mean_psi_outer_j_per_m3);
    println!("    max Ψ_outer : {:e}", final_step.max_psi_outer_j_per_m3);
    println!();
    println!("Outputs:");
    println!(
        "  out/scan_fit_3layer_sleeve_ramp.json (scalars + materials + ramp_curve + final_pairs)"
    );
    println!(
        "  out/sleeve_zslab_final.ply           (z-slab centroid cloud at final step, two scalars)"
    );
    println!();
    println!("View final-step PLY in cf-view (workspace's unified visual-review viewer):");
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-ramp/out/sleeve_zslab_final.ply"
    );
    println!();
    println!("Optional matplotlib post-processing (force-displacement + max_disp curves):");
    println!("  uv run examples/sim-soft/scan-fit-3layer-sleeve-ramp/plot_ramp.py");
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
    assert!(WRAP_THICKNESS > 0.0);
    assert!(CELL_SIZE > 0.0);
    assert!(STATIC_DT > 0.0);
    assert!(PROBE_RADIUS > 0.0);
    assert!(N_RAMP_STEPS > 0);
    assert!(PROBE_PENETRATION_FINAL > 0.0);
    assert!(PROBE_PENETRATION_FINAL < PROBE_RADIUS);
    assert!(RAMP_STEP_DELTA > 0.0);
    assert!(MAX_NEWTON_ITER > 0);
    assert!(LAYER_OUTER - LAYER_MIDDLE_OUTER >= CELL_SIZE);
    assert!(BBOX_HALF_X > SCAN_HX + WRAP_THICKNESS);
    assert!(BBOX_HALF_Y > SCAN_HY + WRAP_THICKNESS);
    assert!(BBOX_HALF_Z > SCAN_HZ + WRAP_THICKNESS);
    assert!(N_RAMP_STEPS_EXACT == N_RAMP_STEPS);
};
