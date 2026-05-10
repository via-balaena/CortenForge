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

//! scan-fit-3layer-sleeve-yeoh-ramp — row 23 (F4.1 Yeoh consumer): the
//! load-bearing first-consumer of the Yeoh hyperelastic foundation
//! shipped in PR #235 (`e0c2f856`). Same scan + 3-layer sleeve + rigid
//! intrusion probe geometry as row 22
//! [`scan-fit-3layer-sleeve-ramp`](../scan-fit-3layer-sleeve-ramp);
//! constitutive model swapped from [`sim_soft::NeoHookean`] to
//! [`sim_soft::Yeoh`] (additive `C₂(I₁−3)²` extension over NH's
//! deviatoric kernel, same NH-style compressibility — see
//! [Yeoh arc memo][arcmemo]); ramp extended from 12 × 0.5 mm = 6 mm
//! to 16 × 0.5 mm = 8 mm so the `+4` past-NH-wall steps test whether
//! Yeoh's wider validity envelope translates into solver-clean
//! convergence at the user-target physical depth.
//!
//! [arcmemo]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_yeoh_hyperelastic_arc.md
//! [mem]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_layered_silicone_device.md
//! [v2spec]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_row_22_v2_spec.md
//!
//! # Why Yeoh, not Neo-Hookean
//!
//! Row 22's post-ship investigation (banked at [v2 spec memo][v2spec]
//! §"Post-ship investigation") identified the wall at `max_disp ≈
//! 7 mm` as the **Neo-Hookean validity domain** tripping fail-closed.
//! Phase 4 Decision Q's `validate_F_in_domain` refuses to evaluate NH
//! past `max_stretch_deviation < 1.0`; at 6.5 mm cavity-wall
//! displacement, one tet under the probe has principal stretches
//! `[2.06, 1.22, 0.073]` — both extremes are past NH's calibrated
//! faithful range, with the **0.073 (92.7 % compression)** value the
//! load-bearing failure (the symmetric NH gate happens to catch both
//! at the `max(|σᵢ - 1|) > 1.0` threshold; that's a coincidence of
//! NH's specific bound).
//!
//! Yeoh has per-anchor **asymmetric** validity bounds:
//! `max_principal_stretch ≈ 5-9` (per-family tensile cap, calibrated
//! from Smooth-On TDS elongation-at-break) and `min_principal_stretch
//! = 0.30` (engineering-aggressive default, no published Yeoh-on-
//! silicone compression bound exists per the arc memo §A2 web-search
//! recon). The compressive 0.073 row 22 saw is well inside Yeoh's
//! 0.30 floor. Whether the SOLVER also handles the past-6.5-mm regime
//! (Armijo `α_min = 2⁻²¹` floor was the v2-spike secondary wall,
//! independent of the constitutive model) is the load-bearing unknown
//! the first run answers.
//!
//! # Why a ramp, not a single step
//!
//! Same rationale as row 22 inherited from row 21: deeper penetration
//! single-stepped on this geometry is expected to push the iter-0
//! penalty gradient out of Newton's basin. The quasi-static ramp
//! circumvents the single-step basin issue: each step's `x_prev` is
//! the previous step's `x_final` (cavity wall already deformed to
//! the previous step's equilibrium), so the iter-0 gradient is
//! bounded by the per-step delta only.
//!
//! # Pipeline
//!
//! 1. **Scan stand-in via `Solid::cuboid`** — same as row 22.
//!    `(SCAN_HX, SCAN_HY, SCAN_HZ) = (0.020, 0.015, 0.040) m`
//!    half-extents.
//!
//! 2. **Outer envelope via `Solid::offset(WRAP_THICKNESS)`** — same.
//!
//! 3. **Sleeve body via `Solid::subtract`** — same.
//!
//! 4. **`SdfMeshedTetMesh<Yeoh>` build** via PR #235's
//!    `from_sdf_yeoh` (F4.0 generic-mesh-over-Material refactor —
//!    `SdfMeshedTetMesh<M = NeoHookean>` defaults to NH for legacy
//!    consumers; this row writes `<Yeoh>` explicitly to pick the
//!    Yeoh slot). Mesh is rebuilt fresh at every ramp step
//!    (deterministic BCC + IS pipeline). Cost: ~16× row 21's single
//!    rebuild ≈ ~2 s release.
//!
//! 5. **3-layer `MaterialField`** via `MaterialField::from_yeoh_fields`
//!    (three scalar fields: μ, C₂, λ — `C₁ = μ/2` is derived per
//!    arc memo D2). Inner = Path-2 parametric construction via
//!    [`SiliconeMaterial::from_effective_shore`] at
//!    `ShoreReading::DoubleZero(20.0)` (Slacker-softened Ecoflex
//!    00-30 proxy at effective Shore 00-20; the bracket weight at
//!    20 is 1.0 so the produced parameters bit-coincide with the
//!    `ECOFLEX_00_20` anchor — the [`ConstructionSource::Interpolated`]
//!    provenance tag distinguishes the construction path from a
//!    direct anchor reference). Middle = `DRAGON_SKIN_10A.to_yeoh()`
//!    (Path 1 — direct anchor → Yeoh). Outer =
//!    `DRAGON_SKIN_20A.to_yeoh()` (Path 1).
//!
//! 6. **Quasi-static intrusion ramp** — `N_RAMP_STEPS = 16` steps of
//!    `RAMP_STEP_DELTA = 0.5 mm` each, reaching
//!    `PROBE_PENETRATION_FINAL = 8 mm`. At step `k` (1-indexed): the
//!    probe sits at `probe_z_k = SCAN_HZ - PROBE_RADIUS + k *
//!    RAMP_STEP_DELTA`; `PenaltyRigidContact::new(vec![probe_k])`
//!    receives the new probe; `replay_step(x_prev=x_final[k-1],
//!    v_prev=0, ...)` solves the per-step equilibrium. `STATIC_DT =
//!    1.0 s` collapses the inertial term per step.
//!    `MAX_NEWTON_ITER = 150` (vs row 22's 100) hedges against the
//!    extra 4 ramp steps + Yeoh's `C₂(I₁−3)²` Newton-direction drift.
//!
//! 7. **Per-step + final-step readouts** — at every ramp step, capture
//!    `(iter_count, final_residual_norm, n_active_pairs,
//!    force_total_z, mean_force_magnitude, max_force_magnitude,
//!    mean_disp, max_disp, mean_psi_inner/middle/outer,
//!    max_psi_outer)` into a `RampStepResult`. Per-tet `Ψ_t =
//!    Material::energy(F_t)` calls `Yeoh::energy` on every Yeoh tet.
//!    The final step (step 16 at 8 mm) drives the headline
//!    captured-bit anchors + the PLY z-slab artifact.
//!
//! 8. **Readouts** —
//!    - JSON `out/scan_fit_3layer_sleeve_yeoh_ramp.json`: 4-section
//!      schema (scalars at final step + `material_layers` provenance
//!      with Yeoh-specific fields `c2_pa`,
//!      `validity_max/min_principal_stretch`, `source` tag +
//!      `ramp_curve` 16-element array + `final_contact_pairs`
//!      per-pair detail at step 16 only).
//!    - PLY `out/sleeve_boundary_final.ply`: full 3D body via
//!      [`sim_soft::viz::boundary_surface`] (F1.1 lift), with
//!      sequential `displacement_magnitude` + categorical
//!      `material_id` per-vertex (volume-weighted averaged from
//!      per-tet). Replaces the pre-F1.5 z-slab centroid cloud's
//!      reduce-to-2D framing.
//!    - PLY `out/sleeve_slab_cut_z0_final.ply`: equatorial
//!      cross-section at `z = 0` via
//!      [`sim_soft::viz::slab_cut`] (F1.1 lift). Marching-tetrahedra
//!      intersection; per-vertex scalars linearly interpolated along
//!      cross-edges. Catches the propagated secondary response 40 mm
//!      below the contact zone — exposes the radial material-shell
//!      partition AND the cavity-wall displacement at the equator
//!      from one cut.
//!    - Optional `plot_ramp.py` (PEP 723 + matplotlib): dual-axis
//!      depth × `force_z` + depth × `max_disp` force-displacement
//!      curve. Run via `uv run plot_ramp.py`.
//!    - `verify_*` runtime gates (12 anchor groups, see "Numerical
//!      anchors" in `README.md`).
//!
//! # Why z = 0 for the slab cut
//!
//! Pattern (aa) banked at row 16 N+3: hollow / interior-cavity /
//! partial-occlusion bodies need an axis-aligned cut to expose
//! interior structure. The 3-layer sleeve's doubly-hollow geometry
//! cuts cleanly at `z = 0` — the equatorial cross-section is 40 mm
//! BELOW the contact zone at `z ≈ 0.040 m`, so it catches the
//! propagated secondary response (NOT the contact-zone signal
//! directly) and exposes the radial shells + cavity wall in one
//! frame. F1.5 retrofit replaces the centroid cloud the row's
//! pre-F1.5 emit produced with a marching-tet cross-section via the
//! public viz API.
//!
//! # Sanitization
//!
//! Per the [device memo][mem]'s sanitization directive: the scanned
//! reference geometry is referred to as "scanned reference geometry"
//! or "scan stand-in" throughout this crate's prose. No anatomical
//! references appear in any tracked surface. The cuboid placeholder
//! is a parametric synthetic stand-in — the pipeline demonstration is
//! the workflow ("scan-shaped body → wrap by offset → carve cavity →
//! 3-material Yeoh FEM → multi-step rigid intrusion ramp"), not the
//! cuboid's specific geometry; production runs swap the cuboid for a
//! real scan via row 15's STL-import path without any other code
//! change.
//!
//! # Run
//!
//! ```sh
//! cargo run -p example-sim-soft-scan-fit-3layer-sleeve-yeoh-ramp --release
//! ```
//!
//! Per `feedback_release_mode_heavy_tests` — release mode is required.
//! The 16-step ramp at ~75 k tets through faer's sparse Cholesky takes
//! ~40-90 s release. The `CELL_SIZE = 0.004 m` (4 mm) is sized so
//! each of the 6/4/4 mm layers carries at least one BCC cell across
//! thickness; finer cells (e.g., `0.002 m`) trip an SPD pivot at the
//! FIRST ramp step (empirically tested at row-22 v2-spec spike time).
//!
//! Optional matplotlib post-processing:
//!
//! ```sh
//! uv run examples/sim-soft/scan-fit-3layer-sleeve-yeoh-ramp/plot_ramp.py
//! ```

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use cf_design::Solid;
use mesh_io::save_ply_attributed;
use mesh_types::Vector3;
use nalgebra::Matrix3;
use serde_json::{Value, json};
use sim_ml_chassis::Tensor;
use sim_soft::material::silicone_table::{DRAGON_SKIN_10A, DRAGON_SKIN_20A};
use sim_soft::{
    Aabb3, BoundaryConditions, ConstructionSource, CpuNewtonSolver, Field, LayeredScalarField,
    Material, MaterialField, Mesh, MeshingHints, PenaltyRigidContact,
    PenaltyRigidContactYeohSolver, Plane, Sdf, SdfMeshedTetMesh, ShoreReading, SiliconeMaterial,
    Solver, SolverConfig, Tet4, Vec3, VertexId, Yeoh, boundary_surface, design_slab_cut,
    design_surface, design_surface_deformed, pick_vertices_by_predicate, referenced_vertices,
    slab_cut,
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

/// Number of ramp steps. 16 × 0.5 mm/step = 8 mm total — the
/// user-target physical intrusion. Row 22 stopped at 12 steps × 0.5 mm
/// = 6 mm because Neo-Hookean's symmetric `max_stretch_deviation < 1.0`
/// validity gate refused at ~6.5 mm (the post-ship investigation found
/// a tet under the probe with principal stretches `[2.06, 1.22, 0.073]`
/// — the 0.073 compressive value was the load-bearing failure). Yeoh's
/// per-anchor asymmetric bounds (`max_principal_stretch ≈ 5-9` per
/// family, `min_principal_stretch = 0.30` engineering-aggressive
/// default) admit that envelope; whether the SOLVER also handles the
/// past-6.5-mm regime (Armijo `α_min = 2⁻²¹` floor was the v2-spike
/// secondary wall, independent of the constitutive model) is the load-
/// bearing unknown the first run answers.
const N_RAMP_STEPS: usize = 16;

/// Final probe penetration depth (m). 8 mm — the user-target physical
/// intrusion. The v2 spike characterised row 22's reach as `max_disp
/// ≈ 7 mm` at the wall; row 23 tests whether Yeoh's wider validity
/// envelope translates the wall-clearance from a constitutive issue
/// to a (possibly absent, possibly present) solver issue.
const PROBE_PENETRATION_FINAL: f64 = 0.008;

/// Penetration delta per ramp step (m). 0.5 mm/step — preserved from
/// row 22 so per-step convergence behavior is comparable.
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

/// Newton iter cap per ramp step. Bumped from row 22's 100 to 150 to
/// absorb (a) the additional 4 ramp steps (13-16) past row 22's 6 mm
/// terminus and (b) Yeoh's `C₂(I₁−3)²` stiffening, which can either
/// shorten the per-step Newton path (smoother stress-strain curve) or
/// lengthen it (different search-direction geometry in the new
/// constitutive). 150 is a hedge with empirical headroom; the bootstrap
/// run characterises actual per-step iter envelope and the locked
/// `IT_COUNT_RAMP_EXACT` array below pins it bit-exactly.
const MAX_NEWTON_ITER: usize = 150;

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

/// Bit-exact tolerance for the F4 const-fn `to_yeoh()` Yeoh-parameters
/// round-trip (μ + λ + C₂ + asymmetric validity bounds, per F4.0
/// arc-memo §"Implementation status").
const F4_PROVENANCE_EXACT_TOL: f64 = 0.0;

/// Probe `F = diag(1.01, 1, 1)` material-assignment-probe tolerance.
const MATERIAL_PROBE_EXACT_TOL: f64 = 0.0;

// =============================================================================
// Constants — captured first-run anchor bits
// =============================================================================
//
// **Capture provenance** — captured 2026-05-09 at sim-soft `dev` (post
// PR #235 Yeoh-foundation tip `e0c2f856`), rustc 1.95.0
// (`59807616e` 2026-04-14) on macOS arm64 — same toolchain + platform
// as IV-1's reference capture and row 22's. First-run capture
// bootstrapped via `CF_CAPTURE_BITS=1` (pattern (cc) banked at row
// 19): when set, every captured-anchor check is bypassed and a paste-
// ready capture block is printed to stderr; when unset (default),
// every captured-bits gate runs the strict `to_bits()` self-pin
// against the constants below.
//
// Geometry-derived counts (`N_TETS_EXACT`, `N_VERTICES_EXACT`,
// `N_REFERENCED_EXACT`, `N_PINNED_EXACT`, the per-shell tet counts,
// the z-slab counts) are bit-equal to row 22's captures — the BCC + IS
// pipeline is deterministic on the same SDF + hints; only the material
// model differs, and material doesn't affect the discretisation.

/// Total tet count after BCC + Isosurface Stuffing on the sleeve body.
/// Bit-equal to row 22's `N_TETS_EXACT = 74_628` (geometry + BCC + IS
/// pipeline are deterministic on the same SDF + hints).
const N_TETS_EXACT: usize = 74_628;

/// Total mesh vertex count, including BCC corners not referenced by
/// any tet. Bit-equal to row 22.
const N_VERTICES_EXACT: usize = 31_966;

/// Vertices referenced by ≥ 1 tet. Bit-equal to row 22.
const N_REFERENCED_EXACT: usize = 17_384;

/// Outer-envelope-surface Dirichlet-pinned vertex count. Bit-equal to
/// row 22.
const N_PINNED_EXACT: usize = 7_046;

/// Per-shell tet counts at first capture. Bit-equal to row 22.
const N_INNER_TETS_EXACT: usize = 25_892;
const N_MIDDLE_TETS_EXACT: usize = 16_656;
const N_OUTER_TETS_EXACT: usize = 32_080;

/// Per-shell tet counts in the `|centroid.z| < CELL_SIZE / 2 = 0.002`
/// z-slab cut for the cf-view PLY artifact (final step only).
/// Bit-equal to row 22 (geometry + slab cut unchanged).
const N_INNER_TETS_ZSLAB_EXACT: usize = 768;
const N_MIDDLE_TETS_ZSLAB_EXACT: usize = 432;
const N_OUTER_TETS_ZSLAB_EXACT: usize = 892;

/// Ramp-step partition gate. Bit-pinned to `N_RAMP_STEPS = 16`.
const N_RAMP_STEPS_EXACT: usize = N_RAMP_STEPS;

/// Active contact-pair count at the FINAL ramp step (depth = 8 mm),
/// filtered to REFERENCED vertices. 50 physical contacts at 8 mm vs
/// row 22's 37 at 6 mm — deeper penetration engages more wrap-cap
/// material on the contact band.
const N_CONTACT_PAIRS_FINAL_EXACT: usize = 50;

/// Per-step Newton iter counts. The chained `replay_step` is
/// deterministic on a fixed toolchain; iter-count drift signals real
/// solver-path regression, not noise. Row 22's NH counts at depths
/// 0.5..6 mm were `[8, 8, 9, 11, 11, 13, 14, 16, 19, 22, 30, 61]`;
/// row 23's Yeoh counts at the same depths are
/// `[8, 8, 9, 10, 11, 12, 13, 15, 16, 19, 23, 27]` — Yeoh's
/// `C₂(I₁−3)²` stiffening yields a smoother Newton path past row 22's
/// step 11 (e.g. step 12 drops 61→27 iters). The past-NH-wall
/// extension `[31, 39, 49, 77]` at depths 6.5..8 mm escalates as
/// expected and remains under the `MAX_NEWTON_ITER = 150` cap with
/// 73-iter margin at step 16.
const IT_COUNT_RAMP_EXACT: [usize; N_RAMP_STEPS] =
    [8, 8, 9, 10, 11, 12, 13, 15, 16, 19, 23, 27, 31, 39, 49, 77];

/// Per-step `+z`-component of contact reaction force bits (N), summed
/// over REFERENCED vertices only. Force is in `+z` direction (probe
/// pushes wrap-cap material UP), grows monotonically with deeper
/// penetration. Approximate values: `[1.10, 1.89, 2.97, 4.20, 5.64,
/// 7.35, 9.44, 11.92, 14.72, 17.89, 21.49, 25.60, 30.27, 35.51, 41.79,
/// 49.28] N`. The past-6 mm tail (steps 13-16) climbs faster than
/// linear extrapolation from row 22's NH curve — Yeoh's `C₂(I₁−3)²`
/// stiffening engages at the higher-strain regime where NH would have
/// failed validity-closed.
const FORCE_TOTAL_Z_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x3ff1_ae1f_ecfc_3482,
    0x3ffe_3701_b63b_1b78,
    0x4007_b876_1acd_eeef,
    0x4010_ccd3_13cd_4b9f,
    0x4016_8b71_f81d_df8d,
    0x401d_63cd_7f11_9259,
    0x4022_e1ff_73a6_599d,
    0x4027_d5ea_c8ca_8205,
    0x402d_6e4e_7a66_bbd5,
    0x4031_e339_047f_1bc2,
    0x4035_7dc4_9585_e8e0,
    0x4039_9ab3_ac32_828a,
    0x403e_441b_cbf2_fd16,
    0x4041_c173_9fb4_4199,
    0x4044_e566_7d9c_def5,
    0x4048_a441_1f47_e7ee,
];

/// Per-step max body-wide displacement-magnitude bits (m) over all
/// referenced vertices. Approximate values: `[1.48, 1.97, 2.47, 2.95,
/// 3.44, 3.93, 4.41, 4.88, 5.35, 5.82, 6.27, 6.72, 7.15, 7.57, 8.00,
/// 8.45] mm`. Step 16's `max_disp = 8.45 mm` < `WRAP_THICKNESS = 14
/// mm` — the geometric upper-bound gate stays comfortable throughout
/// the 8 mm ramp.
const MAX_DISP_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x3f58_423e_2986_d12c,
    0x3f60_2acc_ce26_2941,
    0x3f64_30f0_cc86_8cd6,
    0x3f68_33b9_339d_2671,
    0x3f6c_323c_b5d5_48e8,
    0x3f70_158b_dd0e_6680,
    0x3f72_0d70_6fe9_7f33,
    0x3f73_fffc_084e_46ab,
    0x3f75_eccd_94df_894d,
    0x3f77_d2b9_1ffd_4735,
    0x3f79_b05e_8e5b_a430,
    0x3f7b_8416_1059_bcb7,
    0x3f7d_4bcd_74de_81fc,
    0x3f7f_04a9_c660_267a,
    0x3f80_6093_1c13_e0e8,
    0x3f81_5097_9889_b741,
];

/// Per-step inner-layer mean strain-energy-density bits (J/m³). Inner
/// is softest (μ = 18 kPa) AND closest to probe → highest mean Ψ
/// throughout the ramp. Final-step value ≈ 606 J/m³ (~2× row 22's
/// 302 J/m³ at 6 mm; Yeoh's `C₂(I₁−3)²` term contributes the extra
/// ~40 % beyond simple depth-doubling).
const MEAN_PSI_INNER_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x4011_eb65_64cf_33f7,
    0x4022_b0db_1a77_230c,
    0x4031_1499_8372_4c87,
    0x403c_9a89_0f24_7c56,
    0x4045_fdaa_a645_dd14,
    0x404f_ca59_1b6b_9609,
    0x4056_0bd8_f7ca_f091,
    0x405d_a2f5_6d98_d11f,
    0x4063_6464_b9d2_ce6a,
    0x4068_c667_760f_a388,
    0x406e_ff67_bbb1_d5f6,
    0x4073_0e94_28d3_00ad,
    0x4077_14f7_a39c_4147,
    0x407b_98c0_a965_7d77,
    0x4080_3f9b_cb1f_4fe4,
    0x4082_f371_6779_42c0,
];

/// Per-step middle-layer mean strain-energy-density bits (J/m³).
/// Final-step value ≈ 295 J/m³.
const MEAN_PSI_MIDDLE_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x3fe3_5bad_1472_aec5,
    0x3ff8_bfc8_f1c7_7705,
    0x400a_1d29_e1c4_d020,
    0x4017_df75_fb75_d616,
    0x4023_ee42_da9c_0c65,
    0x402f_5580_565c_5ded,
    0x4037_7c15_c840_fa7e,
    0x4040_f954_3f40_a149,
    0x4047_d253_c78e_a0aa,
    0x4050_4d8f_1618_4696,
    0x4055_d4f4_7bc0_a55c,
    0x405c_a75a_9ba5_7027,
    0x4062_7471_e281_67b0,
    0x4067_54eb_ff36_63a5,
    0x406d_71fa_ee5c_5da5,
    0x4072_6c0e_e7e9_5277,
];

/// Per-step outer-layer mean strain-energy-density bits (J/m³). Outer
/// is stiffest (μ = 113 kPa) AND outer-Dirichlet-pinned → lowest mean
/// Ψ throughout the ramp. Final-step value ≈ 113 J/m³.
const MEAN_PSI_OUTER_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x3fc0_c9d4_d9aa_fd6e,
    0x3fd6_767d_32ae_dee8,
    0x3fe8_d78c_11a0_1e05,
    0x3ff7_7840_0de5_6d5b,
    0x4004_2de8_46ae_5518,
    0x4010_5965_aa4f_b720,
    0x4019_52ea_2650_8bda,
    0x4022_ec27_b1df_c12e,
    0x402b_72b5_da18_a764,
    0x4033_7058_8bd2_58b2,
    0x403a_fcb8_296d_d29d,
    0x4042_6456_06ab_635c,
    0x4048_a375_f756_a12a,
    0x4050_3464_6d6b_7d9b,
    0x4055_63d9_f408_9237,
    0x405c_32cb_d5fb_1988,
];

/// Final-step (step 16, depth = 8 mm) outer-layer max strain-energy-
/// density bits (J/m³). Durability proxy at the user-target depth:
/// ~49205 J/m³ (~ 436× outer-layer mean ~113 J/m³ — peak localises in
/// tets adjacent to the contact band where probe loads transmit
/// through the radial chain inner → middle → outer; the ratio is
/// higher than row 22's 345× because of Yeoh's high-strain stiffening
/// at the 8 mm engaged-contact regime).
const MAX_PSI_OUTER_FINAL_REF_BITS: u64 = 0x40e8_069d_d6fd_8307;

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
// Inner-layer silicone — Path 2 (parametric) constructor demo
// =============================================================================

/// Path-2 (parametric) construction of the inner-layer silicone via
/// [`SiliconeMaterial::from_effective_shore`]. At `DoubleZero(20.0)`
/// the `(ECOFLEX_00_10, ECOFLEX_00_20)` bracket has `weight = 1.0`
/// (pinned by the test at `silicone_table.rs:893-918`), so the
/// produced `(μ, C₂, λ, density, validity_bounds)` tuple is bit-equal
/// to the `ECOFLEX_00_20` anchor; the
/// [`ConstructionSource::Interpolated { weight: 1.0, .. }`] provenance
/// tag distinguishes the construction path from a direct anchor
/// reference. Demonstrating Path 2 syntactically here is the value —
/// production Slacker-softened recipes that don't coincide with a
/// published anchor reach for the same API with non-degenerate
/// bracketing weights.
fn inner_silicone() -> SiliconeMaterial {
    SiliconeMaterial::from_effective_shore(
        ShoreReading::DoubleZero(20.0),
        Some(
            "Slacker-softened Ecoflex 00-30 at effective Shore 00-20 \
             (Path 2 parametric API; weight=1.0 at ECOFLEX_00_20 anchor)",
        ),
    )
    .expect("DoubleZero(20.0) brackets ECOFLEX family at weight=1.0")
}

// =============================================================================
// 3-layer MaterialField — Yeoh (μ, C₂, λ) per arc-memo D10
// =============================================================================

fn build_material_field() -> MaterialField {
    let scan_for_partition = || Box::new(build_scan_solid()) as Box<dyn Sdf>;
    let inner = inner_silicone();

    let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![inner.mu, DRAGON_SKIN_10A.mu, DRAGON_SKIN_20A.mu],
    ));
    let c2_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![inner.c2, DRAGON_SKIN_10A.c2, DRAGON_SKIN_20A.c2],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![inner.lambda, DRAGON_SKIN_10A.lambda, DRAGON_SKIN_20A.lambda],
    ));
    MaterialField::from_yeoh_fields(mu_field, c2_field, lambda_field)
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
    mesh: &SdfMeshedTetMesh<Yeoh>,
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
    /// Per-step deformed vertex positions. Always captured (F2.3c needs
    /// these per-step for the `design_surface_deformed` PLY series).
    deformed_positions: Vec<Vec3>,
    /// Per-tet strain-energy density (J/m³), indexed by tet ID,
    /// length = `n_tets`. Captured every step (F2.3c needs per-step
    /// psi for the ramp-animation PLY series); pre-F2.3c this was
    /// final-step only and lived inside [`FinalStepData`].
    per_tet_psi: Vec<f64>,
    /// Step 12 only: per-pair readouts for JSON `final_contact_pairs`
    /// and `rest_positions` for the final-step PLY emits. Intermediate
    /// steps drop this to keep memory bounded.
    final_step_data: Option<FinalStepData>,
}

#[derive(Clone, Debug)]
struct FinalStepData {
    rest_positions: Vec<Vec3>,
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

    // v2.5 anchor cleanup (2026-05-08): each per-step readout is
    // filtered to referenced (= solver-active) vertices via
    // `sim_soft::filter_pair_readouts_to_referenced` — the unfiltered
    // readout includes ORPHAN BCC lattice corners not in any tet,
    // ignored by the solver but counted + summed in the raw
    // aggregates (~95-97 % orphan share at this geometry). See
    // pattern (xx) at row 22 patterns memo.

    // Initial x_prev = rest positions (from a one-shot mesh build).
    let initial_mesh = {
        let scan = build_scan_solid();
        let outer = build_outer_envelope(scan.clone());
        let body = build_sleeve_body(outer, scan);
        let hints = build_hints(build_material_field());
        SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&body, &hints)
            .map_err(|e| anyhow::anyhow!("{e:?}"))?
    };
    let mut x_prev_flat: Vec<f64> = vec![0.0; n_dof];
    for (v, p) in initial_mesh.positions().iter().enumerate() {
        x_prev_flat[3 * v] = p.x;
        x_prev_flat[3 * v + 1] = p.y;
        x_prev_flat[3 * v + 2] = p.z;
    }
    drop(initial_mesh);

    let mut results: Vec<RampStepResult> = Vec::with_capacity(N_RAMP_STEPS);

    println!(
        "Quasi-static ramp — solving {N_RAMP_STEPS} steps × {RAMP_STEP_DELTA:.4} m to depth {PROBE_PENETRATION_FINAL} m:"
    );
    for k in 0..N_RAMP_STEPS {
        let depth = (k + 1) as f64 * RAMP_STEP_DELTA;

        // Per-step progress log. Late steps (e.g. step 16 at ~77 iters)
        // can take 10-15 s on a release build; without this line the
        // user sees a silent terminal between the early "starting"
        // print and the post-ramp summary table. Use stderr so the
        // line flushes even when stdout is piped (CI capture).
        eprintln!(
            "  step {:>2}/{} — depth={:.1} mm — solving…",
            k + 1,
            N_RAMP_STEPS,
            depth * 1000.0,
        );

        // Rebuild mesh + bc + contact for this step.
        let scan_k = build_scan_solid();
        let outer_k = build_outer_envelope(scan_k.clone());
        let body_k = build_sleeve_body(outer_k.clone(), scan_k);
        let hints_k = build_hints(build_material_field());
        let mesh_k = SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&body_k, &hints_k)
            .map_err(|e| anyhow::anyhow!("{e:?}"))?;
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

        let solver: PenaltyRigidContactYeohSolver<SdfMeshedTetMesh<Yeoh>> =
            CpuNewtonSolver::new(Tet4, mesh_k, contact_k, cfg, bc_k);
        let step_k = solver.replay_step(&x_prev_t, &v_prev_t, &theta_t, cfg.dt);

        // Inspection mesh + contact for per-pair readout + Ψ extraction.
        let inspection_mesh = {
            let scan_again = build_scan_solid();
            let outer_again = build_outer_envelope(scan_again.clone());
            let body_again = build_sleeve_body(outer_again, scan_again);
            let inspection_hints = build_hints(build_material_field());
            SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&body_again, &inspection_hints)
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
        // v2.5 cleanup: drop orphan BCC corners — see function-prologue
        // comment above. `filter_pair_readouts_to_referenced` is the
        // sim-soft helper added in the same v2.5 commit.
        let readouts = sim_soft::filter_pair_readouts_to_referenced(raw_readouts, referenced);
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

        // Per-tet Ψ aggregates + per-tet psi vector at final step.
        let is_final = k == N_RAMP_STEPS - 1;
        let materials = inspection_mesh.materials();
        let mut sum_in = 0.0;
        let mut sum_mi = 0.0;
        let mut sum_ou = 0.0;
        let mut max_psi_outer = f64::NEG_INFINITY;
        let mut per_tet_psi: Vec<f64> = Vec::with_capacity(tets.len());
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
            per_tet_psi.push(psi_t);
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
        let final_step_data = if is_final {
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
            deformed_positions: positions_k,
            per_tet_psi,
            final_step_data,
        });

        // Per-step completion log (paired with the "solving…" line
        // above). Same eprintln channel so progress + result lines
        // interleave cleanly under stdout/stderr split capture.
        eprintln!(
            "             iter={:>3}  residual={:.2e}  force_z={:+.3e} N  max_disp={:.3e} m  n_pairs={}",
            step_k.iter_count,
            step_k.final_residual_norm,
            force_total_z,
            max_disp_m,
            n_active_pairs,
        );

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
    mesh: &SdfMeshedTetMesh<Yeoh>,
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

fn verify_quality_floors(mesh: &SdfMeshedTetMesh<Yeoh>) {
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
    let inner = inner_silicone();
    // Provenance: inner is Path 2 (interpolated, weight=1.0); the
    // others are Path 1 anchors. The Path-2 source tag is the
    // distinguishing artifact even when the numerics coincide with
    // the high anchor.
    //
    // Weight is bit-exactly 1.0: bracket() at silicone_table.rs:469
    // computes `(20.0 - 10.0) / (20.0 - 10.0)`, which is exact in
    // IEEE 754. Pinned by the test
    // `silicone_table.rs::tests::from_effective_shore_at_anchor_position_returns_anchor_data`
    // (silicone_table.rs:898). The 1e-15 tolerance is a safety margin
    // against future bracket-math refactors that might introduce
    // sub-ULP drift; tighten to `== 1.0` if the bracket invariant is
    // ever lifted to the type system.
    assert!(
        matches!(
            inner.source,
            ConstructionSource::Interpolated { weight, .. } if (weight - 1.0).abs() < 1e-15,
        ),
        "inner_silicone() source tag is not Interpolated{{weight=1.0}}: {:?}",
        inner.source,
    );

    for mat in [&inner, &DRAGON_SKIN_10A, &DRAGON_SKIN_20A] {
        let yr = mat.to_yeoh();
        let id = Matrix3::<f64>::identity();
        assert_relative_eq!(yr.energy(&id), 0.0, epsilon = F4_PROVENANCE_EXACT_TOL);

        let mut f = Matrix3::<f64>::identity();
        f[(0, 0)] = 1.01;
        // Per arc-memo F1 Spike-1 finding: Yeoh::energy must match
        // additive decomposition `nh_part + C₂·(I₁−3)²` to preserve
        // bit-exactness against NH at C₂=0. Mirror that exact shape
        // here so the comparison is bit-exact for nonzero C₂ too.
        let i1 = 1.01_f64.mul_add(1.01, 2.0);
        let j_ln = 1.01_f64.ln();
        let half_mu = 0.5 * mat.mu;
        let half_lambda = 0.5 * mat.lambda;
        let nh_part = half_lambda.mul_add(j_ln * j_ln, half_mu.mul_add(i1 - 3.0, -mat.mu * j_ln));
        let i1m3 = i1 - 3.0;
        // Bit-exact reproduction of `Yeoh::energy`'s additive
        // decomposition (yeoh.rs:130-137): `nh_part + c2 * i1m3 * i1m3`.
        // Folding c2*i1m3 into a mul_add would change rounding and break
        // bit-equality with the implementation.
        #[allow(clippy::suboptimal_flops)]
        let expected = nh_part + mat.c2 * i1m3 * i1m3;
        assert_relative_eq!(yr.energy(&f), expected, epsilon = F4_PROVENANCE_EXACT_TOL,);
    }
}

fn verify_material_assignment_partition(
    mesh: &SdfMeshedTetMesh<Yeoh>,
    shell_idx_per_tet: &[usize],
) {
    let materials = mesh.materials();
    assert_eq!(
        materials.len(),
        shell_idx_per_tet.len(),
        "materials() length does not match per-tet shell-classification length",
    );
    let mut f = Matrix3::<f64>::identity();
    f[(0, 0)] = 1.01;

    let expected_yeoh = [
        inner_silicone().to_yeoh(),
        DRAGON_SKIN_10A.to_yeoh(),
        DRAGON_SKIN_20A.to_yeoh(),
    ];
    for (t, &shell_idx) in shell_idx_per_tet.iter().enumerate() {
        let observed = materials[t].energy(&f);
        let expected = expected_yeoh[shell_idx].energy(&f);
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
    let inner = inner_silicone();
    let inner_source_tag = match &inner.source {
        ConstructionSource::Anchor { name } => format!("Anchor({name})"),
        ConstructionSource::Interpolated {
            low_anchor,
            high_anchor,
            weight,
            ..
        } => format!("Interpolated({low_anchor}↔{high_anchor}, weight={weight})"),
        ConstructionSource::Measured { user_description } => {
            format!("Measured({user_description})")
        }
        // ConstructionSource is #[non_exhaustive]; future variants
        // collapse to their Rust Debug repr inside the JSON `source`
        // string. The JSON document stays parseable, but the field
        // is no longer a structured Anchor/Interpolated/Measured
        // descriptor — it's a `Foo { bar: 1 }`-style escape hatch
        // that surfaces the variant name to a human reader and
        // signals "extend this match" to the next maintainer.
        other => format!("{other:?}"),
    };
    let materials = json!([
        {
            "name": "inner (Path 2: from_effective_shore DoubleZero(20.0))",
            "shell": "inner",
            "n_tets": inner_count,
            "mu_pa": inner.mu,
            "c2_pa": inner.c2,
            "lambda_pa": inner.lambda,
            "density_kg_m3": inner.density,
            "validity_max_principal_stretch": inner.validity_max_principal_stretch,
            "validity_min_principal_stretch": inner.validity_min_principal_stretch,
            "source": inner_source_tag,
            "proxy_for": "Slacker-softened Ecoflex 00-30 (effective Shore 00-20)",
        },
        {
            "name": "DRAGON_SKIN_10A",
            "shell": "middle",
            "n_tets": middle_count,
            "mu_pa": DRAGON_SKIN_10A.mu,
            "c2_pa": DRAGON_SKIN_10A.c2,
            "lambda_pa": DRAGON_SKIN_10A.lambda,
            "density_kg_m3": DRAGON_SKIN_10A.density,
            "validity_max_principal_stretch": DRAGON_SKIN_10A.validity_max_principal_stretch,
            "validity_min_principal_stretch": DRAGON_SKIN_10A.validity_min_principal_stretch,
            "source": "Anchor(DRAGON_SKIN_10A)",
            "proxy_for": "DS10A + Cu mesh + carbon black (effective Shore 15-18A)",
        },
        {
            "name": "DRAGON_SKIN_20A",
            "shell": "outer",
            "n_tets": outer_count,
            "mu_pa": DRAGON_SKIN_20A.mu,
            "c2_pa": DRAGON_SKIN_20A.c2,
            "lambda_pa": DRAGON_SKIN_20A.lambda,
            "density_kg_m3": DRAGON_SKIN_20A.density,
            "validity_max_principal_stretch": DRAGON_SKIN_20A.validity_max_principal_stretch,
            "validity_min_principal_stretch": DRAGON_SKIN_20A.validity_min_principal_stretch,
            "source": "Anchor(DRAGON_SKIN_20A)",
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
// (PLY z-slab inline scratch helpers retired at F1.5)
// =============================================================================
//
// The pre-F1.5 inline `emit_zslab_ply` + `ZslabRecord` (with
// DISPLACEMENT_SCALE = 10.0 amplification) emitted a z-slab per-tet
// centroid cloud at z = 0. Lifted to [`sim_soft::viz::boundary_surface`]
// + [`sim_soft::viz::slab_cut`] at F1.1 / F1.5 retrofit; row 23 now
// emits the full 3D body + the proper triangulated cross-section
// rather than an amplified centroid cloud. Z-slab tet-COUNT
// regression gate (`verify_zslab_counts_exact`) survives — cheap
// centroid filter, no PLY data.

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    println!("scan-fit-3layer-sleeve-yeoh-ramp — row 23 (F4.1 Yeoh consumer)");
    println!();

    // 1-5. Build initial mesh + verifies (geometry + material checks).
    let scan_solid = build_scan_solid();
    let outer_envelope = build_outer_envelope(scan_solid.clone());
    let sleeve_body = build_sleeve_body(outer_envelope.clone(), scan_solid.clone());
    let material_field = build_material_field();
    let hints = build_hints(material_field);
    let mesh = SdfMeshedTetMesh::<Yeoh>::from_sdf_yeoh(&sleeve_body, &hints)
        .map_err(|e| anyhow::anyhow!("{e:?}"))?;

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
    // 16 steps. The upstream `mesh` + `bc` are NOT moved into the
    // ramp's first solver call — the ramp rebuilds its own mesh per
    // step (same SDF, same hints, deterministic BCC + IS → bit-equal
    // mesh). The pre-F1.5 explicit `drop(mesh)` was retired here —
    // F1.5's `sim_soft::viz::{boundary_surface, slab_cut}` calls at
    // the post-ramp PLY emit step need `&dyn Mesh<Yeoh>` access, so
    // the mesh stays alive until function-end. Mesh memory at this
    // row's size is a few MB — negligible vs. the solver's per-step
    // working set.
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

    let json_path = out_dir.join("scan_fit_3layer_sleeve_yeoh_ramp.json");
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

    // PLY emits via `sim_soft::viz` public API (F1.5 retrofit; see
    // `sim/L0/soft/src/viz/mod.rs` + `project_sim_soft_viz_arc.md`).
    let final_step = results.last().expect("ramp produced no results");
    let final_data = final_step
        .final_step_data
        .as_ref()
        .expect("final ramp step missing FinalStepData");
    let half_cell = 0.5 * CELL_SIZE;

    // z-slab tet-COUNT regression gate (cheap centroid filter — no
    // PLY data accumulation; pre-F1.5 PLY-emit path retired).
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
        match shell_idx_per_tet[tet_idx] {
            0 => n_inner_z += 1,
            1 => n_middle_z += 1,
            _ => n_outer_z += 1,
        }
    }
    verify_zslab_counts_exact(n_inner_z, n_middle_z, n_outer_z);

    // Per-tet displacement magnitude across the full mesh + per-tet
    // material id (radial shell index). Both feed boundary-surface
    // and slab-cut emits via the public viz API.
    let displacement_per_tet: Vec<f64> = tets
        .iter()
        .map(|&[v0, v1, v2, v3]| {
            let rest = (final_data.rest_positions[v0 as usize]
                + final_data.rest_positions[v1 as usize]
                + final_data.rest_positions[v2 as usize]
                + final_data.rest_positions[v3 as usize])
                / 4.0;
            let deformed = (final_step.deformed_positions[v0 as usize]
                + final_step.deformed_positions[v1 as usize]
                + final_step.deformed_positions[v2 as usize]
                + final_step.deformed_positions[v3 as usize])
                / 4.0;
            (deformed - rest).norm()
        })
        .collect();
    let material_id_per_tet: Vec<f64> = shell_idx_per_tet.iter().map(|&s| s as f64).collect();
    let mut per_tet_scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    per_tet_scalars.insert("displacement_magnitude", &displacement_per_tet);
    per_tet_scalars.insert("material_id", &material_id_per_tet);
    per_tet_scalars.insert("psi_j_per_m3", &final_step.per_tet_psi);

    let bd_ply_path = out_dir.join("sleeve_boundary_final.ply");
    let bd_attr = boundary_surface(&mesh, &per_tet_scalars).map_err(|e| anyhow::anyhow!("{e}"))?;
    save_ply_attributed(&bd_attr, &bd_ply_path, true)?;

    let slab_ply_path = out_dir.join("sleeve_slab_cut_z0_final.ply");
    let slab_attr = slab_cut(
        &mesh,
        Plane {
            axis: 2,
            value: 0.0,
        },
        &per_tet_scalars,
    )
    .map_err(|e| anyhow::anyhow!("{e}"))?;
    save_ply_attributed(&slab_attr, &slab_ply_path, true)?;

    // F2.2 design-mesh emits — `design_slab_cut` (marching-squares on
    // design SDF) + `design_surface` (marching cubes on design SDF) +
    // barycentric scalar interp from the analysis tet mesh. Decouples
    // display from sim per the F2 viz arc; emits alongside the F1
    // boundary_surface + slab_cut artifacts so both conventions stay
    // available.
    let body_for_viz = {
        let scan = build_scan_solid();
        let outer = build_outer_envelope(scan.clone());
        build_sleeve_body(outer, scan)
    };
    let design_bounds = Aabb3::new(
        Vec3::new(-BBOX_HALF_X, -BBOX_HALF_Y, -BBOX_HALF_Z),
        Vec3::new(BBOX_HALF_X, BBOX_HALF_Y, BBOX_HALF_Z),
    );
    let design_resolution = CELL_SIZE / 4.0;

    let design_slab_path = out_dir.join("sleeve_design_slab_cut_z0_final.ply");
    let design_slab_attr = design_slab_cut(
        &body_for_viz,
        &mesh,
        Plane {
            axis: 2,
            value: 0.0,
        },
        &design_bounds,
        design_resolution,
        &per_tet_scalars,
    )
    .map_err(|e| anyhow::anyhow!("{e}"))?;
    save_ply_attributed(&design_slab_attr, &design_slab_path, true)?;

    let design_surface_path = out_dir.join("sleeve_design_surface_final.ply");
    let design_surface_attr = design_surface(
        &body_for_viz,
        &mesh,
        &design_bounds,
        design_resolution,
        &per_tet_scalars,
    )
    .map_err(|e| anyhow::anyhow!("{e}"))?;
    save_ply_attributed(&design_surface_attr, &design_surface_path, true)?;

    // F2.3c — per-step deformed PLY series. One design_surface_deformed
    // per ramp step (16 total). rest_positions is constant (deterministic
    // BCC + IS → bit-equal mesh per step); deformation evolves per step.
    // amplify=10 makes Yeoh's mm-scale deformations visible on the
    // ~50 mm body. Reuse `final_data.rest_positions` from the F1 emit
    // block above — same FinalStepData; no need to re-bind.
    let rest_positions = &final_data.rest_positions;
    let amplify = 10.0_f64;
    for step_result in &results {
        let step_idx = step_result.step;
        let step_displacement: Vec<Vec3> = (0..n_vertices)
            .map(|i| step_result.deformed_positions[i] - rest_positions[i])
            .collect();
        let step_displacement_per_tet: Vec<f64> = tets
            .iter()
            .map(|&[v0, v1, v2, v3]| {
                let rest = (rest_positions[v0 as usize]
                    + rest_positions[v1 as usize]
                    + rest_positions[v2 as usize]
                    + rest_positions[v3 as usize])
                    / 4.0;
                let deformed = (step_result.deformed_positions[v0 as usize]
                    + step_result.deformed_positions[v1 as usize]
                    + step_result.deformed_positions[v2 as usize]
                    + step_result.deformed_positions[v3 as usize])
                    / 4.0;
                (deformed - rest).norm()
            })
            .collect();
        let mut step_scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        step_scalars.insert("displacement_magnitude", &step_displacement_per_tet);
        step_scalars.insert("material_id", &material_id_per_tet);
        step_scalars.insert("psi_j_per_m3", &step_result.per_tet_psi);
        let step_attr = design_surface_deformed(
            &body_for_viz,
            &mesh,
            &design_bounds,
            design_resolution,
            &step_scalars,
            &step_displacement,
            amplify,
        )
        .map_err(|e| anyhow::anyhow!("{e}"))?;
        let step_path = out_dir.join(format!(
            "sleeve_design_surface_deformed_step_{step_idx:02}.ply"
        ));
        save_ply_attributed(&step_attr, &step_path, true)?;
    }

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
    println!("    inner  (Path 2: DoubleZero(20.0)→Yeoh) : {n_inner}");
    println!("    middle (DRAGON_SKIN_10A → Yeoh)        : {n_middle}");
    println!("    outer  (DRAGON_SKIN_20A → Yeoh)        : {n_outer}");
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
        "  out/scan_fit_3layer_sleeve_yeoh_ramp.json (scalars + Yeoh materials + ramp_curve + final_pairs)"
    );
    println!(
        "  out/sleeve_boundary_final.ply             (full 3D body via sim_soft::viz::boundary_surface, displacement_magnitude + material_id per-vertex)"
    );
    println!(
        "  out/sleeve_slab_cut_z0_final.ply          (cross-section at z = 0 via sim_soft::viz::slab_cut, marching-tet)"
    );
    println!(
        "  out/sleeve_design_slab_cut_z0_final.ply   (cross-section at z = 0 via sim_soft::viz::design_slab_cut, marching-squares-filled on design SDF)"
    );
    println!(
        "  out/sleeve_design_surface_final.ply       (full 3D body via sim_soft::viz::design_surface, marching-cubes on design SDF)"
    );
    println!(
        "  out/sleeve_design_surface_deformed_step_01.ply..step_12.ply  (F2.3c ramp animation series: per-step deformed body via sim_soft::viz::design_surface_deformed at amplify=10)"
    );
    println!();
    println!("View final-step PLYs in cf-view (workspace's unified visual-review viewer):");
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-ramp/out/sleeve_boundary_final.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-ramp/out/sleeve_slab_cut_z0_final.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-ramp/out/sleeve_design_slab_cut_z0_final.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-ramp/out/sleeve_design_surface_final.ply"
    );
    println!();
    println!("Optional matplotlib post-processing (force-displacement + max_disp curves):");
    println!("  uv run examples/sim-soft/scan-fit-3layer-sleeve-yeoh-ramp/plot_ramp.py");
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
