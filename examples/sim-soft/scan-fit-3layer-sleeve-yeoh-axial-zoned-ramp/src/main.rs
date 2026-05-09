// `usize as f64` casts on referenced-vertex / per-shell / per-zone /
// per-step counts for mean computations. Counts ≤ ~150 k here, well
// within f64 mantissa exact range. Same allowance as rows 6+8+9+10+11
// +16+18+20+21+22+23.
#![allow(clippy::cast_precision_loss)]
// `cast_possible_wrap` on `usize → VertexId` (u32) packing for vertex
// indices in the BCC + stuffing pipeline. Vertex counts are bounded by
// the BCC-lattice's i32-safe `n_lattice` cap inherited from
// `BccLattice::new`. Same allowance as rows 8+9+10+11+15+16+20+21+23.
#![allow(clippy::cast_possible_wrap)]
// `cast_possible_truncation` on `pid as u32` packing for primitive
// indices in the per-pair-readout walk (one rigid primitive in this
// row). Same allowance as rows 18+20+21+23.
#![allow(clippy::cast_possible_truncation)]
// `expect()` on `Matrix3::try_inverse()` for the per-tet `D_rest`
// reference shape derivative. `D_rest` is invertible iff the tet has
// positive signed volume — `verify_quality_floors` is the
// corresponding pre-condition gate. Same precedent as row 20+21+23.
#![allow(clippy::expect_used)]
// `print_summary` is a single museum-plaque stdout writer; splitting
// fragments the visual format without information gain. Same allowance
// as rows 4+5+6+9+10+11+15+16+19+20+21+23.
#![allow(clippy::too_many_lines)]
// Domain-meaningful naming pairs (`mu_proximal` / `mu_distal`,
// `outer_envelope` / `sleeve_body`, `pos_pinned` / `pos_active`)
// distinguish operand-vs-receiver across the heterogeneous-CSG and
// multi-field composition sites. Same allowance as rows 6+10+11+16+20
// +21+23.
#![allow(clippy::similar_names)]
// Math-symbol-laden physics doc-comments use formatted-text patterns
// — `Ψ̄_inner` / `Ψ̄_middle` / `Ψ̄_outer` / `Ψ̄_proximal` / `Ψ̄_distal`
// for per-shell-zone mean strain-energy density, plus references to
// SNAKE_CASE constants like AXIAL_SPLIT and AXIAL_BAND_HALF_WIDTH —
// that doc_markdown flags as "looks like an undefined identifier"
// even when the macron/subscript syntax resists clean backticking
// (Unicode combining marks confuse the heuristic). Wrapping every
// site fragments the readable typography of these formulas. Same
// readability tradeoff as row 23's verify_per_step_strain_energy_ordering
// regular `//` comment block; row 24's doc-comment-attached Ψ̄ refs
// trip the lint where row 23's plain-comment refs do not.
#![allow(clippy::doc_markdown)]

//! scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp — row 24 (F4.1+v3
//! axial-zoned variation): the first user of the `BlendedScalarField`
//! over an axial half-space SDF composed onto row 23's radial
//! `LayeredScalarField` material field. Same scan + 3-layer sleeve +
//! rigid intrusion probe geometry as
//! [row 23 `scan-fit-3layer-sleeve-yeoh-ramp`](../scan-fit-3layer-sleeve-yeoh-ramp/);
//! constitutive model (Yeoh) and ramp depth (8 mm in 16 × 0.5 mm
//! steps) carry through verbatim — the ONLY differentiator is that
//! each shell's `(μ, c2, λ)` becomes axially zoned: a "soft tip /
//! stiff anchor" stack with the row 23 anchor set in the proximal (+z,
//! contact end) zone and a one-Shore-step-stiffer sibling stack in the
//! distal (−z, anchored end) zone. See [Yeoh arc memo][arcmemo]
//! Roadmap §"v3 axial-zoned variation" for the architectural rationale
//! and [v3 spec memo][spec] for the locked design + decisions.
//!
//! [arcmemo]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_yeoh_hyperelastic_arc.md
//! [spec]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_row_24_v3_axial_zoned_spec.md
//! [mem]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_layered_silicone_device.md
//! [m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md
//! [rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md
//!
//! # Why axial zoning
//!
//! Row 23 ships the user-target 8 mm physical intrusion on a uniform
//! 3-shell radial silicone stack (proximal-only equivalent in this
//! row's terminology). Real soft-actuator designs commonly grade
//! stiffness along the long axis: the contact-end (probe-facing) is
//! softer for compliance, the anchor-end (away from contact) is
//! stiffer for structural support. This row demonstrates that
//! cf-design's [`BlendedScalarField`] (smoothstep blend over a chosen
//! SDF, between two child `Field<f64>`s) composes cleanly with
//! row 23's [`LayeredScalarField`] (SDF-thresholded radial layers) to
//! produce a 2-zone-axial × 3-shell-radial material field with **no
//! new `Field<f64>` impl required** — only an inline 5-line
//! [`Sdf`]-impl `AxialHalfSpace` for the axial blend region. The
//! composition is per-parameter independent (one `BlendedScalarField`
//! each for μ, C₂, λ) and shares the same axial SDF + band, so the
//! smoothstep weight is identical at every reference-space probe →
//! the band-zone Yeoh sample at any tet is a coherent
//! `(μ_blend, c2_blend, λ_blend)` triple, not a malformed mix of
//! parameters from different axial points.
//!
//! # Material recipe — soft tip / stiff anchor (per spec memo §"Material recipe")
//!
//! | Shell | Proximal anchor (+z, contact end) | Distal anchor (−z, anchor end) | μ contrast |
//! |---|---|---|---|
//! | Inner (φ < 6 mm)            | `ECOFLEX_00_20` (μ = 18 kPa)  | `ECOFLEX_00_30` (μ = 23 kPa)  | ×1.28 |
//! | Middle (6 mm ≤ φ < 10 mm)   | `DRAGON_SKIN_10A` (μ = 51 kPa)| `DRAGON_SKIN_15` (μ = 92 kPa) | ×1.80 |
//! | Outer (φ ≥ 10 mm)           | `DRAGON_SKIN_20A` (μ = 113 kPa)| `DRAGON_SKIN_30A` (μ = 198 kPa)| ×1.75 |
//!
//! All six anchors are F4 `pub const` Path-1 entries (no Path-2
//! parametric construction in this row — row 23 already exercises the
//! Path-2 surface; row 24 isolates the axial-composition variable).
//! Per [arc memo §D3][arcmemo] the proximal/distal pair within each
//! shell stays within the same Smooth-On family (Ecoflex inner,
//! Dragon Skin middle + outer) so the smoothstep weight in the band
//! interpolates linearly through the F4 Shore-space convention.
//!
//! Peak axial μ-gradient (at the band's interior `|z − SPLIT| <
//! BAND_HALF`) is `(μ_distal − μ_proximal) / (2 · band_half_width) ≈
//! 4.1 GPa/m` for the middle shell and `8.5 GPa/m` for the outer
//! shell — comparable to or smaller than the existing inner-middle
//! radial step (`(51 − 18) kPa / CELL_SIZE ≈ 8.25 GPa/m`), so
//! Newton's tangent stiffness landscape is no harsher than row 23's
//! already-converged regime.
//!
//! # Axial half-space
//!
//! ```rust,ignore
//! const AXIAL_SPLIT_Z: f64 = 0.0;            // body equator
//! const AXIAL_BAND_HALF_WIDTH: f64 = 0.005;  // 10 mm full smoothstep band
//!
//! struct AxialHalfSpace { split_z: f64 }
//! impl Sdf for AxialHalfSpace {
//!     fn eval(&self, p: Point3<f64>) -> f64 { p.z - self.split_z }
//!     fn grad(&self, _: Point3<f64>) -> Vector3<f64> { Vector3::new(0.0, 0.0, 1.0) }
//! }
//! ```
//!
//! Sign convention (load-bearing — get this wrong and proximal/distal
//! flip): with `phi = p.z − SPLIT_Z`, `phi < 0` means `z < SPLIT`
//! (the distal, −z half of the body). [`BlendedScalarField`]'s doc
//! says "inside_field dominates at SDF-negative samples", so
//! `inside_field = distal` and `outside_field = proximal`.
//!
//! # Pipeline
//!
//! 1. **Scan stand-in via `Solid::cuboid`** — `(SCAN_HX, SCAN_HY,
//!    SCAN_HZ) = (0.020, 0.015, 0.040) m` half-extents. Same as row
//!    23.
//!
//! 2. **Outer envelope via `Solid::offset(WRAP_THICKNESS)`** — same.
//!
//! 3. **Sleeve body via `Solid::subtract`** — same.
//!
//! 4. **`SdfMeshedTetMesh<Yeoh>` build** via PR #235's
//!    `from_sdf_yeoh` — same as row 23. Mesh is rebuilt fresh at
//!    every ramp step (deterministic BCC + IS pipeline).
//!
//! 5. **3-zone × 3-shell `MaterialField` via [`BlendedScalarField`]
//!    over [`LayeredScalarField`]**. Each of `(μ, C₂, λ)` builds two
//!    radial `LayeredScalarField`s (proximal anchor stack + distal
//!    anchor stack) then wraps them in a `BlendedScalarField` over
//!    the axial half-space SDF — see [`build_axial_zoned_material_field`].
//!    The three `BlendedScalarField`s share the same axial SDF + band,
//!    so the smoothstep weight is bit-identical at every reference-
//!    space probe.
//!
//! 6. **Quasi-static intrusion ramp** — `N_RAMP_STEPS = 16` × 0.5 mm
//!    to 8 mm (same as row 23). `MAX_NEWTON_ITER = 150` (same).
//!
//! 7. **Per-step + final-step readouts** — at every ramp step,
//!    capture the same `RampStepResult` shape as row 23 (per-shell
//!    radial Ψ̄). At the FINAL step, additionally bin per-tet Ψ
//!    by 3-zone × 3-shell (9 cells) for a single-frame snapshot of
//!    the strain-energy partition under the soft-tip / stiff-anchor
//!    pattern.
//!
//! 8. **Readouts** —
//!    - JSON `out/scan_fit_3layer_sleeve_yeoh_axial_zoned_ramp.json`:
//!      5-section schema (scalars at final step + `axial_zoning`
//!      metadata + `material_layers` nested per-shell with
//!      proximal_anchor + distal_anchor + `ramp_curve` 16-element
//!      array + `final_contact_pairs` per-pair detail at step 16
//!      only).
//!    - PLY `out/sleeve_xslab_final.ply`: x-slab per-tet centroid
//!      cloud at the FINAL step, rendered at REST positions (no
//!      displacement amplification — sim-soft viz arc option-1 baby
//!      step, see [vizarc] memory file). The body's rectangle shape
//!      is preserved; the contact-zone story shows up as a
//!      `psi_j_per_m3` strain-energy heatmap rather than as
//!      geometric explosion. PLY carries six per-vertex scalars:
//!      categorical `material_id` (radial shell), canonical
//!      `zone_id`, extra `axial_zone_id` (cf-view selector mirror
//!      — axial proximal/band/distal sharp partition), sequential
//!      `displacement_magnitude` (true physical magnitude,
//!      unscaled), sequential `mu_sampled_pa` (per-tet sampled μ
//!      — visualises the axial blend directly), and sequential
//!      `psi_j_per_m3` (per-tet strain-energy density — the FEM
//!      stress-heatmap headline). **NEW slab cut vs row 23's
//!      z-slab**: the axial material gradient is invisible in a
//!      z-slab at z = 0 because the entire z-slab sits inside the
//!      smoothstep band; an x-slab at x = 0 spans the full axial
//!      range so the soft-tip / stiff-anchor gradient is
//!      eyes-on-pixels visible. Z-slab tet counts are RETAINED as
//!      bit-equal regression gates (cheap centroid filter; not
//!      emitted as PLY).
//!    - PLY `out/sleeve_boundary_final.ply`: F1.0 SCRATCH PROTOTYPE
//!      of the [vizarc]'s tet-mesh-native `boundary_surface()`
//!      primitive — full 3D body as a triangulated boundary surface
//!      coloured by per-vertex `psi_j_per_m3` (volume-weighted
//!      averaging from per-tet psi). Replaces the x-slab's
//!      reduce-to-2D framing with the canonical FEM-viz convention
//!      (3D body, sequential heatmap, rotate to inspect). Spike
//!      crate at `examples/sim-soft/spade-delaunay-spike/` documents
//!      the falsified Delaunay-of-centroids architecture this
//!      replaces. F1.1 lifts the helper to `sim/L0/soft/src/viz/`
//!      after eyes-on-pixels passes; F1.2 retires the x-slab emit.
//!    - PLY `out/sleeve_slab_cut_x0_final.ply`: F1.3 SCRATCH
//!      PROTOTYPE of the [vizarc]'s `slab_cut()` primitive —
//!      marching-tetrahedra cross-section of the body at the x = 0
//!      plane, coloured by linearly-interpolated per-vertex
//!      `psi_j_per_m3` along cut edges. Exposes the inner cavity
//!      profile + axial proximal/band/distal strain gradient that
//!      the closed boundary-surface PLY hides; the FEM-canonical
//!      "clipping plane" view. Pulled forward from F1.3 (originally
//!      after F1.1 + F1.2) per 2026-05-10 user request to see
//!      inside the cavity-fitting wrap. F1.6 ships an open-mouth
//!      row 25 variant that gives the FEM-correct version of "see
//!      inside" (cavity has an actual physical opening through the
//!      +z face); slab_cut is the meantime view on closed-body row
//!      24.
//!
//! [vizarc]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_viz_arc.md
//!    - Optional `plot_ramp.py` (PEP 723 + matplotlib): same
//!      dual-axis depth × `force_z` + depth × `max_disp` curve as
//!      row 23. Run via `uv run plot_ramp.py`.
//!    - `verify_*` runtime gates (16 anchor groups — row 23's 12 + 4
//!      NEW: 9-cell zone × shell partition + zone × shell Ψ̄
//!      ordering + cross-zone Ψ̄ sanity + blend-zone material
//!      midplane provenance).
//!
//! # Why x-slab over z-slab
//!
//! Row 23's z-slab cut at z = 0 (the body equator) catches the
//! propagated radial response of the wrap shell — the cut is 40 mm
//! BELOW the contact zone at z ≈ +SCAN_HZ. For row 24 the equator
//! sits at the AXIAL_SPLIT, INSIDE the smoothstep band — the entire
//! z-slab samples blended-band material, which obliterates the
//! soft-tip / stiff-anchor visualisation that this row demonstrates.
//! An x-slab at x = 0 cuts perpendicular to the long axis and spans
//! the full z range, exposing the proximal-pure / band / distal-pure
//! axial structure as a 2-D centroid cloud. Per
//! `feedback_visual_review_is_the_test` (eyes-on-pixels gate after
//! the code gates on visual-emitting rows), the slab cut has to
//! serve the row's headline — row 24's headline is the axial
//! material gradient, so the cut has to expose it.
//!
//! # Sanitization
//!
//! Per the [device memo][mem]'s sanitization directive: the scanned
//! reference geometry is referred to as "scanned reference geometry"
//! or "scan stand-in" throughout this crate's prose. No anatomical
//! references appear in any tracked surface. The cuboid placeholder
//! is a parametric synthetic stand-in — the pipeline demonstration is
//! the workflow ("scan-shaped body → wrap by offset → carve cavity →
//! 3-shell × 2-zone Yeoh FEM → multi-step rigid intrusion ramp"), not
//! the cuboid's specific geometry; production runs swap the cuboid
//! for a real scan via row 15's STL-import path without any other
//! code change.
//!
//! # Run
//!
//! ```sh
//! cargo run -p example-sim-soft-scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp --release
//! ```
//!
//! Per [`feedback_release_mode_heavy_tests`][rel] — release mode is
//! required. Per-step runtime is similar to row 23 (~3-6 s per step
//! late in the ramp); 16-step total ~40-90 s release. The
//! `CELL_SIZE = 0.004 m` (4 mm) is sized so each of the 6/4/4 mm
//! layers carries at least one BCC cell across thickness; finer
//! cells (e.g., `0.002 m`) trip an SPD pivot at the FIRST ramp step
//! (empirically tested at row-22 v2-spec spike time, applies to row
//! 24 by inheritance — same mesh + meshing pipeline).
//!
//! Optional matplotlib post-processing:
//!
//! ```sh
//! uv run examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/plot_ramp.py
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
use sim_soft::material::silicone_table::{
    DRAGON_SKIN_10A, DRAGON_SKIN_15, DRAGON_SKIN_20A, DRAGON_SKIN_30A, ECOFLEX_00_20, ECOFLEX_00_30,
};
use sim_soft::{
    Aabb3, BlendedScalarField, BoundaryConditions, CpuNewtonSolver, Field, LayeredScalarField,
    Material, MaterialField, Mesh, MeshingHints, PenaltyRigidContact,
    PenaltyRigidContactYeohSolver, Sdf, SdfMeshedTetMesh, SiliconeMaterial, Solver, SolverConfig,
    Tet4, Vec3, VertexId, Yeoh, pick_vertices_by_predicate, referenced_vertices,
};

// =============================================================================
// Constants — scan stand-in geometry (axis-aligned cuboid)
// =============================================================================
//
// All geometry constants inherited verbatim from row 21 v1 / row 22 /
// row 23 — v3 keeps the cuboid + sphere geometry; the ONLY
// differentiators are the axial-zoning constants below + the material
// recipe.

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

/// BCC lattice spacing (m). 4 mm — same as rows 21/22/23. Finer cells
/// (`0.002 m`) trip SPD at the first ramp step.
const CELL_SIZE: f64 = 0.004;

// =============================================================================
// Constants — rigid intrusion probe (sphere, repositioned per step)
// =============================================================================

/// Spherical probe radius (m). 12 mm — same as rows 21/22/23.
const PROBE_RADIUS: f64 = 0.012;

// =============================================================================
// Constants — quasi-static ramp
// =============================================================================

/// Number of ramp steps. Same 16 × 0.5 mm/step = 8 mm as row 23.
const N_RAMP_STEPS: usize = 16;

/// Final probe penetration depth (m). 8 mm — same as row 23 (the
/// user-target physical intrusion).
const PROBE_PENETRATION_FINAL: f64 = 0.008;

/// Penetration delta per ramp step (m). 0.5 mm/step — same as row 23.
const RAMP_STEP_DELTA: f64 = PROBE_PENETRATION_FINAL / N_RAMP_STEPS as f64;

// =============================================================================
// Constants — solver
// =============================================================================

/// Static-regime time step per ramp step (s). At `dt = 1.0 s`, the
/// `M / Δt²` inertial term collapses ~4 orders below the stiffness
/// contribution per step; each `replay_step` from the previous step's
/// `x_final` converges to a fresh static equilibrium far below
/// `tol = 1e-10`. Same idiom as row 23 + rows 11/14/16/18/20/21/22.
const STATIC_DT: f64 = 1.0;

/// Newton iter cap per ramp step. Same 150 as row 23 — the soft-tip /
/// stiff-anchor gradient is comparable in magnitude to row 23's
/// existing radial inner-middle step (~8.25 GPa/m), so iter envelope
/// is expected within row 23's 73-iter margin at step 16. If the
/// first run trips the cap, escalation order per spec memo §"Risk
/// register": (a) cap → 200, (b) widen band 0.005 → 0.008 m, (c)
/// reduce stiffness contrast.
const MAX_NEWTON_ITER: usize = 150;

// =============================================================================
// Constants — axial zoning (NEW for row 24)
// =============================================================================

/// Axial split plane z-coordinate (m). Equator. Symmetric placement
/// puts the body's mass evenly between proximal and distal zones, so
/// the soft-tip / stiff-anchor partition is a balanced experiment
/// (37.5 % / 25 % / 37.5 % distal / band / proximal by z extent).
const AXIAL_SPLIT_Z: f64 = 0.0;

/// Smoothstep band half-width (m). 5 mm half = 10 mm full band, ~2.5
/// BCC cells across. Tighter than the per-tet stiffness gradient
/// would warrant alone — the band needs to resolve at least a few
/// BCC cells so the smoothstep blending isn't aliased onto the
/// lattice. Wider bands (e.g. 0.008 m) reduce the peak gradient
/// linearly; tighter bands (e.g. 0.003 m, ~1 cell) risk per-cell
/// step-function artefacts that the smoothstep is meant to remove.
const AXIAL_BAND_HALF_WIDTH: f64 = 0.005;

// =============================================================================
// Constants — tolerances
// =============================================================================

/// IV-1 sparse-tier rel-tol for captured per-step force / displacement
/// / per-layer Ψ̄ bits. ~75 k tets × 16 chained steps through faer's
/// sparse Cholesky lives at the IV-1 sparse-at-scale tier; `1e-12`
/// admits sparse-solver SIMD/FMA noise while catching real
/// regressions. Same precedent as rows 6+10+11+16+20+21+22+23.
const SPARSE_REL_TOL: f64 = 1.0e-12;

/// Absolute floor for the captured-bits comparison.
const SPARSE_EPS_ABS: f64 = 1.0e-12;

/// Bit-exact tolerance for the F4 const-fn `to_yeoh()` Yeoh-parameters
/// round-trip (μ + λ + C₂ + asymmetric validity bounds, per F4.0
/// arc-memo §"Implementation status").
const F4_PROVENANCE_EXACT_TOL: f64 = 0.0;

/// Probe `F = diag(1.01, 1, 1)` material-assignment-probe tolerance.
const MATERIAL_PROBE_EXACT_TOL: f64 = 0.0;

/// Bit-exact tolerance for the blend-zone midplane sample. At
/// `phi = 0` (the AXIAL_SPLIT plane) [`BlendedScalarField`]'s
/// smoothstep yields `outside_weight = 0.5` exactly per the field's
/// doc-comment; the sampled blend value is then
/// `(distal + proximal) / 2` bit-exact in IEEE 754. The 1e-15
/// tolerance is a safety margin against future smoothstep-implementation
/// refactors that might introduce sub-ULP drift; tighten to `== mean`
/// if the bit-exact midplane invariant is ever lifted to the type
/// system.
const BLEND_MIDPLANE_TOL: f64 = 1.0e-15;

// =============================================================================
// Constants — captured first-run anchor bits
// =============================================================================
//
// **Capture provenance** — captured 2026-05-10 at sim-soft `dev`
// (post row 23 ship `da1280fc`), rustc 1.95.0 (`59807616e`
// 2026-04-14) on macOS arm64 — same toolchain + platform as IV-1's
// reference capture and rows 22 + 23. First-run capture bootstrapped
// via `CF_CAPTURE_BITS=1` (pattern (cc) banked at row 19).
//
// Geometry-derived counts (`N_TETS_EXACT`, `N_VERTICES_EXACT`,
// `N_REFERENCED_EXACT`, `N_PINNED_EXACT`, the per-shell tet counts,
// the z-slab counts) are bit-equal to rows 22 + 23 — the BCC + IS
// pipeline is deterministic on the same SDF + hints; only the
// material model differs, and material doesn't affect the
// discretisation.

/// Total tet count after BCC + Isosurface Stuffing on the sleeve body.
/// Bit-equal to rows 22 + 23.
const N_TETS_EXACT: usize = 74_628;

/// Total mesh vertex count, including BCC corners not referenced by
/// any tet. Bit-equal to rows 22 + 23.
const N_VERTICES_EXACT: usize = 31_966;

/// Vertices referenced by ≥ 1 tet. Bit-equal to rows 22 + 23.
const N_REFERENCED_EXACT: usize = 17_384;

/// Outer-envelope-surface Dirichlet-pinned vertex count. Bit-equal to
/// rows 22 + 23.
const N_PINNED_EXACT: usize = 7_046;

/// Per-shell tet counts. Bit-equal to rows 22 + 23 (radial partition
/// is a centroid-vs-scan-SDF probe; unaffected by axial zoning).
const N_INNER_TETS_EXACT: usize = 25_892;
const N_MIDDLE_TETS_EXACT: usize = 16_656;
const N_OUTER_TETS_EXACT: usize = 32_080;

/// Per-shell tet counts in the `|centroid.z| < CELL_SIZE / 2 = 0.002`
/// z-slab. Bit-equal to rows 22 + 23. RETAINED as cross-row regression
/// gate even though no z-slab PLY is emitted (cheap centroid filter).
const N_INNER_TETS_ZSLAB_EXACT: usize = 768;
const N_MIDDLE_TETS_ZSLAB_EXACT: usize = 432;
const N_OUTER_TETS_ZSLAB_EXACT: usize = 892;

/// Per-zone-shell tet counts at first capture. Sharp partition at
/// `centroid.z > +AXIAL_BAND_HALF_WIDTH` ⇒ proximal,
/// `< -AXIAL_BAND_HALF_WIDTH` ⇒ distal, else BAND. NEW for row 24.
/// Sums to `N_TETS_EXACT`. NOTE: zone partition is ON THE SHARP
/// `±BAND_HALF` boundary, NOT on the smoothstep — band tets get
/// classified as "BAND" but their sampled `(μ, c2, λ)` is not an
/// arithmetic mean (the smoothstep weight varies across the band's
/// 10 mm width).
const N_DISTAL_INNER_TETS_EXACT: usize = 11_600;
const N_DISTAL_MIDDLE_TETS_EXACT: usize = 7_496;
const N_DISTAL_OUTER_TETS_EXACT: usize = 14_606;
const N_BAND_INNER_TETS_EXACT: usize = 2_572;
const N_BAND_MIDDLE_TETS_EXACT: usize = 1_558;
const N_BAND_OUTER_TETS_EXACT: usize = 2_782;
const N_PROXIMAL_INNER_TETS_EXACT: usize = 11_720;
const N_PROXIMAL_MIDDLE_TETS_EXACT: usize = 7_602;
const N_PROXIMAL_OUTER_TETS_EXACT: usize = 14_692;

/// Per-zone-shell tet counts in the `|centroid.x| < CELL_SIZE / 2 =
/// 0.002` x-slab. NEW for row 24. The x-slab is the visualisation cut
/// for the axial gradient.
const N_DISTAL_INNER_TETS_XSLAB_EXACT: usize = 618;
const N_DISTAL_MIDDLE_TETS_XSLAB_EXACT: usize = 218;
const N_DISTAL_OUTER_TETS_XSLAB_EXACT: usize = 661;
const N_BAND_INNER_TETS_XSLAB_EXACT: usize = 130;
const N_BAND_MIDDLE_TETS_XSLAB_EXACT: usize = 38;
const N_BAND_OUTER_TETS_XSLAB_EXACT: usize = 123;
const N_PROXIMAL_INNER_TETS_XSLAB_EXACT: usize = 620;
const N_PROXIMAL_MIDDLE_TETS_XSLAB_EXACT: usize = 220;
const N_PROXIMAL_OUTER_TETS_XSLAB_EXACT: usize = 664;

/// Ramp-step partition gate. Bit-pinned to `N_RAMP_STEPS = 16`.
const N_RAMP_STEPS_EXACT: usize = N_RAMP_STEPS;

/// Active contact-pair count at the FINAL ramp step (depth = 8 mm),
/// filtered to REFERENCED vertices. **Bit-equal to row 23's 50** —
/// contact happens entirely in the proximal zone, where row 24's
/// material stack matches row 23's verbatim, and distal stiffness
/// has no effect on the contact-pair count at the contact band.
const N_CONTACT_PAIRS_FINAL_EXACT: usize = 50;

/// Per-step Newton iter counts. **Bit-equal to row 23's pattern**
/// `[8, 8, 9, 10, 11, 12, 13, 15, 16, 19, 23, 27, 31, 39, 49, 77]`
/// — the soft-tip / stiff-anchor axial gradient does NOT escalate
/// Newton's path on this geometry. Expected (per spec memo §"Risk
/// register") was a modest increase since the gradient is comparable
/// to the existing radial inner-middle step; observed (per first
/// bake 2026-05-10): no change. The 73-iter margin at step 16
/// carries through.
const IT_COUNT_RAMP_EXACT: [usize; N_RAMP_STEPS] =
    [8, 8, 9, 10, 11, 12, 13, 15, 16, 19, 23, 27, 31, 39, 49, 77];

/// Per-step `+z`-component of contact reaction force bits (N), summed
/// over REFERENCED vertices only. Approximate values: `[1.10, 1.89,
/// 2.97, 4.20, 5.64, 7.35, 9.44, 11.92, 14.72, 17.89, 21.49, 25.60,
/// 30.27, 35.51, 41.79, 49.28] N`. **Final-step force = 49.28 N is
/// numerically indistinguishable from row 23's 49.28 N** at the
/// printed precision — the contact zone is in the proximal half
/// (where row 24's material stack matches row 23 verbatim), so the
/// integrated `+z` reaction at the contact band tracks row 23
/// closely. Captured bits differ from row 23 in the trailing digits
/// because the body-wide `replay_step` solution still reflects the
/// stiffer distal half via the wrap-shell coupling.
const FORCE_TOTAL_Z_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x3ff1_ae20_12bf_442d,
    0x3ffe_3702_21f9_d155,
    0x4007_b876_96db_0086,
    0x4010_ccd3_6c02_89ac,
    0x4016_8b72_88e5_533b,
    0x401d_63ce_5dd2_43d1,
    0x4022_e200_2e14_5fcf,
    0x4027_d5eb_bf0b_a139,
    0x402d_6e4f_b3da_24b1,
    0x4031_e339_c6e1_dba7,
    0x4035_7dc5_8211_f92c,
    0x4039_9ab4_d736_ff14,
    0x403e_441d_3724_b614,
    0x4041_c174_73dd_453e,
    0x4044_e567_742f_231d,
    0x4048_a442_3551_d9fc,
];

/// Per-step body-wide max displacement-magnitude bits (m). Approximate
/// values: `[1.48, 1.97, 2.47, 2.95, 3.44, 3.93, 4.41, 4.88, 5.35,
/// 5.82, 6.27, 6.72, 7.15, 7.57, 8.00, 8.45] mm`. Final-step
/// `max_disp = 8.45 mm` < `WRAP_THICKNESS = 14 mm` — the geometric
/// upper bound stays comfortable. Bit-distinguishable from row 23
/// in the trailing digits but the deformation envelope is
/// numerically indistinguishable at the printed precision (the
/// peak displacement is at the contact band in the proximal half).
const MAX_DISP_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x3f58_423e_2974_0bcc,
    0x3f60_2acc_cdfa_ffc2,
    0x3f64_30f0_cc42_81a7,
    0x3f68_33b9_3362_7ea2,
    0x3f6c_323c_b59b_dd69,
    0x3f70_158b_dcd1_6990,
    0x3f72_0d70_6f38_0683,
    0x3f73_fffc_076f_2ada,
    0x3f75_eccd_93d1_0fd9,
    0x3f77_d2b9_1eba_37ae,
    0x3f79_b05e_8cd6_e184,
    0x3f7b_8416_0e7a_5cf5,
    0x3f7d_4bcd_7286_81fb,
    0x3f7f_04a9_c340_3042,
    0x3f80_6093_1aba_d560,
    0x3f81_5097_9721_d1e5,
];

/// Per-step inner-shell mean strain-energy-density bits (J/m³).
/// Aggregated over ALL inner tets regardless of axial zone — the
/// per-step record preserves row 23's radial-only shape for
/// captured-bits continuity; the per-zone-shell breakdown lives only
/// in the FINAL step (see `MEAN_PSI_*_FINAL_BITS` below).
const MEAN_PSI_INNER_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x4011_eb66_3d81_e445,
    0x4022_b0dc_509c_eacf,
    0x4031_149b_1262_2122,
    0x403c_9a8c_2258_625a,
    0x4045_fdad_472e_22b9,
    0x404f_ca5d_63bc_cf8b,
    0x4056_0bdc_70c7_8d44,
    0x405d_a2fa_e665_81de,
    0x4063_6468_bc03_2b04,
    0x4068_c66d_0a67_0ca4,
    0x406e_ff6f_35a2_a7f3,
    0x4073_0e99_1a61_3d3d,
    0x4077_14fe_0a16_f5ea,
    0x407b_98c8_c627_d2d8,
    0x4080_3fa0_d5ff_82bf,
    0x4082_f377_94f6_3ef8,
];

/// Per-step middle-shell mean strain-energy-density bits (J/m³).
const MEAN_PSI_MIDDLE_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x3fe3_5bac_1a80_ea3b,
    0x3ff8_bfc7_a020_6236,
    0x400a_1d28_504a_aa94,
    0x4017_df74_7a85_13ba,
    0x4023_ee41_9fc2_d6b6,
    0x402f_557e_6556_5fcd,
    0x4037_7c14_4077_7509,
    0x4040_f953_141f_6f2b,
    0x4047_d252_1fda_2373,
    0x4050_4d8d_f9b3_d35d,
    0x4055_d4f3_0cf3_db2e,
    0x405c_a758_c658_8c4e,
    0x4062_7470_b82d_d583,
    0x4067_54ea_8f40_345d,
    0x406d_71f9_34f3_fccc,
    0x4072_6c0d_dbd8_e90e,
];

/// Per-step outer-shell mean strain-energy-density bits (J/m³).
const MEAN_PSI_OUTER_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [
    0x3fc0_c9cf_640d_1054,
    0x3fd6_7675_3e59_3ccb,
    0x3fe8_d782_03cf_7c37,
    0x3ff7_7836_29af_edde,
    0x4004_2ddf_c3ac_1b0d,
    0x4010_595e_b9ca_fa64,
    0x4019_52de_d8d6_9e10,
    0x4022_ec1e_be2e_39e4,
    0x402b_72a8_ac75_4686,
    0x4033_704f_55a4_0127,
    0x403a_fcab_c4ca_001d,
    0x4042_644d_d447_9594,
    0x4048_a36b_53af_d9c3,
    0x4050_345d_a92e_b47e,
    0x4055_63d1_8628_7cd3,
    0x405c_32c1_73d4_3015,
];

/// Final-step (step 16, depth = 8 mm) outer-shell max strain-energy-
/// density bits (J/m³). ≈ 49 209 J/m³, numerically indistinguishable
/// from row 23's ≈ 49 205 J/m³ at the printed precision (peak Ψ_outer
/// is in the contact-band-adjacent outer-shell tets, all in the
/// proximal half).
const MAX_PSI_OUTER_FINAL_REF_BITS: u64 = 0x40e8_069e_4bb1_6b3a;

/// Final-step per-zone-shell mean Ψ bits (J/m³). 9 cells: 3 zones ×
/// 3 shells. The captured ordering matches expectation (per spec
/// memo §"Anchor strategy"): per-shell radial preserved
/// (`Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` at any zone) AND per-zone
/// (`Ψ̄_proximal > Ψ̄_band > Ψ̄_distal` at any shell — contact is
/// at the proximal end + softer material there → strain concentrates
/// proximally; the band's blended material sits between by
/// construction). Approximate values (J/m³, three sig figs from the
/// `print_summary` post-run table):
///
/// | Zone     | inner    | middle   | outer    |
/// |----------|----------|----------|----------|
/// | distal   | 6.84e-4  | 5.03e-4  | 1.80e-4  |
/// | band     | 1.93e-2  | 1.52e-2  | 5.17e-3  |
/// | proximal | 1.34e3   | 6.46e2   | 2.46e2   |
///
/// Proximal Ψ̄ values are roughly 2× row 23's per-shell body-averaged
/// means (proximal-only inner ≈ 1340 vs row 23's body-averaged
/// inner ≈ 606) — concentration is what's expected when the strain
/// field is confined to the proximal half by the stiffer distal
/// anchor. Distal Ψ̄ is six orders smaller — the distal half barely
/// deforms at all under proximal contact loading.
const MEAN_PSI_DISTAL_INNER_FINAL_BITS: u64 = 0x3f46_6547_d048_6414;
const MEAN_PSI_DISTAL_MIDDLE_FINAL_BITS: u64 = 0x3f40_7be5_dcd4_e36e;
const MEAN_PSI_DISTAL_OUTER_FINAL_BITS: u64 = 0x3f27_8685_9ca2_410c;
const MEAN_PSI_BAND_INNER_FINAL_BITS: u64 = 0x3f93_cd59_08be_c1a5;
const MEAN_PSI_BAND_MIDDLE_FINAL_BITS: u64 = 0x3f8f_18cd_bcd2_5889;
const MEAN_PSI_BAND_OUTER_FINAL_BITS: u64 = 0x3f75_2a60_5392_aec9;
const MEAN_PSI_PROXIMAL_INNER_FINAL_BITS: u64 = 0x4094_eef2_83ca_5ab2;
const MEAN_PSI_PROXIMAL_MIDDLE_FINAL_BITS: u64 = 0x4084_2e69_fd95_1e19;
const MEAN_PSI_PROXIMAL_OUTER_FINAL_BITS: u64 = 0x406e_c90b_3d8d_8c9b;

// =============================================================================
// Axial half-space SDF — the v3 differentiator
// =============================================================================

/// Half-space SDF whose zero set is the plane `z = split_z`. Used as
/// the blend SDF for [`BlendedScalarField`] in
/// [`build_axial_zoned_material_field`]. Five-line inline impl is
/// cleaner than a [`Solid::cuboid`]-based clipped-box approximation;
/// the half-space is exactly what the smoothstep needs.
///
/// Sign convention: `phi = p.z − split_z`. Distal (−z) half has
/// `phi < 0`; proximal (+z) half has `phi > 0`. With `BlendedScalarField`
/// reading `inside_field` at `phi < 0` (per its doc-comment), the
/// distal stack passes as `inside_field` and the proximal stack as
/// `outside_field`.
struct AxialHalfSpace {
    split_z: f64,
}

impl Sdf for AxialHalfSpace {
    fn eval(&self, p: Point3<f64>) -> f64 {
        p.z - self.split_z
    }

    fn grad(&self, _p: Point3<f64>) -> Vector3<f64> {
        Vector3::new(0.0, 0.0, 1.0)
    }
}

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

/// Probe `Solid` at the given penetration depth (m). At `depth =
/// 0.001 m` (1 mm) this matches row 21 v1's static fit-pose probe
/// verbatim; the ramp loop calls this at progressively deeper depths
/// up to `PROBE_PENETRATION_FINAL = 0.008 m`.
fn build_probe_solid_at_depth(depth: f64) -> Solid {
    let probe_z = SCAN_HZ - PROBE_RADIUS + depth;
    Solid::sphere(PROBE_RADIUS).translate(Vector3::new(0.0, 0.0, probe_z))
}

// =============================================================================
// 3-shell × 2-zone Yeoh MaterialField — the v3 differentiator
// =============================================================================

/// Build the axial-zoned 3-shell Yeoh `MaterialField`. Per-parameter
/// composition: each of `(μ, C₂, λ)` builds two radial
/// [`LayeredScalarField`]s (proximal + distal anchor stacks), then
/// wraps them in a [`BlendedScalarField`] over the shared
/// [`AxialHalfSpace`] with `AXIAL_BAND_HALF_WIDTH` smoothstep band.
/// All three blends share the same axial SDF + band, so the
/// smoothstep weight is bit-identical at every reference-space probe
/// → the band-zone Yeoh sample at any tet is a coherent
/// `(μ_blend, c2_blend, λ_blend)` triple.
///
/// Sign convention: `inside_field = distal_layered`,
/// `outside_field = proximal_layered` (per [`AxialHalfSpace`]'s
/// doc-comment — `phi < 0` ⇔ distal half).
fn build_axial_zoned_material_field() -> MaterialField {
    let axial = || {
        Box::new(AxialHalfSpace {
            split_z: AXIAL_SPLIT_Z,
        }) as Box<dyn Sdf>
    };
    let scan_for_partition = || Box::new(build_scan_solid()) as Box<dyn Sdf>;
    let band = AXIAL_BAND_HALF_WIDTH;

    // μ — three layered fields per zone, blended axially.
    let mu_proximal: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![ECOFLEX_00_20.mu, DRAGON_SKIN_10A.mu, DRAGON_SKIN_20A.mu],
    ));
    let mu_distal: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![ECOFLEX_00_30.mu, DRAGON_SKIN_15.mu, DRAGON_SKIN_30A.mu],
    ));
    let mu_field: Box<dyn Field<f64>> = Box::new(BlendedScalarField::new(
        axial(),
        mu_distal,
        mu_proximal,
        band,
    ));

    // C₂ — analogous.
    let c2_proximal: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![ECOFLEX_00_20.c2, DRAGON_SKIN_10A.c2, DRAGON_SKIN_20A.c2],
    ));
    let c2_distal: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![ECOFLEX_00_30.c2, DRAGON_SKIN_15.c2, DRAGON_SKIN_30A.c2],
    ));
    let c2_field: Box<dyn Field<f64>> = Box::new(BlendedScalarField::new(
        axial(),
        c2_distal,
        c2_proximal,
        band,
    ));

    // λ — analogous.
    let lambda_proximal: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![
            ECOFLEX_00_20.lambda,
            DRAGON_SKIN_10A.lambda,
            DRAGON_SKIN_20A.lambda,
        ],
    ));
    let lambda_distal: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![
            ECOFLEX_00_30.lambda,
            DRAGON_SKIN_15.lambda,
            DRAGON_SKIN_30A.lambda,
        ],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(BlendedScalarField::new(
        axial(),
        lambda_distal,
        lambda_proximal,
        band,
    ));

    MaterialField::from_yeoh_fields(mu_field, c2_field, lambda_field)
}

// =============================================================================
// Per-tet shell + zone classification
// =============================================================================

/// Open-left, closed-right boundary convention to match
/// [`LayeredScalarField`]'s `partition_point(|&t| t <= phi)` behavior
/// bit-equally. Same as rows 21 / 22 / 23.
fn shell_at_phi(phi: f64) -> usize {
    if phi < LAYER_INNER {
        0
    } else if phi < LAYER_MIDDLE_OUTER {
        1
    } else {
        2
    }
}

/// Sharp axial-zone classification at the BAND_HALF boundaries.
/// Distinct from the smoothstep-driven material sampling — this is
/// for the per-zone-shell tet count partition + final-step Ψ̄
/// aggregation.
///
/// `0 = distal (-z)`, `1 = band (transition)`, `2 = proximal (+z)`.
fn zone_at_z(z: f64) -> usize {
    if z < AXIAL_SPLIT_Z - AXIAL_BAND_HALF_WIDTH {
        0
    } else if z < AXIAL_SPLIT_Z + AXIAL_BAND_HALF_WIDTH {
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
// Per-tet deformation gradient (same as rows 21 / 22 / 23)
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
    /// Step 16 only: the per-pair readouts for JSON
    /// `final_contact_pairs` and the final-step PLY x-slab + the
    /// 9-cell zone × shell mean Ψ partition (intermediate steps drop
    /// these to keep memory bounded).
    final_step_data: Option<FinalStepData>,
}

#[derive(Clone, Debug)]
struct FinalStepData {
    rest_positions: Vec<Vec3>,
    deformed_positions: Vec<Vec3>,
    pair_records: Vec<Value>,
    /// 3 zones × 3 shells = 9 cells, indexed `mean_psi_zone_shell[z][s]`
    /// where `z = 0/1/2` is distal/band/proximal and `s = 0/1/2` is
    /// inner/middle/outer.
    mean_psi_zone_shell: [[f64; 3]; 3],
    /// Per-tet strain-energy density (J/m³), indexed by tet ID, length =
    /// `n_tets`. Persisted at the final step for the x-slab PLY's
    /// `psi_j_per_m3` heatmap scalar (sim-soft viz arc option 1 baby
    /// step — see [memo] for the gold-standard option 3 follow-up).
    /// At intermediate steps this is dropped to keep memory bounded.
    /// Same `Material::energy(F_t)` per-tet computation as the radial
    /// and zone-shell aggregations; banking each value into the
    /// vector is a per-tet store with cap-allocate at the start,
    /// not a separate pass.
    ///
    /// [memo]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_viz_arc.md
    per_tet_psi: Vec<f64>,
}

// =============================================================================
// solve_ramp — chained replay_step over N_RAMP_STEPS
// =============================================================================

#[allow(clippy::too_many_arguments)]
fn solve_ramp(
    n_vertices: usize,
    referenced: &[VertexId],
    tets: &[[VertexId; 4]],
    shell_idx_per_tet: &[usize],
    zone_idx_per_tet: &[usize],
    n_inner: usize,
    n_middle: usize,
    n_outer: usize,
    n_zone_shell: [[usize; 3]; 3],
) -> Result<Vec<RampStepResult>> {
    let n_dof = 3 * n_vertices;

    // v2.5 anchor cleanup (2026-05-08): each per-step readout is
    // filtered to referenced (= solver-active) vertices via
    // `sim_soft::filter_pair_readouts_to_referenced` — see pattern
    // (xx) at row 22 patterns memo.

    // Initial x_prev = rest positions (from a one-shot mesh build).
    let initial_mesh = {
        let scan = build_scan_solid();
        let outer = build_outer_envelope(scan.clone());
        let body = build_sleeve_body(outer, scan);
        let hints = build_hints(build_axial_zoned_material_field());
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

        // Per-step progress log per row 23 pattern (bbb).
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
        let hints_k = build_hints(build_axial_zoned_material_field());
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
            let inspection_hints = build_hints(build_axial_zoned_material_field());
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

        // Per-tet Ψ aggregates — radial-only for the per-step record
        // (preserves row 23's RampStepResult shape; zone breakdown +
        // per-tet vector happen at FINAL step only).
        let materials = inspection_mesh.materials();
        let mut sum_in = 0.0;
        let mut sum_mi = 0.0;
        let mut sum_ou = 0.0;
        let mut max_psi_outer = f64::NEG_INFINITY;
        // Zone × shell sums + per-tet ψ vector populated only at the
        // final step.
        let is_final = k == N_RAMP_STEPS - 1;
        let mut sum_zs: [[f64; 3]; 3] = [[0.0; 3]; 3];
        let mut per_tet_psi_final: Vec<f64> = if is_final {
            Vec::with_capacity(tets.len())
        } else {
            Vec::new()
        };
        for (t, &verts) in tets.iter().enumerate() {
            let f = deformation_gradient(verts, &rest_pos_k, &positions_k);
            let psi_t = materials[t].energy(&f);
            let s = shell_idx_per_tet[t];
            match s {
                0 => sum_in += psi_t,
                1 => sum_mi += psi_t,
                _ => {
                    sum_ou += psi_t;
                    if psi_t > max_psi_outer {
                        max_psi_outer = psi_t;
                    }
                }
            }
            if is_final {
                let z = zone_idx_per_tet[t];
                sum_zs[z][s] += psi_t;
                per_tet_psi_final.push(psi_t);
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

        // Final step: compute zone × shell means + capture per-pair
        // detail for JSON + PLY emit.
        let final_step_data = if is_final {
            let mut mean_psi_zone_shell: [[f64; 3]; 3] = [[0.0; 3]; 3];
            for z in 0..3 {
                for s in 0..3 {
                    let n = n_zone_shell[z][s];
                    mean_psi_zone_shell[z][s] = if n == 0 { 0.0 } else { sum_zs[z][s] / n as f64 };
                }
            }
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
                mean_psi_zone_shell,
                per_tet_psi: per_tet_psi_final,
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

        eprintln!(
            "             iter={:>3}  residual={:.2e}  force_z={:+.3e} N  max_disp={:.3e} m  n_pairs={}",
            step_k.iter_count,
            step_k.final_residual_norm,
            force_total_z,
            max_disp_m,
            n_active_pairs,
        );

        x_prev_flat.clone_from(&step_k.x_final);
    }

    Ok(results)
}

// =============================================================================
// Verifications — 16 anchor groups (12 carry-through + 4 NEW for v3)
// =============================================================================

fn capturing_bits() -> bool {
    std::env::var("CF_CAPTURE_BITS").is_ok()
}

#[allow(clippy::too_many_arguments)]
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

/// NEW for row 24: 9-cell zone × shell partition sums to total.
fn verify_zone_shell_counts_exact(n_zone_shell: [[usize; 3]; 3]) {
    if capturing_bits() {
        let labels = [
            ("DISTAL_INNER", 0, 0),
            ("DISTAL_MIDDLE", 0, 1),
            ("DISTAL_OUTER", 0, 2),
            ("BAND_INNER", 1, 0),
            ("BAND_MIDDLE", 1, 1),
            ("BAND_OUTER", 1, 2),
            ("PROXIMAL_INNER", 2, 0),
            ("PROXIMAL_MIDDLE", 2, 1),
            ("PROXIMAL_OUTER", 2, 2),
        ];
        for (name, z, s) in labels {
            eprintln!("const N_{name}_TETS_EXACT: usize = {};", n_zone_shell[z][s]);
        }
        return;
    }
    let expected = [
        [
            N_DISTAL_INNER_TETS_EXACT,
            N_DISTAL_MIDDLE_TETS_EXACT,
            N_DISTAL_OUTER_TETS_EXACT,
        ],
        [
            N_BAND_INNER_TETS_EXACT,
            N_BAND_MIDDLE_TETS_EXACT,
            N_BAND_OUTER_TETS_EXACT,
        ],
        [
            N_PROXIMAL_INNER_TETS_EXACT,
            N_PROXIMAL_MIDDLE_TETS_EXACT,
            N_PROXIMAL_OUTER_TETS_EXACT,
        ],
    ];
    let mut total = 0usize;
    for z in 0..3 {
        for s in 0..3 {
            assert_eq!(n_zone_shell[z][s], expected[z][s], "n_zone_shell[{z}][{s}]");
            total += n_zone_shell[z][s];
        }
    }
    assert_eq!(
        total, N_TETS_EXACT,
        "zone × shell partition sums to N_TETS_EXACT",
    );
}

/// NEW for row 24: 9-cell zone × shell partition for the x-slab cut.
fn verify_xslab_zone_shell_counts_exact(n_zone_shell_xslab: [[usize; 3]; 3]) {
    if capturing_bits() {
        let labels = [
            ("DISTAL_INNER", 0, 0),
            ("DISTAL_MIDDLE", 0, 1),
            ("DISTAL_OUTER", 0, 2),
            ("BAND_INNER", 1, 0),
            ("BAND_MIDDLE", 1, 1),
            ("BAND_OUTER", 1, 2),
            ("PROXIMAL_INNER", 2, 0),
            ("PROXIMAL_MIDDLE", 2, 1),
            ("PROXIMAL_OUTER", 2, 2),
        ];
        for (name, z, s) in labels {
            eprintln!(
                "const N_{name}_TETS_XSLAB_EXACT: usize = {};",
                n_zone_shell_xslab[z][s],
            );
        }
        return;
    }
    let expected = [
        [
            N_DISTAL_INNER_TETS_XSLAB_EXACT,
            N_DISTAL_MIDDLE_TETS_XSLAB_EXACT,
            N_DISTAL_OUTER_TETS_XSLAB_EXACT,
        ],
        [
            N_BAND_INNER_TETS_XSLAB_EXACT,
            N_BAND_MIDDLE_TETS_XSLAB_EXACT,
            N_BAND_OUTER_TETS_XSLAB_EXACT,
        ],
        [
            N_PROXIMAL_INNER_TETS_XSLAB_EXACT,
            N_PROXIMAL_MIDDLE_TETS_XSLAB_EXACT,
            N_PROXIMAL_OUTER_TETS_XSLAB_EXACT,
        ],
    ];
    for z in 0..3 {
        for s in 0..3 {
            assert_eq!(
                n_zone_shell_xslab[z][s], expected[z][s],
                "n_zone_shell_xslab[{z}][{s}]",
            );
        }
    }
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
    // Same convention as row 23: post-v2.5 referenced-only filtering;
    // probe enters from above pushing wrap-cap material in `+z`.
    // Strict-adjacent monotonicity at step 1 → step 2 NOT required;
    // strict from step 2 onward.
    let first = &results[0];
    let last = results.last().expect("ramp produced no results");
    assert!(
        last.force_total_z_n > first.force_total_z_n,
        "ramp endpoint sanity: final step force_z {:e} N not greater than first step {:e} N",
        last.force_total_z_n,
        first.force_total_z_n,
    );

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
    // Same `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` ordering as row 23 — radial
    // partition is unaffected by axial zoning (the per-shell mean
    // averages over both proximal + distal + band tets within the
    // shell).
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

/// Verify that all 6 anchors used in the proximal + distal stacks
/// produce the expected Yeoh (μ, λ, c2) tuple via `to_yeoh()` and that
/// the additive decomposition `nh_part + C₂(I₁−3)²` holds bit-exactly
/// at a probe `F = diag(1.01, 1, 1)`. Carries row 23's
/// `verify_material_provenance` shape over the row 24 6-anchor set.
fn verify_material_provenance() {
    let anchors: [&SiliconeMaterial; 6] = [
        &ECOFLEX_00_20,
        &ECOFLEX_00_30,
        &DRAGON_SKIN_10A,
        &DRAGON_SKIN_15,
        &DRAGON_SKIN_20A,
        &DRAGON_SKIN_30A,
    ];
    for mat in anchors {
        let yr = mat.to_yeoh();
        let id = Matrix3::<f64>::identity();
        assert_relative_eq!(yr.energy(&id), 0.0, epsilon = F4_PROVENANCE_EXACT_TOL);

        let mut f = Matrix3::<f64>::identity();
        f[(0, 0)] = 1.01;
        // Per arc-memo F1 Spike-1 finding (mirrored from row 23
        // `verify_material_provenance`): bit-exact additive
        // decomposition.
        let i1 = 1.01_f64.mul_add(1.01, 2.0);
        let j_ln = 1.01_f64.ln();
        let half_mu = 0.5 * mat.mu;
        let half_lambda = 0.5 * mat.lambda;
        let nh_part = half_lambda.mul_add(j_ln * j_ln, half_mu.mul_add(i1 - 3.0, -mat.mu * j_ln));
        let i1m3 = i1 - 3.0;
        // Folding `c2 * i1m3` into a mul_add would change rounding
        // and break bit-equality with `Yeoh::energy`.
        #[allow(clippy::suboptimal_flops)]
        let expected = nh_part + mat.c2 * i1m3 * i1m3;
        assert_relative_eq!(yr.energy(&f), expected, epsilon = F4_PROVENANCE_EXACT_TOL,);
    }
}

/// Verify per-tet material assignment across BOTH the 3-shell radial
/// partition AND the proximal-pure / distal-pure axial regions. In
/// the band zone the sampled material is a smoothstep interpolation
/// — band-tet assignment is checked separately via
/// [`verify_blend_zone_material_provenance`] at the midplane sample
/// where the smoothstep weight is bit-exactly 0.5.
///
/// Tets in the proximal-pure region (centroid.z >
/// AXIAL_SPLIT + AXIAL_BAND_HALF_WIDTH) sample the proximal anchor
/// per shell; tets in the distal-pure region (centroid.z <
/// AXIAL_SPLIT - AXIAL_BAND_HALF_WIDTH) sample the distal anchor per
/// shell. Band tets are skipped here.
fn verify_material_assignment_partition(
    mesh: &SdfMeshedTetMesh<Yeoh>,
    shell_idx_per_tet: &[usize],
    zone_idx_per_tet: &[usize],
) {
    let materials = mesh.materials();
    assert_eq!(
        materials.len(),
        shell_idx_per_tet.len(),
        "materials() length does not match per-tet shell-classification length",
    );
    assert_eq!(
        materials.len(),
        zone_idx_per_tet.len(),
        "materials() length does not match per-tet zone-classification length",
    );
    let mut f = Matrix3::<f64>::identity();
    f[(0, 0)] = 1.01;

    let proximal_yeoh = [
        ECOFLEX_00_20.to_yeoh(),
        DRAGON_SKIN_10A.to_yeoh(),
        DRAGON_SKIN_20A.to_yeoh(),
    ];
    let distal_yeoh = [
        ECOFLEX_00_30.to_yeoh(),
        DRAGON_SKIN_15.to_yeoh(),
        DRAGON_SKIN_30A.to_yeoh(),
    ];

    let mut n_proximal_checked = 0usize;
    let mut n_distal_checked = 0usize;
    for (t, (&shell_idx, &zone_idx)) in shell_idx_per_tet
        .iter()
        .zip(zone_idx_per_tet.iter())
        .enumerate()
    {
        let observed = materials[t].energy(&f);
        match zone_idx {
            // Distal-pure
            0 => {
                let expected = distal_yeoh[shell_idx].energy(&f);
                assert!(
                    (observed - expected).abs() <= MATERIAL_PROBE_EXACT_TOL,
                    "tet {t} distal shell {shell_idx}: observed energy {observed} != expected {expected}",
                );
                n_distal_checked += 1;
            }
            // Band — sampled material varies with the smoothstep;
            // skip exact-match check here, see
            // `verify_blend_zone_material_provenance`.
            1 => {}
            // Proximal-pure
            _ => {
                let expected = proximal_yeoh[shell_idx].energy(&f);
                assert!(
                    (observed - expected).abs() <= MATERIAL_PROBE_EXACT_TOL,
                    "tet {t} proximal shell {shell_idx}: observed energy {observed} != expected {expected}",
                );
                n_proximal_checked += 1;
            }
        }
    }
    assert!(
        n_proximal_checked > 0,
        "no proximal-pure tets encountered — axial zoning check is vacuous",
    );
    assert!(
        n_distal_checked > 0,
        "no distal-pure tets encountered — axial zoning check is vacuous",
    );
}

/// NEW for row 24: at a tet centroid lying on the AXIAL_SPLIT plane,
/// the sampled material is the bit-exact arithmetic mean of the
/// proximal and distal anchors per shell. BlendedScalarField's
/// smoothstep at `phi == 0` yields `outside_weight = 0.5` exactly per
/// the doc-comment, so `(distal + proximal) / 2` is bit-exact in
/// IEEE 754. Pin one tet centroid CLOSEST to z = AXIAL_SPLIT in each
/// shell and verify the sampled energy matches the computed mean
/// energy.
fn verify_blend_zone_material_provenance(
    mesh: &SdfMeshedTetMesh<Yeoh>,
    tets: &[[VertexId; 4]],
    rest_positions: &[Vec3],
    shell_idx_per_tet: &[usize],
) {
    let materials = mesh.materials();
    let mut f = Matrix3::<f64>::identity();
    f[(0, 0)] = 1.01;

    // For each shell, find the tet whose centroid is closest to the
    // AXIAL_SPLIT plane.
    let mut closest_per_shell: [Option<(usize, f64)>; 3] = [None; 3];
    for (t, &[v0, v1, v2, v3]) in tets.iter().enumerate() {
        let centroid = (rest_positions[v0 as usize]
            + rest_positions[v1 as usize]
            + rest_positions[v2 as usize]
            + rest_positions[v3 as usize])
            / 4.0;
        let distance_from_split = (centroid.z - AXIAL_SPLIT_Z).abs();
        let s = shell_idx_per_tet[t];
        match closest_per_shell[s] {
            None => closest_per_shell[s] = Some((t, distance_from_split)),
            Some((_, d_prev)) if distance_from_split < d_prev => {
                closest_per_shell[s] = Some((t, distance_from_split));
            }
            Some(_) => {}
        }
    }

    let mu_pairs = [
        (ECOFLEX_00_20.mu, ECOFLEX_00_30.mu),
        (DRAGON_SKIN_10A.mu, DRAGON_SKIN_15.mu),
        (DRAGON_SKIN_20A.mu, DRAGON_SKIN_30A.mu),
    ];
    let c2_pairs = [
        (ECOFLEX_00_20.c2, ECOFLEX_00_30.c2),
        (DRAGON_SKIN_10A.c2, DRAGON_SKIN_15.c2),
        (DRAGON_SKIN_20A.c2, DRAGON_SKIN_30A.c2),
    ];
    let lambda_pairs = [
        (ECOFLEX_00_20.lambda, ECOFLEX_00_30.lambda),
        (DRAGON_SKIN_10A.lambda, DRAGON_SKIN_15.lambda),
        (DRAGON_SKIN_20A.lambda, DRAGON_SKIN_30A.lambda),
    ];

    for (s, slot) in closest_per_shell.iter().enumerate() {
        let (t, d) = slot.expect("every shell has at least one tet");
        let centroid = {
            let [v0, v1, v2, v3] = tets[t];
            (rest_positions[v0 as usize]
                + rest_positions[v1 as usize]
                + rest_positions[v2 as usize]
                + rest_positions[v3 as usize])
                / 4.0
        };
        // The check is meaningful only if the centroid is well inside
        // the band — outside the band the smoothstep weight isn't 0.5.
        // BCC + IS produces tets whose centroids are at ~CELL_SIZE/4
        // off-grid, so the closest tet to z = 0 sits within
        // CELL_SIZE/2 of the split plane. AXIAL_BAND_HALF_WIDTH = 5 mm
        // = 1.25 × CELL_SIZE, so the closest tet IS in the band.
        assert!(
            d < AXIAL_BAND_HALF_WIDTH,
            "shell {s}: closest tet centroid distance from split = {d} m exceeds band half-width {AXIAL_BAND_HALF_WIDTH} m — band is too tight relative to BCC cell",
        );

        // Build the expected midplane Yeoh from the per-shell anchor
        // pair, with each parameter as the IEEE-754-exact arithmetic
        // mean of proximal and distal anchors.
        let (mu_prox, mu_dist) = mu_pairs[s];
        let (c2_prox, c2_dist) = c2_pairs[s];
        let (lambda_prox, lambda_dist) = lambda_pairs[s];
        // The midplane sampled values reflect the tet's actual
        // centroid z (not exactly z = 0) — compute the smoothstep
        // weight for that centroid and use it to interpolate. At
        // |z − SPLIT| << BAND_HALF the weight is approximately 0.5
        // but not bit-exactly so for a non-zero centroid offset.
        // Mirror BlendedScalarField::sample's FMA pattern at
        // `field/layered.rs:213-227` exactly so the comparison stays
        // bit-tight: `outside_weight = s² · (3 − 2s)` via
        // `mul_add(-s, 3)`, then the lerp via
        // `(1 − w).mul_add(inside, w · outside)`.
        let phi = centroid.z - AXIAL_SPLIT_Z;
        let s_norm =
            ((phi + AXIAL_BAND_HALF_WIDTH) / (2.0 * AXIAL_BAND_HALF_WIDTH)).clamp(0.0, 1.0);
        let w_outside = s_norm * s_norm * 2.0_f64.mul_add(-s_norm, 3.0);
        // BlendedScalarField: `inside_field = distal`, `outside_field
        // = proximal`. At phi > 0 (proximal half), w_outside →
        // proximal weight; at phi < 0 (distal half), w_outside → 0
        // and the distal value dominates.
        let mu_blend = (1.0 - w_outside).mul_add(mu_dist, w_outside * mu_prox);
        let c2_blend = (1.0 - w_outside).mul_add(c2_dist, w_outside * c2_prox);
        let lambda_blend = (1.0 - w_outside).mul_add(lambda_dist, w_outside * lambda_prox);
        let expected_yeoh = Yeoh::from_lame_and_c2(mu_blend, lambda_blend, c2_blend);
        let expected_energy = expected_yeoh.energy(&f);
        let observed = materials[t].energy(&f);
        assert_relative_eq!(
            observed,
            expected_energy,
            epsilon = BLEND_MIDPLANE_TOL,
            max_relative = BLEND_MIDPLANE_TOL,
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

/// NEW for row 24: per-zone-shell mean Ψ̄ at the FINAL step. 9 cells.
/// Two ordering gates beyond the bit-pin:
/// (a) Per-zone radial: Ψ̄_inner > Ψ̄_middle > Ψ̄_outer (within zone).
/// (b) Per-shell axial: Ψ̄_proximal > Ψ̄_distal (within shell, at
///     the deep penetration). Contact is at the proximal end + softer
///     material there → strain concentrates proximally; the band's
///     blended material sits between by construction.
fn verify_zone_shell_psi_final(results: &[RampStepResult]) {
    let final_step = results.last().expect("ramp produced no results");
    let data = final_step
        .final_step_data
        .as_ref()
        .expect("final step missing FinalStepData");
    let psi = &data.mean_psi_zone_shell;

    if capturing_bits() {
        let labels = [
            ("DISTAL_INNER", 0, 0),
            ("DISTAL_MIDDLE", 0, 1),
            ("DISTAL_OUTER", 0, 2),
            ("BAND_INNER", 1, 0),
            ("BAND_MIDDLE", 1, 1),
            ("BAND_OUTER", 1, 2),
            ("PROXIMAL_INNER", 2, 0),
            ("PROXIMAL_MIDDLE", 2, 1),
            ("PROXIMAL_OUTER", 2, 2),
        ];
        for (name, z, s) in labels {
            eprintln!(
                "const MEAN_PSI_{name}_FINAL_BITS: u64 = 0x{:016x};",
                psi[z][s].to_bits(),
            );
        }
        return;
    }

    let expected = [
        [
            MEAN_PSI_DISTAL_INNER_FINAL_BITS,
            MEAN_PSI_DISTAL_MIDDLE_FINAL_BITS,
            MEAN_PSI_DISTAL_OUTER_FINAL_BITS,
        ],
        [
            MEAN_PSI_BAND_INNER_FINAL_BITS,
            MEAN_PSI_BAND_MIDDLE_FINAL_BITS,
            MEAN_PSI_BAND_OUTER_FINAL_BITS,
        ],
        [
            MEAN_PSI_PROXIMAL_INNER_FINAL_BITS,
            MEAN_PSI_PROXIMAL_MIDDLE_FINAL_BITS,
            MEAN_PSI_PROXIMAL_OUTER_FINAL_BITS,
        ],
    ];
    for z in 0..3 {
        for s in 0..3 {
            assert_relative_eq!(
                psi[z][s],
                f64::from_bits(expected[z][s]),
                max_relative = SPARSE_REL_TOL,
                epsilon = SPARSE_EPS_ABS,
            );
        }
    }

    // Per-zone radial ordering: `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer`.
    for (z, row) in psi.iter().enumerate() {
        let zone_name = match z {
            0 => "distal",
            1 => "band",
            _ => "proximal",
        };
        assert!(
            row[0] > row[1],
            "zone {zone_name}: Ψ̄_inner ({:e}) ≯ Ψ̄_middle ({:e})",
            row[0],
            row[1],
        );
        assert!(
            row[1] > row[2],
            "zone {zone_name}: Ψ̄_middle ({:e}) ≯ Ψ̄_outer ({:e})",
            row[1],
            row[2],
        );
    }

    // Per-shell axial ordering: `Ψ̄_proximal > Ψ̄_distal` at the
    // final step. Band sits between by construction (smoothstep
    // mixing assigns continuous material parameters; the strain
    // field is also continuous).
    let proximal_row = &psi[2];
    let distal_row = &psi[0];
    for (s, (&prox, &dist)) in proximal_row.iter().zip(distal_row.iter()).enumerate() {
        let shell_name = match s {
            0 => "inner",
            1 => "middle",
            _ => "outer",
        };
        assert!(
            prox > dist,
            "shell {shell_name}: Ψ̄_proximal ({prox:e}) ≯ Ψ̄_distal ({dist:e}) — contact is at the proximal end with softer material; strain should concentrate proximally",
        );
    }
}

// =============================================================================
// JSON readout
// =============================================================================

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn write_json_readout(
    path: &Path,
    n_tets: usize,
    n_vertices: usize,
    n_referenced: usize,
    n_pinned: usize,
    inner_count: usize,
    middle_count: usize,
    outer_count: usize,
    n_zone_shell: [[usize; 3]; 3],
    results: &[RampStepResult],
) -> Result<()> {
    let final_step = results.last().expect("ramp produced no results");
    let final_data = final_step
        .final_step_data
        .as_ref()
        .expect("final step missing FinalStepData");
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

    let axial_zoning = json!({
        "axial_split_z_m": AXIAL_SPLIT_Z,
        "axial_band_half_width_m": AXIAL_BAND_HALF_WIDTH,
        "blend_field": "BlendedScalarField (cubic Hermite smoothstep, C¹ at band edges, bit-exact 0.5 at midplane)",
        "blend_sdf": "AxialHalfSpace (phi = z - AXIAL_SPLIT_Z, gradient = +z; distal half has phi<0)",
        "zone_partition_convention": "sharp at |z - AXIAL_SPLIT| = AXIAL_BAND_HALF_WIDTH; smoothstep applies to material sampling, not to zone classification",
        "tet_counts": {
            "distal_inner": n_zone_shell[0][0],
            "distal_middle": n_zone_shell[0][1],
            "distal_outer": n_zone_shell[0][2],
            "band_inner": n_zone_shell[1][0],
            "band_middle": n_zone_shell[1][1],
            "band_outer": n_zone_shell[1][2],
            "proximal_inner": n_zone_shell[2][0],
            "proximal_middle": n_zone_shell[2][1],
            "proximal_outer": n_zone_shell[2][2],
        },
        "final_step_mean_psi_j_per_m3": {
            "distal_inner": final_data.mean_psi_zone_shell[0][0],
            "distal_middle": final_data.mean_psi_zone_shell[0][1],
            "distal_outer": final_data.mean_psi_zone_shell[0][2],
            "band_inner": final_data.mean_psi_zone_shell[1][0],
            "band_middle": final_data.mean_psi_zone_shell[1][1],
            "band_outer": final_data.mean_psi_zone_shell[1][2],
            "proximal_inner": final_data.mean_psi_zone_shell[2][0],
            "proximal_middle": final_data.mean_psi_zone_shell[2][1],
            "proximal_outer": final_data.mean_psi_zone_shell[2][2],
        },
    });

    let material_layers = json!([
        {
            "shell": "inner",
            "n_tets": inner_count,
            "proximal_anchor": {
                "name": "ECOFLEX_00_20",
                "mu_pa": ECOFLEX_00_20.mu,
                "c2_pa": ECOFLEX_00_20.c2,
                "lambda_pa": ECOFLEX_00_20.lambda,
                "density_kg_m3": ECOFLEX_00_20.density,
                "validity_max_principal_stretch": ECOFLEX_00_20.validity_max_principal_stretch,
                "validity_min_principal_stretch": ECOFLEX_00_20.validity_min_principal_stretch,
                "construction_path": "Path 1 (anchor)",
            },
            "distal_anchor": {
                "name": "ECOFLEX_00_30",
                "mu_pa": ECOFLEX_00_30.mu,
                "c2_pa": ECOFLEX_00_30.c2,
                "lambda_pa": ECOFLEX_00_30.lambda,
                "density_kg_m3": ECOFLEX_00_30.density,
                "validity_max_principal_stretch": ECOFLEX_00_30.validity_max_principal_stretch,
                "validity_min_principal_stretch": ECOFLEX_00_30.validity_min_principal_stretch,
                "construction_path": "Path 1 (anchor)",
            },
            "engineering_role": "skin-contact softness; soft tip, one-Shore-step stiffer at the anchor end",
        },
        {
            "shell": "middle",
            "n_tets": middle_count,
            "proximal_anchor": {
                "name": "DRAGON_SKIN_10A",
                "mu_pa": DRAGON_SKIN_10A.mu,
                "c2_pa": DRAGON_SKIN_10A.c2,
                "lambda_pa": DRAGON_SKIN_10A.lambda,
                "density_kg_m3": DRAGON_SKIN_10A.density,
                "validity_max_principal_stretch": DRAGON_SKIN_10A.validity_max_principal_stretch,
                "validity_min_principal_stretch": DRAGON_SKIN_10A.validity_min_principal_stretch,
                "construction_path": "Path 1 (anchor)",
            },
            "distal_anchor": {
                "name": "DRAGON_SKIN_15",
                "mu_pa": DRAGON_SKIN_15.mu,
                "c2_pa": DRAGON_SKIN_15.c2,
                "lambda_pa": DRAGON_SKIN_15.lambda,
                "density_kg_m3": DRAGON_SKIN_15.density,
                "validity_max_principal_stretch": DRAGON_SKIN_15.validity_max_principal_stretch,
                "validity_min_principal_stretch": DRAGON_SKIN_15.validity_min_principal_stretch,
                "construction_path": "Path 1 (anchor)",
            },
            "engineering_role": "conductive composite proxy; softer at contact, stiffer at anchor",
        },
        {
            "shell": "outer",
            "n_tets": outer_count,
            "proximal_anchor": {
                "name": "DRAGON_SKIN_20A",
                "mu_pa": DRAGON_SKIN_20A.mu,
                "c2_pa": DRAGON_SKIN_20A.c2,
                "lambda_pa": DRAGON_SKIN_20A.lambda,
                "density_kg_m3": DRAGON_SKIN_20A.density,
                "validity_max_principal_stretch": DRAGON_SKIN_20A.validity_max_principal_stretch,
                "validity_min_principal_stretch": DRAGON_SKIN_20A.validity_min_principal_stretch,
                "construction_path": "Path 1 (anchor)",
            },
            "distal_anchor": {
                "name": "DRAGON_SKIN_30A",
                "mu_pa": DRAGON_SKIN_30A.mu,
                "c2_pa": DRAGON_SKIN_30A.c2,
                "lambda_pa": DRAGON_SKIN_30A.lambda,
                "density_kg_m3": DRAGON_SKIN_30A.density,
                "validity_max_principal_stretch": DRAGON_SKIN_30A.validity_max_principal_stretch,
                "validity_min_principal_stretch": DRAGON_SKIN_30A.validity_min_principal_stretch,
                "construction_path": "Path 1 (anchor)",
            },
            "engineering_role": "structural stiffness; strongest gradient (×1.75 μ at distal vs proximal)",
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
    let final_pairs = final_data.pair_records.clone();
    let document = json!({
        "scalars": scalars,
        "axial_zoning": axial_zoning,
        "material_layers": material_layers,
        "ramp_curve": ramp_curve,
        "final_contact_pairs": final_pairs,
    });
    std::fs::write(path, serde_json::to_string_pretty(&document)?)?;
    Ok(())
}

// =============================================================================
// PLY x-slab artifact emit (final step only) — NEW slab cut for v3
// =============================================================================

#[derive(Clone, Copy)]
struct XslabRecord {
    centroid: Vec3,
    // `deformed_centroid` was used at row 24 N+0/N+1 for the
    // amplified PLY render; dropped at N+3 (sim-soft viz arc option
    // 1: render at rest positions). Displacement magnitude is now
    // computed at the call site BEFORE pushing the record, so this
    // record only needs the rest centroid.
}

#[allow(clippy::too_many_arguments)]
fn emit_xslab_ply(
    path: &Path,
    records: &[XslabRecord],
    displacement_magnitudes: &[f64],
    material_ids: &[f64],
    zone_ids: &[u32],
    mu_sampled_pa: &[f64],
    psi_j_per_m3: &[f64],
) -> Result<()> {
    assert_eq!(records.len(), displacement_magnitudes.len());
    assert_eq!(records.len(), material_ids.len());
    assert_eq!(records.len(), zone_ids.len());
    assert_eq!(records.len(), mu_sampled_pa.len());
    assert_eq!(records.len(), psi_j_per_m3.len());

    let mut geometry = IndexedMesh::new();
    for r in records {
        // Render at REST positions (no displacement amplification) per
        // sim-soft viz arc option-1 baby step (memory file
        // `project_sim_soft_viz_arc.md`). The body's rest-shape
        // rectangle is preserved; the `psi_j_per_m3` PLY extra (sequential
        // viridis under cf-view) carries the contact-zone strain-energy
        // concentration as a heatmap, which is the canonical FEM-viz
        // story for soft-material contact problems. Earlier rows
        // (21/22/23) used `DISPLACEMENT_SCALE = 10.0` to amplify
        // the deformation field, but at this row's contact intensity
        // (8 mm penetration on a 108 mm body) the 10× amplification
        // sent the contact-band tets ~85 mm above the body's rest
        // extent, reading like a fluid spray rather than a soft
        // solid (user feedback 2026-05-10). The displacement field
        // is preserved as an unscaled `displacement_magnitude` PLY
        // extra so the deformation story is still readable via that
        // scalar.
        geometry
            .vertices
            .push(Point3::new(r.centroid.x, r.centroid.y, r.centroid.z));
    }
    // No faces — x-slab is a per-tet centroid cloud, point-only PLY.
    // Real triangulated surface mesh from tet centroids is the
    // option-3 follow-up arc in `project_sim_soft_viz_arc.md`.

    let mut mesh = AttributedMesh::new(geometry);
    let disp_f32: Vec<f32> = displacement_magnitudes.iter().map(|&v| v as f32).collect();
    let mat_f32: Vec<f32> = material_ids.iter().map(|&v| v as f32).collect();
    let mu_f32: Vec<f32> = mu_sampled_pa.iter().map(|&v| v as f32).collect();
    let psi_f32: Vec<f32> = psi_j_per_m3.iter().map(|&v| v as f32).collect();
    // Use AttributedMesh's canonical `zone_ids` slot (Vec<u32>) — the
    // PLY exporter routes this to a reserved `zone_id` property
    // automatically; `insert_extra("zone_id", …)` is rejected as a
    // reserved-name collision (`mesh-io/src/ply.rs:322`).
    mesh.zone_ids = Some(zone_ids.to_vec());
    // Mirror the zone-id data as an `axial_zone_id` per-vertex extra
    // (Vec<f32>) so cf-view's scalar selector picks it up — the
    // viewer at `cf-viewer/src/lib.rs:63` enumerates only
    // `mesh.extras.keys()`, not canonical AttributedMesh slots, so
    // the canonical `zone_ids` data is invisible in the Scalar
    // dropdown without this mirror. The reserved-name guard at
    // `mesh-io/src/ply.rs:322` blocks the bare `zone_id` extra; the
    // disambiguated `axial_zone_id` name is accepted.
    let zone_f32: Vec<f32> = zone_ids.iter().map(|&v| v as f32).collect();
    mesh.insert_extra("displacement_magnitude", disp_f32)?;
    mesh.insert_extra("material_id", mat_f32)?;
    mesh.insert_extra("axial_zone_id", zone_f32)?;
    mesh.insert_extra("mu_sampled_pa", mu_f32)?;
    mesh.insert_extra("psi_j_per_m3", psi_f32)?;
    save_ply_attributed(&mesh, path, true)?;
    Ok(())
}

// =============================================================================
// PLY boundary-surface artifact emit (final step only) — F1.0 SCRATCH PROTOTYPE
// =============================================================================
//
// Throwaway scratch implementation of the sim-soft viz arc's
// `boundary_surface()` primitive (see `project_sim_soft_viz_arc.md`
// §"Pivoted architecture — tet-mesh-native primitives"). Renders the
// row's full 3D body as a triangulated boundary surface coloured by
// per-vertex strain-energy density `psi_j_per_m3`, replacing the
// x-slab centroid cloud's reduce-to-2D framing with the canonical
// FEM-viz convention (3D body, sequential heatmap, rotate to inspect).
//
// F1.0 scope: this row only, psi-only scalar, volume-weighted
// per-vertex averaging. F1.1 lift to `sim/L0/soft/src/viz/mod.rs`
// (with HashMap<&str, &[f64]> multi-scalar API + unit tests +
// docstrings) is conditional on this prototype passing eyes-on-pixels
// in cf-view per `feedback_visual_review_is_the_test`. F1.2 retires
// the x-slab emit. Spike-before-lock discipline per
// `feedback_spike_before_trust_analytical` — the Delaunay-of-centroids
// spike just taught us this lesson the hard way.
//
// Architectural finding worth banking before lift: `Mesh<M>` already
// exposes `boundary_faces() -> &[[VertexId; 3]]` precomputed at
// construction with documented outward winding (right-handed tet
// vertex order; see `sim/L0/soft/src/mesh/mod.rs:120-139`). The
// memo's open question 2 (winding) and 4 (trait surface) are answered
// by the trait contract — F1.1 does not need to write
// face-to-tet-adjacency counting.
// Volume-weighted per-vertex psi averaging. Each tet contributes its
// psi to each of its 4 vertices weighted by tet volume. Avoids small
// boundary-fitting tets dominating per-vertex averages on
// Isosurface-Stuffed organic meshes (chosen over uniform after the
// 2026-05-10 planning discussion — uniform looks identical on row
// 24's near-uniform BCC mesh but degrades on the production target
// of organic scanned shapes; banking the durable choice upfront
// avoids an F1.0→F1.1 averaging-strategy discontinuity).
// signed_volume comes out positive for right-handed tets per
// pipeline Decision H (no negative-volume survivor); .abs() is
// defensive belt-and-braces. Shared between F1.0 boundary_surface
// and F1.3 slab_cut helpers; lifts to viz module at F1.1 alongside
// the emit primitives.
fn volume_weighted_per_vertex_psi(
    n_vertices: usize,
    tets: &[[VertexId; 4]],
    signed_volumes: &[f64],
    psi_per_tet: &[f64],
) -> Vec<f64> {
    assert_eq!(signed_volumes.len(), tets.len());
    assert_eq!(psi_per_tet.len(), tets.len());
    let mut accum_psi_vol = vec![0.0_f64; n_vertices];
    let mut accum_vol = vec![0.0_f64; n_vertices];
    for (t, &[v0, v1, v2, v3]) in tets.iter().enumerate() {
        let vol = signed_volumes[t].abs();
        let weighted_psi = psi_per_tet[t] * vol;
        for v in [v0, v1, v2, v3] {
            accum_psi_vol[v as usize] += weighted_psi;
            accum_vol[v as usize] += vol;
        }
    }
    (0..n_vertices)
        .map(|v| {
            if accum_vol[v] > 0.0 {
                accum_psi_vol[v] / accum_vol[v]
            } else {
                // Orphan vertex (no incident tets) — should not occur
                // on the BCC + IS pipeline's connected meshes; emit
                // 0 so the scalar slot stays valid for the PLY.
                0.0
            }
        })
        .collect()
}

fn emit_boundary_surface_ply(
    path: &Path,
    positions: &[Vec3],
    boundary_faces: &[[u32; 3]],
    psi_per_vertex: &[f64],
) -> Result<()> {
    assert_eq!(psi_per_vertex.len(), positions.len());

    let mut geometry = IndexedMesh::new();
    for p in positions {
        geometry.vertices.push(Point3::new(p.x, p.y, p.z));
    }
    // Boundary face indices are u32 = VertexId; copy directly. Faces
    // index into `geometry.vertices` by full mesh VertexId so all
    // n_vertices positions are emitted (interior vertices appear as
    // unreferenced; PLY readers tolerate this and cf-view renders
    // faces only). F1.1 polish: compact remap to boundary-vertex
    // subset would shrink the PLY ~5×.
    for face in boundary_faces {
        geometry.faces.push(*face);
    }

    let psi_f32: Vec<f32> = psi_per_vertex.iter().map(|&v| v as f32).collect();
    let mut attr = AttributedMesh::new(geometry);
    attr.insert_extra("psi_j_per_m3", psi_f32)?;
    save_ply_attributed(&attr, path, true)?;
    Ok(())
}

// =============================================================================
// PLY slab-cut artifact emit (final step only) — F1.3 SCRATCH PROTOTYPE
// =============================================================================
//
// Throwaway scratch implementation of the sim-soft viz arc's
// `slab_cut()` primitive (see `project_sim_soft_viz_arc.md`
// §"Pivoted architecture — Primitive 2"). Intersects the tet mesh
// with an axis-aligned plane via marching-tetrahedra; emits the cut
// polygon mesh with per-vertex `psi_j_per_m3` linearly interpolated
// from the volume-weighted per-vertex psi field.
//
// Pulled forward from F1.3 (originally scheduled after F1.1 lift +
// F1.2 retrofit) per 2026-05-10 user discussion: row 24's wrap is
// meant to model an insertion-cavity device, and slab_cut exposes
// the interior cavity + axial strain gradient that the closed
// boundary-surface PLY hides. F1.6 ships row 25 with an open-mouth
// wrap variant that gives the FEM-correct version of "see inside";
// slab_cut is the meantime view.
//
// Algorithm (5 cases by symmetry from 16 sign patterns):
// - 4-above / 4-below: tet entirely on one side, no contribution.
// - 1-above-3-below or 3-above-1-below: 3 cross-edges between the
//   singleton and the 3-set; emit 1 triangle.
// - 2-above-2-below: 4 cross-edges (a×c, a×d, b×c, b×d for above
//   {a,b} and below {c,d}); emit a quad split into 2 triangles.
// Cross-points are linearly interpolated along edges; positions and
// per-vertex psi share the same parameter t. Cross-points are
// dedup'd by sorted edge key so a shared edge (interior edges
// appear in 2 tets) produces one cross-vertex.
//
// Winding: each emitted triangle is reordered (if needed) so its
// normal projection onto the plane axis is non-negative. This keeps
// all cut polygons facing the +axis side, satisfying cf-view's
// default single-sided lighting (mirrors the Mesh trait's outward
// winding contract for boundary_faces).
#[allow(clippy::too_many_arguments)]
fn slab_cut_intersect_edge(
    va: VertexId,
    vb: VertexId,
    positions: &[Vec3],
    psi_per_vertex: &[f64],
    sd: &[f64],
    edge_to_idx: &mut std::collections::HashMap<(VertexId, VertexId), u32>,
    cut_positions: &mut Vec<Point3<f64>>,
    cut_psi: &mut Vec<f64>,
) -> u32 {
    let (lo, hi) = if va < vb { (va, vb) } else { (vb, va) };
    if let Some(&idx) = edge_to_idx.get(&(lo, hi)) {
        return idx;
    }
    let sd_a = sd[va as usize];
    let sd_b = sd[vb as usize];
    // Parameter t from va toward vb where the plane crosses
    // (0 at va, 1 at vb). Sign-difference denominator is non-zero
    // because va and vb are on opposite sides of the plane (by
    // caller construction).
    let t = sd_a / (sd_a - sd_b);
    let p_a = positions[va as usize];
    let p_b = positions[vb as usize];
    let cross = p_a + (p_b - p_a) * t;
    let psi_a = psi_per_vertex[va as usize];
    let psi_b = psi_per_vertex[vb as usize];
    let cross_psi_val = (1.0 - t).mul_add(psi_a, t * psi_b);
    let new_idx = cut_positions.len() as u32;
    cut_positions.push(Point3::new(cross.x, cross.y, cross.z));
    cut_psi.push(cross_psi_val);
    edge_to_idx.insert((lo, hi), new_idx);
    new_idx
}

fn align_winding_to_axis(
    tri: [u32; 3],
    cut_positions: &[Point3<f64>],
    plane_axis: usize,
) -> [u32; 3] {
    let p0 = cut_positions[tri[0] as usize];
    let p1 = cut_positions[tri[1] as usize];
    let p2 = cut_positions[tri[2] as usize];
    let a = p1 - p0;
    let b = p2 - p0;
    let cross_on_axis = match plane_axis {
        0 => a.y.mul_add(b.z, -(a.z * b.y)),
        1 => a.z.mul_add(b.x, -(a.x * b.z)),
        _ => a.x.mul_add(b.y, -(a.y * b.x)),
    };
    if cross_on_axis >= 0.0 {
        tri
    } else {
        [tri[0], tri[2], tri[1]]
    }
}

fn emit_slab_cut_ply(
    path: &Path,
    positions: &[Vec3],
    tets: &[[VertexId; 4]],
    psi_per_vertex: &[f64],
    plane_axis: usize,
    plane_value: f64,
) -> Result<()> {
    assert!(plane_axis < 3);
    assert_eq!(psi_per_vertex.len(), positions.len());

    // Per-vertex signed distance: positive = above (axis side),
    // non-positive = below. Vertices exactly on the plane (sd == 0)
    // classify as below, which makes the cross-edge interpolation at
    // t = 0 collapse the cross-point onto the mesh vertex (harmless
    // degeneracy).
    let sd: Vec<f64> = positions
        .iter()
        .map(|p| p[plane_axis] - plane_value)
        .collect();

    let mut edge_to_idx: std::collections::HashMap<(VertexId, VertexId), u32> =
        std::collections::HashMap::new();
    let mut cut_positions: Vec<Point3<f64>> = Vec::new();
    let mut cut_psi: Vec<f64> = Vec::new();
    let mut cut_faces: Vec<[u32; 3]> = Vec::new();

    for &[v0, v1, v2, v3] in tets {
        let verts = [v0, v1, v2, v3];
        let mut above = [0u32; 4];
        let mut na = 0usize;
        let mut below = [0u32; 4];
        let mut nb = 0usize;
        for &v in &verts {
            if sd[v as usize] > 0.0 {
                above[na] = v;
                na += 1;
            } else {
                below[nb] = v;
                nb += 1;
            }
        }
        let push_tri = |faces: &mut Vec<[u32; 3]>, cuts: &[Point3<f64>], tri: [u32; 3]| {
            faces.push(align_winding_to_axis(tri, cuts, plane_axis));
        };
        match (na, nb) {
            (4, 0) | (0, 4) => {}
            (1, 3) => {
                let s = above[0];
                let i0 = slab_cut_intersect_edge(
                    s,
                    below[0],
                    positions,
                    psi_per_vertex,
                    &sd,
                    &mut edge_to_idx,
                    &mut cut_positions,
                    &mut cut_psi,
                );
                let i1 = slab_cut_intersect_edge(
                    s,
                    below[1],
                    positions,
                    psi_per_vertex,
                    &sd,
                    &mut edge_to_idx,
                    &mut cut_positions,
                    &mut cut_psi,
                );
                let i2 = slab_cut_intersect_edge(
                    s,
                    below[2],
                    positions,
                    psi_per_vertex,
                    &sd,
                    &mut edge_to_idx,
                    &mut cut_positions,
                    &mut cut_psi,
                );
                push_tri(&mut cut_faces, &cut_positions, [i0, i1, i2]);
            }
            (3, 1) => {
                let s = below[0];
                let i0 = slab_cut_intersect_edge(
                    s,
                    above[0],
                    positions,
                    psi_per_vertex,
                    &sd,
                    &mut edge_to_idx,
                    &mut cut_positions,
                    &mut cut_psi,
                );
                let i1 = slab_cut_intersect_edge(
                    s,
                    above[1],
                    positions,
                    psi_per_vertex,
                    &sd,
                    &mut edge_to_idx,
                    &mut cut_positions,
                    &mut cut_psi,
                );
                let i2 = slab_cut_intersect_edge(
                    s,
                    above[2],
                    positions,
                    psi_per_vertex,
                    &sd,
                    &mut edge_to_idx,
                    &mut cut_positions,
                    &mut cut_psi,
                );
                push_tri(&mut cut_faces, &cut_positions, [i0, i1, i2]);
            }
            (2, 2) => {
                // Quad cross-section. Above = {a, b}, below = {c, d}.
                // Cross-edges: a-c, b-c, b-d, a-d. Going around the
                // quad's perimeter: i_ac → i_bc → i_bd → i_ad → i_ac.
                // Triangulate via the i_ac--i_bd diagonal.
                let i_ac = slab_cut_intersect_edge(
                    above[0],
                    below[0],
                    positions,
                    psi_per_vertex,
                    &sd,
                    &mut edge_to_idx,
                    &mut cut_positions,
                    &mut cut_psi,
                );
                let i_bc = slab_cut_intersect_edge(
                    above[1],
                    below[0],
                    positions,
                    psi_per_vertex,
                    &sd,
                    &mut edge_to_idx,
                    &mut cut_positions,
                    &mut cut_psi,
                );
                let i_bd = slab_cut_intersect_edge(
                    above[1],
                    below[1],
                    positions,
                    psi_per_vertex,
                    &sd,
                    &mut edge_to_idx,
                    &mut cut_positions,
                    &mut cut_psi,
                );
                let i_ad = slab_cut_intersect_edge(
                    above[0],
                    below[1],
                    positions,
                    psi_per_vertex,
                    &sd,
                    &mut edge_to_idx,
                    &mut cut_positions,
                    &mut cut_psi,
                );
                push_tri(&mut cut_faces, &cut_positions, [i_ac, i_bc, i_bd]);
                push_tri(&mut cut_faces, &cut_positions, [i_ac, i_bd, i_ad]);
            }
            _ => unreachable!("tet has 4 vertices; (na, nb) sums to 4"),
        }
    }

    let mut geometry = IndexedMesh::new();
    geometry.vertices = cut_positions;
    geometry.faces = cut_faces;

    let psi_f32: Vec<f32> = cut_psi.iter().map(|&v| v as f32).collect();
    let mut attr = AttributedMesh::new(geometry);
    attr.insert_extra("psi_j_per_m3", psi_f32)?;
    save_ply_attributed(&attr, path, true)?;
    Ok(())
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    println!("scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp — row 24 (F4.1+v3 axial-zoned)");
    println!();

    // 1-5. Build initial mesh + verifies (geometry + material checks).
    let scan_solid = build_scan_solid();
    let outer_envelope = build_outer_envelope(scan_solid.clone());
    let sleeve_body = build_sleeve_body(outer_envelope.clone(), scan_solid.clone());
    let material_field = build_axial_zoned_material_field();
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

    // Per-tet shell + zone classification at rest centroids.
    let mut n_inner = 0usize;
    let mut n_middle = 0usize;
    let mut n_outer = 0usize;
    let mut shell_idx_per_tet: Vec<usize> = Vec::with_capacity(n_tets);
    let mut zone_idx_per_tet: Vec<usize> = Vec::with_capacity(n_tets);
    let mut n_zone_shell: [[usize; 3]; 3] = [[0; 3]; 3];
    for &[v0, v1, v2, v3] in &tets {
        let centroid = (positions[v0 as usize]
            + positions[v1 as usize]
            + positions[v2 as usize]
            + positions[v3 as usize])
            / 4.0;
        let phi = scan_solid.eval(nalgebra::Point3::from(centroid));
        let s = shell_at_phi(phi);
        let z = zone_at_z(centroid.z);
        shell_idx_per_tet.push(s);
        zone_idx_per_tet.push(z);
        match s {
            0 => n_inner += 1,
            1 => n_middle += 1,
            _ => n_outer += 1,
        }
        n_zone_shell[z][s] += 1;
    }

    // Quality + counts gates BEFORE the ramp.
    verify_quality_floors(&mesh);
    verify_counts_exact(
        &mesh,
        &referenced,
        &bc.pinned_vertices,
        n_inner,
        n_middle,
        n_outer,
    );
    verify_zone_shell_counts_exact(n_zone_shell);
    verify_material_assignment_partition(&mesh, &shell_idx_per_tet, &zone_idx_per_tet);
    verify_material_provenance();
    verify_blend_zone_material_provenance(&mesh, &tets, &positions, &shell_idx_per_tet);

    // Stash the small mesh-derived data the F1.0 boundary-surface PLY
    // emit needs (boundary face indices + per-tet signed volumes).
    // Captured here so the existing `drop(mesh)` below can free the
    // bulk mesh memory before the ramp loop.
    let boundary_faces: Vec<[u32; 3]> = mesh.boundary_faces().to_vec();
    let signed_volumes: Vec<f64> = mesh.quality().signed_volume.clone();

    // 6. Quasi-static intrusion ramp.
    drop(mesh);
    drop(bc);
    let _outer_kept = outer_envelope; // silence unused

    let results = solve_ramp(
        n_vertices,
        &referenced,
        &tets,
        &shell_idx_per_tet,
        &zone_idx_per_tet,
        n_inner,
        n_middle,
        n_outer,
        n_zone_shell,
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
    verify_zone_shell_psi_final(&results);

    // 8. JSON + PLY readouts.
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;

    let json_path = out_dir.join("scan_fit_3layer_sleeve_yeoh_axial_zoned_ramp.json");
    write_json_readout(
        &json_path,
        n_tets,
        n_vertices,
        referenced.len(),
        n_pinned,
        n_inner,
        n_middle,
        n_outer,
        n_zone_shell,
        &results,
    )?;

    // PLY x-slab (final step only) — NEW slab cut for v3.
    let final_step = results.last().expect("ramp produced no results");
    let final_data = final_step
        .final_step_data
        .as_ref()
        .expect("final ramp step missing FinalStepData");
    let half_cell = 0.5 * CELL_SIZE;

    // Sample the same axial-zoned material field as the solver to
    // record per-tet sampled μ for the PLY's sequential
    // `mu_sampled_pa` extra. Independent rebuild so we don't disturb
    // the solver's mesh consumption.
    let sampling_field = build_axial_zoned_material_field();

    let mut xslab_records: Vec<XslabRecord> = Vec::new();
    let mut xslab_disp: Vec<f64> = Vec::new();
    let mut xslab_mat: Vec<f64> = Vec::new();
    let mut xslab_zone: Vec<u32> = Vec::new();
    let mut xslab_mu: Vec<f64> = Vec::new();
    let mut xslab_psi: Vec<f64> = Vec::new();
    let mut n_zone_shell_xslab: [[usize; 3]; 3] = [[0; 3]; 3];
    let mut n_inner_z_carry = 0usize;
    let mut n_middle_z_carry = 0usize;
    let mut n_outer_z_carry = 0usize;
    for (tet_idx, &[v0, v1, v2, v3]) in tets.iter().enumerate() {
        let rest_centroid = (final_data.rest_positions[v0 as usize]
            + final_data.rest_positions[v1 as usize]
            + final_data.rest_positions[v2 as usize]
            + final_data.rest_positions[v3 as usize])
            / 4.0;
        // z-slab carry-through: filter on `|cz| < half_cell` for
        // bit-equality with row 23's z-slab counts (regression gate;
        // not emitted as PLY).
        if rest_centroid.z.abs() < half_cell {
            match shell_idx_per_tet[tet_idx] {
                0 => n_inner_z_carry += 1,
                1 => n_middle_z_carry += 1,
                _ => n_outer_z_carry += 1,
            }
        }
        // x-slab: filter on `|cx| < half_cell` — the v3 differentiator.
        if rest_centroid.x.abs() >= half_cell {
            continue;
        }
        let deformed_centroid = (final_data.deformed_positions[v0 as usize]
            + final_data.deformed_positions[v1 as usize]
            + final_data.deformed_positions[v2 as usize]
            + final_data.deformed_positions[v3 as usize])
            / 4.0;
        let s = shell_idx_per_tet[tet_idx];
        let z = zone_idx_per_tet[tet_idx];
        n_zone_shell_xslab[z][s] += 1;
        let mu_sampled = sampling_field.sample_yeoh(rest_centroid).mu();
        xslab_records.push(XslabRecord {
            centroid: rest_centroid,
        });
        xslab_disp.push((deformed_centroid - rest_centroid).norm());
        xslab_mat.push(s as f64);
        xslab_zone.push(z as u32);
        xslab_mu.push(mu_sampled);
        xslab_psi.push(final_data.per_tet_psi[tet_idx]);
    }
    verify_zslab_counts_exact(n_inner_z_carry, n_middle_z_carry, n_outer_z_carry);
    verify_xslab_zone_shell_counts_exact(n_zone_shell_xslab);

    let ply_path = out_dir.join("sleeve_xslab_final.ply");
    emit_xslab_ply(
        &ply_path,
        &xslab_records,
        &xslab_disp,
        &xslab_mat,
        &xslab_zone,
        &xslab_mu,
        &xslab_psi,
    )?;

    // F1.0 + F1.3 scratch viz primitives — sim-soft viz arc option-3
    // tet-mesh-native architecture. boundary_surface() emits the full
    // 3D body coloured by per-vertex psi; slab_cut() emits the
    // cross-section at x = 0 so the cavity + axial strain gradient
    // are visible from outside the closed body. Both share the same
    // volume-weighted per-vertex psi field. The x-slab centroid PLY
    // stays as the canonical emit at F1.0/F1.3; F1.2 retires it once
    // the surface + cut representations are verified as the better
    // defaults.
    let psi_per_vertex =
        volume_weighted_per_vertex_psi(n_vertices, &tets, &signed_volumes, &final_data.per_tet_psi);

    let bd_ply_path = out_dir.join("sleeve_boundary_final.ply");
    emit_boundary_surface_ply(&bd_ply_path, &positions, &boundary_faces, &psi_per_vertex)?;

    let slab_ply_path = out_dir.join("sleeve_slab_cut_x0_final.ply");
    emit_slab_cut_ply(
        &slab_ply_path,
        &positions,
        &tets,
        &psi_per_vertex,
        0,   // x-axis cut plane
        0.0, // x = 0 (matches existing x-slab centroid convention)
    )?;

    // 9. Museum-plaque summary.
    print_summary(
        n_tets,
        n_vertices,
        referenced.len(),
        n_pinned,
        n_inner,
        n_middle,
        n_outer,
        n_zone_shell,
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
    n_zone_shell: [[usize; 3]; 3],
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
    println!("Axial zoning (NEW for v3):");
    println!("  AXIAL_SPLIT_Z            : {AXIAL_SPLIT_Z} m (equator)");
    println!("  AXIAL_BAND_HALF_WIDTH    : {AXIAL_BAND_HALF_WIDTH} m (10 mm full band)");
    println!("  Proximal stack (+z, contact): Ecoflex 00-20 / DS10A / DS20A");
    println!("  Distal stack   (-z, anchor) : Ecoflex 00-30 / DS15  / DS30A");
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
    println!("    inner  : {n_inner}");
    println!("    middle : {n_middle}");
    println!("    outer  : {n_outer}");
    println!("  per-zone × shell tet counts (distal / band / proximal × inner / middle / outer):");
    let zone_names = ["distal  ", "band    ", "proximal"];
    for z in 0..3 {
        println!(
            "    {:<8} : inner = {:>5}  middle = {:>5}  outer = {:>5}",
            zone_names[z], n_zone_shell[z][0], n_zone_shell[z][1], n_zone_shell[z][2],
        );
    }
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
    let psi = &final_step
        .final_step_data
        .as_ref()
        .expect("final step missing FinalStepData")
        .mean_psi_zone_shell;
    println!(
        "Final step ({}, depth = {} mm):",
        final_step.step,
        final_step.depth_m * 1000.0
    );
    println!("  Per-shell mean strain-energy density (J/m³):");
    println!("    Ψ̄_inner  : {:e}", final_step.mean_psi_inner_j_per_m3);
    println!("    Ψ̄_middle : {:e}", final_step.mean_psi_middle_j_per_m3);
    println!("    Ψ̄_outer  : {:e}", final_step.mean_psi_outer_j_per_m3);
    println!("    max Ψ_outer : {:e}", final_step.max_psi_outer_j_per_m3);
    println!("  Per-zone × shell mean Ψ̄ (J/m³):");
    println!("    zone     | inner       middle      outer");
    for z in 0..3 {
        println!(
            "    {:<8} | {:>10.3e}  {:>10.3e}  {:>10.3e}",
            zone_names[z], psi[z][0], psi[z][1], psi[z][2],
        );
    }
    println!();
    println!("Outputs:");
    println!(
        "  out/scan_fit_3layer_sleeve_yeoh_axial_zoned_ramp.json (scalars + axial_zoning + 3-shell × 2-zone Yeoh materials + ramp_curve + final_pairs)"
    );
    println!(
        "  out/sleeve_xslab_final.ply                             (x-slab centroid cloud at final step at REST positions, six scalars: material_id, zone_id, axial_zone_id, displacement_magnitude, mu_sampled_pa, psi_j_per_m3)"
    );
    println!(
        "  out/sleeve_boundary_final.ply                          (F1.0 scratch boundary-surface PLY — full 3D body, per-vertex volume-weighted psi_j_per_m3 heatmap; sim-soft viz arc option-3)"
    );
    println!(
        "  out/sleeve_slab_cut_x0_final.ply                       (F1.3 scratch slab-cut PLY — marching-tet cross-section at x = 0, per-vertex psi via linear interp; exposes cavity + axial gradient)"
    );
    println!();
    println!("View final-step PLYs in cf-view (workspace's unified visual-review viewer):");
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/out/sleeve_xslab_final.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/out/sleeve_boundary_final.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/out/sleeve_slab_cut_x0_final.ply"
    );
    println!();
    println!("Optional matplotlib post-processing (force-displacement + max_disp curves):");
    println!(
        "  uv run examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/plot_ramp.py"
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
    // Axial zoning placement: the band must fit entirely within the
    // body's z extent so neither proximal nor distal pure regions
    // collapse to empty.
    assert!(AXIAL_BAND_HALF_WIDTH > 0.0);
    assert!(AXIAL_SPLIT_Z + AXIAL_BAND_HALF_WIDTH < SCAN_HZ + WRAP_THICKNESS);
    assert!(AXIAL_SPLIT_Z - AXIAL_BAND_HALF_WIDTH > -(SCAN_HZ + WRAP_THICKNESS));
    // Band must span at least one BCC cell (smoothstep would alias on
    // the lattice if the band is sub-cell).
    assert!(2.0 * AXIAL_BAND_HALF_WIDTH >= CELL_SIZE);
};
