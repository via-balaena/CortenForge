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
//!    - PLY `out/sleeve_boundary_final.ply`: full 3D body emitted
//!      via [`sim_soft::viz::boundary_surface`] (F1.1 lift of the
//!      F1.0 scratch prototype, see [vizarc] memory file). Per-vertex
//!      `psi_j_per_m3` projected from per-tet psi via volume-weighted
//!      averaging; outward winding inherited from
//!      [`sim_soft::Mesh::boundary_faces`]. Replaces the pre-F1.2
//!      x-slab centroid cloud's reduce-to-2D framing with the
//!      canonical FEM-viz convention (3D body, sequential heatmap,
//!      rotate to inspect). The Delaunay-of-centroids architecture
//!      that preceded this primitive was falsified by an 8-iteration
//!      spike (banked in the [vizarc] memory).
//!    - PLY `out/sleeve_slab_cut_x0_final.ply`: cross-section at
//!      x = 0 emitted via [`sim_soft::viz::slab_cut`] (F1.1 lift of
//!      the F1.3 scratch prototype). Marching-tetrahedra
//!      intersection of the tet mesh with the cutting plane;
//!      per-vertex `psi_j_per_m3` linearly interpolated along cross-
//!      edges. Exposes the inner cavity profile + axial
//!      proximal/band/distal strain gradient that the closed
//!      boundary-surface PLY hides — the FEM-canonical "clipping
//!      plane" view of an enclosed cavity. F1.6 ships row 25 with
//!      an open-mouth wrap variant that gives the FEM-correct
//!      version of "see inside" (cavity has an actual physical
//!      opening through the +z face); slab_cut is the meantime view
//!      on this closed-body row.
//!
//!    The pre-F1.2 x-slab centroid PLY (`sleeve_xslab_final.ply`)
//!    was retired at F1.2 (sim-soft viz arc retrofit) — its
//!    z-slab and x-slab per-cell POPULATION gates survive as
//!    cheap centroid filters but no PLY data accumulates.
//!
//! [vizarc]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_viz_arc.md
//!    - Optional `plot_ramp.py` (PEP 723 + matplotlib): same
//!      dual-axis depth × `force_z` + depth × `max_disp` curve as
//!      row 23. Run via `uv run plot_ramp.py`.
//!    - `verify_*` runtime gates — pipeline-emergent structural +
//!      physics invariants (see "Numerical anchors" in `README.md`).
//!      This row is a Rule-B `validator`: its oracles are read from the
//!      real 16-step ramp (positive tet volume, fully-populated
//!      zone × shell decomposition, per-step convergence,
//!      force-displacement monotonicity, radial `Ψ̄_inner>middle>outer`
//!      per zone + axial `Ψ̄_proximal>Ψ̄_distal` per shell ordering,
//!      displacement bound, per-shell material routing). The pre-Rule-B
//!      captured-bit self-pins + the blend-zone smoothstep self-mirror
//!      were stripped (constitutive + `BlendedScalarField` blend +
//!      mesher correctness is lib-owned; see the de-frag note over the
//!      `Verifications` section). Robust 8 mm convergence routes the
//!      calibrated Yeoh validity bounds via `from_yeoh_fields_with_bounds`.
//!
//! # Why x = 0 for the slab cut
//!
//! Row 23's z-slab convention at z = 0 (the body equator) catches
//! the propagated radial response of the wrap shell — the cut is
//! 40 mm BELOW the contact zone at z ≈ +SCAN_HZ. For row 24 the
//! equator sits at AXIAL_SPLIT_Z = 0, INSIDE the smoothstep band, so
//! a z-slab samples blended-band material everywhere and
//! obliterates the soft-tip / stiff-anchor visualisation this row
//! demonstrates. The x = 0 slab cut (the [`sim_soft::viz::Plane`]
//! passed to [`sim_soft::viz::slab_cut`] below) cuts perpendicular
//! to the long axis and spans the full z range, exposing the
//! proximal-pure / band / distal-pure axial structure as a true
//! marching-tet cross-section (not the pre-F1.2 centroid cloud).
//! Per `feedback_visual_review_is_the_test`, the cut has to serve
//! the row's headline — row 24's headline is the axial material
//! gradient, so the cut has to expose it.
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

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use anyhow::Result;
use cf_design::Solid;
use mesh_io::save_ply_attributed;
use mesh_types::{Point3, Vector3};
use nalgebra::Matrix3;
use serde_json::{Value, json};
use sim_ml_chassis::Tensor;
use sim_soft::material::silicone_table::{
    DRAGON_SKIN_10A, DRAGON_SKIN_15, DRAGON_SKIN_20A, DRAGON_SKIN_30A, ECOFLEX_00_20, ECOFLEX_00_30,
};
use sim_soft::{
    Aabb3, BlendedScalarField, BoundaryConditions, CpuNewtonSolver, Field, LayeredScalarField,
    Material, MaterialField, Mesh, MeshingHints, PenaltyRigidContact,
    PenaltyRigidContactYeohSolver, Plane, Sdf, SdfMeshedTetMesh, Solver, SolverConfig, Tet4, Vec3,
    VertexId, Yeoh, boundary_surface, design_slab_cut, design_surface, design_surface_deformed,
    pick_vertices_by_predicate, referenced_vertices, slab_cut,
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

    // Per-anchor calibrated Yeoh validity bounds, blended axially exactly like
    // μ/C₂/λ. Only the TENSILE cap (`0.8·λ_break`) is applied at sample time
    // (H4-2-C `with_max_principal_stretch_only`); the compressive field
    // (`0.20`) is threaded for API completeness but dropped at sample
    // (compressive net = det F > 0). WITHOUT these, the bounds-less
    // `from_yeoh_fields` leaves each per-tet Yeoh's validity `None`, falling
    // through to the legacy Neo-Hookean σ=2.0 ceiling — ~3× tighter than
    // silicone's real ~4-9 envelope — which the 8 mm solve grazes near step 16
    // (see rows 23 + the row-24 README "Why the bounds matter" note).
    let max_stretch_proximal: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![
            ECOFLEX_00_20.validity_max_principal_stretch,
            DRAGON_SKIN_10A.validity_max_principal_stretch,
            DRAGON_SKIN_20A.validity_max_principal_stretch,
        ],
    ));
    let max_stretch_distal: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![
            ECOFLEX_00_30.validity_max_principal_stretch,
            DRAGON_SKIN_15.validity_max_principal_stretch,
            DRAGON_SKIN_30A.validity_max_principal_stretch,
        ],
    ));
    let max_stretch_field: Box<dyn Field<f64>> = Box::new(BlendedScalarField::new(
        axial(),
        max_stretch_distal,
        max_stretch_proximal,
        band,
    ));
    let min_stretch_proximal: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![
            ECOFLEX_00_20.validity_min_principal_stretch,
            DRAGON_SKIN_10A.validity_min_principal_stretch,
            DRAGON_SKIN_20A.validity_min_principal_stretch,
        ],
    ));
    let min_stretch_distal: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![
            ECOFLEX_00_30.validity_min_principal_stretch,
            DRAGON_SKIN_15.validity_min_principal_stretch,
            DRAGON_SKIN_30A.validity_min_principal_stretch,
        ],
    ));
    let min_stretch_field: Box<dyn Field<f64>> = Box::new(BlendedScalarField::new(
        axial(),
        min_stretch_distal,
        min_stretch_proximal,
        band,
    ));

    MaterialField::from_yeoh_fields_with_bounds(
        mu_field,
        c2_field,
        lambda_field,
        max_stretch_field,
        min_stretch_field,
    )
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
        roller_vertices: Vec::new(),
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
    /// Per-step deformed vertex positions. Always captured (F2.3c
    /// needs these per-step for the `design_surface_deformed` PLY
    /// series).
    deformed_positions: Vec<Vec3>,
    /// Per-tet strain-energy density (J/m³), indexed by tet ID,
    /// length = `n_tets`. Captured every step (F2.3c needs per-step
    /// psi for the ramp-animation PLY series); pre-F2.3c this was
    /// final-step only and lived inside [`FinalStepData`].
    per_tet_psi: Vec<f64>,
    /// Step 16 only: the per-pair readouts for JSON
    /// `final_contact_pairs` and the final-step PLY x-slab + the
    /// 9-cell zone × shell mean Ψ partition (intermediate steps drop
    /// these to keep memory bounded).
    final_step_data: Option<FinalStepData>,
}

#[derive(Clone, Debug)]
struct FinalStepData {
    rest_positions: Vec<Vec3>,
    pair_records: Vec<Value>,
    /// 3 zones × 3 shells = 9 cells, indexed `mean_psi_zone_shell[z][s]`
    /// where `z = 0/1/2` is distal/band/proximal and `s = 0/1/2` is
    /// inner/middle/outer.
    mean_psi_zone_shell: [[f64; 3]; 3],
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
        let mut per_tet_psi: Vec<f64> = Vec::with_capacity(tets.len());
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
                pair_records,
                mean_psi_zone_shell,
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
// Verifications — structural + physics gates
// =============================================================================
//
// Rule-B de-frag (mirrors rows 21 + 23): the pre-Rule-B captured-bit self-pins
// (per-shell + z-slab + zone×shell + x-slab count freezes, the per-step
// iter-count freeze, the per-step + final + 9 zone×shell force/displacement/Ψ̄
// `to_bits()` pins, the `to_yeoh()` additive-decomposition provenance mirror
// AND the blend-zone smoothstep self-mirror) were STRIPPED. Constitutive
// correctness (Yeoh closed form + additive decomposition + `to_yeoh()`
// round-trip) is lib-owned (`yeoh_contract.rs` + `silicone_table.rs` tests);
// per-shell routing is lib-owned (`sdf_material_tagging.rs` IV-4); the
// `BlendedScalarField` cubic-Hermite blend is lib-owned
// (`blended_material_composition.rs`). What survives is the scene's emergent
// physics read from the real 16-step ramp: valid mesh, fully-populated
// zone×shell decomposition, per-step convergence, force-displacement
// monotonicity, radial + axial strict Ψ̄ ordering, displacement bound, and
// per-shell material routing — robust to FP drift.

/// Structural mesh invariants — resolution-robust, no exact-count freeze.
fn verify_mesh_structure(
    mesh: &SdfMeshedTetMesh<Yeoh>,
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
}

/// z-slab populations — each shell contributes ≥ 1 tet to the `z = 0` cut.
fn verify_zslab_populations(inner_zslab: usize, middle_zslab: usize, outer_zslab: usize) {
    assert!(inner_zslab > 0, "z-slab inner shell empty");
    assert!(middle_zslab > 0, "z-slab middle shell empty");
    assert!(outer_zslab > 0, "z-slab outer shell empty");
}

/// NEW for row 24: 9-cell zone × shell partition sums to total.
/// The 3-zone × 3-shell decomposition is fully populated — every axial zone
/// (distal / band / proximal) × radial shell (inner / middle / outer) cell has
/// ≥ 1 tet, so the axial-zoning demonstration is meaningful. Structural
/// (non-empty), not exact-count (the specific counts are a mesher artifact).
fn verify_zone_shell_populations(n_zone_shell: [[usize; 3]; 3]) {
    for (z, row) in n_zone_shell.iter().enumerate() {
        for (s, &count) in row.iter().enumerate() {
            assert!(count > 0, "zone × shell cell [{z}][{s}] is empty");
        }
    }
}

/// The x-slab (`x = 0`) viz cut populates every zone × shell cell, so the
/// axial material gradient is visible across the full decomposition.
/// Structural (non-empty), not exact-count.
fn verify_xslab_zone_shell_populations(n_zone_shell_xslab: [[usize; 3]; 3]) {
    for (z, row) in n_zone_shell_xslab.iter().enumerate() {
        for (s, &count) in row.iter().enumerate() {
            assert!(count > 0, "x-slab zone × shell cell [{z}][{s}] is empty");
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

/// The final ramp step engaged contact — at least one active referenced-vertex
/// pair. (The exact count was a mesher/discretization artifact; non-empty is
/// the invariant that matters.)
fn verify_contact_engaged(results: &[RampStepResult]) {
    let final_step = results.last().expect("ramp produced no results");
    assert!(
        final_step.n_active_pairs > 0,
        "no active contact pairs at the final ramp step — probe did not engage the cavity wall",
    );
}

/// Per-tet material routing across BOTH the 3-shell radial partition AND the
/// proximal-pure / distal-pure axial zones — the scene's axial-zoning
/// invariant. Reads the real per-tet `Yeoh` from `mesh.materials()` via public
/// `.mu()` / `.c2()` / `.lambda()` and compares to the shell's F4 entry for the
/// tet's axial zone (proximal stack for proximal-pure tets, distal stack for
/// distal-pure) with an exact `to_bits()` `==` — a routing check, NOT a
/// constitutive mirror. Band tets are skipped (their material is a smoothstep
/// blend, whose cubic-Hermite kernel + per-tet composition is lib-owned:
/// `blended_material_composition.rs` + `sdf_material_tagging.rs` IV-4). Each
/// pure zone is verified non-vacuously (≥ 1 tet). The Yeoh closed form +
/// additive decomposition + the `to_yeoh()` round-trip are lib-owned
/// (`yeoh_contract.rs` + `silicone_table.rs::tests::
/// to_yeoh_round_trips_yeoh_fields_for_each_anchor`).
fn verify_material_routing(
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

    let assert_routed = |t: usize, zone: &str, shell_idx: usize, expected: &Yeoh| {
        let observed = &materials[t];
        assert!(
            observed.mu().to_bits() == expected.mu().to_bits()
                && observed.c2().to_bits() == expected.c2().to_bits()
                && observed.lambda().to_bits() == expected.lambda().to_bits(),
            "tet {t} {zone} shell {shell_idx}: routed (μ, C₂, λ) = ({}, {}, {}) != table ({}, {}, {})",
            observed.mu(),
            observed.c2(),
            observed.lambda(),
            expected.mu(),
            expected.c2(),
            expected.lambda(),
        );
    };

    let mut n_proximal_checked = 0usize;
    let mut n_distal_checked = 0usize;
    for (t, (&shell_idx, &zone_idx)) in shell_idx_per_tet
        .iter()
        .zip(zone_idx_per_tet.iter())
        .enumerate()
    {
        match zone_idx {
            // Distal-pure
            0 => {
                assert_routed(t, "distal", shell_idx, &distal_yeoh[shell_idx]);
                n_distal_checked += 1;
            }
            // Band — smoothstep-blended material, lib-owned; skip here.
            1 => {}
            // Proximal-pure
            _ => {
                assert_routed(t, "proximal", shell_idx, &proximal_yeoh[shell_idx]);
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

/// The row-24 headline: the zone × shell mean-Ψ̄ table obeys BOTH orderings at
/// the final (deepest) step — radial `Ψ̄_inner > Ψ̄_middle > Ψ̄_outer` within
/// each axial zone, AND axial `Ψ̄_proximal > Ψ̄_distal` within each shell (the
/// physical consequence of the soft-proximal / stiff-distal material gradient
/// under proximal-end contact). Structural orderings read from the real solve
/// — the pre-Rule-B captured-bit pins on the 9 cells were stripped.
fn verify_zone_shell_psi_final(results: &[RampStepResult]) {
    let final_step = results.last().expect("ramp produced no results");
    let data = final_step
        .final_step_data
        .as_ref()
        .expect("final step missing FinalStepData");
    let psi = &data.mean_psi_zone_shell;

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
// (PLY x-slab + F1.0/F1.3 inline scratch helpers retired at F1.2)
// =============================================================================
//
// The x-slab centroid PLY emit + F1.0/F1.3 scratch prototypes
// (`emit_xslab_ply`, `emit_boundary_surface_ply`, `emit_slab_cut_ply`,
// `volume_weighted_per_vertex_psi`, `slab_cut_intersect_edge`,
// `align_winding_to_axis`) were lifted into
// [`sim_soft::viz::boundary_surface`] + [`sim_soft::viz::slab_cut`]
// (public API at `sim/L0/soft/src/viz/mod.rs`) at F1.1 and the
// inline scratch helpers + the x-slab emit are dropped here at F1.2.
// The z-slab and x-slab per-cell POPULATION gates
// (`verify_zslab_populations`, `verify_xslab_zone_shell_populations`)
// survive as cheap centroid filters — no PLY emit, just centroid
// classification + non-empty invariants.

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

    // Quality + structure gates BEFORE the ramp (structural invariants on the
    // deterministic mesh + full zone × shell decomposition; no exact-count freeze).
    verify_quality_floors(&mesh);
    verify_mesh_structure(
        &mesh,
        &referenced,
        &bc.pinned_vertices,
        n_inner,
        n_middle,
        n_outer,
    );
    verify_zone_shell_populations(n_zone_shell);
    verify_material_routing(&mesh, &shell_idx_per_tet, &zone_idx_per_tet);

    // 6. Quasi-static intrusion ramp. The pre-F1.2 explicit
    // `drop(mesh)` was retired here — F1.2's
    // `sim_soft::viz::{boundary_surface, slab_cut}` calls at the
    // post-ramp PLY emit step need `&dyn Mesh<Yeoh>` access, so the
    // mesh stays alive through the ramp + emits and gets dropped at
    // function-end naturally. Mesh memory at this row's size
    // (~32 k vertices, ~75 k tets) is a few MB — negligible
    // relative to the solver's per-step working set.
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

    // 7. Per-step + final-step verifies (structural + physics gates).
    verify_per_step_solver_converges(&results);
    verify_force_displacement_monotone(&results);
    verify_per_step_strain_energy_ordering(&results);
    verify_per_step_max_disp_bounded(&results);
    verify_contact_engaged(&results);
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

    // PLY emits (final step only) via `sim_soft::viz` public API
    // (F1.2 retrofit at sim-soft viz arc — see `viz/mod.rs` +
    // `project_sim_soft_viz_arc.md`).
    let final_step = results.last().expect("ramp produced no results");
    let final_data = final_step
        .final_step_data
        .as_ref()
        .expect("final ramp step missing FinalStepData");
    let half_cell = 0.5 * CELL_SIZE;

    // z-slab + x-slab tet-COUNT regression gates. Cheap centroid
    // filter — no PLY emit — surviving as bit-equal regression
    // checks (z-slab from row 23 carry-through; x-slab from v3
    // axial differentiator). The PLY-emit-with-data path was
    // retired at F1.2; the count partitions stay because they
    // cost ~one centroid-comparison per tet and pin a useful
    // geometric invariant.
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
        if rest_centroid.z.abs() < half_cell {
            match shell_idx_per_tet[tet_idx] {
                0 => n_inner_z_carry += 1,
                1 => n_middle_z_carry += 1,
                _ => n_outer_z_carry += 1,
            }
        }
        if rest_centroid.x.abs() < half_cell {
            let s = shell_idx_per_tet[tet_idx];
            let z = zone_idx_per_tet[tet_idx];
            n_zone_shell_xslab[z][s] += 1;
        }
    }
    verify_zslab_populations(n_inner_z_carry, n_middle_z_carry, n_outer_z_carry);
    verify_xslab_zone_shell_populations(n_zone_shell_xslab);

    // Boundary surface + slab cut via `sim_soft::viz` public API.
    // Both helpers consume `&dyn Mesh<Yeoh>` and emit
    // `AttributedMesh` with the supplied per-tet scalars projected
    // to per-vertex via volume-weighted averaging. The slab cut
    // additionally interpolates per-vertex psi onto each cross-edge
    // intersection point. F1.5 will retrofit rows 20/22/23 to use
    // the same API; F1.6 ships row 25 with an open-mouth wrap that
    // exercises the open-boundary topology.
    let mut per_tet_scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    per_tet_scalars.insert("psi_j_per_m3", &final_step.per_tet_psi);

    let bd_ply_path = out_dir.join("sleeve_boundary_final.ply");
    let bd_attr = boundary_surface(&mesh, &per_tet_scalars).map_err(|e| anyhow::anyhow!("{e}"))?;
    save_ply_attributed(&bd_attr, &bd_ply_path, true)?;

    let slab_ply_path = out_dir.join("sleeve_slab_cut_x0_final.ply");
    let slab_attr = slab_cut(
        &mesh,
        Plane {
            axis: 0,
            value: 0.0,
        },
        &per_tet_scalars,
    )
    .map_err(|e| anyhow::anyhow!("{e}"))?;
    save_ply_attributed(&slab_attr, &slab_ply_path, true)?;

    // F2.2 design-mesh emits — `design_slab_cut` (x = 0 plane to match
    // the F1 emit; reveals the axial-zoned material partition along
    // the body's long axis) + `design_surface` (full 3D body via
    // marching cubes on the design SDF) + barycentric scalar interp
    // from the analysis tet mesh. Decouples display from sim per the
    // F2 viz arc; emits alongside the F1 boundary_surface + slab_cut
    // artifacts so both conventions stay available.
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

    let design_slab_path = out_dir.join("sleeve_design_slab_cut_x0_final.ply");
    let design_slab_attr = design_slab_cut(
        &body_for_viz,
        &mesh,
        Plane {
            axis: 0,
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
    // per ramp step (16 total — row 24 uses N_RAMP_STEPS=16 for finer
    // resolution on the axial-zoned material partition). amplify=10
    // makes Yeoh's mm-scale deformations visible. Reuse
    // `final_data.rest_positions` from the F1 emit block above — same
    // FinalStepData; no need to re-bind.
    let rest_positions = &final_data.rest_positions;
    let amplify = 10.0_f64;
    for step_result in &results {
        let step_idx = step_result.step;
        let step_displacement: Vec<Vec3> = (0..n_vertices)
            .map(|i| step_result.deformed_positions[i] - rest_positions[i])
            .collect();
        let mut step_scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
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
        "  out/sleeve_boundary_final.ply                          (full 3D body via sim_soft::viz::boundary_surface, per-vertex volume-weighted psi_j_per_m3 heatmap)"
    );
    println!(
        "  out/sleeve_slab_cut_x0_final.ply                       (cross-section at x = 0 via sim_soft::viz::slab_cut, marching-tet, per-vertex psi via linear interp)"
    );
    println!(
        "  out/sleeve_design_slab_cut_x0_final.ply                (cross-section at x = 0 via sim_soft::viz::design_slab_cut, marching-squares-filled on design SDF)"
    );
    println!(
        "  out/sleeve_design_surface_final.ply                    (full 3D body via sim_soft::viz::design_surface, marching-cubes on design SDF)"
    );
    println!(
        "  out/sleeve_design_surface_deformed_step_01.ply..step_16.ply  (F2.3c ramp animation series: per-step deformed body via sim_soft::viz::design_surface_deformed at amplify=10)"
    );
    println!();
    println!("View final-step PLYs in cf-view (workspace's unified visual-review viewer):");
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/out/sleeve_boundary_final.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/out/sleeve_slab_cut_x0_final.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/out/sleeve_design_slab_cut_x0_final.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/out/sleeve_design_surface_final.ply"
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
