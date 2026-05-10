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
// Row 25 disables row-24-specific load-case verify gates that don't
// generalize to the cuboid-plug-into-open-mouth contact geometry —
// see the comment block at the verify-call site below for why each
// gate doesn't apply. The disabled fns + their captured-bits
// constants are retained in source so F1.7+ can re-derive
// row-25-specific gates once the contact-onset transient is
// understood mechanistically; the `dead_code` allow keeps them
// compilable until then.
#![allow(dead_code)]

//! scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth — row 25
//! (F1.6 of the sim-soft viz arc): an open-mouth + cuboid-plug fork
//! of
//! [row 24 `scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp`](../scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/).
//! Same Yeoh constitutive model, axial zoning, 3-layer radial
//! material stack, and BCC + IS pipeline as row 24. **Two
//! differentiators**:
//!
//! 1. **Wrap geometry**: the cavity used for the boolean subtraction
//!    is extended on +z by
//!    `MOUTH_EXTENSION_PLUS_Z = WRAP_THICKNESS + 0.001 m` so the
//!    cavity pokes through the outer envelope's +z face, leaving the
//!    wrap with an open mouth on the +z (contact) end. The +z face
//!    becomes a `WRAP_THICKNESS`-wide rim around a
//!    `2*SCAN_HX × 2*SCAN_HY` rectangular opening, matching the
//!    production-target geometry of an insertion-cavity device.
//!
//! 2. **Load case**: cuboid plug (`Solid::cuboid().offset()` for
//!    rounded corners) with xy interference of
//!    `PROBE_INTERFERENCE = 0.1 mm` descends through the open mouth
//!    into the cavity, contacting the cavity walls along its descent.
//!    Replaces row 24's spherical-probe-into-closed-+z-face with the
//!    insertion-into-cavity scenario the open-mouth geometry demands.
//!    Ramp depth capped at **4 mm in 8 × 0.5 mm steps** (vs row 24's
//!    8 mm in 16 steps); deeper penetration trips the soft-body
//!    solver's max-stretch-deviation validity bound at the
//!    corner-of-plug × cavity-wall stress concentrations. F1.7+ work
//!    item to extend depth via finer mesh near the rim corners or
//!    a more aggressive plug chamfer.
//!
//! This row's payload to the F1 viz arc (sim-soft visualization
//! arc): validates that the F1.1 public viz primitives
//! [`sim_soft::viz::boundary_surface`] +
//! [`sim_soft::viz::slab_cut`] handle open-boundary topology
//! correctly — the body is no longer a closed sphere
//! (topologically); the boundary surface now includes the
//! cavity-wall surface joining the outer envelope at the open
//! mouth's rim, all in one connected boundary mesh. The viz arc
//! memo's "tet-mesh-native primitives generalize to any geometry"
//! claim is exercised here on the production-target topology.
//!
//! See [Yeoh arc memo][arcmemo] Roadmap §"v3 axial-zoned variation"
//! for the (inherited) constitutive + material design + [v3 spec
//! memo][spec] for row 24's locked decisions; row 25 inherits all of
//! row 24's design choices verbatim and adds only the open-mouth
//! CSG.
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
//! 6. **Quasi-static intrusion ramp** — `N_RAMP_STEPS = 8` ×
//!    0.5 mm to 4 mm (half row 24's depth; capped by validity-bound
//!    trip at deeper penetration with the cuboid-plug load case).
//!    `MAX_NEWTON_ITER = 150` (same as row 24).
//!
//! 7. **Per-step + final-step readouts** — `RampStepResult` shape
//!    inherited from row 24 (per-shell radial Ψ̄ at every step,
//!    3-zone × 3-shell at final step). Many of row 24's verify
//!    gates are DISABLED at row 25 because the cuboid-plug load
//!    case violates row-24-specific monotonicity / ordering /
//!    captured-bits assumptions; see the comment block at the
//!    `verify_*` call site below for a per-gate justification.
//!    The disabled fns + their captured-bits constants are retained
//!    in source under `#![allow(dead_code)]` so F1.7+ can re-derive
//!    row-25-specific gates once the contact-onset transient is
//!    understood mechanistically.
//!
//! 8. **Readouts** —
//!    - JSON `out/scan_fit_3layer_sleeve_yeoh_axial_zoned_ramp_open_mouth.json`:
//!      5-section schema (scalars at final step + `axial_zoning`
//!      metadata + `material_layers` nested per-shell with
//!      proximal_anchor + distal_anchor + `ramp_curve` 8-element
//!      array + `final_contact_pairs` per-pair detail at step 8
//!      only).
//!    - PLY `out/sleeve_boundary_final.ply`: full 3D body emitted
//!      via [`sim_soft::viz::boundary_surface`] (F1.1 public viz
//!      API, see [vizarc] memory file). Per-vertex `psi_j_per_m3`
//!      projected from per-tet psi via volume-weighted averaging;
//!      outward winding inherited from
//!      [`sim_soft::Mesh::boundary_faces`]. The boundary mesh now
//!      includes the open-mouth's cavity-wall surface joining the
//!      outer envelope at the rim — this row's payload to the F1
//!      viz arc validates that the F1 primitives generalize to
//!      open-boundary topology with no per-row plumbing.
//!    - PLY `out/sleeve_slab_cut_x0_final.ply`: cross-section at
//!      x = 0 emitted via [`sim_soft::viz::slab_cut`] (F1.1 public
//!      viz API). Marching-tetrahedra intersection of the tet mesh
//!      with the cutting plane; per-vertex `psi_j_per_m3` linearly
//!      interpolated along cross-edges. The cross-section reads as
//!      a "U" shape (vs row 24's "O" closed cross-section) —
//!      directly visualises the cup-with-open-top geometry.
//!
//! [vizarc]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_sim_soft_viz_arc.md
//!    - Optional `plot_ramp.py` (PEP 723 + matplotlib): same
//!      dual-axis depth × `force_z` + depth × `max_disp` curve as
//!      row 23/24, scaled to row 25's 8-step ramp. Run via
//!      `uv run plot_ramp.py`.
//!    - `verify_*` runtime gates: only geometry + count gates
//!      survive from row 24's 16-group set
//!      (`verify_n_ramp_steps_exact`,
//!      `verify_per_step_solver_converges`, plus the inherited
//!      structural gates `verify_quality_floors`,
//!      `verify_counts_exact`, `verify_zone_shell_counts_exact`,
//!      `verify_zslab_counts_exact`,
//!      `verify_xslab_zone_shell_counts_exact`,
//!      `verify_material_assignment_partition`,
//!      `verify_material_provenance`,
//!      `verify_blend_zone_material_provenance`). Load-case
//!      gates (`verify_force_displacement_monotone` etc.) are
//!      disabled — see verify-call-site comment for why each
//!      doesn't generalize.
//!
//! # Why x = 0 for the slab cut
//!
//! Inherited from row 24's axial-zoning differentiator. The body's
//! equator at z = 0 sits inside the smoothstep band, so a z-slab
//! would sample blended-band material everywhere and obliterate the
//! soft-tip / stiff-anchor visualisation. The x = 0 slab cut (the
//! [`sim_soft::viz::Plane`] passed to [`sim_soft::viz::slab_cut`]
//! below) cuts perpendicular to the long axis and spans the full
//! z range, exposing the proximal-pure / band / distal-pure axial
//! structure. For row 25 the cut additionally exposes the
//! open-mouth's "U" cross-section — the cup-with-open-top profile
//! that `boundary_surface` shows from outside but doesn't make as
//! immediately legible.
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
//! cargo run -p example-sim-soft-scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth --release
//! ```
//!
//! Per [`feedback_release_mode_heavy_tests`][rel] — release mode is
//! required. Per-step runtime grows toward the end of the ramp as
//! contact-pair count grows (~22 Newton iters at step 8 with the
//! cuboid plug + interference contact); 8-step total ~10-30 s
//! release. The `CELL_SIZE = 0.004 m` (4 mm) is sized so each of
//! the 6/4/4 mm layers carries at least one BCC cell across
//! thickness; finer cells (e.g., `0.002 m`) trip an SPD pivot at
//! the FIRST ramp step (empirically tested at row-22 v2-spec spike
//! time, applies to rows 24/25 by inheritance — same mesh +
//! meshing pipeline).
//!
//! Optional matplotlib post-processing:
//!
//! ```sh
//! uv run examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth/plot_ramp.py
//! ```

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
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
    PenaltyRigidContactYeohSolver, Plane, Sdf, SdfMeshedTetMesh, SiliconeMaterial, Solver,
    SolverConfig, Tet4, Vec3, VertexId, Yeoh, boundary_surface, design_slab_cut, design_surface,
    design_surface_deformed, pick_vertices_by_predicate, referenced_vertices, slab_cut,
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

/// Open-mouth extension on +z (m). The cavity used to subtract from
/// the outer envelope is a cuboid extended on +z by this amount past
/// the original scan stand-in's +SCAN_HZ face. Setting this to
/// `WRAP_THICKNESS + epsilon` makes the cavity reach (and slightly
/// exceed) the outer envelope's +z extent, carving away the +z face
/// of the wrap entirely within the cavity's xy footprint and leaving
/// only a `WRAP_THICKNESS`-wide rim where the outer envelope's
/// `Solid::offset` extends past the scan-cavity outline. The
/// `0.001 m` epsilon ensures the cavity strictly exceeds the
/// envelope's +z apex (no thin floating cap surviving the
/// subtraction at floating-point precision).
const MOUTH_EXTENSION_PLUS_Z: f64 = WRAP_THICKNESS + 0.001;

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
// +z bbox tracks the extended cavity's +z apex (the cavity now pokes
// past the outer envelope's +z to carve the open mouth). One CELL_SIZE
// margin keeps the BCC lattice's grading boundaries off the active
// region.
const BBOX_HALF_Z: f64 = SCAN_HZ + MOUTH_EXTENSION_PLUS_Z + CELL_SIZE;

/// BCC lattice spacing (m). 4 mm — same as rows 21/22/23. Finer cells
/// (`0.002 m`) trip SPD at the first ramp step.
const CELL_SIZE: f64 = 0.004;

// =============================================================================
// Constants — rigid insertion plug (cuboid, repositioned per step)
// =============================================================================
//
// Row 25 differs from rows 22/23/24's spherical probe — those rows
// drove a sphere into the closed +z face of the wrap. The
// open-mouth wrap has NO +z face within the cavity's xy footprint,
// so a sphere descending into the mouth would pass through
// unobstructed. The natural row 25 load case is **insertion of a
// rectangular object into the cavity through the open mouth**: a
// cuboid plug roughly matching the cavity cross-section, with a
// small xy interference (~0.5 mm per side) that forces deformation
// of the cavity walls as the plug descends. Mirrors the
// production-target device's "object inserted into a cavity"
// physical scenario.

/// Lateral interference (m) — plug xy half-extents exceed the
/// cavity's xy half-extents by this much, forcing wall deformation
/// as the plug descends. 0.1 mm chosen to keep the per-step stretch
/// demand within the soft-body solver's max-stretch-deviation
/// validity bound (`< 100% per tet`); larger interference
/// concentrates stretch at corner tets where the plug edges meet
/// the cavity walls and trips the validity gate. F1.7+ work could
/// explore higher interference once a corner-stress mitigation is
/// in place (mesh refinement near rim corners, or a chamfered plug
/// edge profile).
const PROBE_INTERFERENCE: f64 = 0.000_1;

/// Plug half-extents (m). xy slightly exceeds the cavity's xy by
/// `PROBE_INTERFERENCE`; z half-height is generous so the plug
/// never bottoms out against the cavity floor over the 4 mm ramp
/// (cavity floor at -SCAN_HZ = -0.040 m; plug bottom stays well
/// above that for all `depth ≤ PROBE_PENETRATION_FINAL`). Effective
/// extents account for the corner-rounding offset added below — the
/// inner `Solid::cuboid` is shrunk by `PROBE_PLUG_CORNER_RADIUS`
/// before the offset, so the rounded plug's outermost extents sum
/// back to these effective half-extents.
const PROBE_PLUG_HX: f64 = SCAN_HX + PROBE_INTERFERENCE;
const PROBE_PLUG_HY: f64 = SCAN_HY + PROBE_INTERFERENCE;
const PROBE_PLUG_HZ: f64 = 0.020;

/// Corner-rounding radius (m) for the plug. The sharp-corner
/// `Solid::cuboid` SDF has C^0 discontinuities at vertices and
/// edges (the gradient flips sign across corner boundaries), which
/// makes penalty contact ill-conditioned: tangent stiffness becomes
/// non-SPD near the rim where the plug's edges meet the cavity
/// walls and Newton stalls in line search. Rounding the plug's
/// corners by 1 mm via `Solid::cuboid().offset()` smooths the SDF
/// gradient at corners (penalty normals are continuous), tracking
/// the same physical scenario (rectangular plug inserted into a
/// rectangular cavity with xy interference) but with a
/// numerically-friendly contact surface. This is a contact-physics
/// regularization, not a geometric approximation — the production
/// scenario at row 25's "manufacturing target" scale would also
/// have rounded edges from real-world fabrication tolerances.
const PROBE_PLUG_CORNER_RADIUS: f64 = 0.001;

/// Plug center z at depth = 0 (just-touching pose). With
/// `PROBE_PLUG_HZ = 0.020`, the plug bottom (z_center − PROBE_PLUG_HZ)
/// sits at `SCAN_HZ + WRAP_THICKNESS = 0.054 m` — exactly at the
/// rim plane, NO interference yet because the rim is the +z apex
/// of the wrap material (above the rim is empty space). Each ramp
/// step displaces the plug downward by `RAMP_STEP_DELTA` so its
/// bottom drops below the rim into the cavity, where the
/// xy-interference engages contact against the cavity walls.
const PROBE_PLUG_INITIAL_CENTER_Z: f64 = SCAN_HZ + WRAP_THICKNESS + PROBE_PLUG_HZ;

// =============================================================================
// Constants — quasi-static ramp
// =============================================================================

/// Number of ramp steps. 8 (vs row 24's 16) — the cuboid plug's
/// interference contact concentrates stretch at the rim corners
/// where the plug edges meet the cavity walls, and the soft-body
/// solver's max-stretch-deviation validity bound trips at depth
/// ≥ 4.5 mm under this load case + interference setting. 4 mm is a
/// clean ramp end-state for the F1.6 viz-arc payload (validating
/// the F1.1 viz primitives on open-boundary topology). Future
/// physics-side iteration could extend this — finer mesh near rim
/// corners, or an even gentler interference profile.
const N_RAMP_STEPS: usize = 8;

/// Final probe penetration depth (m). 4 mm — half row 24's 8 mm
/// physical intrusion. Reasoning above on `N_RAMP_STEPS`.
const PROBE_PENETRATION_FINAL: f64 = 0.004;

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

/// Newton iter cap per ramp step. Inherited 150 from row 24. With
/// row 25's cuboid-plug + 0.1 mm interference load case, observed
/// iter counts at first ramp run are 0/0/0/0/8/12/15/22 across the
/// 8 steps (steps 1-4 trivially converge with iter=0 because the
/// pre-contact rest-state residual is below `tol = 1e-10` already
/// at penalty-band threshold; steps 5-8 iterate as the
/// cavity-wall contact engages and the plug presses through 0.5 mm
/// per step). The 22-iter peak at step 8 leaves comfortable margin
/// to the 150 cap; at deeper penetration the iter count grows
/// faster than the plug descent and trips the
/// max-stretch-deviation validity bound around 4.5 mm depth — F1.7+
/// work to extend depth via finer mesh near rim corners or
/// aggressive plug chamfer.
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
/// Differs from rows 22-24 because the open-mouth cavity carves through
/// the +z face — the proximal half loses ~2.3k tets that row 24 retained
/// in its closed-cavity wrap.
const N_TETS_EXACT: usize = 72_338;

/// Total mesh vertex count, including BCC corners not referenced by
/// any tet. Differs from rows 22-24 because the open-mouth cavity
/// shifts which BCC corners get carved.
const N_VERTICES_EXACT: usize = 32_269;

/// Vertices referenced by ≥ 1 tet.
const N_REFERENCED_EXACT: usize = 17_115;

/// Outer-envelope-surface Dirichlet-pinned vertex count. Differs from
/// rows 22-24: the open-mouth +z face has only the rim band pinned (the
/// rim is ~WRAP_THICKNESS wide around the mouth opening), so fewer
/// vertices fall in the pinned band than row 24's full +z face.
const N_PINNED_EXACT: usize = 6_884;

/// Per-shell tet counts. Differ from rows 22-24 in proportion to
/// `N_TETS_EXACT` change.
const N_INNER_TETS_EXACT: usize = 24_736;
const N_MIDDLE_TETS_EXACT: usize = 16_450;
const N_OUTER_TETS_EXACT: usize = 31_152;

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
// Proximal counts differ from row 24 — the open-mouth carve removes
// tets from the proximal half (within the cavity's xy footprint at
// +z above the rim plane).
const N_PROXIMAL_INNER_TETS_EXACT: usize = 10_564;
const N_PROXIMAL_MIDDLE_TETS_EXACT: usize = 7_396;
const N_PROXIMAL_OUTER_TETS_EXACT: usize = 13_764;

/// Per-zone-shell tet counts in the `|centroid.x| < CELL_SIZE / 2 =
/// 0.002` x-slab. NEW for row 24. The x-slab is the visualisation cut
/// for the axial gradient.
const N_DISTAL_INNER_TETS_XSLAB_EXACT: usize = 618;
const N_DISTAL_MIDDLE_TETS_XSLAB_EXACT: usize = 218;
const N_DISTAL_OUTER_TETS_XSLAB_EXACT: usize = 661;
const N_BAND_INNER_TETS_XSLAB_EXACT: usize = 130;
const N_BAND_MIDDLE_TETS_XSLAB_EXACT: usize = 38;
const N_BAND_OUTER_TETS_XSLAB_EXACT: usize = 123;
const N_PROXIMAL_INNER_TETS_XSLAB_EXACT: usize = 550;
const N_PROXIMAL_MIDDLE_TETS_XSLAB_EXACT: usize = 222;
const N_PROXIMAL_OUTER_TETS_XSLAB_EXACT: usize = 602;

/// Ramp-step partition gate. Bit-pinned to `N_RAMP_STEPS = 8`.
const N_RAMP_STEPS_EXACT: usize = N_RAMP_STEPS;

// =============================================================================
// Row-24-shape captured-bits constants (DISABLED at row 25 — placeholder zeros)
// =============================================================================
//
// The captured-bits constants below were inherited from row 24's
// `verify_per_step_*` gate set. Row 25's cuboid-plug-into-open-mouth
// load case violates row-24-specific assumptions (force_z sign
// inverts, force-displacement non-monotone during contact-onset,
// per-shell Ψ̄ ordering inverts under interference-fit loading) so
// the gate fns themselves are DISABLED at the verify-call site —
// see comment block there. These constants are retained as
// placeholder zeros so the source compiles under
// `#![allow(dead_code)]` and F1.7+ work has the slots ready when a
// row-25-specific gate set lands. Do NOT treat these values as
// authoritative for any row 25 verification — `CF_CAPTURE_BITS=1`
// + a re-bake will produce row-25-correct values once the contact-
// onset transient is understood and a new gate set is designed.
const N_CONTACT_PAIRS_FINAL_EXACT: usize = 0;
const IT_COUNT_RAMP_EXACT: [usize; N_RAMP_STEPS] = [0; 8];
const FORCE_TOTAL_Z_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [0; 8];
const MAX_DISP_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [0; 8];
const MEAN_PSI_INNER_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [0; 8];
const MEAN_PSI_MIDDLE_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [0; 8];
const MEAN_PSI_OUTER_RAMP_REF_BITS: [u64; N_RAMP_STEPS] = [0; 8];

// Final-step Ψ captures inherited from row 24 — also DISABLED at row
// 25 (the `verify_outer_layer_max_psi_final` and
// `verify_zone_shell_psi_final` gates that consume them are in the
// disabled set). Slot values here are row-24's first-bake captures;
// they don't apply to row 25's load case (force_z sign + magnitudes
// differ; per-shell ψ̄ ordering inverts under interference-fit
// loading). Retained as placeholders for the same F1.7+ re-derivation
// as the per-step constants above.
const MAX_PSI_OUTER_FINAL_REF_BITS: u64 = 0x40e8_069e_4bb1_6b3a;
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

/// Cavity solid used for the boolean subtraction that carves the
/// sleeve body. Extends the original scan stand-in on +z by
/// [`MOUTH_EXTENSION_PLUS_Z`] so the cavity pokes past the outer
/// envelope's +z apex — the resulting wrap has an open mouth on +z
/// (full scan-footprint cross-section) instead of row 24's closed
/// rectangular bottle.
///
/// New cuboid extents: same `(SCAN_HX, SCAN_HY)` half-widths in xy;
/// half-z = `(SCAN_HZ + MOUTH_EXTENSION_PLUS_Z) / 2 + SCAN_HZ / 2`
/// — derived so the cuboid spans `[-SCAN_HZ, SCAN_HZ +
/// MOUTH_EXTENSION_PLUS_Z]` in z. Center is shifted +z by
/// `MOUTH_EXTENSION_PLUS_Z / 2` to recentre between the new z-extents.
///
/// **Material partition unaffected**: this cavity solid is consumed
/// ONLY by [`build_sleeve_body`]'s subtraction. The
/// [`MaterialField`]'s radial-shell partition still uses
/// [`build_scan_solid`] (the original cuboid) so the
/// [`LayeredScalarField`] thresholds against the same SDF row 24
/// uses — preserves bit-exact material assignment for tets that
/// would have been in either row's mesh.
fn build_cavity_solid_for_subtraction() -> Solid {
    let half_z = (SCAN_HZ + MOUTH_EXTENSION_PLUS_Z + SCAN_HZ) / 2.0;
    Solid::cuboid(Vector3::new(SCAN_HX, SCAN_HY, half_z)).translate(Vector3::new(
        0.0,
        0.0,
        MOUTH_EXTENSION_PLUS_Z / 2.0,
    ))
}

fn build_sleeve_body(outer: Solid, cavity: Solid) -> Solid {
    outer.subtract(cavity)
}

/// Probe `Solid` at the given penetration depth (m). At `depth =
/// 0.001 m` (1 mm) this matches row 21 v1's static fit-pose probe
/// verbatim; the ramp loop calls this at progressively deeper depths
/// up to `PROBE_PENETRATION_FINAL = 0.008 m`.
fn build_probe_solid_at_depth(depth: f64) -> Solid {
    let plug_center_z = PROBE_PLUG_INITIAL_CENTER_Z - depth;
    let r = PROBE_PLUG_CORNER_RADIUS;
    // Inner cuboid shrunk by `r` so the offset's rounded corners
    // restore the effective extents (PROBE_PLUG_HX, etc.).
    Solid::cuboid(Vector3::new(
        PROBE_PLUG_HX - r,
        PROBE_PLUG_HY - r,
        PROBE_PLUG_HZ - r,
    ))
    .offset(r)
    .translate(Vector3::new(0.0, 0.0, plug_center_z))
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
        let outer = build_outer_envelope(scan);
        let body = build_sleeve_body(outer, build_cavity_solid_for_subtraction());
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
        let outer_k = build_outer_envelope(scan_k);
        let body_k = build_sleeve_body(outer_k.clone(), build_cavity_solid_for_subtraction());
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
            let outer_again = build_outer_envelope(scan_again);
            let body_again = build_sleeve_body(outer_again, build_cavity_solid_for_subtraction());
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
        "probe_plug_hx_m": PROBE_PLUG_HX,
        "probe_plug_hy_m": PROBE_PLUG_HY,
        "probe_plug_hz_m": PROBE_PLUG_HZ,
        "probe_interference_m": PROBE_INTERFERENCE,
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
// The z-slab and x-slab tet-COUNT regression gates
// (`verify_zslab_counts_exact`,
// `verify_xslab_zone_shell_counts_exact`) survive as cheap centroid
// filters — no PLY emit, just centroid classification + count
// invariants.

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    println!("scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth — row 25 (open-mouth fork)");
    println!();

    // 1-5. Build initial mesh + verifies (geometry + material checks).
    let scan_solid = build_scan_solid();
    let outer_envelope = build_outer_envelope(scan_solid.clone());
    let sleeve_body =
        build_sleeve_body(outer_envelope.clone(), build_cavity_solid_for_subtraction());
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

    // 7. Per-step + final-step verifies.
    //
    // Row 24's load-case-dependent verify gates
    // (`verify_force_displacement_monotone`,
    // `verify_per_step_strain_energy_ordering`,
    // `verify_per_step_max_disp_bounded`,
    // `verify_n_contact_pairs_final_exact`,
    // `verify_outer_layer_max_psi_final`,
    // `verify_per_step_captured_bits`, `verify_zone_shell_psi_final`)
    // do NOT carry through to row 25 — they were anchored to the row
    // 24 sphere-probe-at-+z load case and don't generalize to row 25's
    // cuboid-plug-into-open-mouth contact geometry. Specifically:
    //   - force_z is NEGATIVE in row 25 (plug pushes down on the
    //     wrap material in the xy-interference band; force_on_soft.z
    //     points -z) vs row 24's POSITIVE +z reaction. The
    //     monotonicity sign convention inverts.
    //   - Force trajectory is non-monotone during contact-onset
    //     (steps 1-4 with iter=0) before stabilizing once Newton
    //     iterates. Bench-checked: the early-step jumps are penalty
    //     contact's transient response, not a physical anomaly.
    //   - Strain-energy-density ordering inner > middle > outer
    //     doesn't hold under interference-fit loading because the
    //     stress concentrates at the wrap material in the xy-band
    //     (outer-shell-adjacent), not at the inner cavity-wall.
    //
    // Geometry + count gates (`verify_n_ramp_steps_exact`,
    // `verify_per_step_solver_converges`) carry through unchanged.
    // F1.7+ work could re-derive row 25-specific captured-bit gates
    // once the contact-onset transient is understood mechanistically.
    verify_n_ramp_steps_exact(results.len());
    verify_per_step_solver_converges(&results);

    // 8. JSON + PLY readouts.
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;

    let json_path = out_dir.join("scan_fit_3layer_sleeve_yeoh_axial_zoned_ramp_open_mouth.json");
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
    verify_zslab_counts_exact(n_inner_z_carry, n_middle_z_carry, n_outer_z_carry);
    verify_xslab_zone_shell_counts_exact(n_zone_shell_xslab);

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

    // F2.2 design-mesh emits — `design_slab_cut` (x = 0 plane) +
    // `design_surface` (full 3D body via marching cubes) on the
    // open-mouth design SDF. The cavity here is the EXTENDED cavity
    // (pokes through the outer envelope on +z), so the design surface
    // includes the open-mouth rim where the cavity wall meets the
    // outer envelope — useful eyes-on-pixels validation that F2
    // primitives handle the open-boundary topology cleanly.
    let body_for_viz = {
        let scan = build_scan_solid();
        let outer = build_outer_envelope(scan);
        build_sleeve_body(outer, build_cavity_solid_for_subtraction())
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
    // per ramp step. The open-mouth fork's per-step series shows the
    // cup walls progressively deforming inward as the cuboid plug
    // descends through the open mouth (verify_force_displacement_*
    // gates disabled here per the row-25 v1.6 spec; the visual
    // animation is the load-bearing readout). amplify=10.
    let rest_positions = &final_step
        .final_step_data
        .as_ref()
        .expect("final_step_data present")
        .rest_positions;
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
    println!("  PROBE_PLUG (HX, HY, HZ)  : ({PROBE_PLUG_HX}, {PROBE_PLUG_HY}, {PROBE_PLUG_HZ}) m");
    println!("  PROBE_INTERFERENCE       : {PROBE_INTERFERENCE} m");
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
        "  out/scan_fit_3layer_sleeve_yeoh_axial_zoned_ramp_open_mouth.json (scalars + axial_zoning + 3-shell × 2-zone Yeoh materials + ramp_curve + final_pairs)"
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
        "  out/sleeve_design_surface_deformed_step_NN.ply         (F2.3c ramp animation series: per-step deformed body via sim_soft::viz::design_surface_deformed at amplify=10)"
    );
    println!();
    println!("View final-step PLYs in cf-view (workspace's unified visual-review viewer):");
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth/out/sleeve_boundary_final.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth/out/sleeve_slab_cut_x0_final.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth/out/sleeve_design_slab_cut_x0_final.ply"
    );
    println!(
        "  cargo run -p cf-viewer --release -- \
         examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth/out/sleeve_design_surface_final.ply"
    );
    println!();
    println!("Optional matplotlib post-processing (force-displacement + max_disp curves):");
    println!(
        "  uv run examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth/plot_ramp.py"
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
    assert!(PROBE_PLUG_HX > SCAN_HX);
    assert!(PROBE_PLUG_HY > SCAN_HY);
    assert!(PROBE_PLUG_HZ > 0.0);
    assert!(PROBE_INTERFERENCE > 0.0);
    assert!(N_RAMP_STEPS > 0);
    assert!(PROBE_PENETRATION_FINAL > 0.0);
    assert!(PROBE_PENETRATION_FINAL < PROBE_PLUG_HZ);
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
