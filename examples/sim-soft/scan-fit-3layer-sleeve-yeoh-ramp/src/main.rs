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
//! 5. **3-layer `MaterialField`** via
//!    `MaterialField::from_yeoh_fields_with_bounds` (five scalar
//!    fields: μ, C₂, λ + the per-anchor calibrated tensile /
//!    compressive validity caps — `C₁ = μ/2` is derived per arc memo
//!    D2). The two extra bound fields are load-bearing at 8 mm: the
//!    bounds-less `from_yeoh_fields` would leave each per-tet Yeoh's
//!    validity `None`, falling through to the legacy Neo-Hookean
//!    symmetric ceiling `max_stretch_deviation = 1.0` (σ ∈ [0, 2]),
//!    which the deep-penetration solve grazes near step 16 even
//!    though a Yeoh silicone's real envelope is `0.8·λ_break ≈ 6-9`.
//!    Inner = Path-2 parametric construction via
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
//!    The per-step force / displacement / Ψ̄ series drive the
//!    force-displacement monotonicity + strict Ψ̄-ordering gates and
//!    the JSON `ramp_curve`; the final step (step 16 at 8 mm) drives
//!    the PLY viz artifacts.
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
//!    - `verify_*` runtime gates — pipeline-emergent structural +
//!      physics invariants (see "Numerical anchors" in `README.md`).
//!      This row is a Rule-B `validator`: its oracles are read from
//!      the real 16-step ramp (positive tet volume, per-step
//!      convergence, force-displacement monotonicity, strict Ψ̄
//!      ordering, displacement bound, per-shell material routing) —
//!      the pre-Rule-B captured-bit self-pins were stripped
//!      (constitutive + mesher correctness is lib-owned; see the
//!      de-frag note over the `Verifications` section).
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
/// constitutive). 150 is a hedge with empirical headroom; the per-step
/// convergence gate (`verify_per_step_solver_converges`) asserts every step
/// stays under this cap (observed max is 77 iters at step 16).
const MAX_NEWTON_ITER: usize = 150;

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
    // Per-anchor calibrated Yeoh validity bounds (tensile `0.8·λ_break`,
    // compressive `0.20`), lifted from the F4 `SiliconeMaterial` anchors into
    // two partition fields exactly like μ/c2/λ. WITHOUT these, the bounds-less
    // `from_yeoh_fields` leaves each per-tet Yeoh's validity `None`, so it
    // falls through to the LEGACY Neo-Hookean symmetric ceiling
    // `max_stretch_deviation = 1.0` (σ ∈ [0, 2]) — ~3× tighter than silicone's
    // real ~6-9 envelope. That NH placeholder is what trips the solver's
    // fail-closed validity gate near the 8 mm target (a contact-zone tet grazes
    // σ ≈ 2.0), even though a Yeoh silicone stretches far past 100 %. Routing
    // the real caps via `from_yeoh_fields_with_bounds` (which closes the
    // "MaterialField-drops-bounds" gap, material_field.rs) gives the deep-
    // penetration solve its true ~3-4× stretch margin. Physics unchanged: the
    // bound only governs the fail-closed gate, not the energy/force response.
    let max_stretch_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![
            inner.validity_max_principal_stretch,
            DRAGON_SKIN_10A.validity_max_principal_stretch,
            DRAGON_SKIN_20A.validity_max_principal_stretch,
        ],
    ));
    let min_stretch_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        scan_for_partition(),
        vec![LAYER_INNER, LAYER_MIDDLE_OUTER],
        vec![
            inner.validity_min_principal_stretch,
            DRAGON_SKIN_10A.validity_min_principal_stretch,
            DRAGON_SKIN_20A.validity_min_principal_stretch,
        ],
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
        roller_vertices: Vec::new(),
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
// Verifications — structural + physics gates
// =============================================================================
//
// Rule-B de-frag (mirrors row 21): the pre-Rule-B captured-bit self-pins
// (per-shell + z-slab + contact-pair count freezes, the per-step iter-count
// freeze, the 80 per-step + final force/displacement/Ψ̄ `to_bits()` pins, the
// `to_yeoh()` additive-decomposition provenance mirror) were STRIPPED — they
// froze one run's FP trajectory on one toolchain. Constitutive correctness
// (Yeoh closed form + additive decomposition + `to_yeoh()` round-trip) is
// lib-owned (`yeoh_contract.rs` + `silicone_table.rs::tests::
// to_yeoh_round_trips_yeoh_fields_for_each_anchor` +
// `..._from_effective_shore_at_anchor_position_returns_anchor_data`); per-shell
// routing is lib-owned (`sdf_material_tagging.rs` IV-4). What survives is the
// scene's emergent physics read from the real 16-step ramp: valid mesh,
// per-step convergence, per-step force-displacement monotonicity, per-step
// strict Ψ̄ ordering, per-step displacement bound, and per-shell material
// routing — robust to FP drift.

/// Structural mesh invariants — resolution-robust, no exact-count freeze
/// (see the row-21 template `verify_mesh_structure`).
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
    // at every ramp step. The 8 mm final depth produces ~8.5 mm peak
    // body-wide displacement, still well under 14 mm. (The penalty
    // equilibrium can push the cavity wall farther than the rigid
    // penetration depth — that's expected behaviour.)
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

/// Per-tet material routing — the `MaterialField` assigned every tet the
/// `(μ, C₂, λ)` of the shell its centroid falls in by distance-from-scan.
/// Reads the real per-tet `Yeoh` from `mesh.materials()` via the public
/// `.mu()` / `.c2()` / `.lambda()` accessors and compares to the shell's F4
/// table entry with an exact `to_bits()` `==` (both come from the same anchor
/// through the same `to_yeoh()`, so a correctly routed tet is bit-identical —
/// a routing check, NOT a constitutive-arithmetic mirror). The Yeoh closed
/// form + additive decomposition + the `to_yeoh()` round-trip are lib-owned
/// (`yeoh_contract.rs` + `silicone_table.rs::tests::
/// to_yeoh_round_trips_yeoh_fields_for_each_anchor`), as is the inner layer's
/// Path-2 `from_effective_shore` weight-1.0 provenance
/// (`..._from_effective_shore_at_anchor_position_returns_anchor_data`). Each
/// shell is verified non-vacuously (≥ 1 tet).
fn verify_material_routing(mesh: &SdfMeshedTetMesh<Yeoh>, shell_idx_per_tet: &[usize]) {
    let materials = mesh.materials();
    assert_eq!(
        materials.len(),
        shell_idx_per_tet.len(),
        "materials() length does not match per-tet shell-classification length",
    );
    let expected_yeoh = [
        inner_silicone().to_yeoh(),
        DRAGON_SKIN_10A.to_yeoh(),
        DRAGON_SKIN_20A.to_yeoh(),
    ];
    let mut checked = [0usize; 3];
    for (t, &shell_idx) in shell_idx_per_tet.iter().enumerate() {
        let observed = &materials[t];
        let expected = &expected_yeoh[shell_idx];
        assert!(
            observed.mu().to_bits() == expected.mu().to_bits()
                && observed.c2().to_bits() == expected.c2().to_bits()
                && observed.lambda().to_bits() == expected.lambda().to_bits(),
            "tet {t} shell {shell_idx}: routed (μ, C₂, λ) = ({}, {}, {}) != table ({}, {}, {})",
            observed.mu(),
            observed.c2(),
            observed.lambda(),
            expected.mu(),
            expected.c2(),
            expected.lambda(),
        );
        checked[shell_idx] += 1;
    }
    assert!(
        checked.iter().all(|&c| c > 0),
        "material routing not exercised on every shell (per-shell tet counts {checked:?})",
    );
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
// rather than an amplified centroid cloud. The z-slab per-shell
// population gate (`verify_zslab_populations`) survives — cheap
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
    verify_mesh_structure(
        &mesh,
        &referenced,
        &bc.pinned_vertices,
        n_inner,
        n_middle,
        n_outer,
    );
    verify_material_routing(&mesh, &shell_idx_per_tet);

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

    // 7. Per-step verifies (structural + physics gates read from the ramp).
    verify_per_step_solver_converges(&results);
    verify_force_displacement_monotone(&results);
    verify_per_step_strain_energy_ordering(&results);
    verify_per_step_max_disp_bounded(&results);
    verify_contact_engaged(&results);

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
    verify_zslab_populations(n_inner_z, n_middle_z, n_outer_z);

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
};
