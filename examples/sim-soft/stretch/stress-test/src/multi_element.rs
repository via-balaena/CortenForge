//! multi-element — Phase 2 multi-element FEM assembly under uniform
//! Dirichlet stretch.
//!
//! Solver-driven 48-tet hex-grid (`uniform_block(2, 0.1, ...)`) under
//! uniform Dirichlet stretch on the 26 boundary vertices, with the
//! single interior vertex (ID 13 at the cube's center) free to converge
//! under Newton. The headline claim — **per-element deformation
//! gradient `F` is uniform `diag(λ, 1, 1)` across all 48 tets within
//! FP tolerance** — is the canonical "homogeneous Dirichlet boundary
//! data uniquely determines the homogeneous deformation" mechanics
//! result, here gated on the multi-element FEM assembly path doing
//! the work end-to-end (per-tet mass + sparse pattern at construction;
//! per-iter `assemble_global_int_force` + `assemble_free_hessian_triplets`
//! + sparse Cholesky on the 3-DOF free-block).
//!
//! Per inventory Q4 row 6 visualization, JSON-only (no `cf-view`,
//! no spatial artifact); the per-tet uniformity record IS the readout
//! signal. Per `feedback_math_pass_first_handauthored`, a clean
//! `cargo run --release` exit-0 IS the correctness signal — every claim
//! sits behind an `assert_*` in a `verify_*` anchor group below.
//!
//! # Quasi-static — `density = 0`
//!
//! Backward Euler with non-zero mass and `v_prev = 0` would not
//! converge to the pure-static homogeneous deformation in one step:
//! the `(M / Δt²)·(x − x_prev)` inertia recall pulls v_13 toward its
//! rest position, so the converged v_13 sits between rest and
//! `D · v_13_rest`. Setting `density = 0` zeros the lumped mass entirely
//! and reduces the backward-Euler residual to pure elasticity
//! (`f_int(x) = f_ext = 0`); the static minimizer of the homogeneous-NH
//! action under affine boundary data is `x = D · X` exactly, recovered
//! to within Newton's `tol = 1e-10` N residual bound. This is the
//! standard "quasi-static FEM" technique: we want the elasticity
//! equilibrium, so we suppress inertia.
//!
//! # Deformation gradient
//!
//! `F = diag(λ, 1, 1)` constrained — same Lamé pair as row 5
//! (the `neo_hookean` module) and the same anchor stretch
//! `λ = LAMBDA_STRETCH = 1.20`, but here imposed via Dirichlet pinning
//! of every boundary vertex to `D · X_rest` rather than evaluated
//! directly. The transverse stress `P_22 = Λ ln(λ) ≠ 0` reflects the
//! constrained-transverse semantics — this is *not* row 5's
//! traction-free path; here `λ_t = 1` is *prescribed* by the boundary
//! pin, not solved for.
//!
//! # Closed forms (compressible NH, μ shear, Λ first-Lamé) at `F = diag(λ, 1, 1)`
//!
//! ```text
//!     I_1   = λ² + 2
//!     J     = λ
//!     ψ     = (μ/2)(λ² − 1) + (Λ/2)(ln λ)² − μ ln λ
//!     P_11  = μ(λ − 1/λ) + Λ ln(λ) / λ
//!     P_22  = P_33 = Λ ln(λ)              (NOT zero — constrained, not traction-free)
//!     F_ij  = 0  for i ≠ j               (diagonal F)
//! ```
//!
//! The genuine oracle here is the **multi-element FEM ASSEMBLY** — that
//! homogeneous Dirichlet boundary data, stitched local→global across all 48
//! tets and solved on the free block, reproduces the uniform deformation. That
//! is example-only: `tests/uniaxial_fem_coupon.rs` covers the single-element
//! homogeneous-reproduction, but not the all-48-tet uniformity. The
//! *constitutive* law at `F = diag(λ, 1, 1)` (the `P`/`ψ` closed form) is owned
//! by the `sim-soft` `NeoHookean` lib tests (`diag(s,1,1)` + `diag(a,b,b)`), so
//! this module reads the real per-tet `P`/`ψ` into the JSON but does not
//! re-assert their values against the closed form.
//!
//! # Anchor groups (all assertions exit-0 on success)
//!
//! - **Referenced-vertices canonical pattern** — `referenced_vertices(&mesh)` returns `[0..27)`;
//!   pinning via `pick_vertices_by_predicate` + orphan-filter is the canonical idiom for
//!   spatial-predicate BC construction (no-op orphan-filter on hand-built meshes;
//!   load-bearing for SDF-meshed scenes per `tests/sdf_forward_map_gradcheck.rs`).
//! - **Mesh topology** — `n_vertices == 27`, `n_tets == 48`, interior vertex ID == 13.
//! - **Boundary partition** — 26 boundary vertices pinned, 1 interior free (vertex ID 13).
//! - **Validity in-domain at `x_prev`** — `max |σᵢ − 1|` per tet stays below NH's
//!   declared `max_stretch_deviation` boundary at the off-equilibrium initial guess.
//! - **Solver converges** — `iter_count < cfg.max_newton_iter`; `final_residual_norm < cfg.tol`.
//! - **Pinned `x_final` exact** — every pinned vertex's xyz stay bit-equal to `D · X_rest`.
//! - **Interior `x_final` near homogeneous** — vertex 13 lands at `D · X_rest[13]` within
//!   relative `1e-12` (Newton-convergence noise floor).
//! - **Per-element `F` uniform diag** (the assembly headline) — every tet's `F` matches
//!   `diag(λ, 1, 1)` at relative `1e-12` (or absolute `1e-12` for the off-diagonal-zero entries).
//! - **Uniformity spread bounded** — `max P_11 − min P_11` across all 48 tets within
//!   a tight FP-noise bound (Newton convergence + sparse-solve drift).

#![allow(
    // `doc_markdown` flags Unicode math notation (`λ`, `μ`, `Λ`, `σᵢ`, `ψ`)
    // as if they were unbacktrick-quoted code identifiers. Same allowance
    // as row 5.
    clippy::doc_markdown,
    // `try_inverse().expect(...)` on `J_0` for the canonical mesh is
    // contractually safe in this scene (`uniform_block` produces
    // right-handed tets per its constructor doc). Same pattern as
    // `multi_element_grad_scaling.rs`'s file-level `expect_used` allow.
    clippy::expect_used,
    // `usize as u32` on `mesh.n_tets()` (max 48 here, < u32::MAX) — the
    // standard Mesh-trait API tax mirrored in `mesh::referenced_vertices`
    // and the solver assembly methods (see `backward_euler.rs:212` and
    // `mesh/hand_built.rs:382`).
    clippy::cast_possible_truncation,
    // `print_summary` is a museum-plaque stdout writer — splitting into
    // sub-helpers fragments the visual format without information gain.
    // Same allowance as row 4 / row 5's print_summary.
    clippy::too_many_lines
)]

use std::path::Path;

use anyhow::{Context, Result};
use approx::assert_relative_eq;
use nalgebra::Matrix3;
use serde_json::json;
use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTape, CpuTet4NHSolver, HandBuiltTetMesh,
    InversionHandling, Material, MaterialField, Mesh, NeoHookean, NewtonStep, NullContact, Solver,
    SolverConfig, Tet4, Vec3, VertexId, pick_vertices_by_predicate, referenced_vertices,
};

// =============================================================================
// Constants — canonical Ecoflex-class compressible NH (`ν ≈ 0.4`).
// =============================================================================

/// Shear modulus (Pa). Same value as row 5 / row 4 / `SoftScene::one_tet_cube`.
const MU_PA: f64 = 1.0e5;

/// First Lamé parameter (Pa). `Λ = 4μ` ⇒ `ν = 0.4` compressible NH.
const LAMBDA_PA: f64 = 4.0e5;

/// Cells per cube edge — `uniform_block` requires `n >= 2` and even.
/// `n = 2` is the smallest legal value AND the smallest mesh that has a
/// truly interior vertex (the cube's center at vertex ID 13 below).
/// Larger `n` would balloon the bit-pin scope without strengthening the
/// uniformity claim — single interior at 1 is sufficient to gate
/// "homogeneous deformation under affine boundary data" pedagogy.
const N_PER_EDGE: usize = 2;

/// Cube edge length (m) — decimeter scale per the walking-skeleton scope §2
/// convention (`SingleTetMesh::new`'s 0.1m), continued into rows 4, 5.
const EDGE_LEN: f64 = 0.1;

/// Total vertex count = `(N_PER_EDGE + 1)^3 = 27`.
const N_VERTICES: usize = 27;

/// Total tet count = `6 · N_PER_EDGE^3 = 48` (Coxeter-Freudenthal-Kuhn 6-tets-per-cell).
const N_TETS: usize = 48;

/// Total free-DOF-candidate count `3 · N_VERTICES = 81` (= total DOFs;
/// the actual `n_free` is 3 since 26 of 27 vertices are pinned).
const N_DOF: usize = 3 * N_VERTICES;

/// Vertex ID of the interior vertex `(1, 1, 1)` in the `uniform_block(2, ...)`
/// layout. Stride formula: `vid(i, j, k) = i + j·(nx+1) + k·(nx+1)·(ny+1)`
/// with `nx = ny = nz = 2`, so `vid(1,1,1) = 1 + 3 + 9 = 13`.
const INTERIOR_VERTEX_ID: VertexId = 13;

/// Boundary vertex count = `N_VERTICES − 1` (one interior).
const N_BOUNDARY_VERTICES: usize = N_VERTICES - 1;

/// Prescribed uniaxial stretch `λ = F_11` along `+x̂`. Same anchor stretch as
/// row 5's tensile working point for cross-row continuity (and `|λ − 1| = 0.20`
/// stays well inside NH's `max_stretch_deviation = 1.0` validity boundary).
const LAMBDA_STRETCH: f64 = 1.20;

/// Spatial-predicate epsilon for boundary detection (m). Mesh vertex coordinates
/// are computed as `i · 0.05` and `2 · 0.05 = 0.1` — exact in IEEE-754, so any
/// epsilon below the inter-vertex spacing (`0.05` m) catches boundary verts
/// without ambiguity. `1e-12` mirrors the JSON anchor tolerance.
const BOUNDARY_PREDICATE_EPS: f64 = 1.0e-12;

/// Closed-form-vs-observed relative tolerance — same value as row 5.
/// Both the per-tet `F = J · J_0^-1` and `Material::first_piola(F)` paths do
/// `nalgebra::Matrix3` arithmetic; observed agreement is at the few-ULP level
/// (~1e-15). `1e-12` admits expression-tree reordering noise + sparse-solver
/// SIMD/FMA noise on the v_13 free-DOF without flapping.
const REL_TOL: f64 = 1.0e-12;

/// Absolute floor (Pa, m, or J/m³ — context-dependent) for relative
/// comparisons that touch zero. `1e-12` is below typical NH stress
/// magnitudes (~1e5 Pa) by 17 orders.
const EPS_ABS: f64 = 1.0e-12;

/// Bound on `max P_11 − min P_11` across all 48 tets at convergence (Pa).
/// Newton tol is `1e-10` N residual, propagating through the 3-DOF
/// elastic tangent (~1e4 N/m at this configuration) into ~1e-14 m position
/// drift on v_13, then through the per-tet `F = J · J_0^-1` (with
/// `J_0^-1` ~ 20 m⁻¹) into ~1e-13 strain drift, then through the NH
/// tangent (~1e5 Pa per unit strain) into ~1e-8 Pa stress drift. Bound
/// at `1e-6` Pa = 11 orders below the closed-form `P_11(1.20) ≈ 9.7e4` Pa
/// — six orders of slack above the noise floor, six orders below any
/// real regression.
const UNIFORMITY_SPREAD_BOUND_PA: f64 = 1.0e-6;

// =============================================================================
// Per-element F from positions: F = J · J_0^-1
// =============================================================================
//
// The solver computes `F` per tet via `F_ij = Σ_a x_{a,i} · ∂N_a/∂X_j` (the
// shape-gradient form), with `∂N_a/∂X_j = grad_xi N_a · J_0^-1`. The
// equivalent closed-form expression for Tet4 (constant gradients) is
// `F = J · J_0^-1` where `J = [v_1−v_0, v_2−v_0, v_3−v_0]` (current) and
// `J_0` = same expression at rest. This example uses the closed form
// inline since the solver's `deformation_gradient` is private to its
// module; the math is 5 lines, exercises the per-tet local-vs-global
// stitch via `mesh.tet_vertices(tet_id)` indexing, and produces the
// same `F` (in scalar IEEE-754 arithmetic) as the solver's hot path.

/// Per-tet `F = J · J_0^-1` where columns of `J` are vertex differences
/// from `verts[0]` in the current configuration, columns of `J_0` are
/// the same in the rest configuration.
fn tet_deformation_gradient(
    rest_positions: &[Vec3],
    curr_flat: &[f64],
    verts: [VertexId; 4],
) -> Matrix3<f64> {
    let v_idx = |a: usize| verts[a] as usize;
    let curr = |a: usize| {
        let i = v_idx(a);
        Vec3::new(curr_flat[3 * i], curr_flat[3 * i + 1], curr_flat[3 * i + 2])
    };
    let r0 = rest_positions[v_idx(0)];
    let r1 = rest_positions[v_idx(1)];
    let r2 = rest_positions[v_idx(2)];
    let r3 = rest_positions[v_idx(3)];
    let c0 = curr(0);
    let c1 = curr(1);
    let c2 = curr(2);
    let c3 = curr(3);
    let j_0 = Matrix3::from_columns(&[r1 - r0, r2 - r0, r3 - r0]);
    let j = Matrix3::from_columns(&[c1 - c0, c2 - c0, c3 - c0]);
    let j_0_inv = j_0
        .try_inverse()
        .expect("singular reference jacobian — malformed rest mesh");
    j * j_0_inv
}

/// `max |σᵢ − 1|` over the singular values of `F`. Same metric the
/// solver's `check_validity_at_step_start` evaluates per tet.
fn max_stretch_deviation(f: &Matrix3<f64>) -> f64 {
    let svd = f.svd_unordered(false, false);
    svd.singular_values
        .iter()
        .map(|s| (s - 1.0).abs())
        .fold(0.0_f64, f64::max)
}

// =============================================================================
// Scene + solver builder
// =============================================================================

/// Apply the Dirichlet stretch `D = diag(λ, 1, 1)` to a rest position.
fn stretch(rest: Vec3, lambda: f64) -> Vec3 {
    Vec3::new(lambda * rest.x, rest.y, rest.z)
}

/// Spatial predicate: a vertex is on the cube's boundary if any
/// coordinate is at `0` or `EDGE_LEN`. Mesh coordinates are computed
/// exactly via integer-cell-index arithmetic, so any epsilon below
/// the inter-vertex spacing catches boundary verts without ambiguity.
fn on_boundary(p: &Vec3) -> bool {
    let near_0 = |x: f64| x.abs() < BOUNDARY_PREDICATE_EPS;
    let near_l = |x: f64| (x - EDGE_LEN).abs() < BOUNDARY_PREDICATE_EPS;
    near_0(p.x) || near_l(p.x) || near_0(p.y) || near_l(p.y) || near_0(p.z) || near_l(p.z)
}

/// Assemble the canonical 27-vertex / 48-tet mesh + boundary-pin BC + initial
/// state per the architecture decisions in the module docstring. Returns
/// `(mesh, boundary_pinned_vertex_ids, x_prev_flat, expected_v13_stretched)`.
///
/// Initial guess `x_prev`: 26 boundary at `D · X_rest`, vertex 13 at REST.
/// This is **off-equilibrium by construction** — Newton has actual work to
/// do (3-DOF elastic relaxation of v_13 toward its homogeneous position
/// `D · v_13_rest`). The quasi-static `density = 0` config (set up in
/// [`build_solver`]) makes `D · v_13_rest` the exact backward-Euler
/// equilibrium, recovered to within Newton's residual tol.
fn build_scene() -> (HandBuiltTetMesh, Vec<VertexId>, Vec<f64>, Vec3) {
    let mesh = HandBuiltTetMesh::uniform_block(
        N_PER_EDGE,
        EDGE_LEN,
        &MaterialField::uniform(MU_PA, LAMBDA_PA),
    );

    // Canonical BC-construction pattern: pick by spatial predicate, then
    // orphan-filter via `referenced_vertices`. The orphan filter is a
    // no-op on hand-built meshes (every vertex IS referenced by some
    // tet by construction — see `mesh::hand_built` doc), but it's
    // load-bearing on SDF-meshed scenes where the BCC lattice retains
    // unreferenced corners outside the SDF zero set (per
    // `tests/sdf_forward_map_gradcheck.rs`'s `referenced_vertices`-
    // filtered argmax pattern). Showing the canonical idiom here means
    // future SDF-meshed examples can reuse it verbatim.
    let referenced = referenced_vertices(&mesh);
    let boundary: Vec<VertexId> = pick_vertices_by_predicate(&mesh, on_boundary)
        .into_iter()
        .filter(|v| referenced.contains(v))
        .collect();

    let positions = mesh.positions();
    let mut x_prev_flat = vec![0.0; 3 * positions.len()];
    for (v, &rest) in positions.iter().enumerate() {
        let v_id = v as VertexId;
        let pos = if v_id == INTERIOR_VERTEX_ID {
            // Interior vertex starts at REST — off-equilibrium initial
            // guess so Newton iterates the assembly path.
            rest
        } else {
            stretch(rest, LAMBDA_STRETCH)
        };
        x_prev_flat[3 * v] = pos.x;
        x_prev_flat[3 * v + 1] = pos.y;
        x_prev_flat[3 * v + 2] = pos.z;
    }

    let v13_rest = positions[INTERIOR_VERTEX_ID as usize];
    let expected_v13_stretched = stretch(v13_rest, LAMBDA_STRETCH);
    (mesh, boundary, x_prev_flat, expected_v13_stretched)
}

/// Build the quasi-static solver — `SolverConfig::skeleton()` with `density = 0`
/// to suppress the backward-Euler inertia term `(M / Δt²)·(x − x_hat)`.
/// With zero mass, the residual reduces to `f_int(x) − f_ext = 0` (pure
/// elasticity) and the static minimizer of homogeneous-NH under affine
/// Dirichlet boundary data — i.e., the homogeneous deformation `x = D · X` —
/// becomes the exact equilibrium, recovered to within Newton's residual tol.
fn build_solver(
    mesh: HandBuiltTetMesh,
    boundary: Vec<VertexId>,
) -> (CpuTet4NHSolver<HandBuiltTetMesh>, SolverConfig) {
    let mut cfg = SolverConfig::skeleton();
    cfg.density = 0.0;
    let bc = BoundaryConditions {
        pinned_vertices: boundary,
        roller_vertices: Vec::new(),
        loaded_vertices: vec![],
    };
    let solver = CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
    (solver, cfg)
}

// =============================================================================
// Per-tet record — verify_* + JSON consumers
// =============================================================================

#[derive(Debug, Clone)]
struct TetRecord {
    tet_id: u32,
    verts: [VertexId; 4],
    contains_interior: bool,
    f: Matrix3<f64>,
    p: Matrix3<f64>,
    psi: f64,
    max_stretch_dev: f64,
}

/// Build per-tet records from the converged `x_final` + per-tet topology
/// snapshot. Walks 48 tets, computes `F` via `F = J · J_0^-1`, calls
/// `Material::first_piola(F)` and `Material::energy(F)`. The walk IS
/// the per-tet local→global stitch — every tet reads its own 4-vertex
/// view of the global `x_final` via the captured `tet_verts` indices
/// (snapshotted before the solver consumed the mesh).
fn build_tet_records(
    rest_positions: &[Vec3],
    tet_verts: &[[VertexId; 4]],
    mat: &NeoHookean,
    x_final: &[f64],
) -> Vec<TetRecord> {
    debug_assert_eq!(tet_verts.len(), N_TETS);
    let mut records = Vec::with_capacity(N_TETS);
    for (tet_id, &verts) in tet_verts.iter().enumerate() {
        let f = tet_deformation_gradient(rest_positions, x_final, verts);
        let p = mat.first_piola(&f);
        let psi = mat.energy(&f);
        let max_stretch_dev = max_stretch_deviation(&f);
        let contains_interior = verts.contains(&INTERIOR_VERTEX_ID);
        records.push(TetRecord {
            tet_id: tet_id as u32,
            verts,
            contains_interior,
            f,
            p,
            psi,
            max_stretch_dev,
        });
    }
    records
}

// =============================================================================
// verify_referenced_vertices_full
// =============================================================================

fn verify_referenced_vertices_full(mesh: &HandBuiltTetMesh) {
    let refs = referenced_vertices(mesh);
    assert_eq!(
        refs.len(),
        N_VERTICES,
        "referenced_vertices length drift: got {}, expected {N_VERTICES} \
         (hand-built mesh has no orphans by construction)",
        refs.len(),
    );
    for (i, &v) in refs.iter().enumerate() {
        assert_eq!(
            v as usize, i,
            "referenced_vertices[{i}] = {v} drifted from sequential layout — \
             hand-built meshes' BTreeSet collection should produce [0..N_VERTICES) \
             since every vertex appears in at least one tet"
        );
    }
}

// =============================================================================
// verify_mesh_topology
// =============================================================================

fn verify_mesh_topology(mesh: &HandBuiltTetMesh) {
    assert_eq!(
        mesh.n_vertices(),
        N_VERTICES,
        "mesh.n_vertices drift: got {}, expected {N_VERTICES}",
        mesh.n_vertices(),
    );
    assert_eq!(
        mesh.n_tets(),
        N_TETS,
        "mesh.n_tets drift: got {}, expected {N_TETS}",
        mesh.n_tets(),
    );
    // Interior vertex (1, 1, 1) at the cube's center: stride formula
    // `1 + 1·3 + 1·9 = 13`. Its rest position must be `(L/2, L/2, L/2)`.
    let v13 = mesh.positions()[INTERIOR_VERTEX_ID as usize];
    let half = EDGE_LEN / 2.0;
    assert_relative_eq!(v13.x, half, max_relative = REL_TOL, epsilon = EPS_ABS);
    assert_relative_eq!(v13.y, half, max_relative = REL_TOL, epsilon = EPS_ABS);
    assert_relative_eq!(v13.z, half, max_relative = REL_TOL, epsilon = EPS_ABS);
}

// =============================================================================
// verify_boundary_partition
// =============================================================================

fn verify_boundary_partition(boundary: &[VertexId]) {
    assert_eq!(
        boundary.len(),
        N_BOUNDARY_VERTICES,
        "boundary partition drift: got {} pinned, expected {N_BOUNDARY_VERTICES} \
         (= N_VERTICES − 1, with the single interior vertex left free)",
        boundary.len(),
    );
    assert!(
        !boundary.contains(&INTERIOR_VERTEX_ID),
        "interior vertex ID {INTERIOR_VERTEX_ID} should NOT be in the boundary pin set \
         — that's the free DOF Newton solves for"
    );
    // Ascending-order check (load-bearing for the deterministic free-DOF
    // index map the solver builds — Decision M / scope §15 D-3). Both
    // `pick_vertices_by_predicate` and `referenced_vertices` walk in
    // ascending `VertexId` order, so the filter result should also be
    // ascending. A regression that introduced HashMap-based collection
    // anywhere upstream would surface here.
    for window in boundary.windows(2) {
        assert!(
            window[0] < window[1],
            "boundary partition not ascending: {} >= {}",
            window[0],
            window[1],
        );
    }
}

// =============================================================================
// verify_validity_in_bounds_at_x_prev
// =============================================================================

fn verify_validity_in_bounds_at_x_prev(mesh: &HandBuiltTetMesh, mat: &NeoHookean, x_prev: &[f64]) {
    // Read NH's published validity domain — its `max_stretch_deviation` IS
    // the in-domain bound each tet must stay below, and the demo relies on
    // its `RequireOrientation` inversion policy (satisfied by det F > 0).
    let validity = mat.validity();
    assert!(
        matches!(validity.inversion, InversionHandling::RequireOrientation),
        "NH inversion handler drift: got {:?}, expected RequireOrientation",
        validity.inversion,
    );

    let rest = mesh.positions();
    for tet_id in 0..N_TETS as u32 {
        let verts = mesh.tet_vertices(tet_id);
        let f = tet_deformation_gradient(rest, x_prev, verts);
        let dev = max_stretch_deviation(&f);
        assert!(
            dev < validity.max_stretch_deviation,
            "tet {tet_id} at x_prev: max|σ-1| = {dev} >= NH bound \
             {} — the off-equilibrium initial guess (boundary stretched, interior \
             at rest) has produced an out-of-domain F. Investigate whether \
             LAMBDA_STRETCH or N_PER_EDGE moved enough to push interior-adjacent \
             tets past the boundary.",
            validity.max_stretch_deviation,
        );
    }
}

// =============================================================================
// verify_solver_converges
// =============================================================================

fn verify_solver_converges(step: &NewtonStep<CpuTape>, cfg: &SolverConfig) {
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
}

// =============================================================================
// verify_pinned_x_final_exact
// =============================================================================

fn verify_pinned_x_final_exact(boundary: &[VertexId], x_prev: &[f64], step: &NewtonStep<CpuTape>) {
    // Pinned-DOF semantics per `BoundaryConditions::pinned_vertices`:
    // the solver leaves pinned DOFs unchanged from `x_prev`. Bit-equality
    // is the contract here, since the solver-side update is gated by
    // `if let Some(free_idx) = full_to_free_idx[i]` — pinned DOFs are
    // never written to. This is stronger than a "≈ rest position"
    // assertion because we WANT the prescribed-stretched positions
    // bit-equal, not just close.
    for &v_id in boundary {
        for axis in 0..3 {
            let dof = 3 * (v_id as usize) + axis;
            let prev_bits = x_prev[dof].to_bits();
            let final_bits = step.x_final[dof].to_bits();
            assert_eq!(
                final_bits, prev_bits,
                "pinned vertex {v_id} axis {axis} (DOF {dof}) drifted: x_prev = {} \
                 (bits {prev_bits:#018x}), x_final = {} (bits {final_bits:#018x}). \
                 The solver should leave pinned DOFs bit-equal to x_prev — any \
                 drift indicates a regression in the pin-vs-free DOF dispatch.",
                x_prev[dof], step.x_final[dof],
            );
        }
    }
}

// =============================================================================
// verify_interior_x_final_close_to_homogeneous
// =============================================================================

fn verify_interior_x_final_close_to_homogeneous(
    step: &NewtonStep<CpuTape>,
    expected_v13_stretched: Vec3,
) -> Vec3 {
    let dof = 3 * INTERIOR_VERTEX_ID as usize;
    let observed = Vec3::new(
        step.x_final[dof],
        step.x_final[dof + 1],
        step.x_final[dof + 2],
    );
    assert_relative_eq!(
        observed.x,
        expected_v13_stretched.x,
        max_relative = REL_TOL,
        epsilon = EPS_ABS
    );
    assert_relative_eq!(
        observed.y,
        expected_v13_stretched.y,
        max_relative = REL_TOL,
        epsilon = EPS_ABS
    );
    assert_relative_eq!(
        observed.z,
        expected_v13_stretched.z,
        max_relative = REL_TOL,
        epsilon = EPS_ABS
    );
    observed
}

// =============================================================================
// verify_per_element_F_uniform_diag
// =============================================================================

fn verify_per_element_f_uniform_diag(records: &[TetRecord]) {
    for rec in records {
        // Diagonal entries match `diag(λ, 1, 1)`.
        assert_relative_eq!(
            rec.f[(0, 0)],
            LAMBDA_STRETCH,
            max_relative = REL_TOL,
            epsilon = EPS_ABS
        );
        assert_relative_eq!(
            rec.f[(1, 1)],
            1.0,
            max_relative = REL_TOL,
            epsilon = EPS_ABS
        );
        assert_relative_eq!(
            rec.f[(2, 2)],
            1.0,
            max_relative = REL_TOL,
            epsilon = EPS_ABS
        );
        // Off-diagonals match zero within absolute floor.
        for i in 0..3 {
            for j in 0..3 {
                if i == j {
                    continue;
                }
                assert!(
                    rec.f[(i, j)].abs() < EPS_ABS,
                    "tet {}: F[({i},{j})] = {:e} exceeds off-diagonal floor {EPS_ABS:e}",
                    rec.tet_id,
                    rec.f[(i, j)],
                );
            }
        }
    }
}

// =============================================================================
// verify_uniformity_spread_bounded
// =============================================================================

fn verify_uniformity_spread_bounded(records: &[TetRecord]) -> f64 {
    let mut min_p11 = f64::INFINITY;
    let mut max_p11 = f64::NEG_INFINITY;
    for rec in records {
        let p11 = rec.p[(0, 0)];
        if p11 < min_p11 {
            min_p11 = p11;
        }
        if p11 > max_p11 {
            max_p11 = p11;
        }
    }
    let spread = max_p11 - min_p11;
    assert!(
        spread < UNIFORMITY_SPREAD_BOUND_PA,
        "P_11 uniformity spread {spread:e} Pa exceeds bound {UNIFORMITY_SPREAD_BOUND_PA:e} Pa \
         across {N_TETS} tets (min {min_p11:e}, max {max_p11:e}). Either Newton's residual \
         tol moved or the per-tet F → P pipeline has lost determinism."
    );
    spread
}

// =============================================================================
// JSON emit
// =============================================================================

fn save_json(
    records: &[TetRecord],
    cfg: &SolverConfig,
    step: &NewtonStep<CpuTape>,
    interior_observed: Vec3,
    expected_v13_stretched: Vec3,
    spread_p11: f64,
    path: &Path,
) -> Result<()> {
    let tets: Vec<_> = records
        .iter()
        .map(|r| {
            json!({
                "tet_id": r.tet_id,
                "verts": [r.verts[0], r.verts[1], r.verts[2], r.verts[3]],
                "contains_interior_vertex": r.contains_interior,
                "F_diag":  [r.f[(0, 0)], r.f[(1, 1)], r.f[(2, 2)]],
                "F_offdiag_max_abs": (0..3).flat_map(|i| (0..3)
                    .filter(move |&j| i != j)
                    .map(move |j| r.f[(i, j)].abs()))
                    .fold(0.0_f64, f64::max),
                "P_diag":  [r.p[(0, 0)], r.p[(1, 1)], r.p[(2, 2)]],
                "P_offdiag_max_abs": (0..3).flat_map(|i| (0..3)
                    .filter(move |&j| i != j)
                    .map(move |j| r.p[(i, j)].abs()))
                    .fold(0.0_f64, f64::max),
                "psi": r.psi,
                "max_stretch_deviation": r.max_stretch_dev,
            })
        })
        .collect();

    let record = json!({
        "scene": {
            "mesh": {
                "constructor": "HandBuiltTetMesh::uniform_block",
                "n_per_edge": N_PER_EDGE,
                "edge_len_m": EDGE_LEN,
                "n_vertices": N_VERTICES,
                "n_tets": N_TETS,
                "interior_vertex_id": INTERIOR_VERTEX_ID,
            },
            "material_uniform": {
                "mu_Pa": MU_PA,
                "lambda_Pa": LAMBDA_PA,
                "nu_compressible": 0.4,
            },
            "solver_config": {
                "dt_s": cfg.dt,
                "tol_N": cfg.tol,
                "max_newton_iter": cfg.max_newton_iter,
                "density_kg_m3_quasi_static": cfg.density,
            },
            "boundary_condition": {
                "kind": "Dirichlet (homogeneous stretch)",
                "n_pinned": N_BOUNDARY_VERTICES,
                "n_free": 1,
                "free_vertex_id": INTERIOR_VERTEX_ID,
                "stretch_form": "F = diag(lambda, 1, 1)",
                "lambda": LAMBDA_STRETCH,
            },
        },
        // The kinematic (Dirichlet-imposed) expectations the assembly must
        // reproduce — F = diag(λ,1,1) and the interior vertex at D·X_rest.
        // Constitutive stress/energy at this F is owned by the sim-soft
        // NeoHookean lib tests; per-tet observed P/ψ are in `tets` below.
        "expected_homogeneous": {
            "F_diag": [LAMBDA_STRETCH, 1.0, 1.0],
            "v13_stretched": [expected_v13_stretched.x, expected_v13_stretched.y, expected_v13_stretched.z],
        },
        "step_result": {
            "iter_count": step.iter_count,
            "final_residual_norm_N": step.final_residual_norm,
            "v13_observed": [interior_observed.x, interior_observed.y, interior_observed.z],
            "v13_displacement_from_rest_x_m": interior_observed.x - EDGE_LEN / 2.0,
            "v13_residual_vs_homogeneous_m": [
                (interior_observed.x - expected_v13_stretched.x).abs(),
                (interior_observed.y - expected_v13_stretched.y).abs(),
                (interior_observed.z - expected_v13_stretched.z).abs(),
            ],
        },
        "uniformity": {
            "p11_min": records.iter().map(|r| r.p[(0, 0)]).fold(f64::INFINITY, f64::min),
            "p11_max": records.iter().map(|r| r.p[(0, 0)]).fold(f64::NEG_INFINITY, f64::max),
            "p11_spread_Pa": spread_p11,
            "p11_spread_bound_Pa": UNIFORMITY_SPREAD_BOUND_PA,
        },
        "tets": tets,
    });

    let file = std::fs::File::create(path)
        .with_context(|| format!("failed to create {}", path.display()))?;
    serde_json::to_writer_pretty(&file, &record)
        .with_context(|| format!("failed to serialize record to {}", path.display()))?;
    Ok(())
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

fn print_summary(
    records: &[TetRecord],
    cfg: &SolverConfig,
    step: &NewtonStep<CpuTape>,
    interior_observed: Vec3,
    expected_v13_stretched: Vec3,
    spread_p11: f64,
    path: &Path,
) {
    println!("==== multi_element ====");
    println!();
    println!(
        "input  : HandBuiltTetMesh::uniform_block(n = {N_PER_EDGE}, edge = {EDGE_LEN} m, uniform NH)"
    );
    println!("         {N_VERTICES} vertices, {N_TETS} tets (CFK 6-tets-per-cell, 8 cells)");
    println!("         material: μ = {MU_PA:e} Pa, Λ = {LAMBDA_PA:e} Pa  (Ecoflex-class, ν ≈ 0.4)");
    println!("         BC      : Dirichlet — pin every boundary vertex to D · X_rest");
    println!(
        "                   ({N_BOUNDARY_VERTICES} of {N_VERTICES} pinned; vertex ID {INTERIOR_VERTEX_ID} = (1,1,1) is the lone interior)"
    );
    println!(
        "         stretch : D = diag(λ, 1, 1), λ = {LAMBDA_STRETCH}  (constrained, NOT traction-free)"
    );
    println!(
        "         config  : Δt = {} s, tol = {:e} N, density = {} kg/m³ (quasi-static)",
        cfg.dt, cfg.tol, cfg.density
    );
    println!(
        "         x_prev  : 26 boundary at D·X_rest, vertex {INTERIOR_VERTEX_ID} at REST  (off-equilibrium initial guess)"
    );
    println!();
    println!(
        "Kinematic expectation the assembly must reproduce (F = diag({LAMBDA_STRETCH}, 1, 1)):"
    );
    println!(
        "  v13_stretched (D·X_rest) : ({:>9.6}, {:>9.6}, {:>9.6}) m",
        expected_v13_stretched.x, expected_v13_stretched.y, expected_v13_stretched.z
    );
    println!("  (constitutive P/ψ at this F owned by sim-soft's NeoHookean lib tests)");
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!("  referenced_vertices_full      : referenced_vertices(&mesh) == [0..27)");
    println!("  mesh_topology                 : 27 verts, 48 tets, interior vertex ID == 13");
    println!("  boundary_partition            : 26 boundary pinned, vertex 13 free, ascending");
    println!(
        "  validity_in_bounds_at_x_prev  : per-tet max|σ-1| < NH bound at off-equilibrium x_prev"
    );
    println!("  solver_converges              : iter_count < max_newton_iter; residual < tol");
    println!("  pinned_x_final_exact          : every pinned DOF bit-equal to x_prev");
    println!(
        "  interior_x_final_close        : v13 final position vs D·v13_rest at rel {REL_TOL:e}"
    );
    println!("  per_element_F_uniform_diag    : every tet F vs diag(λ, 1, 1) at rel {REL_TOL:e}");
    println!(
        "  uniformity_spread_bounded     : max P_11 - min P_11 < {UNIFORMITY_SPREAD_BOUND_PA:e} Pa across {N_TETS} tets"
    );
    println!();
    println!("Step result:");
    println!("  iter_count               : {:>3}", step.iter_count);
    println!(
        "  final_residual_norm      : {:>13e} N  (tol {:e})",
        step.final_residual_norm, cfg.tol
    );
    println!(
        "  v13 observed             : ({:>13.10}, {:>13.10}, {:>13.10}) m",
        interior_observed.x, interior_observed.y, interior_observed.z
    );
    println!(
        "  v13 expected             : ({:>13.10}, {:>13.10}, {:>13.10}) m",
        expected_v13_stretched.x, expected_v13_stretched.y, expected_v13_stretched.z
    );
    let dx = interior_observed.x - expected_v13_stretched.x;
    let dy = interior_observed.y - expected_v13_stretched.y;
    let dz = interior_observed.z - expected_v13_stretched.z;
    println!(
        "  v13 |Δ|                  : ({:>13.3e}, {:>13.3e}, {:>13.3e}) m",
        dx.abs(),
        dy.abs(),
        dz.abs()
    );
    println!();
    println!("Per-tet uniformity ({N_TETS} tets):");
    let p11_min = records
        .iter()
        .map(|r| r.p[(0, 0)])
        .fold(f64::INFINITY, f64::min);
    let p11_max = records
        .iter()
        .map(|r| r.p[(0, 0)])
        .fold(f64::NEG_INFINITY, f64::max);
    let max_dev = records
        .iter()
        .map(|r| r.max_stretch_dev)
        .fold(0.0_f64, f64::max);
    let n_with_v13 = records.iter().filter(|r| r.contains_interior).count();
    println!(
        "  tets containing v13      : {:>3}  (the other {} are isolated — F bit-equal D)",
        n_with_v13,
        N_TETS - n_with_v13
    );
    println!("  P_11 min                 : {p11_min:>13.6e} Pa");
    println!("  P_11 max                 : {p11_max:>13.6e} Pa");
    println!(
        "  P_11 spread              : {spread_p11:>13.3e} Pa  (bound {UNIFORMITY_SPREAD_BOUND_PA:e} Pa)"
    );
    println!("  max|σᵢ-1| across tets    : {max_dev:>13.6}     (< 1.0 strict in-domain)");
    println!();
    println!("JSON   : {}", path.display());
    println!("         per-tet record + scene + expected_homogeneous + uniformity verdict");
    println!(
        "         jq '.uniformity'                                      {}",
        path.display()
    );
    println!(
        "         jq '.tets | map({{tet_id, contains_interior_vertex, P_11: .P_diag[0]}})'   {}",
        path.display()
    );
}

// =============================================================================
// main
// =============================================================================

pub fn run() -> Result<()> {
    let mat = NeoHookean::from_lame(MU_PA, LAMBDA_PA);
    let (mesh, boundary, x_prev_flat, expected_v13_stretched) = build_scene();

    // Pre-solver topology + BC anchors.
    verify_referenced_vertices_full(&mesh);
    verify_mesh_topology(&mesh);
    verify_boundary_partition(&boundary);
    verify_validity_in_bounds_at_x_prev(&mesh, &mat, &x_prev_flat);

    // Snapshot the topology + rest geometry the post-solve `build_tet_records`
    // pass needs, so we don't have to rebuild the mesh after the solver
    // takes ownership. Both cheap reads — `positions()` is `&[Vec3]`,
    // `tet_vertices()` is by-value.
    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();
    let tet_verts: Vec<[VertexId; 4]> = (0..mesh.n_tets() as u32)
        .map(|t| mesh.tet_vertices(t))
        .collect();

    // Build solver (consumes mesh + boundary). `cfg` is owned by the
    // solver; we receive a copy for residual / iter-count assertions.
    let (solver, cfg) = build_solver(mesh, boundary.clone());

    // Run the step. Empty θ since BC has no loaded vertices.
    let x_prev = Tensor::from_slice(&x_prev_flat, &[N_DOF]);
    let v_prev = Tensor::zeros(&[N_DOF]);
    let theta = Tensor::from_slice(&[], &[0]);
    let step = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);

    verify_solver_converges(&step, &cfg);
    verify_pinned_x_final_exact(&boundary, &x_prev_flat, &step);
    let interior_observed =
        verify_interior_x_final_close_to_homogeneous(&step, expected_v13_stretched);

    // Per-tet records — uses the snapshot captured above.
    let records = build_tet_records(&rest_positions, &tet_verts, &mat, &step.x_final);

    verify_per_element_f_uniform_diag(&records);
    let spread_p11 = verify_uniformity_spread_bounded(&records);

    // JSON + stdout summary.
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("out")
        .join("multi_element");
    std::fs::create_dir_all(&out_dir)
        .with_context(|| format!("failed to create {}", out_dir.display()))?;
    let out_path = out_dir.join("multi_element_stretch.json");
    save_json(
        &records,
        &cfg,
        &step,
        interior_observed,
        expected_v13_stretched,
        spread_p11,
        &out_path,
    )?;

    print_summary(
        &records,
        &cfg,
        &step,
        interior_observed,
        expected_v13_stretched,
        spread_p11,
        &out_path,
    );

    Ok(())
}
