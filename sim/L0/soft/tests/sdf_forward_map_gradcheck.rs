//! III-3 — `ForwardMap` survives mesher output, end-to-end.
//!
//! Per scope memo §1 III-3 row + Decisions K / L / O + §6 S-8 lens (viii).
//!
//! Drives a `CpuTet4NHSolver` over the canonical sphere mesh
//! (`SphereSdf { radius: 0.1 }`, `cell_size = 0.02`, bbox `[-0.12, 0.12]³`)
//! at Stage-1 `+ẑ` traction θ. Boundary conditions:
//!
//! - **Pin set** = `pick_vertices_by_predicate(&mesh, |p| p.z < 0.0)` —
//!   the entire bottom hemisphere. Conservative (~half the vertices,
//!   far more than the 6 strictly needed to constrain rigid-body
//!   modes); mirrors the walking-skeleton "majority-pin, single-load"
//!   pattern at sphere scale and ensures well-conditioned Hessian for
//!   the larger DOF count (Decision K + Decision O mitigation a).
//! - **Load vertex** = single max-z over `referenced_vertices` filtered
//!   by `p.z > 0.0`. The orphan-filter through `referenced_vertices`
//!   IS load-bearing: `SdfMeshedTetMesh::from_sdf` retains every BCC
//!   lattice point in `positions()` including unreferenced corners
//!   (e.g. the bbox top corner at `z = +0.12`); raw argmax over
//!   `positions()` would pick an orphan whose +ẑ load has no
//!   element-energy contribution, silently zeroing the gradient and
//!   passing the gradcheck `(0 = 0)` vacuously. (Memo §3 Decision K
//!   post-pivot revision frames this as "defense-in-depth"; in
//!   practice on the BCC pipeline it is the load-bearing filter.)
//!
//! Asserts:
//!
//! 1. `peak_bound` and `stiffness_bound` finite — the on-tape
//!    composition path Phase 2 commit 9 generalized.
//! 2. `pressure_uniformity` and `coverage` are `NaN` per the Phase 2
//!    NaN-sentinel contract preserved at commit 9.
//! 3. `RewardBreakdown::score_with(&weights)` returns a finite scalar
//!    via the NaN-skip path.
//! 4. `∂(reward)/∂θ` matches central-difference FD at `ε = 1.5e-8` to
//!    a 5-digit relative-error bar (Decision O; mirrors
//!    walking-skeleton + Phase 2).
//! 5. Determinism — bit-equal `grad_θ` and `reward` across two
//!    same-process runs. First sparse-solver-at-scale data point in
//!    this codebase (~3,000 tets / ~2,400 DOFs vs Phase 2's ≤24); a
//!    sparse-factor or Newton-iter non-determinism at scale would
//!    surface here.
//!
//! **Scope clarification (Decision L).** III-3 validates that Phase
//! 2's IFT machinery survives the BCC mesher's output quality —
//! Newton converges within budget, the sparse Hessian factors,
//! autograd gives correct gradients on the mesher-meshed body. III-3
//! does NOT validate gradients flowing through the SDF→mesh boundary
//! (Part 6 Ch 05 open problem). θ-perturbation only; mesh topology is
//! fixed by construction during the FD probes. Topology-crossing FD
//! wrapper is Phase H.
//!
//! **Wall-clock budget (S-8 lens viii).** Recorded per probe via
//! `Instant::now()` and surfaced through `eprintln!` (visible under
//! `--nocapture`); the test fails if total wall-clock exceeds 60 s
//! per scope memo §8 commit 8 soft budget. Run with `--release` per
//! `feedback_release_mode_heavy_tests`.

// `to_bits()` comparisons in the determinism test are intentional;
// `theta_val` (f64) vs `theta` (Tensor) is the same meaningful
// distinction the walking-skeleton + Phase 2 gradchecks use; helper
// `expect()`s document Solver::step + canonical-mesh contract
// violations rather than runtime input conditions.
#![allow(clippy::expect_used, clippy::float_cmp, clippy::similar_names)]

use std::time::Instant;

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::mesh::referenced_vertices;
use sim_soft::readout::scene::pick_vertices_by_predicate;
use sim_soft::sdf_bridge::{Aabb3, MeshingHints, SdfMeshedTetMesh, SphereSdf};
use sim_soft::{
    BasicObservable, BoundaryConditions, CpuNewtonSolver, CpuTape, EditResult, ForwardMap,
    GradientEstimate, LoadAxis, Mesh, NeoHookean, NullContact, RewardWeights, SceneInitial,
    SkeletonForwardMap, Solver, SolverConfig, Tet4, Vec3, VertexId,
};

// ── Canonical scene fixtures ─────────────────────────────────────────────

const RADIUS: f64 = 0.1;
const CELL_SIZE: f64 = 0.02;
const BBOX_HALF_EXTENT: f64 = 0.12;

const THETA_0: f64 = 10.0;

const fn gradcheck_weights() -> RewardWeights {
    RewardWeights {
        pressure_uniformity: 0.0,
        coverage: 0.0,
        peak_bound: 1.0,
        stiffness_bound: 1.0e-6,
    }
}

fn canonical_hints() -> MeshingHints {
    MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(-BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT, -BBOX_HALF_EXTENT),
            Vec3::new(BBOX_HALF_EXTENT, BBOX_HALF_EXTENT, BBOX_HALF_EXTENT),
        ),
        cell_size: CELL_SIZE,
    }
}

fn canonical_sphere_mesh() -> SdfMeshedTetMesh {
    SdfMeshedTetMesh::from_sdf(&SphereSdf { radius: RADIUS }, &canonical_hints())
        .expect("canonical sphere scene should mesh successfully")
}

/// Pick the bottom-hemisphere pin set and the single max-z load
/// vertex (filtered through `referenced_vertices` per the doc-comment
/// above — orphans at the +z bbox corners would otherwise win the
/// argmax and silently zero the gradient).
fn boundary_setup(mesh: &SdfMeshedTetMesh) -> (Vec<VertexId>, VertexId) {
    let pinned = pick_vertices_by_predicate(mesh, |p| p.z < 0.0);
    let positions = mesh.positions();
    let load = referenced_vertices(mesh)
        .into_iter()
        .filter(|&v| positions[v as usize].z > 0.0)
        .max_by(|&a, &b| {
            positions[a as usize]
                .z
                .partial_cmp(&positions[b as usize].z)
                .expect("BCC lattice positions are finite by construction")
        })
        .expect("at least one referenced vertex with positive z must exist");
    (pinned, load)
}

/// Assemble a fresh `SkeletonForwardMap` over the SDF-meshed sphere.
/// A fresh instance per call keeps `stashed_theta_var` clean and lets
/// the FD loop reuse the same canonical mesh (cloned in once per
/// probe — much cheaper than re-meshing the SDF six times across the
/// gradcheck).
fn build_forward_map(
    mesh: SdfMeshedTetMesh,
    pinned: Vec<VertexId>,
    load: VertexId,
) -> SkeletonForwardMap {
    let cfg = SolverConfig::skeleton();

    let positions = mesh.positions();
    let n_dof = 3 * positions.len();
    let mut x_prev_flat = vec![0.0; n_dof];
    for (v, pos) in positions.iter().enumerate() {
        x_prev_flat[3 * v] = pos.x;
        x_prev_flat[3 * v + 1] = pos.y;
        x_prev_flat[3 * v + 2] = pos.z;
    }
    let initial = SceneInitial {
        x_prev: Tensor::from_slice(&x_prev_flat, &[n_dof]),
        v_prev: Tensor::zeros(&[n_dof]),
    };

    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        loaded_vertices: vec![(load, LoadAxis::AxisZ)],
    };

    let solver: Box<dyn Solver<Tape = CpuTape>> = Box::new(CpuNewtonSolver::new(
        NeoHookean::from_lame(1e5, 4e5),
        Tet4,
        mesh,
        NullContact,
        cfg,
        bc,
    ));

    // Stage-1 peak_bound = x_final[load_z_dof] — the deflection of the
    // single load vertex along the +ẑ load axis. Mirrors walking-
    // skeleton's `vec![11]` (vertex 3, z-DOF) at sphere scale.
    let load_z_dof = 3 * (load as usize) + 2;
    let observable = BasicObservable::new(vec![load_z_dof]);

    SkeletonForwardMap::new(solver, observable, initial, gradcheck_weights())
}

/// Forward + backward at `theta_val`, returning the analytic gradient
/// of θ plus the primal reward scalar.
fn evaluate_with_gradient(theta_val: f64) -> (f64, f64) {
    let mesh = canonical_sphere_mesh();
    let (pinned, load) = boundary_setup(&mesh);
    let mut fm = build_forward_map(mesh, pinned, load);

    let mut tape = Tape::new();
    let theta = Tensor::from_slice(&[theta_val], &[1]);

    let (rb, _edit) = fm.evaluate(&theta, &mut tape);
    let reward = rb.score_with(&gradcheck_weights());

    let (grad, estimate) = fm.gradient(&theta, &tape);
    assert!(matches!(estimate, GradientEstimate::Exact));
    assert!(grad.shape() == [1]);

    (grad.as_slice()[0], reward)
}

/// Primal-only forward at `theta_val`, used inside the FD loop. The
/// wasted backward (driven by `evaluate`) is fine for two FD probes.
fn reward_only(theta_val: f64) -> f64 {
    let mesh = canonical_sphere_mesh();
    let (pinned, load) = boundary_setup(&mesh);
    let mut fm = build_forward_map(mesh, pinned, load);

    let mut tape = Tape::new();
    let theta = Tensor::from_slice(&[theta_val], &[1]);
    let (rb, _edit) = fm.evaluate(&theta, &mut tape);
    rb.score_with(&gradcheck_weights())
}

// ── Sanity precheck: mesh + BC setup are non-trivial ─────────────────────

#[test]
fn canonical_sphere_scene_is_non_trivial() {
    // Sanity floor for the rest of the suite — a degenerate empty mesh
    // or a missing load vertex would silently make the gradcheck pass
    // vacuously. Mirrors III-1's `sphere_mesh_construction_succeeds`
    // pattern at the III-3 BC layer.
    let mesh = canonical_sphere_mesh();
    let (pinned, load) = boundary_setup(&mesh);

    assert!(
        mesh.n_tets() > 100,
        "canonical sphere scene should produce a substantial mesh; got {} tets",
        mesh.n_tets(),
    );
    assert!(
        pinned.len() > 50,
        "bottom-hemisphere pin set should be substantial (~half the mesh); got {}",
        pinned.len(),
    );
    let load_z = mesh.positions()[load as usize].z;
    assert!(
        load_z > 0.0,
        "load vertex z must be positive (top hemisphere); got {load_z}",
    );
    // The load vertex must be referenced (i.e. participate in some tet)
    // — referenced_vertices was filtered against, so this is a
    // belt-and-suspenders check.
    assert!(
        referenced_vertices(&mesh).contains(&load),
        "load vertex {load} must be referenced — referenced_vertices \
         filter is load-bearing for orphan exclusion",
    );
    assert!(
        !pinned.contains(&load),
        "load vertex {load} must not also be pinned — \
         CpuNewtonSolver::new rejects this overlap",
    );
}

// ── III-3 gradient correctness ───────────────────────────────────────────

#[test]
fn sdf_forward_map_full_stack_gradcheck() {
    // h = √ε for f64; 5-digit relative-error bar per Decision O.
    const H: f64 = 1.5e-8;
    const REL_ERR_BOUND: f64 = 1e-5;

    // Mesh stats are stable across runs by III-1; build once outside
    // timing so the eprintln below reports figures without paying for
    // two extra mesh builds inside the wall-clock window.
    let stats_mesh = canonical_sphere_mesh();
    let n_tets_label = stats_mesh.n_tets();
    let n_dof_label = 3 * stats_mesh.n_vertices();
    drop(stats_mesh);

    let started = Instant::now();

    let (analytic_grad, reward_at_theta) = evaluate_with_gradient(THETA_0);
    assert!(
        reward_at_theta.is_finite(),
        "reward scalar must be finite at θ={THETA_0}, got {reward_at_theta:e}",
    );

    let l_plus = reward_only(THETA_0 + H);
    let l_minus = reward_only(THETA_0 - H);
    let fd_grad = (l_plus - l_minus) / (2.0 * H);

    let denom = fd_grad.abs().max(1e-12);
    let rel_err = (analytic_grad - fd_grad).abs() / denom;

    let elapsed = started.elapsed();
    eprintln!(
        "III-3 wall-clock: {elapsed:?} — 1 grad + 2 FD probes \
         on a ~{n_tets_label}-tet / ~{n_dof_label}-DOF SDF-meshed sphere \
         (first sparse-solver-at-scale data point per scope memo §6 \
         S-8 lens viii). Soft budget: 60 s.",
    );

    assert!(
        rel_err <= REL_ERR_BOUND,
        "III-3 gradcheck failed: analytic = {analytic_grad:.6e}, \
         FD = {fd_grad:.6e}, rel_err = {rel_err:.3e} > {REL_ERR_BOUND:.0e}. \
         Diagnose in scope memo §3 Decision O order: (1) verify III-2 \
         is green — a bad boundary tet may be degrading Hessian \
         conditioning, (2) confirm bottom-hemisphere pin still \
         constrains all 6 rigid-body modes (assert pinned.len() > 50 \
         in canonical_sphere_scene_is_non_trivial), (3) check \
         referenced_vertices filter caught the orphan max-z bbox \
         corner (load.z should be ≈0.1, not 0.12), (4) only loosen \
         the bar after mesh quality is verified clean.",
    );

    assert!(
        elapsed.as_secs() < 60,
        "III-3 wall-clock {elapsed:?} exceeded 60 s soft budget per \
         scope memo §8 commit 8. Surface as material plan change per \
         S-8 lens (viii) — investigate Newton iteration count, \
         sparse-factor cost, or mesh conditioning before merge.",
    );
}

// ── III-3 reward composition: NaN sentinels + finite primals ─────────────

#[test]
fn sdf_forward_map_reward_breakdown_nan_sentinels() {
    // BasicObservable populates peak_bound + stiffness_bound as finite
    // and leaves pressure_uniformity + coverage as NaN per the Phase 2
    // commit 9 generalization (preserves the walking-skeleton
    // NaN-sentinel contract on the multi-vertex path). Full-stack
    // verification on the SDF-meshed scene that score_with's NaN-skip
    // handles the sentinel without poisoning the reward scalar.
    let mesh = canonical_sphere_mesh();
    let (pinned, load) = boundary_setup(&mesh);
    let mut fm = build_forward_map(mesh, pinned, load);

    let mut tape = Tape::new();
    let theta = Tensor::from_slice(&[THETA_0], &[1]);
    let (rb, edit) = fm.evaluate(&theta, &mut tape);

    assert_eq!(edit, EditResult::ParameterOnly);
    assert!(
        rb.pressure_uniformity.is_nan(),
        "pressure_uniformity must be NaN per Phase 2 contract, got {}",
        rb.pressure_uniformity,
    );
    assert!(
        rb.coverage.is_nan(),
        "coverage must be NaN per Phase 2 contract, got {}",
        rb.coverage,
    );
    assert!(
        rb.peak_bound.is_finite(),
        "peak_bound must be finite, got {}",
        rb.peak_bound,
    );
    assert!(
        rb.stiffness_bound.is_finite(),
        "stiffness_bound must be finite, got {}",
        rb.stiffness_bound,
    );

    // NaN + finite weights must not poison the primal scalar. Use
    // intentionally non-zero NaN-field weights to prove the is_nan
    // branch drops them (IEEE 754: NaN × 0 = NaN, so multiply-by-zero
    // alone wouldn't rescue).
    let poison_weights = RewardWeights {
        pressure_uniformity: 42.0,
        coverage: 42.0,
        peak_bound: 1.0,
        stiffness_bound: 0.0,
    };
    let scalar = rb.score_with(&poison_weights);
    assert!(
        scalar.is_finite(),
        "score_with must silently drop NaN fields, got {scalar}",
    );
    assert_eq!(
        scalar.to_bits(),
        rb.peak_bound.to_bits(),
        "NaN-skip score_with must equal w_peak · peak_bound on the \
         SDF-meshed scene",
    );
}

// ── III-3 sparse-solver-at-scale determinism ─────────────────────────────

#[test]
fn sdf_forward_map_determinism() {
    // First codebase data point on sparse-solver determinism at scale.
    // Phase 2 II-1 bit-equality tests run on ≤24 DOFs; III-3 here
    // exercises the same contract on ~2,400 DOFs — a non-deterministic
    // sparse factor or non-deterministic Newton iter would surface as
    // a bit-difference in either grad or reward.
    let (grad_a, reward_a) = evaluate_with_gradient(THETA_0);
    let (grad_b, reward_b) = evaluate_with_gradient(THETA_0);
    assert_eq!(
        grad_a.to_bits(),
        grad_b.to_bits(),
        "SDF-meshed grad_θ not bit-equal at scale: {grad_a:e} vs {grad_b:e}. \
         Likely culprits: a HashMap on the assembly path, non-deterministic \
         rayon reduction, or an iteration-order bug in CpuNewtonSolver's \
         sparse-pattern build (scope memo §3 Decision M D-1..D-7).",
    );
    assert_eq!(
        reward_a.to_bits(),
        reward_b.to_bits(),
        "SDF-meshed reward scalar not bit-equal: {reward_a:e} vs {reward_b:e}. \
         Same diagnosis as grad_θ — primal and gradient share the same \
         Newton solve; non-determinism would hit both.",
    );
}
