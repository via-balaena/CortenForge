//! Tet10 ladder rung 7 — the material-parameter adjoint on the quadratic element.
//!
//! The material channel is the ONE adjoint RHS that was genuinely single-point
//! before rung 7: `assemble_material_residual_grad` read the single corner
//! [`ElementGeometry`] and looped `0..4`. Rung 7 widens it to the per-Gauss-point
//! form — the `assemble_global_int_force` internal-force loop re-run with `∂P/∂p_k`
//! in place of `P`, integrated over the element's `G` Stroud points across all `N`
//! nodes. (The adjoint *tangent* `A` and the reaction/state RHS were already
//! per-Gauss-point since rung 4 — FD-gated in the sibling
//! `tet10_dirichlet_reaction_sensitivity.rs` / `tet10_state_sensitivity.rs`; only
//! this RHS needed the widen.)
//!
//! Gates:
//! 1. **Tet10 forward** `∂x*/∂μ`, `∂x*/∂λ` match a re-solve central FD on a loaded
//!    Tet10 block (the oracle re-runs the full nonlinear Newton at `p±ε`, touching
//!    none of the `A⁻¹` machinery — a genuine cross-check that the widened per-GP
//!    RHS is right, not merely self-consistent).
//! 2. **Tet10 reverse** `MaterialStepVjp` on a tape matches the forward-sensitivity
//!    dual AND a re-solve FD of `Σx*` (the widen is single-sourced, so this also
//!    exercises the reverse path).
//! 3. **Tet4 byte-identity** — the widened RHS must leave the Tet4 (`G = 1`) path
//!    bit-for-bit unchanged. `element_node_ids::<_, _, 4>` returns exactly
//!    `tet_vertices` and `gauss[0]` duplicates the old `ElementGeometry`
//!    `(grad_x_n, volume)` bit-for-bit, so the loop reproduces the pre-rung-7
//!    arithmetic op-for-op. The existing Tet4 `material_sensitivity.rs` is
//!    tolerance-only (`1e-5`) and would miss ULP drift; this pins the exact bits
//!    (fingerprints frozen from the pre-widen code).

#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::excessive_precision
)]

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::element::Tet10;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet10NHSolver, HandBuiltTetMesh, LoadAxis,
    MaterialField, Mesh, NullContact, Solver, SolverConfig, Tet4, Tet10Mesh, Vec3, VertexId,
};

// ── Scene constants (mirror the Tet4 `material_sensitivity.rs`) ────────────────
const N: usize = 2;
const EDGE: f64 = 0.1;
const MU0: f64 = 3.0e4;
const LAMBDA0: f64 = 1.2e5;
const DT: f64 = 1.0; // static — large dt damps the M/dt² inertia term
const THETA: f64 = 5.0; // small +ẑ load on the top face (small-strain, valid)

/// The linear (Tet4) cube at material `(mu, lambda)`.
fn tet4_block(mu: f64, lambda: f64) -> HandBuiltTetMesh {
    HandBuiltTetMesh::uniform_block(N, EDGE, &MaterialField::uniform(mu, lambda))
}

/// Corner ids on the bottom (`z ≈ 0`) and top (`z ≈ EDGE`) faces — the pins and
/// the +ẑ load. Both are CORNER vertices (`id < n_corners`); the enriched Tet10
/// midsides on those faces stay free (irrelevant to the FD cross-check, which
/// only needs analytic == re-solve).
fn face_corners(mesh: &HandBuiltTetMesh) -> (Vec<VertexId>, Vec<VertexId>) {
    let pos = mesh.positions();
    let bottom = (0..mesh.n_vertices() as VertexId)
        .filter(|&v| pos[v as usize].z.abs() < 1e-9)
        .collect();
    let top = (0..mesh.n_vertices() as VertexId)
        .filter(|&v| (pos[v as usize].z - EDGE).abs() < 1e-9)
        .collect();
    (bottom, top)
}

/// Rest positions (vertex-major xyz) of a mesh.
fn rest_dofs<M: Mesh<sim_soft::NeoHookean>>(mesh: &M) -> Vec<f64> {
    let p = mesh.positions();
    let mut x = vec![0.0_f64; 3 * p.len()];
    for (v, pt) in p.iter().enumerate() {
        x[3 * v] = pt.x;
        x[3 * v + 1] = pt.y;
        x[3 * v + 2] = pt.z;
    }
    x
}

/// A Tet10 solver at material `(mu, lambda)` plus its rest DOFs: enrich the
/// linear cube, pin the bottom-face corners, load the top-face corners `+ẑ`.
fn tet10_solver(mu: f64, lambda: f64) -> (CpuTet10NHSolver<Tet10Mesh>, Vec<f64>) {
    let cube = tet4_block(mu, lambda);
    let (pinned, loaded) = face_corners(&cube);
    let mesh = Tet10Mesh::from_tet4(&cube);
    let rest = rest_dofs(&mesh);
    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded.into_iter().map(|v| (v, LoadAxis::AxisZ)).collect(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.max_newton_iter = 80;
    (
        CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc),
        rest,
    )
}

/// `x_final` of one static loaded Tet10 step at material `(mu, lambda)`.
fn solve(mu: f64, lambda: f64) -> Vec<f64> {
    let (s, x0) = tet10_solver(mu, lambda);
    let n = x0.len();
    s.replay_step(
        &Tensor::from_slice(&x0, &[n]),
        &Tensor::from_slice(&vec![0.0; n], &[n]),
        &Tensor::from_slice(&[THETA], &[1]),
        DT,
    )
    .x_final
}

/// Relative L2 over all DOFs (pinned entries are exactly zero in both analytic
/// and FD, so the whole-vector norm is well-posed).
fn rel_l2(a: &[f64], b: &[f64]) -> f64 {
    let num: f64 = a.iter().zip(b).map(|(x, y)| (x - y).powi(2)).sum();
    let den: f64 = b.iter().map(|y| y * y).sum();
    (num / den).sqrt()
}

#[test]
fn tet10_equilibrium_material_sensitivity_matches_resolve_fd() {
    let (solver, _) = tet10_solver(MU0, LAMBDA0);
    let x_final = solve(MU0, LAMBDA0);

    // ── ∂x*/∂μ ──
    let an_mu = solver.equilibrium_material_sensitivity(&x_final, None, DT, 0);
    let de = MU0 * 1e-6;
    let fd_mu: Vec<f64> = solve(MU0 + de, LAMBDA0)
        .iter()
        .zip(&solve(MU0 - de, LAMBDA0))
        .map(|(a, b)| (a - b) / (2.0 * de))
        .collect();
    let rel_mu = rel_l2(&an_mu, &fd_mu);
    let max_mu = an_mu.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);

    // ── ∂x*/∂λ ──
    let an_l = solver.equilibrium_material_sensitivity(&x_final, None, DT, 1);
    let dl = LAMBDA0 * 1e-6;
    let fd_l: Vec<f64> = solve(MU0, LAMBDA0 + dl)
        .iter()
        .zip(&solve(MU0, LAMBDA0 - dl))
        .map(|(a, b)| (a - b) / (2.0 * dl))
        .collect();
    let rel_l = rel_l2(&an_l, &fd_l);
    let max_l = an_l.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);

    eprintln!(
        "Tet10 material sensitivity: ‖∂x*/∂μ‖_∞={max_mu:.3e} rel={rel_mu:.3e}  \
         ‖∂x*/∂λ‖_∞={max_l:.3e} rel={rel_l:.3e}"
    );
    assert!(
        max_mu > 1e-9 && max_l > 1e-9,
        "sensitivities implausibly small"
    );
    assert!(
        rel_mu < 1e-5,
        "Tet10 ∂x*/∂μ disagrees with re-solve FD: {rel_mu:e}"
    );
    assert!(
        rel_l < 1e-5,
        "Tet10 ∂x*/∂λ disagrees with re-solve FD: {rel_l:e}"
    );
}

/// A smooth, deterministic, non-inverting midside curvature for the block —
/// pushes each midside off its edge midpoint so `construct` routes those
/// elements through the isoparametric per-Gauss-point path.
fn curve_block_midsides(mesh: Tet10Mesh) -> Tet10Mesh {
    mesh.with_curved_midsides(|p| p + Vec3::new(0.02 * p.y, -0.015 * p.z, 0.018 * p.x))
}

/// A CURVED Tet10 solver at material `(mu, lambda)` (same BCs as `tet10_solver`).
fn curved_tet10_solver(mu: f64, lambda: f64) -> (CpuTet10NHSolver<Tet10Mesh>, Vec<f64>) {
    let cube = tet4_block(mu, lambda);
    let (pinned, loaded) = face_corners(&cube);
    let mesh = curve_block_midsides(Tet10Mesh::from_tet4(&cube));
    let rest = rest_dofs(&mesh);
    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded.into_iter().map(|v| (v, LoadAxis::AxisZ)).collect(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.max_newton_iter = 80;
    (
        CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc),
        rest,
    )
}

/// `x_final` of one static loaded step on the CURVED Tet10 block.
fn curved_solve(mu: f64, lambda: f64) -> Vec<f64> {
    let (s, x0) = curved_tet10_solver(mu, lambda);
    let n = x0.len();
    s.replay_step(
        &Tensor::from_slice(&x0, &[n]),
        &Tensor::from_slice(&vec![0.0; n], &[n]),
        &Tensor::from_slice(&[THETA], &[1]),
        DT,
    )
    .x_final
}

/// The differentiable adjoint flows correctly through CURVED elements. The
/// material sensitivity reads the per-Gauss-point `(b)` cache (rung 7), which
/// curvature enters — so `∂x*/∂μ` on a curved Tet10 block must still match a
/// re-solve central FD (each re-solve rebuilds the SAME curved mesh at `μ ± ε`).
/// Confirms the curved per-GP geometry is consistent between the forward
/// stiffness and the adjoint RHS.
#[test]
fn tet10_curved_material_sensitivity_matches_resolve_fd() {
    let (solver, _) = curved_tet10_solver(MU0, LAMBDA0);
    let x_final = curved_solve(MU0, LAMBDA0);

    let an_mu = solver.equilibrium_material_sensitivity(&x_final, None, DT, 0);
    let de = MU0 * 1e-6;
    let fd_mu: Vec<f64> = curved_solve(MU0 + de, LAMBDA0)
        .iter()
        .zip(&curved_solve(MU0 - de, LAMBDA0))
        .map(|(a, b)| (a - b) / (2.0 * de))
        .collect();
    let rel_mu = rel_l2(&an_mu, &fd_mu);
    let max_mu = an_mu.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);

    eprintln!("Curved Tet10 ∂x*/∂μ: ‖·‖_∞={max_mu:.3e} rel={rel_mu:.3e}");
    assert!(max_mu > 1e-9, "curved sensitivity implausibly small");
    assert!(
        rel_mu < 1e-5,
        "curved Tet10 ∂x*/∂μ disagrees with re-solve FD: {rel_mu:e}",
    );
}

/// The reverse-mode `MaterialStepVjp` on the Tet10 element: `μ → x*`, `backward`
/// seeding ones (implicit `L = Σx*`), so `grad(μ) = ∂(Σx*)/∂μ`. Validated against
/// the forward-sensitivity sum (the dual through the same factored `A`) AND a
/// re-solve FD of `Σx*` over μ — the widen is single-sourced (`assemble_material_
/// residual_grad` feeds both the forward and reverse paths), so this proves the
/// reverse path on the widened RHS.
#[test]
fn tet10_material_step_vjp_backward_matches_forward_and_fd() {
    let (solver, _) = tet10_solver(MU0, LAMBDA0);
    let x_final = solve(MU0, LAMBDA0);
    let nd = x_final.len();

    let mut tape = Tape::new();
    let mu_var = tape.param_tensor(Tensor::from_slice(&[MU0], &[1]));
    let xstar = tape.push_custom(
        &[mu_var],
        Tensor::from_slice(&x_final, &[nd]),
        Box::new(solver.material_step_vjp(&x_final, DT, 0)),
    );
    tape.backward(xstar);
    let grad_mu = tape.grad_tensor(mu_var).as_slice()[0];

    let fwd_sum: f64 = solver
        .equilibrium_material_sensitivity(&x_final, None, DT, 0)
        .iter()
        .sum();
    let de = MU0 * 1e-6;
    let fd = (solve(MU0 + de, LAMBDA0).iter().sum::<f64>()
        - solve(MU0 - de, LAMBDA0).iter().sum::<f64>())
        / (2.0 * de);

    eprintln!(
        "Tet10 material VJP: grad(μ) tape={grad_mu:.6e}  forward-sum={fwd_sum:.6e}  FD={fd:.6e}"
    );
    assert!(grad_mu.abs() > 1e-9, "∂(Σx*)/∂μ implausibly small");
    assert!(
        (grad_mu - fwd_sum).abs() / fwd_sum.abs() < 1e-10,
        "reverse VJP {grad_mu} != forward-sensitivity sum {fwd_sum}"
    );
    assert!(
        (grad_mu - fd).abs() / fd.abs() < 1e-5,
        "reverse VJP {grad_mu} disagrees with re-solve FD {fd}"
    );
}

/// Tet4 byte-identity: the rung-7 widen of `assemble_material_residual_grad` must
/// leave the linear (`G = 1`) path bit-for-bit unchanged. The fingerprints below
/// were frozen from the pre-widen single-point code on this exact fixture
/// (`uniform_block(2, 0.1)` `NeoHookean`, `μ = 3e4`, `λ = 1.2e5`, `θ = 5.0`, static
/// `dt = 1`, pin `z ≈ 0`, load `z ≈ EDGE` `+ẑ`). A ULP drift in the widened loop
/// flips the XOR fingerprint (and the L2), which the tolerance-only Tet4
/// `material_sensitivity.rs` would not catch.
#[test]
fn tet4_material_sensitivity_byte_identical_after_rung7_widen() {
    // Frozen pre-widen fingerprints (μ = param 0, λ = param 1).
    const XOR_MU: u64 = 0x00c7_6d43_8bee_c6f0;
    const XOR_LAMBDA: u64 = 0x001f_5932_7b04_f434;
    const L2_MU: f64 = 5.679_541_865_261_674_9e-7;
    const L2_LAMBDA: f64 = 3.229_397_969_816_624_6e-8;

    let cube = tet4_block(MU0, LAMBDA0);
    let (pinned, loaded) = face_corners(&cube);
    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded.into_iter().map(|v| (v, LoadAxis::AxisZ)).collect(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.max_newton_iter = 50;
    let solver = CpuNewtonSolver::new(Tet4, cube.clone(), NullContact, cfg, bc);

    let mut x = vec![0.0_f64; 3 * cube.n_vertices()];
    for (c, p) in x.chunks_exact_mut(3).zip(cube.positions().iter()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    let n = x.len();
    let x_final = solver
        .replay_step(
            &Tensor::from_slice(&x, &[n]),
            &Tensor::from_slice(&vec![0.0; n], &[n]),
            &Tensor::from_slice(&[THETA], &[1]),
            DT,
        )
        .x_final;

    for (pk, (xor_ref, l2_ref)) in [(XOR_MU, L2_MU), (XOR_LAMBDA, L2_LAMBDA)]
        .into_iter()
        .enumerate()
    {
        let g = solver.equilibrium_material_sensitivity(&x_final, None, DT, pk);
        let xor: u64 = g.iter().map(|v| v.to_bits()).fold(0u64, |a, b| a ^ b);
        let l2: f64 = g.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert_eq!(
            xor, xor_ref,
            "Tet4 material sensitivity (param {pk}) XOR fingerprint changed: \
             {xor:#018x} != {xor_ref:#018x} — the rung-7 widen perturbed the linear path"
        );
        assert_eq!(
            l2.to_bits(),
            l2_ref.to_bits(),
            "Tet4 material sensitivity (param {pk}) L2 changed: {l2:e} != {l2_ref:e}"
        );
    }
}
