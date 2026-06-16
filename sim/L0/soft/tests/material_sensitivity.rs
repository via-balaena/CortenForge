//! Keystone S5 (PR1) — soft material-parameter sensitivity `∂x*/∂(μ, λ)`.
//!
//! The soft autograd (`NewtonStepVjp`) differentiates the converged equilibrium
//! `x*` w.r.t. the applied load θ only. `CpuNewtonSolver::equilibrium_material_sensitivity`
//! extends the IFT machinery to the MATERIAL parameters: the parameters enter
//! the residual only through the elastic internal force, so
//! `∂r/∂p = ∂f_int/∂p` assembles from the stress derivative
//! `Material::first_piola_param_grad(F) = ∂P/∂p`, and `∂x*/∂p = −A⁻¹·(∂r/∂p)`
//! reuses the SAME tangent factored at `x_final`. This is the gradient the
//! co-design optimizer ultimately consumes (it rides the S4 soft↔rigid
//! crossing — swap the load handle for a material handle).
//!
//! Gates: (1) `NeoHookean::first_piola_param_grad` (`∂P/∂μ = F − F⁻ᵀ`,
//! `∂P/∂λ = ln(J)·F⁻ᵀ`) matches a central FD of `first_piola` over each
//! parameter; (2) the assembled `∂x*/∂μ` and `∂x*/∂λ` match a re-solve central
//! FD on a loaded block (no contact — isolates the material physics). The
//! oracle re-runs the full nonlinear Newton solve at `p±ε`, touching none of
//! the `A⁻¹` machinery — a genuine cross-check.

#![allow(clippy::expect_used)]

use nalgebra::Matrix3;
use sim_ml_chassis::{Tape, Tensor};
use sim_soft::readout::scene::pick_vertices_by_predicate;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, LoadAxis, Material, MaterialField, Mesh,
    NeoHookean, NullContact, Solver, SolverConfig, Tet4, VertexId,
};

// ── Scene constants ───────────────────────────────────────────────────────
const N: usize = 2;
const EDGE: f64 = 0.1;
const MU0: f64 = 3.0e4;
const LAMBDA0: f64 = 1.2e5;
const DT: f64 = 1.0; // static — large dt damps the M/dt² inertia term
const THETA: f64 = 5.0; // small +ẑ load on the top face (small-strain, valid)

type NhSolver = CpuNewtonSolver<Tet4, HandBuiltTetMesh, NullContact>;

fn block(mu: f64, lambda: f64) -> HandBuiltTetMesh {
    HandBuiltTetMesh::uniform_block(N, EDGE, &MaterialField::uniform(mu, lambda))
}

fn bc() -> BoundaryConditions {
    let pinned: Vec<VertexId> =
        pick_vertices_by_predicate(&block(MU0, LAMBDA0), |p| p.z.abs() < 1e-9);
    let loaded: Vec<VertexId> =
        pick_vertices_by_predicate(&block(MU0, LAMBDA0), |p| (p.z - EDGE).abs() < 1e-9);
    BoundaryConditions::new(
        pinned,
        loaded.iter().map(|&v| (v, LoadAxis::AxisZ)).collect(),
    )
}

fn build(mu: f64, lambda: f64) -> NhSolver {
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.max_newton_iter = 50;
    CpuNewtonSolver::new(Tet4, block(mu, lambda), NullContact, cfg, bc())
}

fn n_dof() -> usize {
    3 * block(MU0, LAMBDA0).n_vertices()
}

fn x_rest() -> Vec<f64> {
    let mut x = vec![0.0_f64; n_dof()];
    for (c, p) in x
        .chunks_exact_mut(3)
        .zip(block(MU0, LAMBDA0).positions().iter())
    {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    x
}

/// `x_final` of one static loaded step at material `(mu, lambda)`.
fn solve(mu: f64, lambda: f64) -> Vec<f64> {
    let nd = n_dof();
    build(mu, lambda)
        .replay_step(
            &Tensor::from_slice(&x_rest(), &[nd]),
            &Tensor::from_slice(&vec![0.0; nd], &[nd]),
            &Tensor::from_slice(&[THETA], &[1]),
            DT,
        )
        .x_final
}

/// Relative L2 over the free DOFs.
fn rel_free(a: &[f64], b: &[f64], free: &[usize]) -> f64 {
    let num: f64 = free.iter().map(|&i| (a[i] - b[i]).powi(2)).sum();
    let den: f64 = free.iter().map(|&i| b[i] * b[i]).sum();
    (num / den).sqrt()
}

#[test]
fn neohookean_param_stress_grad_matches_fd() {
    // A non-trivial deformation gradient (stretch + shear, det > 0).
    let f = Matrix3::new(1.2, 0.1, 0.0, 0.0, 0.95, 0.05, 0.0, 0.0, 1.1);
    assert!(f.determinant() > 0.0);
    let nh = NeoHookean::from_lame(MU0, LAMBDA0);
    let grads = nh.first_piola_param_grad(&f);
    assert_eq!(grads.len(), 2, "NeoHookean exposes (μ, λ)");

    let de = MU0 * 1e-6;
    let dp_dmu_fd = (NeoHookean::from_lame(MU0 + de, LAMBDA0).first_piola(&f)
        - NeoHookean::from_lame(MU0 - de, LAMBDA0).first_piola(&f))
        / (2.0 * de);
    let dl = LAMBDA0 * 1e-6;
    let dp_dl_fd = (NeoHookean::from_lame(MU0, LAMBDA0 + dl).first_piola(&f)
        - NeoHookean::from_lame(MU0, LAMBDA0 - dl).first_piola(&f))
        / (2.0 * dl);

    let e_mu = (grads[0] - dp_dmu_fd).norm() / dp_dmu_fd.norm();
    let e_l = (grads[1] - dp_dl_fd).norm() / dp_dl_fd.norm();
    eprintln!("∂P/∂μ rel={e_mu:.3e}  ∂P/∂λ rel={e_l:.3e}");
    assert!(e_mu < 1e-6, "∂P/∂μ (=F−F⁻ᵀ) disagrees with FD: {e_mu:e}");
    assert!(e_l < 1e-6, "∂P/∂λ (=ln(J)·F⁻ᵀ) disagrees with FD: {e_l:e}");
}

#[test]
fn equilibrium_material_sensitivity_matches_resolve_fd() {
    let solver = build(MU0, LAMBDA0);
    // Pinned-base DOFs have zero sensitivity in both analytic and FD (they don't
    // move), so a relative L2 over ALL DOFs is well-posed and exact.
    let free: Vec<usize> = (0..n_dof()).collect();
    let x_final = solve(MU0, LAMBDA0);

    // ── ∂x*/∂μ ──
    let an_mu = solver.equilibrium_material_sensitivity(&x_final, None, DT, 0);
    let de = MU0 * 1e-6;
    let fd_mu: Vec<f64> = solve(MU0 + de, LAMBDA0)
        .iter()
        .zip(&solve(MU0 - de, LAMBDA0))
        .map(|(a, b)| (a - b) / (2.0 * de))
        .collect();
    let rel_mu = rel_free(&an_mu, &fd_mu, &free);
    let max_mu = free.iter().map(|&i| an_mu[i].abs()).fold(0.0_f64, f64::max);

    // ── ∂x*/∂λ ──
    let an_l = solver.equilibrium_material_sensitivity(&x_final, None, DT, 1);
    let dl = LAMBDA0 * 1e-6;
    let fd_l: Vec<f64> = solve(MU0, LAMBDA0 + dl)
        .iter()
        .zip(&solve(MU0, LAMBDA0 - dl))
        .map(|(a, b)| (a - b) / (2.0 * dl))
        .collect();
    let rel_l = rel_free(&an_l, &fd_l, &free);
    let max_l = free.iter().map(|&i| an_l[i].abs()).fold(0.0_f64, f64::max);

    eprintln!(
        "S5 material sensitivity: ‖∂x*/∂μ‖_∞={max_mu:.3e} rel={rel_mu:.3e}  \
         ‖∂x*/∂λ‖_∞={max_l:.3e} rel={rel_l:.3e}"
    );
    assert!(
        max_mu > 1e-9 && max_l > 1e-9,
        "sensitivities implausibly small"
    );
    assert!(
        rel_mu < 1e-5,
        "∂x*/∂μ disagrees with re-solve FD: {rel_mu:e}"
    );
    assert!(rel_l < 1e-5, "∂x*/∂λ disagrees with re-solve FD: {rel_l:e}");
}

/// The reverse-mode `MaterialStepVjp` on a chassis tape: a one-node tape
/// `μ → x*` (the material VJP), `backward` (seeding ones ⇒ the implicit loss is
/// `L = Σ x*`), so `grad(μ) = ∂(Σx*)/∂μ`. Validated against the forward
/// sensitivity sum (the dual through the same factored `A`) AND a re-solve FD of
/// `Σx*` over μ — proving the reverse VJP, not merely self-consistency.
#[test]
fn material_step_vjp_backward_matches_forward_and_fd() {
    let solver = build(MU0, LAMBDA0);
    let nd = n_dof();
    let x_final = solve(MU0, LAMBDA0);

    // Tape: μ (leaf) → x* (MaterialStepVjp). backward seeds cot = ones[n_dof],
    // i.e. ∂(Σx*)/∂x*, so grad(μ) = ∂(Σx*)/∂μ.
    let mut tape = Tape::new();
    let mu_var = tape.param_tensor(Tensor::from_slice(&[MU0], &[1]));
    let xstar = tape.push_custom(
        &[mu_var],
        Tensor::from_slice(&x_final, &[nd]),
        Box::new(solver.material_step_vjp(&x_final, DT, 0)),
    );
    tape.backward(xstar);
    let grad_mu = tape.grad_tensor(mu_var).as_slice()[0];

    // Dual: Σ of the forward sensitivity ∂x*/∂μ (same factored A).
    let fwd_sum: f64 = solver
        .equilibrium_material_sensitivity(&x_final, None, DT, 0)
        .iter()
        .sum();
    // Independent: re-solve FD of Σx* over μ.
    let de = MU0 * 1e-6;
    let fd = (solve(MU0 + de, LAMBDA0).iter().sum::<f64>()
        - solve(MU0 - de, LAMBDA0).iter().sum::<f64>())
        / (2.0 * de);

    eprintln!(
        "S5 material VJP: grad(μ) tape={grad_mu:.6e}  forward-sum={fwd_sum:.6e}  FD={fd:.6e}"
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

/// A material that does NOT override `first_piola_param_grad` inherits the
/// trait default (empty) — so it exposes no differentiable parameters and a
/// material sensitivity over it is zero (never silently-wrong). `Yeoh` is such
/// a material today (only `NeoHookean` overrides), so it pins the default
/// contract without needing to construct the `#[non_exhaustive]` `ValidityDomain`.
#[test]
fn unadapted_material_param_grad_is_empty() {
    use sim_soft::Yeoh;
    let y = Yeoh::from_lame_and_c2(MU0, LAMBDA0, 1.0e3);
    assert!(
        y.first_piola_param_grad(&Matrix3::identity()).is_empty(),
        "a material without an override must expose no differentiable params (default empty)"
    );
}
