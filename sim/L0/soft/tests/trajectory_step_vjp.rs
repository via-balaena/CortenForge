//! Keystone time-adjoint (PR2) — `TrajectoryStepVjp` per-step machine-exact gate.
//!
//! `TrajectoryStepVjp` fuses the prev-state (PR1), material (S5), and
//! contact-pose (S3) adjoints of ONE converged Newton step into a single op with
//! four parents `[x_prev, v_prev, param, pose]`, so a single shared `A·λ = g_free`
//! solve yields every parent cotangent. Each constituent is FD-gated on its own
//! (`state_sensitivity.rs`, `material_sensitivity.rs`, `soft_pose_sensitivity.rs`);
//! this gate validates the FUSION — that the one tape node routes the right
//! cotangent to each of the four parents — by checking all four against
//! independent re-solve FDs in the deeply-engaged, stable-active-set regime.
//!
//! The FD oracle re-runs the full nonlinear backward-Euler Newton solve at the
//! perturbed input (prev state, material, or plane height), touching NONE of the
//! tape / `A` machinery — a genuine cross-check, not an affine identity.

// precision: the small integer pattern index `k % 5` is exact as f64.
#![allow(
    clippy::expect_used,
    clippy::similar_names,
    clippy::cast_precision_loss
)]

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::readout::scene::pick_vertices_by_predicate;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh,
    PenaltyRigidContact, PenaltyRigidContactSolver, RigidPlane, Solver, SolverConfig, Tet4, Vec3,
    VertexId,
};

const N_PER_EDGE: usize = 4;
const EDGE: f64 = 0.1;
const MU: f64 = 3.0e4;
const LAMBDA: f64 = 4.0 * MU;
const KAPPA: f64 = 3.0e4;
const D_HAT: f64 = 1.0e-2;
const DT: f64 = 1.0e-3;
const PENETRATION: f64 = 1.0e-3;
const SETTLE_STEPS: usize = 400;

fn field(mu: f64, lambda: f64) -> MaterialField {
    MaterialField::uniform(mu, lambda)
}

fn block(mu: f64, lambda: f64) -> HandBuiltTetMesh {
    HandBuiltTetMesh::uniform_block(N_PER_EDGE, EDGE, &field(mu, lambda))
}

fn bottom_pins() -> Vec<VertexId> {
    pick_vertices_by_predicate(&block(MU, LAMBDA), |p| p.z.abs() < 1e-9)
}

fn solver(height: f64, mu: f64, lambda: f64) -> PenaltyRigidContactSolver<HandBuiltTetMesh> {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -height);
    let contact = PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT);
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    CpuNewtonSolver::new(
        Tet4,
        block(mu, lambda),
        contact,
        cfg,
        BoundaryConditions::new(bottom_pins(), Vec::new()),
    )
}

/// One converged step's `x_final` from `(x_prev, v_prev)` at `(height, mu, lambda)`.
fn solve_step(height: f64, mu: f64, lambda: f64, x_prev: &[f64], v_prev: &[f64]) -> Vec<f64> {
    let n = x_prev.len();
    solver(height, mu, lambda)
        .replay_step(
            &Tensor::from_slice(x_prev, &[n]),
            &Tensor::from_slice(v_prev, &[n]),
            &Tensor::zeros(&[0]),
            DT,
        )
        .x_final
}

/// Deeply-engaged near-equilibrium: plane penetrating the top face by `PENETRATION`.
fn settle() -> (f64, Vec<f64>, Vec<f64>) {
    let mesh = block(MU, LAMBDA);
    let n = 3 * mesh.n_vertices();
    let height = EDGE - PENETRATION;
    let mut x = vec![0.0_f64; n];
    for (c, p) in x.chunks_exact_mut(3).zip(mesh.positions().iter()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    let mut v = vec![0.0_f64; n];
    for _ in 0..SETTLE_STEPS {
        let xf = solve_step(height, MU, LAMBDA, &x, &v);
        for (vi, (&xf_i, &xo)) in v.iter_mut().zip(xf.iter().zip(x.iter())) {
            *vi = (xf_i - xo) / DT;
        }
        x = xf;
    }
    (height, x, v)
}

fn free_dofs() -> Vec<usize> {
    let mut free = Vec::new();
    for (v, p) in block(MU, LAMBDA).positions().iter().enumerate() {
        if p.z.abs() >= 1e-9 {
            free.extend([3 * v, 3 * v + 1, 3 * v + 2]);
        }
    }
    free
}

/// Validates that `TrajectoryStepVjp` routes the correct cotangent to each of its
/// four parents — every one matched to an independent re-solve FD.
#[test]
fn trajectory_step_vjp_all_four_parents_match_fd() {
    let (height, x_prev, v_prev) = settle();
    let n = x_prev.len();
    let s = solver(height, MU, LAMBDA);
    let x_final = solve_step(height, MU, LAMBDA, &x_prev, &v_prev);

    // L = Σx*: build the 4-parent node and backward (cotangent = ones).
    let dir = Vec3::new(0.0, 0.0, 1.0);
    let mut tape = Tape::new();
    let xprev_var = tape.param_tensor(Tensor::from_slice(&x_prev, &[n]));
    let vprev_var = tape.param_tensor(Tensor::from_slice(&v_prev, &[n]));
    let mu_var = tape.param_tensor(Tensor::from_slice(&[MU], &[1]));
    let pose_var = tape.param_tensor(Tensor::from_slice(&[height], &[1]));
    let xstar = tape.push_custom(
        &[xprev_var, vprev_var, mu_var, pose_var],
        Tensor::from_slice(&x_final, &[n]),
        Box::new(s.trajectory_step_vjp(&x_final, DT, 0, dir)),
    );
    tape.backward(xstar);
    let g_xprev = tape.grad_tensor(xprev_var).as_slice().to_vec();
    let g_vprev = tape.grad_tensor(vprev_var).as_slice().to_vec();
    let g_mu = tape.grad_tensor(mu_var).as_slice()[0];
    let g_pose = tape.grad_tensor(pose_var).as_slice()[0];

    let sum = |x: &[f64]| -> f64 { x.iter().sum() };
    let dot = |a: &[f64], b: &[f64]| -> f64 { a.iter().zip(b).map(|(x, y)| x * y).sum() };
    let eps_pos = 1.0e-7;

    // (1) x_prev — directional FD along a free-only pattern.
    let mut d = vec![0.0_f64; n];
    for (k, &i) in free_dofs().iter().enumerate() {
        d[i] = ((k % 5) as f64 - 2.0) * 0.3 + 0.1;
    }
    let xp_plus: Vec<f64> = x_prev
        .iter()
        .zip(&d)
        .map(|(a, e)| a + eps_pos * e)
        .collect();
    let xp_minus: Vec<f64> = x_prev
        .iter()
        .zip(&d)
        .map(|(a, e)| a - eps_pos * e)
        .collect();
    let fd_xprev = (sum(&solve_step(height, MU, LAMBDA, &xp_plus, &v_prev))
        - sum(&solve_step(height, MU, LAMBDA, &xp_minus, &v_prev)))
        / (2.0 * eps_pos);
    let rev_xprev = dot(&g_xprev, &d);

    // (2) v_prev — directional FD.
    let vp_plus: Vec<f64> = v_prev
        .iter()
        .zip(&d)
        .map(|(a, e)| a + eps_pos * e)
        .collect();
    let vp_minus: Vec<f64> = v_prev
        .iter()
        .zip(&d)
        .map(|(a, e)| a - eps_pos * e)
        .collect();
    let fd_vprev = (sum(&solve_step(height, MU, LAMBDA, &x_prev, &vp_plus))
        - sum(&solve_step(height, MU, LAMBDA, &x_prev, &vp_minus)))
        / (2.0 * eps_pos);
    let rev_vprev = dot(&g_vprev, &d);

    // (3) μ — re-solve with the block rebuilt at μ±ε (λ held fixed).
    let eps_mu = MU * 1e-6;
    let fd_mu = (sum(&solve_step(height, MU + eps_mu, LAMBDA, &x_prev, &v_prev))
        - sum(&solve_step(height, MU - eps_mu, LAMBDA, &x_prev, &v_prev)))
        / (2.0 * eps_mu);

    // (4) pose — re-solve at height ± ε (the plane translation δ ≡ height).
    let eps_h = 1.0e-7;
    let fd_pose = (sum(&solve_step(height + eps_h, MU, LAMBDA, &x_prev, &v_prev))
        - sum(&solve_step(height - eps_h, MU, LAMBDA, &x_prev, &v_prev)))
        / (2.0 * eps_h);

    eprintln!(
        "TrajectoryStepVjp parents vs FD:\n  x_prev rev={rev_xprev:.6e} fd={fd_xprev:.6e}\n  \
         v_prev rev={rev_vprev:.6e} fd={fd_vprev:.6e}\n  μ rev={g_mu:.6e} fd={fd_mu:.6e}\n  \
         pose rev={g_pose:.6e} fd={fd_pose:.6e}"
    );

    let rel = |a: f64, b: f64| (a - b).abs() / b.abs().max(1e-12);
    // Each cotangent is a genuine contributor (nonzero) and matches its FD.
    assert!(
        rev_xprev.abs() > 1e-6 && rel(rev_xprev, fd_xprev) < 1e-5,
        "x_prev cotangent"
    );
    // v_prev tolerates a looser FD bound: its factor is M/Δt (vs M/Δt² for
    // x_prev), so the term — and its FD signal-to-noise — is Δt(=1e-3) smaller.
    assert!(
        rev_vprev.abs() > 1e-9 && rel(rev_vprev, fd_vprev) < 1e-4,
        "v_prev cotangent"
    );
    assert!(g_mu.abs() > 1e-9 && rel(g_mu, fd_mu) < 1e-5, "μ cotangent");
    assert!(
        g_pose.abs() > 1e-3 && rel(g_pose, fd_pose) < 1e-5,
        "pose cotangent"
    );
}

/// The `param_idx = 1` (λ) slot of the fused VJP is independently FD-valid too
/// (the multi-step trajectory gate only checks the λ = 4μ combination).
#[test]
fn trajectory_step_vjp_lambda_parent_matches_fd() {
    let (height, x_prev, v_prev) = settle();
    let n = x_prev.len();
    let s = solver(height, MU, LAMBDA);
    let x_final = solve_step(height, MU, LAMBDA, &x_prev, &v_prev);

    let mut tape = Tape::new();
    let xprev_var = tape.param_tensor(Tensor::from_slice(&x_prev, &[n]));
    let vprev_var = tape.param_tensor(Tensor::from_slice(&v_prev, &[n]));
    let lam_var = tape.param_tensor(Tensor::from_slice(&[LAMBDA], &[1]));
    let pose_var = tape.param_tensor(Tensor::from_slice(&[height], &[1]));
    let xstar = tape.push_custom(
        &[xprev_var, vprev_var, lam_var, pose_var],
        Tensor::from_slice(&x_final, &[n]),
        Box::new(s.trajectory_step_vjp(&x_final, DT, 1, Vec3::new(0.0, 0.0, 1.0))),
    );
    tape.backward(xstar);
    let g_lambda = tape.grad_tensor(lam_var).as_slice()[0];

    let sum = |x: &[f64]| -> f64 { x.iter().sum() };
    let eps_l = LAMBDA * 1e-6;
    let fd_lambda = (sum(&solve_step(height, MU, LAMBDA + eps_l, &x_prev, &v_prev))
        - sum(&solve_step(height, MU, LAMBDA - eps_l, &x_prev, &v_prev)))
        / (2.0 * eps_l);
    eprintln!("λ parent: rev={g_lambda:.6e} fd={fd_lambda:.6e}");
    assert!(
        g_lambda.abs() > 1e-9 && (g_lambda - fd_lambda).abs() / fd_lambda.abs() < 1e-5,
        "λ cotangent rev {g_lambda} != FD {fd_lambda}"
    );
}

/// The COMBINED-weights VJP (`trajectory_step_vjp_combined`) routes the *total*
/// gradient of a tied design variable through ONE material parent: with weights
/// `[1, 4]` (the coupling's stiffness scale `μ = p, λ = 4p`) the parent cotangent
/// equals `∂/∂μ + 4·∂/∂λ`, matched two ways — (a) against the sum of the two
/// single-`param_idx` cotangents (a pure linearity check), and (b) against an
/// independent re-solve FD that rebuilds the block along the `λ = 4μ` line (the
/// genuine total derivative). This is the gradient the joint design+policy loop
/// reads from one backward.
#[test]
fn trajectory_step_vjp_combined_weights_match_total() {
    let (height, x_prev, v_prev) = settle();
    let n = x_prev.len();
    let s = solver(height, MU, LAMBDA);
    let x_final = solve_step(height, MU, LAMBDA, &x_prev, &v_prev);
    let dir = Vec3::new(0.0, 0.0, 1.0);
    let sum = |x: &[f64]| -> f64 { x.iter().sum() };

    // (a) combined [1, 4] cotangent on a single design-variable parent.
    let mut tape = Tape::new();
    let xprev_var = tape.param_tensor(Tensor::from_slice(&x_prev, &[n]));
    let vprev_var = tape.param_tensor(Tensor::from_slice(&v_prev, &[n]));
    let p_var = tape.param_tensor(Tensor::from_slice(&[MU], &[1])); // design var value = μ
    let pose_var = tape.param_tensor(Tensor::from_slice(&[height], &[1]));
    let xstar = tape.push_custom(
        &[xprev_var, vprev_var, p_var, pose_var],
        Tensor::from_slice(&x_final, &[n]),
        Box::new(s.trajectory_step_vjp_combined(&x_final, DT, &[1.0, 4.0], dir)),
    );
    tape.backward(xstar);
    let g_combined = tape.grad_tensor(p_var).as_slice()[0];

    // (b) sum of the two single-param cotangents (linearity).
    let g_single_sum = {
        let single = |idx: usize, val: f64| {
            let mut t = Tape::new();
            let xp = t.param_tensor(Tensor::from_slice(&x_prev, &[n]));
            let vp = t.param_tensor(Tensor::from_slice(&v_prev, &[n]));
            let pv = t.param_tensor(Tensor::from_slice(&[val], &[1]));
            let po = t.param_tensor(Tensor::from_slice(&[height], &[1]));
            let xs = t.push_custom(
                &[xp, vp, pv, po],
                Tensor::from_slice(&x_final, &[n]),
                Box::new(s.trajectory_step_vjp(&x_final, DT, idx, dir)),
            );
            t.backward(xs);
            t.grad_tensor(pv).as_slice()[0]
        };
        single(0, MU) + 4.0 * single(1, LAMBDA)
    };

    // (c) independent re-solve FD along the λ = 4μ line (μ → μ+ε, λ → 4(μ+ε)).
    let eps = MU * 1e-6;
    let fd_total = (sum(&solve_step(
        height,
        MU + eps,
        4.0 * (MU + eps),
        &x_prev,
        &v_prev,
    )) - sum(&solve_step(
        height,
        MU - eps,
        4.0 * (MU - eps),
        &x_prev,
        &v_prev,
    ))) / (2.0 * eps);

    eprintln!(
        "combined [1,4]: rev={g_combined:.6e} single-sum={g_single_sum:.6e} fd(line)={fd_total:.6e}"
    );
    let rel = |a: f64, b: f64| (a - b).abs() / b.abs().max(1e-12);
    assert!(
        rel(g_combined, g_single_sum) < 1e-12,
        "combined cotangent {g_combined} != single-param sum {g_single_sum} (linearity)"
    );
    assert!(
        g_combined.abs() > 1e-9 && rel(g_combined, fd_total) < 1e-5,
        "combined cotangent {g_combined} != rebuild-line FD {fd_total}"
    );
}
