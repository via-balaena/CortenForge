//! Keystone time-adjoint (PR1) — soft prev-state sensitivity `∂x*/∂(x_prev, v_prev)`.
//!
//! The soft autograd (`NewtonStepVjp`/`MaterialStepVjp`) differentiates the
//! converged equilibrium `x*` w.r.t. a scalar load / material parameter. The
//! multi-step time-adjoint needs the converged step differentiated w.r.t. the
//! PREVIOUS state that threads one step to the next:
//! `CpuNewtonSolver::equilibrium_state_sensitivity` (forward JVP) and
//! `state_step_vjp` → `StateStepVjp` (reverse, two parents). The previous state
//! enters the backward-Euler residual only through the predictor
//! `x̂ = x_prev + Δt·v_prev`, so `∂r/∂x_prev = −(M/Δt²)`, `∂r/∂v_prev = −(M/Δt)`
//! (diagonal lumped mass), and `∂x*/∂(·) = −A⁻¹·∂r/∂(·)` reuses the SAME tangent
//! factored at `x_final`. With these per-step state Jacobians on the tape, one
//! `tape.backward` chains across a rollout (PR2 builds the coupled trajectory).
//!
//! Gates: (1) the forward JVP `∂x*` for a prev-position and a prev-velocity
//! direction each match a re-solve central FD (the oracle re-runs the full
//! nonlinear Newton at perturbed prev-state — touching no `A⁻¹`); (2) the
//! reverse `StateStepVjp` on a tape (`L = Σx*`) matches the forward-sensitivity
//! dual AND a re-solve FD of `Σx*`, for both the `x_prev` and `v_prev` parents;
//! (3) the world-pinned base carries exactly zero cotangent (it is a constant
//! outside the differentiable rollout thread).

// expect/precision: test idioms. similar_names: the parallel `_x`/`_v`
// (x_prev/v_prev) probe naming is the clearest scheme here. float_cmp: the
// pinned-base cotangent is EXACTLY 0.0 (the VJP scatters onto free DOFs only and
// never touches pinned entries), so an exact `assert_eq!(·, 0.0)` is the intent.
#![allow(
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::float_cmp
)]

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::readout::scene::pick_vertices_by_predicate;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, LoadAxis, MaterialField, Mesh,
    NullContact, Solver, SolverConfig, Tet4, VertexId,
};

// ── Scene constants ───────────────────────────────────────────────────────
const N: usize = 2;
const EDGE: f64 = 0.1;
const MU: f64 = 3.0e4;
const LAMBDA: f64 = 1.2e5;
// Small dt: the M/Δt² inertia term genuinely competes with elasticity, so the
// prev-state sensitivity is non-trivial (neither ≈0 nor ≈I).
const DT: f64 = 1.0e-2;
const THETA: f64 = 8.0; // +ẑ load on the top face → real deformation

type NhSolver = CpuNewtonSolver<Tet4, HandBuiltTetMesh, NullContact>;

fn block() -> HandBuiltTetMesh {
    HandBuiltTetMesh::uniform_block(N, EDGE, &MaterialField::uniform(MU, LAMBDA))
}

fn build() -> NhSolver {
    let pinned: Vec<VertexId> = pick_vertices_by_predicate(&block(), |p| p.z.abs() < 1e-9);
    let loaded: Vec<VertexId> = pick_vertices_by_predicate(&block(), |p| (p.z - EDGE).abs() < 1e-9);
    let bc = BoundaryConditions::new(
        pinned,
        loaded.iter().map(|&v| (v, LoadAxis::AxisZ)).collect(),
    );
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.max_newton_iter = 50;
    CpuNewtonSolver::new(Tet4, block(), NullContact, cfg, bc)
}

fn n_dof() -> usize {
    3 * block().n_vertices()
}

/// Rest positions (vertex-major xyz).
fn x_rest() -> Vec<f64> {
    let mut x = vec![0.0_f64; n_dof()];
    for (c, p) in x.chunks_exact_mut(3).zip(block().positions().iter()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    x
}

/// Free-DOF indices = all DOFs of non-base (z>0) vertices. The z=0 base is
/// world-pinned (Dirichlet) and constant across the rollout — NOT part of the
/// differentiable state thread.
fn free_dofs() -> Vec<usize> {
    let mut free = Vec::new();
    for (v, p) in block().positions().iter().enumerate() {
        if p.z.abs() >= 1e-9 {
            free.extend([3 * v, 3 * v + 1, 3 * v + 2]);
        }
    }
    free
}

/// One step's converged `x*` from `(x_prev, v_prev)` at load `THETA` (no tape).
fn solve(s: &NhSolver, x_prev: &[f64], v_prev: &[f64]) -> Vec<f64> {
    let n = x_prev.len();
    s.replay_step(
        &Tensor::from_slice(x_prev, &[n]),
        &Tensor::from_slice(v_prev, &[n]),
        &Tensor::from_slice(&[THETA], &[1]),
        DT,
    )
    .x_final
}

/// A reproducible free-only perturbation direction (deterministic pattern).
fn direction(seed: u64) -> Vec<f64> {
    let mut d = vec![0.0_f64; n_dof()];
    for (k, &i) in free_dofs().iter().enumerate() {
        d[i] = (((k as u64 + seed) % 5) as f64 - 2.0) * 0.3 + 0.1;
    }
    d
}

fn rel_free(a: &[f64], b: &[f64]) -> f64 {
    let num: f64 = free_dofs().iter().map(|&i| (a[i] - b[i]).powi(2)).sum();
    let den: f64 = free_dofs().iter().map(|&i| b[i] * b[i]).sum();
    (num / den.max(1e-300)).sqrt()
}

#[test]
fn forward_state_jvp_matches_resolve_fd() {
    let s = build();
    let n = n_dof();
    let x0 = x_rest();
    let v0 = vec![0.0_f64; n];
    let x1 = solve(&s, &x0, &v0);
    let zeros = vec![0.0_f64; n];
    let eps = 1.0e-7;

    // ── ∂x* for a prev-POSITION direction (dv_prev = 0) ──
    let dxp = direction(1);
    let an_x = s.equilibrium_state_sensitivity(&x1, None, DT, &dxp, &zeros);
    let xp_plus: Vec<f64> = x0.iter().zip(&dxp).map(|(a, d)| a + eps * d).collect();
    let xp_minus: Vec<f64> = x0.iter().zip(&dxp).map(|(a, d)| a - eps * d).collect();
    let fd_x: Vec<f64> = solve(&s, &xp_plus, &v0)
        .iter()
        .zip(&solve(&s, &xp_minus, &v0))
        .map(|(a, b)| (a - b) / (2.0 * eps))
        .collect();
    let rel_x = rel_free(&an_x, &fd_x);
    let mag_x = free_dofs()
        .iter()
        .map(|&i| an_x[i].abs())
        .fold(0.0, f64::max);

    // ── ∂x* for a prev-VELOCITY direction (dx_prev = 0) ──
    let dvp = direction(3);
    let an_v = s.equilibrium_state_sensitivity(&x1, None, DT, &zeros, &dvp);
    let vp_plus: Vec<f64> = v0.iter().zip(&dvp).map(|(a, d)| a + eps * d).collect();
    let vp_minus: Vec<f64> = v0.iter().zip(&dvp).map(|(a, d)| a - eps * d).collect();
    let fd_v: Vec<f64> = solve(&s, &x0, &vp_plus)
        .iter()
        .zip(&solve(&s, &x0, &vp_minus))
        .map(|(a, b)| (a - b) / (2.0 * eps))
        .collect();
    let rel_v = rel_free(&an_v, &fd_v);
    let mag_v = free_dofs()
        .iter()
        .map(|&i| an_v[i].abs())
        .fold(0.0, f64::max);

    eprintln!(
        "forward state JVP: ∂x*/∂x_prev·d ‖·‖_∞={mag_x:.3e} rel={rel_x:.3e}  \
         ∂x*/∂v_prev·d ‖·‖_∞={mag_v:.3e} rel={rel_v:.3e}"
    );
    assert!(
        mag_x > 1e-9 && mag_v > 1e-9,
        "sensitivities implausibly small"
    );
    assert!(
        rel_x < 1e-5,
        "∂x*/∂x_prev·d disagrees with re-solve FD: {rel_x:e}"
    );
    assert!(
        rel_v < 1e-5,
        "∂x*/∂v_prev·d disagrees with re-solve FD: {rel_v:e}"
    );
}

/// The reverse-mode `StateStepVjp` on a chassis tape: a one-node tape
/// `(x_prev, v_prev) → x*`, `backward` seeding ones (so the implicit loss is
/// `L = Σx*`), so `grad(x_prev) = ∂(Σx*)/∂x_prev`, `grad(v_prev) = ∂(Σx*)/∂v_prev`.
/// Validated, for a probe direction `d`, against the forward-sensitivity dual
/// `Σ equilibrium_state_sensitivity(d)` AND a re-solve FD of `Σx*` — proving the
/// reverse VJP, not mere self-consistency. Also checks the pinned base is zero.
#[test]
fn state_step_vjp_backward_matches_forward_and_fd() {
    let s = build();
    let n = n_dof();
    let x0 = x_rest();
    let v0 = vec![0.0_f64; n];
    let x1 = solve(&s, &x0, &v0);

    // Tape: (x_prev, v_prev) leaves → x* (StateStepVjp). backward seeds cot =
    // ones[n_dof] = ∂(Σx*)/∂x*, so grad(x_prev)=∂(Σx*)/∂x_prev etc.
    let mut tape = Tape::new();
    let xprev_var = tape.param_tensor(Tensor::from_slice(&x0, &[n]));
    let vprev_var = tape.param_tensor(Tensor::from_slice(&v0, &[n]));
    let xstar = tape.push_custom(
        &[xprev_var, vprev_var],
        Tensor::from_slice(&x1, &[n]),
        Box::new(s.state_step_vjp(&x1, DT)),
    );
    tape.backward(xstar);
    let grad_xprev = tape.grad_tensor(xprev_var).as_slice().to_vec();
    let grad_vprev = tape.grad_tensor(vprev_var).as_slice().to_vec();

    // Probe directions: reverse·d  vs  forward-sum(d)  vs  re-solve FD of Σx*.
    let eps = 1.0e-7;
    let zeros = vec![0.0_f64; n];
    let dot = |a: &[f64], b: &[f64]| -> f64 { a.iter().zip(b).map(|(x, y)| x * y).sum() };

    // x_prev parent.
    let dxp = direction(1);
    let rev_x = dot(&grad_xprev, &dxp);
    let fwd_x: f64 = s
        .equilibrium_state_sensitivity(&x1, None, DT, &dxp, &zeros)
        .iter()
        .sum();
    let xp_plus: Vec<f64> = x0.iter().zip(&dxp).map(|(a, d)| a + eps * d).collect();
    let xp_minus: Vec<f64> = x0.iter().zip(&dxp).map(|(a, d)| a - eps * d).collect();
    let fd_x = (solve(&s, &xp_plus, &v0).iter().sum::<f64>()
        - solve(&s, &xp_minus, &v0).iter().sum::<f64>())
        / (2.0 * eps);

    // v_prev parent.
    let dvp = direction(3);
    let rev_v = dot(&grad_vprev, &dvp);
    let fwd_v: f64 = s
        .equilibrium_state_sensitivity(&x1, None, DT, &zeros, &dvp)
        .iter()
        .sum();
    let vp_plus: Vec<f64> = v0.iter().zip(&dvp).map(|(a, d)| a + eps * d).collect();
    let vp_minus: Vec<f64> = v0.iter().zip(&dvp).map(|(a, d)| a - eps * d).collect();
    let fd_v = (solve(&s, &x0, &vp_plus).iter().sum::<f64>()
        - solve(&s, &x0, &vp_minus).iter().sum::<f64>())
        / (2.0 * eps);

    eprintln!(
        "state VJP: x_prev rev={rev_x:.6e} fwd={fwd_x:.6e} fd={fd_x:.6e} | \
         v_prev rev={rev_v:.6e} fwd={fwd_v:.6e} fd={fd_v:.6e}"
    );
    assert!(
        rev_x.abs() > 1e-9 && rev_v.abs() > 1e-9,
        "VJP·d implausibly small"
    );
    // Reverse == forward dual (same factored A; machine-exact).
    assert!(
        (rev_x - fwd_x).abs() / fwd_x.abs() < 1e-10,
        "x_prev: rev {rev_x} != fwd {fwd_x}"
    );
    assert!(
        (rev_v - fwd_v).abs() / fwd_v.abs() < 1e-10,
        "v_prev: rev {rev_v} != fwd {fwd_v}"
    );
    // Reverse == independent re-solve FD.
    assert!(
        (rev_x - fd_x).abs() / fd_x.abs() < 1e-5,
        "x_prev: rev {rev_x} != FD {fd_x}"
    );
    assert!(
        (rev_v - fd_v).abs() / fd_v.abs() < 1e-5,
        "v_prev: rev {rev_v} != FD {fd_v}"
    );

    // The world-pinned base carries exactly zero cotangent (constant leaf).
    for (v, p) in block().positions().iter().enumerate() {
        if p.z.abs() < 1e-9 {
            for ax in 0..3 {
                assert_eq!(grad_xprev[3 * v + ax], 0.0, "pinned x_prev DOF nonzero");
                assert_eq!(grad_vprev[3 * v + ax], 0.0, "pinned v_prev DOF nonzero");
            }
        }
    }
}
