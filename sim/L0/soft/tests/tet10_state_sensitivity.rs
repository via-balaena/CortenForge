//! Tet10 ladder rung 7 — the time-adjoint (prev-state) sensitivity on the
//! quadratic element, and the dynamic mass-gate the static oracles cannot see.
//!
//! `equilibrium_state_sensitivity` (forward JVP) and `state_step_vjp`
//! (`StateStepVjp`, reverse) differentiate the converged step w.r.t. the previous
//! state `(x_prev, v_prev)`, which enters the backward-Euler residual only through
//! the predictor `x̂ = x_prev + Δt·v_prev`: `∂r/∂x_prev = −(M/Δt²)`,
//! `∂r/∂v_prev = −(M/Δt)` (the diagonal lumped mass), and `∂x*/∂(·) = −A⁻¹·∂r/∂(·)`
//! reuses the SAME tangent factored at `x_final`.
//!
//! For Tet10 the tangent `A` is per-Gauss-point (rung 4) and the mass diagonal is
//! the HRZ lump (rung 3b), so lifting the rung-7 `N == 4` diff-guard is all the
//! state channel needs — no RHS change (the mass diagonal is already the HRZ
//! lump). These gates run at a small `Δt` where `M/Δt²` genuinely competes with
//! stiffness, so a wrong Tet10 mass in the ADJOINT would surface here (the static
//! Lamé/#676 oracles are `STATIC_DT` and mass-blind). The absolute mass magnitude
//! is separately pinned by the crate-internal `tet10_hrz_mass_*` gates — the
//! forward and adjoint here share `mass_per_dof`, so a consistently-wrong mass
//! would pass BOTH sides of the FD (the wrong-but-matching trap).
//!
//! Gates: (1) the forward JVP for a prev-position and a prev-velocity direction
//! each match a re-solve central FD (the oracle re-runs the full nonlinear Newton
//! at the perturbed prev-state — touching no `A⁻¹`); (2) the reverse
//! `StateStepVjp` on a tape (`L = Σx*`) matches the forward dual AND a re-solve FD,
//! for both parents; (3) the world-pinned base carries exactly zero cotangent.

#![allow(
    clippy::expect_used,
    clippy::similar_names,
    clippy::float_cmp,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::element::Tet10;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet10NHSolver, HandBuiltTetMesh, LoadAxis,
    MaterialField, Mesh, NullContact, Solver, SolverConfig, Tet10Mesh, VertexId,
};

const N: usize = 2;
const EDGE: f64 = 0.1;
const MU: f64 = 3.0e4;
const LAMBDA: f64 = 1.2e5;
// Small dt: the M/Δt² inertia term genuinely competes with elasticity, so the
// prev-state sensitivity is non-trivial (neither ≈0 nor ≈I) and a wrong ADJOINT
// mass would move the reading.
const DT: f64 = 1.0e-2;
const THETA: f64 = 8.0; // +ẑ load on the top face → real deformation

/// A Tet10 solver (pin bottom-face corners, load top-face corners `+ẑ`) plus its
/// rest DOFs and free-DOF list.
fn build() -> (CpuTet10NHSolver<Tet10Mesh>, Vec<f64>, Vec<usize>) {
    let cube = HandBuiltTetMesh::uniform_block(N, EDGE, &MaterialField::uniform(MU, LAMBDA));
    let pos = cube.positions().to_vec();
    let n_corners = cube.n_vertices();
    let pinned: Vec<VertexId> = (0..n_corners as VertexId)
        .filter(|&v| pos[v as usize].z.abs() < 1e-9)
        .collect();
    let loaded: Vec<(VertexId, LoadAxis)> = (0..n_corners as VertexId)
        .filter(|&v| (pos[v as usize].z - EDGE).abs() < 1e-9)
        .map(|v| (v, LoadAxis::AxisZ))
        .collect();
    assert!(!pinned.is_empty() && !loaded.is_empty());
    let mesh = Tet10Mesh::from_tet4(&cube);
    let p = mesh.positions();
    let mut rest = vec![0.0_f64; 3 * p.len()];
    for (v, pt) in p.iter().enumerate() {
        rest[3 * v] = pt.x;
        rest[3 * v + 1] = pt.y;
        rest[3 * v + 2] = pt.z;
    }
    // Free DOFs = every DOF whose vertex is not a pinned corner. The uniform
    // block has no orphan midsides, so rung 3b frees all midsides and every
    // non-pinned corner — exactly the solver's `free_dof_indices` (no public
    // accessor, so we reconstruct it from the pins the BC just set).
    let n_dof = 3 * p.len();
    let pinned_set: std::collections::HashSet<usize> = pinned.iter().map(|&v| v as usize).collect();
    let free: Vec<usize> = (0..n_dof)
        .filter(|&i| !pinned_set.contains(&(i / 3)))
        .collect();
    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded,
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.max_newton_iter = 80;
    let solver = CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);
    (solver, rest, free)
}

/// One step's converged `x*` from `(x_prev, v_prev)` at load `THETA` (no tape).
fn solve(s: &CpuTet10NHSolver<Tet10Mesh>, x_prev: &[f64], v_prev: &[f64]) -> Vec<f64> {
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
fn direction(free: &[usize], n: usize, seed: u64) -> Vec<f64> {
    let mut d = vec![0.0_f64; n];
    for (k, &i) in free.iter().enumerate() {
        d[i] = (((k as u64 + seed) % 5) as f64 - 2.0) * 0.3 + 0.1;
    }
    d
}

fn rel_free(free: &[usize], a: &[f64], b: &[f64]) -> f64 {
    let num: f64 = free.iter().map(|&i| (a[i] - b[i]).powi(2)).sum();
    let den: f64 = free.iter().map(|&i| b[i] * b[i]).sum();
    (num / den.max(1e-300)).sqrt()
}

#[test]
fn tet10_forward_state_jvp_matches_resolve_fd() {
    let (s, x0, free) = build();
    let n = x0.len();
    let v0 = vec![0.0_f64; n];
    let x1 = solve(&s, &x0, &v0);
    let zeros = vec![0.0_f64; n];
    let eps = 1.0e-7;

    // ── ∂x* for a prev-POSITION direction (dv_prev = 0) ──
    let dxp = direction(&free, n, 1);
    let an_x = s.equilibrium_state_sensitivity(&x1, None, DT, &dxp, &zeros);
    let xp: Vec<f64> = x0.iter().zip(&dxp).map(|(a, d)| a + eps * d).collect();
    let xm: Vec<f64> = x0.iter().zip(&dxp).map(|(a, d)| a - eps * d).collect();
    let fd_x: Vec<f64> = solve(&s, &xp, &v0)
        .iter()
        .zip(&solve(&s, &xm, &v0))
        .map(|(a, b)| (a - b) / (2.0 * eps))
        .collect();
    let rel_x = rel_free(&free, &an_x, &fd_x);
    let mag_x = free.iter().map(|&i| an_x[i].abs()).fold(0.0, f64::max);

    // ── ∂x* for a prev-VELOCITY direction (dx_prev = 0) ──
    let dvp = direction(&free, n, 3);
    let an_v = s.equilibrium_state_sensitivity(&x1, None, DT, &zeros, &dvp);
    let vp: Vec<f64> = v0.iter().zip(&dvp).map(|(a, d)| a + eps * d).collect();
    let vm: Vec<f64> = v0.iter().zip(&dvp).map(|(a, d)| a - eps * d).collect();
    let fd_v: Vec<f64> = solve(&s, &x0, &vp)
        .iter()
        .zip(&solve(&s, &x0, &vm))
        .map(|(a, b)| (a - b) / (2.0 * eps))
        .collect();
    let rel_v = rel_free(&free, &an_v, &fd_v);
    let mag_v = free.iter().map(|&i| an_v[i].abs()).fold(0.0, f64::max);

    eprintln!(
        "Tet10 state JVP: ∂x*/∂x_prev ‖·‖_∞={mag_x:.3e} rel={rel_x:.3e}  \
         ∂x*/∂v_prev ‖·‖_∞={mag_v:.3e} rel={rel_v:.3e}"
    );
    assert!(
        mag_x > 1e-9 && mag_v > 1e-9,
        "sensitivities implausibly small"
    );
    assert!(
        rel_x < 1e-5,
        "Tet10 ∂x*/∂x_prev disagrees with FD: {rel_x:e}"
    );
    assert!(
        rel_v < 1e-5,
        "Tet10 ∂x*/∂v_prev disagrees with FD: {rel_v:e}"
    );
}

#[test]
fn tet10_state_step_vjp_backward_matches_forward_and_fd() {
    let (s, x0, free) = build();
    let n = x0.len();
    let v0 = vec![0.0_f64; n];
    let x1 = solve(&s, &x0, &v0);

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

    let eps = 1.0e-7;
    let zeros = vec![0.0_f64; n];
    let dot = |a: &[f64], b: &[f64]| -> f64 { a.iter().zip(b).map(|(x, y)| x * y).sum() };

    let dxp = direction(&free, n, 1);
    let rev_x = dot(&grad_xprev, &dxp);
    let fwd_x: f64 = s
        .equilibrium_state_sensitivity(&x1, None, DT, &dxp, &zeros)
        .iter()
        .sum();
    let xp: Vec<f64> = x0.iter().zip(&dxp).map(|(a, d)| a + eps * d).collect();
    let xm: Vec<f64> = x0.iter().zip(&dxp).map(|(a, d)| a - eps * d).collect();
    let fd_x = (solve(&s, &xp, &v0).iter().sum::<f64>() - solve(&s, &xm, &v0).iter().sum::<f64>())
        / (2.0 * eps);

    let dvp = direction(&free, n, 3);
    let rev_v = dot(&grad_vprev, &dvp);
    let fwd_v: f64 = s
        .equilibrium_state_sensitivity(&x1, None, DT, &zeros, &dvp)
        .iter()
        .sum();
    let vp: Vec<f64> = v0.iter().zip(&dvp).map(|(a, d)| a + eps * d).collect();
    let vm: Vec<f64> = v0.iter().zip(&dvp).map(|(a, d)| a - eps * d).collect();
    let fd_v = (solve(&s, &x0, &vp).iter().sum::<f64>() - solve(&s, &x0, &vm).iter().sum::<f64>())
        / (2.0 * eps);

    eprintln!(
        "Tet10 state VJP: x_prev rev={rev_x:.6e} fwd={fwd_x:.6e} fd={fd_x:.6e} | \
         v_prev rev={rev_v:.6e} fwd={fwd_v:.6e} fd={fd_v:.6e}"
    );
    assert!(
        rev_x.abs() > 1e-9 && rev_v.abs() > 1e-9,
        "VJP·d implausibly small"
    );
    assert!(
        (rev_x - fwd_x).abs() / fwd_x.abs() < 1e-10,
        "x_prev: rev {rev_x} != fwd {fwd_x}"
    );
    assert!(
        (rev_v - fwd_v).abs() / fwd_v.abs() < 1e-10,
        "v_prev: rev {rev_v} != fwd {fwd_v}"
    );
    assert!(
        (rev_x - fd_x).abs() / fd_x.abs() < 1e-5,
        "x_prev: rev {rev_x} != FD {fd_x}"
    );
    assert!(
        (rev_v - fd_v).abs() / fd_v.abs() < 1e-5,
        "v_prev: rev {rev_v} != FD {fd_v}"
    );

    // The world-pinned DOFs carry exactly zero cotangent (constant leaves).
    for (i, (&gx, &gv)) in grad_xprev.iter().zip(&grad_vprev).enumerate() {
        if !free.contains(&i) {
            assert_eq!(gx, 0.0, "pinned x_prev DOF {i} nonzero");
            assert_eq!(gv, 0.0, "pinned v_prev DOF {i} nonzero");
        }
    }
}
