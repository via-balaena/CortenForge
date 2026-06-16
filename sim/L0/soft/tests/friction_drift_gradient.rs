//! Keystone friction leaf PR3b-1 (sim-soft half) — DIFFERENTIABILITY of the moving-collider
//! friction drift. With the Woodbury correction built at the drift-consistent reference, the
//! IFT adjoint `A` stays exact under a moving collider, so the forward sensitivities carry the
//! drift. (1) `∂x*/∂μ` (material) stays machine-exact when a nonzero collider drift is present.
//! (2) `∂x*/∂Δ_surf` (the NEW drift sensitivity) — the drift enters only the friction term
//! (`∂r/∂Δ_surf = −∇²D`), reusing the same factored `A`.
//! Both are FD-gated against a friction re-solve. The coupling (PR3b-2) contracts
//! `∂x*/∂Δ_surf` with `∂Δ_surf/∂(rigid velocity)` to thread the two-way grip feedback.
//!
//! Scene = PR2's `friction_diff` (load-driven, so `∂x*/∂μ` is non-degenerate) + a fixed drift.

#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names
)]

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, LoadAxis, MaterialField, Mesh,
    PenaltyRigidContact, RigidPlane, Solver, SolverConfig, Tet4, Vec3, pick_vertices_by_predicate,
};

const EDGE: f64 = 0.1;
const DT: f64 = 1.0e-3;
const KAPPA: f64 = 5.0e3;
const GRAV_UP: f64 = 10.0;
const MU0: f64 = 3.0e4;
const LAMBDA0: f64 = 1.2e5;
const FRIC_MU: f64 = 3.0;
const EPS_V: f64 = 0.1;
const FX_TOTAL: f64 = 40.0;
const DRIFT: f64 = 5.0e-4; // deep-slip collider drift (the grip regime)

fn block(mu: f64, lambda: f64) -> HandBuiltTetMesh {
    HandBuiltTetMesh::uniform_block(4, EDGE, &MaterialField::uniform(mu, lambda))
}
fn n() -> usize {
    block(MU0, LAMBDA0).n_vertices()
}
fn x_rest() -> Vec<f64> {
    let mesh = block(MU0, LAMBDA0);
    let mut x = vec![0.0_f64; 3 * mesh.n_vertices()];
    for (i, p) in mesh.positions().iter().enumerate() {
        x[3 * i] = p.x;
        x[3 * i + 1] = p.y;
        x[3 * i + 2] = p.z;
    }
    x
}
fn top_vertices() -> Vec<usize> {
    let mesh = block(MU0, LAMBDA0);
    let rest = mesh.positions();
    (0..mesh.n_vertices())
        .filter(|&i| (rest[i].z - EDGE).abs() < 1e-9)
        .collect()
}
fn theta() -> Vec<f64> {
    let top = top_vertices();
    let fx = FX_TOTAL / top.len() as f64;
    let mut th = vec![0.0_f64; 3 * top.len()];
    for k in 0..top.len() {
        th[3 * k] = fx;
    }
    th
}
fn build(
    mu: f64,
    lambda: f64,
    drift_x: f64,
) -> CpuNewtonSolver<Tet4, HandBuiltTetMesh, PenaltyRigidContact> {
    build_dh(mu, lambda, drift_x, 0.0)
}
/// As [`build`] but the ceiling plane is translated `+ẑ` by `dh` (the `pose_dir`
/// height perturbation used to FD-gate `∂F/∂height`); `dh = 0` recovers `build`.
fn build_dh(
    mu: f64,
    lambda: f64,
    drift_x: f64,
    dh: f64,
) -> CpuNewtonSolver<Tet4, HandBuiltTetMesh, PenaltyRigidContact> {
    let mesh = block(mu, lambda);
    let pinned = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    let top = top_vertices();
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -(EDGE + dh));
    let contact = PenaltyRigidContact::with_params([plane], KAPPA, 5.0e-3);
    let loaded: Vec<(u32, LoadAxis)> = top
        .iter()
        .map(|&i| (i as u32, LoadAxis::FullVector))
        .collect();
    let bc = BoundaryConditions::new(pinned, loaded);
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.gravity_z = GRAV_UP;
    cfg.friction_mu = FRIC_MU;
    cfg.friction_eps_v = EPS_V;
    cfg.max_newton_iter = 80;
    cfg.max_line_search_backtracks = 60;
    CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc)
        .with_friction_surface_drift(Vec3::new(drift_x, 0.0, 0.0))
}
fn solve(mu: f64, lambda: f64, drift_x: f64) -> Vec<f64> {
    let nd = 3 * n();
    let top = top_vertices();
    build(mu, lambda, drift_x)
        .replay_step(
            &Tensor::from_slice(&x_rest(), &[nd]),
            &Tensor::from_slice(&vec![0.0_f64; nd], &[nd]),
            &Tensor::from_slice(&theta(), &[3 * top.len()]),
            DT,
        )
        .x_final
}
fn rel(a: &[f64], b: &[f64]) -> f64 {
    let num: f64 = a
        .iter()
        .zip(b)
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f64>()
        .sqrt();
    let den: f64 = b.iter().map(|y| y * y).sum::<f64>().sqrt();
    num / den.max(1e-30)
}

/// `∂x*/∂μ` stays machine-exact with a nonzero collider drift (the drift-consistent Woodbury
/// keeps the adjoint `A` exact). Without the drift-consistent reference this lands ~73% off.
#[test]
fn material_sensitivity_with_drift_matches_resolve_fd() {
    let solver = build(MU0, LAMBDA0, DRIFT);
    let x_prev = x_rest();
    let x_final = solve(MU0, LAMBDA0, DRIFT);
    let an = solver.equilibrium_material_sensitivity(&x_final, Some(&x_prev), DT, 0);
    let de = MU0 * 1e-6;
    let fd: Vec<f64> = solve(MU0 + de, LAMBDA0, DRIFT)
        .iter()
        .zip(&solve(MU0 - de, LAMBDA0, DRIFT))
        .map(|(a, b)| (a - b) / (2.0 * de))
        .collect();
    let r = rel(&an, &fd);
    let max: f64 = an.iter().map(|v| v.abs()).fold(0.0, f64::max);
    eprintln!("PR3b ∂x*/∂μ (drift={DRIFT:.1e}): ‖·‖∞={max:.3e}  rel-vs-FD={r:.3e}");
    assert!(max > 1e-9, "sensitivity implausibly small");
    assert!(r < 1e-5, "∂x*/∂μ with drift disagrees with FD: {r:e}");
}

/// `∂x*/∂Δ_surf` — the new drift sensitivity — matches a re-solve FD over the collider drift.
/// The drift enters only the friction term, so it reuses the same drift-consistent factored A.
#[test]
fn drift_sensitivity_matches_resolve_fd() {
    let solver = build(MU0, LAMBDA0, DRIFT);
    let x_prev = x_rest();
    let x_final = solve(MU0, LAMBDA0, DRIFT);
    let dir = Vec3::new(1.0, 0.0, 0.0);
    let an = solver.equilibrium_drift_sensitivity(&x_final, &x_prev, DT, dir);
    // FD: re-solve x* at drift ± ε along x, central difference.
    let eps = 1e-7;
    let fd: Vec<f64> = solve(MU0, LAMBDA0, DRIFT + eps)
        .iter()
        .zip(&solve(MU0, LAMBDA0, DRIFT - eps))
        .map(|(a, b)| (a - b) / (2.0 * eps))
        .collect();
    let r = rel(&an, &fd);
    let max: f64 = an.iter().map(|v| v.abs()).fold(0.0, f64::max);
    eprintln!("PR3b ∂x*/∂Δ_surf: ‖·‖∞={max:.3e}  rel-vs-FD={r:.3e}");
    assert!(max > 1e-6, "drift sensitivity implausibly small");
    assert!(r < 1e-5, "∂x*/∂Δ_surf disagrees with FD: {r:e}");
}

/// SLICE 1 (coupling-facing readout): the friction REACTION on the rigid collider
/// `F = (Σ ∇D)·react_dir` and all three of its sensitivities match a force-only
/// re-evaluation FD. `∂F/∂x*` is taken along a GENERIC direction (z-component → the
/// normal-force λ-coupling is exercised, not just the frozen-lag slip term); `∂F/∂Δ_surf`
/// and `∂F/∂height` perturb the collider drift and the plane translation with `x*` held.
#[test]
fn reaction_gradients_match_resolve_fd() {
    let solver = build(MU0, LAMBDA0, DRIFT);
    let x_prev = x_rest();
    let x_final = solve(MU0, LAMBDA0, DRIFT);
    let nd = x_final.len();
    let react = Vec3::new(1.0, 0.5, -0.3);
    let drift_dir = Vec3::new(1.0, 0.0, 0.0);
    let pose_dir = Vec3::new(0.0, 0.0, 1.0);
    let g = solver.friction_reaction_gradients(&x_final, &x_prev, DT, react, drift_dir, pose_dir);

    let force_at = |x: &[f64]| {
        build(MU0, LAMBDA0, DRIFT)
            .friction_reaction_gradients(x, &x_prev, DT, react, drift_dir, pose_dir)
            .force
    };
    // (1) ∂F/∂x* — generic direction including z (λ-coupling) vs central FD.
    let mut d = vec![0.0_f64; nd];
    for (k, e) in d.iter_mut().enumerate() {
        *e = ((k % 7) as f64 - 3.0) * 0.2 + 0.05;
    }
    let eps = 1e-8;
    let xp: Vec<f64> = x_final.iter().zip(&d).map(|(a, e)| a + eps * e).collect();
    let xm: Vec<f64> = x_final.iter().zip(&d).map(|(a, e)| a - eps * e).collect();
    let fd_dx = (force_at(&xp) - force_at(&xm)) / (2.0 * eps);
    let an_dx: f64 = g.dforce_dx.iter().zip(&d).map(|(a, b)| a * b).sum();
    // (2) ∂F/∂Δ_surf — perturb the collider drift, x* fixed.
    let eps_d = 1e-7;
    let f_dp = build(MU0, LAMBDA0, DRIFT + eps_d)
        .friction_reaction_gradients(&x_final, &x_prev, DT, react, drift_dir, pose_dir)
        .force;
    let f_dm = build(MU0, LAMBDA0, DRIFT - eps_d)
        .friction_reaction_gradients(&x_final, &x_prev, DT, react, drift_dir, pose_dir)
        .force;
    let fd_drift = (f_dp - f_dm) / (2.0 * eps_d);
    // (3) ∂F/∂height — translate the plane +z, x* fixed.
    let eps_h = 1e-7;
    let f_hp = build_dh(MU0, LAMBDA0, DRIFT, eps_h)
        .friction_reaction_gradients(&x_final, &x_prev, DT, react, drift_dir, pose_dir)
        .force;
    let f_hm = build_dh(MU0, LAMBDA0, DRIFT, -eps_h)
        .friction_reaction_gradients(&x_final, &x_prev, DT, react, drift_dir, pose_dir)
        .force;
    let fd_h = (f_hp - f_hm) / (2.0 * eps_h);
    // (4) ∂F/∂x_prev — perturb the step-start config (the friction reference
    // `x_start = x_prev + Δ_surf`), x* and drift fixed. Generic direction.
    let xpp: Vec<f64> = x_prev.iter().zip(&d).map(|(a, e)| a + eps * e).collect();
    let xpm: Vec<f64> = x_prev.iter().zip(&d).map(|(a, e)| a - eps * e).collect();
    let force_xprev = |xp: &[f64]| {
        solver
            .friction_reaction_gradients(&x_final, xp, DT, react, drift_dir, pose_dir)
            .force
    };
    let fd_dxprev = (force_xprev(&xpp) - force_xprev(&xpm)) / (2.0 * eps);
    let an_dxprev: f64 = g.dforce_dxprev.iter().zip(&d).map(|(a, b)| a * b).sum();

    let relq = |a: f64, b: f64| (a - b).abs() / b.abs().max(1e-12);
    eprintln!(
        "reaction grads vs FD:\n  ∂F/∂x*(gen) an={an_dx:.6e} fd={fd_dx:.6e} rel={:.3e}\n  \
         ∂F/∂Δ_surf an={:.6e} fd={fd_drift:.6e} rel={:.3e}\n  \
         ∂F/∂height an={:.6e} fd={fd_h:.6e} rel={:.3e}\n  \
         ∂F/∂x_prev an={an_dxprev:.6e} fd={fd_dxprev:.6e} rel={:.3e}",
        relq(an_dx, fd_dx),
        g.dforce_ddrift,
        relq(g.dforce_ddrift, fd_drift),
        g.dforce_dheight,
        relq(g.dforce_dheight, fd_h),
        relq(an_dxprev, fd_dxprev),
    );
    assert!(an_dx.abs() > 1e-3 && relq(an_dx, fd_dx) < 1e-6, "∂F/∂x*");
    assert!(
        g.dforce_ddrift.abs() > 1e-3 && relq(g.dforce_ddrift, fd_drift) < 1e-5,
        "∂F/∂Δ_surf"
    );
    assert!(
        g.dforce_dheight.abs() > 1e-3 && relq(g.dforce_dheight, fd_h) < 1e-5,
        "∂F/∂height"
    );
    assert!(
        an_dxprev.abs() > 1e-3 && relq(an_dxprev, fd_dxprev) < 1e-6,
        "∂F/∂x_prev"
    );
}

/// SLICE 1 (tape node): the friction-grip `TrajectoryStepVjp` (five parents — the
/// frictionless four plus the moving-collider drift `Δ_surf`) routes the drift cotangent
/// correctly. With `L = Σx*`, the drift parent's gradient matches a re-solve FD over the
/// collider drift, and the material parent stays FD-exact under the Woodbury grip factor.
#[test]
fn trajectory_step_vjp_grip_drift_parent_matches_fd() {
    let solver = build(MU0, LAMBDA0, DRIFT);
    let x_prev = x_rest();
    let x_final = solve(MU0, LAMBDA0, DRIFT);
    let nd = x_final.len();
    let pose_dir = Vec3::new(0.0, 0.0, 1.0);
    let drift_dir = Vec3::new(1.0, 0.0, 0.0);

    let mut tape = Tape::new();
    let xprev_var = tape.param_tensor(Tensor::from_slice(&x_prev, &[nd]));
    let vprev_var = tape.param_tensor(Tensor::from_slice(&vec![0.0_f64; nd], &[nd]));
    let mu_var = tape.param_tensor(Tensor::from_slice(&[MU0], &[1]));
    let pose_var = tape.param_tensor(Tensor::from_slice(&[EDGE], &[1]));
    let drift_var = tape.param_tensor(Tensor::from_slice(&[DRIFT], &[1]));
    let xstar = tape.push_custom(
        &[xprev_var, vprev_var, mu_var, pose_var, drift_var],
        Tensor::from_slice(&x_final, &[nd]),
        Box::new(solver.trajectory_step_vjp_grip(&x_final, &x_prev, DT, 0, pose_dir, drift_dir)),
    );
    tape.backward(xstar);
    let g_drift = tape.grad_tensor(drift_var).as_slice()[0];
    let g_mu = tape.grad_tensor(mu_var).as_slice()[0];
    let g_xprev = tape.grad_tensor(xprev_var).as_slice().to_vec();

    let sum = |x: &[f64]| -> f64 { x.iter().sum() };
    let eps = 1e-7;
    let fd_drift = (sum(&solve(MU0, LAMBDA0, DRIFT + eps))
        - sum(&solve(MU0, LAMBDA0, DRIFT - eps)))
        / (2.0 * eps);
    let eps_mu = MU0 * 1e-6;
    let fd_mu = (sum(&solve(MU0 + eps_mu, LAMBDA0, DRIFT))
        - sum(&solve(MU0 - eps_mu, LAMBDA0, DRIFT)))
        / (2.0 * eps_mu);
    // x_prev parent — the state parent now carries the friction Hessian coupling beyond
    // M/Δt² (the friction reference `x_start = x_prev + Δ_surf`). Directional FD over the
    // FREE x_prev DOFs (re-solving x*), which captures both the inertia and friction terms.
    let top = top_vertices();
    let mut d = vec![0.0_f64; nd];
    for (i, p) in block(MU0, LAMBDA0).positions().iter().enumerate() {
        if p.z.abs() >= 1e-9 {
            d[3 * i] = ((i % 5) as f64 - 2.0) * 0.3 + 0.1;
            d[3 * i + 1] = ((i % 3) as f64 - 1.0) * 0.2;
            d[3 * i + 2] = ((i % 4) as f64 - 1.5) * 0.15;
        }
    }
    let solve_xprev = |xp: &[f64]| -> Vec<f64> {
        build(MU0, LAMBDA0, DRIFT)
            .replay_step(
                &Tensor::from_slice(xp, &[nd]),
                &Tensor::from_slice(&vec![0.0_f64; nd], &[nd]),
                &Tensor::from_slice(&theta(), &[3 * top.len()]),
                DT,
            )
            .x_final
    };
    let eps_x = 1e-7;
    let xpp: Vec<f64> = x_prev.iter().zip(&d).map(|(a, e)| a + eps_x * e).collect();
    let xpm: Vec<f64> = x_prev.iter().zip(&d).map(|(a, e)| a - eps_x * e).collect();
    let fd_xprev = (sum(&solve_xprev(&xpp)) - sum(&solve_xprev(&xpm))) / (2.0 * eps_x);
    let rev_xprev: f64 = g_xprev.iter().zip(&d).map(|(a, b)| a * b).sum();

    let relq = |a: f64, b: f64| (a - b).abs() / b.abs().max(1e-12);
    eprintln!(
        "grip VJP parents vs FD:\n  drift rev={g_drift:.6e} fd={fd_drift:.6e} rel={:.3e}\n  \
         μ rev={g_mu:.6e} fd={fd_mu:.6e} rel={:.3e}\n  \
         x_prev rev={rev_xprev:.6e} fd={fd_xprev:.6e} rel={:.3e}",
        relq(g_drift, fd_drift),
        relq(g_mu, fd_mu),
        relq(rev_xprev, fd_xprev),
    );
    assert!(
        g_drift.abs() > 1e-6 && relq(g_drift, fd_drift) < 1e-5,
        "drift cotangent rev {g_drift} != FD {fd_drift}"
    );
    assert!(g_mu.abs() > 1e-9 && relq(g_mu, fd_mu) < 1e-5, "μ cotangent");
    assert!(
        rev_xprev.abs() > 1e-6 && relq(rev_xprev, fd_xprev) < 1e-5,
        "x_prev cotangent rev {rev_xprev} != FD {fd_xprev}"
    );
}
