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
    clippy::cast_precision_loss
)]

use sim_ml_chassis::Tensor;
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
    let mesh = block(mu, lambda);
    let pinned = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    let top = top_vertices();
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -EDGE);
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
