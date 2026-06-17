//! Friction-coefficient leaf (sim-soft half): the forward sensitivity `∂x*/∂μ_c` of the soft
//! equilibrium w.r.t. the Coulomb friction COEFFICIENT. Friction is LINEAR in `μ_c`
//! (`∇D = μ_c·λⁿ·(t·grad2)`), so the analytic IFT path `−A⁻¹·(∇D/μ_c)` matches a friction
//! re-solve FD to the FD floor — and, unlike the material lever `∂x*/∂μ` (which needed a
//! compliant block for FD conditioning), it stays machine-exact EVEN at a STIFF block. An
//! FD-step sweep is the discriminator: a correct analytic bottoms out in a roundoff-vs-truncation
//! V (truncation arm `∝ de²`); a missing term would plateau at a fixed nonzero floor.
//!
//! Scene = the friction drift-gradient scene (load-driven + moving collider drift).

#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names
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
const FRIC_MU: f64 = 3.0;
const EPS_V: f64 = 0.1;
const FX_TOTAL: f64 = 40.0;
const DRIFT: f64 = 5.0e-4;

fn block(mu: f64, lambda: f64) -> HandBuiltTetMesh {
    HandBuiltTetMesh::uniform_block(4, EDGE, &MaterialField::uniform(mu, lambda))
}
fn x_rest(mu: f64, lambda: f64) -> Vec<f64> {
    let mesh = block(mu, lambda);
    let mut x = vec![0.0_f64; 3 * mesh.n_vertices()];
    for (i, p) in mesh.positions().iter().enumerate() {
        x[3 * i] = p.x;
        x[3 * i + 1] = p.y;
        x[3 * i + 2] = p.z;
    }
    x
}
fn top_vertices(mu: f64, lambda: f64) -> Vec<usize> {
    let mesh = block(mu, lambda);
    let rest = mesh.positions();
    (0..mesh.n_vertices())
        .filter(|&i| (rest[i].z - EDGE).abs() < 1e-9)
        .collect()
}
fn theta(mu: f64, lambda: f64) -> Vec<f64> {
    let top = top_vertices(mu, lambda);
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
    fric_mu: f64,
) -> CpuNewtonSolver<Tet4, HandBuiltTetMesh, PenaltyRigidContact> {
    let mesh = block(mu, lambda);
    let pinned = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    let top = top_vertices(mu, lambda);
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
    cfg.friction_mu = fric_mu;
    cfg.friction_eps_v = EPS_V;
    cfg.max_newton_iter = 80;
    cfg.max_line_search_backtracks = 60;
    CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc)
        .with_friction_surface_drift(Vec3::new(DRIFT, 0.0, 0.0))
}
fn solve(mu: f64, lambda: f64, fric_mu: f64) -> Vec<f64> {
    let nd = 3 * block(mu, lambda).n_vertices();
    let top = top_vertices(mu, lambda);
    build(mu, lambda, fric_mu)
        .replay_step(
            &Tensor::from_slice(&x_rest(mu, lambda), &[nd]),
            &Tensor::from_slice(&vec![0.0_f64; nd], &[nd]),
            &Tensor::from_slice(&theta(mu, lambda), &[3 * top.len()]),
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

fn check(mu: f64, lambda: f64, label: &str) {
    let solver = build(mu, lambda, FRIC_MU);
    let x_prev = x_rest(mu, lambda);
    let x_final = solve(mu, lambda, FRIC_MU);
    let an = solver.equilibrium_friction_coeff_sensitivity(&x_final, &x_prev, DT);
    let max: f64 = an.iter().map(|v| v.abs()).fold(0.0, f64::max);
    // FD-step sweep: analytic-exact ⇒ rel bottoms out (roundoff-vs-truncation V); a missing
    // term ⇒ rel plateaus at a fixed nonzero floor regardless of step.
    //
    // Gating on `min(rel)` is sound, not fragile: a systematic error ε (a dropped/wrong term)
    // lifts the WHOLE V — at the bottom truncation→0 so `rel ≈ ε`, hence `min(rel)` is a LOWER
    // BOUND on ε and any ε > 1e-6 fails the assert. The only way the min could dip below 1e-6
    // despite a real bug is an FD truncation that coincidentally cancels ε at one `de` across
    // the entire DOF VECTOR at once — a measure-zero coincidence, not a failure mode. The sweep
    // (vs a hardcoded step) auto-finds the V-bottom, which sits at a slightly different `de` for
    // the stiff vs compliant block.
    let mut best = f64::INFINITY;
    for k in 2..=9 {
        let de = FRIC_MU * 10f64.powi(-k);
        let fd: Vec<f64> = solve(mu, lambda, FRIC_MU + de)
            .iter()
            .zip(&solve(mu, lambda, FRIC_MU - de))
            .map(|(a, b)| (a - b) / (2.0 * de))
            .collect();
        let r = rel(&an, &fd);
        eprintln!("  [{label}] de=μ_c·1e-{k}: rel={r:.3e}");
        best = best.min(r);
    }
    eprintln!("∂x*/∂μ_c [{label}] μ_mat={mu:.1e}: ‖·‖∞={max:.3e}  best-rel-vs-FD={best:.3e}");
    assert!(
        max > 1e-9,
        "[{label}] sensitivity implausibly small — no lever"
    );
    assert!(
        best < 1e-6,
        "[{label}] ∂x*/∂μ_c disagrees with FD even at best step: {best:e}"
    );
}

#[test]
fn friction_coeff_sensitivity_matches_fd_compliant() {
    check(3.0e3, 1.2e4, "compliant");
}

#[test]
fn friction_coeff_sensitivity_matches_fd_stiff() {
    check(3.0e4, 1.2e5, "stiff");
}
