//! Keystone friction leaf PR2 — friction DIFFERENTIABILITY. The forward sensitivities now
//! carry the friction adjoint exactly: `factor_at_position` factors the frozen-lag symmetric
//! tangent `A_sym` and wraps it in a Woodbury correction for the asymmetric `∂λⁿ/∂x` term the
//! lag drops (friction is non-conservative, so that part is non-symmetric). This gates
//! `∂x*/∂μ` (material) against a friction re-solve FD — MACHINE-EXACT, and decisively better
//! than the frozen-lag-only adjoint (which S0 pinned at a systematic ~2e-3). The reverse-mode
//! tape VJPs stay friction-guarded until the coupling leaf (PR3). See `friction_recon.md`.

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
const MU0: f64 = 3.0e4; // NeoHookean μ (shear modulus) — NOT the friction coefficient
const LAMBDA0: f64 = 1.2e5;
const FRIC_MU: f64 = 3.0;
const FX_TOTAL: f64 = 40.0;

fn block(mu: f64, lambda: f64) -> HandBuiltTetMesh {
    HandBuiltTetMesh::uniform_block(4, EDGE, &MaterialField::uniform(mu, lambda))
}

fn n() -> usize {
    block(MU0, LAMBDA0).n_vertices()
}

/// Flattened rest positions = the step-start `xᵗ` (= `x_prev`).
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

fn build(mu: f64, lambda: f64) -> CpuNewtonSolver<Tet4, HandBuiltTetMesh, PenaltyRigidContact> {
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
    cfg.friction_eps_v = 0.1;
    cfg.max_newton_iter = 80;
    cfg.max_line_search_backtracks = 60;
    CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc)
}

/// `x_final` of one forward friction step at material `(mu, lambda)`.
fn solve(mu: f64, lambda: f64) -> Vec<f64> {
    let nd = 3 * n();
    let top = top_vertices();
    build(mu, lambda)
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

/// `∂x*/∂μ` with friction active is MACHINE-EXACT against a friction re-solve FD — the
/// Woodbury `∂λⁿ/∂x` correction closes the frozen-lag gap S0 measured (~2e-3 → FD floor).
#[test]
fn material_sensitivity_with_friction_matches_resolve_fd() {
    let solver = build(MU0, LAMBDA0);
    let x_final = solve(MU0, LAMBDA0);
    let x_prev = x_rest();

    // Friction-exact forward sensitivity (Some(x_prev) ⇒ the Woodbury adjoint). The
    // frozen-lag-only adjoint (`None` under friction) is the guarded reverse-mode path; S0
    // already pinned it at a systematic ~2e-3, so the gate just asserts machine-exactness.
    let analytic = solver.equilibrium_material_sensitivity(&x_final, Some(&x_prev), DT, 0);

    let de = MU0 * 1.0e-5;
    let fd: Vec<f64> = solve(MU0 + de, LAMBDA0)
        .iter()
        .zip(&solve(MU0 - de, LAMBDA0))
        .map(|(a, b)| (a - b) / (2.0 * de))
        .collect();
    let mag = fd.iter().map(|y| y * y).sum::<f64>().sqrt();

    let rel_exact = rel(&analytic, &fd);
    eprintln!("PR2 friction ∂x*/∂μ: ‖fd‖={mag:.3e}  Woodbury-exact rel={rel_exact:.3e}");
    assert!(mag > 1e-9, "sensitivity implausibly small");
    assert!(
        rel_exact < 1e-6,
        "friction-exact ∂x*/∂μ must match re-solve FD: {rel_exact:e}"
    );
}

// SCOPE NOTE (why there is no pose-friction gate yet): the adjoint tangent `A` is the SAME for
// the material and pose sensitivities (`factor_at_position`), and the material gate above proves
// it is friction-exact (the Woodbury `∂λⁿ/∂x` correction). The POSE sensitivity additionally
// needs `∂r/∂pose` to carry the friction force's explicit pose dependence (a moving plane
// changes `λⁿ` ⇒ changes the friction force) — a term `assemble_pose_residual_grad` does not yet
// include (under friction the pose sensitivity lands ~4e-3, the missing term, vs machine-exact
// for material). That pose-residual friction derivative belongs with the coupling leaf (PR3),
// where `equilibrium_pose_sensitivity` is consumed. Tracked in `docs/keystone/friction_recon.md`.
