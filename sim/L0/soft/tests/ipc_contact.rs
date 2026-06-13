//! IPC contact (PR1) — the C²-barrier contact model `IpcRigidContact`.
//!
//! IPC replaces the stepping-stone penalty contact with a divergent log-barrier
//! energy (Li et al. 2020): `E = κ·b(d, d̂)`, `b = −(d−d̂)²·ln(d/d̂)`. It is the
//! soft-body architecture's committed contact formulation, motivated here by the
//! keystone time-adjoint's measured penalty gradient degradation (see
//! `docs/ipc/recon.md`). Slotting in as a `ContactModel`, the solver assembles it
//! exactly like penalty (`active_pairs → gradient → residual`, `hessian → A`).
//!
//! Gates:
//! 1. the assembled `energy`/`gradient`/`hessian` match FD of each other (the
//!    barrier derivatives, validated through the `ContactModel` surface);
//! 2. **the divergent barrier CONVERGES** in the Newton solve for the keystone
//!    block under load (recon risk R2), and **never penetrates** — every vertex
//!    keeps a strictly positive gap (`sd > 0`), the structural property penalty
//!    cannot give (penalty allows finite overlap);
//! 3. the IPC `pose_residual_derivative` matches a re-solve FD (the keystone
//!    gradient path — `equilibrium_pose_sensitivity` with IPC contact).

#![allow(clippy::expect_used)]

use sim_ml_chassis::Tensor;
use sim_soft::readout::scene::pick_vertices_by_predicate;
use sim_soft::{
    BoundaryConditions, ContactModel, ContactPair, CpuNewtonSolver, HandBuiltTetMesh,
    IpcRigidContact, LoadAxis, MaterialField, Mesh, RigidPlane, Solver, SolverConfig, Tet4, Vec3,
    VertexId,
};

type IpcSolver = CpuNewtonSolver<Tet4, HandBuiltTetMesh, IpcRigidContact>;

const N_PER_EDGE: usize = 4;
const EDGE: f64 = 0.1;
const MU: f64 = 3.0e4;
const KAPPA: f64 = 1.0e4;
const D_HAT: f64 = 1.0e-3;
const DT: f64 = 1.0e-3;
const SETTLE_STEPS: usize = 400;

fn block() -> HandBuiltTetMesh {
    HandBuiltTetMesh::uniform_block(N_PER_EDGE, EDGE, &MaterialField::uniform(MU, 4.0 * MU))
}

fn bottom_pins() -> Vec<VertexId> {
    pick_vertices_by_predicate(&block(), |p| p.z.abs() < 1e-9)
}

fn top_face() -> Vec<VertexId> {
    pick_vertices_by_predicate(&block(), |p| (p.z - EDGE).abs() < 1e-9)
}

/// IPC contact with a downward ceiling plane at `height` (`sd = height − z`).
fn contact(height: f64) -> IpcRigidContact {
    IpcRigidContact::with_params(
        vec![RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -height)],
        KAPPA,
        D_HAT,
    )
}

/// Solver with the IPC plane at `height` and an upward `+ẑ` load on the top face
/// (pushes the block toward the ceiling — the barrier must hold it out).
fn solver(height: f64) -> IpcSolver {
    let loaded = top_face().iter().map(|&v| (v, LoadAxis::AxisZ)).collect();
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.max_newton_iter = 80;
    CpuNewtonSolver::new(
        Tet4,
        block(),
        contact(height),
        cfg,
        BoundaryConditions::new(bottom_pins(), loaded),
    )
}

fn x_rest() -> Vec<f64> {
    let mut x = vec![0.0_f64; 3 * block().n_vertices()];
    for (c, p) in x.chunks_exact_mut(3).zip(block().positions().iter()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    x
}

fn solve_step(s: &IpcSolver, x_prev: &[f64], v_prev: &[f64], theta: f64) -> Vec<f64> {
    let n = x_prev.len();
    s.replay_step(
        &Tensor::from_slice(x_prev, &[n]),
        &Tensor::from_slice(v_prev, &[n]),
        &Tensor::from_slice(&[theta], &[1]),
        DT,
    )
    .x_final
}

/// Minimum gap `sd = height − z` over the top-face vertices at positions `x`.
fn min_gap(height: f64, x: &[f64]) -> f64 {
    top_face()
        .iter()
        .map(|&v| {
            let z = x[3 * v as usize + 2];
            height - z
        })
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn barrier_energy_gradient_hessian_match_fd() {
    // A single vertex–plane pair at a gap inside the band (0 < sd < d̂).
    let height = 0.1 + 0.4 * D_HAT; // top rest vertex (z=0.1) sits at sd = 0.4·d̂
    let c = contact(height);
    let pair = ContactPair::Vertex {
        vertex_id: 0,
        primitive_id: 0,
    };
    // Place vertex 0 at z so that sd = height − z is engaged.
    let base = Vec3::new(0.0, 0.0, EDGE);
    let positions = |dz: f64| vec![Vec3::new(base.x, base.y, base.z + dz)];

    let eps = 1.0e-9;
    // ∂E/∂z vs gradient z-component.
    let g = c.gradient(&pair, &positions(0.0));
    let grad_z = g.contributions[0].1.z;
    let fd_grad =
        (c.energy(&pair, &positions(eps)) - c.energy(&pair, &positions(-eps))) / (2.0 * eps);
    // ∂(grad_z)/∂z vs hessian zz.
    let h = c.hessian(&pair, &positions(0.0));
    let hess_zz = h.contributions[0].2[(2, 2)];
    let fd_hess = (c.gradient(&pair, &positions(eps)).contributions[0].1.z
        - c.gradient(&pair, &positions(-eps)).contributions[0].1.z)
        / (2.0 * eps);
    eprintln!(
        "IPC barrier: grad_z={grad_z:.6e} fd={fd_grad:.6e} | hess_zz={hess_zz:.6e} fd={fd_hess:.6e}"
    );
    assert!(
        grad_z.abs() > 1e-6 && (grad_z - fd_grad).abs() / fd_grad.abs() < 1e-5,
        "gradient vs FD"
    );
    assert!(
        hess_zz.abs() > 1e-3 && (hess_zz - fd_hess).abs() / fd_hess.abs() < 1e-5,
        "hessian vs FD"
    );
    // Barrier Hessian is SPD on the contact axis (b'' > 0 ⇒ Newton-friendly).
    assert!(
        hess_zz > 0.0,
        "barrier Hessian must be positive (b'' > 0 on (0,d̂))"
    );
}

#[test]
fn ipc_settle_converges_and_never_penetrates() {
    // Plane just above the rest top face; an upward load drives the block toward
    // it. The divergent barrier must (a) converge the Newton solve every step and
    // (b) hold every vertex strictly below the plane (sd > 0) — non-penetration.
    let height = EDGE + 0.5 * D_HAT; // sd_rest = 0.5·d̂ (engaged, not penetrating)
    let theta = 4.0; // upward push on the top face
    let s = solver(height);
    let n = 3 * block().n_vertices();
    let mut x = x_rest();
    let mut v = vec![0.0_f64; n];
    let mut worst_gap = f64::INFINITY;
    for _ in 0..SETTLE_STEPS {
        let xf = solve_step(&s, &x, &v, theta); // panics if the Newton solve diverges
        for (vi, (&xf_i, &xo)) in v.iter_mut().zip(xf.iter().zip(x.iter())) {
            *vi = (xf_i - xo) / DT;
        }
        x = xf;
        worst_gap = worst_gap.min(min_gap(height, &x));
        assert!(
            x.iter().all(|c| c.is_finite()),
            "solve produced non-finite positions"
        );
    }
    let final_gap = min_gap(height, &x);
    eprintln!(
        "IPC settle: worst gap over rollout = {worst_gap:.3e} m, final gap = {final_gap:.3e} m"
    );
    // R2: converged (we got here without a panic) and the gap never went ≤ 0.
    assert!(
        worst_gap > 0.0,
        "IPC must never penetrate: worst gap {worst_gap:e} ≤ 0"
    );
    // The barrier is genuinely loaded (the block was pushed well into the band).
    assert!(
        final_gap < D_HAT,
        "contact should be engaged (gap < d̂), got {final_gap:e}"
    );
}

#[test]
fn ipc_pose_sensitivity_matches_resolve_fd() {
    // The keystone gradient path through IPC: ∂x*/∂(plane pose) vs a re-solve FD,
    // validating the IpcRigidContact::pose_residual_derivative override.
    let height = EDGE + 0.5 * D_HAT;
    let theta = 4.0;
    // Settle to a deeply-engaged near-equilibrium.
    let s = solver(height);
    let n = 3 * block().n_vertices();
    let mut x = x_rest();
    let mut v = vec![0.0_f64; n];
    for _ in 0..SETTLE_STEPS {
        let xf = solve_step(&s, &x, &v, theta);
        for (vi, (&xf_i, &xo)) in v.iter_mut().zip(xf.iter().zip(x.iter())) {
            *vi = (xf_i - xo) / DT;
        }
        x = xf;
    }
    let x_final = solve_step(&s, &x, &v, theta);
    // Analytic ∂x*/∂δ (plane translation along +ẑ), reusing the tangent at x_final.
    let analytic = s.equilibrium_pose_sensitivity(&x_final, DT, Vec3::new(0.0, 0.0, 1.0));

    // FD oracle: re-solve at height ± ε from the SAME (x, v), central-difference.
    let eps = 1.0e-7;
    let solve_at = |h: f64| -> Vec<f64> { solve_step(&solver(h), &x, &v, theta) };
    let fd: Vec<f64> = solve_at(height + eps)
        .iter()
        .zip(&solve_at(height - eps))
        .map(|(a, b)| (a - b) / (2.0 * eps))
        .collect();

    let free: Vec<usize> = (0..n).collect();
    let num: f64 = free.iter().map(|&i| (analytic[i] - fd[i]).powi(2)).sum();
    let den: f64 = free.iter().map(|&i| fd[i] * fd[i]).sum();
    let rel = (num / den.max(1e-300)).sqrt();
    let mag = analytic.iter().fold(0.0_f64, |m, &a| m.max(a.abs()));
    eprintln!("IPC pose sensitivity: ‖∂x*/∂δ‖_∞={mag:.3e} rel-to-FD={rel:.3e}");
    assert!(mag > 1e-6, "pose sensitivity implausibly small");
    assert!(rel < 1e-4, "IPC ∂x*/∂δ disagrees with re-solve FD: {rel:e}");
}
