//! Tet10 ladder rung 7 — the load-θ adjoint (`NewtonStepVjp`) on the quadratic
//! element.
//!
//! `Solver::step` pushes `NewtonStepVjp` onto the tape: the load adjoint
//! `∂x*/∂θ = −A⁻¹·(∂r/∂θ)`, reusing the tangent factored at `x_final`
//! (`factor_at_position`). Lifting the rung-7 `N == 4` diff-guard unlocks this
//! channel for Tet10 alongside the material / reaction / state channels, so it
//! gets its own FD gate here.
//!
//! The RHS `∂r/∂θ` is `−e` on each loaded vertex's loaded free DOF — the external
//! load is a NODAL point force (`assemble_external_force` writes `θ` directly onto
//! the vertex DOF, no shape-function integration), so this RHS is element-order
//! INDEPENDENT and needs no widen. This gate confirms that empirically: the IFT
//! analytic `∂L/∂θ` (`L = x_final[loss_dof]`) matches a central FD of a re-solve
//! to ≥5 digits, and two runs are bit-deterministic.

#![allow(
    clippy::expect_used,
    clippy::similar_names,
    clippy::cast_possible_truncation
)]

use sim_ml_chassis::{Tape, Tensor, Var};
use sim_soft::element::Tet10;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet10NHSolver, HandBuiltTetMesh, IndexOp, LoadAxis,
    MaterialField, Mesh, NullContact, Solver, SolverConfig, Tet10Mesh, VertexId,
};

const N: usize = 2;
const EDGE: f64 = 0.1;
const MU: f64 = 3.0e4;
const LAMBDA: f64 = 1.2e5;
const DT: f64 = 1.0; // static
const THETA_0: f64 = 8.0; // +ẑ broadcast load on the top face
const H: f64 = 1.5e-8; // ≈ √ε for f64 (central-FD step)

/// Build a Tet10 block (pin bottom-face corners, AxisZ-load top-face corners),
/// returning the solver, its rest DOFs, the loaded-DOF count (for the θ RHS
/// shape), and a loss DOF = a loaded top corner's z-index (a free DOF that
/// responds to θ).
fn build() -> (CpuTet10NHSolver<Tet10Mesh>, Vec<f64>, usize) {
    let cube = HandBuiltTetMesh::uniform_block(N, EDGE, &MaterialField::uniform(MU, LAMBDA));
    let pos = cube.positions().to_vec();
    let n_corners = cube.n_vertices();
    let pinned: Vec<VertexId> = (0..n_corners as VertexId)
        .filter(|&v| pos[v as usize].z.abs() < 1e-9)
        .collect();
    let top: Vec<VertexId> = (0..n_corners as VertexId)
        .filter(|&v| (pos[v as usize].z - EDGE).abs() < 1e-9)
        .collect();
    assert!(!pinned.is_empty() && !top.is_empty());
    let loss_dof = 3 * top[0] as usize + 2;

    let mesh = Tet10Mesh::from_tet4(&cube);
    let p = mesh.positions();
    let mut rest = vec![0.0_f64; 3 * p.len()];
    for (v, pt) in p.iter().enumerate() {
        rest[3 * v] = pt.x;
        rest[3 * v + 1] = pt.y;
        rest[3 * v + 2] = pt.z;
    }
    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: top.into_iter().map(|v| (v, LoadAxis::AxisZ)).collect(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.max_newton_iter = 80;
    let solver = CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);
    (solver, rest, loss_dof)
}

/// One forward + backward: the IFT analytic `∂(x_final[loss_dof])/∂θ`, plus
/// `x_final` for the determinism check.
fn forward_backward(theta_val: f64) -> (f64, Vec<f64>) {
    let (mut solver, rest, loss_dof) = build();
    let n = rest.len();
    let x_prev = Tensor::from_slice(&rest, &[n]);
    let v_prev = Tensor::zeros(&[n]);

    let mut tape = Tape::new();
    let theta_var: Var = tape.param_tensor(Tensor::from_slice(&[theta_val], &[1]));
    let step = solver.step(&mut tape, &x_prev, &v_prev, theta_var, DT);
    let x_final_var = step
        .x_final_var
        .expect("Solver::step must populate x_final_var");

    let l_scalar = tape.value_tensor(x_final_var).as_slice()[loss_dof];
    let l_var = tape.push_custom(
        &[x_final_var],
        Tensor::from_slice(&[l_scalar], &[1]),
        Box::new(IndexOp::new(loss_dof, n)),
    );
    tape.backward(l_var);
    let grad_theta = tape.grad_tensor(theta_var).as_slice()[0];
    (grad_theta, step.x_final)
}

/// Primal-only forward: `x_final[loss_dof]` at load `theta_val` (no tape).
fn forward_only(theta_val: f64) -> f64 {
    let (solver, rest, loss_dof) = build();
    let n = rest.len();
    let step = solver.replay_step(
        &Tensor::from_slice(&rest, &[n]),
        &Tensor::zeros(&[n]),
        &Tensor::from_slice(&[theta_val], &[1]),
        DT,
    );
    step.x_final[loss_dof]
}

#[test]
fn tet10_load_theta_gradcheck_central_fd() {
    const REL_ERR_BOUND: f64 = 1e-5;
    let (analytic, _x_final) = forward_backward(THETA_0);
    let fd = (forward_only(THETA_0 + H) - forward_only(THETA_0 - H)) / (2.0 * H);
    let rel = (analytic - fd).abs() / fd.abs().max(1e-12);
    eprintln!("Tet10 load-θ gradcheck: analytic={analytic:.6e} FD={fd:.6e} rel={rel:.3e}");
    assert!(analytic.abs() > 1e-9, "∂x*/∂θ implausibly small");
    assert!(
        rel <= REL_ERR_BOUND,
        "Tet10 load-θ IFT adjoint disagrees with central FD: rel={rel:e}"
    );
}

#[test]
fn tet10_load_theta_determinism() {
    let (grad_a, x_a) = forward_backward(THETA_0);
    let (grad_b, x_b) = forward_backward(THETA_0);
    assert_eq!(grad_a.to_bits(), grad_b.to_bits(), "grad_θ not bit-equal");
    assert_eq!(x_a.len(), x_b.len());
    for (i, (&a, &b)) in x_a.iter().zip(&x_b).enumerate() {
        assert_eq!(a.to_bits(), b.to_bits(), "x_final[{i}] not bit-equal");
    }
}
