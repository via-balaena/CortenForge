//! Tet10 ladder rung 7 — the Dirichlet-reaction sensitivity on the quadratic
//! element (the bonded-endplate reaction adjoint's forward JVP).
//!
//! `equilibrium_dirichlet_reaction_sensitivity` gives `∂(reaction)/∂(pinned
//! target)` of a converged constrained equilibrium — the Schur complement of the
//! stiffness, `dR_p/dx_p = K_pf·A_ff⁻¹·K_fp − K_pp`, applied to a motion of the
//! Dirichlet targets. It reuses `internal_force_tangent_matvec` (`K·v`) for the
//! two matvecs and `factor_at_position` for `A_ff⁻¹`.
//!
//! Both of those are already per-Gauss-point for Tet10 (rung 4), so lifting the
//! rung-7 `N == 4` diff-guard is all this channel needs — no RHS change. This gate
//! confirms it end-to-end: the analytic JVP must match a re-solve central FD of
//! `nodal_reaction_forces` on the pinned DOFs (the oracle re-runs the full
//! nonlinear Newton at the perturbed targets, touching no `A⁻¹`).

#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]

use sim_ml_chassis::Tensor;
use sim_soft::element::Tet10;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet10NHSolver, HandBuiltTetMesh, MaterialField, Mesh,
    NullContact, Solver, SolverConfig, Tet10Mesh, VertexId,
};

const N: usize = 2;
const EDGE: f64 = 0.1;
const MU: f64 = 3.0e4;
const LAMBDA: f64 = 1.2e5;
const STATIC_DT: f64 = 1.0e3; // quasi-static bond (M/Δt² ≈ 0, reaction = −f_int)
const DELTA: f64 = 0.05 * EDGE; // axial compression of the upper face

/// A `BondedSandwich` Tet10 solver: pin BOTH z-face corner sets, no external
/// load. Returns the solver, its rest DOFs, and the upper-face corner ids (the
/// face we perturb).
fn build() -> (CpuTet10NHSolver<Tet10Mesh>, Vec<f64>, Vec<VertexId>) {
    let cube = HandBuiltTetMesh::uniform_block(N, EDGE, &MaterialField::uniform(MU, LAMBDA));
    let pos = cube.positions().to_vec();
    let n_corners = cube.n_vertices();
    let lower: Vec<VertexId> = (0..n_corners as VertexId)
        .filter(|&v| pos[v as usize].z.abs() < 1e-9)
        .collect();
    let upper: Vec<VertexId> = (0..n_corners as VertexId)
        .filter(|&v| (pos[v as usize].z - EDGE).abs() < 1e-9)
        .collect();
    assert!(!lower.is_empty() && !upper.is_empty());
    let mut pinned = lower;
    pinned.extend(upper.iter().copied());

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
        loaded_vertices: Vec::new(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = 80;
    let solver = CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);
    (solver, rest, upper)
}

fn rel_on(idx: &[usize], a: &[f64], b: &[f64]) -> f64 {
    let num: f64 = idx.iter().map(|&i| (a[i] - b[i]).powi(2)).sum();
    let den: f64 = idx.iter().map(|&i| b[i] * b[i]).sum();
    (num / den.max(1e-300)).sqrt()
}

#[test]
fn tet10_dirichlet_reaction_jvp_matches_resolve_fd() {
    let (s, x_rest, upper) = build();
    let n = x_rest.len();
    let v0 = vec![0.0_f64; n];
    let theta = Tensor::from_slice(&[], &[0]);

    // Operating point: compress the upper face by DELTA in −z.
    let mut x_op = x_rest;
    for &v in &upper {
        x_op[3 * v as usize + 2] -= DELTA;
    }
    let solve = |xp: &[f64]| -> Vec<f64> {
        s.replay_step(
            &Tensor::from_slice(xp, &[n]),
            &Tensor::from_slice(&v0, &[n]),
            &theta,
            STATIC_DT,
        )
        .x_final
    };
    let x1 = solve(&x_op);

    // Perturb the upper (pinned) face targets only; the direction is zero on the
    // free DOFs (the JVP contract — a motion of the Dirichlet targets).
    let mut dir = vec![0.0_f64; n];
    for (k, &v) in upper.iter().enumerate() {
        dir[3 * v as usize] = 0.2 * (((k % 3) as f64) - 1.0);
        dir[3 * v as usize + 2] = -0.5;
    }
    let an = s.equilibrium_dirichlet_reaction_sensitivity(&x1, STATIC_DT, &dir);

    // Metric on the pinned (reaction-bearing) DOFs: the upper face we perturbed
    // plus its opposing lower face both carry reactions.
    let pinned_dofs: Vec<usize> = {
        // Pinned DOFs = the perturbed-direction support's face DOFs; but the
        // reaction lives on ALL pinned DOFs. Reconstruct: a DOF is pinned iff its
        // reaction sensitivity (analytic OR FD) is non-negligible on the faces —
        // simplest is to take both z-faces. Rebuild the pinned corner set.
        let pos = HandBuiltTetMesh::uniform_block(N, EDGE, &MaterialField::uniform(MU, LAMBDA))
            .positions()
            .to_vec();
        let n_corners = pos.len();
        (0..n_corners)
            .filter(|&v| pos[v].z.abs() < 1e-9 || (pos[v].z - EDGE).abs() < 1e-9)
            .flat_map(|v| [3 * v, 3 * v + 1, 3 * v + 2])
            .collect()
    };

    // Re-solve central FD of nodal_reaction_forces, eps-swept.
    let reaction = |xp: &[f64]| -> Vec<f64> {
        let xr = solve(xp);
        s.nodal_reaction_forces(&xr, xp, STATIC_DT)
    };
    let mut best = f64::INFINITY;
    for &e in &[1e-4, 1e-5, 1e-6, 1e-7] {
        let eps = e * EDGE;
        let xp: Vec<f64> = x_op.iter().zip(&dir).map(|(a, d)| a + eps * d).collect();
        let xm: Vec<f64> = x_op.iter().zip(&dir).map(|(a, d)| a - eps * d).collect();
        let rp = reaction(&xp);
        let rm = reaction(&xm);
        let fd: Vec<f64> = rp
            .iter()
            .zip(&rm)
            .map(|(a, b)| (a - b) / (2.0 * eps))
            .collect();
        best = best.min(rel_on(&pinned_dofs, &an, &fd));
    }
    let mag = pinned_dofs.iter().map(|&i| an[i].abs()).fold(0.0, f64::max);
    eprintln!(
        "Tet10 reaction JVP: n_pinned_dof={} ‖·‖_∞={mag:.3e} best_rel={best:.3e}",
        pinned_dofs.len()
    );
    assert!(mag > 1.0, "reaction sensitivity implausibly small");
    assert!(
        best < 1e-6,
        "Tet10 reaction JVP disagrees with re-solve FD: best_rel={best:e}"
    );
}
