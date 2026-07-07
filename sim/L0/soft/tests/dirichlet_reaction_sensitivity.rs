//! Rung 6d (PR1) — FD gate for the constrained (Dirichlet) **reaction sensitivity**
//! `∂(reaction)/∂(pinned target)`, the forward JVP of the bonded-disc adjoint.
//!
//! Scene = the `BondedSandwich` soft core: a Neo-Hookean block pinned on BOTH z-faces
//! (the two "endplates"), free interior. We perturb the UPPER pinned face's Dirichlet
//! targets and compare the analytic
//! [`CpuNewtonSolver::equilibrium_dirichlet_reaction_sensitivity`] against an
//! INDEPENDENT re-solve FD of the reaction readout
//! ([`CpuNewtonSolver::nodal_reaction_forces`]) — the FD re-runs the real Newton solve
//! at each perturbed target, so it never touches the analytic factor/Schur machinery.
//! Evaluated at a compressed operating point so the nonlinear tangent is exercised.

#![allow(
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::float_cmp,
    clippy::similar_names
)]

use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh, NullContact,
    Solver, SolverConfig, Tet4, VertexId, pick_vertices_by_predicate,
};

const N: usize = 2;
const EDGE: f64 = 0.1;
const MU: f64 = 3.0e4;
const LAMBDA: f64 = 1.2e5;
const STATIC_DT: f64 = 1.0e3; // quasi-static bond regime
const DELTA: f64 = 0.05 * EDGE; // 5% axial compression operating point

type NhSolver = CpuNewtonSolver<Tet4, HandBuiltTetMesh, NullContact>;

fn block() -> HandBuiltTetMesh {
    HandBuiltTetMesh::uniform_block(N, EDGE, &MaterialField::uniform(MU, LAMBDA))
}
fn lower_verts() -> Vec<VertexId> {
    pick_vertices_by_predicate(&block(), |p| p.z.abs() < 1e-9)
}
fn upper_verts() -> Vec<VertexId> {
    pick_vertices_by_predicate(&block(), |p| (p.z - EDGE).abs() < 1e-9)
}
fn n_dof() -> usize {
    3 * block().n_vertices()
}

/// Both z-faces pinned (Dirichlet), no loaded vertices — the two-endplate bond.
fn build() -> NhSolver {
    let mut pinned = lower_verts();
    pinned.extend(upper_verts());
    let bc = BoundaryConditions::new(pinned, Vec::new());
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = 50;
    CpuNewtonSolver::new(Tet4, block(), NullContact, cfg, bc)
}

fn x_rest() -> Vec<f64> {
    let mut x = vec![0.0_f64; n_dof()];
    for (c, p) in x.chunks_exact_mut(3).zip(block().positions().iter()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    x
}

/// Full-DOF perturbation direction on the UPPER pinned face only (the moving endplate):
/// a net downward push plus a lateral/tilt pattern so the direction is rich (not pure
/// axial). Zero on the free interior and the lower face (the contract for `dx_pinned`).
fn direction() -> Vec<f64> {
    let mut d = vec![0.0_f64; n_dof()];
    for (k, &v) in upper_verts().iter().enumerate() {
        let i = v as usize;
        d[3 * i] = 0.4 * ((k % 3) as f64 - 1.0); // x tilt
        d[3 * i + 1] = 0.3 * ((k % 2) as f64 - 0.5); // y tilt
        d[3 * i + 2] = -1.0; // net compression
    }
    d
}

/// The operating-point `x_prev`: free interior warm-started at rest, lower face at rest,
/// upper face compressed straight down by `DELTA` — then displaced by `alpha·dir`.
fn x_prev_at(alpha: f64, dir: &[f64]) -> Vec<f64> {
    let mut x = x_rest();
    for &v in &upper_verts() {
        x[3 * v as usize + 2] -= DELTA; // operating-point compression
    }
    for (xi, di) in x.iter_mut().zip(dir) {
        *xi += alpha * di;
    }
    x
}

fn solve(s: &NhSolver, x_prev: &[f64]) -> Vec<f64> {
    let n = x_prev.len();
    s.replay_step(
        &Tensor::from_slice(x_prev, &[n]),
        &Tensor::zeros(&[n]),
        &Tensor::zeros(&[0]),
        STATIC_DT,
    )
    .x_final
}

/// Reaction `−f_int` at a re-solved perturbed target (the independent FD oracle).
fn reaction(s: &NhSolver, x_prev: &[f64]) -> Vec<f64> {
    let x_final = solve(s, x_prev);
    s.nodal_reaction_forces(&x_final, x_prev, STATIC_DT)
}

/// Relative L2 error over the pinned (endplate) DOFs — where the reaction lives.
fn rel_pinned(a: &[f64], b: &[f64]) -> f64 {
    let pinned: Vec<usize> = lower_verts()
        .iter()
        .chain(&upper_verts())
        .flat_map(|&v| {
            let i = v as usize;
            [3 * i, 3 * i + 1, 3 * i + 2]
        })
        .collect();
    let num: f64 = pinned.iter().map(|&i| (a[i] - b[i]).powi(2)).sum();
    let den: f64 = pinned.iter().map(|&i| b[i] * b[i]).sum();
    (num / den.max(1e-300)).sqrt()
}

#[test]
fn dirichlet_reaction_jvp_matches_resolve_fd() {
    let s = build();
    let dir = direction();

    // Analytic sensitivity at the compressed operating point.
    let x_final0 = solve(&s, &x_prev_at(0.0, &dir));
    let an = s.equilibrium_dirichlet_reaction_sensitivity(&x_final0, STATIC_DT, &dir);

    // Liveness: the sensitivity must be substantial (a stiff endplate reaction), so the
    // FD-match below is not a vacuous 0 ≈ 0. Compression stiffness is O(μ·EDGE) ≫ 1.
    let live = an.iter().map(|x| x.abs()).fold(0.0, f64::max);
    eprintln!("  max |dR| = {live:.3e}");
    assert!(
        live > 1.0,
        "reaction sensitivity is degenerate (max |dR| = {live:.3e})"
    );

    // Independent re-solve FD, eps-swept (should be flat — a smooth bilateral bond).
    let mut best = f64::INFINITY;
    for e in [1e-4, 1e-5, 1e-6, 1e-7] {
        let eps = e * EDGE;
        let rp = reaction(&s, &x_prev_at(eps, &dir));
        let rm = reaction(&s, &x_prev_at(-eps, &dir));
        let fd: Vec<f64> = rp
            .iter()
            .zip(&rm)
            .map(|(a, b)| (a - b) / (2.0 * eps))
            .collect();
        let rel = rel_pinned(&an, &fd);
        eprintln!("  eps={e:.1e}·EDGE  rel(analytic, FD) = {rel:.3e}");
        best = best.min(rel);
    }
    assert!(
        best < 1e-6,
        "Dirichlet reaction JVP must match re-solve FD; best rel = {best:.3e}"
    );

    // Conservation of the reaction derivative: the internal force is self-equilibrated
    // (Σ f_int ≡ 0), so Σ dR = 0 per axis, for ANY perturbation (Newton's third law in
    // the derivative). Independent of the FD above.
    for axis in 0..3 {
        let sum: f64 = (0..n_dof() / 3).map(|v| an[3 * v + axis]).sum();
        let scale: f64 = an.iter().map(|x| x.abs()).fold(0.0, f64::max).max(1e-30);
        eprintln!("  Σ dR axis {axis} / scale = {:.2e}", sum / scale);
        assert!(
            (sum / scale).abs() < 1e-9,
            "reaction-derivative not conserved on axis {axis}: {:.2e}",
            sum / scale
        );
    }
}
