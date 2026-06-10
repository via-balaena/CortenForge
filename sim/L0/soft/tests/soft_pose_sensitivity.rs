//! Keystone S3 — soft-pose sensitivity `∂x*/∂(plane pose)`, FD-validated.
//!
//! The soft solver's autograd ([`NewtonStepVjp`]) is the implicit-function-
//! theorem adjoint of one converged Newton step, but it differentiates the
//! equilibrium `x*` w.r.t. the applied *load* `θ` only — the contact-plane
//! pose is baked into the contact at construction and is not a tape input.
//! [`CpuNewtonSolver::equilibrium_pose_sensitivity`] closes that gap: it
//! returns `∂x*/∂δ = −A⁻¹·(∂r/∂δ)` for a unit rigid translation of the
//! contact primitive, reusing the SAME tangent `A` factored at `x_final`
//! that the forward Newton step converged with (and that the load adjoint
//! reuses).
//!
//! This is the keystone's deepest leaf — the implicit soft-re-equilibration
//! term that lifts the explicit coupled-step Jacobian (sim-coupling S2) to
//! the total single-step derivative.
//!
//! ## The gate is an *independent* numeric validation
//!
//! The analytic path solves the linear IFT system `A·w = −∂r/∂δ` once,
//! reusing the converged factor. The finite-difference oracle re-runs the
//! full nonlinear backward-Euler Newton solve at perturbed plane heights
//! (`height ± ε`) and central-differences `x_final` — it touches NONE of
//! the analytic machinery (`A`, `∂r/∂δ`, the factor). Because the soft solve
//! is nonlinear (`NeoHookean` elasticity + penalty contact), this agreement is
//! a genuine cross-check of the IFT adjoint, not an affine identity.
//!
//! ## Scope (documented caps)
//!
//! Differentiated in the contact-ENGAGED regime where the active set is
//! stable across the FD perturbation (asserted) — the penalty active-set
//! boundary is non-smooth (IPC the deferred cure). The plane has a constant
//! normal (`∂n̂/∂δ = 0`); curved-primitive normal curvature is a documented
//! deferral. See `docs/keystone/s3_soft_pose_sensitivity_recon.md`.
//!
//! [`NewtonStepVjp`]: sim_soft::NewtonStepVjp

#![allow(
    // Test fixtures destructure helper returns directly; `expect_used`
    // kept enabled for any future `Result`-returning helper. Mirrors the
    // sibling `penalty_compressive_block.rs` precedent.
    clippy::expect_used
)]

use sim_ml_chassis::Tensor;
use sim_soft::readout::scene::pick_vertices_by_predicate;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, MaterialField, Mesh,
    PenaltyRigidContact, PenaltyRigidContactSolver, RigidPlane, Solver, SolverConfig, Tet4, Vec3,
    VertexId,
};

// ── Scene constants (the keystone coupling regime) ────────────────────────

/// Soft block resolution (must be EVEN for `uniform_block`).
const N_PER_EDGE: usize = 4;
/// Block edge length (m).
const EDGE: f64 = 0.1;
/// First Lamé `μ` (Pa); `λ = 4μ` ⇒ `ν = 0.4`.
const MU: f64 = 3.0e4;
/// Penalty stiffness (N/m).
const KAPPA: f64 = 3.0e4;
/// Penalty contact band (m).
const D_HAT: f64 = 1.0e-2;
/// Lockstep / dynamic-soft time-step (s) — the keystone coupling value.
const DT: f64 = 1.0e-3;
/// Engaged plane penetration into the top face (m).
const PENETRATION: f64 = 1.0e-3;
/// Settling steps to reach a deeply-engaged near-equilibrium.
const SETTLE_STEPS: usize = 400;

fn field() -> MaterialField {
    MaterialField::uniform(MU, 4.0 * MU)
}

fn block() -> HandBuiltTetMesh {
    HandBuiltTetMesh::uniform_block(N_PER_EDGE, EDGE, &field())
}

fn bottom_pins() -> Vec<VertexId> {
    pick_vertices_by_predicate(&block(), |p| p.z.abs() < 1e-9)
}

/// Re-solve ONE dynamic backward-Euler step from `(x_prev, v_prev)` with the
/// downward contact plane at `height`, returning `x_final`. Rebuilds the
/// solver per call (the keystone coupling rebuilds the soft solver per step).
fn solve_step(height: f64, x_prev: &[f64], v_prev: &[f64]) -> Vec<f64> {
    let n_dof = x_prev.len();
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -height);
    let contact = PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT);
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    let solver: PenaltyRigidContactSolver<HandBuiltTetMesh> = CpuNewtonSolver::new(
        Tet4,
        block(),
        contact,
        cfg,
        BoundaryConditions::new(bottom_pins(), Vec::new()),
    );
    solver
        .replay_step(
            &Tensor::from_slice(x_prev, &[n_dof]),
            &Tensor::from_slice(v_prev, &[n_dof]),
            &Tensor::zeros(&[0]),
            DT,
        )
        .x_final
}

/// Settle to a deeply-engaged near-equilibrium with the plane penetrating the
/// top face by `PENETRATION`. Returns `(height, x_prev, v_prev)`.
fn settle() -> (f64, Vec<f64>, Vec<f64>) {
    let mesh = block();
    let n_dof = 3 * mesh.n_vertices();
    let height = EDGE - PENETRATION;
    let mut x = vec![0.0_f64; n_dof];
    for (chunk, p) in x.chunks_exact_mut(3).zip(mesh.positions().iter()) {
        chunk[0] = p.x;
        chunk[1] = p.y;
        chunk[2] = p.z;
    }
    let mut v = vec![0.0_f64; n_dof];
    for _ in 0..SETTLE_STEPS {
        let xf = solve_step(height, &x, &v);
        for (vi, (&xf_i, &xo)) in v.iter_mut().zip(xf.iter().zip(x.iter())) {
            *vi = (xf_i - xo) / DT;
        }
        x = xf;
    }
    (height, x, v)
}

/// Active-pair count at `x` with the plane at `height` — used to assert the
/// active set is stable across the FD perturbation (the engaged-regime gate).
fn active_count(height: f64, x: &[f64]) -> usize {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -height);
    let contact = PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT);
    let positions: Vec<Vec3> = x
        .chunks_exact(3)
        .map(|c| Vec3::new(c[0], c[1], c[2]))
        .collect();
    contact.per_pair_readout(&block(), &positions).len()
}

#[test]
fn pose_sensitivity_matches_resolve_fd() {
    let (height, x_prev, v_prev) = settle();
    let n_dof = x_prev.len();

    // Build the solver at the settle height and take the converged step
    // whose x* we differentiate.
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -height);
    let contact = PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT);
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    let solver: PenaltyRigidContactSolver<HandBuiltTetMesh> = CpuNewtonSolver::new(
        Tet4,
        block(),
        contact,
        cfg,
        BoundaryConditions::new(bottom_pins(), Vec::new()),
    );
    let x_final = solver
        .replay_step(
            &Tensor::from_slice(&x_prev, &[n_dof]),
            &Tensor::from_slice(&v_prev, &[n_dof]),
            &Tensor::zeros(&[0]),
            DT,
        )
        .x_final;

    // Analytic: ∂x*/∂height = −A⁻¹·(∂r/∂height), reusing A at x_final.
    // Raising the plane height = translating the primitive along +ẑ.
    let analytic = solver.equilibrium_pose_sensitivity(&x_final, DT, Vec3::new(0.0, 0.0, 1.0));

    // FD oracle: re-solve at height ± ε, central-difference. Two ε to confirm
    // smooth convergence; the analytic path is touched nowhere here.
    let fd = |eps: f64| -> Vec<f64> {
        let xp = solve_step(height + eps, &x_prev, &v_prev);
        let xm = solve_step(height - eps, &x_prev, &v_prev);
        xp.iter()
            .zip(xm.iter())
            .map(|(a, b)| (a - b) / (2.0 * eps))
            .collect::<Vec<f64>>()
    };
    let fd1 = fd(1.0e-6);
    let fd2 = fd(5.0e-7);

    // Active set must be stable across the perturbation (engaged regime).
    let n_active = active_count(height, &x_final);
    assert!(
        n_active > 0,
        "scene must be contact-engaged, got 0 active pairs"
    );
    for (lbl, h) in [("+ε", height + 1.0e-6), ("-ε", height - 1.0e-6)] {
        let x = solve_step(h, &x_prev, &v_prev);
        assert_eq!(
            active_count(h, &x),
            n_active,
            "active set flipped at {lbl} — outside the stable-active-set regime; \
             tighten PENETRATION or ε",
        );
    }

    // Relative L2 metrics.
    let rel = |a: &[f64], b: &[f64]| -> f64 {
        let num: f64 = a.iter().zip(b).map(|(x, y)| (x - y).powi(2)).sum();
        let den: f64 = b.iter().map(|y| y * y).sum();
        (num / den).sqrt()
    };
    let two_eps = rel(&fd1, &fd2);
    let analytic_vs_fd = rel(&analytic, &fd1);
    let max_dx = analytic.iter().fold(0.0_f64, |m, &x| m.max(x.abs()));

    eprintln!(
        "S3 pose sensitivity: n_active={n_active}, ‖∂x*/∂h‖_∞={max_dx:.4e}, \
         two-ε rel={two_eps:.3e}, analytic-vs-FD rel={analytic_vs_fd:.3e}"
    );

    // FD smoothly converged (two ε agree) → the step is differentiable here.
    assert!(
        two_eps < 1e-3,
        "FD not converged across ε (two-ε rel {two_eps:e}) — step not smooth in this regime",
    );
    // Physical sanity: a nonzero sensitivity (the engaged top vertices track
    // the rising plane).
    assert!(
        max_dx > 0.1,
        "‖∂x*/∂h‖_∞ = {max_dx:e} implausibly small for an engaged block — \
         sensitivity not wired",
    );
    // The headline gate: analytic IFT solve == nonlinear re-solve FD.
    assert!(
        analytic_vs_fd < 1e-5,
        "analytic ∂x*/∂height disagrees with re-solve FD: rel {analytic_vs_fd:e}",
    );
}

/// A pose-independent contact (`NullContact`, via the default
/// `pose_residual_derivative`) yields a zero sensitivity — the active set
/// contributes nothing to `∂r/∂δ`. Guards the default-empty trait method +
/// the all-zeros scatter path.
#[test]
fn null_contact_pose_sensitivity_is_zero() {
    use sim_soft::contact::NullContact;

    let mesh = block();
    let n_dof = 3 * mesh.n_vertices();
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    let solver: CpuNewtonSolver<Tet4, HandBuiltTetMesh, NullContact> = CpuNewtonSolver::new(
        Tet4,
        block(),
        NullContact,
        cfg,
        BoundaryConditions::new(bottom_pins(), Vec::new()),
    );
    // Rest positions as x_final — value is irrelevant; NullContact has no
    // active pairs, so ∂r/∂δ is identically zero regardless of position.
    let mut x_final = vec![0.0_f64; n_dof];
    for (chunk, p) in x_final.chunks_exact_mut(3).zip(mesh.positions().iter()) {
        chunk[0] = p.x;
        chunk[1] = p.y;
        chunk[2] = p.z;
    }
    let s = solver.equilibrium_pose_sensitivity(&x_final, DT, Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(s.len(), n_dof);
    assert!(
        s.iter().all(|&x| x == 0.0),
        "NullContact pose sensitivity must be all-zero",
    );
}
