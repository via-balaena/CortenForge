//! `CpuNewtonSolver::step` smoke test — Stage-1 θ convergence.
//!
//! Canonical 1-tet scene (spec §2): decimeter-edge NH body, `v_0, v_1, v_2`
//! pinned Dirichlet, `v_3` free. Stage-1 θ per scope §2 R-6: length-1
//! `Tensor<f64>` = traction magnitude along `+ẑ` applied to `v_3`.
//!
//! Checks:
//!
//! - Newton converges within `config.max_newton_iter = 10`.
//! - Final free-DOF residual norm below `config.tol = 1e-10`.
//! - `v_3`'s `z`-DOF moves in the force direction (`+ẑ`).
//! - Displacement magnitude sits in the dimensional-analysis band for
//!   the condensed stiffness `A_33` at rest.
//!
//! Dimensional analysis at rest, linear estimate:
//!
//! ```text
//! A_33 = (m/Δt²)·I + (V/L²) · diag(μ, μ, 2μ+λ)
//!      ≈ 430·I + diag(1.67e3, 1.67e3, 1e4)  [kg·m/s² per m]
//!      = diag(2.1e3, 2.1e3, 1.04e4) N/m
//! ```
//!
//! θ = 10 N on `+ẑ` gives `δz ≈ 10/1.04e4 ≈ 9.6e-4 m` (~1% strain). This
//! is below scope §2's "5-15% strain" estimate — the scope's scaling
//! `K ~ μV/L²` undercounts the constrained-modulus coefficient
//! `(2μ + λ) = 6μ` for our `λ = 4μ`. Drift noted as scope §13 `SD-3`.

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::{
    CpuNewtonSolver, NullContact, SkeletonSolver, SoftScene, Solver, SolverConfig, Tet4,
};

#[test]
fn stage_1_traction_converges() {
    let cfg = SolverConfig::skeleton();
    let (mesh, bc, initial) = SoftScene::one_tet_cube();

    let mut solver: SkeletonSolver = CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);

    // Stage-1 θ: length-1 tensor = magnitude along +ẑ on v_3. Registered
    // as a tape parameter so `Solver::step` can attach `NewtonStepVjp`
    // with `theta_var` as parent (step 5 / scope §9 step 5).
    let mut tape = Tape::new();
    let theta_var = tape.param_tensor(Tensor::from_slice(&[10.0], &[1]));

    let step = solver.step(
        &mut tape,
        &initial.x_prev,
        &initial.v_prev,
        theta_var,
        cfg.dt,
    );

    // Convergence within Newton budget. Spec §3 R-1 predicts 3-5 iter
    // from rest at Stage-1 θ; loop exits via convergence return, so
    // iter_count is always < max_newton_iter (never reaches equality).
    assert!(
        step.iter_count < cfg.max_newton_iter,
        "Newton did not converge within budget: {} >= {}",
        step.iter_count,
        cfg.max_newton_iter,
    );
    assert!(
        step.final_residual_norm < cfg.tol,
        "Final residual norm {} not below tol {}",
        step.final_residual_norm,
        cfg.tol,
    );

    // v_3 DOFs live at indices 9 (x), 10 (y), 11 (z).
    let x_prev = initial.x_prev.as_slice();
    let dx = step.x_final[9] - x_prev[9];
    let dy = step.x_final[10] - x_prev[10];
    let dz = step.x_final[11] - x_prev[11];

    // Force is along +ẑ — z-displacement must be positive.
    assert!(
        dz > 0.0,
        "v_3 must move in +ẑ under +z traction, got dz = {dz}"
    );

    // At F=I the NH tangent is diagonal (isotropy — no (i, i+3j) off-
    // diagonal coupling unless i=j=2). Condensed A_33 inherits that
    // structure, so the Newton step is pure +ẑ at the linear level;
    // NH's geometric nonlinearity at ~1% strain induces at most
    // O(strain² · dz) ~ 1e-7 m on dx/dy, well below the 1e-5 m bound.
    assert!(
        dx.abs() < 1e-5,
        "dx = {dx} — expected near-zero at F=I isotropic tangent (< 1e-5 m)"
    );
    assert!(
        dy.abs() < 1e-5,
        "dy = {dy} — expected near-zero at F=I isotropic tangent (< 1e-5 m)"
    );

    // Linear estimate δz ≈ 9.6e-4 m at θ = 10 N (see module docstring).
    // Widen to (0.5 mm, 1.5 mm) for NH nonlinearity margin.
    assert!(
        (5.0e-4..=1.5e-3).contains(&dz),
        "dz = {dz} outside dimensional-analysis band [5e-4, 1.5e-3] m at θ = 10 N"
    );

    // Sanity on return-shape: x_final has 12 entries (vertex-major, 4 nodes).
    assert_eq!(
        step.x_final.len(),
        12,
        "NewtonStep.x_final should be 12 entries (4 vertices × 3 DOFs)"
    );

    // Pinned vertices (v_0, v_1, v_2) must not have moved.
    for (i, (&got, &expected)) in step.x_final.iter().zip(x_prev.iter()).take(9).enumerate() {
        assert!(
            (got - expected).abs() < 1e-14,
            "Dirichlet DOF {i} moved: x_final[{i}] = {got} vs x_prev[{i}] = {expected}",
        );
    }
}
