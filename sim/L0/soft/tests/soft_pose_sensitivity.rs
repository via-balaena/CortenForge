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
//! normal (`∂n̂/∂δ = 0`); a CURVED primitive's normal also swings as it
//! translates — the `−H·u` curvature term ([`Sdf::hessian`]) that the
//! `sphere_pose_sensitivity_matches_resolve_fd` gate validates below.
//! See `docs/keystone/s3_soft_pose_sensitivity_recon.md`.
//!
//! [`Sdf::hessian`]: sim_soft::Sdf::hessian
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
    PenaltyRigidContact, PenaltyRigidContactSolver, RigidPlane, RigidTwist, Solver, SolverConfig,
    SphereSdf, Tet4, TranslatedSdf, Vec3, VertexId,
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
    let analytic = solver.equilibrium_pose_sensitivity(
        &x_final,
        None,
        DT,
        RigidTwist::translation(Vec3::new(0.0, 0.0, 1.0)),
    );

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

/// Downward unit normal of the contact plane after a rotation by `theta`
/// about the world `+ŷ` axis, `n̂(θ) = R(ŷ,θ)·(0,0,−1) = (−sinθ, 0, −cosθ)`.
fn rotated_normal(theta: f64) -> Vec3 {
    Vec3::new(-theta.sin(), 0.0, -theta.cos())
}

/// Re-solve ONE dynamic step with the contact plane rigidly *rotated* by
/// `theta` about `+ŷ` through `pivot` (on the flat plane), returning
/// `x_final`. The plane stays through `pivot`: `offset(θ) = pivot·n̂(θ)`.
/// The black-box FD oracle for the rotating-normal sensitivity — touches
/// none of the analytic twist / `A⁻¹` machinery.
fn solve_rotated_step(theta: f64, pivot: Vec3, x_prev: &[f64], v_prev: &[f64]) -> Vec<f64> {
    let n_dof = x_prev.len();
    let n = rotated_normal(theta);
    let plane = RigidPlane::new(n, pivot.dot(&n));
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

/// Active-pair count with the plane rotated by `theta` about `pivot` —
/// the rotating-normal analog of [`active_count`], for the stable-active-
/// set assertion.
fn rotated_active_count(theta: f64, pivot: Vec3, x: &[f64]) -> usize {
    let n = rotated_normal(theta);
    let plane = RigidPlane::new(n, pivot.dot(&n));
    let contact = PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT);
    let positions: Vec<Vec3> = x
        .chunks_exact(3)
        .map(|c| Vec3::new(c[0], c[1], c[2]))
        .collect();
    contact.per_pair_readout(&block(), &positions).len()
}

/// The rotating-normal gate (this leaf): the analytic forward sensitivity
/// `∂x*/∂θ` to a plane *rotation* (a tilting contact normal, the
/// `∂n̂/∂δ ≠ 0` term the S3 translation adjoint dropped) matches a
/// nonlinear re-solve central FD.
///
/// Differentiated at the flat (`θ = 0`) deeply-engaged equilibrium: a
/// downward normal `n̂₀ = (0,0,−1)` tilting about `+ŷ` through the top-face
/// center redirects the contact force, so `∂x*/∂θ` is the new direction
/// term `(dE/dsd)·(ŷ×n̂)` plus the `p·δn̂ − v·n̂` magnitude shift. The FD
/// oracle re-runs the full Newton solve with a rigidly *rotated*
/// [`RigidPlane`] at `θ ± ε` — independent of the analytic twist /
/// `equilibrium_pose_sensitivity` path, so the agreement validates the
/// rotating-normal pose-residual derivative itself.
#[test]
fn rotation_sensitivity_matches_resolve_fd() {
    let (height, x_prev, v_prev) = settle();
    let n_dof = x_prev.len();
    // Pivot: center of the engaged top face, on the flat plane (z = height).
    let pivot = Vec3::new(EDGE / 2.0, EDGE / 2.0, height);

    // The converged flat step whose x* we differentiate (same as the
    // translation gate — the rotation enters only through ∂r/∂θ).
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

    // Analytic: ∂x*/∂θ = −A⁻¹·(∂r/∂θ), the plane rotating about +ŷ through
    // `pivot` at unit rate — RigidTwist::rotation_about.
    let twist = RigidTwist::rotation_about(Vec3::new(0.0, 1.0, 0.0), pivot);
    let analytic = solver.equilibrium_pose_sensitivity(&x_final, None, DT, twist);

    // FD oracle: re-solve the rotated step at θ = ±ε, central-difference.
    let fd = |eps: f64| -> Vec<f64> {
        let xp = solve_rotated_step(eps, pivot, &x_prev, &v_prev);
        let xm = solve_rotated_step(-eps, pivot, &x_prev, &v_prev);
        xp.iter()
            .zip(xm.iter())
            .map(|(a, b)| (a - b) / (2.0 * eps))
            .collect::<Vec<f64>>()
    };
    let fd1 = fd(1.0e-6);
    let fd2 = fd(5.0e-7);

    // Active set stable across the rotation perturbation (engaged regime).
    let n_active = rotated_active_count(0.0, pivot, &x_final);
    assert!(
        n_active > 0,
        "scene must be contact-engaged, got 0 active pairs"
    );
    for (lbl, th) in [("+ε", 1.0e-6), ("-ε", -1.0e-6)] {
        let x = solve_rotated_step(th, pivot, &x_prev, &v_prev);
        assert_eq!(
            rotated_active_count(th, pivot, &x),
            n_active,
            "active set flipped at {lbl} — outside the stable-active-set regime",
        );
    }

    let rel = |a: &[f64], b: &[f64]| -> f64 {
        let num: f64 = a.iter().zip(b).map(|(x, y)| (x - y).powi(2)).sum();
        let den: f64 = b.iter().map(|y| y * y).sum();
        (num / den).sqrt()
    };
    let two_eps = rel(&fd1, &fd2);
    let analytic_vs_fd = rel(&analytic, &fd1);
    let max_dx = analytic.iter().fold(0.0_f64, |m, &x| m.max(x.abs()));

    eprintln!(
        "rotating-normal sensitivity: n_active={n_active}, ‖∂x*/∂θ‖_∞={max_dx:.4e}, \
         two-ε rel={two_eps:.3e}, analytic-vs-FD rel={analytic_vs_fd:.3e}"
    );

    // FD smoothly converged (the rotated step is differentiable here).
    assert!(
        two_eps < 1e-3,
        "FD not converged across ε (two-ε rel {two_eps:e}) — step not smooth under rotation",
    );
    // Physical sanity: the tilt redirects the force, so x* genuinely moves.
    assert!(
        max_dx > 1e-3,
        "‖∂x*/∂θ‖_∞ = {max_dx:e} implausibly small — rotation sensitivity not wired",
    );
    // The headline gate: analytic rotating-normal IFT solve == re-solve FD.
    assert!(
        analytic_vs_fd < 1e-5,
        "analytic ∂x*/∂θ disagrees with re-solve FD: rel {analytic_vs_fd:e}",
    );
}

// ── Curved-primitive pose sensitivity (the `−H·u` curvature term) ──────────

/// Sphere radius (m) and its lateral centre (the block's top-face centre).
const SPHERE_R: f64 = 0.04;

/// A `SphereSdf` posed at the block's top-face centre, raised to `center_z` — the
/// shared `TranslatedSdf` wrapper (translation leaves the Hessian unchanged, so
/// the curvature term is the sphere's own `∇²sd`).
fn sphere_at(center_z: f64) -> TranslatedSdf<SphereSdf> {
    TranslatedSdf {
        inner: SphereSdf { radius: SPHERE_R },
        offset: Vec3::new(EDGE / 2.0, EDGE / 2.0, center_z),
    }
}

/// Re-solve ONE dynamic step with the sphere centred at `center_z` (its south
/// pole pressing into the top face), returning `x_final`. The curved-primitive
/// analog of [`solve_step`]; the FD oracle moves `center_z` (a `+ẑ` translation
/// of the primitive) and touches none of the analytic machinery.
fn sphere_solve_step(center_z: f64, x_prev: &[f64], v_prev: &[f64]) -> Vec<f64> {
    let n_dof = x_prev.len();
    let contact = PenaltyRigidContact::with_params(vec![sphere_at(center_z)], KAPPA, D_HAT);
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.max_newton_iter = 80; // the curved contact's deep engagement needs more iterations
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

fn sphere_active_count(center_z: f64, x: &[f64]) -> usize {
    let contact = PenaltyRigidContact::with_params(vec![sphere_at(center_z)], KAPPA, D_HAT);
    let positions: Vec<Vec3> = x
        .chunks_exact(3)
        .map(|c| Vec3::new(c[0], c[1], c[2]))
        .collect();
    contact.per_pair_readout(&block(), &positions).len()
}

/// **The curvature gate (this leaf's headline).** `∂x*/∂(sphere +ẑ translation)`
/// from the keystone [`equilibrium_pose_sensitivity`] matches a nonlinear
/// re-solve FD — to the SAME machine-exact `< 1e-5` as the plane. This passes
/// ONLY with the `−H·u` curvature term in `pose_residual_derivative`: without it
/// the sphere's normal-swing-under-translation is dropped and the error sits at
/// ~7e-3 (measured), ~700× the plane. A flat plane (`H = 0`) is unaffected.
///
/// [`equilibrium_pose_sensitivity`]: sim_soft::CpuNewtonSolver::equilibrium_pose_sensitivity
#[test]
fn sphere_pose_sensitivity_matches_resolve_fd() {
    // Settle to engaged equilibrium: sphere south pole `PENETRATION` below the top.
    let center_z = EDGE + SPHERE_R - PENETRATION;
    let mesh = block();
    let n_dof = 3 * mesh.n_vertices();
    let mut x = vec![0.0_f64; n_dof];
    for (chunk, p) in x.chunks_exact_mut(3).zip(mesh.positions().iter()) {
        chunk[0] = p.x;
        chunk[1] = p.y;
        chunk[2] = p.z;
    }
    let mut v = vec![0.0_f64; n_dof];
    for _ in 0..SETTLE_STEPS {
        let xf = sphere_solve_step(center_z, &x, &v);
        for (vi, (&xf_i, &xo)) in v.iter_mut().zip(xf.iter().zip(x.iter())) {
            *vi = (xf_i - xo) / DT;
        }
        x = xf;
    }
    let (x_prev, v_prev) = (x.clone(), v.clone());

    let contact = PenaltyRigidContact::with_params(vec![sphere_at(center_z)], KAPPA, D_HAT);
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.max_newton_iter = 80;
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

    // Analytic: ∂x*/∂(sphere +ẑ translation) — a unit translation of the primitive.
    let analytic = solver.equilibrium_pose_sensitivity(
        &x_final,
        None,
        DT,
        RigidTwist::translation(Vec3::new(0.0, 0.0, 1.0)),
    );

    // FD oracle: re-solve with the sphere centre moved ±ε in z.
    let fd = |eps: f64| -> Vec<f64> {
        let xp = sphere_solve_step(center_z + eps, &x_prev, &v_prev);
        let xm = sphere_solve_step(center_z - eps, &x_prev, &v_prev);
        xp.iter()
            .zip(xm.iter())
            .map(|(a, b)| (a - b) / (2.0 * eps))
            .collect::<Vec<f64>>()
    };
    let fd1 = fd(1.0e-6);
    let fd2 = fd(5.0e-7);

    // Active set stable across the perturbation (engaged regime).
    let n_active = sphere_active_count(center_z, &x_final);
    assert!(
        n_active > 0,
        "scene must be contact-engaged, got 0 active pairs"
    );
    for (lbl, cz) in [("+ε", center_z + 1.0e-6), ("-ε", center_z - 1.0e-6)] {
        let x = sphere_solve_step(cz, &x_prev, &v_prev);
        assert_eq!(
            sphere_active_count(cz, &x),
            n_active,
            "active set flipped at {lbl} — outside the stable-active-set regime",
        );
    }

    let rel = |a: &[f64], b: &[f64]| -> f64 {
        let num: f64 = a.iter().zip(b).map(|(x, y)| (x - y).powi(2)).sum();
        let den: f64 = b.iter().map(|y| y * y).sum();
        (num / den).sqrt()
    };
    let two_eps = rel(&fd1, &fd2);
    let analytic_vs_fd = rel(&analytic, &fd1);
    let max_dx = analytic.iter().fold(0.0_f64, |m, &x| m.max(x.abs()));

    eprintln!(
        "sphere (curved) pose sensitivity: n_active={n_active}, ‖∂x*/∂z‖_∞={max_dx:.4e}, \
         two-ε rel={two_eps:.3e}, analytic-vs-FD rel={analytic_vs_fd:.3e}"
    );

    assert!(
        two_eps < 1e-3,
        "FD not converged across ε (two-ε rel {two_eps:e}) — step not smooth in this regime",
    );
    assert!(
        max_dx > 0.1,
        "‖∂x*/∂z‖_∞ = {max_dx:e} implausibly small for an engaged sphere contact",
    );
    // The headline: machine-exact, SAME gate as the plane — only the `−H·u`
    // curvature term makes this pass (without it the error is ~7e-3).
    assert!(
        analytic_vs_fd < 1e-5,
        "analytic ∂x*/∂(sphere z) disagrees with re-solve FD: rel {analytic_vs_fd:e} \
         — the curved-primitive `−H·u` curvature term is missing or wrong",
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
    let s = solver.equilibrium_pose_sensitivity(
        &x_final,
        None,
        DT,
        RigidTwist::translation(Vec3::new(0.0, 0.0, 1.0)),
    );
    assert_eq!(s.len(), n_dof);
    assert!(
        s.iter().all(|&x| x == 0.0),
        "NullContact pose sensitivity must be all-zero",
    );
}
