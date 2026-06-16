//! Keystone friction leaf PR1 — the smoothed-Coulomb friction term in the FORWARD soft
//! solve. The textbook test: a soft block held against a rigid surface by a normal load
//! (here UPWARD gravity + the penalty band pressing the block into a ceiling plane, bottom
//! pinned) is dragged tangentially by a horizontal load. The Coulomb friction opposes the
//! resulting tangential slide. Subtracting the load-free settling, the LOAD-INDUCED slide
//! collapses sharply once friction is on — and monotonically as `μ` grows. This gates the
//! friction physics end-to-end through the Newton solve (residual + its friction Hessian).
//!
//! `ε_v = 0.1 m/s` (transition width `w = dt·ε_v = 1e-4 m`) is the gate's converging
//! choice; a sharper stick (smaller `ε_v`, condition number `~1/(ε_v·dt)`) needs the
//! LM-regularized solve — a follow-on. See `docs/keystone/friction_recon.md`.

// expect_used: a contact-solve test on a valid scene does not fail. cast_*: vertex-index
// (`u32`) and count (`f64`) casts on a small hand-built block are benign and exact.
#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, LoadAxis, MaterialField, Mesh,
    PenaltyRigidContact, RigidPlane, Solver, SolverConfig, Tet4, Vec3, pick_vertices_by_predicate,
};

const EDGE: f64 = 0.1;
const DT: f64 = 1.0e-3;
const KAPPA: f64 = 5.0e3;
const GRAV_UP: f64 = 10.0; // +z gravity presses the block into the ceiling

/// The friction scene: a pinned-bottom soft block pressed into a ceiling plane by upward
/// gravity, with a horizontal drag `fx_total` (N) over the top (contact) vertices. Returns
/// the assembled solver plus the step inputs (x0, v0, theta) and the geometry needed to read
/// the slide. Shared by the forward gate and the gradient-guard test.
struct FrictionScene {
    solver: CpuNewtonSolver<Tet4, HandBuiltTetMesh, PenaltyRigidContact>,
    x0: Vec<f64>,
    v0: Vec<f64>,
    theta: Vec<f64>,
    top: Vec<usize>,
    rest: Vec<Vec3>,
    n: usize,
}

fn build_scene(mu: f64, fx_total: f64) -> FrictionScene {
    let field = MaterialField::uniform(3.0e4, 1.2e5);
    let mesh = HandBuiltTetMesh::uniform_block(4, EDGE, &field);
    let n = mesh.n_vertices();
    let rest: Vec<Vec3> = mesh.positions().to_vec();
    let pinned = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    let top: Vec<usize> = (0..n)
        .filter(|&i| (rest[i].z - EDGE).abs() < 1e-9)
        .collect();

    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -EDGE); // ceiling at the block top
    let contact = PenaltyRigidContact::with_params([plane], KAPPA, 5.0e-3);
    let loaded: Vec<(u32, LoadAxis)> = top
        .iter()
        .map(|&i| (i as u32, LoadAxis::FullVector))
        .collect();
    let bc = BoundaryConditions::new(pinned, loaded);
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.gravity_z = GRAV_UP;
    cfg.friction_mu = mu;
    cfg.friction_eps_v = 0.1;
    cfg.max_newton_iter = 80;
    cfg.max_line_search_backtracks = 60;
    let solver = CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);

    let mut x0 = vec![0.0_f64; 3 * n];
    for (i, p) in rest.iter().enumerate() {
        x0[3 * i] = p.x;
        x0[3 * i + 1] = p.y;
        x0[3 * i + 2] = p.z;
    }
    let v0 = vec![0.0_f64; 3 * n];
    let fx = fx_total / top.len() as f64;
    let mut theta = vec![0.0_f64; 3 * top.len()];
    for k in 0..top.len() {
        theta[3 * k] = fx; // [fx, 0, 0] per loaded vertex
    }
    FrictionScene {
        solver,
        x0,
        v0,
        theta,
        top,
        rest,
        n,
    }
}

/// One forward step with friction `mu` and a horizontal drag `fx_total` (N) over the top
/// (contact) vertices; returns their average x-displacement.
fn top_slide(mu: f64, fx_total: f64) -> f64 {
    let s = build_scene(mu, fx_total);
    let x_final = s
        .solver
        .replay_step(
            &Tensor::from_slice(&s.x0, &[3 * s.n]),
            &Tensor::from_slice(&s.v0, &[3 * s.n]),
            &Tensor::from_slice(&s.theta, &[3 * s.top.len()]),
            DT,
        )
        .x_final;
    let sum: f64 = s.top.iter().map(|&i| x_final[3 * i] - s.rest[i].x).sum();
    sum / s.top.len() as f64
}

/// The load-INDUCED tangential slide (loaded minus the load-free settling) at friction `mu`.
fn induced_slide(mu: f64, fx: f64) -> f64 {
    top_slide(mu, fx) - top_slide(mu, 0.0)
}

#[test]
fn friction_holds_the_tangential_slide() {
    let fx = 40.0;
    let free = induced_slide(0.0, fx); // frictionless: slides freely under the drag
    let held = induced_slide(3.0, fx); // friction: the slide is held near zero
    println!("load-induced slide:  frictionless = {free:.3e} m;  μ=3 = {held:.3e} m");
    assert!(
        free > 1e-5,
        "frictionless slide must be material, got {free:.3e}"
    );
    assert!(
        held.abs() < 0.05 * free,
        "friction must hold the slide: μ=3 {held:.3e} ≪ frictionless {free:.3e}"
    );
}

/// The load-induced slide decreases monotonically as `μ` (the Coulomb cone) grows.
#[test]
fn slide_decreases_monotonically_with_friction() {
    let fx = 40.0;
    let slides: Vec<f64> = [0.0, 0.5, 1.0, 2.0, 3.0]
        .iter()
        .map(|&m| induced_slide(m, fx))
        .collect();
    println!("induced slide vs μ: {slides:?}");
    for w in slides.windows(2) {
        assert!(
            w[1] <= w[0] + 1e-9,
            "more friction must not increase the slide: {slides:?}"
        );
    }
    assert!(
        slides[4] < 0.1 * slides[0],
        "μ=3 must hold most of the frictionless slide"
    );
}

/// PR1 is forward-only: the differentiable `step` (which re-factors the IFT adjoint tangent —
/// friction-free in PR1) must PANIC with friction enabled rather than silently return a
/// gradient inconsistent with the forward solve. `replay_step` (forward-only) stays allowed.
#[test]
#[should_panic(expected = "friction gradients are not yet supported")]
fn differentiable_step_rejects_friction() {
    let mut s = build_scene(3.0, 40.0);
    let mut tape = Tape::new();
    let theta_var = tape.param_tensor(Tensor::from_slice(&s.theta, &[3 * s.top.len()]));
    let _ = s.solver.step(
        &mut tape,
        &Tensor::from_slice(&s.x0, &[3 * s.n]),
        &Tensor::from_slice(&s.v0, &[3 * s.n]),
        theta_var,
        DT,
    );
}
