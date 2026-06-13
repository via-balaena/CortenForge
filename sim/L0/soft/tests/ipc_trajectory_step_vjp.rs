//! IPC localization probe (recon §9) — `TrajectoryStepVjp` per-step machine-exact
//! gate with `IpcRigidContact` (the C²-barrier) in the tangent.
//!
//! The penalty sibling `trajectory_step_vjp.rs` validated the fused four-parent VJP
//! (`[x_prev, v_prev, param, pose]`) only with PENALTY contact, whose Hessian
//! curvature is the constant `κ`. PR1's IPC pose check used the FORWARD
//! `equilibrium_pose_sensitivity`, not the reverse `TrajectoryStepVjp` pose
//! cotangent. This gate re-runs the four-parent FD cross-check with `IpcRigidContact`
//! in the tangent, where the per-pair curvature is the VARYING `κ·b''(sd)` — the
//! single-step factor whose multi-step composition leaves the open residual
//! (`docs/ipc/recon.md` §9). If every parent matches FD here, the residual is NOT in
//! the sim-soft fused VJP and the hunt moves to the coupling chain.
//!
//! IPC cannot penetrate, so engagement comes from an upward `+ẑ` load on the top face
//! pressing the block into a ceiling plane sitting just above the rest face
//! (`height = EDGE + ½·d̂`) — the same engaged scene as PR1's
//! `ipc_pose_sensitivity_matches_resolve_fd`, swept over loads to vary the engagement
//! depth (`b''(sd)`). The FD oracle re-runs the full nonlinear
//! backward-Euler Newton solve at the perturbed input (prev state, material μ, or
//! plane height) — a genuine cross-check, not an affine identity.

// precision: the small integer pattern index `k % 5` is exact as f64.
#![allow(
    clippy::expect_used,
    clippy::similar_names,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use sim_ml_chassis::autograd::VjpOp;
use sim_ml_chassis::{Tape, Tensor};
use sim_soft::readout::scene::pick_vertices_by_predicate;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, IpcRigidContact, IpcRigidContactSolver,
    LoadAxis, MaterialField, Mesh, RigidPlane, Solver, SolverConfig, Tet4, Vec3, VertexId,
};

const N_PER_EDGE: usize = 4;
const EDGE: f64 = 0.1;
const MU: f64 = 3.0e4;
const LAMBDA: f64 = 4.0 * MU;
const KAPPA: f64 = 3.0e4;
const D_HAT: f64 = 1.0e-2;
const DT: f64 = 1.0e-3;
const SETTLE_STEPS: usize = 400;

fn field(mu: f64, lambda: f64) -> MaterialField {
    MaterialField::uniform(mu, lambda)
}

fn block(mu: f64, lambda: f64) -> HandBuiltTetMesh {
    HandBuiltTetMesh::uniform_block(N_PER_EDGE, EDGE, &field(mu, lambda))
}

fn bottom_pins() -> Vec<VertexId> {
    pick_vertices_by_predicate(&block(MU, LAMBDA), |p| p.z.abs() < 1e-9)
}

fn top_face() -> Vec<VertexId> {
    pick_vertices_by_predicate(&block(MU, LAMBDA), |p| (p.z - EDGE).abs() < 1e-9)
}

/// Solver: IPC ceiling plane at `height` (`sd = height − z`) + an upward `+ẑ` load on
/// the top face (the barrier must hold the block out, `sd > 0`).
fn solver(height: f64, mu: f64, lambda: f64) -> IpcRigidContactSolver<HandBuiltTetMesh> {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), -height);
    let contact = IpcRigidContact::with_params(vec![plane], KAPPA, D_HAT);
    let loaded = top_face().iter().map(|&v| (v, LoadAxis::AxisZ)).collect();
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.max_newton_iter = 80;
    CpuNewtonSolver::new(
        Tet4,
        block(mu, lambda),
        contact,
        cfg,
        BoundaryConditions::new(bottom_pins(), loaded),
    )
}

/// One converged step's `x_final` from `(x_prev, v_prev)` at `(height, mu, lambda)`
/// under the upward load `load`.
fn solve_step(
    height: f64,
    mu: f64,
    lambda: f64,
    load: f64,
    x_prev: &[f64],
    v_prev: &[f64],
) -> Vec<f64> {
    let n = x_prev.len();
    solver(height, mu, lambda)
        .replay_step(
            &Tensor::from_slice(x_prev, &[n]),
            &Tensor::from_slice(v_prev, &[n]),
            &Tensor::from_slice(&[load], &[1]),
            DT,
        )
        .x_final
}

/// Engaged near-equilibrium at upward load `load`: the load presses the top face into
/// the barrier, resting at a positive `sd < d̂` (NO penetration — the IPC property
/// penalty lacks). Heavier loads push the block deeper (smaller `sd`, larger `b''`).
fn settle(load: f64) -> (f64, Vec<f64>, Vec<f64>) {
    let mesh = block(MU, LAMBDA);
    let n = 3 * mesh.n_vertices();
    let height = EDGE + 0.5 * D_HAT;
    let mut x = vec![0.0_f64; n];
    for (c, p) in x.chunks_exact_mut(3).zip(mesh.positions().iter()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    let mut v = vec![0.0_f64; n];
    for _ in 0..SETTLE_STEPS {
        let xf = solve_step(height, MU, LAMBDA, load, &x, &v);
        for (vi, (&xf_i, &xo)) in v.iter_mut().zip(xf.iter().zip(x.iter())) {
            *vi = (xf_i - xo) / DT;
        }
        x = xf;
    }
    (height, x, v)
}

fn free_dofs() -> Vec<usize> {
    let mut free = Vec::new();
    for (v, p) in block(MU, LAMBDA).positions().iter().enumerate() {
        if p.z.abs() >= 1e-9 {
            free.extend([3 * v, 3 * v + 1, 3 * v + 2]);
        }
    }
    free
}

/// Min top-face gap `sd = height − z` at positions `x` — the engagement depth.
fn min_sd(height: f64, x: &[f64]) -> f64 {
    top_face()
        .iter()
        .map(|&v| height - x[3 * v as usize + 2])
        .fold(f64::INFINITY, f64::min)
}

/// Validates that `TrajectoryStepVjp` routes the correct cotangent to each of its
/// four parents under IPC's varying curvature — every one matched to an independent
/// re-solve FD. This is the recon §9 localization probe, swept across engagement
/// depths (marginal `sd ≈ 0.9·d̂` where `b''→0` through firmly engaged `sd ≈ 0.3·d̂`)
/// to separate "marginal regime" from "factor bug" on the sim-soft side.
#[test]
fn ipc_trajectory_step_vjp_all_four_parents_match_fd() {
    // Loads chosen to span the engagement band the soft barrier admits: 4 → marginal
    // (sd ≈ 0.92·d̂, b''→0), 200 → firmly engaged (sd ≈ 0.60·d̂, b'' well clear of the
    // d̂ zero). The block + soft barrier resist deeper compression, so this is the
    // firm end this scene reaches — enough to separate "marginal regime" from a real
    // factor bug on the sim-soft side.
    for &load in &[4.0_f64, 50.0, 200.0] {
        check_four_parents(load);
    }
}

fn check_four_parents(load: f64) {
    let (height, x_prev, v_prev) = settle(load);
    let n = x_prev.len();
    let s = solver(height, MU, LAMBDA);
    let x_final = solve_step(height, MU, LAMBDA, load, &x_prev, &v_prev);

    let sd = min_sd(height, &x_final);
    eprintln!("\n[load={load}] IPC settled min sd = {sd:.6e} (d̂ = {D_HAT:.1e})");
    assert!(
        sd > 1e-4 && sd < D_HAT,
        "engaged, non-penetrating, clear of d̂: sd={sd:e}"
    );

    // L = Σx*: build the 4-parent node and backward (cotangent = ones).
    let dir = Vec3::new(0.0, 0.0, 1.0);
    let mut tape = Tape::new();
    let xprev_var = tape.param_tensor(Tensor::from_slice(&x_prev, &[n]));
    let vprev_var = tape.param_tensor(Tensor::from_slice(&v_prev, &[n]));
    let mu_var = tape.param_tensor(Tensor::from_slice(&[MU], &[1]));
    let pose_var = tape.param_tensor(Tensor::from_slice(&[height], &[1]));
    let xstar = tape.push_custom(
        &[xprev_var, vprev_var, mu_var, pose_var],
        Tensor::from_slice(&x_final, &[n]),
        Box::new(s.trajectory_step_vjp(&x_final, DT, 0, dir)),
    );
    tape.backward(xstar);
    let g_xprev = tape.grad_tensor(xprev_var).as_slice().to_vec();
    let g_vprev = tape.grad_tensor(vprev_var).as_slice().to_vec();
    let g_mu = tape.grad_tensor(mu_var).as_slice()[0];
    let g_pose = tape.grad_tensor(pose_var).as_slice()[0];

    let sum = |x: &[f64]| -> f64 { x.iter().sum() };
    let dot = |a: &[f64], b: &[f64]| -> f64 { a.iter().zip(b).map(|(x, y)| x * y).sum() };
    let eps_pos = 1.0e-7;

    // (1) x_prev — directional FD along a free-only pattern.
    let mut d = vec![0.0_f64; n];
    for (k, &i) in free_dofs().iter().enumerate() {
        d[i] = ((k % 5) as f64 - 2.0) * 0.3 + 0.1;
    }
    let xp_plus: Vec<f64> = x_prev
        .iter()
        .zip(&d)
        .map(|(a, e)| a + eps_pos * e)
        .collect();
    let xp_minus: Vec<f64> = x_prev
        .iter()
        .zip(&d)
        .map(|(a, e)| a - eps_pos * e)
        .collect();
    let fd_xprev = (sum(&solve_step(height, MU, LAMBDA, load, &xp_plus, &v_prev))
        - sum(&solve_step(height, MU, LAMBDA, load, &xp_minus, &v_prev)))
        / (2.0 * eps_pos);
    let rev_xprev = dot(&g_xprev, &d);

    // (2) v_prev — directional FD.
    let vp_plus: Vec<f64> = v_prev
        .iter()
        .zip(&d)
        .map(|(a, e)| a + eps_pos * e)
        .collect();
    let vp_minus: Vec<f64> = v_prev
        .iter()
        .zip(&d)
        .map(|(a, e)| a - eps_pos * e)
        .collect();
    let fd_vprev = (sum(&solve_step(height, MU, LAMBDA, load, &x_prev, &vp_plus))
        - sum(&solve_step(height, MU, LAMBDA, load, &x_prev, &vp_minus)))
        / (2.0 * eps_pos);
    let rev_vprev = dot(&g_vprev, &d);

    // (3) μ — re-solve with the block rebuilt at μ±ε (λ held fixed). The μ signal
    // here is small (∂Σx*/∂μ ~ 1e-8); a larger relative step lifts the central
    // difference above the per-solve roundoff floor (the function is near-linear in μ
    // over this window, so truncation stays negligible).
    let eps_mu = MU * 1e-4;
    let fd_mu = (sum(&solve_step(
        height,
        MU + eps_mu,
        LAMBDA,
        load,
        &x_prev,
        &v_prev,
    )) - sum(&solve_step(
        height,
        MU - eps_mu,
        LAMBDA,
        load,
        &x_prev,
        &v_prev,
    ))) / (2.0 * eps_mu);

    // (4) pose — re-solve at height ± ε (the plane translation δ ≡ height). This is
    // the cotangent that varies with `b''` (κ·b''·(−n̂·dir)·n̂) — the IPC-specific term.
    let eps_h = 1.0e-7;
    let fd_pose = (sum(&solve_step(
        height + eps_h,
        MU,
        LAMBDA,
        load,
        &x_prev,
        &v_prev,
    )) - sum(&solve_step(
        height - eps_h,
        MU,
        LAMBDA,
        load,
        &x_prev,
        &v_prev,
    ))) / (2.0 * eps_h);

    eprintln!(
        "IPC TrajectoryStepVjp parents vs FD:\n  x_prev rev={rev_xprev:.6e} fd={fd_xprev:.6e}\n  \
         v_prev rev={rev_vprev:.6e} fd={fd_vprev:.6e}\n  μ rev={g_mu:.6e} fd={fd_mu:.6e}\n  \
         pose rev={g_pose:.6e} fd={fd_pose:.6e}"
    );

    let rel = |a: f64, b: f64| (a - b).abs() / b.abs().max(1e-12);
    assert!(
        rev_xprev.abs() > 1e-6 && rel(rev_xprev, fd_xprev) < 1e-5,
        "x_prev cotangent: rev={rev_xprev:e} fd={fd_xprev:e}"
    );
    assert!(
        rev_vprev.abs() > 1e-9 && rel(rev_vprev, fd_vprev) < 1e-4,
        "v_prev cotangent: rev={rev_vprev:e} fd={fd_vprev:e}"
    );
    assert!(
        g_mu.abs() > 1e-9 && rel(g_mu, fd_mu) < 1e-5,
        "μ cotangent: rev={g_mu:e} fd={fd_mu:e}"
    );
    assert!(
        g_pose.abs() > 1e-3 && rel(g_pose, fd_pose) < 1e-5,
        "pose cotangent: rev={g_pose:e} fd={fd_pose:e}"
    );
}

/// Scalar readout `L = Σ wᵢ·x*ᵢ` over a custom weight vector — a downstream node so
/// `tape.backward` seeds the soft node with the SPARSE cotangent `w` (instead of the
/// uniform `ones` that `Σx*` produces). Parents `[x*]` (`[n_dof]`), output `[1]`.
#[derive(Debug)]
struct WeightedReadout {
    w: Vec<f64>,
}

impl VjpOp for WeightedReadout {
    fn op_id(&self) -> &'static str {
        "test::WeightedReadout"
    }
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        let c = cotangent.as_slice()[0];
        let slot = parent_cotans[0].as_mut_slice();
        for (s, &wi) in slot.iter_mut().zip(&self.w) {
            *s += c * wi;
        }
    }
}

/// The coupling feeds the soft node a CONTACT-CONCENTRATED cotangent (from the
/// `∂fz/∂x*` readout — sparse on the top-face z-DOFs, weighted by `−cᵥ·n̂`), NOT the
/// uniform `ones` the `Σx*` test uses. A VJP correct for `ones` need not be correct
/// for a sparse cotangent (free/pinned-DOF restriction bugs hide under a uniform
/// seed). This re-checks all four parents under `L = Σ_top x*.z` — the readout
/// pattern the keystone coupling actually backpropagates.
#[test]
fn ipc_trajectory_step_vjp_contact_pattern_cotangent() {
    let load = 50.0_f64;
    let (height, x_prev, v_prev) = settle(load);
    let n = x_prev.len();
    let s = solver(height, MU, LAMBDA);
    let x_final = solve_step(height, MU, LAMBDA, load, &x_prev, &v_prev);

    // Weight vector: the top-face z-DOFs (the contact-active pattern), 0 elsewhere.
    let mut w = vec![0.0_f64; n];
    for &v in &top_face() {
        w[3 * v as usize + 2] = 1.0;
    }
    let readout = |x: &[f64]| -> f64 { x.iter().zip(&w).map(|(a, b)| a * b).sum() };

    let dir = Vec3::new(0.0, 0.0, 1.0);
    let mut tape = Tape::new();
    let xprev_var = tape.param_tensor(Tensor::from_slice(&x_prev, &[n]));
    let vprev_var = tape.param_tensor(Tensor::from_slice(&v_prev, &[n]));
    let mu_var = tape.param_tensor(Tensor::from_slice(&[MU], &[1]));
    let pose_var = tape.param_tensor(Tensor::from_slice(&[height], &[1]));
    let xstar = tape.push_custom(
        &[xprev_var, vprev_var, mu_var, pose_var],
        Tensor::from_slice(&x_final, &[n]),
        Box::new(s.trajectory_step_vjp(&x_final, DT, 0, dir)),
    );
    // Downstream weighting node ⇒ the soft node is seeded with the sparse cotangent w.
    let l_var = tape.push_custom(
        &[xstar],
        Tensor::from_slice(&[readout(&x_final)], &[1]),
        Box::new(WeightedReadout { w: w.clone() }),
    );
    tape.backward(l_var);
    let g_xprev = tape.grad_tensor(xprev_var).as_slice().to_vec();
    let g_vprev = tape.grad_tensor(vprev_var).as_slice().to_vec();
    let g_mu = tape.grad_tensor(mu_var).as_slice()[0];
    let g_pose = tape.grad_tensor(pose_var).as_slice()[0];

    let dot = |a: &[f64], b: &[f64]| -> f64 { a.iter().zip(b).map(|(x, y)| x * y).sum() };
    let eps_pos = 1.0e-7;

    let mut d = vec![0.0_f64; n];
    for (k, &i) in free_dofs().iter().enumerate() {
        d[i] = ((k % 5) as f64 - 2.0) * 0.3 + 0.1;
    }
    let perturb = |base: &[f64], sign: f64| -> Vec<f64> {
        base.iter()
            .zip(&d)
            .map(|(a, e)| a + sign * eps_pos * e)
            .collect()
    };
    let fd_xprev = (readout(&solve_step(
        height,
        MU,
        LAMBDA,
        load,
        &perturb(&x_prev, 1.0),
        &v_prev,
    )) - readout(&solve_step(
        height,
        MU,
        LAMBDA,
        load,
        &perturb(&x_prev, -1.0),
        &v_prev,
    ))) / (2.0 * eps_pos);
    let fd_vprev = (readout(&solve_step(
        height,
        MU,
        LAMBDA,
        load,
        &x_prev,
        &perturb(&v_prev, 1.0),
    )) - readout(&solve_step(
        height,
        MU,
        LAMBDA,
        load,
        &x_prev,
        &perturb(&v_prev, -1.0),
    ))) / (2.0 * eps_pos);
    let eps_mu = MU * 1e-4;
    let fd_mu = (readout(&solve_step(
        height,
        MU + eps_mu,
        LAMBDA,
        load,
        &x_prev,
        &v_prev,
    )) - readout(&solve_step(
        height,
        MU - eps_mu,
        LAMBDA,
        load,
        &x_prev,
        &v_prev,
    ))) / (2.0 * eps_mu);
    let eps_h = 1.0e-7;
    let fd_pose = (readout(&solve_step(
        height + eps_h,
        MU,
        LAMBDA,
        load,
        &x_prev,
        &v_prev,
    )) - readout(&solve_step(
        height - eps_h,
        MU,
        LAMBDA,
        load,
        &x_prev,
        &v_prev,
    ))) / (2.0 * eps_h);

    let rev_xprev = dot(&g_xprev, &d);
    let rev_vprev = dot(&g_vprev, &d);
    eprintln!(
        "IPC contact-pattern cotangent vs FD:\n  x_prev rev={rev_xprev:.6e} fd={fd_xprev:.6e}\n  \
         v_prev rev={rev_vprev:.6e} fd={fd_vprev:.6e}\n  μ rev={g_mu:.6e} fd={fd_mu:.6e}\n  \
         pose rev={g_pose:.6e} fd={fd_pose:.6e}"
    );
    let rel = |a: f64, b: f64| (a - b).abs() / b.abs().max(1e-12);
    assert!(
        rel(rev_xprev, fd_xprev) < 1e-5,
        "x_prev: {rev_xprev:e} vs {fd_xprev:e}"
    );
    assert!(
        rel(rev_vprev, fd_vprev) < 1e-4,
        "v_prev: {rev_vprev:e} vs {fd_vprev:e}"
    );
    assert!(rel(g_mu, fd_mu) < 1e-4, "μ: {g_mu:e} vs {fd_mu:e}");
    assert!(
        rel(g_pose, fd_pose) < 1e-5,
        "pose: {g_pose:e} vs {fd_pose:e}"
    );
}
