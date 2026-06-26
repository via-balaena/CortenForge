//! L1b site-2 isolation: the friction-aware soft tangent `A` on a FINITE sphere collider.
//!
//! `equilibrium_friction_coeff_sensitivity` is `∂x*/∂μ_c = −A⁻¹·(∇D/μ_c)`. The RHS `∇D/μ_c`
//! is curvature-INDEPENDENT (just the friction force / `μ_c`), so this sensitivity tests the
//! tangent `A` ALONE — in particular whether `A` carries the curved-normal term `∂(∇D)/∂x|_curv
//! = DN·H` (the friction tangent frame rotates as the sphere's contact normal turns, `∂n̂/∂x =
//! H`). On the infinite plane `H = 0` so the existing frozen-lag `∇²D` + λ-coupling Woodbury are
//! exact (the plane gate `friction_coeff_gradient.rs` is machine-exact); on the sphere a dropped
//! curved-T term shows up as a curvature-scale (~1e-2) mismatch vs the friction re-solve FD.
//!
//! This isolates the soft tangent from the multi-step rigid carry / reaction readout that the
//! coupling-level `sphere_friction_trajectory_gradient.rs` gate composes. FD-step sweep is the
//! discriminator (a correct analytic bottoms out in a roundoff-vs-truncation V; a missing term
//! plateaus at a fixed floor).

#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names
)]

use nalgebra::Point3;
use sim_ml_chassis::Tensor;
use sim_ml_chassis::autograd::{Tape, VjpOp};
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, LoadAxis, MaterialField, Mesh,
    PenaltyRigidContact, RigidTwist, Sdf, Solver, SolverConfig, SphereSdf, Tet4, Vec3,
    pick_vertices_by_predicate,
};

const EDGE: f64 = 0.1;
const DT: f64 = 1.0e-3;
const KAPPA: f64 = 5.0e3;
const D_HAT: f64 = 5.0e-3;
const GRAV_UP: f64 = 10.0;
const FRIC_MU: f64 = 3.0;
const EPS_V: f64 = 0.1;
const FX_TOTAL: f64 = 40.0;
const DRIFT: f64 = 5.0e-4;
/// Sphere radius — large vs the block so the south-pole patch spans several top-face vertices
/// yet stays curved enough that the geometric stiffness is materially nonzero.
const SPHERE_R: f64 = 0.08;
/// South-pole penetration into the rest top face (z = EDGE).
const PENETRATION: f64 = 3.0e-3;

/// A `SphereSdf` posed at a world `center` (forwards `eval`/`grad`/`hessian`), the finite
/// collider the coupling's `with_sphere_collider` builds.
#[derive(Clone, Debug)]
struct PosedSphere {
    inner: SphereSdf,
    center: Vec3,
}
impl Sdf for PosedSphere {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.inner.eval(p - self.center)
    }
    fn grad(&self, p: Point3<f64>) -> Vec3 {
        self.inner.grad(p - self.center)
    }
    fn hessian(&self, p: Point3<f64>) -> nalgebra::Matrix3<f64> {
        self.inner.hessian(p - self.center)
    }
}

fn sphere() -> PosedSphere {
    PosedSphere {
        inner: SphereSdf { radius: SPHERE_R },
        center: Vec3::new(EDGE / 2.0, EDGE / 2.0, EDGE + SPHERE_R - PENETRATION),
    }
}

fn block(mu: f64, lambda: f64) -> HandBuiltTetMesh {
    HandBuiltTetMesh::uniform_block(4, EDGE, &MaterialField::uniform(mu, lambda))
}
fn x_rest(mu: f64, lambda: f64) -> Vec<f64> {
    let mesh = block(mu, lambda);
    let mut x = vec![0.0_f64; 3 * mesh.n_vertices()];
    for (i, p) in mesh.positions().iter().enumerate() {
        x[3 * i] = p.x;
        x[3 * i + 1] = p.y;
        x[3 * i + 2] = p.z;
    }
    x
}
fn top_vertices(mu: f64, lambda: f64) -> Vec<usize> {
    let mesh = block(mu, lambda);
    let rest = mesh.positions();
    (0..mesh.n_vertices())
        .filter(|&i| (rest[i].z - EDGE).abs() < 1e-9)
        .collect()
}
fn theta(mu: f64, lambda: f64) -> Vec<f64> {
    let top = top_vertices(mu, lambda);
    let fx = FX_TOTAL / top.len() as f64;
    let mut th = vec![0.0_f64; 3 * top.len()];
    for k in 0..top.len() {
        th[3 * k] = fx;
    }
    th
}
fn build(
    mu: f64,
    lambda: f64,
    fric_mu: f64,
) -> CpuNewtonSolver<Tet4, HandBuiltTetMesh, PenaltyRigidContact> {
    let mesh = block(mu, lambda);
    let pinned = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    let top = top_vertices(mu, lambda);
    let contact = PenaltyRigidContact::with_params(vec![sphere()], KAPPA, D_HAT);
    let loaded: Vec<(u32, LoadAxis)> = top
        .iter()
        .map(|&i| (i as u32, LoadAxis::FullVector))
        .collect();
    let bc = BoundaryConditions::new(pinned, loaded);
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.gravity_z = GRAV_UP;
    cfg.friction_mu = fric_mu;
    cfg.friction_eps_v = EPS_V;
    cfg.max_newton_iter = 80;
    cfg.max_line_search_backtracks = 60;
    CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc)
        .with_friction_surface_drift(Vec3::new(DRIFT, 0.0, 0.0))
}
fn solve(mu: f64, lambda: f64, fric_mu: f64) -> Vec<f64> {
    let nd = 3 * block(mu, lambda).n_vertices();
    let top = top_vertices(mu, lambda);
    build(mu, lambda, fric_mu)
        .replay_step(
            &Tensor::from_slice(&x_rest(mu, lambda), &[nd]),
            &Tensor::from_slice(&vec![0.0_f64; nd], &[nd]),
            &Tensor::from_slice(&theta(mu, lambda), &[3 * top.len()]),
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

// react/drift/pose directions matching the coupling's free-body friction carry.
const fn react_dir() -> Vec3 {
    Vec3::new(-1.0, 0.0, 0.0)
}
const fn drift_dir() -> Vec3 {
    Vec3::new(1.0, 0.0, 0.0)
}
const fn pose_dir() -> Vec3 {
    Vec3::new(0.0, 0.0, 1.0)
}

/// Site 1 isolation: the friction-REACTION readout `F = (Σ∇D)·react_dir` and its `∂F/∂x*` /
/// `∂F/∂height` (the `friction_reaction_gradients` the coupling routes onto the rigid tape)
/// match direct FD on the sphere. The curved-normal term `DN·H` (state) and `DN·(−H·ẑ)`
/// (height) is what a sphere needs over the plane's constant normal.
#[test]
fn friction_sphere_reaction_readout_matches_fd() {
    let mu = 3.0e3;
    let lambda = 1.2e4;
    let x_prev = x_rest(mu, lambda);
    let x_final = solve(mu, lambda, FRIC_MU);
    let solver = build(mu, lambda, FRIC_MU);
    let rg = solver.friction_reaction_gradients(
        &x_final,
        &x_prev,
        DT,
        react_dir(),
        drift_dir(),
        pose_dir(),
    );
    // ∂F/∂x* : perturb each soft DOF, central-difference F.
    let force_at = |x: &[f64]| {
        build(mu, lambda, FRIC_MU)
            .friction_reaction_gradients(x, &x_prev, DT, react_dir(), drift_dir(), pose_dir())
            .force
    };
    let eps = 1e-8;
    let mut num = 0.0;
    let mut den = 0.0;
    for i in 0..x_final.len() {
        if rg.dforce_dx[i].abs() < 1e-12 {
            continue;
        }
        let (mut xp, mut xm) = (x_final.clone(), x_final.clone());
        xp[i] += eps;
        xm[i] -= eps;
        let fd = (force_at(&xp) - force_at(&xm)) / (2.0 * eps);
        num += (rg.dforce_dx[i] - fd).powi(2);
        den += fd * fd;
    }
    let rel_x = (num / den.max(1e-30)).sqrt();
    eprintln!("∂F/∂x* rel-vs-FD = {rel_x:.3e}");
    assert!(den.sqrt() > 1e-6, "∂F/∂x* implausibly small");
    assert!(rel_x < 1e-5, "∂F/∂x* disagrees with FD: {rel_x:e}");

    // ∂F/∂height : move the sphere centre by ±ε along +ẑ (pose_dir), holding x* fixed.
    let force_at_height = |dz: f64| {
        let mesh = block(mu, lambda);
        let pinned = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
        let top = top_vertices(mu, lambda);
        let mut s = sphere();
        s.center.z += dz;
        let contact = PenaltyRigidContact::with_params(vec![s], KAPPA, D_HAT);
        let loaded: Vec<(u32, LoadAxis)> = top
            .iter()
            .map(|&i| (i as u32, LoadAxis::FullVector))
            .collect();
        let bc = BoundaryConditions::new(pinned, loaded);
        let mut cfg = SolverConfig::skeleton();
        cfg.dt = DT;
        cfg.gravity_z = GRAV_UP;
        cfg.friction_mu = FRIC_MU;
        cfg.friction_eps_v = EPS_V;
        cfg.max_newton_iter = 80;
        cfg.max_line_search_backtracks = 60;
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc)
            .with_friction_surface_drift(Vec3::new(DRIFT, 0.0, 0.0))
            .friction_reaction_gradients(
                &x_final,
                &x_prev,
                DT,
                react_dir(),
                drift_dir(),
                pose_dir(),
            )
            .force
    };
    let eps_h = 1e-8;
    let fd_h = (force_at_height(eps_h) - force_at_height(-eps_h)) / (2.0 * eps_h);
    let rel_h = (rg.dforce_dheight - fd_h).abs() / fd_h.abs().max(1e-12);
    eprintln!(
        "∂F/∂height analytic={:.6e} fd={fd_h:.6e} rel={rel_h:.3e}",
        rg.dforce_dheight
    );
    assert!(fd_h.abs() > 1e-6, "∂F/∂height implausibly small");
    assert!(rel_h < 1e-5, "∂F/∂height disagrees with FD: {rel_h:e}");
}

/// Re-solve x* with the sphere centre shifted by `dz` along +ẑ (the height carry) — the FD
/// oracle for `∂x*/∂height` under friction.
fn solve_with_sphere_dz(mu: f64, lambda: f64, dz: f64) -> Vec<f64> {
    let mesh = block(mu, lambda);
    let nd = 3 * mesh.n_vertices();
    let pinned = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    let top = top_vertices(mu, lambda);
    let mut s = sphere();
    s.center.z += dz;
    let contact = PenaltyRigidContact::with_params(vec![s], KAPPA, D_HAT);
    let loaded: Vec<(u32, LoadAxis)> = top
        .iter()
        .map(|&i| (i as u32, LoadAxis::FullVector))
        .collect();
    let bc = BoundaryConditions::new(pinned, loaded);
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.gravity_z = GRAV_UP;
    cfg.friction_mu = FRIC_MU;
    cfg.friction_eps_v = EPS_V;
    cfg.max_newton_iter = 80;
    cfg.max_line_search_backtracks = 60;
    CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc)
        .with_friction_surface_drift(Vec3::new(DRIFT, 0.0, 0.0))
        .replay_step(
            &Tensor::from_slice(&x_rest(mu, lambda), &[nd]),
            &Tensor::from_slice(&vec![0.0_f64; nd], &[nd]),
            &Tensor::from_slice(&theta(mu, lambda), &[3 * top.len()]),
            DT,
        )
        .x_final
}

/// Site 2+3 isolation on the HEIGHT channel: `∂x*/∂height` (the soft equilibrium's sensitivity
/// to the sphere translating +ẑ) under friction matches the re-solve FD. Exercises the friction
/// tangent `A` (curved-T, site 2) AND the friction pose-residual grad (site 3) together — the
/// channel the multi-step grip gradient's height↔friction feedback rides.
#[test]
fn friction_sphere_pose_sensitivity_matches_fd() {
    let mu = 3.0e3;
    let lambda = 1.2e4;
    let x_prev = x_rest(mu, lambda);
    let x_final = solve(mu, lambda, FRIC_MU);
    let an = build(mu, lambda, FRIC_MU).equilibrium_pose_sensitivity(
        &x_final,
        Some(&x_prev),
        DT,
        RigidTwist::translation(Vec3::new(0.0, 0.0, 1.0)),
    );
    let max: f64 = an.iter().map(|v| v.abs()).fold(0.0, f64::max);
    let mut best = f64::INFINITY;
    for k in 3..=9 {
        let dz = 10f64.powi(-k);
        let fd: Vec<f64> = solve_with_sphere_dz(mu, lambda, dz)
            .iter()
            .zip(&solve_with_sphere_dz(mu, lambda, -dz))
            .map(|(a, b)| (a - b) / (2.0 * dz))
            .collect();
        let r = rel(&an, &fd);
        eprintln!("  dz=1e-{k}: rel={r:.3e}");
        best = best.min(r);
    }
    eprintln!("sphere ∂x*/∂height: ‖·‖∞={max:.3e}  best-rel-vs-FD={best:.3e}");
    assert!(max > 1e-9, "∂x*/∂height implausibly small");
    assert!(
        best < 1e-5,
        "sphere ∂x*/∂height disagrees with FD even at best step: {best:e}"
    );
}

/// Site-2-via-drift isolation: `∂x*/∂Δ_surf` (the moving-collider drift sensitivity) under
/// friction on the sphere. The drift RHS has no curved term (`n̂` ⊥ the friction reference
/// shift), but it solves through the curved-T tangent `A`, so this is a second independent
/// check of site 2 in a different RHS direction.
#[test]
fn friction_sphere_drift_sensitivity_matches_fd() {
    let mu = 3.0e3;
    let lambda = 1.2e4;
    let x_prev = x_rest(mu, lambda);
    let x_final = solve(mu, lambda, FRIC_MU);
    let dir = Vec3::new(1.0, 0.0, 0.0);
    let an = build(mu, lambda, FRIC_MU).equilibrium_drift_sensitivity(&x_final, &x_prev, DT, dir);
    // FD: re-solve with the surface drift shifted by ±ε·dir.
    let solve_drift = |ddrift: f64| {
        let mesh = block(mu, lambda);
        let nd = 3 * mesh.n_vertices();
        let pinned = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
        let top = top_vertices(mu, lambda);
        let contact = PenaltyRigidContact::with_params(vec![sphere()], KAPPA, D_HAT);
        let loaded: Vec<(u32, LoadAxis)> = top
            .iter()
            .map(|&i| (i as u32, LoadAxis::FullVector))
            .collect();
        let bc = BoundaryConditions::new(pinned, loaded);
        let mut cfg = SolverConfig::skeleton();
        cfg.dt = DT;
        cfg.gravity_z = GRAV_UP;
        cfg.friction_mu = FRIC_MU;
        cfg.friction_eps_v = EPS_V;
        cfg.max_newton_iter = 80;
        cfg.max_line_search_backtracks = 60;
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc)
            .with_friction_surface_drift(Vec3::new(DRIFT + ddrift, 0.0, 0.0))
            .replay_step(
                &Tensor::from_slice(&x_rest(mu, lambda), &[nd]),
                &Tensor::from_slice(&vec![0.0_f64; nd], &[nd]),
                &Tensor::from_slice(&theta(mu, lambda), &[3 * top.len()]),
                DT,
            )
            .x_final
    };
    let mut best = f64::INFINITY;
    for k in 4..=9 {
        let de = 10f64.powi(-k);
        let fd: Vec<f64> = solve_drift(de)
            .iter()
            .zip(&solve_drift(-de))
            .map(|(a, b)| (a - b) / (2.0 * de))
            .collect();
        best = best.min(rel(&an, &fd));
    }
    eprintln!("sphere ∂x*/∂drift: best-rel-vs-FD={best:.3e}");
    assert!(best < 1e-5, "∂x*/∂drift disagrees with FD: {best:e}");
}

/// Sum-reduce VJP for the reverse-mode check below.
#[derive(Debug)]
struct SumVjp;
impl VjpOp for SumVjp {
    fn op_id(&self) -> &'static str {
        "test::Sum"
    }
    fn vjp(&self, cot: &Tensor<f64>, parents: &mut [Tensor<f64>]) {
        let c = cot.as_slice()[0];
        for x in parents[0].as_mut_slice() {
            *x += c;
        }
    }
}

/// Reverse-mode (A⁻ᵀ) validation of the curved-T tangent on the sphere: the grip step VJP's
/// `∂(Σx*)/∂μ` (one `tape.backward`) must equal the sum of the forward
/// `equilibrium_material_sensitivity` (`A⁻¹`). The two contract the SAME `(A, ∂r/∂μ)` with
/// transpose vs forward; a wrong transpose of the asymmetric curved-T Woodbury columns shows
/// up here (and ONLY here — every other isolation is forward-mode).
#[test]
fn friction_sphere_reverse_mode_matches_forward() {
    let mu = 3.0e3;
    let lambda = 1.2e4;
    let x_prev = x_rest(mu, lambda);
    let x_final = solve(mu, lambda, FRIC_MU);
    let nd = x_final.len();
    let solver = build(mu, lambda, FRIC_MU);
    // Forward Σ∂x*/∂μ.
    let fwd: f64 = solver
        .equilibrium_material_sensitivity(&x_final, Some(&x_prev), DT, 0)
        .iter()
        .sum();
    // Reverse via the grip step VJP (pose +ẑ, drift +x̂ — same as the coupling).
    let grip = solver.trajectory_step_vjp_grip(
        &x_final,
        &x_prev,
        DT,
        0,
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    let mut tape = Tape::new();
    let xp = tape.constant_tensor(Tensor::from_slice(&x_prev, &[nd]));
    let vp = tape.constant_tensor(Tensor::from_slice(&vec![0.0; nd], &[nd]));
    let p = tape.param_tensor(Tensor::from_slice(&[mu], &[1]));
    let z = tape.constant_tensor(Tensor::from_slice(&[0.0], &[1]));
    let dr = tape.constant_tensor(Tensor::from_slice(&[0.0], &[1]));
    let xstar = tape.push_custom(
        &[xp, vp, p, z, dr],
        Tensor::from_slice(&x_final, &[nd]),
        Box::new(grip),
    );
    let l = tape.push_custom(
        &[xstar],
        Tensor::from_slice(&[x_final.iter().sum()], &[1]),
        Box::new(SumVjp),
    );
    tape.backward(l);
    let rev = tape.grad_tensor(p).as_slice()[0];
    let rel = (fwd - rev).abs() / fwd.abs().max(1e-30);
    eprintln!("forward Σ∂x*/∂μ={fwd:.6e}  reverse={rev:.6e}  rel={rel:.3e}");
    assert!(fwd.abs() > 1e-12, "forward sensitivity implausibly small");
    assert!(
        rel < 1e-9,
        "reverse (A⁻ᵀ) disagrees with forward (A⁻¹): {rel:e}"
    );
}

/// `∂x*/∂μ_c` through the sphere friction tangent matches the friction re-solve FD — the
/// curved-T term in `A` is what makes it match (a flat `A` plateaus at ~1e-2).
#[test]
fn friction_sphere_coeff_sensitivity_matches_fd() {
    let mu = 3.0e3;
    let lambda = 1.2e4;
    let solver = build(mu, lambda, FRIC_MU);
    let x_prev = x_rest(mu, lambda);
    let x_final = solve(mu, lambda, FRIC_MU);
    let an = solver.equilibrium_friction_coeff_sensitivity(&x_final, &x_prev, DT);
    let max: f64 = an.iter().map(|v| v.abs()).fold(0.0, f64::max);
    let mut best = f64::INFINITY;
    for k in 2..=9 {
        let de = FRIC_MU * 10f64.powi(-k);
        let fd: Vec<f64> = solve(mu, lambda, FRIC_MU + de)
            .iter()
            .zip(&solve(mu, lambda, FRIC_MU - de))
            .map(|(a, b)| (a - b) / (2.0 * de))
            .collect();
        let r = rel(&an, &fd);
        eprintln!("  de=μ_c·1e-{k}: rel={r:.3e}");
        best = best.min(r);
    }
    eprintln!("sphere ∂x*/∂μ_c: ‖·‖∞={max:.3e}  best-rel-vs-FD={best:.3e}");
    assert!(
        max > 1e-9,
        "sensitivity implausibly small — no friction lever"
    );
    assert!(
        best < 1e-6,
        "sphere ∂x*/∂μ_c disagrees with FD even at best step: {best:e} (curved-T missing in A?)"
    );
}
