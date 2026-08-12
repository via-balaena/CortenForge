//! R1.2 — the reduced gradient. Two questions, deliberately gated apart.
//!
//! ## G1 (the kill gate): is the reduced adjoint the exact gradient of the reduced model?
//!
//! This is a statement about the constant-`Φ` implicit-function-theorem algebra and
//! nothing else. It is answered by finite-differencing `ReducedNewtonSolver::step`
//! itself, to the crate's gradcheck tolerance, and it is where an error in the
//! projection, the covector handling, or the tangent reflection actually shows up as a
//! wrong number.
//!
//! ## G2 (measured, loosely bounded): how far is it from the ORACLE's gradient?
//!
//! This is a statement about the *basis*, comparing the derivatives of two different
//! functions. R1.0/R1.1 already measured that their states differ by ~0.8–1.1 %, so
//! demanding gradcheck agreement here would fail for basis truncation — a cost already
//! priced — rather than for anything about differentiability. Gating the two together
//! would make a gate that cannot fail for the reason it claims.
//!
//! ## The prediction G2 exists to test
//!
//! A Galerkin adjoint `μ = Φ(ΦᵀAΦ)⁻¹Φᵀg` is the best approximation to the true adjoint
//! `λ = A⁻¹g` **in the energy norm over `span(Φ)`**. So the gradient error is the
//! *adjoint* field's representability in the basis — and the basis was fitted to
//! **displacement** snapshots, which were never asked to span an adjoint. Three
//! cotangents are measured, spanning how far each sits from the training ensemble:
//!
//! - **`face-z`** — `z` over the loaded face. Its adjoint is the response to a uniform
//!   top-face traction: the training loads' own class, a Gaussian patch of infinite
//!   width. The in-family case.
//! - **`node-z`** — one node's `z`. Same direction as the training loads, but a point
//!   load where they were smooth patches.
//! - **`Σx*`** — ones on every DOF, so unit forces along `x` and `y` as well. The
//!   training ensemble loaded `z` only, making this the out-of-family case.
//!
//! One cotangent alone would be the hand-picked fixture that agrees with whatever
//! hypothesis it was chosen for, and `face-z` in particular is the control that stops
//! "the gradient is bad" from being answered with "you asked for an exotic objective".
//!
//! ⚠ The ordering this predicted was **wrong**. Localization was expected to dominate;
//! it does not. `Σx*` (out-of-family *direction*) is the worst by a wide margin and
//! `node-z` (in-family direction, localized) sits between it and `face-z`. What the
//! basis has never been loaded along costs more than what it has never resolved.
//!
//! ## What is NOT tested here, and must not be claimed
//!
//! That holding `Φ` constant is the right *modelling* choice. G1 differentiates a
//! constant-`Φ` model with `Φ` held constant in the finite difference too, so it is
//! exact by construction and blind to the question. G2 is the number that prices the
//! decision — end to end, against the model with no basis at all — and it is reported,
//! not asserted tightly.

#![allow(
    clippy::expect_used,
    // Counters and step indices convert to f64 far below 2^53.
    clippy::cast_precision_loss,
    // `a`/`b`/`c`/`d`/`w`/`s`/`h`/`q` mirror the maths (the four unit-interval draws,
    // the Gaussian width, the LCG state, the FD step, reduced coordinates); longer
    // names would obscure the correspondence.
    clippy::many_single_char_names,
    // A failed oracle or reduced step is a broken gate, not a runtime condition.
    clippy::panic,
    clippy::too_many_lines
)]

use std::fmt::Write as _;
use std::time::Instant;

use sim_ml_chassis::{Tape, Tensor, Var};
use sim_soft::solver::backward_euler::reduced::{
    Inner, PodBasis, ReducedNewtonSolver, SnapshotSet,
};
use sim_soft::{
    BoundaryConditions, CpuTet4NHSolver, HandBuiltTetMesh, IndexOp, LoadAxis, MaterialField, Mesh,
    NullContact, Solver, SolverConfig, Tet4, Vec3, VertexId, pick_vertices_by_predicate,
};

const LX: f64 = 0.020;
const LY: f64 = 0.020;
const H: f64 = 0.006;
const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0 * MU;
const DENSITY: f64 = 1030.0;
const DT: f64 = 1.0 / 60.0;

/// Training trajectories and retained modes — R1.1's ensemble verbatim, so G2's numbers
/// sit on the same basis the 0.79 % projection floor and the 1.45–1.91x Galerkin
/// overhead were measured on. Changing either here would make R1.2's numbers
/// incomparable with the rung below it for no gain.
const N_TRAIN: usize = 48;
const STEPS: usize = 5;
const R_MODES: usize = 40;

/// Newton tolerance — tighter than R1.1's 1e-6, but **not for the reason it looks like,
/// and the difference was measured rather than assumed.**
///
/// A finite difference divides by its step, so residual slack that is invisible in a
/// forward trajectory can become the noise floor of `(J(θ+h) − J(θ−h)) / 2h`. That is
/// real, and it is what the first pilot tripped over. It is **not** what fixed it: a
/// negative control re-ran G1 at 1e-6 with the widened `FD_REL_THETA` below, and the
/// worst disagreement was 9.68e-7 — still an order of magnitude inside the gate. The
/// step size did all the work; the tolerance did none.
///
/// It stays at 1e-10 anyway, for margin rather than necessity: at 1e-6 the kill gate
/// clears by ~10x, at 1e-10 by ~350x, and a gate that can only be trusted to one order
/// on a noise-dominated quantity is one bad day from flaking. Recording *why* it is here
/// matters more than the value — a future reader tightening a tolerance to fix an FD
/// should know this knob was tried and measured not to be the one.
const TOL: f64 = 1.0e-10;

/// Relative finite-difference step for the **load** gradient.
///
/// Set by a measured sweep, not by convention. Perturbing one θ component moves `J` by
/// a small fraction of what the solver's stopping criterion leaves on the table, so the
/// central difference is noise-limited, and the noise falls as `1/h`. Measured on the
/// worst-conditioned of the three components checked (`j = 866`, the last loaded
/// vertex's `z`, whose gradient is ~9x smaller than the patch-centre component's and so
/// has the least signal to lift above the noise):
///
/// | `h / ‖θ‖` | rel. disagreement |
/// |---:|---:|
/// | 1e-6 | 1.05e-5 |
/// | 1e-5 | 6.59e-7 |
/// | **1e-4** | **2.30e-8** |
/// | 1e-3 | 8.26e-10 |
///
/// Monotone to the largest step tried — so the error is noise, falling as `1/h`, and
/// truncation never becomes the binding term over this range. That the difference
/// *converges to* the analytic value is the substance of the kill gate. 1e-4 sits three
/// orders inside the gate without pushing the perturbation to ~1 % of θ.
///
/// This constant, not `TOL`, is what moved the gate from failing to passing.
const FD_REL_THETA: f64 = 1.0e-4;

/// Relative finite-difference step for the **material** gradient — the crate's
/// convention (`material_sensitivity.rs`). A material perturbation moves every element,
/// so the signal is large and this step already lands at ~2e-7; it needs no widening.
const FD_REL_MATERIAL: f64 = 1.0e-6;

struct Rig {
    solver: CpuTet4NHSolver<HandBuiltTetMesh>,
    x_rest: Vec<f64>,
    loaded: Vec<VertexId>,
    n_dof: usize,
}

#[derive(Clone, Copy, Debug)]
struct Traj {
    cx: f64,
    cy: f64,
    w: f64,
    p0: f64,
}

/// Build the fixture at a given first Lamé parameter. `mu` is a parameter so the
/// material finite difference can rebuild the solver at `μ ± δ` while reusing the SAME
/// basis — which is exactly the constant-`Φ` model whose gradient is under test.
fn rig(mu: f64) -> Rig {
    rig_with_tol(mu, TOL)
}

/// Rig at an explicit Newton tolerance.
///
/// `TOL` is tightened to 1e-10 for G1's finite differences, and that is **not** a fair
/// setting to time at. R1.1 measured `‖Φᵀr‖/‖r‖` between 1e-7 and 1e-10, so a fixed
/// absolute tolerance is a far harsher demand on the *projected* residual the reduced
/// solve drives than on the full residual the oracle drives — timing both at 1e-10
/// silently penalises the reduced path. The R1.3 sweep therefore times at the production
/// 1e-6, which is what R1.1 gated and what a consumer would run.
fn rig_with_tol(mu: f64, tol: f64) -> Rig {
    let field = MaterialField::uniform(mu, LAMBDA);
    let mesh = HandBuiltTetMesh::cantilever_bilayer_beam(16, 16, 6, LX, LY, H, &field);
    let n_dof = 3 * mesh.n_vertices();
    let mut x_rest = vec![0.0; n_dof];
    for (c, p) in x_rest.chunks_exact_mut(3).zip(mesh.positions()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    let pins = pick_vertices_by_predicate(&mesh, |p: &Vec3| p.z.abs() < 1e-12);
    let loaded: Vec<VertexId> =
        pick_vertices_by_predicate(&mesh, |p: &Vec3| (p.z - H).abs() < 1e-12);
    let bc = BoundaryConditions {
        pinned_vertices: pins,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::FullVector)).collect(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.density = DENSITY;
    cfg.max_newton_iter = 200;
    cfg.tol = tol;
    Rig {
        solver: CpuTet4NHSolver::new(Tet4, mesh, NullContact, cfg, bc),
        x_rest,
        loaded,
        n_dof,
    }
}

/// Deterministic sample of the parameter box — R1.1's LCG, so training and held-out
/// trajectories are the same ones that rung measured.
fn sample(k: u64) -> Traj {
    let mut s = k
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(1_442_695_040_888_963_407);
    let mut next = || {
        s = s.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        ((s >> 33) as f64) / ((1u64 << 31) as f64)
    };
    let (a, b, c, d) = (next(), next(), next(), next());
    Traj {
        cx: 0.007 + 0.006 * a,
        cy: 0.007 + 0.006 * b,
        w: 0.0025 + 0.0015 * c,
        p0: 0.10 + 0.20 * d,
    }
}

/// The load vector at step `s` of trajectory `t`: a Gaussian traction patch ramped
/// linearly to full magnitude.
fn theta_at(r: &Rig, t: Traj, s: usize) -> Vec<f64> {
    let frac = s as f64 / STEPS as f64;
    let mut th = vec![0.0; 3 * r.loaded.len()];
    for (i, &vid) in r.loaded.iter().enumerate() {
        let (px, py) = (r.x_rest[3 * vid as usize], r.x_rest[3 * vid as usize + 1]);
        let d2 = (px - t.cx).powi(2) + (py - t.cy).powi(2);
        th[3 * i + 2] = -frac * t.p0 * (-d2 / (2.0 * t.w * t.w)).exp();
    }
    th
}

/// Oracle trajectory: the converged FULL state after each step.
fn run_full(r: &Rig, t: Traj, steps: usize) -> Vec<Vec<f64>> {
    let mut x = r.x_rest.clone();
    let mut v = vec![0.0; r.n_dof];
    let mut out = Vec::new();
    for s in 1..=steps {
        let th = theta_at(r, t, s);
        let step = r.solver.replay_step(
            &Tensor::from_slice(&x, &[r.n_dof]),
            &Tensor::from_slice(&v, &[r.n_dof]),
            &Tensor::from_slice(&th, &[th.len()]),
            DT,
        );
        for i in 0..r.n_dof {
            v[i] = (step.x_final[i] - x[i]) / DT;
        }
        x = step.x_final;
        out.push(x.clone());
    }
    out
}

/// Train the R1.1 basis.
fn snapshots(r: &Rig) -> SnapshotSet {
    let fd = r.solver.free_dof_indices().to_vec();
    let mut set = SnapshotSet::new(fd.len());
    for k in 0..N_TRAIN as u64 {
        for x in &run_full(r, sample(k), STEPS) {
            set.push(&SnapshotSet::free_displacement(x, &r.x_rest, &fd));
        }
    }
    set
}

/// Fit the basis at a given rank. Split from [`snapshots`] so the R1.3 sweep can refit
/// many ranks over ONE ensemble — refitting is cheap, re-simulating 48 trajectories is
/// not, and sharing the snapshots keeps every rank comparable on identical training data.
fn fit_at(r: &Rig, set: &SnapshotSet, rank: usize) -> PodBasis {
    PodBasis::fit(set, Inner::Mass, &r.solver.mass_per_free_dof(), 1.0, rank).expect("basis fits")
}

fn train(r: &Rig) -> PodBasis {
    fit_at(r, &snapshots(r), R_MODES)
}

/// Carry the reduced trajectory to the start of the final step, in reduced coordinates
/// throughout — feeding it a re-projected oracle state would replace the model whose
/// gradient is under test with a different one.
fn reduced_state_before_last_step(
    reduced: &ReducedNewtonSolver<
        '_,
        Tet4,
        HandBuiltTetMesh,
        NullContact,
        sim_soft::NeoHookean,
        4,
        1,
    >,
    r: &Rig,
    t: Traj,
) -> (Vec<f64>, Vec<f64>) {
    let mut q = vec![0.0; reduced.n_modes()];
    let mut qdot = vec![0.0; reduced.n_modes()];
    for s in 1..STEPS {
        let th = theta_at(r, t, s);
        let step = reduced
            .step(&q, &qdot, &Tensor::from_slice(&th, &[th.len()]), DT)
            .unwrap_or_else(|e| panic!("reduced step {s} failed: {e:?}"));
        q = step.q;
        qdot = step.qdot;
    }
    (q, qdot)
}

/// `J = w · x` for a full-DOF cotangent `w`. Constrained DOFs contribute a constant
/// that cancels in every finite difference taken here.
fn objective(w: &[f64], x: &[f64]) -> f64 {
    w.iter().zip(x).map(|(a, b)| a * b).sum()
}

/// Relative L2 distance between two gradients.
fn rel_l2(a: &[f64], b: &[f64]) -> f64 {
    // `zip` would silently truncate to the shorter of the two and quietly UNDER-report
    // the error — the one failure mode a comparison helper must not have.
    assert!(
        a.len() == b.len(),
        "gradient lengths differ: {} vs {}",
        a.len(),
        b.len()
    );
    let den = b.iter().map(|v| v * v).sum::<f64>().sqrt();
    assert!(
        den > 0.0,
        "reference gradient is zero — the gate would be vacuous"
    );
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y) * (x - y))
        .sum::<f64>()
        .sqrt()
        / den
}

/// The three cotangents G2 contrasts, returned in `COTANGENTS` order together with the
/// probe DOF the localized one picks out. See the module header for why each is here
/// and what the contrast between them buys.
fn cotangents(r: &Rig) -> (Vec<f64>, Vec<f64>, Vec<f64>, usize) {
    let w_sum = vec![1.0; r.n_dof];
    // In-family: `z` over the loaded face. Its adjoint is the response to a uniform
    // top-face `z` traction — a Gaussian patch of infinite width, i.e. the training
    // ensemble's own loading class. If even this cotangent's gradient is far off, the
    // problem is not that the objective was exotic.
    let mut w_face_z = vec![0.0; r.n_dof];
    for &vid in &r.loaded {
        w_face_z[3 * vid as usize + 2] = 1.0;
    }
    // The loaded (top-face) vertex nearest the patch box centre.
    let centre = 0.010;
    let mut best = (f64::INFINITY, r.loaded[0]);
    for &vid in &r.loaded {
        let (px, py) = (r.x_rest[3 * vid as usize], r.x_rest[3 * vid as usize + 1]);
        let d2 = (px - centre).powi(2) + (py - centre).powi(2);
        if d2 < best.0 {
            best = (d2, vid);
        }
    }
    let probe = 3 * best.1 as usize + 2;
    let mut w_local = vec![0.0; r.n_dof];
    w_local[probe] = 1.0;
    (w_sum, w_face_z, w_local, probe)
}

/// The oracle adjoint `λ = A⁻¹ g_free`, scattered to full DOFs.
///
/// Recovered through the public `equilibrium_state_sensitivity`, which computes
/// `A⁻¹((M/Δt²)·dx_prev + (M/Δt)·dv_prev)` with the tangent factored at `x_final`.
/// Feeding it `dv_prev = 0` and `dx_prev = Δt²·g/m` makes the bracket exactly `g`, so
/// what comes back is `λ` — no re-derivation of the adjoint solve here, which would put
/// the diagnostic and the thing it diagnoses on the same code.
fn oracle_adjoint(r: &Rig, x_final: &[f64], x_prev: &[f64], w: &[f64]) -> Vec<f64> {
    let fd = r.solver.free_dof_indices();
    let mass = r.solver.mass_per_free_dof();
    let mut dx_prev = vec![0.0; r.n_dof];
    for (k, &i) in fd.iter().enumerate() {
        dx_prev[i] = DT * DT * w[i] / mass[k];
    }
    r.solver
        .equilibrium_state_sensitivity(x_final, Some(x_prev), DT, &dx_prev, &vec![0.0; r.n_dof])
}

/// `dJ/dθ_j = λ[k_j]` — the full-order load gradient read off an adjoint field. The
/// load enters the residual as `−f_ext(θ)` with `f_ext` copying `θ` onto the loaded
/// DOFs, so the contraction collapses to an index. This is the same closed form
/// `NewtonStepVjp` implements, and it is cross-checked against that implementation
/// wherever the tape can express the cotangent.
fn load_grad_from_adjoint(r: &Rig, lambda_full: &[f64]) -> Vec<f64> {
    let mut out = Vec::with_capacity(3 * r.loaded.len());
    for &vid in &r.loaded {
        let v = 3 * vid as usize;
        out.extend([lambda_full[v], lambda_full[v + 1], lambda_full[v + 2]]);
    }
    out
}

// ── G1: the kill gate ────────────────────────────────────────────────────────

/// Gradcheck tolerance — the crate's re-solve FD tolerance
/// (`material_sensitivity.rs`, `soft_pose_sensitivity.rs`).
const GRADCHECK_TOL: f64 = 1.0e-5;

#[cfg_attr(
    debug_assertions,
    ignore = "release-only — 48 training trajectories plus finite-difference re-solves; \
              rerun with `cargo test --release` to include"
)]
#[test]
fn reduced_gradient_matches_finite_difference_on_the_reduced_model() {
    let r = rig(MU);
    let basis = train(&r);
    let reduced = ReducedNewtonSolver::new(&r.solver, &basis, &r.x_rest);

    let t = sample(900);
    let (q_prev, qdot_prev) = reduced_state_before_last_step(&reduced, &r, t);
    let theta = theta_at(&r, t, STEPS);
    let (w_sum, w_face_z, w_local, _) = cotangents(&r);

    let solve = |th: &[f64], sol: &CpuTet4NHSolver<HandBuiltTetMesh>| -> Vec<f64> {
        let red = ReducedNewtonSolver::new(sol, &basis, &r.x_rest);
        let step = red
            .step(
                &q_prev,
                &qdot_prev,
                &Tensor::from_slice(th, &[th.len()]),
                DT,
            )
            .expect("reduced step converges");
        red.expand_to_full(&step.q)
    };

    let x_star = solve(&theta, &r.solver);
    let x_prev = reduced.expand_to_full(&q_prev);

    // Hoisted: the perturbed-material rigs do not depend on the cotangent, and
    // rebuilding a mesh + solver per iteration would triple the work for nothing.
    let dm = MU * FD_REL_MATERIAL;
    let rig_p = rig(MU + dm);
    let rig_m = rig(MU - dm);

    for (ci, w) in [&w_sum, &w_face_z, &w_local].iter().enumerate() {
        let name = COTANGENTS[ci];
        let adj = reduced
            .adjoint(&x_star, Some(&x_prev), DT, w)
            .expect("reduced adjoint factors");
        let g_theta = reduced.load_gradient(&adj);
        assert!(
            g_theta.len() == theta.len(),
            "load gradient length {} != θ length {}",
            g_theta.len(),
            theta.len()
        );

        // (a) Directional FD along a deterministic pseudo-random unit direction. One
        // pair of re-solves tests every one of the 867 components at once; a
        // component-wise sweep would leave most of them unexercised.
        // Seeded by the cotangent INDEX, not by `name.len()` — all three names are five
        // characters, so the length would have handed every cotangent the same direction
        // while reading as though it varied them.
        let mut s = 0x9E37_79B9_7F4A_7C15_u64 ^ (ci as u64 + 1);
        let mut dir: Vec<f64> = (0..theta.len())
            .map(|_| {
                s = s.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
                ((s >> 33) as f64) / ((1u64 << 31) as f64) - 0.5
            })
            .collect();
        let dn = dir.iter().map(|v| v * v).sum::<f64>().sqrt();
        for v in &mut dir {
            *v /= dn;
        }
        let theta_norm = theta.iter().map(|v| v * v).sum::<f64>().sqrt();
        let h = FD_REL_THETA * theta_norm;
        let bump = |sign: f64| -> Vec<f64> {
            theta
                .iter()
                .zip(&dir)
                .map(|(a, d)| a + sign * h * d)
                .collect()
        };
        let fd_dir = (objective(w, &solve(&bump(1.0), &r.solver))
            - objective(w, &solve(&bump(-1.0), &r.solver)))
            / (2.0 * h);
        let an_dir: f64 = g_theta.iter().zip(&dir).map(|(a, d)| a * d).sum();
        let rel_dir = (an_dir - fd_dir).abs() / fd_dir.abs();

        // (b) Three individual components, which localize a bug the directional
        // contraction could average away.
        let mut worst_comp = 0.0_f64;
        for &j in &[
            2_usize,
            3 * (r.loaded.len() / 2) + 2,
            3 * r.loaded.len() - 1,
        ] {
            let hj = FD_REL_THETA * theta_norm;
            let mut tp = theta.clone();
            let mut tm = theta.clone();
            tp[j] += hj;
            tm[j] -= hj;
            let fd = (objective(w, &solve(&tp, &r.solver)) - objective(w, &solve(&tm, &r.solver)))
                / (2.0 * hj);
            assert!(
                fd.abs() > 1e-12,
                "[{name}] θ component {j} has no measurable effect on J — the check \
                 would pass vacuously"
            );
            worst_comp = worst_comp.max((g_theta[j] - fd).abs() / fd.abs());
        }

        // (c) The material parameter — a genuinely assembled `∂r/∂p`, where the load
        // path's is the trivial `−e_k`. Same basis on both sides: the finite difference
        // holds `Φ` constant exactly as the analytic derivative assumes.
        let an_mat = reduced.material_gradient(&adj, 0);
        let fd_mat = (objective(w, &solve(&theta, &rig_p.solver))
            - objective(w, &solve(&theta, &rig_m.solver)))
            / (2.0 * dm);
        let rel_mat = (an_mat - fd_mat).abs() / fd_mat.abs();

        println!(
            "R1.2 G1 [{name}]: θ-directional rel={rel_dir:.3e} (an={an_dir:.6e} fd={fd_dir:.6e}), \
             θ-component worst rel={worst_comp:.3e}, material rel={rel_mat:.3e} \
             (an={an_mat:.6e} fd={fd_mat:.6e})"
        );

        assert!(
            an_dir.abs() > 1e-12 && an_mat.abs() > 1e-12,
            "[{name}] gradients implausibly small — the gate would pass vacuously"
        );
        assert!(
            rel_dir < GRADCHECK_TOL,
            "[{name}] reduced θ-gradient disagrees with FD on the reduced model: {rel_dir:e}"
        );
        assert!(
            worst_comp < GRADCHECK_TOL,
            "[{name}] reduced θ-gradient component disagrees with FD: {worst_comp:e}"
        );
        assert!(
            rel_mat < GRADCHECK_TOL,
            "[{name}] reduced material gradient disagrees with FD: {rel_mat:e}"
        );
    }
}

// ── G2: measured against the oracle ──────────────────────────────────────────

/// Cotangents, in the order every G2 array below indexes them.
const COTANGENTS: [&str; 3] = ["sum-x", "face-z", "node-z"];

/// Upper bound on the reduced gradient's relative L2 error against the oracle's, per
/// cotangent — **a record of what was measured, not a specification of what is
/// acceptable.**
///
/// Measured over three held-out trajectories, worst case: `sum-x` 0.816, `face-z` 0.246,
/// `node-z` 0.642. The spread within each cotangent is tiny (`face-z` ran 0.240–0.246),
/// so these bounds carry R1.1's ~30 % headroom rather than hugging the observation — a
/// gate that passes by 5 % is a gate that flakes.
///
/// The numbers are large on purpose. R1.2's finding is that they *are* large, and this
/// gate exists to catch them getting larger still, not to assert they are fine.
const MAX_GRADIENT_ERROR: [f64; 3] = [1.05, 0.32, 0.85];

/// Lower bound on `cos(g_reduced, g_oracle)`, per cotangent — the number that actually
/// decides whether the reduced gradient is usable.
///
/// A surrogate gradient fails in two separable ways, and only one of them is fatal. Too
/// short costs a line search; mis-aimed costs the descent property. Measured worst case:
/// `sum-x` 0.692, `face-z` **0.972**, `node-z` 0.766 — so on the in-family objective the
/// direction is nearly exact even though the L2 error above reads 25 %, and almost all
/// of that error is the magnitude shortfall.
///
/// Headroom is taken on `1 − cos` (the distance from perfect), also at ~30 %.
const MIN_COSINE: [f64; 3] = [0.60, 0.96, 0.69];

/// **R1.2's central finding, pinned as a gate so it cannot rot silently:** the gradient
/// error is far above the displacement error at the same step.
///
/// Formed **within** a trajectory — this rung's write-up had to be corrected once for
/// dividing one trajectory's worst gradient error by another's worst displacement error,
/// which describes no measurement that was ever taken. Measured per (trajectory,
/// cotangent): **21.5x to 185.7x**, the minimum being `face-z` on trajectory 901. The
/// bound sits at 15x, ~30 % below that minimum.
///
/// ⚠ This gate fires if the relationship IMPROVES. That is intended. Consumers are being
/// handed a validity domain built on this result, so a change in it invalidates
/// documentation rather than merely relaxing a bound — rewrite the plan's §13, do not
/// lower this number.
const MIN_AMPLIFICATION: f64 = 15.0;

#[cfg_attr(
    debug_assertions,
    ignore = "release-only — 48 training trajectories plus paired oracle/reduced runs; \
              rerun with `cargo test --release` to include"
)]
#[test]
fn reduced_gradient_against_the_oracle() {
    let mut r = rig(MU);
    let basis = train(&r);
    let (w_sum, w_face_z, w_local, probe) = cotangents(&r);
    let fd_idx = r.solver.free_dof_indices().to_vec();

    // Three held-out trajectories — R1.1's, so the numbers sit beside that rung's. One
    // (trajectory, step) pair would be an anecdote: the claim below is about a
    // *systematic* shortfall, and a systematic claim needs more than one sample.
    let mut worst = [Stat::new(), Stat::new(), Stat::new()];

    for k in 900..903_u64 {
        let t = sample(k);

        // Oracle: carry the FULL trajectory to the last step, then take that step on a
        // tape so `backward` produces the crate's own load gradient — the public
        // differentiable path, not a re-derivation of it here.
        let history = run_full(&r, t, STEPS - 1);
        let x_prev_or = history[STEPS - 2].clone();
        let v_prev_or: Vec<f64> = x_prev_or
            .iter()
            .zip(&history[STEPS - 3])
            .map(|(a, b)| (a - b) / DT)
            .collect();
        let theta = theta_at(&r, t, STEPS);

        let mut oracle_grad = |w_is_sum: bool| -> (Vec<f64>, Vec<f64>) {
            let mut tape = Tape::new();
            let theta_var: Var = tape.param_tensor(Tensor::from_slice(&theta, &[theta.len()]));
            let step = r.solver.step(
                &mut tape,
                &Tensor::from_slice(&x_prev_or, &[r.n_dof]),
                &Tensor::from_slice(&v_prev_or, &[r.n_dof]),
                theta_var,
                DT,
            );
            let x_final = step.x_final.clone();
            let root = step.x_final_var.expect("taped step registers x_final");
            if w_is_sum {
                // Seeding `backward` at the state node puts ones on every DOF — exactly
                // the `Σx*` cotangent.
                tape.backward(root);
            } else {
                let l = tape.push_custom(
                    &[root],
                    Tensor::from_slice(&[x_final[probe]], &[1]),
                    Box::new(IndexOp::new(probe, r.n_dof)),
                );
                tape.backward(l);
            }
            (tape.grad_tensor(theta_var).as_slice().to_vec(), x_final)
        };

        let (g_sum_tape, x_star_or) = oracle_grad(true);
        let (g_local_tape, _) = oracle_grad(false);

        // The tape can express `Σx*` (its own ones-seed) and a single index (`IndexOp`),
        // but not an arbitrary weighted face. Route every oracle gradient through `λ`
        // instead — `dJ/dθ_j = λ[k_j]`, the closed form `NewtonStepVjp` implements — and
        // pin that route against the tape on the two cotangents the tape *can* express.
        // Without this the face-z oracle would rest on an unvalidated re-derivation.
        let g_or: Vec<Vec<f64>> = [&w_sum, &w_face_z, &w_local]
            .iter()
            .map(|w| load_grad_from_adjoint(&r, &oracle_adjoint(&r, &x_star_or, &x_prev_or, w)))
            .collect();
        for (name, viaeq, taped) in [
            ("sum-x", &g_or[0], &g_sum_tape),
            ("node-z", &g_or[2], &g_local_tape),
        ] {
            let e = rel_l2(viaeq, taped);
            assert!(
                e < 1e-10,
                "[{name}] trajectory {k}: the λ-route oracle gradient disagrees with the \
                 tape's by {e:e} — the face-z oracle rests on this equivalence"
            );
        }

        let reduced = ReducedNewtonSolver::new(&r.solver, &basis, &r.x_rest);
        let (q_prev, qdot_prev) = reduced_state_before_last_step(&reduced, &r, t);
        let step = reduced
            .step(
                &q_prev,
                &qdot_prev,
                &Tensor::from_slice(&theta, &[theta.len()]),
                DT,
            )
            .expect("reduced step converges");
        let x_star_red = reduced.expand_to_full(&step.q);
        let x_prev_red = reduced.expand_to_full(&q_prev);

        // Displacement error at the same step — the comparison the whole rung turns on.
        let u_or = SnapshotSet::free_displacement(&x_star_or, &r.x_rest, &fd_idx);
        let u_red = SnapshotSet::free_displacement(&x_star_red, &r.x_rest, &fd_idx);
        let den = u_or.iter().map(|a| a * a).sum::<f64>().sqrt();
        let e_disp = u_or
            .iter()
            .zip(&u_red)
            .map(|(a, b)| (a - b) * (a - b))
            .sum::<f64>()
            .sqrt()
            / den;

        for (i, w) in [&w_sum, &w_face_z, &w_local].iter().enumerate() {
            let name = COTANGENTS[i];
            // (a) The practical surrogate error: each model differentiated at the state
            // it actually reached.
            let adj_own = reduced
                .adjoint(&x_star_red, Some(&x_prev_red), DT, w)
                .expect("reduced adjoint factors");
            let g_red = reduced.load_gradient(&adj_own);
            let e_state = rel_l2(&g_red, &g_or[i]);

            // (b) The same adjoint at the ORACLE's state, which removes the state error
            // entirely and leaves only the Galerkin projection of the adjoint field. The
            // gap between (a) and (b) is what the reduced *trajectory* contributes.
            let adj_at_or = reduced
                .adjoint(&x_star_or, Some(&x_prev_or), DT, w)
                .expect("reduced adjoint factors at the oracle state");
            let e_adjoint = rel_l2(&reduced.load_gradient(&adj_at_or), &g_or[i]);

            // The mechanism, measured beside the claim: how well does the basis
            // represent the ADJOINT field itself? If this tracks the gradient error
            // while the displacement error does not, the diagnosis is that a
            // displacement-trained basis does not span adjoints — not that the solve or
            // the trajectory is at fault.
            let lambda = oracle_adjoint(&r, &x_star_or, &x_prev_or, w);
            let lambda_free: Vec<f64> = fd_idx.iter().map(|&j| lambda[j]).collect();

            // Relative L2 is the wrong number to *decide* on. A gradient supplies a
            // direction and a step length, and those fail differently: a short gradient
            // costs a line search, a mis-aimed one costs the descent property.
            let dot: f64 = g_red.iter().zip(&g_or[i]).map(|(a, b)| a * b).sum();
            let nr = g_red.iter().map(|v| v * v).sum::<f64>().sqrt();
            let no = g_or[i].iter().map(|v| v * v).sum::<f64>().sqrt();

            let e_lambda = basis.projection_error(&lambda_free);
            let cos = dot / (nr * no);
            let ratio = nr / no;
            // Amplification is formed HERE, from this trajectory's own two numbers.
            // Dividing a worst-case gradient error by a worst-case displacement error
            // taken from a different trajectory would describe no measurement that was
            // ever made — and this rung's write-up already had to be corrected once for
            // exactly that pairing mistake.
            let amplification = e_state / e_disp;

            println!(
                "R1.2 G2 [{name}] traj {k}: rel err own-state {e_state:.3e} / oracle-state \
                 {e_adjoint:.3e}, adjoint projection {e_lambda:.3e}, cos {cos:.4}, \
                 ‖g_red‖/‖g_or‖ {ratio:.4}, {amplification:.1}x the displacement error \
                 ({e_disp:.3e})"
            );
            worst[i].observe(e_state, e_lambda, cos, ratio, amplification);
        }
    }

    for (i, name) in COTANGENTS.iter().enumerate() {
        let s = &worst[i];
        println!(
            "R1.2 G2 SUMMARY [{name}]: worst rel err {:.3e}, worst adjoint projection {:.3e}, \
             worst cos {:.4}, magnitude ratio {:.4}–{:.4}, amplification over displacement \
             {:.1}x–{:.1}x",
            s.max_err, s.max_lambda, s.min_cos, s.min_ratio, s.max_ratio, s.min_amp, s.max_amp
        );
        assert!(
            s.max_err < MAX_GRADIENT_ERROR[i],
            "[{name}] reduced-gradient error {:.3e} exceeds the recorded {:.3e} — R1.2's \
             measured surrogate quality has degraded",
            s.max_err,
            MAX_GRADIENT_ERROR[i]
        );
        assert!(
            s.min_cos > MIN_COSINE[i],
            "[{name}] reduced/oracle gradient cosine {:.4} fell below the recorded {:.4} — \
             the surrogate's DIRECTION, not just its magnitude, has degraded",
            s.min_cos,
            MIN_COSINE[i]
        );
        // The reduced gradient measured SHORT in all nine (trajectory, cotangent) pairs.
        // What makes that expected: the Galerkin adjoint minimizes `‖λ − μ‖_A` over the
        // span, so `‖μ‖_A ≤ ‖λ‖_A`. ⚠ That bounds an ENERGY norm of the whole field,
        // while this ratio is a Euclidean norm of the gradient restricted to the loaded
        // DOFs — so the argument motivates the assertion rather than proving it, and the
        // 9/9 observation is what it actually rests on. It is asserted anyway because the
        // other direction is the dangerous one: an OVERSTATED sensitivity reaches a
        // co-design loop with no warning, where a short one only costs a line search.
        assert!(
            s.max_ratio <= 1.0,
            "[{name}] reduced gradient is LONGER than the oracle's ({:.4}) — it measured \
             short in all 9 pilot pairs, and overstating a sensitivity is the failure \
             direction a consumer cannot detect",
            s.max_ratio
        );
        // R1.2's finding, pinned so it cannot rot silently: gradient accuracy does NOT
        // follow from state accuracy. If this fires, the relationship has changed and
        // `docs/SIM_SOFT_R1_REDUCED_BASIS_PLAN.md` §13 must be rewritten, not the bound.
        assert!(
            s.min_amp > MIN_AMPLIFICATION,
            "[{name}] the gradient error is only {:.1}x the displacement error on some \
             trajectory, under the recorded {MIN_AMPLIFICATION:.0}x — R1.2's central \
             finding has changed; rewrite the plan's §13, do not lower this bound",
            s.min_amp
        );
    }
}

/// Worst case over the held-out trajectories, per cotangent.
struct Stat {
    max_err: f64,
    max_lambda: f64,
    min_cos: f64,
    min_ratio: f64,
    max_ratio: f64,
    /// Gradient error divided by displacement error, **paired within a trajectory**.
    min_amp: f64,
    max_amp: f64,
}

impl Stat {
    const fn new() -> Self {
        Self {
            max_err: 0.0,
            max_lambda: 0.0,
            min_cos: f64::INFINITY,
            min_ratio: f64::INFINITY,
            max_ratio: 0.0,
            min_amp: f64::INFINITY,
            max_amp: 0.0,
        }
    }
    const fn observe(&mut self, err: f64, lambda: f64, cos: f64, ratio: f64, amp: f64) {
        self.max_err = self.max_err.max(err);
        self.max_lambda = self.max_lambda.max(lambda);
        self.min_cos = self.min_cos.min(cos);
        self.min_ratio = self.min_ratio.min(ratio);
        self.max_ratio = self.max_ratio.max(ratio);
        self.min_amp = self.min_amp.min(amp);
        self.max_amp = self.max_amp.max(amp);
    }
}

// ── R1.3 control: does a LARGER basis close the adjoint gap? ──────────────────

/// Ranks swept. `40` is R1.2's, `104` is the plan's §2 ceiling for this fixture
/// (`min(n_free / 50, 200)` at 5 202 free DOF) — so the sweep runs the knob to the end of
/// its legal travel rather than to a round number.
const R_SWEEP: [usize; 5] = [10, 20, 40, 80, 104];

/// **R1.3, part 1 — the control that must run before any basis enrichment.**
///
/// R1.2 found the reduced gradient 21.5–186x less accurate than the state, and traced it
/// to the basis not spanning the adjoint field. Two explanations fit that evidence and
/// they call for very different work:
///
/// 1. **the subspace is too SMALL for adjoints** — the missing content is in the tail,
///    and `r` is a knob that fixes it;
/// 2. **the subspace is WRONG for adjoints** — the missing content lies along directions
///    the training ensemble never excited, which carry ~zero singular value and are
///    therefore absent at *any* rank.
///
/// Enrichment with adjoint snapshots only makes sense under (2). Skipping this control
/// would leave a successful enrichment indistinguishable from "we added modes", so it
/// runs first.
///
/// **Prediction recorded before the first run**: (2). Modes are ordered by *displacement*
/// energy, so no rank recovers a direction the snapshots never contained. Expect `Σx*`
/// (out-of-family) to barely improve and `face-z` (in-family) to improve most.
///
/// **No reduced trajectory is run.** R1.2 measured the adjoint at the reduced model's own
/// state and at the oracle's and got the same answer to four digits, so the solver
/// dynamics are not part of this question — every rank is compared at the oracle's state,
/// which isolates the subspace and removes a confound.
#[cfg_attr(
    debug_assertions,
    ignore = "release-only — 48 training trajectories plus a rank sweep; \
              rerun with `cargo test --release` to include"
)]
#[test]
fn adjoint_gap_across_basis_sizes() {
    // Production tolerance: this rung measures the BASIS, not a finite difference, and
    // `TOL`'s 1e-10 would bias the timing against the reduced path (see `rig_with_tol`).
    let r = rig_with_tol(MU, 1.0e-6);
    let set = snapshots(&r);
    let fd_idx = r.solver.free_dof_indices().to_vec();
    let (w_sum, w_face_z, w_local, _) = cotangents(&r);
    let t = sample(900);

    // One held-out trajectory: R1.2 measured a spread of 0.240–0.246 across three for
    // `face-z`, so the rank axis — not the trajectory axis — is what carries information
    // here.
    let history = run_full(&r, t, STEPS);
    let x_star_or = history[STEPS - 1].clone();
    let x_prev_or = history[STEPS - 2].clone();

    let u_or = SnapshotSet::free_displacement(&x_star_or, &r.x_rest, &fd_idx);
    // Oracle adjoints and gradients are rank-independent — computed once, outside the
    // sweep, so every rank is scored against identical references.
    let refs: Vec<(Vec<f64>, Vec<f64>)> = [&w_sum, &w_face_z, &w_local]
        .iter()
        .map(|w| {
            let lambda = oracle_adjoint(&r, &x_star_or, &x_prev_or, w);
            let lambda_free: Vec<f64> = fd_idx.iter().map(|&j| lambda[j]).collect();
            (lambda_free, load_grad_from_adjoint(&r, &lambda))
        })
        .collect();

    // Wall time matters here even though R1's gates are accuracy-only: raising `r` is
    // only a usable answer if the reduced model is still FASTER than the oracle at that
    // rank. `ΦᵀAΦ` grows as `O(n·r²)`, so there is a rank beyond which the knob defeats
    // its own purpose. Measured, not extrapolated — cost estimates have missed every
    // time they were tried in this arc.
    let oracle_start = Instant::now();
    let _ = run_full(&r, t, STEPS);
    let oracle_ms = oracle_start.elapsed().as_secs_f64() * 1e3;

    for &rank in &R_SWEEP {
        let basis = fit_at(&r, &set, rank);
        let reduced = ReducedNewtonSolver::new(&r.solver, &basis, &r.x_rest);
        let disp = basis.projection_error(&u_or);
        let energy = basis.retained_energy_fraction();

        let traj_start = Instant::now();
        let (mut q, mut qdot) = (vec![0.0; rank], vec![0.0; rank]);
        for step_idx in 1..=STEPS {
            let th = theta_at(&r, t, step_idx);
            let st = reduced
                .step(&q, &qdot, &Tensor::from_slice(&th, &[th.len()]), DT)
                .expect("reduced step converges");
            q = st.q;
            qdot = st.qdot;
        }
        let reduced_ms = traj_start.elapsed().as_secs_f64() * 1e3;

        let mut line = format!(
            "R1.3 r={rank:>3}: displacement projection {disp:.3e} (retained energy \
             {energy:.6}), trajectory {reduced_ms:.0} ms vs oracle {oracle_ms:.0} ms \
             ({:.2}x)",
            oracle_ms / reduced_ms
        );
        for (i, w) in [&w_sum, &w_face_z, &w_local].iter().enumerate() {
            let (lambda_free, g_or) = &refs[i];
            let adj = reduced
                .adjoint(&x_star_or, Some(&x_prev_or), DT, w)
                .expect("reduced adjoint factors");
            let g_red = reduced.load_gradient(&adj);
            let dot: f64 = g_red.iter().zip(g_or).map(|(a, b)| a * b).sum();
            let nr = g_red.iter().map(|v| v * v).sum::<f64>().sqrt();
            let no = g_or.iter().map(|v| v * v).sum::<f64>().sqrt();
            // Accumulated rather than printed per line: the four lines of one rank must
            // stay together, and other tests in this binary print concurrently.
            write!(
                line,
                "\n           [{:>6}] adjoint projection {:.3e}, gradient rel err {:.3e}, \
                 cos {:.4}, ‖ratio‖ {:.4}",
                COTANGENTS[i],
                basis.projection_error(lambda_free),
                rel_l2(&g_red, g_or),
                dot / (nr * no),
                nr / no,
            )
            .expect("writing to a String cannot fail");
        }
        println!("{line}");
    }
}

// ── R1.3 part 2: can enrichment buy the accuracy without the rank? ────────────

/// Cotangents whose adjoints join the training set. Held-out by construction: none is
/// any of the three the gate scores.
const N_ENRICH_COTANGENTS: usize = 16;
/// Training trajectories whose converged states the enrichment adjoints are taken at.
const N_ENRICH_STATES: usize = 3;

/// `‖v‖_M` — the basis's own inner product, so normalisation matches how `fit` measures.
fn mass_norm(v: &[f64], mass: &[f64]) -> f64 {
    v.iter()
        .zip(mass)
        .map(|(a, m)| m * a * a)
        .sum::<f64>()
        .sqrt()
}

/// Push `v` scaled to unit `M`-norm.
///
/// **Load-bearing for any mixed set.** A displacement snapshot is ~1e-3 (metres); an
/// adjoint snapshot is `A⁻¹g`, whose scale follows the cotangent and ranged over two
/// orders of magnitude across the families used here. Concatenating raw would let the
/// larger group dictate the leading modes and the POD would be measuring units, not
/// physics.
fn push_unit(set: &mut SnapshotSet, v: &[f64], mass: &[f64]) {
    let n = mass_norm(v, mass);
    assert!(n > 0.0, "a zero snapshot carries no direction to retain");
    let scaled: Vec<f64> = v.iter().map(|a| a / n).collect();
    set.push(&scaled);
}

/// The **declared objective family** the enrichment is drawn from: smooth `z` weightings
/// over the loaded face, and single-node `z` probes.
///
/// This is the honest shape of a goal-oriented basis — it buys accuracy for a *declared*
/// class of objectives, not for all of them, and that class is what a consumer would have
/// to state up front. `Σx*` is deliberately **outside** it (unit forces along `x`/`y`),
/// and stays in the scoring as a built-in negative control: enrichment must not appear to
/// help there, or the experiment is measuring something other than what it claims.
fn enrichment_cotangent(r: &Rig, probe: usize, k: u64) -> Vec<f64> {
    let mut s = k
        .wrapping_mul(0x2545_F491_4F6C_DD1D)
        .wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut next = || {
        s = s.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        ((s >> 33) as f64) / ((1u64 << 31) as f64)
    };
    let mut w = vec![0.0; r.n_dof];
    if k.is_multiple_of(2) {
        // Smooth patch: a Gaussian z-weighting at a random centre and width. `face-z`
        // (uniform over the whole face) is the infinite-width member of this family and
        // is never generated here.
        let (cx, cy) = (0.004 + 0.012 * next(), 0.004 + 0.012 * next());
        let width = 0.002 + 0.004 * next();
        for &vid in &r.loaded {
            let v = 3 * vid as usize;
            let d2 = (r.x_rest[v] - cx).powi(2) + (r.x_rest[v + 1] - cy).powi(2);
            w[v + 2] = (-d2 / (2.0 * width * width)).exp();
        }
    } else {
        // Point probe at a loaded vertex — never the one `node-z` scores.
        // The draw is in [0, 1), scaled by a count well under 2^53 and floored — the
        // truncation the cast lint warns about is exactly the intended index selection,
        // and the value cannot be negative.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let mut idx = (next() * (r.loaded.len() as f64 - 1.0)) as usize;
        if 3 * r.loaded[idx] as usize + 2 == probe {
            idx = (idx + 1) % r.loaded.len();
        }
        w[3 * r.loaded[idx] as usize + 2] = 1.0;
    }
    w
}

/// **R1.3, part 2 — does adjoint enrichment beat the plain-rank frontier at equal cost?**
///
/// Part 1 settled that rank *can* buy gradient accuracy and that the ranks where it does
/// are past the point where reduction stops paying (r=40 runs 1.43x the oracle, r=80
/// runs 0.76x). So the question is not "does enrichment help" but whether it can reach
/// the accuracy of a **much larger** basis at the cost of a small one.
///
/// **Success criterion, fixed before the first run**: at rank 40 — where the reduced
/// model is still ~1.4x the oracle — enrichment must reach the gradient accuracy plain
/// POD needed rank 104 for, on the declared family:
///
/// - `face-z` gradient relative error **≤ 0.101** and cosine **≥ 0.995** (r=104 plain);
/// - `node-z` likewise better than plain r=104's 0.572 / 0.821;
/// - forward displacement projection error must not exceed **1 %** (plain r=40 gives
///   5.31e-3, and modes spent on adjoints are modes not spent on displacements — a
///   gradient bought by wrecking the forward model is not a win);
/// - trajectory still **≥ 1.4x** the oracle.
///
/// Failing that, R1's honest conclusion is that the reduced model is forward-only and the
/// co-design loop stays full-order.
///
/// **Predictions, recorded before running**:
/// 1. Enrichment clears the bar on `face-z` and `node-z`. The missing directions are
///    exactly what is being added, and they are cheap — a handful of modes should carry
///    what 64 extra displacement modes could not.
/// 2. `Σx*` does **not** improve, because it is outside the declared family. If it does
///    improve markedly, the experiment is leaking and the result must not be trusted.
/// 3. Forward accuracy degrades but stays inside 1 %, since 40 modes are now shared.
///
/// The normalised plain basis is scored alongside as a control: without it, any change
/// could be attributed to enrichment when unit-scaling the snapshots caused it.
#[cfg_attr(
    debug_assertions,
    ignore = "release-only — 48 training trajectories, enrichment adjoints, and three \
              paired bases; rerun with `cargo test --release` to include"
)]
#[test]
fn adjoint_enrichment_beats_the_plain_rank_frontier() {
    let r = rig_with_tol(MU, 1.0e-6);
    let fd_idx = r.solver.free_dof_indices().to_vec();
    let mass = r.solver.mass_per_free_dof();
    let (w_sum, w_face_z, w_local, probe) = cotangents(&r);
    let t = sample(900);

    // Displacement snapshots, as every earlier rung collected them.
    let raw = snapshots(&r);
    let mut disp_unit = SnapshotSet::new(fd_idx.len());
    for col in raw.columns() {
        push_unit(&mut disp_unit, col, &mass);
    }

    // Enrichment: adjoints of the declared family, at TRAINING states only.
    let mut enriched = SnapshotSet::new(fd_idx.len());
    for col in raw.columns() {
        push_unit(&mut enriched, col, &mass);
    }
    for k in 0..N_ENRICH_STATES as u64 {
        let hist = run_full(&r, sample(k), STEPS);
        let (x_f, x_p) = (&hist[STEPS - 1], &hist[STEPS - 2]);
        for c in 0..N_ENRICH_COTANGENTS as u64 {
            let w = enrichment_cotangent(&r, probe, k * 97 + c);
            let lambda = oracle_adjoint(&r, x_f, x_p, &w);
            let lambda_free: Vec<f64> = fd_idx.iter().map(|&j| lambda[j]).collect();
            push_unit(&mut enriched, &lambda_free, &mass);
        }
    }

    // References at the held-out trajectory's final step.
    let hist = run_full(&r, t, STEPS);
    let (x_star_or, x_prev_or) = (hist[STEPS - 1].clone(), hist[STEPS - 2].clone());
    let u_or = SnapshotSet::free_displacement(&x_star_or, &r.x_rest, &fd_idx);
    let refs: Vec<Vec<f64>> = [&w_sum, &w_face_z, &w_local]
        .iter()
        .map(|w| load_grad_from_adjoint(&r, &oracle_adjoint(&r, &x_star_or, &x_prev_or, w)))
        .collect();

    let oracle_start = Instant::now();
    let _ = run_full(&r, t, STEPS);
    let oracle_ms = oracle_start.elapsed().as_secs_f64() * 1e3;

    for (label, set) in [("plain-unit", &disp_unit), ("ENRICHED", &enriched)] {
        let basis = fit_at(&r, set, R_MODES);
        let reduced = ReducedNewtonSolver::new(&r.solver, &basis, &r.x_rest);

        let traj_start = Instant::now();
        let (mut q, mut qdot) = (vec![0.0; R_MODES], vec![0.0; R_MODES]);
        for step_idx in 1..=STEPS {
            let th = theta_at(&r, t, step_idx);
            let st = reduced
                .step(&q, &qdot, &Tensor::from_slice(&th, &[th.len()]), DT)
                .expect("reduced step converges");
            q = st.q;
            qdot = st.qdot;
        }
        let reduced_ms = traj_start.elapsed().as_secs_f64() * 1e3;

        let mut line = format!(
            "R1.3p2 [{label:>10}] r={R_MODES}, {} snapshots: displacement projection \
             {:.3e}, trajectory {reduced_ms:.0} ms ({:.2}x oracle)",
            set.len(),
            basis.projection_error(&u_or),
            oracle_ms / reduced_ms,
        );
        for (i, w) in [&w_sum, &w_face_z, &w_local].iter().enumerate() {
            let adj = reduced
                .adjoint(&x_star_or, Some(&x_prev_or), DT, w)
                .expect("reduced adjoint factors");
            let g_red = reduced.load_gradient(&adj);
            let dot: f64 = g_red.iter().zip(&refs[i]).map(|(a, b)| a * b).sum();
            let nr = g_red.iter().map(|v| v * v).sum::<f64>().sqrt();
            let no = refs[i].iter().map(|v| v * v).sum::<f64>().sqrt();
            write!(
                line,
                "\n              [{:>6}] gradient rel err {:.3e}, cos {:.4}, ‖ratio‖ {:.4}",
                COTANGENTS[i],
                rel_l2(&g_red, &refs[i]),
                dot / (nr * no),
                nr / no,
            )
            .expect("writing to a String cannot fail");
        }
        println!("{line}");
    }
}
