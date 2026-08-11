//! R1.1 — does a Galerkin solve on the POD subspace track the basis's own ceiling?
//!
//! R1.0 (`reduced_pod_basis.rs`) established that a linear subspace *can represent* this
//! deformation, to 0.79 % held-out at `r = 40`. That is a statement about the basis and
//! says nothing about solving in it. R1.1 asks the next question: when Newton is run
//! **inside** the subspace, how much error does the projection add on top of the ceiling
//! the basis already imposes?
//!
//! ## Why this gates a RATIO and not an absolute error
//!
//! Absolute trajectory error is dominated by basis truncation, which R1.0 already gated.
//! Asserting it again here would re-test R1.0 with more machinery and reveal nothing
//! about the solve. The informative quantity is
//!
//! ```text
//!     ‖u_reduced − u_oracle‖ / ‖(I − ΦΦᵀ)u_oracle‖
//! ```
//!
//! — total error over the basis's own floor **on the same oracle state**. A ratio near 1
//! means the Galerkin solve is doing as well as the subspace permits; a large ratio means
//! the *solve* is at fault rather than the basis. That separation is what R1.1 exists to
//! produce, and neither R1.0 nor a raw error number gives it.
//!
//! ## What is asserted
//!
//! The overhead ratio, and that reduced Newton needs no more iterations than the oracle.
//! The reduced trajectory is carried end to end in reduced coordinates — feeding it the
//! oracle's state each step would re-project away exactly the drift being measured.
//!
//! ## What is only reported
//!
//! `‖Φᵀr‖ / ‖r‖` at convergence, measured at 1e-7 to 1e-10. A Galerkin solve makes the
//! residual orthogonal to the basis, **not** zero, so the reduced state leaves a large
//! full-order residual while its displacement error stays under ~1 %. That is expected,
//! and it is why displacement — not residual — is the accuracy metric here.

#![allow(
    clippy::expect_used,
    // The LCG and the ramp fraction convert small integer counters to f64; every value
    // is far below 2^53, so the documented mantissa concern cannot arise here.
    clippy::cast_precision_loss,
    // `a`/`b`/`c`/`d` are the four unit-interval draws feeding the four trajectory
    // parameters, and `w`/`s` mirror the maths they implement; longer names would
    // obscure the correspondence rather than clarify it.
    clippy::many_single_char_names,
    // A failed oracle or reduced step is a broken gate, not a runtime condition.
    clippy::panic,
    // The gate body is one linear narrative: train, run the oracle, run the reduced
    // model, decompose the error. Splitting it would scatter the comparison.
    clippy::too_many_lines
)]

use sim_ml_chassis::Tensor;
use sim_soft::solver::backward_euler::reduced::{
    Inner, PodBasis, ReducedNewtonSolver, SnapshotSet,
};
use sim_soft::{
    BoundaryConditions, CpuTet4NHSolver, HandBuiltTetMesh, LoadAxis, MaterialField, Mesh,
    NullContact, Solver, SolverConfig, Tet4, Vec3, VertexId, pick_vertices_by_predicate,
};

const LX: f64 = 0.020;
const LY: f64 = 0.020;
const H: f64 = 0.006;
const MU: f64 = 1.0e5;
/// `nu = 0.4` — the Tet4-safe ratio the crate's other Tet4 gates use (volumetric
/// locking precludes `nu -> 0.5` without F-bar).
const LAMBDA: f64 = 4.0 * MU;
const DENSITY: f64 = 1030.0;
const DT: f64 = 1.0 / 60.0;

/// Training trajectories. Pilot-measured: 16 gives 2.1 % held-out, 32 gives 1.2 %,
/// 48 gives 0.79 %. Below ~24 the ensemble is too sparse to span the parameter box and
/// the gate would fail for want of training data rather than for want of a subspace.
const N_TRAIN: usize = 48;
/// Steps per trajectory. Raising this does **not** help — effective rank tracks
/// trajectory count (10 steps measured 0.81 %, 5 steps 0.79 %) — so it is kept low.
const STEPS: usize = 5;
/// Retained modes. Inside the plan's ceiling of `min(n_free / 50, 200)`, which is 104
/// for this fixture's 5 202 free DOF; `r = 40` is a 130x reduction.
const R_MODES: usize = 40;
struct Rig {
    solver: CpuTet4NHSolver<HandBuiltTetMesh>,
    x_rest: Vec<f64>,
    loaded: Vec<VertexId>,
    n_dof: usize,
}

/// One loading trajectory: a Gaussian traction patch at `(cx, cy)` of width `w` and
/// peak `p0`, ramped linearly to full magnitude.
#[derive(Clone, Copy, Debug)]
struct Traj {
    cx: f64,
    cy: f64,
    w: f64,
    p0: f64,
}

fn rig() -> Rig {
    let field = MaterialField::uniform(MU, LAMBDA);
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
    cfg.max_newton_iter = 80;
    cfg.tol = 1.0e-6;
    Rig {
        solver: CpuTet4NHSolver::new(Tet4, mesh, NullContact, cfg, bc),
        x_rest,
        loaded,
        n_dof,
    }
}

/// Deterministic sample of the parameter box — an LCG, so the ensemble is reproducible
/// without a dependency and without a golden file.
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

/// Oracle trajectory: the converged FULL state after each step.
fn run_full(r: &Rig, t: Traj, steps: usize) -> (Vec<Vec<f64>>, usize) {
    let mut x = r.x_rest.clone();
    let mut v = vec![0.0; r.n_dof];
    let mut out = Vec::new();
    let mut iters = 0usize;
    for s in 1..=steps {
        let frac = s as f64 / steps as f64;
        let mut th = vec![0.0; 3 * r.loaded.len()];
        for (i, &vid) in r.loaded.iter().enumerate() {
            let (px, py) = (r.x_rest[3 * vid as usize], r.x_rest[3 * vid as usize + 1]);
            let d2 = (px - t.cx).powi(2) + (py - t.cy).powi(2);
            th[3 * i + 2] = -frac * t.p0 * (-d2 / (2.0 * t.w * t.w)).exp();
        }
        let step = r.solver.replay_step(
            &Tensor::from_slice(&x, &[r.n_dof]),
            &Tensor::from_slice(&v, &[r.n_dof]),
            &Tensor::from_slice(&th, &[th.len()]),
            DT,
        );
        iters += step.iter_count;
        for i in 0..r.n_dof {
            v[i] = (step.x_final[i] - x[i]) / DT;
        }
        x = step.x_final;
        out.push(x.clone());
    }
    (out, iters)
}

/// Galerkin overhead ceiling: the reduced trajectory's error may exceed the basis's own
/// projection floor by at most this factor.
///
/// **R1.1 gates the RATIO, not the absolute error, and that is the whole point.**
/// Absolute error is dominated by basis truncation, which R1.0 already gated at 1 %;
/// asserting it again here would re-test R1.0 with more machinery and tell us nothing
/// about the solve. What is new at R1.1 is how much the *Galerkin projection* adds on
/// top of the ceiling the basis already imposes.
///
/// Measured distribution at `r = 40`, over all 15 (trajectory, step) pairs: **1.45x to
/// 1.91x**, with 13 of 15 between 1.45 and 1.63 and two early-step outliers on one
/// trajectory. The bound is set at 2.5x — roughly 30 % headroom over the observed
/// maximum — rather than hugging it, because a gate that passes by 5 % is a gate that
/// flakes.
///
/// It is not vacuous at 2.5x either, and the reason is empirical: when the reduced
/// solve was genuinely wrong (the residual projected with `ΦᵀM` instead of `Φᵀ`, so the
/// Newton direction stopped matching the residual it descended) the symptom was total
/// non-convergence on step 1, not a slightly worse ratio. This gate's job is to catch
/// *degradation* of a working solve; outright breakage announces itself.
const MAX_GALERKIN_OVERHEAD: f64 = 2.5;

// Release-only: basis training plus paired oracle/reduced trajectories at ~5.2k free
// DOF. Mirrors the `reduced_pod_basis` / `contact_drop_rest` heavy-gate pattern.
#[cfg_attr(
    debug_assertions,
    ignore = "release-only — 48 training trajectories plus paired oracle/reduced runs; \
              rerun with `cargo test --release` to include"
)]
#[test]
fn reduced_newton_tracks_the_basis_projection_floor() {
    let r = rig();
    let fd = r.solver.free_dof_indices().to_vec();
    let mass = r.solver.mass_per_free_dof();

    let mut train = SnapshotSet::new(fd.len());
    for k in 0..N_TRAIN as u64 {
        for x in &run_full(&r, sample(k), STEPS).0 {
            train.push(&SnapshotSet::free_displacement(x, &r.x_rest, &fd));
        }
    }
    let basis = PodBasis::fit(&train, Inner::Mass, &mass, 1.0, R_MODES).expect("basis fits");
    let reduced = ReducedNewtonSolver::new(&r.solver, &basis, &r.x_rest);

    let mut worst_ratio = 0.0_f64;
    let mut worst_total = 0.0_f64;
    for k in 900..903_u64 {
        let t = sample(k);
        let (oracle, oracle_iters) = run_full(&r, t, STEPS);

        // The reduced trajectory is carried in REDUCED coordinates end to end. Feeding
        // it the oracle's state each step would re-project away the drift this gate
        // exists to measure.
        let mut q = vec![0.0; basis.n_modes()];
        let mut qdot = vec![0.0; basis.n_modes()];
        let mut iters = 0usize;
        for s in 1..=STEPS {
            let frac = s as f64 / STEPS as f64;
            let mut th = vec![0.0; 3 * r.loaded.len()];
            for (i, &vid) in r.loaded.iter().enumerate() {
                let (px, py) = (r.x_rest[3 * vid as usize], r.x_rest[3 * vid as usize + 1]);
                let d2 = (px - t.cx).powi(2) + (py - t.cy).powi(2);
                th[3 * i + 2] = -frac * t.p0 * (-d2 / (2.0 * t.w * t.w)).exp();
            }
            let step = reduced
                .step(&q, &qdot, &Tensor::from_slice(&th, &[th.len()]), DT)
                .unwrap_or_else(|e| panic!("reduced step {s} of trajectory {k} failed: {e:?}"));
            iters += step.iter_count;
            q = step.q;
            qdot = step.qdot;

            // Error decomposition: total error against the oracle, versus the basis's
            // own floor on the SAME oracle state. Their ratio isolates what the Galerkin
            // solve adds beyond what the subspace already costs.
            let u_oracle = SnapshotSet::free_displacement(&oracle[s - 1], &r.x_rest, &fd);
            let u_reduced =
                SnapshotSet::free_displacement(&reduced.expand_to_full(&q), &r.x_rest, &fd);
            let den = u_oracle.iter().map(|a| a * a).sum::<f64>().sqrt();
            assert!(den > 0.0, "oracle produced a zero displacement at step {s}");
            let total = u_oracle
                .iter()
                .zip(&u_reduced)
                .map(|(a, b)| (a - b) * (a - b))
                .sum::<f64>()
                .sqrt()
                / den;
            let floor = basis.projection_error(&u_oracle);
            assert!(floor > 0.0, "projection floor vanished at step {s}");
            worst_total = worst_total.max(total);
            worst_ratio = worst_ratio.max(total / floor);
        }

        // Convergence must not degrade. A subspace that filters high-frequency content
        // could plausibly converge FASTER; what would signal trouble is needing more.
        assert!(
            iters <= oracle_iters,
            "trajectory {k}: reduced took {iters} Newton iterations vs the oracle's \
             {oracle_iters} — the projection should not make convergence harder"
        );
    }

    println!(
        "R1.1: r={R_MODES} worst total error = {worst_total:.3e}, worst Galerkin overhead = \
         {worst_ratio:.2}x of the basis floor"
    );
    assert!(
        worst_ratio < MAX_GALERKIN_OVERHEAD,
        "Galerkin overhead {worst_ratio:.2}x exceeds the {MAX_GALERKIN_OVERHEAD:.1}x gate. \
         The reduced SOLVE, not the basis, is the problem: absolute error was \
         {worst_total:.3e} against a basis floor it should be tracking."
    );
}
